#!/usr/bin/env python3
"""
fpga_erc_fix.py — Fix ERC violations caused by broken U42 pin instantiation
and conflicting global labels.

Fix 1: Replace single (pin "1" ...) entries in each U42 unit block with the
        complete list of pin UUIDs for every pin in that unit.
Fix 2: Delete 45 global_label blocks that conflict with existing local labels.

Author: OpenCode session
"""

import re
import uuid
import sys
import shutil
from pathlib import Path

# ─── Pin lists per unit (from KiCad library) ──────────────────────────────

UNIT_PINS = {
    1: ['AA9', 'AA10', 'AA11', 'AA13', 'AA14', 'AA15', 'AA16', 'AA17', 'AA18',
        'AA19', 'AA20', 'AA21', 'AB10', 'AB11', 'AB12', 'AB13', 'AB14', 'AB15',
        'AB16', 'AB17', 'AB18', 'AB20', 'AB21', 'AB22', 'M14', 'N13', 'N14',
        'N15', 'N17', 'P14', 'P15', 'P16', 'P17', 'P18', 'P19', 'P20', 'P21',
        'P22', 'R14', 'R15', 'R16', 'R17', 'R18', 'R19', 'R21', 'R22', 'T14',
        'T15', 'T16', 'T18', 'T19', 'T20', 'T21', 'T22', 'U15', 'U16', 'U17',
        'U18', 'U19', 'U20', 'U21', 'U22', 'V10', 'V13', 'V14', 'V15', 'V16',
        'V17', 'V18', 'V19', 'V20', 'V22', 'W10', 'W11', 'W12', 'W13', 'W14',
        'W15', 'W16', 'W17', 'W19', 'W20', 'W21', 'W22', 'Y10', 'Y11', 'Y12',
        'Y13', 'Y14', 'Y16', 'Y17', 'Y18', 'Y19', 'Y20', 'Y21', 'Y22'],

    2: ['A13', 'A14', 'A15', 'A16', 'A17', 'A18', 'A19', 'A20', 'A21', 'B13',
        'B14', 'B15', 'B16', 'B17', 'B18', 'B20', 'B21', 'B22', 'C13', 'C14',
        'C15', 'C17', 'C18', 'C19', 'C20', 'C21', 'C22', 'D14', 'D15', 'D16',
        'D17', 'D18', 'D19', 'D20', 'D21', 'D22', 'E13', 'E14', 'E15', 'E16',
        'E17', 'E18', 'E19', 'E21', 'E22', 'F13', 'F14', 'F15', 'F16', 'F18',
        'F19', 'F20', 'F21', 'F22', 'G13', 'G15', 'G16', 'G17', 'G18', 'G19',
        'G20', 'G21', 'G22', 'H13', 'H14', 'H15', 'H16', 'H17', 'H18', 'H19',
        'H20', 'H22', 'J13', 'J14', 'J15', 'J16', 'J17', 'J19', 'J20', 'J21',
        'J22', 'K13', 'K14', 'K16', 'K17', 'K18', 'K19', 'K20', 'K21', 'K22',
        'L13', 'L14', 'L15', 'L16', 'L17', 'L18', 'L19', 'L20', 'L21', 'M13',
        'M15', 'M16', 'M17', 'M18', 'M20', 'M21', 'M22', 'N18', 'N19', 'N20',
        'N21', 'N22'],

    3: ['A1', 'AA1', 'AA3', 'AA4', 'AA5', 'AA6', 'AA7', 'AA8', 'AB1', 'AB2',
        'AB3', 'AB4', 'AB5', 'AB6', 'AB7', 'AB8', 'B1', 'B2', 'C1', 'C2',
        'D1', 'D2', 'E1', 'E2', 'E3', 'F1', 'F2', 'F3', 'F4', 'G1', 'G2',
        'G3', 'G4', 'H2', 'H3', 'H4', 'H5', 'H6', 'J1', 'J2', 'J3', 'J4',
        'J5', 'J6', 'K1', 'K2', 'K3', 'K4', 'K6', 'L1', 'L3', 'L4', 'L5',
        'L6', 'M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'N1', 'N2', 'N3', 'N4',
        'N5', 'P1', 'P2', 'P4', 'P5', 'P6', 'R1', 'R2', 'R3', 'R4', 'R5',
        'R6', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'U1', 'U2', 'U3', 'U5',
        'U6', 'U7', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7', 'V8', 'V9', 'W1',
        'W2', 'W3', 'W4', 'W5', 'W6', 'W7', 'W9', 'Y1', 'Y2', 'Y3', 'Y4',
        'Y6', 'Y7', 'Y8', 'Y9'],

    6: ['F12', 'G11', 'L9', 'L10', 'L12', 'M9', 'M10', 'N9', 'N10', 'N12',
        'R13', 'T12', 'T13', 'U8', 'U9', 'U10', 'U11', 'U12', 'U13', 'V12'],

    7: ['A2', 'A3', 'A5', 'A7', 'A9', 'A11', 'A12', 'A22', 'AA2', 'AA12',
        'AA22', 'AB9', 'AB19', 'B3', 'B12', 'B19', 'C3', 'C6', 'C10', 'C12',
        'C16', 'D3', 'D4', 'D8', 'D12', 'D13', 'E4', 'E5', 'E7', 'E9', 'E11',
        'E12', 'E20', 'F5', 'F11', 'F17', 'G5', 'G6', 'G7', 'G8', 'G9', 'G10',
        'G12', 'G14', 'H1', 'H7', 'H8', 'H9', 'H10', 'H11', 'H12', 'H21',
        'J7', 'J8', 'J9', 'J10', 'J11', 'J12', 'J18', 'K5', 'K7', 'K8', 'K9',
        'K10', 'K11', 'K12', 'K15', 'L2', 'L7', 'L8', 'L11', 'L22', 'M7',
        'M8', 'M11', 'M12', 'M19', 'N6', 'N7', 'N8', 'N11', 'N16', 'P3', 'P7',
        'P8', 'P9', 'P10', 'P11', 'P12', 'P13', 'R7', 'R8', 'R9', 'R10',
        'R11', 'R12', 'R20', 'T7', 'T8', 'T9', 'T10', 'T11', 'T17', 'U4',
        'U14', 'V1', 'V11', 'V21', 'W8', 'W18', 'Y5', 'Y15'],
}

# ─── Conflicting global labels to delete ───────────────────────────────────
# These are nets where our scripts added global_label but schematic already
# has local label — causes same_local_global_label ERC violations.

CONFLICTING_GLOBALS = [
    'ADAR_1_CS_1V8', 'ADAR_2_CS_1V8', 'ADAR_3_CS_1V8', 'ADAR_4_CS_1V8',
    'ADAR_RX_LOAD_1', 'ADAR_RX_LOAD_2', 'ADAR_RX_LOAD_3', 'ADAR_RX_LOAD_4',
    'ADAR_TR_1', 'ADAR_TR_2', 'ADAR_TR_3',
    'ADAR_TX_LOAD_2', 'ADAR_TX_LOAD_3', 'ADAR_TX_LOAD_4',
    'ADC_D0_N', 'ADC_D0_P', 'ADC_D1_N', 'ADC_D1_P',
    'ADC_D2_N', 'ADC_D2_P', 'ADC_D3_N', 'ADC_D3_P',
    'ADC_D4_N', 'ADC_D4_P', 'ADC_D5_N', 'ADC_D5_P',
    'ADC_D6_N', 'ADC_D6_P', 'ADC_D7_P',
    'ADC_OR_N', 'ADC_OR_P',
    'ADC_PWRD',
    'DAC_0', 'DAC_1', 'DAC_2', 'DAC_3', 'DAC_4', 'DAC_5', 'DAC_6', 'DAC_7',
    'DAC_SLEEP',
    'MIX_RX_EN', 'MIX_TX_EN',
    'STM32_MISO_1V8', 'STM32_MOSI_1V8', 'STM32_SCLK_1V8',
]

# ─── U42 unit UUIDs and the broken pin line patterns ──────────────────────

U42_UNITS = {
    1: {'uuid': 'e9638b3f-f26c-46c3-8e59-28daa1ad2eba',
        'pin_line': '(pin "1" (uuid "1a622c3e-0f24-4084-b0c8-078fcceac5ed"))'},
    2: {'uuid': '0ec623ce-e753-465c-b44e-c0e2baa71a36',
        'pin_line': '(pin "1" (uuid "7ce4210d-a68d-49a4-9697-42fe8febe090"))'},
    3: {'uuid': 'efd73f00-7772-46dc-a913-4c2789be1afb',
        'pin_line': '(pin "1" (uuid "948a13a2-d070-4ff4-be03-0b9baffe6bc2"))'},
    6: {'uuid': 'ee23237f-2ed5-41b0-b5e3-2fcc7e9bb5e9',
        'pin_line': '(pin "1" (uuid "bd1010f4-eee4-4a02-8247-d491243e2282"))'},
    7: {'uuid': '09b27ed6-5dd5-4a24-b3d7-1ddffad66eee',
        'pin_line': '(pin "1" (uuid "8a047ae7-6d1e-4c90-a69e-79aa2fb7eb44"))'},
}


def generate_pin_lines(pin_list: list[str], indent: str = '\t\t') -> str:
    """Generate KiCad pin instantiation lines for a list of pin names."""
    lines = []
    for pin_name in pin_list:
        pin_uuid = str(uuid.uuid4())
        lines.append(f'{indent}(pin "{pin_name}" (uuid "{pin_uuid}"))')
    return '\n'.join(lines)


def fix_pin_instantiation(content: str) -> tuple[str, dict]:
    """Fix 1: Replace single-pin entries with full pin lists for all U42 units."""
    stats = {}
    for unit_num, unit_info in U42_UNITS.items():
        pin_list = UNIT_PINS[unit_num]
        old_pin = unit_info['pin_line']

        # The pin line in the file has leading whitespace (two tabs)
        old_pattern = f'\t\t{old_pin}'
        new_pin_lines = generate_pin_lines(pin_list, indent='\t\t')

        if old_pattern in content:
            content = content.replace(old_pattern, new_pin_lines, 1)
            stats[unit_num] = len(pin_list)
            print(f"  Unit {unit_num}: replaced 1 pin → {len(pin_list)} pins")
        else:
            print(f"  WARNING: Unit {unit_num} pin line NOT FOUND: {old_pin}")
            stats[unit_num] = 0

    return content, stats


def fix_conflicting_globals(content: str) -> tuple[str, int]:
    """Fix 2: Delete global_label blocks that conflict with existing local labels.
    
    Each global_label block looks like:
        \\t(global_label "NET_NAME"
        \\t\\t(shape bidirectional)
        \\t\\t(at X Y ANGLE)
        \\t\\t(fields_autoplaced yes)
        \\t\\t(effects ...)
        \\t\\t(uuid "...")
        \\t\\t(property "Intersheetrefs" ...)
        \\t)
    """
    deleted = 0

    for net_name in CONFLICTING_GLOBALS:
        # Match the entire global_label block for this net name
        # Pattern: starts with \t(global_label "NET_NAME"\n and ends with \t)\n
        # The block can span multiple lines with varying content
        pattern = (
            r'\t\(global_label "' + re.escape(net_name) + r'"'
            r'\n(?:\t\t[^\n]*\n)*?'  # body lines (indented with two tabs)
            r'\t\)\n'
        )
        match = re.search(pattern, content)
        if match:
            content = content[:match.start()] + content[match.end():]
            deleted += 1
            print(f"  Deleted global_label \"{net_name}\"")
        else:
            print(f"  WARNING: global_label \"{net_name}\" NOT FOUND")

    return content, deleted


def main():
    sch_path = Path(__file__).parent.parent / 'RADAR_Main_Board_2.kicad_sch'

    if not sch_path.exists():
        print(f"ERROR: Schematic not found: {sch_path}")
        sys.exit(1)

    # Create safety backup
    backup_path = sch_path.with_suffix('.kicad_sch.bak_pre_erc_fix')
    if not backup_path.exists():
        shutil.copy2(sch_path, backup_path)
        print(f"Backup created: {backup_path.name}")
    else:
        print(f"Backup already exists: {backup_path.name}")

    print(f"\nReading: {sch_path.name}")
    content = sch_path.read_text()
    original_lines = content.count('\n')
    print(f"Original: {original_lines} lines")

    # ── Fix 1: Pin instantiation ──
    print("\n=== Fix 1: Replacing pin instantiation ===")
    content, pin_stats = fix_pin_instantiation(content)
    total_pins_added = sum(pin_stats.values())
    # We removed 5 lines (one per unit) and added total_pins_added lines
    net_pin_lines = total_pins_added - 5
    print(f"Total: replaced 5 single-pin lines with {total_pins_added} pin entries (+{net_pin_lines} lines)")

    # ── Fix 2: Delete conflicting global labels ──
    print("\n=== Fix 2: Deleting conflicting global_labels ===")
    content, globals_deleted = fix_conflicting_globals(content)
    print(f"Deleted {globals_deleted} / {len(CONFLICTING_GLOBALS)} conflicting global_labels")

    # ── Write result ──
    new_lines = content.count('\n')
    print(f"\nNew file: {new_lines} lines (was {original_lines}, delta {new_lines - original_lines:+d})")
    sch_path.write_text(content)
    print(f"Written: {sch_path.name}")

    # ── Summary ──
    print("\n=== Summary ===")
    for unit_num in sorted(UNIT_PINS.keys()):
        count = pin_stats.get(unit_num, 0)
        print(f"  Unit {unit_num}: {count} pins")
    print(f"  Global labels deleted: {globals_deleted}")
    print(f"\nExpected ERC impact:")
    print(f"  - ~308 unconnected_wire_endpoint should resolve (pins now recognized)")
    print(f"  - ~260 endpoint_off_grid should resolve (wire endpoints now match pins)")
    print(f"  - ~68 label_dangling should resolve (labels now connect to pins)")
    print(f"  - 5 pin_not_connected for U42 pin '1' should resolve")
    print(f"  - 45 same_local_global_label should resolve")
    print(f"  Total estimated reduction: ~686 violations")


if __name__ == '__main__':
    main()
