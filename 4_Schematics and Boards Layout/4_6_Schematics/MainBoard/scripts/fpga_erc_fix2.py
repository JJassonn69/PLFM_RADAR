#!/usr/bin/env python3
"""
fpga_erc_fix2.py — Follow-up fix: delete orphaned wire stubs left by
deleted global_labels, and delete 2 remaining duplicate ADC_OR global_labels.

The fpga_erc_fix.py script deleted 46 global_labels but left behind the short
wire stubs that connected to them, causing new wire_dangling/unconnected_wire_endpoint
violations. This script:
1. Deletes the 2 remaining ADC_OR_N/ADC_OR_P duplicate global_labels
2. Deletes wire stubs whose endpoints match deleted global_label coordinates
"""

import re
import sys
from pathlib import Path

# All coordinates of global_labels that were (or need to be) deleted
# These are the endpoints of orphaned wire stubs
GLOBAL_LABEL_COORDS = [
    (571.5, 452.12),   # ADAR_1_CS_1V8
    (571.5, 454.66),   # ADAR_2_CS_1V8
    (571.5, 457.2),    # ADAR_3_CS_1V8
    (571.5, 459.74),   # ADAR_4_CS_1V8
    (642.62, 452.12),  # ADAR_RX_LOAD_1
    (642.62, 464.82),  # ADAR_RX_LOAD_2
    (642.62, 459.74),  # ADAR_RX_LOAD_3
    (642.62, 454.66),  # ADAR_RX_LOAD_4
    (571.5, 467.36),   # ADAR_TR_1
    (571.5, 469.9),    # ADAR_TR_2
    (571.5, 472.44),   # ADAR_TR_3
    (642.62, 467.36),  # ADAR_TX_LOAD_2
    (642.62, 462.28),  # ADAR_TX_LOAD_3
    (642.62, 457.2),   # ADAR_TX_LOAD_4
    (144.78, 480.06),  # ADC_D0_N
    (144.78, 469.9),   # ADC_D0_P
    (149.86, 492.76),  # ADC_D1_N
    (149.86, 482.6),   # ADC_D1_P
    (139.7, 508.0),    # ADC_D2_N
    (139.7, 497.84),   # ADC_D2_P
    (287.02, 457.2),   # ADC_D3_N
    (287.02, 447.04),  # ADC_D3_P
    (292.1, 477.52),   # ADC_D4_N
    (292.1, 467.36),   # ADC_D4_P
    (292.1, 502.92),   # ADC_D5_N
    (292.1, 492.76),   # ADC_D5_P
    (284.48, 490.22),  # ADC_D6_N
    (284.48, 480.06),  # ADC_D6_P
    (299.72, 505.46),  # ADC_D7_P
    (264.16, 469.9),   # ADC_OR_N (first copy - already deleted)
    (266.7, 533.4),    # ADC_OR_N (second copy - needs deletion)
    (266.7, 523.24),   # ADC_OR_P (first copy - already deleted)
    (264.16, 467.36),  # ADC_OR_P (second copy - needs deletion)
    (264.16, 490.22),  # ADC_PWRD
    (332.74, 472.44),  # DAC_0
    (332.74, 469.9),   # DAC_1
    (332.74, 462.28),  # DAC_2
    (332.74, 457.2),   # DAC_3
    (332.74, 454.66),  # DAC_4
    (332.74, 452.12),  # DAC_5
    (332.74, 447.04),  # DAC_6
    (332.74, 444.5),   # DAC_7
    (332.74, 482.6),   # DAC_SLEEP
    (419.1, 444.5),    # MIX_RX_EN
    (332.74, 490.22),  # MIX_TX_EN
    (571.5, 464.82),   # STM32_MISO_1V8
    (571.5, 462.28),   # STM32_MOSI_1V8
    (642.62, 472.44),  # STM32_SCLK_1V8
]


def main():
    sch_path = Path(__file__).parent.parent / 'RADAR_Main_Board_2.kicad_sch'

    if not sch_path.exists():
        print(f"ERROR: Schematic not found: {sch_path}")
        sys.exit(1)

    print(f"Reading: {sch_path.name}")
    content = sch_path.read_text()
    original_lines = content.count('\n')
    print(f"Original: {original_lines} lines")

    # ── Fix 1: Delete remaining 2 global_labels ──
    print("\n=== Deleting remaining ADC_OR global_labels ===")
    remaining_globals = ['ADC_OR_N', 'ADC_OR_P']
    globals_deleted = 0
    for net_name in remaining_globals:
        pattern = (
            r'\t\(global_label "' + re.escape(net_name) + r'"'
            r'\n(?:\t\t[^\n]*\n)*?'
            r'\t\)\n'
        )
        match = re.search(pattern, content)
        if match:
            content = content[:match.start()] + content[match.end():]
            globals_deleted += 1
            print(f"  Deleted global_label \"{net_name}\"")
        else:
            print(f"  WARNING: global_label \"{net_name}\" not found")

    # ── Fix 2: Delete orphaned wire stubs ──
    print("\n=== Deleting orphaned wire stubs ===")

    # Build coordinate set for fast lookup
    coord_set = set()
    for x, y in GLOBAL_LABEL_COORDS:
        coord_set.add((round(x, 4), round(y, 4)))

    # Find and remove wire blocks that have an endpoint matching a global_label coordinate
    # Wire block format:
    #   \t(wire\n\t\t(pts\n\t\t\t(xy X1 Y1) (xy X2 Y2)\n\t\t)\n\t\t(stroke\n\t\t\t(width N)\n\t\t\t(type T)\n\t\t)\n\t\t(uuid "U")\n\t)\n
    wire_pattern = re.compile(
        r'\t\(wire\n'
        r'\t\t\(pts\n'
        r'\t\t\t\(xy ([\d.]+) ([\d.]+)\) \(xy ([\d.]+) ([\d.]+)\)\n'
        r'\t\t\)\n'
        r'\t\t\(stroke\n'
        r'\t\t\t\(width [\d.]+\)\n'
        r'\t\t\t\(type \w+\)\n'
        r'\t\t\)\n'
        r'\t\t\(uuid "[^"]+"\)\n'
        r'\t\)\n'
    )

    wires_deleted = 0
    # Process in reverse order to maintain match positions
    matches = list(wire_pattern.finditer(content))
    to_delete = []

    for m in matches:
        x1, y1 = round(float(m.group(1)), 4), round(float(m.group(2)), 4)
        x2, y2 = round(float(m.group(3)), 4), round(float(m.group(4)), 4)

        # Check if either endpoint matches a deleted global_label position
        if (x1, y1) in coord_set or (x2, y2) in coord_set:
            to_delete.append((m.start(), m.end(), x1, y1, x2, y2))

    # Delete in reverse order to maintain positions
    for start, end, x1, y1, x2, y2 in reversed(to_delete):
        content = content[:start] + content[end:]
        wires_deleted += 1

    print(f"  Deleted {wires_deleted} orphaned wire stubs")
    for start, end, x1, y1, x2, y2 in to_delete:
        print(f"    wire ({x1},{y1})-({x2},{y2})")

    # ── Write result ──
    new_lines = content.count('\n')
    print(f"\nNew file: {new_lines} lines (was {original_lines}, delta {new_lines - original_lines:+d})")
    sch_path.write_text(content)
    print(f"Written: {sch_path.name}")

    print(f"\n=== Summary ===")
    print(f"  Global labels deleted: {globals_deleted}")
    print(f"  Wire stubs deleted: {wires_deleted}")
    print(f"  Expected: -2 same_local_global_label, ~-46 wire_dangling, ~-46 unconnected_wire_endpoint")


if __name__ == '__main__':
    main()
