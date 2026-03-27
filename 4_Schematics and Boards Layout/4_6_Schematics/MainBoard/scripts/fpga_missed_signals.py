#!/usr/bin/env python3
"""
FPGA Missed Signals Fix Script
Adds wire stubs + labels for 6 signals that were missed in the original 77-signal migration.

These signals existed in the old XC7A50T schematic but were not included in the migration map.
This script adds new wire + label connections at the correct new XC7A200T pin positions.

Author: OpenCode FPGA swap automation
"""

import uuid as uuid_mod
import sys
import os

# ============================================================
# CONFIGURATION
# ============================================================

SCHEMATIC_FILE = "/Users/ganeshpanth/PLFM_RADAR/4_Schematics and Boards Layout/4_6_Schematics/MainBoard/RADAR_Main_Board_2.kicad_sch"

# Actual placed symbol unit origins (from current schematic)
UNIT_ORIGINS = {
    1: (199.39, 449.58),   # Bank 14 - ADC
    2: (379.73, 449.58),   # Bank 15 - DAC, Digital
    3: (560.07, 449.58),   # Bank 34 - Beamformer
    6: (90.17,  449.58),   # Config/JTAG
    7: (90.17,  299.72),   # Power/GND
}

# The 6 missed signals:
# net_name -> (new_ball, unit, pin_function, lib_pin_x, lib_pin_y)
MISSED_SIGNALS = {
    "ADAR_TR_4":       ("W1",  3, "IO_L5P_T0_34",        -50.8,  30.48),
    "ADAR_TX_LOAD_1":  ("Y1",  3, "IO_L5N_T0_34",        -50.8,  27.94),
    "ADC_DCO_N":       ("W20", 1, "IO_L12N_T1_MRCC_14",   50.8,  -7.62),
    "ADC_DCO_P":       ("W19", 1, "IO_L12P_T1_MRCC_14",   50.8,  -5.08),
    "FPGA_DAC_CLOCK":  ("H19", 2, "IO_L12N_T1_MRCC_15",  -50.8,  -7.62),
    "M3S_VCTRL":       ("M16", 2, "IO_L24N_T3_RS0_15",   -50.8, -68.58),
}

WIRE_LEN = 12.7  # 0.5 inch wire stub length


# ============================================================
# HELPERS
# ============================================================

def gen_uuid():
    return str(uuid_mod.uuid4())

def fmt(val):
    """Format float for KiCad output"""
    if isinstance(val, int) or (isinstance(val, float) and val == int(val)):
        return str(int(val))
    s = f"{val:.4f}"
    s = s.rstrip('0').rstrip('.')
    return s

def build_wire(x1, y1, x2, y2):
    uid = gen_uuid()
    return (f'\t(wire\n\t\t(pts\n\t\t\t(xy {fmt(x1)} {fmt(y1)})\n'
            f'\t\t\t(xy {fmt(x2)} {fmt(y2)})\n\t\t)\n\t\t(stroke\n'
            f'\t\t\t(width 0)\n\t\t\t(type default)\n\t\t)\n'
            f'\t\t(uuid "{uid}")\n\t)')

def build_label(net_name, x, y, angle=0):
    uid = gen_uuid()
    return (f'\t(label "{net_name}"\n\t\t(at {fmt(x)} {fmt(y)} {angle})\n'
            f'\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n'
            f'\t\t\t)\n\t\t)\n\t\t(uuid "{uid}")\n\t)')

def build_global_label(net_name, x, y, angle=0, shape="bidirectional"):
    """Build a global_label for cross-sheet connectivity."""
    uid = gen_uuid()
    prop_uid = gen_uuid()
    # Determine justify based on angle
    if angle == 0:
        justify = "left"
    elif angle == 180:
        justify = "right"
    else:
        justify = "left"
    return (f'\t(global_label "{net_name}"\n'
            f'\t\t(shape {shape})\n'
            f'\t\t(at {fmt(x)} {fmt(y)} {angle})\n'
            f'\t\t(fields_autoplaced yes)\n'
            f'\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n'
            f'\t\t\t)\n\t\t\t(justify {justify})\n'
            f'\t\t)\n\t\t(uuid "{uid}")\n'
            f'\t\t(property "Intersheetrefs" "${{INTERSHEET_REFS}}"\n'
            f'\t\t\t(at 0 0 0)\n'
            f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n'
            f'\t\t\t\t)\n\t\t\t\t(justify {justify})\n'
            f'\t\t\t\t(hide yes)\n'
            f'\t\t\t)\n\t\t)\n\t)')


# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("FPGA Missed Signals Fix: Adding 6 missed signal connections")
    print("=" * 60)

    if not os.path.exists(SCHEMATIC_FILE):
        print(f"ERROR: Schematic not found: {SCHEMATIC_FILE}")
        sys.exit(1)

    # Read schematic
    print(f"\nReading {SCHEMATIC_FILE}...")
    with open(SCHEMATIC_FILE, 'r') as f:
        content = f.read()
    lines = content.split('\n')
    print(f"Read {len(lines)} lines")

    # Build new wire+label entries for each missed signal
    new_entries = []
    print("\n--- Building wire+label entries for 6 missed signals ---")

    for net_name, (new_ball, unit, pin_func, lib_px, lib_py) in MISSED_SIGNALS.items():
        ox, oy = UNIT_ORIGINS[unit]

        # Absolute pin position (same math as original fpga_swap.py — no mirror correction)
        pin_abs_x = ox + lib_px
        pin_abs_y = oy + lib_py

        # Wire stub direction depends on which edge the pin is on
        if lib_px > 0:   # Right-edge pin (Unit 1, Unit 6)
            label_x = pin_abs_x + WIRE_LEN
            label_y = pin_abs_y
            label_angle = 0
        else:             # Left-edge pin (Unit 2, Unit 3)
            label_x = pin_abs_x - WIRE_LEN
            label_y = pin_abs_y
            label_angle = 0

        # Add wire stub
        wire = build_wire(pin_abs_x, pin_abs_y, label_x, label_y)
        new_entries.append(wire)

        # Use local label (consistent with existing 77 signals)
        label = build_label(net_name, label_x, label_y, label_angle)
        new_entries.append(label)

        print(f"  {net_name}: ball={new_ball}, unit={unit}, "
              f"pin=({fmt(pin_abs_x)}, {fmt(pin_abs_y)}), "
              f"label=({fmt(label_x)}, {fmt(label_y)})")

    # For M3S_VCTRL: it was deleted from sheet 2, so we also need a global_label
    # to re-establish cross-sheet connectivity with sheet 3 (RF sheet)
    # Actually, the local label is sufficient since M3S_VCTRL global_labels
    # exist on sheet 3 — KiCad resolves same-name labels across global scope.
    # BUT: local labels only connect within the same sheet.
    # Global labels connect across sheets.
    # So for M3S_VCTRL, we need a global_label on sheet 2.
    #
    # Wait: Actually for ALL nets that need cross-sheet connectivity,
    # we need global_labels. Let me check which of the 6 nets have
    # global_labels elsewhere:
    # - ADAR_TR_4: global_label on sheet 3 (line 28998, 29454)
    # - ADAR_TX_LOAD_1: global_label on sheet 3 (line 26790)
    # - ADC_DCO_N: global_label on sheet 3 (line 26406)
    # - ADC_DCO_P: global_label on sheet 3 (line 27774)
    # - FPGA_DAC_CLOCK: NOT on sheet 3 — uses local label only
    # - M3S_VCTRL: global_labels on sheet 3 (17 instances)
    #
    # For cross-sheet connectivity we need global_labels on sheet 2.
    # The 5 nets that exist as global_labels on sheet 3 need matching
    # global_labels on sheet 2.
    #
    # However — looking at the existing 77 signals, they ALL use local labels.
    # But they connect to other components on the SAME sheet (sheet 2).
    # The 6 missed signals connect to components on SHEET 3 (RF).
    # So local labels won't work for cross-sheet nets!
    #
    # Let me check: do the existing global_labels for these nets still exist
    # on sheet 2? Some were deleted by ERC fix scripts.
    # From our earlier grep: ADAR_TR_4, ADC_DCO_N, ADAR_TX_LOAD_1, ADC_DCO_P
    # still exist on sheet 2 as global_labels. M3S_VCTRL was deleted.
    # FPGA_DAC_CLOCK exists as a local label on sheet 2.
    #
    # So: The existing global_labels on sheet 2 are NOT connected to the
    # new U42 pins (they're floating at old positions). We need the local
    # labels near U42 pins to join the same net. In KiCad, a local label
    # and a global_label with the same name on the same sheet DO connect.
    # So the local labels we're adding WILL connect to the existing
    # global_labels if they share the same net name.
    #
    # For M3S_VCTRL: no global_label remains on sheet 2, so we need to add one.
    # We'll add a global_label right at the wire endpoint (instead of or
    # in addition to the local label).

    # Add global_label for M3S_VCTRL since it was deleted from sheet 2
    # Place it at the same position as the local label
    ox, oy = UNIT_ORIGINS[2]
    lib_px, lib_py = -50.8, -68.58
    pin_abs_x = ox + lib_px
    label_x = pin_abs_x - WIRE_LEN
    label_y = oy + lib_py
    gl = build_global_label("M3S_VCTRL", label_x, label_y, 180)
    new_entries.append(gl)
    print(f"\n  Added global_label for M3S_VCTRL (was deleted from sheet 2)")

    # Insert new entries before the closing ')' of the schematic
    print(f"\n--- Inserting {len(new_entries)} new entries ---")

    # Find the last ')' line
    insert_idx = len(lines) - 1
    while insert_idx >= 0 and lines[insert_idx].strip() != ')':
        insert_idx -= 1

    if insert_idx < 0:
        print("ERROR: Could not find closing parenthesis!")
        sys.exit(1)

    # Insert
    new_content = '\n'.join(new_entries)
    lines.insert(insert_idx, new_content)

    # Write output
    output = '\n'.join(lines)
    with open(SCHEMATIC_FILE, 'w') as f:
        f.write(output)

    final_count = output.count('\n') + 1
    print(f"\nWrote {final_count} lines to {SCHEMATIC_FILE}")

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Added 6 wire stubs + 6 local labels + 1 global_label (M3S_VCTRL)")
    print(f"\nSignal mapping:")
    for net_name, (new_ball, unit, pin_func, _, _) in sorted(MISSED_SIGNALS.items()):
        print(f"  {net_name:<20s} -> ball {new_ball:<4s} (unit {unit}, {pin_func})")
    print(f"\nNext steps:")
    print(f"  1. Run ERC to verify: kicad-cli sch erc --severity-all -o /tmp/erc_post_missed.json RADAR_Main_Board_2.kicad_sch")
    print(f"  2. Update PCB net assignments for the 6 new pads")
    print(f"  3. Update U42_CONNECTION_MAP.md")


if __name__ == "__main__":
    main()
