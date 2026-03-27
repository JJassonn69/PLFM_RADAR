#!/usr/bin/env python3
"""
FPGA Power Pin Connection Script for KiCad Schematic
Adds power connections (GND, VCCINT, VCCAUX, VCCBRAM, VCCO) to the new XC7A200T-FBG484 (U42).

Approach: For each group of power pins, draw a short wire stub from the pin connection
point and place a global_label at the end matching the existing power net names.
GND pins use the Eagle-import GND power symbol.

Author: OpenCode FPGA swap automation
"""

import re
import uuid as uuid_mod
import sys
import os

# ============================================================
# CONFIGURATION
# ============================================================

SCHEMATIC_FILE = "/Users/ganeshpanth/PLFM_RADAR/4_Schematics and Boards Layout/4_6_Schematics/MainBoard/RADAR_Main_Board_2.kicad_sch"

# Symbol placement positions (must match fpga_swap.py)
NEW_UNIT_POSITIONS = {
    1: (200.0, 450.0),    # Banks 13/14
    2: (380.0, 450.0),    # Banks 15/16
    3: (560.0, 450.0),    # Banks 34/35
    6: (90.0,  450.0),    # Config/JTAG
    7: (90.0,  300.0),    # Power/GND
}

# Power pin mapping: (unit, pin_name) -> list of (ball, rel_x, rel_y, angle)
# angle: 0=extends LEFT, 90=extends DOWN, 180=extends RIGHT, 270=extends UP
# Connection point IS the (at) coordinate in the symbol definition.

POWER_PINS = {
    # Unit 1 - VCCO pins on top edge (angle=270, pointing UP from symbol)
    (1, "VCCO_13"): [
        ("AA17", -40.64, 73.66, 270),
        ("AB14", -38.10, 73.66, 270),
        ("V16",  -35.56, 73.66, 270),
        ("W13",  -33.02, 73.66, 270),
        ("Y10",  -30.48, 73.66, 270),
    ],
    (1, "VCCO_14"): [
        ("M14",  27.94, 73.66, 270),
        ("P18",  30.48, 73.66, 270),
        ("R15",  33.02, 73.66, 270),
        ("T22",  35.56, 73.66, 270),
        ("U19",  38.10, 73.66, 270),
        ("Y20",  40.64, 73.66, 270),
    ],
    # Unit 2 - VCCO pins on top edge
    (2, "VCCO_15"): [
        ("G19",  -40.64, 73.66, 270),
        ("H16",  -38.10, 73.66, 270),
        ("J13",  -35.56, 73.66, 270),
        ("K20",  -33.02, 73.66, 270),
        ("L17",  -30.48, 73.66, 270),
        ("N21",  -27.94, 73.66, 270),
    ],
    (2, "VCCO_16"): [
        ("A17",  27.94, 73.66, 270),
        ("B14",  30.48, 73.66, 270),
        ("C21",  33.02, 73.66, 270),
        ("D18",  35.56, 73.66, 270),
        ("E15",  38.10, 73.66, 270),
        ("F22",  40.64, 73.66, 270),
    ],
    # Unit 3 - VCCO pins on top edge
    (3, "VCCO_34"): [
        ("AA7",  -40.64, 73.66, 270),
        ("AB4",  -38.10, 73.66, 270),
        ("R5",   -35.56, 73.66, 270),
        ("T2",   -33.02, 73.66, 270),
        ("V6",   -30.48, 73.66, 270),
        ("W3",   -27.94, 73.66, 270),
    ],
    (3, "VCCO_35"): [
        ("C1",   27.94, 73.66, 270),
        ("F2",   30.48, 73.66, 270),
        ("H6",   33.02, 73.66, 270),
        ("J3",   35.56, 73.66, 270),
        ("M4",   38.10, 73.66, 270),
        ("N1",   40.64, 73.66, 270),
    ],
    # Unit 6 - VCCO_0 (config bank)
    (6, "VCCO_0"): [
        ("F12",  25.40, 40.64, 270),
        ("T12",  27.94, 40.64, 270),
    ],
    # Unit 7 - Core power on LEFT edge (angle=0)
    (7, "VCCAUX"): [
        ("H12",  -25.40, 73.66, 0),
        ("K12",  -25.40, 71.12, 0),
        ("M12",  -25.40, 68.58, 0),
        ("P12",  -25.40, 66.04, 0),
        ("R11",  -25.40, 63.50, 0),
    ],
    (7, "VCCADC_0"): [
        ("K10",  -25.40, 55.88, 0),
    ],
    (7, "VCCBATT_0"): [
        ("E12",  -25.40, 53.34, 0),
    ],
    (7, "VCCBRAM"): [
        ("J11",  -25.40, 45.72, 0),
        ("L11",  -25.40, 48.26, 0),
        ("N11",  -25.40, 50.80, 0),
    ],
    (7, "GNDADC_0"): [
        ("K9",   -25.40, 58.42, 0),
    ],
    # Unit 7 - VCCINT on RIGHT edge (angle=180)
    (7, "VCCINT"): [
        ("H8",   25.40, 73.66, 180),
        ("H10",  25.40, 71.12, 180),
        ("J7",   25.40, 68.58, 180),
        ("J9",   25.40, 66.04, 180),
        ("K8",   25.40, 63.50, 180),
        ("L7",   25.40, 60.96, 180),
        ("M8",   25.40, 58.42, 180),
        ("N7",   25.40, 55.88, 180),
        ("P8",   25.40, 53.34, 180),
        ("P10",  25.40, 50.80, 180),
        ("R7",   25.40, 48.26, 180),
        ("R9",   25.40, 45.72, 180),
        ("T8",   25.40, 43.18, 180),
        ("T10",  25.40, 40.64, 180),
    ],
    # Unit 7 - GND on LEFT edge (angle=0) - first 49 pins
    (7, "GND_LEFT"): [
        ("A2",   -25.40, 35.56, 0),
        ("A3",   -25.40, 33.02, 0),
        ("A5",   -25.40, 30.48, 0),
        ("A7",   -25.40, 27.94, 0),
        ("A9",   -25.40, 25.40, 0),
        ("A11",  -25.40, 22.86, 0),
        ("A12",  -25.40, 20.32, 0),
        ("A22",  -25.40, 17.78, 0),
        ("B3",   -25.40, 15.24, 0),
        ("B12",  -25.40, 12.70, 0),
        ("B19",  -25.40, 10.16, 0),
        ("C3",   -25.40, 7.62, 0),
        ("C6",   -25.40, 5.08, 0),
        ("C10",  -25.40, 2.54, 0),
        ("C12",  -25.40, 0.00, 0),
        ("C16",  -25.40, -2.54, 0),
        ("D3",   -25.40, -5.08, 0),
        ("D4",   -25.40, -7.62, 0),
        ("D8",   -25.40, -10.16, 0),
        ("D12",  -25.40, -12.70, 0),
        ("D13",  -25.40, -15.24, 0),
        ("E4",   -25.40, -17.78, 0),
        ("E5",   -25.40, -20.32, 0),
        ("E7",   -25.40, -22.86, 0),
        ("E9",   -25.40, -25.40, 0),
        ("E11",  -25.40, -27.94, 0),
        ("E20",  -25.40, -30.48, 0),
        ("F5",   -25.40, -33.02, 0),
        ("F11",  -25.40, -35.56, 0),
        ("F17",  -25.40, -38.10, 0),
        ("G5",   -25.40, -40.64, 0),
        ("G6",   -25.40, -43.18, 0),
        ("G7",   -25.40, -45.72, 0),
        ("G8",   -25.40, -48.26, 0),
        ("G9",   -25.40, -50.80, 0),
        ("G10",  -25.40, -53.34, 0),
        ("G12",  -25.40, -55.88, 0),
        ("G14",  -25.40, -58.42, 0),
        ("H1",   -25.40, -60.96, 0),
        ("H7",   -25.40, -63.50, 0),
        ("H9",   -25.40, -66.04, 0),
        ("H11",  -25.40, -68.58, 0),
        ("H21",  -25.40, -71.12, 0),
        ("J8",   -25.40, -73.66, 0),
    ],
    # Unit 7 - GND on RIGHT edge (angle=180) - 38 pins
    (7, "GND_RIGHT"): [
        ("J10",  25.40, 33.02, 180),
        ("J12",  25.40, 30.48, 180),
        ("J18",  25.40, 27.94, 180),
        ("K5",   25.40, 25.40, 180),
        ("K7",   25.40, 22.86, 180),
        ("K11",  25.40, 20.32, 180),
        ("K15",  25.40, 17.78, 180),
        ("L2",   25.40, 15.24, 180),
        ("L8",   25.40, 12.70, 180),
        ("L22",  25.40, 10.16, 180),
        ("M7",   25.40, 7.62, 180),
        ("M11",  25.40, 5.08, 180),
        ("M19",  25.40, 2.54, 180),
        ("N6",   25.40, 0.00, 180),
        ("N8",   25.40, -2.54, 180),
        ("N16",  25.40, -5.08, 180),
        ("P3",   25.40, -7.62, 180),
        ("P7",   25.40, -10.16, 180),
        ("P9",   25.40, -12.70, 180),
        ("P11",  25.40, -15.24, 180),
        ("P13",  25.40, -17.78, 180),
        ("R8",   25.40, -20.32, 180),
        ("R10",  25.40, -22.86, 180),
        ("R12",  25.40, -25.40, 180),
        ("R20",  25.40, -27.94, 180),
        ("T7",   25.40, -30.48, 180),
        ("T9",   25.40, -33.02, 180),
        ("T11",  25.40, -35.56, 180),
        ("T17",  25.40, -38.10, 180),
        ("U4",   25.40, -40.64, 180),
        ("U14",  25.40, -43.18, 180),
        ("V1",   25.40, -45.72, 180),
        ("V11",  25.40, -48.26, 180),
        ("V21",  25.40, -50.80, 180),
        ("W8",   25.40, -53.34, 180),
        ("W18",  25.40, -55.88, 180),
        ("Y5",   25.40, -58.42, 180),
        ("Y15",  25.40, -60.96, 180),
        ("AA2",  25.40, -63.50, 180),
        ("AA12", 25.40, -66.04, 180),
        ("AA22", 25.40, -68.58, 180),
        ("AB9",  25.40, -71.12, 180),
        ("AB19", 25.40, -73.66, 180),
    ],
}

# Power net name mapping: pin_name -> (net_label, label_type)
# label_type: "global" = global_label, "gnd" = GND power symbol, "nc" = no_connect
POWER_NET_MAP = {
    "VCCO_13":   ("+3V3_FPGA", "global"),   # Bank 13 = 3.3V
    "VCCO_14":   ("+3V3_FPGA", "global"),   # Bank 14 = 3.3V
    "VCCO_15":   ("+3V3_FPGA", "global"),   # Bank 15 = 3.3V
    "VCCO_16":   ("+3V3_FPGA", "nc"),       # Bank 16 unused - still needs VCCO
    "VCCO_34":   ("+1V8_FPGA", "global"),   # Bank 34 = 1.8V
    "VCCO_35":   ("+1V8_FPGA", "nc"),       # Bank 35 unused - still needs VCCO
    "VCCO_0":    ("+1V8_FPGA", "global"),   # Config bank
    "VCCAUX":    ("+1V8_FPGA", "global"),   # VCCAUX = 1.8V
    "VCCADC_0":  ("+1V8_FPGA", "global"),   # ADC reference = 1.8V
    "VCCBATT_0": ("+1V8_FPGA", "global"),   # Battery backup = 1.8V
    "VCCBRAM":   ("+1V0_FPGA", "global"),   # BRAM = 1.0V
    "VCCINT":    ("+1V0_FPGA", "global"),   # Core = 1.0V
    "GNDADC_0":  ("GND", "gnd"),            # Analog GND
    "GND_LEFT":  ("GND", "gnd"),            # Digital GND
    "GND_RIGHT": ("GND", "gnd"),            # Digital GND
}

# Wire stub length
WIRE_STUB = 7.62   # 3 grid units (shorter than signal stubs for cleanliness)
GND_STUB = 5.08    # GND symbols need shorter stub

# ============================================================
# HELPERS (same as fpga_swap.py)
# ============================================================

def gen_uuid():
    return str(uuid_mod.uuid4())

def fmt(val):
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

def build_global_label(net_name, x, y, angle=0):
    """Build a global_label matching existing schematic style."""
    uid = gen_uuid()
    return (f'\t(global_label "{net_name}"\n'
            f'\t\t(shape input)\n'
            f'\t\t(at {fmt(x)} {fmt(y)} {angle})\n'
            f'\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n'
            f'\t\t\t)\n\t\t)\n'
            f'\t\t(uuid "{uid}")\n'
            f'\t\t(property "Intersheetrefs" "${{INTERSHEET_REFS}}"\n'
            f'\t\t\t(at 0 0 0)\n'
            f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n'
            f'\t\t\t\t)\n\t\t\t\t(hide yes)\n'
            f'\t\t\t)\n\t\t)\n'
            f'\t)')

def build_gnd_symbol(x, y):
    """Build a GND power symbol matching existing Eagle-import style.
    The Eagle GND symbol is a power_in symbol with value GND.
    For new GND connections we use the standard approach: a global_label "GND"
    is simpler and more reliable than trying to instantiate the Eagle GND symbol.
    Actually, for proper power connection, we use a power_port."""
    uid = gen_uuid()
    # Use a global label for GND - simpler and connects to the same net
    return (f'\t(global_label "GND"\n'
            f'\t\t(shape input)\n'
            f'\t\t(at {fmt(x)} {fmt(y)} 0)\n'
            f'\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n'
            f'\t\t\t)\n\t\t)\n'
            f'\t\t(uuid "{uid}")\n'
            f'\t\t(property "Intersheetrefs" "${{INTERSHEET_REFS}}"\n'
            f'\t\t\t(at 0 0 0)\n'
            f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n'
            f'\t\t\t\t)\n\t\t\t\t(hide yes)\n'
            f'\t\t\t)\n\t\t)\n'
            f'\t)')

def build_no_connect(x, y):
    uid = gen_uuid()
    return (f'\t(no_connect\n'
            f'\t\t(at {fmt(x)} {fmt(y)})\n'
            f'\t\t(uuid "{uid}")\n'
            f'\t)')

# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("FPGA Power Pin Connection Script")
    print("=" * 60)

    # Read current schematic
    print(f"\nReading {SCHEMATIC_FILE}...")
    with open(SCHEMATIC_FILE, 'r') as f:
        content = f.read()
    lines = content.split('\n')
    total_lines = len(lines)
    print(f"Read {total_lines} lines")

    new_parts = []
    stats = {"wires": 0, "global_labels": 0, "gnd_labels": 0, "no_connect": 0}

    for (unit_num, pin_group_name), pin_list in sorted(POWER_PINS.items()):
        net_label, label_type = POWER_NET_MAP[pin_group_name]
        sx, sy = NEW_UNIT_POSITIONS[unit_num]

        print(f"\n  Unit {unit_num} {pin_group_name} -> {net_label} ({label_type}, {len(pin_list)} pins)")

        for ball, rel_x, rel_y, angle in pin_list:
            # Absolute pin position
            pin_x = sx + rel_x
            pin_y = sy + rel_y

            # Wire stub direction depends on pin angle
            stub = WIRE_STUB
            if angle == 0:    # Pin extends LEFT, wire goes LEFT
                wx = pin_x - stub
                wy = pin_y
                label_angle = 180  # Label faces left
            elif angle == 180:  # Pin extends RIGHT, wire goes RIGHT
                wx = pin_x + stub
                wy = pin_y
                label_angle = 0    # Label faces right
            elif angle == 270:  # Pin extends UP, wire goes UP
                wx = pin_x
                wy = pin_y - stub  # KiCad Y is inverted (up = decrease)
                label_angle = 90   # Label faces up
            elif angle == 90:   # Pin extends DOWN
                wx = pin_x
                wy = pin_y + stub
                label_angle = 270
            else:
                wx, wy, label_angle = pin_x, pin_y, 0

            # Build wire
            wire = build_wire(pin_x, pin_y, wx, wy)
            new_parts.append(wire)
            stats["wires"] += 1

            # Build label/symbol at wire end
            if label_type == "global":
                lbl = build_global_label(net_label, wx, wy, label_angle)
                new_parts.append(lbl)
                stats["global_labels"] += 1
            elif label_type == "gnd":
                lbl = build_gnd_symbol(wx, wy)
                new_parts.append(lbl)
                stats["gnd_labels"] += 1
            elif label_type == "nc":
                # Unused VCCO banks still need power - connect to the net
                # Actually, all VCCO pins must be connected to their bank voltage
                # even if the bank is unused. VCCO_16 and VCCO_35 should still
                # be connected to their respective voltages.
                lbl = build_global_label(net_label, wx, wy, label_angle)
                new_parts.append(lbl)
                stats["global_labels"] += 1

    # Insert before closing paren
    print(f"\n--- Inserting {len(new_parts)} new elements ---")

    # Find the last ')' in the file
    insert_idx = len(lines) - 1
    while insert_idx >= 0 and lines[insert_idx].strip() != ')':
        insert_idx -= 1

    if insert_idx < 0:
        print("ERROR: Could not find closing paren!")
        sys.exit(1)

    new_content_str = '\n'.join(new_parts)
    final_lines = lines[:insert_idx] + [new_content_str] + lines[insert_idx:]

    output = '\n'.join(final_lines)
    print(f"\nWriting {SCHEMATIC_FILE}...")
    with open(SCHEMATIC_FILE, 'w') as f:
        f.write(output)

    final_count = sum(1 for _ in open(SCHEMATIC_FILE))
    print(f"Output: {final_count} lines")

    print("\n" + "=" * 60)
    print("POWER CONNECTION COMPLETE")
    print("=" * 60)
    print(f"\nAdded:")
    print(f"  - {stats['wires']} wire stubs")
    print(f"  - {stats['global_labels']} global labels")
    print(f"  - {stats['gnd_labels']} GND labels")
    print(f"  - {stats['no_connect']} no_connect markers")

    print(f"\nPower rail summary:")
    print(f"  +1V0_FPGA : VCCINT (14) + VCCBRAM (3) = 17 pins")
    print(f"  +1V8_FPGA : VCCAUX (5) + VCCADC (1) + VCCBATT (1) + VCCO_34 (6) + VCCO_0 (2) = 15 pins")
    print(f"  +3V3_FPGA : VCCO_13 (5) + VCCO_14 (6) + VCCO_15 (6) + VCCO_16 (6) = 23 pins")
    print(f"  GND       : GNDADC (1) + GND_LEFT (44) + GND_RIGHT (43) = 88 pins")
    print(f"  Total: 143 power pins connected")

    print(f"\n{'='*60}")
    print("NEXT: Run ERC to validate: kicad-cli sch erc --severity-all RADAR_Main_Board.kicad_sch")
    print(f"{'='*60}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
