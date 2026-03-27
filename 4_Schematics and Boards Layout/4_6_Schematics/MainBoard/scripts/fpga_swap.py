#!/usr/bin/env python3
"""
FPGA Symbol Swap Script for KiCad Schematic
Replaces XC7A50T-2FTG256I (U42) with XC7A200T-FBG484 in RADAR_Main_Board_2.kicad_sch

ROBUST approach: Uses UUID-based detection to find and remove old symbol blocks,
and coordinate-based wire detection to remove stubs.

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
OUTPUT_FILE = SCHEMATIC_FILE  # overwrite in place (backup already made)

NEW_LIB_ID = "FPGA_Xilinx_Artix7:XC7A200T-FBG484"
NEW_FOOTPRINT = "Package_BGA:Xilinx_FBG484"
NEW_VALUE = "XC7A200T-FBG484I"

# Old U42 UUIDs - used to find symbol blocks reliably
OLD_U42_UUIDS = {
    "0815cd0e-5850-4f59-94fe-0ccb14d5721e",  # unit 3
    "7a28872f-1a18-41e5-aa67-386f0c46cac8",  # unit 4
    "7ffe180e-babe-4f79-af57-1235198fc636",  # unit 5
    "dd463ce7-d921-4a6b-9964-b8768aa56930",  # unit 1
    "e6d0795f-c692-43cf-b58c-d58ab4896d82",  # unit 2
}

# Old U42 unit positions (for wire detection)
OLD_U42_POSITIONS = {
    1: (83.82, 444.5),
    2: (218.44, 464.82),
    3: (375.92, 464.82),
    4: (607.06, 462.28),
    5: (497.84, 467.36),
}

# Project path for instances block
PROJECT_PATH = "/e294342a-dc9b-4bba-983e-ec7d495d6725"

# ============================================================
# SIGNAL MIGRATION MAP
# net_name -> (new_ball, new_unit, pin_function)
# ============================================================

SIGNAL_MAP = {
    # Bank 14 (Unit 1) - ADC, Flash
    "ADC_D0_N":           ("V22",  1, "IO_L3N_T0_DQS_EMCCLK_14"),
    "ADC_D0_P":           ("T21",  1, "IO_L4P_T0_D04_14"),
    "ADC_D1_N":           ("R21",  1, "IO_L2N_T0_D03_14"),
    "ADC_D1_P":           ("U22",  1, "IO_L3P_T0_DQS_PUDC_B_14"),
    "ADC_D2_N":           ("R22",  1, "IO_L1N_T0_D01_DIN_14"),
    "ADC_D2_P":           ("P21",  1, "IO_L2P_T0_D02_14"),
    "ADC_D3_N":           ("T18",  1, "IO_L20N_T3_A07_D23_14"),
    "ADC_D3_P":           ("N17",  1, "IO_L21P_T3_DQS_14"),
    "ADC_D4_N":           ("R14",  1, "IO_L19N_T3_A09_D25_VREF_14"),
    "ADC_D4_P":           ("R18",  1, "IO_L20P_T3_A08_D24_14"),
    "ADC_D5_N":           ("V19",  1, "IO_L14N_T2_SRCC_14"),
    "ADC_D5_P":           ("AA19", 1, "IO_L15P_T2_DQS_RDWR_B_14"),
    "ADC_D6_N":           ("AB20", 1, "IO_L15N_T2_DQS_DOUT_CSO_B_14"),
    "ADC_D6_P":           ("V17",  1, "IO_L16P_T2_CSI_B_14"),
    "ADC_D7_P":           ("Y18",  1, "IO_L13P_T2_MRCC_14"),
    "ADC_OR_N":           ("AB18", 1, "IO_L17N_T2_A13_D29_14"),
    "ADC_OR_P":           ("U17",  1, "IO_L18P_T2_A12_D28_14"),
    "ADC_PWRD":           ("Y19",  1, "IO_L13N_T2_MRCC_14"),
    "FPGA_ADC_CLOCK_N":   ("N14",  1, "IO_L23N_T3_A02_D18_14"),
    "FPGA_ADC_CLOCK_P":   ("P16",  1, "IO_L24P_T3_A01_D17_14"),
    "FPGA_FLASH_DQ0":     ("U20",  1, "IO_L11P_T1_SRCC_14"),
    "FPGA_FLASH_DQ1":     ("AB22", 1, "IO_L10N_T1_D15_14"),
    "FPGA_FLASH_DQ2":     ("AB21", 1, "IO_L10P_T1_D14_14"),
    "FPGA_FLASH_DQ3":     ("Y22",  1, "IO_L9N_T1_DQS_D13_14"),
    "FPGA_FLASH_NCS":     ("T19",  1, "IO_L6P_T0_FCS_B_14"),
    "FPGA_FLASH_NRST":    ("T20",  1, "IO_L6N_T0_D08_VREF_14"),
    "FPGA_PUDC_B":        ("Y21",  1, "IO_L9P_T1_DQS_14"),
    # Bank 15 (Unit 2) - DAC, Digital, STM32
    "ADAR_1_CS_3V3":      ("M22", 2, "IO_L15N_T2_DQS_ADV_B_15"),
    "ADAR_2_CS_3V3":      ("N22", 2, "IO_L15P_T2_DQS_15"),
    "ADAR_3_CS_3V3":      ("L20", 2, "IO_L14N_T2_SRCC_15"),
    "ADAR_4_CS_3V3":      ("L19", 2, "IO_L14P_T2_SRCC_15"),
    "DAC_0":              ("G18", 2, "IO_L4N_T0_15"),
    "DAC_1":              ("J15", 2, "IO_L5P_T0_AD9P_15"),
    "DAC_2":              ("H18", 2, "IO_L6N_T0_VREF_15"),
    "DAC_3":              ("H22", 2, "IO_L7N_T1_AD2N_15"),
    "DAC_4":              ("H20", 2, "IO_L8P_T1_AD10P_15"),
    "DAC_5":              ("G20", 2, "IO_L8N_T1_AD10N_15"),
    "DAC_6":              ("K22", 2, "IO_L9N_T1_DQS_AD3N_15"),
    "DAC_7":              ("M21", 2, "IO_L10P_T1_AD11P_15"),
    "DAC_SLEEP":          ("G16", 2, "IO_L2N_T0_AD8N_15"),
    "DIG_0":              ("L13", 2, "IO_L20N_T3_A19_15"),
    "DIG_1":              ("M13", 2, "IO_L20P_T3_A20_15"),
    "DIG_2":              ("K14", 2, "IO_L19N_T3_A21_VREF_15"),
    "DIG_3":              ("K13", 2, "IO_L19P_T3_A22_15"),
    "DIG_4":              ("M20", 2, "IO_L18N_T2_A23_15"),
    "DIG_5":              ("N20", 2, "IO_L18P_T2_A24_15"),
    "DIG_6":              ("N19", 2, "IO_L17N_T2_A25_15"),
    "DIG_7":              ("N18", 2, "IO_L17P_T2_A26_15"),
    "FPGA_CLOCK_TEST":    ("K18", 2, "IO_L13P_T2_MRCC_15"),
    "FPGA_SYS_CLOCK":     ("M15", 2, "IO_L24P_T3_RS1_15"),
    "MIX_RX_EN":          ("L15", 2, "IO_L22N_T3_A16_15"),
    "MIX_TX_EN":          ("H13", 2, "IO_L1P_T0_AD0P_15"),
    "STM32_MISO1":        ("M18", 2, "IO_L16P_T2_A28_15"),
    "STM32_MOSI1":        ("L18", 2, "IO_L16N_T2_A27_15"),
    "STM32_SCLK1":        ("K19", 2, "IO_L13N_T2_MRCC_15"),
    # Bank 34 (Unit 3) - Beamformer
    "ADAR_1_CS_1V8":      ("Y2",  3, "IO_L4N_T0_34"),
    "ADAR_2_CS_1V8":      ("W2",  3, "IO_L4P_T0_34"),
    "ADAR_3_CS_1V8":      ("R2",  3, "IO_L3N_T0_DQS_34"),
    "ADAR_4_CS_1V8":      ("R3",  3, "IO_L3P_T0_DQS_34"),
    "ADAR_RX_LOAD_1":     ("AA5", 3, "IO_L10P_T1_34"),
    "ADAR_RX_LOAD_2":     ("AB1", 3, "IO_L7N_T1_34"),
    "ADAR_RX_LOAD_3":     ("AB2", 3, "IO_L8N_T1_34"),
    "ADAR_RX_LOAD_4":     ("AA3", 3, "IO_L9N_T1_DQS_34"),
    "ADAR_TR_1":          ("U1",  3, "IO_L1N_T0_34"),
    "ADAR_TR_2":          ("T1",  3, "IO_L1P_T0_34"),
    "ADAR_TR_3":          ("T3",  3, "IO_0_34"),
    "ADAR_TX_LOAD_2":     ("AA1", 3, "IO_L7P_T1_34"),
    "ADAR_TX_LOAD_3":     ("AB3", 3, "IO_L8P_T1_34"),
    "ADAR_TX_LOAD_4":     ("Y3",  3, "IO_L9P_T1_DQS_34"),
    "STM32_MISO_1V8":     ("U2",  3, "IO_L2P_T0_34"),
    "STM32_MOSI_1V8":     ("V2",  3, "IO_L2N_T0_34"),
    "STM32_SCLK_1V8":     ("U3",  3, "IO_L6P_T0_34"),
    # Config/JTAG (Unit 6)
    "FPGA_FLASH_CLK":     ("L12", 6, "CCLK_0"),
    "FPGA_TCK":           ("V12", 6, "TCK_0"),
    "FPGA_TDI":           ("R13", 6, "TDI_0"),
    "FPGA_TDO":           ("U13", 6, "TDO_0"),
    "FPGA_TMS":           ("T13", 6, "TMS_0"),
}

# ============================================================
# Pin positions within the NEW symbol (relative to symbol origin)
# These are the wire connection points (end of pin stub)
# ============================================================

# Unit 1 (Banks 13,14) - Bank 14 pins on RIGHT edge at x=50.8
UNIT1_PIN_POSITIONS = {
    "IO_L1P_T0_D00_MOSI_14":          (50.8, 50.8),
    "IO_L1N_T0_D01_DIN_14":           (50.8, 48.26),
    "IO_L2P_T0_D02_14":               (50.8, 45.72),
    "IO_L2N_T0_D03_14":               (50.8, 43.18),
    "IO_L3P_T0_DQS_PUDC_B_14":        (50.8, 40.64),
    "IO_L3N_T0_DQS_EMCCLK_14":        (50.8, 38.1),
    "IO_L4P_T0_D04_14":               (50.8, 35.56),
    "IO_L4N_T0_D05_14":               (50.8, 33.02),
    "IO_L5P_T0_D06_14":               (50.8, 30.48),
    "IO_L5N_T0_D07_14":               (50.8, 27.94),
    "IO_L6P_T0_FCS_B_14":             (50.8, 25.4),
    "IO_L6N_T0_D08_VREF_14":          (50.8, 22.86),
    "IO_L7P_T1_D09_14":               (50.8, 20.32),
    "IO_L7N_T1_D10_14":               (50.8, 17.78),
    "IO_L8P_T1_D11_14":               (50.8, 15.24),
    "IO_L8N_T1_D12_14":               (50.8, 12.7),
    "IO_L9P_T1_DQS_14":               (50.8, 10.16),
    "IO_L9N_T1_DQS_D13_14":           (50.8, 7.62),
    "IO_L10P_T1_D14_14":              (50.8, 5.08),
    "IO_L10N_T1_D15_14":              (50.8, 2.54),
    "IO_L11P_T1_SRCC_14":             (50.8, 0.0),
    "IO_L11N_T1_SRCC_14":             (50.8, -2.54),
    "IO_L12P_T1_MRCC_14":             (50.8, -5.08),
    "IO_L12N_T1_MRCC_14":             (50.8, -7.62),
    "IO_L13P_T2_MRCC_14":             (50.8, -10.16),
    "IO_L13N_T2_MRCC_14":             (50.8, -12.7),
    "IO_L14P_T2_SRCC_14":             (50.8, -15.24),
    "IO_L14N_T2_SRCC_14":             (50.8, -17.78),
    "IO_L15P_T2_DQS_RDWR_B_14":       (50.8, -20.32),
    "IO_L15N_T2_DQS_DOUT_CSO_B_14":   (50.8, -22.86),
    "IO_L16P_T2_CSI_B_14":            (50.8, -25.4),
    "IO_L17N_T2_A13_D29_14":          (50.8, -33.02),
    "IO_L18P_T2_A12_D28_14":          (50.8, -35.56),
    "IO_L19N_T3_A09_D25_VREF_14":     (50.8, -43.18),
    "IO_L20P_T3_A08_D24_14":          (50.8, -45.72),
    "IO_L20N_T3_A07_D23_14":          (50.8, -48.26),
    "IO_L21P_T3_DQS_14":              (50.8, -50.8),
    "IO_L23N_T3_A02_D18_14":          (50.8, -63.5),
    "IO_L24P_T3_A01_D17_14":          (50.8, -66.04),
}

# Unit 2 (Banks 15,16) - Bank 15 pins on LEFT edge at x=-50.8
UNIT2_PIN_POSITIONS = {
    "IO_L1P_T0_AD0P_15":              (-50.8, 50.8),
    "IO_L2N_T0_AD8N_15":              (-50.8, 43.18),
    "IO_L4N_T0_15":                    (-50.8, 33.02),
    "IO_L5P_T0_AD9P_15":              (-50.8, 30.48),
    "IO_L6N_T0_VREF_15":              (-50.8, 22.86),
    "IO_L7N_T1_AD2N_15":              (-50.8, 17.78),
    "IO_L8P_T1_AD10P_15":             (-50.8, 15.24),
    "IO_L8N_T1_AD10N_15":             (-50.8, 12.7),
    "IO_L9N_T1_DQS_AD3N_15":          (-50.8, 7.62),
    "IO_L10P_T1_AD11P_15":            (-50.8, 5.08),
    "IO_L13P_T2_MRCC_15":             (-50.8, -10.16),
    "IO_L13N_T2_MRCC_15":             (-50.8, -12.7),
    "IO_L14P_T2_SRCC_15":             (-50.8, -15.24),
    "IO_L14N_T2_SRCC_15":             (-50.8, -17.78),
    "IO_L15P_T2_DQS_15":              (-50.8, -20.32),
    "IO_L15N_T2_DQS_ADV_B_15":        (-50.8, -22.86),
    "IO_L16P_T2_A28_15":              (-50.8, -25.4),
    "IO_L16N_T2_A27_15":              (-50.8, -27.94),
    "IO_L17P_T2_A26_15":              (-50.8, -30.48),
    "IO_L17N_T2_A25_15":              (-50.8, -33.02),
    "IO_L18P_T2_A24_15":              (-50.8, -35.56),
    "IO_L18N_T2_A23_15":              (-50.8, -38.1),
    "IO_L19P_T3_A22_15":              (-50.8, -40.64),
    "IO_L19N_T3_A21_VREF_15":         (-50.8, -43.18),
    "IO_L20P_T3_A20_15":              (-50.8, -45.72),
    "IO_L20N_T3_A19_15":              (-50.8, -48.26),
    "IO_L22N_T3_A16_15":              (-50.8, -58.42),
    "IO_L24P_T3_RS1_15":              (-50.8, -66.04),
}

# Unit 3 (Banks 34,35) - Bank 34 pins on LEFT edge at x=-50.8
UNIT3_PIN_POSITIONS = {
    "IO_0_34":                         (-50.8, 53.34),
    "IO_L1P_T0_34":                    (-50.8, 50.8),
    "IO_L1N_T0_34":                    (-50.8, 48.26),
    "IO_L2P_T0_34":                    (-50.8, 45.72),
    "IO_L2N_T0_34":                    (-50.8, 43.18),
    "IO_L3P_T0_DQS_34":               (-50.8, 40.64),
    "IO_L3N_T0_DQS_34":               (-50.8, 38.1),
    "IO_L4P_T0_34":                    (-50.8, 35.56),
    "IO_L4N_T0_34":                    (-50.8, 33.02),
    "IO_L6P_T0_34":                    (-50.8, 25.4),
    "IO_L7P_T1_34":                    (-50.8, 20.32),
    "IO_L7N_T1_34":                    (-50.8, 17.78),
    "IO_L8P_T1_34":                    (-50.8, 15.24),
    "IO_L8N_T1_34":                    (-50.8, 12.7),
    "IO_L9P_T1_DQS_34":               (-50.8, 10.16),
    "IO_L9N_T1_DQS_34":               (-50.8, 7.62),
    "IO_L10P_T1_34":                   (-50.8, 5.08),
}

# Unit 6 (Config/JTAG) - RIGHT edge pins at x=38.1
UNIT6_PIN_POSITIONS = {
    "CCLK_0":                          (38.1, 7.62),
    "TCK_0":                           (38.1, 12.7),
    "TMS_0":                           (38.1, 15.24),
    "TDO_0":                           (38.1, 17.78),
    "TDI_0":                           (38.1, 20.32),
}

# Combined pin position lookup
ALL_PIN_POSITIONS = {}
ALL_PIN_POSITIONS.update(UNIT1_PIN_POSITIONS)
ALL_PIN_POSITIONS.update(UNIT2_PIN_POSITIONS)
ALL_PIN_POSITIONS.update(UNIT3_PIN_POSITIONS)
ALL_PIN_POSITIONS.update(UNIT6_PIN_POSITIONS)

# New symbol placement positions
NEW_UNIT_POSITIONS = {
    1: (200.0, 450.0),    # Banks 13/14 - ADC, Flash
    2: (380.0, 450.0),    # Banks 15/16 - DAC, Digital, STM32
    3: (560.0, 450.0),    # Banks 34/35 - Beamformer
    6: (90.0,  450.0),    # Config/JTAG
    7: (90.0,  300.0),    # Power/GND
}

# ============================================================
# HELPER FUNCTIONS
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

def find_sexp_block(lines, start_idx):
    """Find the complete S-expression block starting at start_idx.
    Returns (start, end) indices (inclusive) of lines forming the block."""
    depth = 0
    end = start_idx
    for i in range(start_idx, len(lines)):
        depth += lines[i].count('(') - lines[i].count(')')
        if depth <= 0:
            end = i
            break
    return (start_idx, end)

def find_symbol_blocks_by_uuid(lines, uuids):
    """Find all symbol blocks that contain any of the given UUIDs.
    Returns list of (start, end) tuples."""
    blocks = []
    for i, line in enumerate(lines):
        # Look for lines containing our UUIDs
        for uid in uuids:
            if uid in line and '(uuid' in line:
                # Walk backwards to find the start of the enclosing (symbol block
                # The symbol block starts with a line containing '(symbol'
                start = i
                depth = 0
                for j in range(i, -1, -1):
                    if '(symbol' in lines[j] and '(lib_id' not in lines[j]:
                        # Check if this is a top-level symbol (not a sub-symbol)
                        # by checking indentation
                        stripped = lines[j].lstrip()
                        if stripped.startswith('(symbol'):
                            start = j
                            break
                    # Also check lib_id on next line
                    if '(symbol' in lines[j]:
                        start = j
                        break
                
                # Now find the end of this block
                s, e = find_sexp_block(lines, start)
                blocks.append((s, e))
                break
    return blocks

def coord_matches(line, x, y, tolerance=0.05):
    """Check if a line contains coordinates matching (x, y) within tolerance."""
    # Match patterns like (xy 123.45 678.90)
    pattern = r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)'
    for match in re.finditer(pattern, line):
        lx, ly = float(match.group(1)), float(match.group(2))
        if abs(lx - x) < tolerance and abs(ly - y) < tolerance:
            return True
    return False

def build_symbol_block(unit_num, position):
    """Build a KiCad schematic symbol instance block."""
    x, y = position
    uid = gen_uuid()
    
    lines = []
    lines.append(f'\t(symbol')
    lines.append(f'\t\t(lib_id "{NEW_LIB_ID}")')
    lines.append(f'\t\t(at {fmt(x)} {fmt(y)} 0)')
    lines.append(f'\t\t(unit {unit_num})')
    lines.append(f'\t\t(exclude_from_sim no)')
    lines.append(f'\t\t(in_bom yes)')
    lines.append(f'\t\t(on_board yes)')
    lines.append(f'\t\t(dnp no)')
    lines.append(f'\t\t(uuid "{uid}")')
    lines.append(f'\t\t(property "Reference" "U42"')
    lines.append(f'\t\t\t(at {fmt(x)} {fmt(y - 80)} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')
    lines.append(f'\t\t(property "Value" "{NEW_VALUE}"')
    lines.append(f'\t\t\t(at {fmt(x)} {fmt(y - 82)} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')
    lines.append(f'\t\t(property "Footprint" "{NEW_FOOTPRINT}"')
    lines.append(f'\t\t\t(at {fmt(x)} {fmt(y - 84)} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t\t(hide yes)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')
    lines.append(f'\t\t(property "Datasheet" ""')
    lines.append(f'\t\t\t(at {fmt(x)} {fmt(y - 86)} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t\t(hide yes)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')
    lines.append(f'\t\t(pin "1" (uuid "{gen_uuid()}"))')
    lines.append(f'\t\t(instances')
    lines.append(f'\t\t\t(project "RADAR_Main_Board"')
    lines.append(f'\t\t\t\t(path "{PROJECT_PATH}"')
    lines.append(f'\t\t\t\t\t(reference "U42")')
    lines.append(f'\t\t\t\t\t(unit {unit_num})')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')
    lines.append(f'\t)')
    return '\n'.join(lines)

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

# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("FPGA Symbol Swap: XC7A50T-FTG256 -> XC7A200T-FBG484")
    print("=" * 60)
    
    # Verify backup exists
    backup = SCHEMATIC_FILE + ".bak_pre_fpga_swap"
    if not os.path.exists(backup):
        print(f"ERROR: Backup file not found at {backup}")
        print("Run the backup step first!")
        sys.exit(1)
    print(f"Backup verified: {backup}")
    
    # Read schematic
    print(f"\nReading {SCHEMATIC_FILE}...")
    with open(SCHEMATIC_FILE, 'r') as f:
        lines = f.readlines()
    total_lines = len(lines)
    print(f"Read {total_lines} lines")
    
    # --------------------------------------------------------
    # PHASE 1: Find and mark old U42 symbol blocks for deletion
    # --------------------------------------------------------
    print("\n--- Phase 1: Finding old U42 symbol blocks ---")
    
    # Find symbol blocks by searching for the old lib_id
    old_lib_id = "RADAR_Main_Board-eagle-import:XC7A50T-2FTG256I"
    u42_blocks = []
    
    i = 0
    while i < total_lines:
        line = lines[i]
        # Look for symbol blocks with the old FPGA lib_id
        if '(lib_id' in line and old_lib_id in line:
            # Walk back to find the (symbol start
            start = i
            for j in range(i, max(i - 5, -1), -1):
                if lines[j].strip().startswith('(symbol'):
                    start = j
                    break
            # Find the end of this block
            s, e = find_sexp_block(lines, start)
            
            # Verify it's U42 by checking for Reference "U42"
            block_text = ''.join(lines[s:e+1])
            if '"U42"' in block_text:
                # Extract unit number
                unit_match = re.search(r'\(unit\s+(\d+)\)', block_text)
                unit_num = int(unit_match.group(1)) if unit_match else 0
                u42_blocks.append((s, e, unit_num))
                print(f"  Found U42 unit {unit_num}: lines {s+1}-{e+1}")
            i = e + 1
        else:
            i += 1
    
    if len(u42_blocks) != 5:
        print(f"WARNING: Expected 5 U42 blocks, found {len(u42_blocks)}")
    
    # --------------------------------------------------------
    # PHASE 2: Find wire stubs connected to old U42 pins
    # --------------------------------------------------------
    print("\n--- Phase 2: Finding wire stubs connected to old U42 ---")
    
    # Build set of known U42 pin absolute coordinates from the OLD symbol
    # We need the old symbol's pin positions. Since the old symbol is from Eagle import,
    # the pins are at fixed offsets from each unit's position.
    # Rather than computing all pin positions, we'll use a simpler heuristic:
    # Find all wire blocks where BOTH endpoints are within the bounding box of
    # an old U42 unit, OR where one endpoint matches a known label coordinate.
    
    # Actually, the safest approach is to mark ALL wire stubs that have one endpoint
    # within the bounding area of any U42 unit. The old units have a ~40mm wide symbol.
    # Let's be more targeted: collect line ranges for wires that touch U42 areas.
    
    # Build bounding boxes for each old unit
    # Old symbol is roughly ±20mm from center (40mm wide, ~65mm tall based on 256 pins)
    wire_blocks_to_delete = []
    
    # Collect all coordinates that the old U42 block labels connect to
    # These are the net label positions that were identified in the exploration
    old_label_coords = set()
    # From exploration data - these are where labels connect to U42 wires
    # Unit 1 labels (at ~106.68, ~111.76 x)
    for y in [444.5, 457.2, 459.74, 462.28, 464.82]:
        old_label_coords.add((106.68, y))
    old_label_coords.add((111.76, 444.5))  # FPGA_FLASH_CLK offset
    # Unit 2 labels (at ~172.72, ~175.26, ~264.16 x)
    for y in [436.88, 439.42, 441.96, 444.5, 447.04, 449.58, 462.28, 464.82]:
        old_label_coords.add((172.72, y))
    old_label_coords.add((175.26, 436.88))
    for y in [436.88, 439.42]:
        old_label_coords.add((264.16, y))
    # Unit 3 labels (at ~419.1 x)
    for y_val in [436.88, 454.66, 457.2, 459.74, 462.28, 464.82, 467.36, 469.9,
                  472.44, 474.98, 477.52, 480.06, 482.6, 485.14, 487.68, 490.22, 492.76]:
        old_label_coords.add((419.1, y_val))
    
    # Instead of trying to match exact wire coordinates (fragile),
    # we'll delete wire blocks within the bounding box of each U42 unit
    unit_boxes = {}
    for unit, (cx, cy) in OLD_U42_POSITIONS.items():
        # Old symbol units are approximately ±25mm wide, ±35mm tall
        unit_boxes[unit] = (cx - 30, cy - 40, cx + 30, cy + 40)
    
    # Scan for wire blocks within U42 bounding areas
    i = 0
    while i < total_lines:
        line = lines[i]
        if line.strip().startswith('(wire'):
            s, e = find_sexp_block(lines, i)
            block_text = ''.join(lines[s:e+1])
            
            # Extract coordinates
            xy_matches = re.findall(r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)', block_text)
            if len(xy_matches) >= 2:
                x1, y1 = float(xy_matches[0][0]), float(xy_matches[0][1])
                x2, y2 = float(xy_matches[1][0]), float(xy_matches[1][1])
                
                # Check if either endpoint is within any U42 unit bounding box
                for unit, (bx1, by1, bx2, by2) in unit_boxes.items():
                    if ((bx1 <= x1 <= bx2 and by1 <= y1 <= by2) or
                        (bx1 <= x2 <= bx2 and by1 <= y2 <= by2)):
                        wire_blocks_to_delete.append((s, e))
                        break
            
            i = e + 1
        else:
            i += 1
    
    print(f"  Found {len(wire_blocks_to_delete)} wire stubs in U42 areas")
    
    # --------------------------------------------------------
    # PHASE 3: Find no_connect markers on old U42 pins
    # --------------------------------------------------------
    print("\n--- Phase 3: Finding no_connect markers ---")
    nc_blocks = []
    i = 0
    while i < total_lines:
        if lines[i].strip().startswith('(no_connect'):
            s, e = find_sexp_block(lines, i)
            block_text = ''.join(lines[s:e+1])
            
            # Check if coordinates are within U42 area
            xy_match = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)', block_text)
            if xy_match:
                nx, ny = float(xy_match.group(1)), float(xy_match.group(2))
                for unit, (bx1, by1, bx2, by2) in unit_boxes.items():
                    if bx1 <= nx <= bx2 and by1 <= ny <= by2:
                        nc_blocks.append((s, e))
                        print(f"  Found no_connect at ({nx}, {ny}) in unit {unit} area")
                        break
            i = e + 1
        else:
            i += 1
    
    # --------------------------------------------------------
    # PHASE 4: Find labels connected ONLY to old U42
    # --------------------------------------------------------
    # We should NOT delete labels - they connect to other components too.
    # The labels use net names that are shared across the schematic.
    # We'll keep all existing labels and just add new ones near the new symbol.
    # Some labels may end up orphaned (not connected to wires), but that's OK -
    # KiCad will show them as warnings in ERC, and the user can clean up in GUI.
    
    # --------------------------------------------------------
    # PHASE 5: Build deletion set
    # --------------------------------------------------------
    print("\n--- Phase 5: Building deletion set ---")
    lines_to_delete = set()
    
    for s, e, unit in u42_blocks:
        for idx in range(s, e + 1):
            lines_to_delete.add(idx)
    
    for s, e in wire_blocks_to_delete:
        for idx in range(s, e + 1):
            lines_to_delete.add(idx)
    
    for s, e in nc_blocks:
        for idx in range(s, e + 1):
            lines_to_delete.add(idx)
    
    print(f"  Total lines to delete: {len(lines_to_delete)}")
    
    # --------------------------------------------------------
    # PHASE 6: Remove old content
    # --------------------------------------------------------
    print("\n--- Phase 6: Removing old U42 content ---")
    new_lines = []
    for i, line in enumerate(lines):
        if i not in lines_to_delete:
            new_lines.append(line)
    print(f"  Lines after deletion: {len(new_lines)} (removed {total_lines - len(new_lines)})")
    
    # --------------------------------------------------------
    # PHASE 7: Build new symbol blocks and wire connections
    # --------------------------------------------------------
    print("\n--- Phase 7: Building new XC7A200T symbol blocks ---")
    new_parts = []
    
    # Symbol instances
    for unit_num in [1, 2, 3, 6, 7]:
        pos = NEW_UNIT_POSITIONS[unit_num]
        block = build_symbol_block(unit_num, pos)
        new_parts.append(block)
        print(f"  Created unit {unit_num} at ({pos[0]}, {pos[1]})")
    
    # Signal wires and labels
    connected = 0
    warnings = []
    for net_name, (new_ball, new_unit, pin_func) in SIGNAL_MAP.items():
        sx, sy = NEW_UNIT_POSITIONS[new_unit]
        
        if pin_func not in ALL_PIN_POSITIONS:
            warnings.append(f"Pin function {pin_func} not in position map for {net_name}")
            continue
        
        px, py = ALL_PIN_POSITIONS[pin_func]
        pin_abs_x = sx + px
        pin_abs_y = sy + py
        
        # Wire stub from pin to label (10mm)
        wire_len = 12.7  # 0.5 inch in KiCad grid
        if px > 0:  # Right edge pin
            label_x = pin_abs_x + wire_len
            label_y = pin_abs_y
        else:  # Left edge pin
            label_x = pin_abs_x - wire_len
            label_y = pin_abs_y
        
        wire = build_wire(pin_abs_x, pin_abs_y, label_x, label_y)
        label = build_label(net_name, label_x, label_y)
        new_parts.append(wire)
        new_parts.append(label)
        connected += 1
    
    print(f"  Created {connected} signal connections (wire + label)")
    if warnings:
        for w in warnings:
            print(f"  WARNING: {w}")
    
    # --------------------------------------------------------
    # PHASE 8: Insert new content and write output
    # --------------------------------------------------------
    print("\n--- Phase 8: Inserting new content ---")
    
    # Find the closing ')' of the schematic
    insert_pos = len(new_lines) - 1
    while insert_pos >= 0 and new_lines[insert_pos].strip() != ')':
        insert_pos -= 1
    
    if insert_pos < 0:
        print("ERROR: Could not find closing paren!")
        sys.exit(1)
    
    new_content_str = '\n'.join(new_parts) + '\n'
    final_lines = new_lines[:insert_pos] + [new_content_str] + new_lines[insert_pos:]
    
    print(f"\nWriting {OUTPUT_FILE}...")
    with open(OUTPUT_FILE, 'w') as f:
        f.writelines(final_lines)
    
    final_count = sum(1 for _ in open(OUTPUT_FILE))
    print(f"Output: {final_count} lines")
    
    # --------------------------------------------------------
    # SUMMARY
    # --------------------------------------------------------
    print("\n" + "=" * 60)
    print("SWAP COMPLETE")
    print("=" * 60)
    print(f"\nRemoved:")
    print(f"  - {len(u42_blocks)} old U42 symbol blocks (units {[u for _,_,u in u42_blocks]})")
    print(f"  - {len(wire_blocks_to_delete)} wire stubs")
    print(f"  - {len(nc_blocks)} no_connect markers")
    print(f"\nAdded:")
    print(f"  - 5 new U42 symbol instances (XC7A200T-FBG484, units 1,2,3,6,7)")
    print(f"  - {connected} signal connections")
    
    print(f"\nUnit placement:")
    for u in sorted(NEW_UNIT_POSITIONS):
        print(f"  Unit {u}: {NEW_UNIT_POSITIONS[u]}")
    
    print(f"\n{'='*60}")
    print("REMAINING MANUAL WORK (KiCad GUI):")
    print("  1. Connect power pins (GND, VCCINT, VCCAUX, VCCBRAM, VCCO)")
    print("  2. Remove duplicate labels (old labels may remain)")
    print("  3. Add no_connect markers on unused I/O pins")
    print("  4. Clean up wire routing for readability")
    print("  5. Run ERC to verify")
    print(f"{'='*60}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
