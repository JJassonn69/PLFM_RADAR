#!/usr/bin/env python3
"""
FPGA PCB Footprint Swap Script
Replaces U42 BGA256 footprint with Xilinx_FBG484 in the PCB file.
Assigns correct nets to all pads based on the signal and power migration maps.

Author: OpenCode FPGA swap automation
"""

import re
import uuid as uuid_mod
import sys
import os

# ============================================================
# CONFIGURATION
# ============================================================

PCB_FILE = "/Users/ganeshpanth/PLFM_RADAR/4_Schematics and Boards Layout/4_6_Schematics/MainBoard/RADAR_Main_Board.kicad_pcb"
NEW_FP_FILE = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/Package_BGA.pretty/Xilinx_FBG484.kicad_mod"

# U42 position (unchanged)
U42_X = 68.1811
U42_Y = 71.7236

# ============================================================
# COMPLETE BALL-TO-NET MAP FOR XC7A200T-FBG484
# Combines signal map + power map
# ============================================================

BALL_NET_MAP = {
    # ===== SIGNAL PINS (77 connections) =====
    # Bank 14 (Unit 1) - ADC, Flash
    "V22": "ADC_D0_N",
    "T21": "ADC_D0_P",
    "R21": "ADC_D1_N",
    "U22": "ADC_D1_P",
    "R22": "ADC_D2_N",
    "P21": "ADC_D2_P",
    "T18": "ADC_D3_N",
    "N17": "ADC_D3_P",
    "R14": "ADC_D4_N",
    "R18": "ADC_D4_P",
    "V19": "ADC_D5_N",
    "AA19": "ADC_D5_P",
    "AB20": "ADC_D6_N",
    "V17": "ADC_D6_P",
    "Y18": "ADC_D7_P",
    "AB18": "ADC_OR_N",
    "U17": "ADC_OR_P",
    "Y19": "ADC_PWRD",
    "N14": "FPGA_ADC_CLOCK_N",
    "P16": "FPGA_ADC_CLOCK_P",
    "U20": "FPGA_FLASH_DQ0",
    "AB22": "FPGA_FLASH_DQ1",
    "AB21": "FPGA_FLASH_DQ2",
    "Y22": "FPGA_FLASH_DQ3",
    "T19": "FPGA_FLASH_NCS",
    "T20": "FPGA_FLASH_NRST",
    "Y21": "FPGA_PUDC_B",
    # Bank 15 (Unit 2) - DAC, Digital, STM32
    "M22": "ADAR_1_CS_3V3",
    "N22": "ADAR_2_CS_3V3",
    "L20": "ADAR_3_CS_3V3",
    "L19": "ADAR_4_CS_3V3",
    "G18": "DAC_0",
    "J15": "DAC_1",
    "H18": "DAC_2",
    "H22": "DAC_3",
    "H20": "DAC_4",
    "G20": "DAC_5",
    "K22": "DAC_6",
    "M21": "DAC_7",
    "G16": "DAC_SLEEP",
    "L13": "DIG_0",
    "M13": "DIG_1",
    "K14": "DIG_2",
    "K13": "DIG_3",
    "M20": "DIG_4",
    "N20": "DIG_5",
    "N19": "DIG_6",
    "N18": "DIG_7",
    "K18": "FPGA_CLOCK_TEST",
    "M15": "FPGA_SYS_CLOCK",
    "L15": "MIX_RX_EN",
    "H13": "MIX_TX_EN",
    "M18": "STM32_MISO1",
    "L18": "STM32_MOSI1",
    "K19": "STM32_SCLK1",
    # Bank 34 (Unit 3) - Beamformer
    "Y2": "ADAR_1_CS_1V8",
    "W2": "ADAR_2_CS_1V8",
    "R2": "ADAR_3_CS_1V8",
    "R3": "ADAR_4_CS_1V8",
    "AA5": "ADAR_RX_LOAD_1",
    "AB1": "ADAR_RX_LOAD_2",
    "AB2": "ADAR_RX_LOAD_3",
    "AA3": "ADAR_RX_LOAD_4",
    "U1": "ADAR_TR_1",
    "T1": "ADAR_TR_2",
    "T3": "ADAR_TR_3",
    "AA1": "ADAR_TX_LOAD_2",
    "AB3": "ADAR_TX_LOAD_3",
    "Y3": "ADAR_TX_LOAD_4",
    "U2": "STM32_MISO_1V8",
    "V2": "STM32_MOSI_1V8",
    "U3": "STM32_SCLK_1V8",
    # Config/JTAG (Unit 6)
    "L12": "FPGA_FLASH_CLK",
    "V12": "FPGA_TCK",
    "R13": "FPGA_TDI",
    "U13": "FPGA_TDO",
    "T13": "FPGA_TMS",

    # ===== POWER PINS =====
    # VCCO_13 -> +3V3_FPGA
    "AA17": "+3V3_FPGA", "AB14": "+3V3_FPGA", "V16": "+3V3_FPGA",
    "W13": "+3V3_FPGA", "Y10": "+3V3_FPGA",
    # VCCO_14 -> +3V3_FPGA
    "M14": "+3V3_FPGA", "P18": "+3V3_FPGA", "R15": "+3V3_FPGA",
    "T22": "+3V3_FPGA", "U19": "+3V3_FPGA", "Y20": "+3V3_FPGA",
    # VCCO_15 -> +3V3_FPGA
    "G19": "+3V3_FPGA", "H16": "+3V3_FPGA", "J13": "+3V3_FPGA",
    "K20": "+3V3_FPGA", "L17": "+3V3_FPGA", "N21": "+3V3_FPGA",
    # VCCO_16 -> +3V3_FPGA (unused bank, still needs VCCO)
    "A17": "+3V3_FPGA", "B14": "+3V3_FPGA", "C21": "+3V3_FPGA",
    "D18": "+3V3_FPGA", "E15": "+3V3_FPGA", "F22": "+3V3_FPGA",
    # VCCO_34 -> +1V8_FPGA
    "AA7": "+1V8_FPGA", "AB4": "+1V8_FPGA", "R5": "+1V8_FPGA",
    "T2": "+1V8_FPGA", "V6": "+1V8_FPGA", "W3": "+1V8_FPGA",
    # VCCO_35 -> +1V8_FPGA (unused bank, still needs VCCO)
    "C1": "+1V8_FPGA", "F2": "+1V8_FPGA", "H6": "+1V8_FPGA",
    "J3": "+1V8_FPGA", "M4": "+1V8_FPGA", "N1": "+1V8_FPGA",
    # VCCO_0 -> +1V8_FPGA (config bank)
    "F12": "+1V8_FPGA", "T12": "+1V8_FPGA",
    # VCCAUX -> +1V8_FPGA
    "H12": "+1V8_FPGA", "K12": "+1V8_FPGA", "M12": "+1V8_FPGA",
    "P12": "+1V8_FPGA", "R11": "+1V8_FPGA",
    # VCCADC -> +1V8_FPGA
    "K10": "+1V8_FPGA",
    # VCCBATT -> +1V8_FPGA
    "E12": "+1V8_FPGA",
    # VCCBRAM -> +1V0_FPGA
    "J11": "+1V0_FPGA", "L11": "+1V0_FPGA", "N11": "+1V0_FPGA",
    # GNDADC -> GND
    "K9": "GND",
    # VCCINT -> +1V0_FPGA
    "H8": "+1V0_FPGA", "H10": "+1V0_FPGA", "J7": "+1V0_FPGA",
    "J9": "+1V0_FPGA", "K8": "+1V0_FPGA", "L7": "+1V0_FPGA",
    "M8": "+1V0_FPGA", "N7": "+1V0_FPGA", "P8": "+1V0_FPGA",
    "P10": "+1V0_FPGA", "R7": "+1V0_FPGA", "R9": "+1V0_FPGA",
    "T8": "+1V0_FPGA", "T10": "+1V0_FPGA",
    # GND (LEFT side, 44 pins)
    "A2": "GND", "A3": "GND", "A5": "GND", "A7": "GND", "A9": "GND",
    "A11": "GND", "A12": "GND", "A22": "GND", "B3": "GND", "B12": "GND",
    "B19": "GND", "C3": "GND", "C6": "GND", "C10": "GND", "C12": "GND",
    "C16": "GND", "D3": "GND", "D4": "GND", "D8": "GND", "D12": "GND",
    "D13": "GND", "E4": "GND", "E5": "GND", "E7": "GND", "E9": "GND",
    "E11": "GND", "E20": "GND", "F5": "GND", "F11": "GND", "F17": "GND",
    "G5": "GND", "G6": "GND", "G7": "GND", "G8": "GND", "G9": "GND",
    "G10": "GND", "G12": "GND", "G14": "GND", "H1": "GND", "H7": "GND",
    "H9": "GND", "H11": "GND", "H21": "GND", "J8": "GND",
    # GND (RIGHT side, 43 pins)
    "J10": "GND", "J12": "GND", "J18": "GND", "K5": "GND", "K7": "GND",
    "K11": "GND", "K15": "GND", "L2": "GND", "L8": "GND", "L22": "GND",
    "M7": "GND", "M11": "GND", "M19": "GND", "N6": "GND", "N8": "GND",
    "N16": "GND", "P3": "GND", "P7": "GND", "P9": "GND", "P11": "GND",
    "P13": "GND", "R8": "GND", "R10": "GND", "R12": "GND", "R20": "GND",
    "T7": "GND", "T9": "GND", "T11": "GND", "T17": "GND", "U4": "GND",
    "U14": "GND", "V1": "GND", "V11": "GND", "V21": "GND", "W8": "GND",
    "W18": "GND", "Y5": "GND", "Y15": "GND", "AA2": "GND", "AA12": "GND",
    "AA22": "GND", "AB9": "GND", "AB19": "GND",
}

def gen_uuid():
    return str(uuid_mod.uuid4())

# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("FPGA PCB Footprint Swap: BGA256 -> Xilinx_FBG484")
    print("=" * 60)

    # Verify backup
    backup = PCB_FILE + ".bak_pre_fpga_swap"
    if not os.path.exists(backup):
        print(f"ERROR: Backup not found: {backup}")
        sys.exit(1)
    print(f"Backup verified: {backup}")

    # Read PCB file
    print(f"\nReading {PCB_FILE}...")
    with open(PCB_FILE, 'r') as f:
        pcb_content = f.read()
    print(f"Read {len(pcb_content)} chars")

    # Read new footprint
    print(f"Reading {NEW_FP_FILE}...")
    with open(NEW_FP_FILE, 'r') as f:
        fp_content = f.read()
    print(f"Read {len(fp_content)} chars")

    # --------------------------------------------------------
    # PHASE 1: Find old U42 footprint block
    # --------------------------------------------------------
    print("\n--- Phase 1: Finding old U42 footprint ---")

    pos = pcb_content.find('(footprint "BGA256C100P16X16_1700X1700X155"')
    if pos < 0:
        print("ERROR: Old U42 footprint not found!")
        sys.exit(1)

    depth = 0
    end = pos
    for i in range(pos, len(pcb_content)):
        if pcb_content[i] == '(':
            depth += 1
        elif pcb_content[i] == ')':
            depth -= 1
            if depth == 0:
                end = i + 1
                break

    old_block = pcb_content[pos:end]
    line_start = pcb_content[:pos].count('\n') + 1
    line_end = pcb_content[:end].count('\n') + 1
    print(f"  Found at lines {line_start}-{line_end} ({len(old_block)} chars)")

    # --------------------------------------------------------
    # PHASE 2: Build new footprint block
    # --------------------------------------------------------
    print("\n--- Phase 2: Building new U42 footprint ---")

    # Start from the KiCad library footprint template
    # We need to:
    # 1. Replace the footprint name/reference
    # 2. Add the position (at) field
    # 3. Add UUID
    # 4. Update Reference and Value properties
    # 5. Add net assignments to pads

    # Parse the library footprint and rebuild it as an in-PCB footprint
    new_block_lines = []

    # Header
    new_block_lines.append(f'\t(footprint "Package_BGA:Xilinx_FBG484"')
    new_block_lines.append(f'\t\t(layer "F.Cu")')
    new_block_lines.append(f'\t\t(uuid "{gen_uuid()}")')
    new_block_lines.append(f'\t\t(at {U42_X} {U42_Y})')

    # Properties - Reference and Value
    new_block_lines.append(f'\t\t(property "Reference" "U42"')
    new_block_lines.append(f'\t\t\t(at -9.5 -9.5 0)')
    new_block_lines.append(f'\t\t\t(unlocked yes)')
    new_block_lines.append(f'\t\t\t(layer "F.SilkS")')
    new_block_lines.append(f'\t\t\t(uuid "{gen_uuid()}")')
    new_block_lines.append(f'\t\t\t(effects')
    new_block_lines.append(f'\t\t\t\t(font')
    new_block_lines.append(f'\t\t\t\t\t(size 1 1)')
    new_block_lines.append(f'\t\t\t\t\t(thickness 0.15)')
    new_block_lines.append(f'\t\t\t\t)')
    new_block_lines.append(f'\t\t\t\t(justify left bottom)')
    new_block_lines.append(f'\t\t\t)')
    new_block_lines.append(f'\t\t)')
    new_block_lines.append(f'\t\t(property "Value" "XC7A200T-FBG484I"')
    new_block_lines.append(f'\t\t\t(at 0 12.5 0)')
    new_block_lines.append(f'\t\t\t(unlocked yes)')
    new_block_lines.append(f'\t\t\t(layer "F.Fab")')
    new_block_lines.append(f'\t\t\t(uuid "{gen_uuid()}")')
    new_block_lines.append(f'\t\t\t(effects')
    new_block_lines.append(f'\t\t\t\t(font')
    new_block_lines.append(f'\t\t\t\t\t(size 1 1)')
    new_block_lines.append(f'\t\t\t\t\t(thickness 0.15)')
    new_block_lines.append(f'\t\t\t\t)')
    new_block_lines.append(f'\t\t\t)')
    new_block_lines.append(f'\t\t)')
    new_block_lines.append(f'\t\t(property "Footprint" "Package_BGA:Xilinx_FBG484"')
    new_block_lines.append(f'\t\t\t(at 0 14 0)')
    new_block_lines.append(f'\t\t\t(unlocked yes)')
    new_block_lines.append(f'\t\t\t(layer "F.Fab")')
    new_block_lines.append(f'\t\t\t(uuid "{gen_uuid()}")')
    new_block_lines.append(f'\t\t\t(effects')
    new_block_lines.append(f'\t\t\t\t(font')
    new_block_lines.append(f'\t\t\t\t\t(size 1 1)')
    new_block_lines.append(f'\t\t\t\t\t(thickness 0.15)')
    new_block_lines.append(f'\t\t\t\t)')
    new_block_lines.append(f'\t\t\t\t(hide yes)')
    new_block_lines.append(f'\t\t\t)')
    new_block_lines.append(f'\t\t)')

    # Extract and copy non-pad, non-property content from library footprint
    # (courtyard, silkscreen, fab outlines, solder_mask_margin, etc.)
    # We need the graphic elements but NOT the header/properties (we built those above)
    
    # Extract graphic elements from library footprint
    # Skip: (footprint header, (version, (generator, (layer, (descr, (tags, 
    #        (solder_mask_margin - we'll keep it), (property lines)
    # Keep: (fp_line, (fp_rect, (fp_circle, (fp_arc, (fp_text for %R,
    #        (solder_mask_margin, (pad blocks
    
    # Simpler approach: extract everything between properties and pads from the lib footprint,
    # then add modified pads with net assignments
    
    # Extract solder_mask_margin from lib footprint
    sm_match = re.search(r'\(solder_mask_margin\s+([\d.]+)\)', fp_content)
    if sm_match:
        new_block_lines.append(f'\t\t(solder_mask_margin {sm_match.group(1)})')
    
    # Extract all graphic elements (fp_line, fp_rect, fp_circle, fp_arc, fp_poly, fp_text)
    # from the library footprint
    graphic_types = ['fp_line', 'fp_rect', 'fp_circle', 'fp_arc', 'fp_poly', 'fp_text']
    
    idx = 0
    while idx < len(fp_content):
        found = False
        for gtype in graphic_types:
            tag = f'({gtype}'
            if fp_content[idx:idx+len(tag)] == tag:
                # Find the end of this block
                d = 0
                for j in range(idx, len(fp_content)):
                    if fp_content[j] == '(':
                        d += 1
                    elif fp_content[j] == ')':
                        d -= 1
                        if d == 0:
                            graphic_block = fp_content[idx:j+1]
                            # Add tab indentation
                            indented = '\t\t' + graphic_block.replace('\n', '\n\t\t')
                            new_block_lines.append(indented)
                            idx = j + 1
                            found = True
                            break
                break
        if not found:
            idx += 1
    
    # Extract pads from library footprint and add net assignments
    print("  Adding pads with net assignments...")
    pad_count = 0
    net_count = 0
    no_net_count = 0
    
    idx = 0
    while idx < len(fp_content):
        if fp_content[idx:idx+4] == '(pad':
            # Find end of pad block
            d = 0
            for j in range(idx, len(fp_content)):
                if fp_content[j] == '(':
                    d += 1
                elif fp_content[j] == ')':
                    d -= 1
                    if d == 0:
                        pad_text = fp_content[idx:j+1]
                        break
            
            # Extract pad name
            name_match = re.match(r'\(pad\s+"([^"]+)"', pad_text)
            pad_name = name_match.group(1) if name_match else ""
            
            # Look up net
            net_name = BALL_NET_MAP.get(pad_name, "")
            
            # Add net assignment and UUID to pad
            # Replace the closing ')' with net + uuid + ')'
            pad_inner = pad_text[:-1].rstrip()  # remove last )
            
            if net_name:
                pad_inner += f'\n\t\t\t(net "{net_name}")'
                net_count += 1
            else:
                no_net_count += 1
            
            pad_inner += f'\n\t\t\t(uuid "{gen_uuid()}")'
            pad_inner += '\n\t\t)'
            
            # Add indentation
            indented = '\t\t' + pad_inner.replace('\n', '\n\t\t')
            new_block_lines.append(indented)
            
            pad_count += 1
            idx = j + 1
        else:
            idx += 1
    
    # Add embedded_fonts and close
    new_block_lines.append(f'\t\t(embedded_fonts no)')
    new_block_lines.append(f'\t)')
    
    new_block = '\n'.join(new_block_lines)
    
    print(f"  Pads: {pad_count} total, {net_count} with nets, {no_net_count} unconnected")

    # --------------------------------------------------------
    # PHASE 3: Replace old footprint with new
    # --------------------------------------------------------
    print("\n--- Phase 3: Replacing footprint in PCB ---")
    
    new_pcb = pcb_content[:pos] + new_block + pcb_content[end:]
    
    print(f"  Old block: {len(old_block)} chars")
    print(f"  New block: {len(new_block)} chars")
    print(f"  PCB size: {len(pcb_content)} -> {len(new_pcb)} chars")

    # --------------------------------------------------------
    # PHASE 4: Write output
    # --------------------------------------------------------
    print(f"\nWriting {PCB_FILE}...")
    with open(PCB_FILE, 'w') as f:
        f.write(new_pcb)
    
    final_lines = new_pcb.count('\n')
    print(f"Output: {final_lines} lines")

    # --------------------------------------------------------
    # SUMMARY
    # --------------------------------------------------------
    print("\n" + "=" * 60)
    print("PCB FOOTPRINT SWAP COMPLETE")
    print("=" * 60)
    print(f"\n  Old footprint: BGA256C100P16X16_1700X1700X155 (256 pads)")
    print(f"  New footprint: Package_BGA:Xilinx_FBG484 (484 pads)")
    print(f"  Position: ({U42_X}, {U42_Y}) - unchanged")
    print(f"  Pads with net: {net_count}")
    print(f"  Pads unconnected: {no_net_count}")
    
    # Net summary
    nets = list(BALL_NET_MAP.values())
    net_counts = {}
    for n in nets:
        net_counts[n] = net_counts.get(n, 0) + 1
    
    print(f"\n  Net assignment summary:")
    for n in sorted(net_counts.keys()):
        print(f"    {n:30s}: {net_counts[n]} pads")
    
    print(f"\n{'='*60}")
    print("IMPORTANT: All old FPGA traces are now INVALID.")
    print("They connect to wrong pad positions.")
    print("Run DRC to identify unconnected/misrouted nets.")
    print("Re-routing is required for all FPGA connections.")
    print(f"{'='*60}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
