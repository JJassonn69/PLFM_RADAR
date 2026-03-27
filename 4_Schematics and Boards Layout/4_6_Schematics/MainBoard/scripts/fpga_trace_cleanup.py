#!/usr/bin/env python3
"""
fpga_trace_cleanup.py — Delete old FPGA (U42) traces and vias from BGA area.

Strategy:
- The old BGA256 was at center (68.1811, 71.7236) with ±7.5mm pad span.
- The new BGA484 is at the same center with ±10.5mm pad span.
- We need to clear the ENTIRE new BGA area of old traces.
- For SIGNAL nets: delete any segment/via with at least one endpoint in the area.
- For POWER nets (GND, +1V0_FPGA, +1V8_FPGA, +3V3_FPGA): delete only segments
  where BOTH endpoints are in the area, and vias in the area. This preserves
  power routing that passes through or extends beyond the BGA zone.

BGA area with 2mm margin: center ± (10.5 + 2) = ±12.5mm
  X: 55.68 to 80.68
  Y: 59.22 to 84.22
"""

import re
import sys
import os

PCB_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB_FILE = os.path.join(PCB_DIR, "RADAR_Main_Board.kicad_pcb")

# BGA center
CX, CY = 68.1811, 71.7236

# New BGA half-span + margin
MARGIN = 2.0
HALF_SPAN = 10.5 + MARGIN  # 12.5mm

X_MIN = CX - HALF_SPAN  # 55.6811
X_MAX = CX + HALF_SPAN  # 80.6811
Y_MIN = CY - HALF_SPAN  # 59.2236
Y_MAX = CY + HALF_SPAN  # 84.2236

# Power nets — need BOTH endpoints in area to delete
POWER_NETS = {
    "GND", "+1V0_FPGA", "+1V8_FPGA", "+3V3_FPGA",
    # Also protect other board-wide power nets that might pass through
    "+3V3", "+5V", "+1V0", "+1V8", "+12V",
}

# All 95 nets that have traces/vias in the BGA area (from prior analysis).
# Signal nets are everything that was connected to old U42 EXCEPT the power nets.
# We'll identify nets dynamically — any net on a segment/via in the area gets
# evaluated, and we apply the power/signal rule.

# Complete list of signal nets that were on old U42 (from migration map + unmigrated)
SIGNAL_NETS = {
    # ADC signals
    "ADC_D0_N", "ADC_D0_P", "ADC_D1_N", "ADC_D1_P",
    "ADC_D2_N", "ADC_D2_P", "ADC_D3_N", "ADC_D3_P",
    "ADC_D4_N", "ADC_D4_P", "ADC_D5_N", "ADC_D5_P",
    "ADC_D6_N", "ADC_D6_P", "ADC_D7_P",
    "ADC_OR_N", "ADC_OR_P", "ADC_PWRD",
    "FPGA_ADC_CLOCK_N", "FPGA_ADC_CLOCK_P",
    # Unmigrated ADC signals
    "ADC_D7_N", "ADC_DCO_P", "ADC_DCO_N",
    # Flash signals
    "FPGA_FLASH_DQ0", "FPGA_FLASH_DQ1", "FPGA_FLASH_DQ2", "FPGA_FLASH_DQ3",
    "FPGA_FLASH_NCS", "FPGA_FLASH_NRST", "FPGA_FLASH_CLK",
    "FPGA_PUDC_B",
    # ADAR beamformer signals (bank 15)
    "ADAR_1_CS_3V3", "ADAR_2_CS_3V3", "ADAR_3_CS_3V3", "ADAR_4_CS_3V3",
    # ADAR beamformer signals (bank 34)
    "ADAR_1_CS_1V8", "ADAR_2_CS_1V8", "ADAR_3_CS_1V8", "ADAR_4_CS_1V8",
    "ADAR_RX_LOAD_1", "ADAR_RX_LOAD_2", "ADAR_RX_LOAD_3", "ADAR_RX_LOAD_4",
    "ADAR_TR_1", "ADAR_TR_2", "ADAR_TR_3",
    "ADAR_TX_LOAD_2", "ADAR_TX_LOAD_3", "ADAR_TX_LOAD_4",
    # Unmigrated beamformer
    "ADAR_TR_4", "ADAR_TX_LOAD_1",
    # DAC signals
    "DAC_0", "DAC_1", "DAC_2", "DAC_3", "DAC_4", "DAC_5", "DAC_6", "DAC_7",
    "DAC_SLEEP",
    # Unmigrated DAC
    "FPGA_DAC_CLOCK",
    # Digital / STM32
    "DIG_0", "DIG_1", "DIG_2", "DIG_3", "DIG_4", "DIG_5", "DIG_6", "DIG_7",
    "STM32_MISO1", "STM32_MOSI1", "STM32_SCLK1",
    "STM32_MISO_1V8", "STM32_MOSI_1V8", "STM32_SCLK_1V8",
    # Clock / control
    "FPGA_CLOCK_TEST", "FPGA_SYS_CLOCK",
    "MIX_RX_EN", "MIX_TX_EN",
    # JTAG
    "FPGA_TCK", "FPGA_TDI", "FPGA_TDO", "FPGA_TMS",
    # Unmigrated misc
    "M3S_VCTRL",
    # Eagle unnamed nets
    "N$106", "N$107", "N$108", "N$109", "N$114", "N$115", "N$116",
}


def in_area(x, y):
    """Check if a point is within the BGA cleanup area."""
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX


def parse_segment(lines, start_idx):
    """Parse a segment block starting at start_idx. Returns (end_idx, props)."""
    # lines[start_idx] should be '\t(segment\n'
    props = {}
    i = start_idx + 1
    while i < len(lines):
        line = lines[i].strip()
        if line == ')':
            return i, props
        # Parse (key value...) 
        m = re.match(r'\((\w+)\s+(.+)\)', line)
        if m:
            key, val = m.group(1), m.group(2)
            props[key] = val
        i += 1
    return i, props


def parse_via(lines, start_idx):
    """Parse a via block starting at start_idx. Returns (end_idx, props)."""
    props = {}
    i = start_idx + 1
    while i < len(lines):
        line = lines[i].strip()
        if line == ')':
            return i, props
        m = re.match(r'\((\w+)\s+(.+)\)', line)
        if m:
            key, val = m.group(1), m.group(2)
            props[key] = val
        i += 1
    return i, props


def parse_xy(val):
    """Parse 'X Y' into (float, float)."""
    parts = val.split()
    return float(parts[0]), float(parts[1])


def main():
    print(f"Reading PCB file: {PCB_FILE}")
    with open(PCB_FILE, 'r') as f:
        lines = f.readlines()
    
    total_lines = len(lines)
    print(f"Total lines: {total_lines}")
    
    # Find all segments and vias, decide which to delete
    lines_to_delete = set()  # set of line indices to remove
    
    stats = {
        'segments_total': 0,
        'segments_in_area': 0,
        'segments_deleted': 0,
        'segments_kept_power': 0,
        'vias_total': 0,
        'vias_in_area': 0,
        'vias_deleted': 0,
        'vias_kept_power': 0,
    }
    
    deleted_nets = {}  # net -> count of deleted elements
    kept_nets = {}     # net -> count of kept elements (power nets with external reach)
    
    i = 0
    while i < total_lines:
        line = lines[i]
        stripped = line.strip()
        
        if stripped == '(segment':
            stats['segments_total'] += 1
            end_idx, props = parse_segment(lines, i)
            
            # Extract coordinates
            if 'start' in props and 'end' in props:
                sx, sy = parse_xy(props['start'])
                ex, ey = parse_xy(props['end'])
                net = props.get('net', '').strip('"')
                
                start_in = in_area(sx, sy)
                end_in = in_area(ex, ey)
                
                if start_in or end_in:
                    stats['segments_in_area'] += 1
                    
                    should_delete = False
                    
                    if net in SIGNAL_NETS:
                        # Signal net: delete if ANY endpoint in area
                        should_delete = True
                    elif net in POWER_NETS:
                        # Power net: delete only if BOTH endpoints in area
                        if start_in and end_in:
                            should_delete = True
                        else:
                            stats['segments_kept_power'] += 1
                            kept_nets[net] = kept_nets.get(net, 0) + 1
                    else:
                        # Unknown net in area — check if it has any endpoint in area
                        # Be conservative: only delete if BOTH endpoints in area
                        # (treat unknown nets like power nets to be safe)
                        if start_in and end_in:
                            should_delete = True
                        else:
                            stats['segments_kept_power'] += 1
                            kept_nets[net] = kept_nets.get(net, 0) + 1
                    
                    if should_delete:
                        stats['segments_deleted'] += 1
                        deleted_nets[net] = deleted_nets.get(net, 0) + 1
                        for j in range(i, end_idx + 1):
                            lines_to_delete.add(j)
            
            i = end_idx + 1
            continue
            
        elif stripped == '(via':
            stats['vias_total'] += 1
            end_idx, props = parse_via(lines, i)
            
            if 'at' in props:
                vx, vy = parse_xy(props['at'])
                net = props.get('net', '').strip('"')
                
                if in_area(vx, vy):
                    stats['vias_in_area'] += 1
                    
                    should_delete = False
                    
                    if net in SIGNAL_NETS:
                        should_delete = True
                    elif net in POWER_NETS:
                        # For power vias inside BGA area: delete them
                        # These are the old BGA fanout vias — they connect to
                        # old pad positions that no longer exist
                        should_delete = True
                    else:
                        # Unknown net via in area — delete if in the tight BGA zone
                        # (within old BGA span + small margin)
                        OLD_HALF = 7.5 + 1.0  # old BGA span + 1mm
                        if (CX - OLD_HALF <= vx <= CX + OLD_HALF and
                            CY - OLD_HALF <= vy <= CY + OLD_HALF):
                            should_delete = True
                    
                    if should_delete:
                        stats['vias_deleted'] += 1
                        deleted_nets[net] = deleted_nets.get(net, 0) + 1
                        for j in range(i, end_idx + 1):
                            lines_to_delete.add(j)
                    else:
                        stats['vias_kept_power'] += 1
                        kept_nets[net] = kept_nets.get(net, 0) + 1
            
            i = end_idx + 1
            continue
        
        i += 1
    
    # Print summary
    print(f"\n=== TRACE CLEANUP SUMMARY ===")
    print(f"BGA area: X [{X_MIN:.2f}, {X_MAX:.2f}], Y [{Y_MIN:.2f}, {Y_MAX:.2f}]")
    print(f"")
    print(f"Segments total:      {stats['segments_total']}")
    print(f"Segments in area:    {stats['segments_in_area']}")
    print(f"Segments to delete:  {stats['segments_deleted']}")
    print(f"Segments kept (power/unknown reaching outside): {stats['segments_kept_power']}")
    print(f"")
    print(f"Vias total:          {stats['vias_total']}")
    print(f"Vias in area:        {stats['vias_in_area']}")
    print(f"Vias to delete:      {stats['vias_deleted']}")
    print(f"Vias kept (power):   {stats['vias_kept_power']}")
    print(f"")
    print(f"Total lines to remove: {len(lines_to_delete)}")
    print(f"")
    
    print(f"Deleted by net:")
    for net, count in sorted(deleted_nets.items(), key=lambda x: -x[1]):
        ntype = "SIGNAL" if net in SIGNAL_NETS else "POWER" if net in POWER_NETS else "OTHER"
        print(f"  {net:30s} [{ntype:6s}]: {count}")
    
    if kept_nets:
        print(f"\nKept (in area but reaching outside):")
        for net, count in sorted(kept_nets.items(), key=lambda x: -x[1]):
            print(f"  {net:30s}: {count}")
    
    # Write output
    if not lines_to_delete:
        print("\nNothing to delete!")
        return
    
    # Build new file content
    new_lines = []
    for i, line in enumerate(lines):
        if i not in lines_to_delete:
            new_lines.append(line)
    
    removed_lines = total_lines - len(new_lines)
    print(f"\nLines before: {total_lines}")
    print(f"Lines after:  {len(new_lines)}")
    print(f"Lines removed: {removed_lines}")
    
    # Write back
    with open(PCB_FILE, 'w') as f:
        f.writelines(new_lines)
    
    print(f"\nPCB file updated successfully!")


if __name__ == '__main__':
    main()
