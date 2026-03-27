#!/usr/bin/env python3
"""
fpga_trace_cleanup_pass2.py — Second pass: delete ALL remaining trace segments
and vias for old U42 signal nets that still have shorting violations.

These signals must be completely re-routed to the new BGA pad positions anyway,
so any remaining trace stubs are useless and harmful (causing DRC shorts).

We also delete all trace segments for the "unmigrated" nets (ADC_DCO_P/N,
ADC_D7_N, ADAR_TR_4, ADAR_TX_LOAD_1, FPGA_DAC_CLOCK, M3S_VCTRL, N$xxx)
since those signals are dropped from the new design.

Strategy: For each signal net listed below, delete every segment and via on the
ENTIRE board. These traces ran exclusively between U42 and their target
component — they don't serve any other purpose.

Power nets (GND, +3V3_FPGA, +1V0_FPGA, +1V8_FPGA) are NOT touched.
"""

import re
import os

PCB_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB_FILE = os.path.join(PCB_DIR, "RADAR_Main_Board.kicad_pcb")

# ALL signal nets that were on old U42 — these traces ONLY connect U42 to their
# target component. Deleting them board-wide is safe because they need re-routing.
SIGNAL_NETS_TO_DELETE = {
    # ADC signals (all of them, including unmigrated)
    "ADC_D0_N", "ADC_D0_P", "ADC_D1_N", "ADC_D1_P",
    "ADC_D2_N", "ADC_D2_P", "ADC_D3_N", "ADC_D3_P",
    "ADC_D4_N", "ADC_D4_P", "ADC_D5_N", "ADC_D5_P",
    "ADC_D6_N", "ADC_D6_P", "ADC_D7_P", "ADC_D7_N",
    "ADC_OR_N", "ADC_OR_P", "ADC_PWRD",
    "ADC_DCO_P", "ADC_DCO_N",
    "FPGA_ADC_CLOCK_N", "FPGA_ADC_CLOCK_P",
    # Flash signals
    "FPGA_FLASH_DQ0", "FPGA_FLASH_DQ1", "FPGA_FLASH_DQ2", "FPGA_FLASH_DQ3",
    "FPGA_FLASH_NCS", "FPGA_FLASH_NRST", "FPGA_FLASH_CLK",
    "FPGA_PUDC_B",
    # ADAR beamformer signals (all banks)
    "ADAR_1_CS_3V3", "ADAR_2_CS_3V3", "ADAR_3_CS_3V3", "ADAR_4_CS_3V3",
    "ADAR_1_CS_1V8", "ADAR_2_CS_1V8", "ADAR_3_CS_1V8", "ADAR_4_CS_1V8",
    "ADAR_RX_LOAD_1", "ADAR_RX_LOAD_2", "ADAR_RX_LOAD_3", "ADAR_RX_LOAD_4",
    "ADAR_TR_1", "ADAR_TR_2", "ADAR_TR_3", "ADAR_TR_4",
    "ADAR_TX_LOAD_1", "ADAR_TX_LOAD_2", "ADAR_TX_LOAD_3", "ADAR_TX_LOAD_4",
    # DAC signals
    "DAC_0", "DAC_1", "DAC_2", "DAC_3", "DAC_4", "DAC_5", "DAC_6", "DAC_7",
    "DAC_SLEEP",
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


def main():
    print(f"Reading PCB file: {PCB_FILE}")
    with open(PCB_FILE, 'r') as f:
        lines = f.readlines()
    
    total_lines = len(lines)
    print(f"Total lines: {total_lines}")
    
    lines_to_delete = set()
    
    stats = {
        'segments_deleted': 0,
        'vias_deleted': 0,
    }
    deleted_nets = {}
    
    i = 0
    while i < total_lines:
        stripped = lines[i].strip()
        
        if stripped == '(segment' or stripped == '(via':
            elem_type = 'segment' if stripped == '(segment' else 'via'
            # Find end of block
            props = {}
            j = i + 1
            while j < total_lines:
                l = lines[j].strip()
                if l == ')':
                    break
                m = re.match(r'\((\w+)\s+(.+)\)', l)
                if m:
                    props[m.group(1)] = m.group(2)
                j += 1
            end_idx = j
            
            net = props.get('net', '').strip('"')
            
            if net in SIGNAL_NETS_TO_DELETE:
                if elem_type == 'segment':
                    stats['segments_deleted'] += 1
                else:
                    stats['vias_deleted'] += 1
                deleted_nets[net] = deleted_nets.get(net, 0) + 1
                for k in range(i, end_idx + 1):
                    lines_to_delete.add(k)
            
            i = end_idx + 1
            continue
        
        i += 1
    
    print(f"\n=== PASS 2 CLEANUP SUMMARY ===")
    print(f"Segments deleted: {stats['segments_deleted']}")
    print(f"Vias deleted:     {stats['vias_deleted']}")
    print(f"Total lines to remove: {len(lines_to_delete)}")
    
    if deleted_nets:
        print(f"\nDeleted by net:")
        for net, count in sorted(deleted_nets.items(), key=lambda x: -x[1]):
            print(f"  {net:30s}: {count}")
    
    if not lines_to_delete:
        print("\nNothing more to delete!")
        return
    
    new_lines = [line for i, line in enumerate(lines) if i not in lines_to_delete]
    
    print(f"\nLines before: {total_lines}")
    print(f"Lines after:  {len(new_lines)}")
    print(f"Lines removed: {total_lines - len(new_lines)}")
    
    with open(PCB_FILE, 'w') as f:
        f.writelines(new_lines)
    
    print(f"\nPCB file updated successfully!")


if __name__ == '__main__':
    main()
