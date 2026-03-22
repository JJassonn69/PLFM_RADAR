#!/usr/bin/env python3
"""Fix PCB for JLCPCB standard process - conservative fixes only."""
import re
import os

PCB_FILE = "/Users/ganeshpanth/PLFM_RADAR/4_Schematics and Boards Layout/4_6_Schematics/FrequencySynthesizerBoard/Clocks_Freq_Synth_board.kicad_pcb"

with open(PCB_FILE, 'r') as f:
    content = f.read()

via_count = 0
via_fixed = 0
drill_fixed = 0

def fix_via(m):
    global via_count, via_fixed, drill_fixed
    via_count += 1
    block = m.group(0)
    
    size_m = re.search(r'\(size ([\d.]+)\)', block)
    drill_m = re.search(r'\(drill ([\d.]+)\)', block)
    
    if not size_m or not drill_m:
        return block
    
    s = float(size_m.group(1))
    d = float(drill_m.group(1))
    
    # Only fix if below JLCPCB standard minimums
    if s < 0.45:
        block = re.sub(r'\(size [\d.]+\)', '(size 0.45)', block)
        via_fixed += 1
    
    if d < 0.3:
        block = re.sub(r'\(drill [\d.]+\)', '(drill 0.3)', block)
        drill_fixed += 1
    
    return block

via_pattern = re.compile(r'\t\(via\n.*?\t\)', re.DOTALL)
content = via_pattern.sub(fix_via, content)

print(f"Conservative via fixes:")
print(f"  Total vias: {via_count}")
print(f"  Diameter fixed (< 0.45mm): {via_fixed}")
print(f"  Drill fixed (< 0.3mm): {drill_fixed}")

with open(PCB_FILE, 'w') as f:
    f.write(content)
print(f"  Written")
