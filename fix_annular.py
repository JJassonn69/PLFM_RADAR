#!/usr/bin/env python3
"""Fix via annular rings in KiCad PCB for JLCPCB compliance."""
import re
import os

PCB_FILE = "/Users/ganeshpanth/PLFM_RADAR/4_Schematics and Boards Layout/4_6_Schematics/FrequencySynthesizerBoard/Clocks_Freq_Synth_board.kicad_pcb"

# JLCPCB requires 0.15mm annular ring per side
# If drill = 0.3mm, min via = 0.3 + 2*0.15 = 0.6mm
# If drill = 0.3mm, via at 0.5mm gives only 0.1mm annular → violation
MIN_ANNULAR = 0.15

with open(PCB_FILE, 'r') as f:
    content = f.read()

fixed = 0

def fix_via_annular(m):
    global fixed
    block = m.group(0)
    
    size_m = re.search(r'\(size ([\d.]+)\)', block)
    drill_m = re.search(r'\(drill ([\d.]+)\)', block)
    
    if not size_m or not drill_m:
        return block
    
    s = float(size_m.group(1))
    d = float(drill_m.group(1))
    
    annular = (s - d) / 2
    
    if annular < MIN_ANNULAR:
        new_size = round(d + 2 * MIN_ANNULAR, 2)
        block = re.sub(r'\(size [\d.]+\)', f'(size {new_size})', block)
        fixed += 1
    
    return block

via_pattern = re.compile(r'\t\(via\n.*?\t\)', re.DOTALL)
content = via_pattern.sub(fix_via_annular, content)

print(f"Annular ring fixes: {fixed} vias resized")

with open(PCB_FILE, 'w') as f:
    f.write(content)

print(f"Fixed PCB written")
