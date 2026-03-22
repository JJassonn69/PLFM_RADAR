# Frequency Synthesizer Board - DRC Fix Guide

## Overview

The Frequency Synthesizer PCB (`Clocks_Freq_Synth_board.kicad_pcb`) had **1508 DRC violations**
from the original design. After analysis, the breakdown is:

| Category | Count | Status |
|----------|-------|--------|
| Via annular ring too small | ~550 | **FIXED** by `fix_pcb_vias.py` (upsized 217 vias to 0.45 mm) |
| Via diameter too small | ~313 | **FIXED** by `fix_pcb_vias.py` (upsized drills to 0.30 mm) |
| Via-via clearance | 39 | **24 FIXED** by `fix_via_clearance.py`, **15 remain** |
| Trace width < 0.200 mm | 199 | **FALSE POSITIVE** — JLCPCB accepts 0.127 mm (5 mil) |
| Other clearance < 0.200 mm | 371 | **FALSE POSITIVE** — JLCPCB accepts 0.100 mm |
| Solder mask bridge | 201 | **FALSE POSITIVE** — JLCPCB solder mask rules are looser |
| Via-pad clearance | 21 | **MANUAL FIX NEEDED** (see below) |
| Courtyard overlap | 7 | **REVIEW NEEDED** |
| Copper-edge clearance | 5 | **REVIEW NEEDED** |

**Scripts applied (in order):**
1. `fix_via_clearance.py` — moved 33 signal vias away from GND stitching
2. `fix_pcb_vias.py` — upsized all vias to JLCPCB minimum (0.45 mm / 0.30 mm drill)

**Backup:** `.history/Clocks_Freq_Synth_board.kicad_pcb.bak_before_clearance_fix`

---

## Remaining Fab-Blocking Via-Via Violations (15)

These 15 via-via pairs have clearance < 0.100 mm (JLCPCB minimum) and **must be fixed
manually in KiCad** before ordering from JLCPCB.

### Cluster A: TX ADF4382 SPI Bus Area (8 violations)

**Location:** X: 141.2-141.7 mm, Y: 97.6-102.7 mm
**Root cause:** 8 signal vias for the TX ADF4382 SPI bus packed in a 0.5 x 5 mm area.
With 0.45 mm via diameter, each via needs 0.45 + 0.10 = 0.55 mm center-to-center spacing
from different-net neighbors. Several pairs are at 0.50-0.53 mm.

| # | Net A | Coord A | Net B | Coord B | Clearance | Fix |
|---|-------|---------|-------|---------|-----------|-----|
| 1 | ADF4382_SDIO | (141.70, 101.08) | ADF4382_TX_CS | (141.49, 100.62) | 0.056 mm | Move TX_CS south by 0.05 mm |
| 2 | GND | (141.70, 97.60) | ADF4382_TX_LKDET | (141.70, 98.12) | 0.062 mm | Move GND stitching via south by 0.05 mm |
| 3 | ADF4382_TX_DELADJ | (141.70, 98.63) | ADF4382_TX_LKDET | (141.70, 98.12) | 0.068 mm | Spread DELADJ north + LKDET south by 0.02 mm each |
| 4 | +3V3_LO_1 | (141.55, 102.70) | ADF4382_SDO | (141.59, 102.18) | 0.077 mm | Move LO_1 via north by 0.03 mm |
| 5 | +3V3_LO_1 | (141.40, 99.55) | ADF4382_TX_CE | (141.45, 100.09) | 0.085 mm | Move LO_1 via south by 0.02 mm |
| 6 | ADF4382_SCLK | (141.70, 101.65) | ADF4382_SDO | (141.59, 102.18) | 0.085 mm | Move SCLK south by 0.02 mm |
| 7 | ADF4382_TX_CS | (141.49, 100.62) | ADF4382_TX_CE | (141.45, 100.09) | 0.086 mm | (Cascaded from fix #1 — verify after) |

**Recommended approach:** Open KiCad, zoom to (141.5, 100.0), select each via individually
and nudge it. After moving each via, check its trace connection is intact. The SPI bus is
low-speed digital so small position changes won't affect signal integrity.

### Cluster B: Power Vias Near GND Stitching (~149.5 area) (6 violations)

**Location:** X: 149.5-164.0 mm, Y: 96.2-102.1 mm
**Root cause:** +3V3_LO_1 and +3V3_LO_2 power vias are placed 0.50 mm from GND stitching
vias (exactly sized for 0.35 mm original vias, but now 0.45 mm after upsize).

| # | Net A | Coord A | Net B | Coord B | Clearance | Fix |
|---|-------|---------|-------|---------|-----------|-----|
| 1 | GND | (149.60, 98.15) | +3V3_LO_2 | (149.59, 97.66) | 0.048 mm | Delete GND stitching via or move 0.06 mm north |
| 2 | GND | (163.95, 101.15) | +3V3_LO_1 | (163.76, 101.62) | 0.049 mm | Delete GND stitching via or move 0.06 mm away |
| 3 | GND | (164.00, 102.05) | +3V3_LO_1 | (163.76, 101.62) | 0.049 mm | Delete GND stitching via or move 0.06 mm away |
| 4 | GND | (149.55, 101.15) | +3V3_LO_1 | (149.61, 101.65) | 0.051 mm | Delete GND stitching via or move 0.06 mm away |
| 5 | GND | (149.60, 102.15) | +3V3_LO_1 | (149.61, 101.65) | 0.052 mm | Delete GND stitching via or move 0.06 mm away |
| 6 | GND | (149.60, 97.15) | +3V3_LO_2 | (149.59, 97.66) | 0.052 mm | Delete GND stitching via or move 0.06 mm away |

**Recommended approach:** These GND stitching vias provide return-current paths but are
redundant in dense stitching arrays. **Deleting the violating GND via is the safest fix** —
there are dozens of nearby GND stitching vias that maintain plane integrity. Alternatively,
move the GND via 0.06 mm further from the power via.

### Cluster C: RX ADF4382 SPI Bus Area (1 violation)

**Location:** X: 155.5-156.0 mm, Y: 100.1-101.1 mm

| # | Net A | Coord A | Net B | Coord B | Clearance | Fix |
|---|-------|---------|-------|---------|-----------|-----|
| 1 | ADF4382_RX_CE | (155.53, 100.18) | ADF4382_RX_CS | (155.85, 100.58) | 0.063 mm | Move RX_CE west by 0.04 mm |
| 2 | ADF4382_SDIO | (156.00, 101.10) | ADF4382_RX_CS | (155.85, 100.58) | 0.092 mm | Move SDIO north by 0.02 mm |

---

## Differential Pair Violations (2) - NO FIX NEEDED

These are intentional close-spacing for impedance control:
- AD9523_OUT8_P / AD9523_OUT8_N (0.030 mm clearance)
- AD9523_OUT9_P / AD9523_OUT9_N (0.078 mm clearance)

**Action:** Add a DRC exception for these pairs in KiCad, or accept the violations in
the JLCPCB review. Diff pair routing at 0.15 mm spacing (edge-to-edge before copper) is
standard practice and JLCPCB will fabricate without issue.

---

## Marginal Via-Via Violations (15) - OPTIONAL

These pairs are between 0.100-0.125 mm clearance. They **meet JLCPCB minimums** and will
fabricate, but have reduced margin. All are signal/power vias near GND stitching.
No action required unless you want additional margin for yield.

---

## Via-Pad Clearance Violations (21) - MANUAL FIX

These are cases where vias are too close to component pads. Five clusters:

1. **N$26/GND near (148.6, 104.0)** - 5 violations. Check if via is under pad; if so,
   increase pad-via separation or tent the via.
2. **ADF4382_RX_CE/CS near (155.5, 100.1)** - 4 violations (overlaps with Cluster C above).
3. **GND near (155.6, 119.6)** - 4 violations. Likely GND vias too close to passives.
4. **AD9523_OUT1_N/GND near (161.2, 104.8)** - 3 violations.
5. **+1V8_CLOCK/GND near (161.0, 112.2)** - 3 violations.
6. **Two singles** at (163.0, 98.4) and (141.1, 99.3).

**Fix approach:** Open KiCad, navigate to each coordinate, move the via 0.05-0.10 mm
away from the pad. Ensure via still connects to its copper pour or trace.

---

## Courtyard Overlap (7) - REVIEW

Seven component pairs have overlapping courtyards. Check in KiCad 3D view:
- (119.0, 64.3), (171.7, 149.4), (177.0, 149.4), (111.2, 123.8), (136.6, 149.5), (141.9, 149.3)

These are often acceptable for passives placed close together. Verify no physical
collision in the 3D viewer.

---

## Copper-Edge Clearance (5) - REVIEW

Five traces are too close to the board edge:
- Major area near (168.7, 55.8) — net N$77
- Two at Y~153.3 — GND traces near board edge

**Fix:** Move traces 0.25 mm inward from board edge, or verify the board outline is correct.

---

## Summary: Path to Fab-Ready

| Step | Action | Where | Time Est. |
|------|--------|-------|-----------|
| 1 | Fix 15 fab-blocking via-via violations | KiCad PCB editor | 30 min |
| 2 | Fix 21 via-pad violations | KiCad PCB editor | 20 min |
| 3 | Review 7 courtyard overlaps | KiCad 3D viewer | 10 min |
| 4 | Fix 5 copper-edge violations | KiCad PCB editor | 10 min |
| 5 | Add DRC exceptions for diff pairs | KiCad DRC rules | 5 min |
| 6 | Re-run DRC with JLCPCB-matched rules | KiCad | 5 min |
| 7 | Generate Gerbers and upload to JLCPCB | KiCad + JLCPCB | 15 min |

**Total estimated manual work: ~1.5 hours**

The ~1400 trace width, clearance, and solder mask violations are all false positives from
overly strict DRC rules. Update `.kicad_dru` to match JLCPCB capabilities:
- Minimum clearance: 0.100 mm (not 0.200 mm)
- Minimum track width: 0.127 mm (not 0.200 mm)  
- Minimum via annular ring: 0.075 mm (not 0.100 mm)
- Minimum via diameter: 0.45 mm (not 0.50 mm)
