# Frequency Synthesizer Board - DRC Fix Guide

## Overview

The Frequency Synthesizer PCB (`Clocks_Freq_Synth_board.kicad_pcb`) originally reported
**1508 DRC violations** using KiCad defaults. After creating a `.kicad_pro` project file
with JLCPCB-matched design rules and a `.kicad_dru` custom rules file, the count is now:

| Metric | Count |
|--------|-------|
| **Total violations** | 661 |
| **Errors (fab-relevant)** | 9 |
| **Warnings (cosmetic)** | 652 |

The 9 remaining errors are real issues that need manual KiCad GUI fixes.

---

## DRC Reduction History

| Run | Violations | Errors | What Changed |
|-----|-----------|--------|--------------|
| Original (no project file) | 1508 | ~1400+ | KiCad defaults (0.2mm clearance, 0.5mm via) |
| After `.kicad_pro` v1 | 1210 | ~200 | JLCPCB rules, but min_via_diameter=0.45mm too strict |
| After `.kicad_pro` v2 | 683 | 25 | Relaxed via/drill/hole rules for 0.35mm vias |
| After `.kicad_dru` v2 (current) | **661** | **9** | Via-to-via clearance 0.254 -> 0.20mm |

---

## Scripts Applied (in order)

1. `fix_via_clearance.py` - moved 33 signal vias away from GND stitching (force-vector algorithm)
2. `fix_pcb_vias.py` - upsized 217 vias to 0.45mm diameter, 313 drills to 0.30mm

**Backup:** `.history/Clocks_Freq_Synth_board.kicad_pcb.bak_before_clearance_fix`

---

## Project File: `.kicad_pro`

Created with JLCPCB 6-layer design rules:

| Rule | Value | Notes |
|------|-------|-------|
| `min_clearance` | 0.10 mm | JLCPCB 6-layer minimum |
| `min_track_width` | 0.10 mm | JLCPCB accepts 0.09mm on inner |
| `min_via_diameter` | 0.30 mm | Board has 0.35mm and 0.45mm vias |
| `min_via_annular_width` | 0.075 mm | JLCPCB minimum for standard via |
| `min_through_hole_diameter` | 0.15 mm | Board has 0.15mm drill vias |
| `min_hole_clearance` | 0.15 mm | JLCPCB minimum |
| `min_hole_to_hole` | 0.15 mm | JLCPCB minimum |
| `solder_mask_bridge` | warning severity | Cosmetic, not fab-blocking |

---

## Custom Rules File: `.kicad_dru`

Four rules to handle specific board patterns:

1. **Differential pair clearance** (0.10mm) - AD9523 diff pairs are intentionally close-spaced
2. **Via-to-via hole clearance** (0.20mm) - matches JLCPCB minimum for 0.15mm drill vias
3. **Track-to-via clearance** (0.10mm) - dense routing areas
4. **SMD pad-to-pad clearance** (0.10mm) - fine-pitch components

---

## Remaining 9 Errors (All Need Manual KiCad GUI Fix)

### 7 Courtyard Overlaps

Component pairs with overlapping courtyards. These are physically close passive
components (capacitors and inductors) that may or may not collide.

| # | Location (mm) | Components | Fix |
|---|---------------|------------|-----|
| 1 | (108.5, 124.4) | C21 / L9 | Check 3D viewer for collision |
| 2 | (117.9, 69.3) | C45 / L12 | Check 3D viewer for collision |
| 3 | (120.7, 64.0) | L12 / C33 | Check 3D viewer for collision |
| 4 | (139.3, 148.8) | C27 / L10 | Check 3D viewer for collision |
| 5 | (144.4, 149.7) | L10 / C25 | Check 3D viewer for collision |
| 6 | (169.0, 150.0) | C31 / L11 | Check 3D viewer for collision |
| 7 | (179.7, 150.1) | C29 / L11 | Check 3D viewer for collision |

**Recommended approach:** Open in KiCad 3D viewer. If passives don't physically collide
(they're on opposite sides, or the courtyard is conservative), these are safe to accept.
If they do collide, move one component 0.2-0.5mm.

### 2 Copper Edge Clearance

Both at the same location — GND zone copper on In1.Cu and In4.Cu layers too close to
the board edge.

| # | Location (mm) | Actual Clearance | Required | Fix |
|---|---------------|-----------------|----------|-----|
| 1 | (98.5, 55.0) In1.Cu | 0.026 mm | 0.200 mm | Pull back zone fill or adjust board outline |
| 2 | (98.5, 55.0) In4.Cu | 0.026 mm | 0.200 mm | Pull back zone fill or adjust board outline |

**Recommended approach:** Open in KiCad, select the GND zone on In1.Cu and In4.Cu,
increase the zone's "Board Edge Clearance" to 0.25mm (or set it to use the board default).
Then re-pour zones (Edit > Fill All Zones, or press 'B').

---

## 652 Warnings Breakdown (All Cosmetic)

| Type | Count | Impact |
|------|-------|--------|
| Solder mask bridge | 203 | Mask slivers between pads; JLCPCB handles these |
| Silk overlap | 199 | Silkscreen text overlaps; cosmetic only |
| Silk over copper | 199 | Silkscreen on exposed copper; cosmetic only |
| Track dangling | 35 | Dangling track ends; clean up optionally |
| Text thickness | 10 | Silkscreen text too thin; cosmetic |
| Text height | 6 | Silkscreen text too small; cosmetic |

**None of these block fabrication.** JLCPCB will silently fix silk issues and mask slivers.
Dangling tracks can be cleaned up optionally but don't affect connectivity.

---

## Via Population Reference

| Type | Count | Diameter | Drill | Nets |
|------|-------|----------|-------|------|
| Small | 199 | 0.35 mm | 0.15 mm | GND, power rails, AD9523/ADF4382 signals |
| Standard | ~530 | 0.45 mm | 0.30 mm | All others (after upsize script) |

Both are within JLCPCB 6-layer capabilities (min via: 0.15mm drill, 0.25mm pad).

---

## Path to Fab-Ready

| Step | Action | Where | Time Est. |
|------|--------|-------|-----------|
| 1 | Review 7 courtyard overlaps in 3D viewer | KiCad GUI | 10 min |
| 2 | Fix 2 copper-edge clearance (re-pour zones) | KiCad GUI | 5 min |
| 3 | Clean up 35 dangling tracks (optional) | KiCad GUI | 15 min |
| 4 | Re-run DRC to confirm 0 errors | KiCad | 2 min |
| 5 | Generate Gerbers and upload to JLCPCB | KiCad + JLCPCB | 15 min |

**Total estimated manual work: ~30 minutes** (down from ~1.5 hours before `.kicad_pro`)

The board is very close to fab-ready. The 9 errors are minor and the 652 warnings
are all cosmetic items that JLCPCB will fabricate without issue.
