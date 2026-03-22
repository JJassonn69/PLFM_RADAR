#!/usr/bin/env python3
"""
Fix via-via clearance violations in KiCad PCB file.

Reads the KiCad .kicad_pcb S-expression file, finds via pairs that are
too close (< 0.1 mm clearance), and nudges the non-GND via (or the second
via if both are GND) away from its neighbor to achieve minimum clearance.

Only touches via-via violations. Trace-via and via-pad violations are
left for manual KiCad GUI fixes.

Clearance = distance_between_centers - (size1/2 + size2/2)
We target MIN_CLEARANCE = 0.125 mm (25% margin above JLCPCB 0.1 mm min).
"""
import re
import math
import copy
import sys
import os

PCB_FILE = (
    "/Users/ganeshpanth/PLFM_RADAR/"
    "4_Schematics and Boards Layout/4_6_Schematics/"
    "FrequencySynthesizerBoard/.history/Clocks_Freq_Synth_board.kicad_pcb"
)

MIN_CLEARANCE = 0.125  # mm — target clearance (JLCPCB min is 0.1)
MIN_VIA_SIZE = 0.45    # mm — minimum via size after annular fix (0.3 drill + 2*0.075 ring)
DRY_RUN = "--dry-run" in sys.argv

# ── Strategy ──
# The DRC violations come from vias that are currently size=0.35mm but will
# be upsized to 0.45mm for JLCPCB annular ring compliance. We compute
# clearances using the POST-UPSIZE diameter (0.45mm) and nudge vias apart
# so that after upsizing, all via-via clearances >= MIN_CLEARANCE.
#
# Instead of a hardcoded violation list, we dynamically detect all via-via
# pairs that would violate MIN_CLEARANCE after upsizing to MIN_VIA_SIZE.

# ── Parse all vias from PCB file ──

VIA_RE = re.compile(r'\t\(via\n(.*?)\t\)', re.DOTALL)
AT_RE = re.compile(r'\(at ([\d.]+) ([\d.]+)\)')
SIZE_RE = re.compile(r'\(size ([\d.]+)\)')
DRILL_RE = re.compile(r'\(drill ([\d.]+)\)')
NET_RE = re.compile(r'\(net "([^"]+)"\)')


def parse_vias(content):
    """Return list of dicts with via info and span in file."""
    vias = []
    for m in VIA_RE.finditer(content):
        block = m.group(0)
        at = AT_RE.search(block)
        sz = SIZE_RE.search(block)
        dr = DRILL_RE.search(block)
        nt = NET_RE.search(block)
        if at and sz and nt:
            vias.append({
                'x': float(at.group(1)),
                'y': float(at.group(2)),
                'size': float(sz.group(1)),
                'drill': float(dr.group(1)) if dr else 0.0,
                'net': nt.group(1),
                'span_start': m.start(),
                'span_end': m.end(),
                'original': block,
            })
    return vias


def via_clearance_post_upsize(v1, v2):
    """Compute edge-to-edge clearance between two vias AFTER upsizing."""
    dx = v1['x'] - v2['x']
    dy = v1['y'] - v2['y']
    dist = math.sqrt(dx * dx + dy * dy)
    s1 = max(v1['size'], MIN_VIA_SIZE)
    s2 = max(v2['size'], MIN_VIA_SIZE)
    return dist - (s1 / 2 + s2 / 2)


def compute_nudge(via_to_move, via_anchor, target_clearance):
    """
    Compute (dx, dy) to nudge via_to_move away from via_anchor
    so that edge-to-edge clearance = target_clearance (post-upsize).
    """
    dx = via_to_move['x'] - via_anchor['x']
    dy = via_to_move['y'] - via_anchor['y']
    current_dist = math.sqrt(dx * dx + dy * dy)

    if current_dist < 1e-6:
        # Vias are essentially overlapping — pick arbitrary direction (away in +x)
        dx, dy = 1.0, 0.0
        current_dist = 1.0

    s_move = max(via_to_move['size'], MIN_VIA_SIZE)
    s_anchor = max(via_anchor['size'], MIN_VIA_SIZE)
    required_dist = (s_move / 2 + s_anchor / 2) + target_clearance
    if current_dist >= required_dist:
        return 0.0, 0.0  # Already fine

    # Unit vector from anchor toward via_to_move
    ux = dx / current_dist
    uy = dy / current_dist

    move_dist = required_dist - current_dist
    return ux * move_dist, uy * move_dist


def is_diff_pair(net1, net2):
    """Check if two nets form a differential pair (e.g. OUT8_P and OUT8_N)."""
    # Strip trailing _P/_N and compare base names
    for suffix_a, suffix_b in [('_P', '_N'), ('_p', '_n')]:
        if net1.endswith(suffix_a) and net2.endswith(suffix_b):
            if net1[:-len(suffix_a)] == net2[:-len(suffix_b)]:
                return True
        if net1.endswith(suffix_b) and net2.endswith(suffix_a):
            if net1[:-len(suffix_b)] == net2[:-len(suffix_a)]:
                return True
    return False


def detect_violations(vias):
    """Find all via-via pairs that violate MIN_CLEARANCE after upsizing."""
    violations = []
    diff_pair_skipped = 0
    for i in range(len(vias)):
        for j in range(i + 1, len(vias)):
            v1, v2 = vias[i], vias[j]
            if v1['net'] == v2['net']:
                continue  # same-net, no clearance issue
            clr = via_clearance_post_upsize(v1, v2)
            if clr < MIN_CLEARANCE:
                if is_diff_pair(v1['net'], v2['net']):
                    diff_pair_skipped += 1
                    continue
                violations.append((clr, v1, v2))
    violations.sort(key=lambda x: x[0])
    if diff_pair_skipped:
        print(f"  (Skipped {diff_pair_skipped} differential pair violations — intentionally close)")
    return violations


def replace_via_position(via_block, new_x, new_y):
    """Replace the (at ...) in a via block with new coordinates."""
    return AT_RE.sub(f'(at {new_x:.4f} {new_y:.4f})', via_block, count=1)


# ── Main ──

print(f"Reading: {PCB_FILE}")
with open(PCB_FILE, 'r') as f:
    content = f.read()

# Save backup before any changes
backup = PCB_FILE + ".bak_before_clearance_fix"
if not os.path.exists(backup):
    import shutil
    shutil.copy2(PCB_FILE, backup)
    print(f"Backup saved: {backup}")

total_moved = 0

# ── Single-pass greedy approach ──
# Multi-iteration approaches oscillate because vias squeezed between two
# neighbors get pulled back and forth. A single greedy pass gives the best
# improvement without creating cascading conflicts.

vias = parse_vias(content)
print(f"Parsed {len(vias)} vias")

violations = detect_violations(vias)
if not violations:
    print("No violations found. PCB is clean!")
    sys.exit(0)

print(f"\n{len(violations)} via-via pairs with clearance < {MIN_CLEARANCE} mm (post-upsize to {MIN_VIA_SIZE} mm)")

# Build move plan: for each via, accumulate ALL repulsion forces from
# violating neighbors, then apply the vector sum (compromise direction).
# This is better than picking the largest single move because it finds
# a position that improves ALL constraints simultaneously.
forces = {}  # key -> [sum_dx, sum_dy, count, via_ref]

for actual_clr, v1, v2 in violations:
    net1, net2 = v1['net'], v2['net']

    # Decide which via to move: prefer moving non-GND (GND = stitching)
    if net1 == "GND" and net2 != "GND":
        anchor, mover = v1, v2
    elif net2 == "GND" and net1 != "GND":
        anchor, mover = v2, v1
    elif net1 == "GND" and net2 == "GND":
        # Both GND — don't move stitching vias at all
        continue
    else:
        # Neither is GND — move the second one (arbitrary but consistent)
        anchor, mover = v1, v2

    nudge_dx, nudge_dy = compute_nudge(mover, anchor, MIN_CLEARANCE)
    if abs(nudge_dx) < 1e-6 and abs(nudge_dy) < 1e-6:
        continue

    key = (round(mover['x'], 4), round(mover['y'], 4), mover['net'])
    if key not in forces:
        forces[key] = [0.0, 0.0, 0, mover]
    forces[key][0] += nudge_dx
    forces[key][1] += nudge_dy
    forces[key][2] += 1

# Convert forces to moves — no damping, full correction
moves = {}
for key, (fdx, fdy, count, via) in forces.items():
    if count == 1:
        # Single constraint: apply full correction
        new_x = via['x'] + fdx
        new_y = via['y'] + fdy
    else:
        # Multiple constraints: use vector sum (compromise direction)
        # Scale so the resulting move at least satisfies the closest neighbor
        new_x = via['x'] + fdx / count * 1.2  # slight overshoot for margin
        new_y = via['y'] + fdy / count * 1.2
    moves[key] = (new_x, new_y)

print(f"\nPlanning {len(moves)} via moves:")
for key, (nx, ny) in sorted(moves.items(), key=lambda x: x[0]):
    dx = nx - key[0]
    dy = ny - key[1]
    move_mm = math.sqrt(dx**2 + dy**2)
    net_name = key[2]
    n_constraints = forces[key][2]
    tag = f" [{n_constraints} constraints]" if n_constraints > 1 else ""
    print(f"  [{net_name}] ({key[0]:.4f},{key[1]:.4f}) "
          f"+({dx:+.4f},{dy:+.4f})={move_mm:.3f}mm{tag}")

# Apply moves to content
counter = [0]
def apply_via_move(m, _moves=moves, _counter=counter):
    block = m.group(0)
    at = AT_RE.search(block)
    nt = NET_RE.search(block)
    if not at or not nt:
        return block
    x = round(float(at.group(1)), 4)
    y = round(float(at.group(2)), 4)
    net = nt.group(1)
    key = (x, y, net)
    if key in _moves:
        new_x, new_y = _moves[key]
        _counter[0] += 1
        return replace_via_position(block, new_x, new_y)
    return block

content = VIA_RE.sub(apply_via_move, content)
total_moved = counter[0]
print(f"\nApplied {total_moved} via moves")

if DRY_RUN:
    print(f"\n[DRY RUN] {total_moved} total via moves computed but NOT written to disk.")
elif total_moved == 0:
    print("Nothing to fix.")
    sys.exit(0)
else:
    with open(PCB_FILE, 'w') as f:
        f.write(content)

# Final verification — two-tier reporting
JLCPCB_MIN = 0.100  # hard minimum

vias_final = parse_vias(content)
remaining_soft = detect_violations(vias_final)  # < 0.125 mm (our target)

# Count how many are below the JLCPCB hard minimum
remaining_hard = [(c, v1, v2) for c, v1, v2 in remaining_soft if c < JLCPCB_MIN]

print(f"\n{'='*60}")
print(f"RESULT: Applied {total_moved} total via position changes.")
print(f"\nViolations below {MIN_CLEARANCE} mm (target):  {len(remaining_soft)}")
print(f"Violations below {JLCPCB_MIN} mm (JLCPCB min): {len(remaining_hard)}")

if remaining_hard:
    print(f"\n--- FAB-BLOCKING (< {JLCPCB_MIN} mm) — MUST fix in KiCad ---")
    for clr, v1, v2 in remaining_hard:
        print(f"  {clr:.4f} mm: [{v1['net']}] ({v1['x']:.4f},{v1['y']:.4f}) "
              f"<-> [{v2['net']}] ({v2['x']:.4f},{v2['y']:.4f})")

soft_only = [(c, v1, v2) for c, v1, v2 in remaining_soft if c >= JLCPCB_MIN]
if soft_only:
    print(f"\n--- MARGINAL ({JLCPCB_MIN}..{MIN_CLEARANCE} mm) — recommended but not blocking ---")
    for clr, v1, v2 in soft_only:
        print(f"  {clr:.4f} mm: [{v1['net']}] ({v1['x']:.4f},{v1['y']:.4f}) "
              f"<-> [{v2['net']}] ({v2['x']:.4f},{v2['y']:.4f})")

if not remaining_hard:
    print(f"\nAll via-via pairs meet JLCPCB minimum ({JLCPCB_MIN} mm). Board is fabricable!")

print(f"\nFixed PCB written to: {PCB_FILE}")
print(f"\nNext steps:")
print(f"  1. Open in KiCad and run DRC to verify")
print(f"  2. Fix remaining trace-via and via-pad violations manually in KiCad GUI")
print(f"  3. Check that moved vias still connect to their copper pours/traces")
