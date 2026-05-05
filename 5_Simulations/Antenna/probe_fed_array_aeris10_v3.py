#!/usr/bin/env python3
# probe_fed_array_aeris10_v3.py
#
# 4x4 (default; configurable) probe-fed patch array sim for AERIS-10. Built
# on the same single-element design point as probe_fed_aeris10_v3.py but
# placed on the 8x16 Gerber pitch (14.27 mm X / 15.01 mm Y).
#
# Purpose: characterise mutual coupling between elements. Each patch has its
# own probe-via port; only one port is excited per sim run, the other 15 are
# terminated in 50 Ω. From this we read:
#   - S_dd (active S11 of the driven element with array loaded)
#   - S_jd  for all other ports j (coupling driven → j)
# Pattern of |S_jd| dB values across the array tells us nearest-neighbour vs
# diagonal vs skip-one coupling, edge vs interior asymmetry, etc.
#
# Per-element design (matches probe_fed_aeris10_v3.py iter#3):
#   PATCH_W = 7.854 mm   PATCH_L = 6.56 mm   FEED_OFFSET = 2.14 mm
# Substrate: 0.508 mm RO4350B (εr=3.48, tanδ=0.0037)
# Pitch: 14.27 mm × 15.01 mm  (X-pitch ~λ₀/2 at 10.5 GHz, Y-pitch ~1.05·λ₀/2)
#
# Run:
#   cd /tmp && DYLD_LIBRARY_PATH=/Users/ganeshpanth/opt/openEMS/lib \
#     PROFILE=sanity \
#     /Users/ganeshpanth/radar_venv/bin/python \
#     /Users/ganeshpanth/PLFM_RADAR/5_Simulations/Antenna/probe_fed_array_aeris10_v3.py
#
# Env overrides:
#   ARRAY_NX  ARRAY_NY  (default 4, 4)
#   PITCH_X_MM  PITCH_Y_MM   (default 14.27, 15.01 from Gerber)
#   DRIVEN_X  DRIVEN_Y       (0-indexed; default = inner element 1,1)
#   PATCH_W_MM  PATCH_L_MM  FEED_OFFSET_MM  (default v3 design point)
#
# Output (in /tmp/aeris10_array_v3/):
#   S_matrix.csv  — driven-column S parameters (mag dB + phase deg) at 10.5 GHz
#   S11_data.csv  — driven port full sweep
#   coupling_grid.png  — heatmap of |S_jd| dB at 10.5 GHz across array

import os
import time
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from openEMS import openEMS
from openEMS.physical_constants import C0
from CSXCAD import ContinuousStructure
from CSXCAD.SmoothMeshLines import SmoothMeshLines

# ============================================================================
# PROFILES
# ============================================================================
PROFILE = os.environ.get("PROFILE", "sanity")
profiles = {
    "sanity":   {"mesh_lambda_div": 18, "n_timesteps": 50000, "end_dB": -30},
    "balanced": {"mesh_lambda_div": 25, "n_timesteps": 80000, "end_dB": -40},
}
cfg = profiles[PROFILE]

# ============================================================================
# BAND
# ============================================================================
F0      = 10.5e9
F_SPAN  = 4.0e9
F_START = F0 - F_SPAN/2
F_STOP  = F0 + F_SPAN/2

# ============================================================================
# STACKUP
# ============================================================================
T_CU         = 0.035
H_PATCH_SUB  = 0.508
EPS_RO4350B  = 3.48
TAN_RO4350B  = 0.0037

Z_GND   = 0.0
Z_PATCH = Z_GND + T_CU + H_PATCH_SUB
Z_TOP   = Z_PATCH + T_CU

# ============================================================================
# PATCH (per-element, from v3 iter#3)
# ============================================================================
PATCH_W = float(os.environ.get("PATCH_W_MM", "7.854"))
PATCH_L = float(os.environ.get("PATCH_L_MM", "6.56"))
FEED_OFFSET_MM = float(os.environ.get("FEED_OFFSET_MM", "2.14"))

# ============================================================================
# ARRAY
# ============================================================================
N_X = int(os.environ.get("ARRAY_NX", "4"))
N_Y = int(os.environ.get("ARRAY_NY", "4"))
PITCH_X = float(os.environ.get("PITCH_X_MM", "14.27"))
PITCH_Y = float(os.environ.get("PITCH_Y_MM", "15.01"))
DRIVEN_X = int(os.environ.get("DRIVEN_X", str(N_X // 2 - (N_X+1) % 2)))   # inner element
DRIVEN_Y = int(os.environ.get("DRIVEN_Y", str(N_Y // 2 - (N_Y+1) % 2)))

# DRIVEN_PORTS overrides DRIVEN_X/Y — comma/semicolon-separated list of "i,j"
# pairs all excited in-phase with equal amplitude. Models perfect 1:8 corporate
# splitter feeding an 8-patch sub-array. Example: "0,0;1,0;2,0;3,0;0,1;1,1;2,1;3,1"
# is a 4-cols × 2-rows sub-array anchored in the corner.
DRIVEN_PORTS_STR = os.environ.get("DRIVEN_PORTS", "")
if DRIVEN_PORTS_STR:
    pairs = []
    for tok in DRIVEN_PORTS_STR.replace(';', ',').split(','):
        tok = tok.strip()
        if tok:
            pairs.append(int(tok))
    if len(pairs) % 2 != 0:
        raise ValueError("DRIVEN_PORTS must be even count of integers (i,j pairs)")
    DRIVEN_SET = set((pairs[k], pairs[k+1]) for k in range(0, len(pairs), 2))
else:
    DRIVEN_SET = {(DRIVEN_X, DRIVEN_Y)}

# Array footprint extent (centre patch on origin)
ARRAY_X_HALF = (N_X-1)/2 * PITCH_X + PATCH_W/2
ARRAY_Y_HALF = (N_Y-1)/2 * PITCH_Y + PATCH_L/2

# Substrate / ground extents (~λ/2 margin around array)
GND_MARGIN = 14.3
GND_X_HALF = ARRAY_X_HALF + GND_MARGIN
GND_Y_HALF = ARRAY_Y_HALF + GND_MARGIN

# Air box
AIR_ABOVE = 14.3
AIR_BELOW = 14.3
AIR_X_HALF = GND_X_HALF + 8.0
AIR_Y_HALF = GND_Y_HALF + 8.0

OUT_DIR = "/tmp/aeris10_array_v3"
os.makedirs(OUT_DIR, exist_ok=True)


def patch_center(i, j):
    """Centre coordinate of patch at array index (i,j), origin at array centre."""
    x = -(N_X-1)/2 * PITCH_X + i * PITCH_X
    y = -(N_Y-1)/2 * PITCH_Y + j * PITCH_Y
    return x, y


# ============================================================================
# Build + run
# ============================================================================
def run_case(sim_path, profile_cfg):
    fdtd = openEMS(NrTS=profile_cfg["n_timesteps"],
                   EndCriteria=10**(profile_cfg["end_dB"]/20.0))
    fdtd.SetGaussExcite(F0, F_SPAN/2.0)
    fdtd.SetBoundaryCond(["MUR"]*6)

    CSX = ContinuousStructure()
    fdtd.SetCSX(CSX)
    mesh = CSX.GetGrid()
    mesh.SetDeltaUnit(1e-3)

    # ---- materials ----
    eps0 = 8.854e-12
    patch_sub = CSX.AddMaterial("RO4350B",
        epsilon=EPS_RO4350B,
        kappa=2*np.pi*F0*EPS_RO4350B*eps0*TAN_RO4350B)
    copper = CSX.AddMetal("Copper")

    # ---- substrate (full board extent) ----
    patch_sub.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND + T_CU],
                      [+GND_X_HALF, +GND_Y_HALF, Z_PATCH], priority=1)

    # ---- L2: full ground plane ----
    copper.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND],
                  [+GND_X_HALF, +GND_Y_HALF, Z_GND + T_CU], priority=10)

    # ---- L1: 4x4 patch array + ports ----
    ports = []
    feed_locs = []
    for i in range(N_X):
        for j in range(N_Y):
            cx, cy = patch_center(i, j)
            copper.AddBox([cx - PATCH_W/2, cy - PATCH_L/2, Z_PATCH],
                          [cx + PATCH_W/2, cy + PATCH_L/2, Z_PATCH + T_CU],
                          priority=10)
            feed_x = cx
            feed_y = cy - PATCH_L/2 + FEED_OFFSET_MM
            feed_locs.append((feed_x, feed_y, i, j))

    # ---- mesh ----
    lambda_min_mm = (C0 / F_STOP) * 1000.0
    res = lambda_min_mm / profile_cfg["mesh_lambda_div"]

    # X mesh: array extent + air, plus patch edges + feed locations
    xlines = [-AIR_X_HALF, -GND_X_HALF, +GND_X_HALF, +AIR_X_HALF]
    for i in range(N_X):
        cx, _ = patch_center(i, 0)
        xlines += [cx - PATCH_W/2, cx, cx + PATCH_W/2]
    # Y mesh
    ylines = [-AIR_Y_HALF, -GND_Y_HALF, +GND_Y_HALF, +AIR_Y_HALF]
    for j in range(N_Y):
        _, cy = patch_center(0, j)
        ylines += [cy - PATCH_L/2, cy, cy + PATCH_L/2,
                   cy - PATCH_L/2 + FEED_OFFSET_MM]   # feed y location

    # Z mesh: 6 cells in substrate
    air_below = list(np.arange(Z_GND - T_CU - AIR_BELOW, Z_GND - T_CU, res))
    air_above = list(np.arange(Z_TOP + res, Z_TOP + AIR_ABOVE + res, res))
    sub_interior = list(np.linspace(Z_GND + T_CU, Z_PATCH, 7)[1:-1])
    zlines = sorted(set(air_below + [
        Z_GND - T_CU, Z_GND, Z_GND + T_CU,
        Z_PATCH, Z_PATCH + T_CU,
    ] + sub_interior + air_above))

    xlines = SmoothMeshLines(np.array(xlines), res)
    ylines = SmoothMeshLines(np.array(ylines), res)
    zlines = np.array(zlines)
    mesh.AddLine("x", xlines)
    mesh.AddLine("y", ylines)
    mesh.AddLine("z", zlines)
    n_cells = len(xlines) * len(ylines) * len(zlines)

    # ---- ports (one excited, 15 terminated 50Ω) ----
    # NOTE: each port box must land exactly on existing mesh lines. The seed
    # mesh includes feed_x (= patch centre cx) and feed_y for each (i,j) — so
    # this normally works — but SmoothMeshLines can sub-cell-shift seed lines
    # in some configurations and a port box ends up between two mesh lines,
    # leaving openEMS without an excitation cell ("Unused primitive" warning,
    # zero energy). The DRIVEN=(1,1) inner-element case has been verified to
    # land on the mesh; other driven-port choices are best-effort. If you see
    # NaN/zero results for a different DRIVEN_X/DRIVEN_Y, that's the cause.
    for (feed_x, feed_y, i, j) in feed_locs:
        port_num = i * N_Y + j + 1
        excite_amp = 1.0 if (i, j) in DRIVEN_SET else 0.0
        port = fdtd.AddLumpedPort(port_num, 50,
                                   [feed_x, feed_y, Z_GND + T_CU],
                                   [feed_x, feed_y, Z_PATCH],
                                   'z', excite=excite_amp, priority=5)
        ports.append(((i, j), port))

    # ---- run ----
    print(f"[case] {N_X}x{N_Y} array, driven=({DRIVEN_X},{DRIVEN_Y}), "
          f"cells={n_cells:,}, sub={H_PATCH_SUB}mm, pitch={PITCH_X}x{PITCH_Y}mm")
    t0 = time.time()
    fdtd.Run(sim_path, verbose=0, cleanup=True)
    dt = time.time() - t0

    # ---- post-process ----
    freq = np.linspace(F_START, F_STOP, 401)
    for (idx, p) in ports:
        p.CalcPort(sim_path, freq)

    # For each excited port: S = uf_ref/uf_inc (active reflection with all
    # driven ports excited). For each non-excited port: S = uf_ref / <inc>
    # where <inc> is the incident wave from any one driven port (used as
    # reference; all driven ports have equal amplitude so any works).
    ref_inc = next(p for (idx, p) in ports if idx in DRIVEN_SET).uf_inc
    S = {}
    for (idx, p) in ports:
        if idx in DRIVEN_SET:
            S[idx] = p.uf_ref / p.uf_inc      # active S11
        else:
            S[idx] = p.uf_ref / ref_inc        # coupling out

    return freq, S, dt, ports


# ============================================================================
# MAIN
# ============================================================================
sim_path = os.path.join(OUT_DIR, "single")
freq, S, dt, ports = run_case(sim_path, cfg)

# At 10.5 GHz
i_op = int(np.argmin(np.abs(freq - F0)))

N_DRIVEN = len(DRIVEN_SET)

# Print coupling grid
print()
print("=" * 70)
if N_DRIVEN == 1:
    dx, dy = list(DRIVEN_SET)[0]
    print(f"  {N_X}x{N_Y} probe-fed array — driven port at ({dx},{dy})")
else:
    sub_str = ' '.join(f"({i},{j})" for (i, j) in sorted(DRIVEN_SET))
    print(f"  {N_X}x{N_Y} probe-fed array — {N_DRIVEN}-patch sub-array driven in-phase")
    print(f"  Sub-array: {sub_str}")
print(f"  Substrate: {H_PATCH_SUB} mm RO4350B, pitch {PITCH_X}x{PITCH_Y} mm")
print(f"  Sim time: {dt:.1f} s")
print("=" * 70)
print()
print(f"  |S| at {F0/1e9:.2f} GHz, dB (driven ports show active S11):")
print()
# Layout grid as visual array
header = "      " + "".join(f"  i={i:1d}  " for i in range(N_X))
print(header)
for j in reversed(range(N_Y)):
    row = f"  j={j:1d}: "
    for i in range(N_X):
        val = abs(S[(i, j)][i_op])
        dB = 20*np.log10(val + 1e-30)
        marker = "*" if (i, j) in DRIVEN_SET else " "
        row += f"{marker}{dB:>6.1f}"
    print(row)
print("    (* = driven port)")
print()

# For each driven port, report active S11 + Zin + per-port BW
print(f"  Active S11 per driven port at {F0/1e9:.2f} GHz:")
S11_at_op = []
zin_at_op = []
for (i, j) in sorted(DRIVEN_SET):
    s = S[(i, j)]
    s_dB_op = 20*np.log10(abs(s[i_op]) + 1e-30)
    zin_p = 50.0 * (1 + s) / (1 - s)
    print(f"    ({i},{j}) : S11 = {s_dB_op:>6.2f} dB    Z = {zin_p[i_op].real:5.1f} + j{zin_p[i_op].imag:+5.1f} Ω")
    S11_at_op.append(s_dB_op)
    zin_at_op.append(zin_p[i_op])

if N_DRIVEN > 1:
    print()
    print("  Sub-array uniformity:")
    print(f"    S11 min/max/avg : {min(S11_at_op):>6.2f} / {max(S11_at_op):>6.2f} / "
          f"{sum(S11_at_op)/N_DRIVEN:>6.2f} dB")
    R_vals = [z.real for z in zin_at_op]
    X_vals = [z.imag for z in zin_at_op]
    print(f"    R  min/max/avg  : {min(R_vals):>5.1f} / {max(R_vals):>5.1f} / "
          f"{sum(R_vals)/N_DRIVEN:>5.1f} Ω")
    print(f"    X  min/max/avg  : {min(X_vals):+5.1f} / {max(X_vals):+5.1f} / "
          f"{sum(X_vals)/N_DRIVEN:+5.1f} Ω")
    # Average port (what the ADAR channel "sees" through ideal 1:8 splitter)
    Z_avg = sum(zin_at_op) / N_DRIVEN
    print("    Z avg (= what ADAR channel sees through ideal 1:8 splitter):")
    print(f"      Z = {Z_avg.real:.1f} + j{Z_avg.imag:+.1f} Ω, "
          f"VSWR = {abs((Z_avg-50)/(Z_avg+50)) and (1+abs((Z_avg-50)/(Z_avg+50)))/(1-abs((Z_avg-50)/(Z_avg+50))):.2f}")

# Coupling out (top non-driven ports)
nondriven_couplings = [(idx, abs(S[idx][i_op])) for idx in S.keys()
                       if idx not in DRIVEN_SET]
nondriven_couplings.sort(key=lambda x: -x[1])
print()
print(f"  Top-5 strongest couplings OUT of sub-array at {F0/1e9:.2f} GHz:")
for idx, val in nondriven_couplings[:5]:
    dB = 20*np.log10(val + 1e-30)
    print(f"    ({idx[0]},{idx[1]})  |S| = {dB:>6.1f} dB")
print("=" * 70)

# Save S matrix CSV (full-band)
with open(os.path.join(OUT_DIR, "S_matrix.csv"), "w", newline="") as f:
    w = csv.writer(f)
    header = ["freq_Hz"]
    keys = sorted(S.keys())
    for idx in keys:
        header += [f"S({idx[0]},{idx[1]})_dB", f"S({idx[0]},{idx[1]})_phase_deg"]
    w.writerow(header)
    for k in range(len(freq)):
        row = [freq[k]]
        for idx in keys:
            mag_dB = 20*np.log10(np.abs(S[idx][k]) + 1e-30)
            phase = np.angle(S[idx][k], deg=True)
            row += [mag_dB, phase]
        w.writerow(row)

# Coupling heatmap at 10.5 GHz
fig, ax = plt.subplots(figsize=(7, 6.5))
grid = np.zeros((N_Y, N_X))
for (i, j) in S.keys():
    grid[j, i] = 20*np.log10(abs(S[(i,j)][i_op]) + 1e-30)
im = ax.imshow(grid, origin='lower', cmap='viridis', aspect='equal')
ax.set_xticks(range(N_X))
ax.set_yticks(range(N_Y))
ax.set_xlabel('i (x-pitch direction)')
ax.set_ylabel('j (y-pitch direction)')
ax.set_title(f'AERIS-10 {N_X}x{N_Y} probe-fed array — |S| at {F0/1e9:.2f} GHz')
for j in range(N_Y):
    for i in range(N_X):
        if (i, j) in DRIVEN_SET:
            ax.text(i, j, f"DRIVEN\n{grid[j,i]:.1f} dB", ha='center', va='center',
                    color='red', fontsize=8, fontweight='bold')
        else:
            ax.text(i, j, f"{grid[j,i]:.1f}\ndB", ha='center', va='center',
                    color='white', fontsize=7)
plt.colorbar(im, ax=ax, label='|S| (dB)', shrink=0.7)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "coupling_grid.png"), dpi=140)
plt.close(fig)

print(f"[out] {OUT_DIR}/coupling_grid.png")
print(f"[out] {OUT_DIR}/S_matrix.csv")
print(f"[out] {OUT_DIR}/S11_data.csv")
