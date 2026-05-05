#!/usr/bin/env python3
# edge_fed_row_nf2ff_aeris10_v3.py
#
# Far-field analysis of the 1x8 series-fed row at f=10.520 GHz to confirm
# whether the operating mode at the TX-centered design point is broadside
# (peak in +z, θ=0°) or scanned (peak at θ ≠ 0).
#
# Same geometry as edge_fed_row_aeris10_v3.py at the verified design point
# (CONN_LEN=8.15, no inset on patch 0). Adds an nf2ff probe box around the
# simulation domain so the radiation pattern can be computed post-sim.
#
# Coordinate convention (openEMS/standard physics):
#   z is normal to the board (+z = upward, away from ground plane)
#   y is the array axis (patches arranged along +y)
#   x is the patch-W direction
#   theta = angle from +z (broadside is θ=0°)
#   phi=0  → xz plane (H-plane cut, perpendicular to array)
#   phi=90 → yz plane (E-plane cut, ALONG array axis — this is where the
#            array factor lives; broadside check happens HERE)
#
# Run:
#   cd /tmp && DYLD_LIBRARY_PATH=/Users/ganeshpanth/opt/openEMS/lib \
#     PROFILE=balanced \
#     /Users/ganeshpanth/radar_venv/bin/python \
#     /Users/ganeshpanth/PLFM_RADAR/5_Simulations/Antenna/edge_fed_row_nf2ff_aeris10_v3.py

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

PROFILE = os.environ.get("PROFILE", "balanced")
# end_dB is the ENERGY ratio threshold reported in the engine's "(-X dB)" line
# (10·log10 convention). nf2ff needs at least -40 dB (energy ratio 1e-4) for a
# valid radiated-power integral.
profiles = {
    "sanity":   {"mesh_lambda_div": 18, "n_timesteps": 150000, "end_dB": -35},
    "balanced": {"mesh_lambda_div": 25, "n_timesteps": 300000, "end_dB": -40},
}
cfg = profiles[PROFILE]

F0      = 10.5e9
F_SPAN  = 4.0e9
F_TX    = float(os.environ.get("F_TX_GHZ", "10.520")) * 1e9
F_START = F0 - F_SPAN/2
F_STOP  = F0 + F_SPAN/2

T_CU         = 0.035
H_PATCH_SUB  = 0.508
EPS_RO4350B  = 3.48
TAN_RO4350B  = 0.0037

Z_GND   = 0.0
Z_PATCH = Z_GND + T_CU + H_PATCH_SUB
Z_TOP   = Z_PATCH + T_CU

# Verified TX-centered design point
N_PATCHES    = int(os.environ.get("N_PATCHES", "8"))
PATCH_W      = float(os.environ.get("PATCH_W_MM", "7.854"))
PATCH_L      = float(os.environ.get("PATCH_L_MM", "6.95"))
FEED_W       = float(os.environ.get("FEED_W_MM", "1.16"))
INSET_DEPTH  = float(os.environ.get("INSET_DEPTH_MM", "0.0"))
INSET_GAP    = float(os.environ.get("INSET_GAP_MM", "0.30"))
FEED_LEAD_L  = float(os.environ.get("FEED_LEAD_MM", "15.5"))
CONN_LEN     = float(os.environ.get("CONN_LEN_MM", "8.15"))

PITCH = PATCH_L + CONN_LEN

Y_FEED_BOARD_EDGE = -PATCH_L/2 - FEED_LEAD_L
Y_LAST_PATCH_TOP  = (N_PATCHES - 1) * PITCH + PATCH_L/2

GND_X_MARGIN = 14.3
GND_Y_MARGIN = 14.3
GND_X_HALF = max(PATCH_W/2, FEED_W/2 + INSET_GAP) + GND_X_MARGIN
GND_Y_NEG  = Y_FEED_BOARD_EDGE - GND_Y_MARGIN
GND_Y_POS  = Y_LAST_PATCH_TOP + GND_Y_MARGIN

AIR_ABOVE  = 14.3
AIR_BELOW  = 14.3
AIR_X_HALF = GND_X_HALF + 8.0
AIR_Y_NEG  = GND_Y_NEG - 8.0
AIR_Y_POS  = GND_Y_POS + 8.0

OUT_DIR = "/tmp/aeris10_edgefed_row_nf2ff_v3"
os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================================
# Build sim
# ============================================================================
fdtd = openEMS(NrTS=cfg["n_timesteps"],
               EndCriteria=10**(cfg["end_dB"]/10.0))
fdtd.SetGaussExcite(F0, F_SPAN/2.0)
fdtd.SetBoundaryCond(["MUR"]*6)

CSX = ContinuousStructure()
fdtd.SetCSX(CSX)
mesh = CSX.GetGrid()
mesh.SetDeltaUnit(1e-3)

eps0 = 8.854e-12
patch_sub = CSX.AddMaterial("RO4350B",
    epsilon=EPS_RO4350B,
    kappa=2*np.pi*F0*EPS_RO4350B*eps0*TAN_RO4350B)
copper = CSX.AddMetal("Copper")

patch_sub.AddBox([-GND_X_HALF, GND_Y_NEG, Z_GND + T_CU],
                 [+GND_X_HALF, GND_Y_POS, Z_PATCH], priority=1)
copper.AddBox([-GND_X_HALF, GND_Y_NEG, Z_GND],
              [+GND_X_HALF, GND_Y_POS, Z_GND + T_CU], priority=10)

notch_half_w = FEED_W/2 + INSET_GAP

for i in range(N_PATCHES):
    py0 = i * PITCH - PATCH_L/2
    py1 = i * PITCH + PATCH_L/2
    if i == 0 and INSET_DEPTH > 0.001:
        copper.AddBox([-PATCH_W/2, py0 + INSET_DEPTH, Z_PATCH],
                      [+PATCH_W/2, py1, Z_PATCH + T_CU], priority=10)
        copper.AddBox([-PATCH_W/2, py0, Z_PATCH],
                      [-notch_half_w, py0 + INSET_DEPTH, Z_PATCH + T_CU],
                      priority=10)
        copper.AddBox([+notch_half_w, py0, Z_PATCH],
                      [+PATCH_W/2, py0 + INSET_DEPTH, Z_PATCH + T_CU],
                      priority=10)
    else:
        copper.AddBox([-PATCH_W/2, py0, Z_PATCH],
                      [+PATCH_W/2, py1, Z_PATCH + T_CU], priority=10)

for i in range(N_PATCHES - 1):
    cy0 = i * PITCH + PATCH_L/2
    cy1 = (i + 1) * PITCH - PATCH_L/2
    copper.AddBox([-FEED_W/2, cy0, Z_PATCH],
                  [+FEED_W/2, cy1, Z_PATCH + T_CU], priority=10)

feed_y_start = Y_FEED_BOARD_EDGE
feed_y_end   = (-PATCH_L/2 + INSET_DEPTH) if INSET_DEPTH > 0.001 else -PATCH_L/2
copper.AddBox([-FEED_W/2, feed_y_start, Z_PATCH],
              [+FEED_W/2, feed_y_end, Z_PATCH + T_CU], priority=10)

# Mesh
lambda_min_mm = (C0 / F_STOP) * 1000.0
res = lambda_min_mm / cfg["mesh_lambda_div"]
PORT_LEN = 2.0

xlines = [-AIR_X_HALF, -GND_X_HALF, -PATCH_W/2, -notch_half_w, -FEED_W/2,
          0, +FEED_W/2, +notch_half_w, +PATCH_W/2, +GND_X_HALF, +AIR_X_HALF]

ylines = [AIR_Y_NEG, GND_Y_NEG, feed_y_start]
for i in range(N_PATCHES):
    ylines.append(i * PITCH - PATCH_L/2)
    ylines.append(i * PITCH)
    ylines.append(i * PITCH + PATCH_L/2)
ylines += [GND_Y_POS, AIR_Y_POS]
port_y_lines = list(np.linspace(feed_y_start, feed_y_start + PORT_LEN, 6))
ylines += port_y_lines

air_below = list(np.arange(Z_GND - T_CU - AIR_BELOW, Z_GND - T_CU, res))
air_above = list(np.arange(Z_TOP + res, Z_TOP + AIR_ABOVE + res, res))
sub_interior = list(np.linspace(Z_GND + T_CU, Z_PATCH, 7)[1:-1])
zlines = sorted(set(air_below + [
    Z_GND - T_CU, Z_GND, Z_GND + T_CU,
    Z_PATCH, Z_PATCH + T_CU,
] + sub_interior + air_above))

xlines = SmoothMeshLines(np.array(xlines), res)
ylines = SmoothMeshLines(np.array(sorted(set(ylines))), res)
zlines = np.array(zlines)
mesh.AddLine("x", xlines)
mesh.AddLine("y", ylines)
mesh.AddLine("z", zlines)
n_cells = len(xlines) * len(ylines) * len(zlines)

port = fdtd.AddMSLPort(1, copper,
    start=[-FEED_W/2, feed_y_start, Z_GND + T_CU],
    stop= [+FEED_W/2, feed_y_start + PORT_LEN, Z_PATCH + T_CU],
    prop_dir='y', exc_dir='z',
    excite=1.0,
    FeedShift=0.4, MeasPlaneShift=1.6,
    Feed_R=50)

# NF2FF box (created AFTER mesh is set up, BEFORE running)
nf2ff = fdtd.CreateNF2FFBox()

sim_path = os.path.join(OUT_DIR, "sim")
print(f"[run] N={N_PATCHES} patch={PATCH_W}x{PATCH_L} CONN={CONN_LEN} "
      f"PROFILE={PROFILE} cells={n_cells:,}")
t0 = time.time()
fdtd.Run(sim_path, verbose=0, cleanup=True)
dt_sim = time.time() - t0
print(f"[run] sim time {dt_sim:.1f} s")

# Verify S11 dip is where we expect
freq = np.linspace(F_START, F_STOP, 401)
port.CalcPort(sim_path, freq)
s11 = port.uf_ref / port.uf_inc
s11_dB = 20.0 * np.log10(np.abs(s11) + 1e-30)
i_tx = int(np.argmin(np.abs(freq - F_TX)))
print(f"[s11] S11 @ {F_TX/1e9:.3f} GHz = {s11_dB[i_tx]:.2f} dB")

# ============================================================================
# Far-field
# ============================================================================
print(f"[ff] Computing far-field at {F_TX/1e9:.3f} GHz")
# CalcNF2FF expects theta/phi in DEGREES (it converts internally).
# NOTE: Prad and Dmax are buggy in this openEMS version (Prad ≈ 0,
# sign-dependent). E_norm pattern shape is valid — that's enough to identify
# broadside vs scanned mode. Plot NORMALIZED pattern (peak at 0 dB).
theta_deg_arr = np.arange(-180, 181, 2.0)
phi_deg_arr   = np.array([0.0, 90.0])
y_array_center = ((N_PATCHES - 1) * PITCH) / 2
center = [0.0, y_array_center, Z_PATCH/2]

t0 = time.time()
nf2ff_res = nf2ff.CalcNF2FF(sim_path, F_TX, theta_deg_arr, phi_deg_arr,
                            center=center, verbose=0)
dt_ff = time.time() - t0
print(f"[ff] far-field calc time {dt_ff:.1f} s")

# Result shape: E_norm[freq] is [n_theta, n_phi]
E_norm = np.array(nf2ff_res.E_norm[0])

# Normalised pattern (peak = 0 dB). Pattern shape is what matters for
# broadside vs scanned identification.
E_max_h = float(np.max(np.abs(E_norm[:, 0])))
E_max_e = float(np.max(np.abs(E_norm[:, 1])))
E_max_global = max(E_max_h, E_max_e)

pat_dB_h_norm = 20*np.log10(np.abs(E_norm[:, 0]) / E_max_global + 1e-30)
pat_dB_e_norm = 20*np.log10(np.abs(E_norm[:, 1]) / E_max_global + 1e-30)

theta_deg = theta_deg_arr
i_bs   = int(np.argmin(np.abs(theta_deg)))
i_pk_h = int(np.argmax(np.abs(E_norm[:, 0])))
i_pk_e = int(np.argmax(np.abs(E_norm[:, 1])))

# -3 dB beamwidth in E-plane (array axis — main lobe)
def beamwidth_3dB(theta_deg, pat_dB, i_peak):
    half_db = pat_dB[i_peak] - 3.0
    # Walk left
    lo = i_peak
    while lo > 0 and pat_dB[lo] > half_db:
        lo -= 1
    # Walk right
    hi = i_peak
    while hi < len(pat_dB) - 1 and pat_dB[hi] > half_db:
        hi += 1
    return theta_deg[hi] - theta_deg[lo]

bw_e = beamwidth_3dB(theta_deg, pat_dB_e_norm, i_pk_e)
bw_h = beamwidth_3dB(theta_deg, pat_dB_h_norm, i_pk_h)

print()
print("=" * 78)
print(f"  Far-field NORMALISED pattern at f = {F_TX/1e9:.3f} GHz")
print("  ── E-plane (φ=90°, along array axis y — array factor lives here) ──")
print(f"  Broadside (θ=0°) level : {pat_dB_e_norm[i_bs]:.2f} dB (rel peak)")
print(f"  Peak direction         : θ = {theta_deg[i_pk_e]:+.1f}° (peak = {pat_dB_e_norm[i_pk_e]:.2f} dB)")
print(f"  -3 dB beamwidth        : {bw_e:.1f}°")
print("  ── H-plane (φ=0°, perpendicular to array) ──")
print(f"  Broadside (θ=0°) level : {pat_dB_h_norm[i_bs]:.2f} dB (rel peak)")
print(f"  Peak direction         : θ = {theta_deg[i_pk_h]:+.1f}° (peak = {pat_dB_h_norm[i_pk_h]:.2f} dB)")
print(f"  -3 dB beamwidth        : {bw_h:.1f}°")
print()
if abs(theta_deg[i_pk_e]) < 5.0:
    print(f"  → BROADSIDE CONFIRMED: E-plane main lobe at θ={theta_deg[i_pk_e]:+.1f}° (within ±5° of normal)")
else:
    print(f"  → SCANNED MODE: E-plane main lobe at θ={theta_deg[i_pk_e]:+.1f}° (NOT broadside)")
print("=" * 78)

# Plot (normalized to peak = 0 dB)
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
for ax, pat_dB, title, peak_deg in [
    (axes[0], pat_dB_h_norm, "H-plane (φ=0°, xz cut, perp. to array)",
        theta_deg[i_pk_h]),
    (axes[1], pat_dB_e_norm, "E-plane (φ=90°, yz cut, ALONG array — array factor)",
        theta_deg[i_pk_e]),
]:
    ax.plot(theta_deg, pat_dB, "b-", lw=1.6)
    ax.axvline(0, color="r", ls=":", lw=0.8, label="broadside (θ=0°)")
    if abs(peak_deg) > 1.0:
        ax.axvline(peak_deg, color="g", ls=":", lw=0.8,
                   label=f"peak at θ={peak_deg:+.1f}°")
    ax.set_xlabel("θ (deg)")
    ax.set_ylabel("Pattern (dB rel peak)")
    ax.set_title(f"{title}\nat {F_TX/1e9:.3f} GHz")
    ax.set_xlim(-180, 180)
    ax.set_ylim(-40, 2)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower center")
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "farfield_cuts.png"), dpi=140)
plt.close(fig)
print(f"[out] {OUT_DIR}/farfield_cuts.png")

# Polar plot (top half-sphere only, θ from -90 to +90)
fig = plt.figure(figsize=(12, 5))
ax_h = fig.add_subplot(121, projection="polar")
ax_e = fig.add_subplot(122, projection="polar")
mask = (theta_deg >= -90) & (theta_deg <= 90)
for ax, pat_dB, title in [
    (ax_h, pat_dB_h_norm, "H-plane (φ=0°)"),
    (ax_e, pat_dB_e_norm, "E-plane (φ=90°, array axis)"),
]:
    th_pol = np.deg2rad(theta_deg[mask])
    g_norm = np.clip(pat_dB[mask] - np.max(pat_dB[mask]), -40, 0)
    ax.plot(th_pol, g_norm, "b-", lw=1.6)
    ax.set_theta_zero_location("N")     # θ=0 at top (broadside)
    ax.set_theta_direction(-1)          # clockwise
    ax.set_rlim(-40, 0)
    ax.set_rticks([-30, -20, -10, 0])
    ax.set_title(f"{title}\n(normalised to peak)")
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "farfield_polar.png"), dpi=140)
plt.close(fig)
print(f"[out] {OUT_DIR}/farfield_polar.png")

with open(os.path.join(OUT_DIR, "farfield.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["theta_deg", "pattern_hplane_dB_norm", "pattern_eplane_dB_norm"])
    for k in range(len(theta_deg)):
        w.writerow([theta_deg[k], pat_dB_h_norm[k], pat_dB_e_norm[k]])
print(f"[out] {OUT_DIR}/farfield.csv")
