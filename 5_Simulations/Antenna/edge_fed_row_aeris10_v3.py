#!/usr/bin/env python3
# edge_fed_row_aeris10_v3.py
#
# 1xN series-fed row sim for the 2-layer 0.508 mm RO4350B stackup.
# Extends edge_fed_aeris10_v3.py from a single element to a daisy chain.
#
# Topology:
#   PORT (-y board edge) -> 50 Ω feed line (FEED_LEAD_L mm)
#       -> patch_0 (-y edge connected to feed line)
#       -> connecting line (CONN_LEN mm)
#       -> patch_1 (edge-connected) -> connecting line -> ...
#       -> patch_(N-1) (open at +y edge)
#
# The structure is a finite periodic array with a stopband centered at the
# patch self-resonance. The row exhibits an N-mode comb response — N=8 dips
# spanning ~3 GHz with ~0.5 GHz spacing. Operating frequency lands on the
# top-below-stopband mode (deepest dip just below the gap center).
#
# Verified design point (PROFILE=balanced, λ/25 mesh):
#   W=7.854 mm  L=6.95 mm  CONN_LEN=8.15 mm  pitch=15.10 mm
#   INSET_DEPTH=0 (direct edge feed; inset on patch 0 drops Z to ~6 Ω which
#   is unmatchable for N=8 — natural edge-fed Z at array resonance is ~80 Ω,
#   close to 50 Ω so no input matching network is needed)
#   FEED_W=1.16 mm  FEED_LEAD=15.5 mm
#
# Verified result (operating-mode CONN_LEN swept to land dip on TX center):
#   Operating mode at 10.520 GHz: S11 = -18.8 dB, Zin = 76.2 - j9.3 Ω
#   -10 dB BW: 100 MHz (10.470 - 10.570 GHz)
#   Across radar TX 10.510-10.530 GHz: S11 = -17.4 to -18.8 dB (symmetric)
#   At 10.500 GHz (LO): S11 = -15.4 dB
#
# Sensitivity: df/dCONN_LEN ≈ -0.20 GHz/mm (longer CONN → lower op freq).
# To recenter on a different freq:
#   CONN=8.25 → dip at 10.500 GHz   (LO-centered, TX edge falls off)
#   CONN=8.15 → dip at 10.520 GHz   (TX-centered, recommended)
#   CONN=8.20 → dip at 10.510 GHz   (TX-low-edge centered)
#   CONN=8.00 → dip at 10.560 GHz   (above TX band)
#
# CRITICAL difference from edge_fed_aeris10_v3.py: single-element used inset
# (INSET_DEPTH=3.40) to match each patch to 50 Ω; row uses NO inset because
# 8 inset-matched patches in parallel would give Z_in ~ 6 Ω at the row port.
# Direct edge feed with N=8 naturally lands at ~80 Ω (close to 50).
#
# Run:
#   cd /tmp && DYLD_LIBRARY_PATH=/Users/ganeshpanth/opt/openEMS/lib \
#     PROFILE=balanced N_PATCHES=8 \
#     /Users/ganeshpanth/radar_venv/bin/python \
#     /Users/ganeshpanth/PLFM_RADAR/5_Simulations/Antenna/edge_fed_row_aeris10_v3.py
#
# Env overrides:
#   N_PATCHES         (default 8)
#   PATCH_W_MM PATCH_L_MM
#   FEED_W_MM         (50 Ω microstrip on 0.508 mm RO4350B → 1.16 mm)
#   INSET_DEPTH_MM    (0 = edge feed; >0 = inset feed on patch 0 only)
#   INSET_GAP_MM
#   FEED_LEAD_MM      (1·λ_g at f0, line transparent)
#   CONN_LEN_MM       (connecting line between patches)
#   PROFILE           (sanity | balanced; balanced is REQUIRED for accuracy)

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
    "sanity":   {"mesh_lambda_div": 18, "n_timesteps": 100000, "end_dB": -30},
    "balanced": {"mesh_lambda_div": 25, "n_timesteps": 250000, "end_dB": -40},
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
# GEOMETRY
# ============================================================================
N_PATCHES = int(os.environ.get("N_PATCHES", "8"))

PATCH_W = float(os.environ.get("PATCH_W_MM", "7.854"))
PATCH_L = float(os.environ.get("PATCH_L_MM", "6.95"))

FEED_W       = float(os.environ.get("FEED_W_MM", "1.16"))
INSET_DEPTH  = float(os.environ.get("INSET_DEPTH_MM", "0.0"))
INSET_GAP    = float(os.environ.get("INSET_GAP_MM", "0.30"))
FEED_LEAD_L  = float(os.environ.get("FEED_LEAD_MM", "15.5"))

# Connecting line. CONN_LEN=8.15 lands the operating-mode dip at 10.520 GHz
# (radar TX center), giving symmetric -17 to -19 dB across the 10.510-10.530
# chirp band. Pitch = PATCH_L + CONN_LEN = 15.10 mm (vs old Gerber 15.01).
CONN_LEN     = float(os.environ.get("CONN_LEN_MM", "8.15"))

PITCH = PATCH_L + CONN_LEN

# Array spans y from feed-board-edge to last-patch-top (asymmetric layout)
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

OUT_DIR = "/tmp/aeris10_edgefed_row_v3"
os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================================
# Build + run
# ============================================================================
def run_case(profile_cfg, label=""):
    fdtd = openEMS(NrTS=profile_cfg["n_timesteps"],
                   EndCriteria=10**(profile_cfg["end_dB"]/20.0))
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

    # Substrate (single slab spanning the whole row)
    patch_sub.AddBox([-GND_X_HALF, GND_Y_NEG, Z_GND + T_CU],
                     [+GND_X_HALF, GND_Y_POS, Z_PATCH], priority=1)

    # Ground plane (full footprint)
    copper.AddBox([-GND_X_HALF, GND_Y_NEG, Z_GND],
                  [+GND_X_HALF, GND_Y_POS, Z_GND + T_CU], priority=10)

    # ---- Patches ----
    notch_half_w = FEED_W/2 + INSET_GAP

    for i in range(N_PATCHES):
        py0 = i * PITCH - PATCH_L/2     # patch -y edge
        py1 = i * PITCH + PATCH_L/2     # patch +y edge

        if i == 0 and INSET_DEPTH > 0.001:
            # Patch 0: inset feed cut into -y edge
            copper.AddBox([-PATCH_W/2, py0 + INSET_DEPTH, Z_PATCH],
                          [+PATCH_W/2, py1, Z_PATCH + T_CU], priority=10)
            copper.AddBox([-PATCH_W/2, py0, Z_PATCH],
                          [-notch_half_w, py0 + INSET_DEPTH, Z_PATCH + T_CU],
                          priority=10)
            copper.AddBox([+notch_half_w, py0, Z_PATCH],
                          [+PATCH_W/2, py0 + INSET_DEPTH, Z_PATCH + T_CU],
                          priority=10)
        else:
            # Patches 1..N-1 (or patch 0 if INSET_DEPTH=0): solid rectangle
            copper.AddBox([-PATCH_W/2, py0, Z_PATCH],
                          [+PATCH_W/2, py1, Z_PATCH + T_CU], priority=10)

    # ---- Connecting lines (between patch i +y edge and patch i+1 -y edge) ----
    for i in range(N_PATCHES - 1):
        cy0 = i * PITCH + PATCH_L/2
        cy1 = (i + 1) * PITCH - PATCH_L/2
        copper.AddBox([-FEED_W/2, cy0, Z_PATCH],
                      [+FEED_W/2, cy1, Z_PATCH + T_CU], priority=10)

    # ---- Feed line (board edge → patch 0 inset, or edge if INSET_DEPTH=0) ----
    feed_y_start = Y_FEED_BOARD_EDGE
    feed_y_end   = (-PATCH_L/2 + INSET_DEPTH) if INSET_DEPTH > 0.001 else -PATCH_L/2
    copper.AddBox([-FEED_W/2, feed_y_start, Z_PATCH],
                  [+FEED_W/2, feed_y_end, Z_PATCH + T_CU], priority=10)

    # ---- Mesh ----
    lambda_min_mm = (C0 / F_STOP) * 1000.0
    res = lambda_min_mm / profile_cfg["mesh_lambda_div"]

    PORT_LEN = 2.0

    xlines = [-AIR_X_HALF, -GND_X_HALF, -PATCH_W/2, -notch_half_w, -FEED_W/2,
              0, +FEED_W/2, +notch_half_w, +PATCH_W/2, +GND_X_HALF, +AIR_X_HALF]

    ylines = [AIR_Y_NEG, GND_Y_NEG, feed_y_start]
    for i in range(N_PATCHES):
        ylines.append(i * PITCH - PATCH_L/2)
        ylines.append(i * PITCH)
        ylines.append(i * PITCH + PATCH_L/2)
    ylines.append(-PATCH_L/2 + INSET_DEPTH)
    ylines.append(GND_Y_POS)
    ylines.append(AIR_Y_POS)
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

    sim_path = os.path.join(OUT_DIR, label or "row")
    print(f"[case {label}] N={N_PATCHES} patch={PATCH_W:.3f}x{PATCH_L:.3f}mm  "
          f"conn={CONN_LEN:.2f}mm  pitch={PITCH:.2f}mm  cells={n_cells:,}")
    t0 = time.time()
    fdtd.Run(sim_path, verbose=0, cleanup=True)
    dt = time.time() - t0

    freq = np.linspace(F_START, F_STOP, 401)
    port.CalcPort(sim_path, freq)
    s11 = port.uf_ref / port.uf_inc
    s11_dB = 20.0 * np.log10(np.abs(s11) + 1e-30)
    zin    = port.uf_tot / port.if_tot
    vswr   = (1 + np.abs(s11)) / (1 - np.abs(s11) + 1e-30)
    return freq, s11_dB, zin, vswr, dt


def find_resonance(freq, s11_dB):
    """Find the dip nearest 10.5 GHz with S11 < -10 dB (the operating mode of
    the row), plus its contiguous -10 dB BW."""
    below = s11_dB <= -10.0
    if not below.any():
        # No -10 dB region anywhere; fall back to global min in 9.5-11.5
        mask = (freq >= 9.5e9) & (freq <= 11.5e9)
        idx = np.where(mask)[0]
        i_min = idx[int(np.argmin(s11_dB[idx]))]
        return freq[i_min], float(s11_dB[i_min]), 0.0, 0.0, 0.0
    # Find local minima below -10 dB; pick the one nearest 10.5 GHz
    minima = []
    for i in range(2, len(s11_dB)-2):
        if (below[i] and s11_dB[i] < s11_dB[i-1] and s11_dB[i] < s11_dB[i+1]):
            minima.append(i)
    if not minima:
        i_pick = int(np.argmin(np.abs(freq - 10.5e9)))
    else:
        i_pick = min(minima, key=lambda i: abs(freq[i] - 10.5e9))
    f_res = freq[i_pick]
    s11_min = float(s11_dB[i_pick])
    lo = i_pick
    while lo > 0 and below[lo-1]:
        lo -= 1
    hi = i_pick
    while hi < len(below)-1 and below[hi+1]:
        hi += 1
    f_lo, f_hi = freq[lo], freq[hi]
    bw = f_hi - f_lo
    bw_pct = bw / f_res * 100.0
    return f_res, s11_min, f_lo, f_hi, bw_pct


# ============================================================================
# Main
# ============================================================================
freq, s11_dB, zin, vswr, dt = run_case(cfg, label=f"N{N_PATCHES}")
f_res, s11_min, f_lo, f_hi, bw_pct = find_resonance(freq, s11_dB)
i_op  = int(np.argmin(np.abs(freq - 10.5e9)))
i_res = int(np.argmin(np.abs(freq - f_res)))

print()
print("=" * 78)
print(f"  Edge-fed series-fed row N={N_PATCHES} on 0.508 mm RO4350B")
print(f"  W={PATCH_W} L={PATCH_L} inset={INSET_DEPTH}/{INSET_GAP}  "
      f"conn={CONN_LEN}  pitch={PITCH:.2f}")
print(f"  Operating mode (nearest 10.5 GHz): {f_res/1e9:.3f} GHz, {s11_min:.2f} dB")
print(f"  Zin at op mode        : {zin[i_res].real:.1f} + j{zin[i_res].imag:+.1f} Ω")
print("  ── at 10.500 GHz exactly:")
print(f"  S11 @ 10.5GHz         : {s11_dB[i_op]:.2f} dB")
print(f"  Zin @ 10.5GHz         : {zin[i_op].real:.1f} + j{zin[i_op].imag:+.1f} Ω")
print(f"  VSWR @ 10.5GHz        : {vswr[i_op]:.2f}")
print(f"  -10 dB bandwidth      : {(f_hi-f_lo)/1e6:.0f} MHz "
      f"({f_lo/1e9:.3f} - {f_hi/1e9:.3f} GHz, {bw_pct:.2f}%)")
print(f"  Sim time              : {dt:.1f} s")
print("=" * 78)

fig, ax = plt.subplots(figsize=(8.5, 4.5))
ax.plot(freq/1e9, s11_dB, "b-", lw=1.6, label="S11")
ax.axhline(-10, color="r", ls="--", lw=0.8, label="-10 dB")
ax.axvline(f_res/1e9, color="g", ls=":", lw=0.8,
           label=f"min S11 @ {f_res/1e9:.3f} GHz")
if (f_hi-f_lo) > 0:
    ax.axvspan(f_lo/1e9, f_hi/1e9, color="g", alpha=0.10,
               label=f"BW {(f_hi-f_lo)/1e6:.0f} MHz ({bw_pct:.2f}%)")
ax.set_xlabel("Frequency (GHz)")
ax.set_ylabel("S11 (dB)")
ax.set_title(f"AERIS-10 1×{N_PATCHES} Series-Fed Row — 2-layer 0.508 mm RO4350B")
ax.set_xlim(F_START/1e9, F_STOP/1e9)
ax.set_ylim(-40, 0)
ax.grid(True, alpha=0.3)
ax.legend(loc="lower right")
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, f"S11_N{N_PATCHES}.png"), dpi=140)
plt.close(fig)

with open(os.path.join(OUT_DIR, f"S11_data_N{N_PATCHES}.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["freq_Hz", "S11_dB", "Zin_real", "Zin_imag", "VSWR"])
    for k in range(len(freq)):
        w.writerow([freq[k], s11_dB[k], zin[k].real, zin[k].imag, vswr[k]])

print(f"[out] {OUT_DIR}/S11_N{N_PATCHES}.png")
print(f"[out] {OUT_DIR}/S11_data_N{N_PATCHES}.csv")
