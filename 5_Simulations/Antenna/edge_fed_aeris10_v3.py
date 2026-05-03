#!/usr/bin/env python3
# edge_fed_aeris10_v3.py
#
# Single-element edge-fed (inset-feed microstrip) patch antenna sim for the
# 2-layer thicker-substrate option — preserves the old Gerber's series-fed-row
# topology but thickens the patch substrate from 0.102 mm to 0.508 mm RO4350B.
#
# Goal: validate that edge-fed on 0.508 mm gives reasonable BW (target >100 MHz,
# vs probe-fed v3's 180 MHz on the same substrate, vs old design's ~30 MHz on
# 0.102 mm). If BW is good, the 1x8 series-fed row will work; if it's poor,
# the on-top-layer feed traces are coupling to the patch and we need a
# different topology.
#
# Stackup (true 2-layer):
#   L1   Cu 0.035 mm                    ← patch + edge-fed inset + feed line
#   --   RO4350B 0.508 mm  εr=3.48      (patch substrate, sets BW)
#   L2   Cu 0.035 mm                    ← ground plane
#
# Verified design point (PROFILE=balanced, λ/25 mesh):
#   W=7.854 mm  L=6.95 mm  inset_depth=3.40 mm  inset_gap=0.30 mm
#   feed_W=1.16 mm  feed_lead=15.5 mm (1·λ_g at f0, line transparent)
#   f_res = 10.509 GHz, S11 @ 10.5 = -18.5 dB, VSWR = 1.27
#   -10 dB BW = 180 MHz (10.41-10.59 GHz)  ← same as probe-fed v3
#
# Run:
#   cd /tmp && DYLD_LIBRARY_PATH=/Users/ganeshpanth/opt/openEMS/lib \
#     PROFILE=balanced \
#     /Users/ganeshpanth/radar_venv/bin/python \
#     /Users/ganeshpanth/PLFM_RADAR/5_Simulations/Antenna/edge_fed_aeris10_v3.py
#
# Env overrides:
#   PATCH_W_MM PATCH_L_MM
#   FEED_W_MM      (50 Ω microstrip on 0.508 mm RO4350B → 1.16 mm)
#   INSET_DEPTH_MM (inset notch depth from radiating edge)
#   INSET_GAP_MM   (gap between feed line and patch metal in the inset region)
#   FEED_LEAD_MM   (length of feed line before reaching patch edge)

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
# GEOMETRY
# ============================================================================
PATCH_W = float(os.environ.get("PATCH_W_MM", "7.854"))
PATCH_L = float(os.environ.get("PATCH_L_MM", "6.95"))

# 50 Ω microstrip feed on 0.508 mm RO4350B (Hammerstad: W ≈ 1.16 mm)
FEED_W       = float(os.environ.get("FEED_W_MM", "1.16"))
INSET_DEPTH  = float(os.environ.get("INSET_DEPTH_MM", "3.40"))   # ~49% of L for 50 Ω
INSET_GAP    = float(os.environ.get("INSET_GAP_MM", "0.30"))     # clearance both sides of feed line in the inset
FEED_LEAD_L  = float(os.environ.get("FEED_LEAD_MM", "15.5"))     # 1·λ_g at 10.5 GHz: line transparent at f0

GND_X_MARGIN = 14.3
GND_Y_MARGIN = 14.3
GND_X_HALF = max(PATCH_W/2, FEED_W/2 + INSET_GAP) + GND_X_MARGIN
GND_Y_HALF = (PATCH_L/2 + FEED_LEAD_L) + GND_Y_MARGIN

AIR_ABOVE = 14.3
AIR_BELOW = 14.3
AIR_X_HALF = GND_X_HALF + 8.0
AIR_Y_HALF = GND_Y_HALF + 8.0

OUT_DIR = "/tmp/aeris10_edgefed_v3"
os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================================
# Build + run
# ============================================================================
def run_case(patch_w, patch_l, feed_w, inset_depth, inset_gap, feed_lead,
             sim_path, profile_cfg, label=""):
    fdtd = openEMS(NrTS=profile_cfg["n_timesteps"],
                   EndCriteria=10**(profile_cfg["end_dB"]/20.0))
    fdtd.SetGaussExcite(F0, F_SPAN/2.0)
    fdtd.SetBoundaryCond(["MUR"]*6)

    CSX = ContinuousStructure()
    fdtd.SetCSX(CSX)
    mesh = CSX.GetGrid()
    mesh.SetDeltaUnit(1e-3)

    # Materials
    eps0 = 8.854e-12
    patch_sub = CSX.AddMaterial("RO4350B",
        epsilon=EPS_RO4350B,
        kappa=2*np.pi*F0*EPS_RO4350B*eps0*TAN_RO4350B)
    copper = CSX.AddMetal("Copper")

    # Substrate
    patch_sub.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND + T_CU],
                      [+GND_X_HALF, +GND_Y_HALF, Z_PATCH], priority=1)

    # Patch geometry — patch is centred at origin, L is along y, W along x.
    # Inset feed: notch cut into the -y radiating edge for the feed line.
    # The notch is feed_w + 2*inset_gap wide × inset_depth tall.
    notch_half_w = feed_w/2 + inset_gap
    px0, px1 = -patch_w/2, +patch_w/2
    py0, py1 = -patch_l/2, +patch_l/2

    # Patch in 3 boxes around the notch (which is at -y edge, centred on x=0):
    # Box A: full width above the notch (y from py0+inset_depth to py1)
    copper.AddBox([px0, py0 + inset_depth, Z_PATCH],
                  [px1, py1, Z_PATCH + T_CU], priority=10)
    # Box B: left of notch, between patch -y edge and notch top
    copper.AddBox([px0, py0, Z_PATCH],
                  [-notch_half_w, py0 + inset_depth, Z_PATCH + T_CU],
                  priority=10)
    # Box C: right of notch, between patch -y edge and notch top
    copper.AddBox([+notch_half_w, py0, Z_PATCH],
                  [px1, py0 + inset_depth, Z_PATCH + T_CU], priority=10)

    # Feed line: 50 Ω microstrip from board edge (at -y) up into the inset
    # notch. Feed line top reaches inside the notch by `inset_depth` so the
    # feed-trace tip touches the patch at the inset bottom.
    feed_y_start = -GND_Y_HALF + GND_Y_MARGIN          # at edge of ground
    feed_y_end   = py0 + inset_depth                   # tip inside inset
    copper.AddBox([-feed_w/2, feed_y_start, Z_PATCH],
                  [+feed_w/2, feed_y_end, Z_PATCH + T_CU], priority=10)

    # Ground plane (full)
    copper.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND],
                  [+GND_X_HALF, +GND_Y_HALF, Z_GND + T_CU], priority=10)

    # Mesh
    lambda_min_mm = (C0 / F_STOP) * 1000.0
    res = lambda_min_mm / profile_cfg["mesh_lambda_div"]

    PORT_LEN = 2.0
    xlines = [-AIR_X_HALF, -GND_X_HALF, px0, -notch_half_w, -feed_w/2, 0,
              +feed_w/2, +notch_half_w, px1, +GND_X_HALF, +AIR_X_HALF]
    port_y_lines = list(np.linspace(feed_y_start, feed_y_start + PORT_LEN, 6))
    ylines = [-AIR_Y_HALF, -GND_Y_HALF, py0, py0 + inset_depth, 0, py1,
              +GND_Y_HALF, +AIR_Y_HALF] + port_y_lines

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

    # MSLPort: at -y edge of board, on the feed line. 50 Ω microstrip,
    # propagation along +y, excitation in z.
    port = fdtd.AddMSLPort(1, copper,
        start=[-feed_w/2, feed_y_start, Z_GND + T_CU],
        stop= [+feed_w/2, feed_y_start + PORT_LEN, Z_PATCH + T_CU],
        prop_dir='y', exc_dir='z',
        excite=1.0,
        FeedShift=0.4, MeasPlaneShift=1.6,
        Feed_R=50)

    print(f"[case {label}] patch={patch_w:.2f}x{patch_l:.2f}mm  "
          f"feed={feed_w:.2f}mm  inset={inset_depth:.2f}/{inset_gap:.2f}mm  "
          f"cells={n_cells:,}")
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


def find_resonance(freq, s11_dB, zin=None):
    f_res, s11_min = None, None
    if zin is not None:
        mask = (freq >= 9.0e9) & (freq <= 11.5e9)
        idx_band = np.where(mask)[0]
        if len(idx_band) > 1:
            r_band = np.real(zin[idx_band])
            i_pk = idx_band[int(np.argmax(r_band))]
            x = np.imag(zin)
            sign = np.sign(x)
            crossings = np.where(np.diff(sign) != 0)[0]
            crossings_in_band = [c for c in crossings if mask[c]]
            if crossings_in_band:
                k = min(crossings_in_band, key=lambda c: abs(c - i_pk))
                t = -x[k] / (x[k+1] - x[k]) if x[k+1] != x[k] else 0
                f_res = freq[k] + t * (freq[k+1] - freq[k])
                s11_min = s11_dB[k] + t * (s11_dB[k+1] - s11_dB[k])
    if f_res is None:
        imin = int(np.argmin(s11_dB))
        f_res = freq[imin]
        s11_min = float(s11_dB[imin])
    below = s11_dB <= -10.0
    if not below.any():
        return f_res, s11_min, 0.0, 0.0, 0.0
    i_f = int(np.argmin(np.abs(freq - f_res)))
    if not below[i_f]:
        return f_res, s11_min, 0.0, 0.0, 0.0
    lo = i_f
    while lo > 0 and below[lo-1]:
        lo -= 1
    hi = i_f
    while hi < len(below)-1 and below[hi+1]:
        hi += 1
    f_lo, f_hi = freq[lo], freq[hi]
    bw = f_hi - f_lo
    bw_pct = bw / f_res * 100.0
    return f_res, s11_min, f_lo, f_hi, bw_pct


# Main
sim_path = os.path.join(OUT_DIR, "single")
freq, s11_dB, zin, vswr, dt = run_case(
    PATCH_W, PATCH_L, FEED_W, INSET_DEPTH, INSET_GAP, FEED_LEAD_L,
    sim_path, cfg)
f_res, s11_min, f_lo, f_hi, bw_pct = find_resonance(freq, s11_dB, zin)
i_op = int(np.argmin(np.abs(freq - 10.5e9)))
i_res = int(np.argmin(np.abs(freq - f_res)))

print()
print("=" * 70)
print(f"  Edge-fed (inset) on 0.508 mm RO4350B (W={PATCH_W} L={PATCH_L} inset={INSET_DEPTH})")
print(f"  Resonance (R peak + Im=0): {f_res/1e9:.3f} GHz   (target 10.5 GHz)")
print(f"  S11 at resonance     : {s11_min:.2f} dB")
print(f"  Zin at resonance     : {zin[i_res].real:.1f} + j{zin[i_res].imag:+.1f} Ω")
print(f"  ── at 10.500 GHz exactly:")
print(f"  S11 @ 10.5GHz        : {s11_dB[i_op]:.2f} dB")
print(f"  Zin @ 10.5GHz        : {zin[i_op].real:.1f} + j{zin[i_op].imag:+.1f} Ω")
print(f"  VSWR @ 10.5GHz       : {vswr[i_op]:.2f}")
print(f"  -10 dB bandwidth     : {(f_hi-f_lo)/1e6:.0f} MHz "
      f"({f_lo/1e9:.3f} – {f_hi/1e9:.3f} GHz, {bw_pct:.2f}%)")
print(f"  Sim time             : {dt:.1f} s")
print("=" * 70)

fig, ax = plt.subplots(figsize=(8.5, 4.5))
ax.plot(freq/1e9, s11_dB, "b-", lw=1.6, label="S11")
ax.axhline(-10, color="r", ls="--", lw=0.8, label="-10 dB")
ax.axvline(f_res/1e9, color="g", ls=":", lw=0.8,
           label=f"resonance {f_res/1e9:.3f} GHz")
if (f_hi-f_lo) > 0:
    ax.axvspan(f_lo/1e9, f_hi/1e9, color="g", alpha=0.10,
               label=f"BW {(f_hi-f_lo)/1e6:.0f} MHz ({bw_pct:.2f}%)")
ax.set_xlabel("Frequency (GHz)")
ax.set_ylabel("S11 (dB)")
ax.set_title(f"AERIS-10 Edge-Fed (inset) — 2-layer 0.508 mm RO4350B")
ax.set_xlim(F_START/1e9, F_STOP/1e9)
ax.set_ylim(-40, 0)
ax.grid(True, alpha=0.3)
ax.legend(loc="lower right")
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "S11.png"), dpi=140)
plt.close(fig)

with open(os.path.join(OUT_DIR, "S11_data.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["freq_Hz", "S11_dB", "Zin_real", "Zin_imag", "VSWR"])
    for k in range(len(freq)):
        w.writerow([freq[k], s11_dB[k], zin[k].real, zin[k].imag, vswr[k]])

print(f"[out] {OUT_DIR}/S11.png")
print(f"[out] {OUT_DIR}/S11_data.csv")
