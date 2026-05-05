#!/usr/bin/env python3
# probe_fed_aeris10_v3.py
#
# Single-element probe-fed patch antenna sim for AERIS-10 — true 2-layer
# stackup (L1 patch / 0.508 mm RO4350B / L2 ground plane). Probe via through
# ground plane feeds the patch; LumpedPort with R=50 Ω across the substrate
# at the probe location models the coax launch.
#
# Why this topology: aperture-coupled v2 (4-layer Stack_Hybrid) capped at
# ~60 MHz BW because the 0.11 mm L4 backshort acted as a near-short reflector
# — wider BW is fundamentally coupling-limited there. Probe-fed patch on the
# same 0.508 mm patch substrate has no slot bottleneck; physics BW from
# 3.77·(εr-1)/εr²·(W/L)·(h/λ₀) is ~1.6% ≈ 170 MHz at 10.5 GHz.
#
# Patch geometry preserved from the existing 8x16 Gerber (Antenna_16_8.top):
#   W = 7.854 mm  (D10 first dimension; sets X-pitch 14.27 mm in the array)
#   L = 6.56 mm   (tuned at balanced profile to land f_res = 10.51 GHz; old
#                  Gerber's 7.356 mm at 0.102 mm sub gave f_res ~10.6 GHz, the
#                  thicker substrate adds ~1 mm of fringing-edge ΔL each side)
#
# Probe location:
#   y_off = 2.14 mm from -y radiating edge → R_in = 41 Ω, VSWR = 1.18 at 10.5
#   GHz. R_edge fitted from sim ≈ 152 Ω; cos²(π·y_off/L) gives R_in.
#
# Verified design point (PROFILE=balanced, λ/25 mesh, 13 s/run):
#   f_res = 10.510 GHz, S11 = -21.79 dB at 10.5 GHz, Zin = 42.7 + j2.0 Ω
#   -10 dB BW = 180 MHz (10.40 – 10.58 GHz, 1.71%)
#   Compare 4-layer Stack_Hybrid + cap: 60 MHz BW, -19 dB. 3× wider, no cap.
#
# Stackup:
#   L1   Cu 0.035 mm                      ← patch
#   --   RO4350B 0.508 mm  εr=3.48        (patch substrate)
#   L2   Cu 0.035 mm                      ← ground plane (with antipad clearance)
#                                          air below; coax launches up through
#                                          to the probe via from L2 ground.
#
# Run:
#   cd /tmp && DYLD_LIBRARY_PATH=/Users/ganeshpanth/opt/openEMS/lib \
#     PROFILE=balanced  PATCH_L_MM=7.54  FEED_OFFSET_MM=2.5 \
#     /Users/ganeshpanth/radar_venv/bin/python \
#     /Users/ganeshpanth/PLFM_RADAR/5_Simulations/Antenna/probe_fed_aeris10_v3.py
#
# Profiles:
#   sanity    — λ/18 mesh, fast, borderline convergence
#   balanced  — λ/25 mesh, slower, recommended for design verification
#
# Env overrides (all optional):
#   PATCH_W_MM     PATCH_L_MM     FEED_OFFSET_MM (mm from -y radiating edge)
#   FEED_X_MM      (default 0; offset along W-axis, normally 0 for centred feed)

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
# STACKUP (mm) — true 2-layer probe-fed
# ============================================================================
T_CU         = 0.035
H_PATCH_SUB  = 0.508          # RO4350B between L1 patch and L2 ground
EPS_RO4350B  = 3.48
TAN_RO4350B  = 0.0037

# Z layers (L2 ground at z=0, patch on top)
Z_GND   = 0.0
Z_PATCH = Z_GND + T_CU + H_PATCH_SUB
Z_TOP   = Z_PATCH + T_CU

# ============================================================================
# GEOMETRY (mm) — defaults preserve old Gerber's W; L recomputed for 0.508 mm
# ============================================================================
PATCH_W = float(os.environ.get("PATCH_W_MM", "7.854"))
PATCH_L = float(os.environ.get("PATCH_L_MM", "6.56"))

# Probe: feed offset along the L-axis (y), measured from -y radiating edge
# inward. R_in(y_off) = R_edge·cos²(π·y_off/L). y_off=2.14 mm with iter#3 L
# (R_edge≈152 Ω fitted) lands R_in=41 Ω, VSWR=1.18 at 10.5 GHz.
FEED_OFFSET_MM = float(os.environ.get("FEED_OFFSET_MM", "2.14"))
FEED_X_MM      = float(os.environ.get("FEED_X_MM", "0.0"))

# Substrate / ground extents (~λ/2 margin around patch)
GND_X_MARGIN = 14.3
GND_Y_MARGIN = 14.3
GND_X_HALF = PATCH_W/2 + GND_X_MARGIN
GND_Y_HALF = PATCH_L/2 + GND_Y_MARGIN

# Air box (λ/2 above patch, λ/2 below ground)
AIR_ABOVE = 14.3
AIR_BELOW = 14.3
AIR_X_HALF = GND_X_HALF + 8.0
AIR_Y_HALF = GND_Y_HALF + 8.0

OUT_DIR = "/tmp/aeris10_probefed_v3"
os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================================
# Build + run a single FDTD case
# ============================================================================
def run_case(patch_w, patch_l, feed_offset, feed_x, sim_path, profile_cfg, label=""):
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

    # ---- substrate ----
    patch_sub.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND + T_CU],
                      [+GND_X_HALF, +GND_Y_HALF, Z_PATCH], priority=1)

    # ---- L1: patch (centred on origin, L along y, W along x) ----
    copper.AddBox([-patch_w/2, -patch_l/2, Z_PATCH],
                  [+patch_w/2, +patch_l/2, Z_PATCH + T_CU], priority=10)

    # ---- L2: full ground plane ----
    # Single-element sim — antipad clearance around the probe is implicit
    # in the LumpedPort box (FDTD treats the port column as the metal probe
    # and the surrounding cells as substrate). For a multi-element array
    # with real coax launches a physical clearance hole would be added.
    copper.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND],
                  [+GND_X_HALF, +GND_Y_HALF, Z_GND + T_CU], priority=10)

    # ---- mesh ----
    lambda_min_mm = (C0 / F_STOP) * 1000.0
    res = lambda_min_mm / profile_cfg["mesh_lambda_div"]

    # Probe location in patch frame
    feed_y = -patch_l/2 + feed_offset    # offset from -y radiating edge
    feed_x_pos = feed_x

    xlines = [-AIR_X_HALF, -GND_X_HALF, -patch_w/2, feed_x_pos, +patch_w/2,
              +GND_X_HALF, +AIR_X_HALF]
    ylines = [-AIR_Y_HALF, -GND_Y_HALF, -patch_l/2, feed_y, +patch_l/2,
              +GND_Y_HALF, +AIR_Y_HALF]
    # Z mesh: substrate gets ≥6 interior cells for accurate field capture
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

    if os.environ.get("MESH_DEBUG"):
        z_diff = np.diff(zlines)
        print(f"[mesh] x={len(xlines)} y={len(ylines)} z={len(zlines)} cells={n_cells:,}")
        print(f"[mesh] z min/max/avg cell: {z_diff.min()*1e3:.1f}/{z_diff.max()*1e3:.1f}/{z_diff.mean()*1e3:.1f} um")

    # ---- LumpedPort: vertical 50 Ω port across substrate at (feed_x, feed_y) ----
    # The lumped port replaces the coax+source: the 50 Ω resistor sits in the
    # box, the metal column from L2 to L1 is implicit. Excitation is z-direction
    # E-field across the substrate.
    port_start = [feed_x_pos, feed_y, Z_GND + T_CU]
    port_stop  = [feed_x_pos, feed_y, Z_PATCH]
    port = fdtd.AddLumpedPort(1, 50, port_start, port_stop, 'z',
                               excite=1.0, priority=5)

    # ---- run ----
    print(f"[case {label}] patch={patch_w:.2f}x{patch_l:.2f}mm "
          f"feed=({feed_x_pos:.2f},{feed_y:.2f})mm cells={n_cells:,}")
    t0 = time.time()
    fdtd.Run(sim_path, verbose=0, cleanup=True)
    dt = time.time() - t0

    # ---- post-process ----
    freq = np.linspace(F_START, F_STOP, 401)
    port.CalcPort(sim_path, freq)
    s11 = port.uf_ref / port.uf_inc
    s11_dB = 20.0 * np.log10(np.abs(s11) + 1e-30)
    zin    = port.uf_tot / port.if_tot
    vswr   = (1 + np.abs(s11)) / (1 - np.abs(s11) + 1e-30)
    return freq, s11_dB, zin, vswr, dt


def find_resonance(freq, s11_dB, zin=None):
    """Resonance: where R peaks AND Im(Z)=0 nearby. Falls back to min(S11)."""
    f_res, s11_min = None, None
    if zin is not None:
        # Find peak R in the band
        mask = (freq >= 9.0e9) & (freq <= 11.5e9)
        idx_band = np.where(mask)[0]
        if len(idx_band) > 1:
            r_band = np.real(zin[idx_band])
            i_pk = idx_band[int(np.argmax(r_band))]
            # Use the X=0 crossing closest to the R peak
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

    # walk outward to find -10 dB crossings around f_res
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


# ============================================================================
# MAIN
# ============================================================================
sim_path = os.path.join(OUT_DIR, "single")
freq, s11_dB, zin, vswr, dt = run_case(
    PATCH_W, PATCH_L, FEED_OFFSET_MM, FEED_X_MM, sim_path, cfg)
f_res, s11_min, f_lo, f_hi, bw_pct = find_resonance(freq, s11_dB, zin)
i_res = int(np.argmin(np.abs(freq - f_res)))
i_op = int(np.argmin(np.abs(freq - 10.5e9)))

print()
print("=" * 70)
print(f"  Resonance (R peak + Im=0): {f_res/1e9:.3f} GHz   (target 10.5 GHz)")
print(f"  S11 at resonance     : {s11_min:.2f} dB")
print(f"  Zin at resonance     : {zin[i_res].real:.1f} + j{zin[i_res].imag:.1f} Ω")
print("  ── at 10.500 GHz exactly:")
print(f"  S11 @ 10.5GHz        : {s11_dB[i_op]:.2f} dB")
print(f"  Zin @ 10.5GHz        : {zin[i_op].real:.1f} + j{zin[i_op].imag:.1f} Ω")
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
ax.set_title(f"AERIS-10 Probe-Fed Patch v3 — 2-layer 0.508 mm RO4350B "
             f"(W={PATCH_W} L={PATCH_L} y_off={FEED_OFFSET_MM}mm)")
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
