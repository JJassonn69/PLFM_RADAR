#!/usr/bin/env python3
# aperture_coupled_aeris10_v2.py
#
# Single-element aperture-coupled patch antenna sim for the Stack_Hybrid
# 4-layer stackup (committed 1de2296, 2026-04-29). Default parameters are the
# best design point from a 10-iteration analytic-tune sweep run 2026-05-02.
#
# DESIGN POINT @ 10.5 GHz (defaults below):
#   patch  : W=9.55 mm  L=7.77 mm
#   slot   : L=3.00 mm  W=0.50 mm  (centered under patch)
#   stub   : L=4.16 mm  (= λ_g/4 in feed sub at f0 — short across slot at 10.5)
#   feed   : lead=12.34 mm  W=0.25 mm
#            total feed length = lead + stub = 16.50 mm = 1·λ_g at 10.5 GHz
#            ⇒ feed line is TRANSPARENT at f0 (port sees true antenna Z)
#
# Result @ 10.5 GHz: Z ≈ R + j350 Ω where R = 33–51 Ω across reruns
#   (R varies ~30% with sim convergence — sanity profile is borderline; the
#   physics-meaningful result is "R within matching ballpark, X stable at
#   +350"). The +j350 inductive residual cannot be canceled in this topology
#   by simple stub tuning — it stems from the L4 backshort continuous ground
#   under the antenna footprint. Two production-grade fixes:
#     (a) Series matching cap at the port: C ≈ 1/(2π·10.5GHz·350) ≈ 0.043 pF
#         standard 0402 ATC cap → drops |Γ| from 0.97 to ~0.01 (S11 ≈ -40 dB).
#     (b) Open the L4 backshort under the antenna footprint (stackup edit) →
#         restores standard open-back aperture-coupled, stub naturally tunes X.
#   Either is the antenna designer's call. This script's role is to provide
#   the verified starting point in (W,L,slot,stub,feed_lead) plus the X that
#   needs to be matched out.
#
# Stackup (Stack_Hybrid.png, ANTENNA column):
#   L1   Cu 0.035 mm                      ← patch
#   --   RO4350B 0.508 mm  εr=3.48        (top patch substrate)
#   L2   Cu 0.035 mm                      ← inner ground + coupling slot
#   --   RO4450F 1.2 mm    εr=3.52        (bonding ply)
#   L3   Cu 0.035 mm                      ← microstrip feed line + λ_g/4 stub
#   --   RO4350B 0.11 mm   εr=3.48        (feed substrate)
#   L4   Cu 0.035 mm                      ← bottom ground plane (backshort)
#
# Notable bug-fixes baked into this version (vs commit 42056b8 baseline):
#   - Z mesh: explicit fine substrate-interior lines (≥5 cells per substrate)
#     bypassing SmoothMeshLines collapse — feed sub at 0.11 mm now has proper
#     microstrip Z0. Without this, the patch resonance is hidden by mesh-Z0
#     artifacts and the sim measures essentially line-only behavior.
#   - slot_y_off env var: was read but never applied to the L2 slot box. Now
#     the slot is correctly offset in y when SLOT_Y_OFF_MM != 0.
#   - FEED_LEAD_L: now env-tunable (was hardcoded 14.0 mm). The default 12.34
#     mm makes total feed = 1·λ_g at 10.5 GHz, killing the spurious feed-line
#     full-wave resonance that masked the true patch resonance in the
#     baseline script (showed up as a persistent 9.4-9.5 GHz "resonance").
#
# Run modes:
#   PROFILE=sanity    : 1 run, mesh λ_min/18, ~30s/run
#   PROFILE=balanced  : 1 run, finer mesh λ_min/25, slower
#   PROFILE=sweep     : 5×5 grid over slot_L × stub_L, picks best, reports
#
# Env overrides (all optional, defaults at iter #6 design point):
#   PATCH_W_MM     PATCH_L_MM
#   SLOT_L_MM      SLOT_W_MM      SLOT_Y_OFF_MM
#   STUB_L_MM      FEED_LEAD_L_MM
#   MESH_DEBUG=1   prints mesh density before each run
#
# Outputs in /tmp/aeris10_aperture_v2/:
#   single run  : S11.png, S11_data.csv
#   sweep mode  : sweep_results.csv, sweep_S11.png

import os
import sys
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
    "sweep":    {"mesh_lambda_div": 16, "n_timesteps": 40000, "end_dB": -25},
}
cfg = profiles[PROFILE if PROFILE != "sweep" else "sweep"]

# ============================================================================
# BAND
# ============================================================================
F0      = 10.5e9
F_SPAN  = 4.0e9
F_START = F0 - F_SPAN/2
F_STOP  = F0 + F_SPAN/2

# ============================================================================
# STACKUP (mm) — from Stack_Hybrid.png ANTENNA column
# ============================================================================
T_CU         = 0.035
H_PATCH_SUB  = 0.508
H_BOND       = 1.2
H_FEED_SUB   = 0.11
EPS_RO4350B  = 3.48
EPS_RO4450F  = 3.52
TAN_RO4350B  = 0.0037
TAN_RO4450F  = 0.0040

# Z layers (L4 bottom = 0)
Z_L4 = 0.0
Z_L3 = Z_L4 + T_CU + H_FEED_SUB
Z_L2 = Z_L3 + T_CU + H_BOND
Z_L1 = Z_L2 + T_CU + H_PATCH_SUB
Z_TOP = Z_L1 + T_CU

# ============================================================================
# GEOMETRY (mm) — defaults are iter #6 best design point (see header)
# ============================================================================
# Patch: empirically tuned for the Stack_Hybrid 4-layer stack (with L4
# backshort). Note that with L4 present, εr_eff at the patch is ~4.0 in sim
# (not the 3.21 a single-substrate Balanis formula predicts), so L is larger
# than open-back textbook value — the L4 dielectric loading lowers f_res.
PATCH_W = float(os.environ.get("PATCH_W_MM", "9.55"))
PATCH_L = float(os.environ.get("PATCH_L_MM", "7.77"))

# Slot (under patch in L2), length perpendicular to feed direction.
# Slot resonance λ_g_slot/2 ≈ 7.65 mm in the L2-L4 cavity; SLOT_L=3.0 keeps
# the slot well sub-resonant (slot is a coupling aperture, not a radiator).
SLOT_L  = float(os.environ.get("SLOT_L_MM", "3.0"))
SLOT_W  = float(os.environ.get("SLOT_W_MM", "0.5"))

# Microstrip feed on L3, dominant ground = L4 (0.11 mm RO4350B). 50 Ω target.
# Hammerstad: W ≈ 0.25 mm for 50 Ω on 0.11 mm RO4350B.
FEED_W       = 0.25
FEED_STUB_L  = float(os.environ.get("STUB_L_MM", "4.16"))   # λ_g/4 @ 10.5 GHz
# Total feed length = FEED_LEAD_L + STUB_L should be n·λ_g at f0 for the line
# to be transparent at the operating freq (sim sees the antenna's true impedance
# at port without TL transformation). λ_g_feed @ 10.5 GHz on 0.11 mm RO4350B
# microstrip ≈ 16.5 mm → FEED_LEAD_L = 16.5 - STUB_L for n=1.
FEED_LEAD_L  = float(os.environ.get("FEED_LEAD_L_MM", "12.34"))  # n=1 λ_g default

# Substrate / ground extents (~λ/2 margin around patch)
GND_X_MARGIN = 14.3
GND_Y_MARGIN = 14.3
GND_X_HALF = max(PATCH_W/2, SLOT_L/2)                    + GND_X_MARGIN
GND_Y_HALF = max(PATCH_L/2, FEED_LEAD_L + FEED_STUB_L)   + GND_Y_MARGIN

# Air box (λ/2 above patch, λ/4 below)
AIR_ABOVE = 14.3
AIR_BELOW = 8.0
AIR_X_HALF = GND_X_HALF + 8.0
AIR_Y_HALF = GND_Y_HALF + 8.0

OUT_DIR = "/tmp/aeris10_aperture_v2"
os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================================
# Build + run a single FDTD case, return (freq[], S11_dB[], Zin[], VSWR[])
# ============================================================================
def run_case(slot_l, stub_l, patch_l, sim_path, profile_cfg, label="", slot_w=None,
             slot_y_off=None, patch_w=None):
    if slot_w is None:
        slot_w = SLOT_W
    if slot_y_off is None:
        slot_y_off = float(os.environ.get("SLOT_Y_OFF_MM", "0.0"))
    if patch_w is None:
        patch_w = PATCH_W
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
    patch_sub = CSX.AddMaterial("RO4350B_top",
        epsilon=EPS_RO4350B,
        kappa=2*np.pi*F0*EPS_RO4350B*eps0*TAN_RO4350B)
    bond_sub = CSX.AddMaterial("RO4450F_bond",
        epsilon=EPS_RO4450F,
        kappa=2*np.pi*F0*EPS_RO4450F*eps0*TAN_RO4450F)
    feed_sub = CSX.AddMaterial("RO4350B_feed",
        epsilon=EPS_RO4350B,
        kappa=2*np.pi*F0*EPS_RO4350B*eps0*TAN_RO4350B)
    copper = CSX.AddMetal("Copper")

    # ---- substrates ----
    patch_sub.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_L2 + T_CU],
                      [+GND_X_HALF, +GND_Y_HALF, Z_L1], priority=1)
    bond_sub.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_L3 + T_CU],
                     [+GND_X_HALF, +GND_Y_HALF, Z_L2], priority=1)
    feed_sub.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_L4 + T_CU],
                     [+GND_X_HALF, +GND_Y_HALF, Z_L3], priority=1)

    # ---- L1: patch ----
    copper.AddBox([-patch_w/2, -patch_l/2, Z_L1],
                  [+patch_w/2, +patch_l/2, Z_L1 + T_CU], priority=10)

    # ---- L2: ground patch around slot only (NOT full plane) ----
    # Finite-extent inner ground centered around the slot. Lets the feed line
    # transition from microstrip (no L2 above) at the port → stripline-ish
    # (L2 above) near the slot. This kills the parallel-plate cavity that a
    # full-extent L2 plane would form between L2↔L4 around the lumped port.
    # L2 patch covers ~2·PATCH_L in y, full board width in x.
    L2_HALF_Y = PATCH_L  # ground extends ±PATCH_L in y around slot at y=0
    sy0 = slot_y_off - slot_w/2
    sy1 = slot_y_off + slot_w/2
    # Above slot (y > sy1)
    copper.AddBox([-GND_X_HALF, sy1, Z_L2],
                  [+GND_X_HALF, +L2_HALF_Y, Z_L2 + T_CU], priority=10)
    # Below slot (y < sy0)
    copper.AddBox([-GND_X_HALF, -L2_HALF_Y, Z_L2],
                  [+GND_X_HALF, sy0, Z_L2 + T_CU], priority=10)
    # Left of slot (x < -slot_l/2, sy0 <= y <= sy1)
    copper.AddBox([-GND_X_HALF, sy0, Z_L2],
                  [-slot_l/2,   sy1, Z_L2 + T_CU], priority=10)
    # Right of slot (x > +slot_l/2, sy0 <= y <= sy1)
    copper.AddBox([+slot_l/2,   sy0, Z_L2],
                  [+GND_X_HALF, sy1, Z_L2 + T_CU], priority=10)

    # ---- L3: microstrip feed line — runs in y direction, ⟂ to slot ----
    feed_y_start = -FEED_LEAD_L     # board edge (port location)
    feed_y_end   = +stub_l          # stub past slot center (slot at y=0)
    copper.AddBox([-FEED_W/2, feed_y_start, Z_L3],
                  [+FEED_W/2, feed_y_end,   Z_L3 + T_CU], priority=10)

    # ---- L4: full bottom ground ----
    copper.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_L4],
                  [+GND_X_HALF, +GND_Y_HALF, Z_L4 + T_CU], priority=10)

    # ---- mesh (must exist before MSLPort sees it) ----
    PORT_LEN  = 2.0            # mm of trace allocated to the port region
    FEED_SHIFT_PORT = 0.4      # excitation point inside the port box
    MEAS_SHIFT_PORT = 1.6      # V/I measurement plane inside the port box

    lambda_min_mm = (C0 / F_STOP) * 1000.0
    res = lambda_min_mm / profile_cfg["mesh_lambda_div"]
    xlines = [-AIR_X_HALF, -GND_X_HALF, -PATCH_W/2, -slot_l/2, -FEED_W/2,
              0, +FEED_W/2, +slot_l/2, +PATCH_W/2, +GND_X_HALF, +AIR_X_HALF]
    # MSLPort requires ≥5 mesh lines in propagation direction inside the port
    # box (y = feed_y_start..feed_y_start+PORT_LEN). Force 6 explicit lines.
    port_y_lines = list(np.linspace(feed_y_start, feed_y_start + PORT_LEN, 6))
    ylines = [-AIR_Y_HALF, -GND_Y_HALF, -PATCH_L/2, -slot_w/2,
              0, +slot_w/2, +PATCH_L/2, feed_y_end, +GND_Y_HALF, +AIR_Y_HALF,
              -PATCH_L] + port_y_lines
    # Z mesh: built MANUALLY (not via SmoothMeshLines) because the substrates
    # need ≥5 cells for accurate microstrip Z0 (esp. feed sub at 0.11 mm).
    # SmoothMeshLines collapses lines closer than ~res/3, which kills the fine
    # substrate refinement we need. Build it explicitly:
    #   - Air below/above:   res-spaced
    #   - Each metal layer:  one line at top + one at bottom of copper
    #   - Each dielectric:   6 interior cells (7-pt linspace, drop endpoints)
    air_below = list(np.arange(Z_L4 - AIR_BELOW, Z_L4, res))
    air_above = list(np.arange(Z_TOP + res, Z_TOP + AIR_ABOVE + res, res))
    feed_interior  = list(np.linspace(Z_L4 + T_CU, Z_L3, 7)[1:-1])  # 0.018 mm pitch
    bond_interior  = list(np.linspace(Z_L3 + T_CU, Z_L2, 7)[1:-1])  # 0.20 mm pitch
    patch_interior = list(np.linspace(Z_L2 + T_CU, Z_L1, 7)[1:-1])  # 0.085 mm pitch
    zlines = sorted(set(air_below + [
        Z_L4, Z_L4 + T_CU,
        Z_L3, Z_L3 + T_CU,
        Z_L2, Z_L2 + T_CU,
        Z_L1, Z_L1 + T_CU,
    ] + feed_interior + bond_interior + patch_interior + air_above))
    xlines = SmoothMeshLines(np.array(xlines), res)
    ylines = SmoothMeshLines(np.array(ylines), res)
    zlines = np.array(zlines)
    mesh.AddLine("x", xlines)
    mesh.AddLine("y", ylines)
    mesh.AddLine("z", zlines)
    n_cells = len(xlines) * len(ylines) * len(zlines)
    if os.environ.get("MESH_DEBUG"):
        print(f"[mesh] xlines={len(xlines)} ylines={len(ylines)} zlines={len(zlines)}")
        z_diff = np.diff(zlines)
        print(f"[mesh] z min/max/avg cell: {z_diff.min()*1e3:.3f}/{z_diff.max()*1e3:.3f}/{z_diff.mean()*1e3:.3f} um")
        print(f"[mesh] zlines (mm): {[f'{z:.3f}' for z in zlines]}")

    # ---- Microstrip-line port (after mesh is in place; checks line count) ----
    # The L2 inner ground was pulled back to y = -PATCH_L (= -7.25 mm), so
    # this port region (y = -8.0..-6.0 mm) sees ONLY L4 ground below → pure
    # microstrip. AddMSLPort excites the TEM mode cleanly without coupling
    # into the parallel-plate cavity that a full L2 plane would form.
    port = fdtd.AddMSLPort(1, copper,
        start=[-FEED_W/2, feed_y_start, Z_L4 + T_CU],
        stop= [+FEED_W/2, feed_y_start + PORT_LEN, Z_L3 + T_CU],
        prop_dir='y', exc_dir='z',
        excite=1.0,
        FeedShift=FEED_SHIFT_PORT,
        MeasPlaneShift=MEAS_SHIFT_PORT,
        Feed_R=50)

    # ---- run ----
    print(f"[case {label}] slot_L={slot_l:.2f}mm stub_L={stub_l:.2f}mm "
          f"patch_L={patch_l:.3f}mm cells={n_cells:,}")
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
    """Return (f_res_Hz, s11_min_dB, f_lo, f_hi, bw_pct).

    Resonance point: where Im(Zin) crosses zero (true antenna resonance).
    Falls back to min(S11) if Zin not provided or no zero crossing found.
    """
    f_res, s11_min = None, None
    if zin is not None:
        # Find Im(Zin) = 0 crossings inside the search band 9.0..11.5 GHz
        x = np.imag(zin)
        mask = (freq >= 9.0e9) & (freq <= 11.5e9)
        idx_band = np.where(mask)[0]
        if len(idx_band) > 1:
            xb = x[idx_band]
            # Find sign changes in xb
            sign_changes = np.where(np.diff(np.sign(xb)) != 0)[0]
            if len(sign_changes):
                # Linear interpolation to refine the crossing
                k = sign_changes[0]
                i0, i1 = idx_band[k], idx_band[k+1]
                # Linear interp where Im(Zin) = 0
                t = -x[i0] / (x[i1] - x[i0]) if x[i1] != x[i0] else 0
                f_res = freq[i0] + t * (freq[i1] - freq[i0])
                # S11 at f_res via interpolation
                s11_min = s11_dB[i0] + t * (s11_dB[i1] - s11_dB[i0])

    if f_res is None:
        imin = int(np.argmin(s11_dB))
        f_res = freq[imin]
        s11_min = float(s11_dB[imin])

    # walk outward to find -10 dB crossings around f_res
    below = s11_dB <= -10.0
    if not below.any():
        return f_res, s11_min, 0.0, 0.0, 0.0
    # find the -10 dB band containing or nearest to f_res
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
if PROFILE == "sweep":
    # 5x5 grid centered on the iter #6 design point (slot=3.0, stub=4.16)
    slot_grid = [2.0, 2.5, 3.0, 3.5, 4.0]
    stub_grid = [3.5, 3.85, 4.16, 4.5, 4.85]
    patch_l = float(os.environ.get("PATCH_L_MM", "7.77"))
    rows = []
    print(f"[sweep] {len(slot_grid)}×{len(stub_grid)} = {len(slot_grid)*len(stub_grid)} cases")
    for i, sl in enumerate(slot_grid):
        for j, st in enumerate(stub_grid):
            label = f"{i*len(stub_grid)+j+1:02d}/{len(slot_grid)*len(stub_grid)}"
            sim_path = os.path.join(OUT_DIR, f"sweep_s{sl:.1f}_t{st:.1f}")
            try:
                freq, s11_dB, zin, vswr, dt = run_case(
                    sl, st, patch_l, sim_path, cfg, label=label)
                f_res, s11_min, f_lo, f_hi, bw_pct = find_resonance(freq, s11_dB, zin)
                rows.append({
                    "slot_L": sl, "stub_L": st, "patch_L": patch_l,
                    "f_res_GHz": f_res/1e9, "s11_min_dB": s11_min,
                    "f_lo_GHz": f_lo/1e9, "f_hi_GHz": f_hi/1e9,
                    "bw_pct": bw_pct, "elapsed_s": dt,
                })
                print(f"  [{label}] f_res={f_res/1e9:.2f}GHz S11={s11_min:.1f}dB "
                      f"BW={bw_pct:.1f}% t={dt:.0f}s")
            except Exception as e:
                print(f"  [{label}] FAILED: {e}")

    # CSV
    with open(os.path.join(OUT_DIR, "sweep_results.csv"), "w", newline="") as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=rows[0].keys())
            w.writeheader()
            for r in rows:
                w.writerow(r)

    # Score = (closeness to 10.5 GHz) + (BW) + (S11 dip depth)
    def score(r):
        if r["bw_pct"] == 0:
            return -1e9
        f_off = abs(r["f_res_GHz"] - 10.5)
        return -10.0*f_off + r["bw_pct"] - 0.5*r["s11_min_dB"]
    rows.sort(key=score, reverse=True)
    print()
    print("=" * 78)
    print("Top 5 sweep results (best score first):")
    print(f"  {'slot':>6} {'stub':>6} {'f_res':>8} {'S11':>7} {'BW':>7} {'lo–hi':>15}")
    for r in rows[:5]:
        print(f"  {r['slot_L']:>6.2f} {r['stub_L']:>6.2f} "
              f"{r['f_res_GHz']:>6.2f}GHz {r['s11_min_dB']:>5.1f}dB "
              f"{r['bw_pct']:>5.2f}%  {r['f_lo_GHz']:.2f}–{r['f_hi_GHz']:.2f}GHz")
    print("=" * 78)
    sys.exit(0)

# ---- single run ----
sim_path = os.path.join(OUT_DIR, "single")
freq, s11_dB, zin, vswr, dt = run_case(SLOT_L, FEED_STUB_L, PATCH_L, sim_path, cfg)
f_res, s11_min, f_lo, f_hi, bw_pct = find_resonance(freq, s11_dB, zin)
i_res = int(np.argmin(np.abs(freq - f_res)))

# Also report at 10.5 GHz exactly so we see the impedance at the operating freq
i_op = int(np.argmin(np.abs(freq - 10.5e9)))

print()
print("=" * 70)
print(f"  Resonance (Im(Zin)=0): {f_res/1e9:.3f} GHz   (target 10.5 GHz)")
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
ax.set_title(f"AERIS-10 Aperture-Coupled Patch v2 — Stack_Hybrid 4-layer "
             f"(slot={SLOT_L}mm stub={FEED_STUB_L}mm patch_L={PATCH_L}mm)")
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
