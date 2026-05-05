#!/usr/bin/env python3
# production_beams_verify_aeris10.py
#
# Verify the ACTUAL production beamforming tables
# (main.cpp:initializeBeamMatrices() + setCustomBeamPattern16()) steer the
# antenna correctly. setBeamAngle() is dead code; this is the real path.
#
# Production code (main.cpp lines 260-265, 467-498, 565-596):
#   const float phase_differences[31] = { 160, 80, 53.33, ..., -160 };
#   matrix1[bp][el] = degTo7Bit(el * phase_differences[bp])         for bp=0..14
#   matrix2[bp][el] = degTo7Bit(el * phase_differences[bp + 16])    for bp=0..14
#   vector_0[el]   = 0
#
# Then in runRadarPulseSequence():
#   for bp 0..14:
#     setCustomBeamPattern16(matrix1[bp], TX/RX) → fire chirps
#     setCustomBeamPattern16(vector_0,  TX/RX) → fire chirps
#     setCustomBeamPattern16(matrix2[bp], TX/RX) → fire chirps
#
# Concerns to check via sim:
#   * Does the labeled "positive phase difference" produce a peak at a positive
#     angle, or does the wiring/sign convention invert it?
#   * Are matrix1[bp] and matrix2[bp] mirror-image scan angles for symmetric
#     coverage, or asymmetric (bp=0 → +62.7°/-3.4°, bp=14 → +3.4°/-62.7°)?
#   * What's the SLL at large scan angles (62.7° is near boresight loss limit)?

import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

F0       = 10.5e9
C0       = 3.0e8
LAMBDA   = C0 / F0
D_X      = LAMBDA / 2
N_TOTAL  = 16
PHASE_STATES = 128

OUT_DIR = "/tmp/aeris10_array_factor"
os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================================
# Reproduce firmware's degreesTo7BitPhase exactly (main.cpp:349-358)
# ============================================================================
def deg_to_7bit(deg):
    while deg < 0:
        deg += 360.0
    while deg >= 360.0:
        deg -= 360.0
    return int((deg / 360.0) * 128) % 128


# ============================================================================
# Production phase_differences[31] — main.cpp:260
# ============================================================================
PHASE_DIFFERENCES = [
    160.0, 80.0, 53.333, 40.0, 32.0, 26.667, 22.857, 20.0, 17.778, 16.0,
    14.545, 13.333, 12.308, 11.429, 10.667, 0.0,
    -10.667, -11.429, -12.308, -13.333, -14.545, -16.0, -17.778, -20.0,
    -22.857, -26.667, -32.0, -40.0, -53.333, -80.0, -160.0
]


def initializeBeamMatrices_python():
    """Reproduce main.cpp:initializeBeamMatrices() exactly."""
    matrix1 = np.zeros((15, 16), dtype=int)
    matrix2 = np.zeros((15, 16), dtype=int)
    vector_0 = np.zeros(16, dtype=int)
    for bp in range(15):
        phase_diff = PHASE_DIFFERENCES[bp]
        for el in range(16):
            matrix1[bp][el] = deg_to_7bit(el * phase_diff)
        phase_diff = PHASE_DIFFERENCES[bp + 16]
        for el in range(16):
            matrix2[bp][el] = deg_to_7bit(el * phase_diff)
    return matrix1, matrix2, vector_0


# ============================================================================
# Single-row embedded-element pattern (cached from earlier nf2ff sim)
# ============================================================================
def load_single_row_pattern(path="/tmp/aeris10_edgefed_row_nf2ff_v3/farfield.csv"):
    th, h_dB = [], []
    with open(path) as f:
        r = csv.reader(f); next(r)
        for row in r:
            th.append(float(row[0]))
            h_dB.append(float(row[1]))
    return np.array(th), 10**(np.array(h_dB)/20.0)


# ============================================================================
# Array factor at φ=0 H-plane cut (x is the 16-element scanning axis)
# ============================================================================
def array_factor_h(theta_deg_arr, phase_codes):
    k = 2*np.pi/LAMBDA
    th_rad = np.deg2rad(theta_deg_arr)
    phases_rad = np.asarray(phase_codes) * (2*np.pi/PHASE_STATES)
    af = np.zeros(len(th_rad), dtype=complex)
    for n in range(len(phase_codes)):
        af += np.exp(1j*(k*n*D_X*np.sin(th_rad) + phases_rad[n]))
    return np.abs(af)


def total_pattern_dB(theta_deg_arr, phase_codes, h_pat_lin):
    af = array_factor_h(theta_deg_arr, phase_codes)
    pat_lin = af * h_pat_lin
    return 20*np.log10(pat_lin/np.max(pat_lin) + 1e-30), pat_lin


# ============================================================================
# Beam analysis utility
# ============================================================================
def analyse_beam(theta_deg, pat_dB):
    i_pk = int(np.argmax(pat_dB))
    pk_th = theta_deg[i_pk]
    # 3 dB beamwidth
    half = pat_dB[i_pk] - 3.0
    lo, hi = i_pk, i_pk
    while lo > 0 and pat_dB[lo] > half: lo -= 1
    while hi < len(pat_dB) - 1 and pat_dB[hi] > half: hi += 1
    bw3 = theta_deg[hi] - theta_deg[lo]
    # SLL: walk to first nulls bracketing the main lobe
    null_lo, null_hi = lo, hi
    while null_lo > 0 and pat_dB[null_lo - 1] < pat_dB[null_lo]:
        null_lo -= 1
    while null_hi < len(pat_dB) - 1 and pat_dB[null_hi + 1] < pat_dB[null_hi]:
        null_hi += 1
    side_mask = np.ones(len(pat_dB), dtype=bool)
    side_mask[null_lo:null_hi+1] = False
    if side_mask.any():
        i_sll = int(np.argmax(np.where(side_mask, pat_dB, -100)))
        sll_dB = pat_dB[i_sll] - pat_dB[i_pk]
        sll_th = theta_deg[i_sll]
    else:
        sll_dB, sll_th = -np.inf, np.nan
    return pk_th, bw3, sll_dB, sll_th


def expected_angle(phase_diff_deg):
    """Per-element phase shift to physical scan angle.

    Element n at x_n=n·d driven with phase φ_n=n·phase_diff_deg (positive).
    Far-field: E(θ) = Σ exp(j·n·(phase_diff_rad + k·d·sin(θ)))
    Peak when phase_diff_rad + k·d·sin(θ) = 0
    → sin(θ_peak) = -phase_diff_rad/(k·d) = -phase_diff_deg/180  (at d=λ/2)
    """
    sin_th = -phase_diff_deg / 180.0
    if abs(sin_th) > 1:
        return None
    return np.rad2deg(np.arcsin(sin_th))


# ============================================================================
# Main verification
# ============================================================================
def main():
    theta_deg, h_pat_lin = load_single_row_pattern()
    matrix1, matrix2, vector_0 = initializeBeamMatrices_python()

    print("=" * 95)
    print("  PRODUCTION beamforming verification — main.cpp:initializeBeamMatrices()")
    print("  Path: setCustomBeamPattern16(matrix*) → ADAR1000 phase shifters")
    print("=" * 95)
    print(f"{'iter':>4} {'matrix1 (cmd)':>30} {'vector_0 (cmd)':>20} {'matrix2 (cmd)':>30}")
    print(f"{'  ':>4} {'phase_diff:label:actual_pk':>30} {'      0:0°:actual':>20} "
          f"{'phase_diff:label:actual_pk':>30}")
    print("-" * 95)

    rows = []
    for bp in range(15):
        # matrix1
        pd1 = PHASE_DIFFERENCES[bp]
        label1 = expected_angle(pd1)
        pat1, _ = total_pattern_dB(theta_deg, matrix1[bp], h_pat_lin)
        pk1, bw1, sll1, _ = analyse_beam(theta_deg, pat1)
        # vector_0
        pat0, _ = total_pattern_dB(theta_deg, vector_0, h_pat_lin)
        pk0, bw0, sll0, _ = analyse_beam(theta_deg, pat0)
        # matrix2
        pd2 = PHASE_DIFFERENCES[bp + 16]
        label2 = expected_angle(pd2)
        pat2, _ = total_pattern_dB(theta_deg, matrix2[bp], h_pat_lin)
        pk2, bw2, sll2, _ = analyse_beam(theta_deg, pat2)
        l1str = f"{pd1:+7.2f}°" + (f":{label1:+5.1f}°" if label1 is not None else ":n/a")
        l2str = f"{pd2:+7.2f}°" + (f":{label2:+5.1f}°" if label2 is not None else ":n/a")
        print(f"{bp:>4} {l1str+':'+f'{pk1:+5.1f}°':>30} "
              f"{f'pk={pk0:+4.1f}°':>20} "
              f"{l2str+':'+f'{pk2:+5.1f}°':>30}")
        rows.append((bp, pd1, label1, pk1, bw1, sll1,
                          pd2, label2, pk2, bw2, sll2))
    print("=" * 95)
    print(f"  vector_0 broadside check: peak θ={pk0:+.1f}°, BW3={bw0:.1f}°, SLL={sll0:+.1f} dB")
    print()

    # Save CSV
    with open(os.path.join(OUT_DIR, "production_beams.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bp", "m1_phase_diff", "m1_expected", "m1_actual_peak",
                    "m1_bw3", "m1_sll_dB",
                    "m2_phase_diff", "m2_expected", "m2_actual_peak",
                    "m2_bw3", "m2_sll_dB"])
        for r in rows:
            w.writerow(r)
    print(f"[out] {OUT_DIR}/production_beams.csv")

    # Plot all 31 patterns overlaid + a sequence sample
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    ax = axes[0]
    cmap = plt.cm.viridis
    for bp in range(15):
        pat1, _ = total_pattern_dB(theta_deg, matrix1[bp], h_pat_lin)
        pat2, _ = total_pattern_dB(theta_deg, matrix2[bp], h_pat_lin)
        ax.plot(theta_deg, pat1, color=cmap(bp/14.), lw=1.0, alpha=0.7)
        ax.plot(theta_deg, pat2, color=cmap(bp/14.), lw=1.0, alpha=0.7, ls='--')
    pat0, _ = total_pattern_dB(theta_deg, vector_0, h_pat_lin)
    ax.plot(theta_deg, pat0, "k-", lw=2.0, label="vector_0 (broadside)")
    ax.set_xlim(-90, 90)
    ax.set_ylim(-30, 2)
    ax.set_xlabel("θ (deg)")
    ax.set_ylabel("Pattern (dB rel each peak)")
    ax.set_title("All 31 production beams (matrix1 solid, matrix2 dashed) "
                 "+ vector_0 (black)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower center")

    # Iteration-by-iteration scan trajectory
    ax = axes[1]
    iter_pos_pks = [analyse_beam(theta_deg, total_pattern_dB(theta_deg, matrix1[bp], h_pat_lin)[0])[0] for bp in range(15)]
    iter_neg_pks = [analyse_beam(theta_deg, total_pattern_dB(theta_deg, matrix2[bp], h_pat_lin)[0])[0] for bp in range(15)]
    iters = list(range(15))
    ax.plot(iters, iter_pos_pks, "ro-", label="matrix1 actual peak (commanded +pd)")
    ax.plot(iters, iter_neg_pks, "bs-", label="matrix2 actual peak (commanded -pd)")
    ax.axhline(0, color="k", ls=":", lw=0.8, label="broadside (vector_0)")
    ax.set_xlabel("beam_pos iteration (0..14)")
    ax.set_ylabel("Actual beam peak θ (deg)")
    ax.set_title("Production scan sequence — beam peak per iteration")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "production_beams.png"), dpi=140)
    plt.close(fig)
    print(f"[out] {OUT_DIR}/production_beams.png")

    # Summary observations
    print()
    print("OBSERVATIONS")
    print("------------")
    print("1) Broadside vector_0 → peak at θ={:+.1f}°: {} (sanity check)"
          .format(pk0, "OK" if abs(pk0) < 2 else "BROKEN"))
    print()
    # Sign convention
    pd_at_bp7 = PHASE_DIFFERENCES[7]   # +20°
    label_at_bp7 = expected_angle(pd_at_bp7)
    pk_at_bp7 = analyse_beam(theta_deg, total_pattern_dB(theta_deg, matrix1[7], h_pat_lin)[0])[0]
    print("2) Sign convention check (matrix1[bp=7], phase_diff = +20°):")
    print("   Comment in main.cpp says \"positive steering angles\".")
    print("   Math says positive phase_diff steers to NEGATIVE θ.")
    print(f"   Sim peak θ = {pk_at_bp7:+.1f}° (predicted {label_at_bp7:+.1f}°).")
    if pk_at_bp7 * 1.0 < 0:
        print("   → Comment is MISLEADING: matrix1 actually steers to NEGATIVE elevations.")
    else:
        print("   → Sim agrees with comment (positive steering).")
    print()
    # Symmetry / asymmetry
    print("3) Symmetry of matrix1[bp] vs matrix2[bp]:")
    print(f"   At bp=0:  matrix1 → {iter_pos_pks[0]:+5.1f}°,  matrix2 → {iter_neg_pks[0]:+5.1f}°")
    print(f"   At bp=14: matrix1 → {iter_pos_pks[14]:+5.1f}°, matrix2 → {iter_neg_pks[14]:+5.1f}°")
    if abs(abs(iter_pos_pks[0]) - abs(iter_neg_pks[0])) > 5:
        print("   → ASYMMETRIC: matrix1[bp] and matrix2[bp] are NOT mirror images.")
        print("   → Likely indexing intent: matrix2 should use phase_differences[30 - bp]")
        print("     (mirror), not phase_differences[bp + 16] (current).")
    else:
        print("   → Symmetric.")
    print()
    # Coverage
    all_pks = sorted(iter_pos_pks + [pk0] + iter_neg_pks)
    gaps = np.diff(all_pks)
    print(f"4) Angular coverage (sorted unique scan angles, total {len(set(all_pks))}):")
    print(f"   Min: {min(all_pks):+.1f}°, Max: {max(all_pks):+.1f}°")
    print(f"   Largest gap: {max(gaps):.1f}° between {all_pks[int(np.argmax(gaps))]:+.1f}° "
          f"and {all_pks[int(np.argmax(gaps))+1]:+.1f}°")
    print(f"   Smallest gap: {min(g for g in gaps if g > 0.1):.2f}° "
          f"(near broadside — heavily oversampled)")
    print("   1/n distribution → dense near broadside, sparse at large angles.")
    print()
    # SLL at extreme scan
    pk_extreme = max(rows, key=lambda r: abs(r[3] or 0))
    print(f"5) Worst-case SLL at max scan: bp={pk_extreme[0]}, "
          f"matrix1 peak={pk_extreme[3]:+.1f}°, SLL={pk_extreme[5]:+.1f} dB")
    if pk_extreme[5] > -10:
        print("   → SLL exceeds -10 dB at extreme scan. Significant scan loss + "
              "degraded sidelobe rejection (expected at near-grazing scan).")
    print()
    print("6) setBeamAngle() (the 4-broadcast bug we found earlier) is DEAD CODE")
    print("   in production. main.cpp uses initializeBeamMatrices() +")
    print("   setCustomBeamPattern16() exclusively. Fixing setBeamAngle() has zero")
    print("   risk of regressing production behaviour.")


if __name__ == "__main__":
    main()
