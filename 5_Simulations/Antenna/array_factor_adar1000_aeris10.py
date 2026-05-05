#!/usr/bin/env python3
# array_factor_adar1000_aeris10.py
#
# Phased-array beam-forming verification using the ADAR1000 firmware's actual
# phase-shifter codes. Combines:
#   * The single-row 1x8 series-fed embedded element pattern (from
#     edge_fed_row_nf2ff_aeris10_v3.py at 10.520 GHz, cached) for the y-axis
#     row pattern.
#   * A 16-element x-axis array factor at d = λ/2 = 14.286 mm pitch (matches
#     the firmware's `element_spacing = wavelength/2` constant).
#   * The firmware phase computation EXACTLY (ADAR1000_Manager.cpp:714-729):
#     `calculatePhaseSettings()` only fills 4 phases (one per chip channel),
#     and the broadcast loop applies the same 4-phase pattern to all 4 chips.
#   * The 7-bit (128-state, 2.8125 deg/step) ADAR1000 phase quantization.
#
# Verifications a radar engineer would run at this stage:
#   1. Beam steering accuracy (commanded vs simulated peak angle).
#   2. Sidelobe and grating-lobe levels at multiple scan angles.
#   3. Scan loss (peak gain vs scan angle).
#   4. Null steering: place a deep null at a chosen angle.
#   5. Compare FIRMWARE behaviour vs CORRECT 16-element progressive phasing
#      to expose the per-chip-broadcast bug.
#
# Inputs:
#   /tmp/aeris10_edgefed_row_nf2ff_v3/farfield.csv   (cached single-row pattern)
#
# Outputs:
#   /tmp/aeris10_array_factor/scan_*.png             (1D cuts at scan angles)
#   /tmp/aeris10_array_factor/scan_loss.png          (peak gain vs scan)
#   /tmp/aeris10_array_factor/null_steering.png
#   /tmp/aeris10_array_factor/firmware_vs_correct.png

import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ============================================================================
# Constants (match firmware ADAR1000_Manager.cpp:714-729 exactly)
# ============================================================================
F0       = 10.5e9
C0       = 3.0e8
LAMBDA   = C0 / F0                  # 28.5714 mm
D_X      = LAMBDA / 2               # 14.2857 mm — element_spacing in firmware
N_TOTAL  = 16                       # 4 chips × 4 channels
N_PER_CHIP = 4
N_CHIPS  = 4

# ADAR1000 7-bit phase resolution
PHASE_STATES = 128
PHASE_LSB_DEG = 360.0 / PHASE_STATES   # 2.8125 deg/code

OUT_DIR = "/tmp/aeris10_array_factor"
os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================================
# Embedded element pattern (single-row 1x8 at 10.520 GHz)
# ============================================================================
def load_single_row_pattern(path="/tmp/aeris10_edgefed_row_nf2ff_v3/farfield.csv"):
    """Returns theta_deg, h_pat_lin, e_pat_lin (linear |E|, peak normalised to 1)."""
    th, h_dB, e_dB = [], [], []
    with open(path) as f:
        r = csv.reader(f); next(r)
        for row in r:
            th.append(float(row[0]))
            h_dB.append(float(row[1]))
            e_dB.append(float(row[2]))
    th = np.array(th); h_dB = np.array(h_dB); e_dB = np.array(e_dB)
    # CSV stores normalised dB rel peak. Convert to linear amplitude.
    h_lin = 10 ** (h_dB / 20.0)
    e_lin = 10 ** (e_dB / 20.0)
    return th, h_lin, e_lin


# ============================================================================
# Phase code generators
# ============================================================================
def firmware_phase_codes(angle_deg):
    """Replicate ADAR1000Manager::calculatePhaseSettings() + the broadcast
    loop in setBeamAngle(): 4 phases computed, same pattern applied to all
    4 chips. Returns 16 ADAR1000 phase codes (uint8 values 0..127)."""
    angle_rad = np.deg2rad(angle_deg)
    # firmware: phase_shift = 2π·d·sin(θ)/λ
    phase_shift = (2 * np.pi * D_X * np.sin(angle_rad)) / LAMBDA
    codes_4 = np.zeros(N_PER_CHIP, dtype=int)
    for i in range(N_PER_CHIP):
        ph = (i * phase_shift) % (2 * np.pi)
        codes_4[i] = int(round(ph / (2 * np.pi) * PHASE_STATES)) % PHASE_STATES
    # Broadcast: same 4-element pattern repeated to all 4 chips
    return np.tile(codes_4, N_CHIPS)


def correct_phase_codes(angle_deg):
    """Proper 16-element progressive phase shift (what the firmware should do)."""
    angle_rad = np.deg2rad(angle_deg)
    phase_shift = (2 * np.pi * D_X * np.sin(angle_rad)) / LAMBDA
    codes = np.zeros(N_TOTAL, dtype=int)
    for n in range(N_TOTAL):
        ph = (n * phase_shift) % (2 * np.pi)
        codes[n] = int(round(ph / (2 * np.pi) * PHASE_STATES)) % PHASE_STATES
    return codes


def codes_to_radians(codes):
    return np.asarray(codes) * (2 * np.pi / PHASE_STATES)


# ============================================================================
# Array factor at the φ=0 (H-plane) cut, for an arbitrary 16-element phase set
# ============================================================================
def array_factor_hplane(theta_deg_arr, phase_codes, amplitudes=None):
    """Compute |AF(θ)| at φ=0 (H-plane). x_n = n*d. Element 0 at x=0, element
    15 at x=15·d. AF(θ) = Σ a_n · exp(j·k·x_n·sin(θ)) · exp(j·φ_n)."""
    if amplitudes is None:
        amplitudes = np.ones(len(phase_codes))
    k = 2 * np.pi / LAMBDA
    th_rad = np.deg2rad(theta_deg_arr)
    phases_rad = codes_to_radians(phase_codes)
    af = np.zeros(len(th_rad), dtype=complex)
    for n in range(len(phase_codes)):
        xn = n * D_X
        af += amplitudes[n] * np.exp(1j * (k * xn * np.sin(th_rad) + phases_rad[n]))
    return np.abs(af)


def total_pattern_dB(theta_deg_arr, phase_codes, h_pat_lin):
    """Single-row pattern × |AF| → normalised dB rel peak."""
    af = array_factor_hplane(theta_deg_arr, phase_codes)
    pat_lin = af * h_pat_lin
    pat_dB = 20 * np.log10(pat_lin / np.max(pat_lin) + 1e-30)
    return pat_dB


def find_peak(theta_deg_arr, pat_dB):
    i = int(np.argmax(pat_dB))
    return theta_deg_arr[i], pat_dB[i]


def find_main_lobe(theta_deg_arr, pat_dB, search_window=None):
    """Find the deepest dip / peak in a window. Returns (peak_angle, peak_dB,
    bw3dB, sll_dB, sll_angle)."""
    if search_window is None:
        mask = np.ones(len(theta_deg_arr), dtype=bool)
    else:
        lo, hi = search_window
        mask = (theta_deg_arr >= lo) & (theta_deg_arr <= hi)
    idx = np.where(mask)[0]
    i_pk_local = idx[int(np.argmax(pat_dB[idx]))]
    peak_angle = theta_deg_arr[i_pk_local]
    peak_dB = pat_dB[i_pk_local]
    # 3 dB beamwidth around peak
    half = peak_dB - 3.0
    lo_i = i_pk_local
    while lo_i > 0 and pat_dB[lo_i] > half:
        lo_i -= 1
    hi_i = i_pk_local
    while hi_i < len(pat_dB) - 1 and pat_dB[hi_i] > half:
        hi_i += 1
    bw3 = theta_deg_arr[hi_i] - theta_deg_arr[lo_i]
    # First null walk
    null_lo, null_hi = lo_i, hi_i
    while null_lo > 0 and pat_dB[null_lo - 1] < pat_dB[null_lo]:
        null_lo -= 1
    while null_hi < len(pat_dB) - 1 and pat_dB[null_hi + 1] < pat_dB[null_hi]:
        null_hi += 1
    # Sidelobes outside the null-bracketed main lobe
    side_mask = np.ones(len(pat_dB), dtype=bool)
    side_mask[null_lo:null_hi + 1] = False
    if side_mask.any():
        i_sll = int(np.argmax(np.where(side_mask, pat_dB, -100)))
        sll_dB = pat_dB[i_sll] - peak_dB
        sll_angle = theta_deg_arr[i_sll]
    else:
        sll_dB, sll_angle = -np.inf, np.nan
    return peak_angle, peak_dB, bw3, sll_dB, sll_angle


# ============================================================================
# Verifications
# ============================================================================
def main():
    theta_deg, h_pat_lin, e_pat_lin = load_single_row_pattern()
    print(f"[load] embedded element pattern: {len(theta_deg)} samples, "
          f"theta {theta_deg.min():.0f}..{theta_deg.max():.0f}°")
    print(f"[const] λ={LAMBDA*1e3:.3f} mm, d=λ/2={D_X*1e3:.3f} mm, "
          f"N={N_TOTAL} (4 chips × 4 ch)")
    print(f"[const] phase LSB = {PHASE_LSB_DEG:.4f} deg/code (7-bit)")
    print()

    # ------------------------------------------------------------------
    # 1) Steering accuracy at multiple commanded angles
    # ------------------------------------------------------------------
    angles_to_test = [0, 5, 10, 15, 20, 30, 45]
    print("=" * 90)
    print("  TEST 1: Beam steering accuracy — firmware vs correct (φ=0 H-plane)")
    print("=" * 90)
    print(f"{'cmd':>5}  | {'firmware':>40} | {'correct':>40}")
    print(f"{'deg':>5}  | {'peak deg':>10} {'BW3':>6} {'SLL dB':>7} {'SLL deg':>8} | "
          f"{'peak deg':>10} {'BW3':>6} {'SLL dB':>7} {'SLL deg':>8}")
    print("-" * 90)

    rows_for_csv = []
    for ang in angles_to_test:
        codes_fw = firmware_phase_codes(ang)
        codes_co = correct_phase_codes(ang)
        pat_fw = total_pattern_dB(theta_deg, codes_fw, h_pat_lin)
        pat_co = total_pattern_dB(theta_deg, codes_co, h_pat_lin)
        pk_fw = find_main_lobe(theta_deg, pat_fw)
        pk_co = find_main_lobe(theta_deg, pat_co)
        print(f"{ang:>5}  | {pk_fw[0]:>+10.1f} {pk_fw[2]:>5.1f}° {pk_fw[3]:>+6.1f} "
              f"{pk_fw[4]:>+7.1f}° | {pk_co[0]:>+10.1f} {pk_co[2]:>5.1f}° "
              f"{pk_co[3]:>+6.1f} {pk_co[4]:>+7.1f}°")
        rows_for_csv.append((ang, pk_fw[0], pk_fw[2], pk_fw[3], pk_fw[4],
                             pk_co[0], pk_co[2], pk_co[3], pk_co[4]))
    print()

    with open(os.path.join(OUT_DIR, "steering_table.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cmd_deg", "fw_peak_deg", "fw_bw3", "fw_sll_dB", "fw_sll_deg",
                    "co_peak_deg", "co_bw3", "co_sll_dB", "co_sll_deg"])
        for r in rows_for_csv:
            w.writerow(r)
    print(f"[out] {OUT_DIR}/steering_table.csv")

    # ------------------------------------------------------------------
    # 2) Side-by-side patterns at scan angles 0°, 15°, 30°, 45°
    # ------------------------------------------------------------------
    show_angles = [0, 15, 30, 45]
    fig, axes = plt.subplots(len(show_angles), 1, figsize=(11, 3.3*len(show_angles)),
                              sharex=True)
    for ax, ang in zip(axes, show_angles):
        codes_fw = firmware_phase_codes(ang)
        codes_co = correct_phase_codes(ang)
        pat_fw = total_pattern_dB(theta_deg, codes_fw, h_pat_lin)
        pat_co = total_pattern_dB(theta_deg, codes_co, h_pat_lin)
        ax.plot(theta_deg, pat_co, "g-", lw=1.4, label="correct 16-elem (gold)")
        ax.plot(theta_deg, pat_fw, "r-", lw=1.4, label="firmware (4-elem broadcast)")
        ax.axvline(ang, color="k", ls=":", lw=0.8, label=f"commanded θ={ang}°")
        ax.axvline(-ang, color="grey", ls=":", lw=0.6, alpha=0.5,
                   label=f"-cmd θ={-ang}° (sign-flip)")
        ax.set_xlim(-90, 90)
        ax.set_ylim(-40, 2)
        ax.set_ylabel("Pattern (dB)")
        ax.set_title(f"setBeamAngle({ang}°) — H-plane (φ=0, x-scan) at 10.520 GHz")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="lower right", fontsize=8)
    axes[-1].set_xlabel("θ (deg)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "firmware_vs_correct.png"), dpi=140)
    plt.close(fig)
    print(f"[out] {OUT_DIR}/firmware_vs_correct.png")

    # ------------------------------------------------------------------
    # 3) Scan loss curve (peak gain vs commanded angle, both fw and correct)
    # ------------------------------------------------------------------
    scan_angles = np.arange(-60, 61, 2)
    peak_dB_fw = []
    peak_dB_co = []
    actual_peak_fw = []
    actual_peak_co = []
    # Reference broadside peak for absolute scan-loss — peak in dB rel peak is 0;
    # we want amplitude relative to broadside, so compute |E|² without normalisation.

    def total_amp_lin(theta_deg_arr, phase_codes):
        af = array_factor_hplane(theta_deg_arr, phase_codes)
        return af * h_pat_lin

    ref_lin = total_amp_lin(theta_deg, np.zeros(N_TOTAL, dtype=int))
    ref_peak = float(np.max(ref_lin))
    for ang in scan_angles:
        codes_fw = firmware_phase_codes(ang)
        codes_co = correct_phase_codes(ang)
        amp_fw = total_amp_lin(theta_deg, codes_fw)
        amp_co = total_amp_lin(theta_deg, codes_co)
        peak_dB_fw.append(20*np.log10(np.max(amp_fw)/ref_peak + 1e-30))
        peak_dB_co.append(20*np.log10(np.max(amp_co)/ref_peak + 1e-30))
        i_fw = int(np.argmax(amp_fw))
        i_co = int(np.argmax(amp_co))
        actual_peak_fw.append(theta_deg[i_fw])
        actual_peak_co.append(theta_deg[i_co])

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    ax = axes[0]
    ax.plot(scan_angles, peak_dB_co, "g-", lw=1.6, label="correct 16-elem")
    ax.plot(scan_angles, peak_dB_fw, "r-", lw=1.6, label="firmware (4-elem broadcast)")
    # Theoretical scan loss = cos(θ) (single-element factor) → in dB: 20·log10(cos)
    th_th = np.linspace(-60, 60, 121)
    ax.plot(th_th, 20*np.log10(np.cos(np.deg2rad(th_th))),
            "k--", lw=1.0, alpha=0.6, label="cos(θ) ideal scan loss")
    ax.set_xlabel("Commanded scan angle (deg)")
    ax.set_ylabel("Peak gain rel broadside (dB)")
    ax.set_title("Scan loss vs commanded angle")
    ax.set_xlim(-60, 60)
    ax.set_ylim(-25, 2)
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1]
    ax.plot(scan_angles, actual_peak_co, "g-", lw=1.6, label="correct 16-elem")
    ax.plot(scan_angles, actual_peak_fw, "r-", lw=1.6, label="firmware")
    ax.plot(scan_angles, scan_angles, "k--", lw=1.0, alpha=0.6,
            label="ideal (peak = cmd)")
    ax.plot(scan_angles, -scan_angles, "k:", lw=1.0, alpha=0.4,
            label="sign-flipped (peak = -cmd)")
    ax.set_xlabel("Commanded scan angle (deg)")
    ax.set_ylabel("Actual peak angle (deg)")
    ax.set_title("Beam pointing accuracy")
    ax.set_xlim(-60, 60)
    ax.set_ylim(-90, 90)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "scan_loss.png"), dpi=140)
    plt.close(fig)
    print(f"[out] {OUT_DIR}/scan_loss.png")

    # ------------------------------------------------------------------
    # 4) Null-steering: place a null at a chosen angle (LCMV-style minimal)
    # ------------------------------------------------------------------
    # Set main beam at θ=0, with an explicit null at θ_null=20° using simple
    # phase-only synthesis: subtract a unit-amplitude vector pointed at θ_null.
    th_null = 20.0
    k = 2*np.pi/LAMBDA
    n = np.arange(N_TOTAL)
    a_main = np.exp(1j * 0.0 * n)
    a_null = np.exp(1j * k * n * D_X * np.sin(np.deg2rad(th_null)))
    # Project: a' = a_main - <a_null, a_main>/<a_null, a_null> * a_null
    proj = (np.vdot(a_null, a_main) / np.vdot(a_null, a_null)) * a_null
    a_steered = a_main - proj
    # Convert complex weights to phase codes (drop amplitude variation —
    # ADAR1000 phase shifters are constant-amplitude; we keep amplitude=1 and
    # use phase only for an honest sim).
    phases_null = np.angle(a_steered) % (2*np.pi)
    codes_null = np.round(phases_null / (2*np.pi) * PHASE_STATES).astype(int) % PHASE_STATES
    pat_null = total_pattern_dB(theta_deg, codes_null, h_pat_lin)
    # Reference: no null
    pat_bs = total_pattern_dB(theta_deg, np.zeros(N_TOTAL, dtype=int), h_pat_lin)

    # Find depth of null in the steered pattern at θ=20°
    i_null = int(np.argmin(np.abs(theta_deg - th_null)))
    null_depth = pat_null[i_null] - 0.0  # rel peak

    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(theta_deg, pat_bs, "k-", lw=1.2, alpha=0.5, label="broadside, no null")
    ax.plot(theta_deg, pat_null, "b-", lw=1.6,
            label=f"phase-only null @ θ={th_null}°")
    ax.axvline(th_null, color="r", ls=":", lw=0.8, label=f"target null θ={th_null}°")
    ax.axhline(-30, color="grey", ls="--", lw=0.6, alpha=0.5)
    ax.set_xlim(-90, 90)
    ax.set_ylim(-50, 2)
    ax.set_xlabel("θ (deg)")
    ax.set_ylabel("Pattern (dB)")
    ax.set_title(f"Null-steering — broadside main beam with null at θ={th_null}° "
                 f"(achieved depth: {null_depth:.1f} dB rel peak)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "null_steering.png"), dpi=140)
    plt.close(fig)
    print(f"[out] {OUT_DIR}/null_steering.png")

    # ------------------------------------------------------------------
    # 5) Phase-quantization effect (compare unquantized continuous phase
    #    to 7-bit quantized phase at θ=15°)
    # ------------------------------------------------------------------
    ang = 15
    angle_rad = np.deg2rad(ang)
    phase_shift = (2 * np.pi * D_X * np.sin(angle_rad)) / LAMBDA
    phases_continuous = np.array([(n*phase_shift) % (2*np.pi) for n in range(N_TOTAL)])
    codes_quant = correct_phase_codes(ang)

    def total_pattern_dB_continuous(theta_deg_arr, phases_rad, h_pat_lin):
        k = 2*np.pi/LAMBDA
        th_rad = np.deg2rad(theta_deg_arr)
        af = np.zeros(len(th_rad), dtype=complex)
        for n in range(len(phases_rad)):
            af += np.exp(1j*(k*n*D_X*np.sin(th_rad) + phases_rad[n]))
        amp = np.abs(af) * h_pat_lin
        return 20*np.log10(amp / np.max(amp) + 1e-30)

    pat_cont = total_pattern_dB_continuous(theta_deg, phases_continuous, h_pat_lin)
    pat_quant = total_pattern_dB(theta_deg, codes_quant, h_pat_lin)
    pk_cont = find_main_lobe(theta_deg, pat_cont)
    pk_quant = find_main_lobe(theta_deg, pat_quant)
    print()
    print("=" * 90)
    print("  TEST 2: 7-bit phase quantization vs continuous (at cmd 15°)")
    print(f"  Continuous phase: peak θ={pk_cont[0]:+.1f}°, BW3={pk_cont[2]:.1f}°, "
          f"SLL={pk_cont[3]:+.1f} dB")
    print(f"  Quantized 7-bit : peak θ={pk_quant[0]:+.1f}°, BW3={pk_quant[2]:.1f}°, "
          f"SLL={pk_quant[3]:+.1f} dB")
    print(f"  → quantization adds {pk_cont[3] - pk_quant[3]:+.2f} dB to the SLL "
          f"(positive = quantized has worse SLL)")
    print("=" * 90)

    # ------------------------------------------------------------------
    # 6) Grating-lobe envelope check
    # ------------------------------------------------------------------
    # Theoretical: grating lobes at sin(θ_g) = ±λ/d - sin(θ_0). At d=λ/2, NO
    # grating lobes for any scan angle (since |λ/d - sin(θ_0)| ≥ 1 always).
    # The firmware's 4-element broadcast effectively makes super-pitch d_super
    # = 4d = 2λ → grating lobes at sin(θ_g) = ±λ/(4d) ± sin(θ_0) = ±0.5 ± sin(θ_0).
    print()
    print("  TEST 3: Grating-lobe geometry")
    print("  Element pitch d = λ/2 → no real-space grating lobes at any scan ✓")
    print("  Firmware's 4-elem broadcast → super-pitch d_super = 4d = 2λ")
    print("  → grating lobes appear at sin(θ_g) = ±0.5 ± sin(θ_0)")
    for ang in [0, 15, 30, 45]:
        sin0 = np.sin(np.deg2rad(ang))
        gl = []
        for sign in [+1, -1]:
            sin_g = sign*0.5 + sin0  # firmware steers to -ang due to sign convention
            if abs(sin_g) <= 1:
                gl.append(np.rad2deg(np.arcsin(sin_g)))
        print(f"    cmd {ang:+d}° → grating lobes at: {[f'{g:+.1f}°' for g in gl]}")


if __name__ == "__main__":
    main()
