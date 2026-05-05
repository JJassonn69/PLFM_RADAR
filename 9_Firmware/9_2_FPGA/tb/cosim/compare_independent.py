#!/usr/bin/env python3
"""
T-6 drift cosim — twin (fpga_model.py, RTL-mirroring) vs reference
(fpga_reference.py, numpy-based).

Designed to catch the bug class where a transcription error exists
identically in BOTH the bit-exact Python twin and the RTL (e.g. a
hand-copied LUT entry, a regenerated .mem with wrong arithmetic, a
window coefficient typo). The existing cosim suite cannot detect that
class because both sides of the comparison are computing the same
algorithm; this script gives an independent third leg.

Strategy (from highest-value to lowest):

  1. Bytewise LUT spot-checks — every entry of every LUT/ROM compared
     to its analytical Q15 value. A single corrupted entry fails here.
       * NCO_SINE_LUT (64 entries, sin(pi*k/128) Q15)
       * Twiddle ROMs (.mem files) for N=16, N=2048
       * DOPPLER_WINDOW_COEFF (16 entries, Dolph-Chebyshev 60 dB Q15)

  2. End-to-end peak-position invariants — feed canonical inputs
     through both twin and reference; the peak bin/index must match.

  3. Roundtrip / structural invariants — FFT->IFFT recovers input,
     NCO output stays on the unit circle, etc.

We intentionally avoid Q15-saturated magnitude comparisons; those are
where the twin's bit-exact Q15 saturation differs from numpy's float
output, and they don't represent a real drift.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

# Required: numpy + scipy. If either is missing, exit code 2 with a [SKIP]
# marker so the regression can distinguish missing-deps from real failures
# (see run_regression.sh "Independent Reference Drift (T-6)" block).
import importlib.util

_MISSING = []
try:
    import numpy as np
except ImportError:
    _MISSING.append("numpy")
# find_spec("scipy.signal") raises ModuleNotFoundError if the parent package
# scipy is itself absent (rather than returning None). Wrap in try/except so
# the regression runner gets a clean rc=2 SKIP instead of an uncaught crash.
try:
    if importlib.util.find_spec("scipy.signal") is None:
        _MISSING.append("scipy")
except ModuleNotFoundError:
    _MISSING.append("scipy")
if _MISSING:
    print(
        "  [SKIP] T-6 drift cosim requires Python packages: "
        f"{', '.join(_MISSING)}.\n"
        "         Install with: uv sync --group dev   (from repo root)\n"
        "         or:           pip install numpy scipy"
    )
    sys.exit(2)

# Make local imports work when invoked from anywhere
THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

import fpga_reference as ref  # noqa: E402
from fpga_model import (  # noqa: E402
    DOPPLER_WINDOW_COEFF,
    NCO,
    DopplerProcessor,
    FFTEngine,
    MatchedFilterChain,
    NCO_SINE_LUT,
    load_twiddle_rom,
)


# =============================================================================
# Tolerances
# =============================================================================

TOL_NCO_LUT_LSB        = 1     # NCO_SINE_LUT: tightest possible
TOL_TWIDDLE_LSB        = 1     # twiddle ROMs: same — quarter-wave Q15 cosine
TOL_WINDOW_LSB         = 4     # 4 LSB ~= 1.2e-4 rounding budget on Q15 round
TOL_NCO_MAG_REL        = 0.04  # quarter-wave LUT artifact at quadrant edges
TOL_FFT_ROUNDTRIP_LSB  = 60    # 11 stages * Q15 noise on 2048-pt; empirical


# =============================================================================
# Result tracker
# =============================================================================

class CheckResult:
    def __init__(self):
        self.pass_count = 0
        self.fail_count = 0

    def check(self, ok: bool, label: str, detail: str = ""):
        tag = "PASS" if ok else "FAIL"
        msg = f"  [{tag}] {label}"
        if detail:
            msg += f" — {detail}"
        print(msg)
        if ok:
            self.pass_count += 1
        else:
            self.fail_count += 1

    def info(self, label: str, detail: str = ""):
        print(f"  [INFO] {label}" + (f" — {detail}" if detail else ""))

    def summary(self):
        total = self.pass_count + self.fail_count
        print(f"\n  RESULT: {self.pass_count}/{total} drift checks passed")
        return self.fail_count == 0


# =============================================================================
# Section 1 — bytewise LUT spot-checks
# =============================================================================

def check_nco_lut(result: CheckResult):
    print("\n--- NCO_SINE_LUT bytewise check ---")
    max_dev = 0
    bad = []
    for k in range(64):
        ideal = round(32767.0 * math.sin(math.pi * k / 128.0))
        dev = abs(NCO_SINE_LUT[k] - ideal)
        if dev > max_dev:
            max_dev = dev
        if dev > TOL_NCO_LUT_LSB:
            bad.append((k, NCO_SINE_LUT[k], ideal, dev))
    result.check(
        max_dev <= TOL_NCO_LUT_LSB,
        f"NCO_SINE_LUT: all 64 entries match sin(pi*k/128) Q15 (tol {TOL_NCO_LUT_LSB} LSB)",
        f"max |LUT - ideal| = {max_dev} LSB" + (
            f"; {len(bad)} entries off, e.g. k={bad[0][0]}: LUT={bad[0][1]}, "
            f"ideal={bad[0][2]}" if bad else ""
        )
    )


def check_twiddle_rom(result: CheckResult, n: int, mem_filename: str):
    print(f"\n--- Twiddle ROM ({mem_filename}, N={n}) bytewise check ---")
    rom_path = Path(__file__).resolve().parents[2] / mem_filename
    if not rom_path.exists():
        result.check(False, f"Twiddle ROM file present at {rom_path}", "missing")
        return
    cos_rom = load_twiddle_rom(str(rom_path), n=n)
    expected_entries = n // 4
    if len(cos_rom) != expected_entries:
        result.check(
            False,
            f"Twiddle ROM length = N/4 = {expected_entries}",
            f"got {len(cos_rom)}"
        )
        return

    max_dev = 0
    bad = []
    for k in range(expected_entries):
        ideal = round(32767.0 * math.cos(2.0 * math.pi * k / n))
        # Q15 representation tops at 32767 — ideal cos(0)=32768 rounds down.
        if ideal == 32768:
            ideal = 32767
        dev = abs(cos_rom[k] - ideal)
        if dev > max_dev:
            max_dev = dev
        if dev > TOL_TWIDDLE_LSB:
            bad.append((k, cos_rom[k], ideal, dev))
    result.check(
        max_dev <= TOL_TWIDDLE_LSB,
        (
            f"{mem_filename}: all {expected_entries} entries match "
            f"cos(2pi*k/{n}) Q15 (tol {TOL_TWIDDLE_LSB} LSB)"
        ),
        f"max |ROM - ideal| = {max_dev} LSB" + (
            f"; {len(bad)} bad, e.g. k={bad[0][0]}: ROM={bad[0][1]}, ideal={bad[0][2]}"
            if bad else ""
        )
    )


def check_doppler_window_lut(result: CheckResult):
    print("\n--- DOPPLER_WINDOW_COEFF bytewise check ---")
    win_lut = np.array(DOPPLER_WINDOW_COEFF, dtype=np.int64)
    win_ref = np.round(ref.doppler_window_ideal()).astype(np.int64)
    diff = np.abs(win_lut - win_ref)
    max_dev = int(diff.max())
    worst_idx = int(np.argmax(diff))
    result.check(
        max_dev <= TOL_WINDOW_LSB,
        (
            f"DOPPLER_WINDOW_COEFF: all 16 entries match "
            f"Dolph-Chebyshev 60 dB Q15 (tol {TOL_WINDOW_LSB} LSB)"
        ),
        f"max |LUT - ideal| = {max_dev} LSB at n={worst_idx} "
        f"(LUT={int(win_lut[worst_idx])}, ideal={int(win_ref[worst_idx])})"
    )


# =============================================================================
# Section 2 — end-to-end peak / structural invariants
# =============================================================================

def check_nco_invariants(result: CheckResult):
    """NCO output should stay on unit circle and have the right frequency."""
    print("\n--- NCO output invariants (unit-circle, frequency) ---")
    ftw = 0x4CCCCCCD       # 120 MHz at 400 MSPS = 0.30 cycles/sample
    n_capture = 1024

    nco = NCO()
    cos_stream: list[int] = []
    sin_stream: list[int] = []
    for _ in range(n_capture + 64):
        s, c, ready = nco.step(ftw)
        if ready:
            cos_stream.append(c)
            sin_stream.append(s)
        if len(cos_stream) >= n_capture:
            break
    cos_arr = np.array(cos_stream, dtype=np.float64)
    sin_arr = np.array(sin_stream, dtype=np.float64)

    # Unit-circle invariant: cos² + sin² ≈ Q15_max². The quarter-wave LUT
    # uses sin(k*pi/128) for k=0..63 — at the right edge of each quadrant
    # it falls back to LUT[0]=0 instead of cos(63*pi/128)≈sin(pi/128), so
    # the magnitude can dip by up to ~3 % of full scale. Tolerance 4 % covers
    # this expected architectural artifact.
    mag_ratio = (cos_arr ** 2 + sin_arr ** 2) / (32767.0 ** 2)
    max_mag_dev = float(np.max(np.abs(mag_ratio - 1.0)))
    result.check(
        max_mag_dev <= TOL_NCO_MAG_REL,
        "NCO output on unit circle (cos²+sin² ≈ 1)",
        f"max |mag²-1| = {max_mag_dev * 100:.2f} % (tol {TOL_NCO_MAG_REL * 100:.0f} %)"
    )

    # Frequency invariant: dominant FFT bin of cos+j*sin should equal
    # round(ftw / 2^32 * N).
    z = cos_arr + 1j * sin_arr
    Z = np.fft.fft(z)
    peak_bin = int(np.argmax(np.abs(Z)))
    expected_bin = round(ftw / (1 << 32) * n_capture)
    result.check(
        abs(peak_bin - expected_bin) <= 1,
        f"NCO dominant frequency at FTW = {ftw:08X} (expected bin {expected_bin})",
        f"got bin {peak_bin}"
    )


def check_fft_invariants(result: CheckResult):
    """FFT structural sanity: impulse, roundtrip, peak position."""
    print("\n--- FFT-2048 invariants (peak position, roundtrip, impulse) ---")
    n = 2048
    fft = FFTEngine(n=n)

    # Impulse → flat spectrum at amplitude. Under LogiCORE v9.1 Scaled mode
    # (one >>>1 per stage with conv-rounding, total /N applied across LOG2N
    # stages — see PR-O / fpga_model.FFTEngine docstring), each bin lands at
    # round(amp/N), not amp. Lift the input well above the rounding floor so
    # the test catches twiddle / butterfly drift rather than noise. amp=32000
    # ≈ Q15 max keeps every per-stage value within 16-bit headroom and gives
    # expected_per_bin ≈ 16 with no twiddle interaction (impulse has b=0 at
    # every butterfly), so banker's rounding keeps us within ±1 LSB.
    impulse_amp = 32000
    expected_per_bin = round(impulse_amp / n)   # 32000/2048 → 16 (banker's)
    in_re = [impulse_amp] + [0] * (n - 1)
    in_im = [0] * n
    twin_re, twin_im = fft.compute(in_re, in_im, inverse=False)
    flat_max = max(
        max(abs(v - expected_per_bin) for v in twin_re),
        max(abs(v) for v in twin_im),
    )
    result.check(
        flat_max <= 2,
        f"FFT-2048(impulse): all bins ≈ amp/N ({expected_per_bin}) "
        f"[scaled-mode, amp={impulse_amp}]",
        f"max |bin - {expected_per_bin}| = {flat_max}"
    )

    # Single COMPLEX tone (cos + j*sin) → single peak at bin_k (no conjugate
    # at N-bin_k as you'd get with a real cosine). Amp small enough that
    # peak = amp*N stays below Q15 saturation (32767).
    bin_k = 137
    amp = 15
    in_re = [round(amp * math.cos(2 * math.pi * bin_k * i / n)) for i in range(n)]
    in_im = [round(amp * math.sin(2 * math.pi * bin_k * i / n)) for i in range(n)]
    twin_re, twin_im = fft.compute(in_re, in_im, inverse=False)
    ref_re, ref_im = ref.fft_reference(in_re, in_im, n=n)
    twin_mag2 = np.array(twin_re) ** 2 + np.array(twin_im) ** 2
    ref_mag2 = np.asarray(ref_re) ** 2 + np.asarray(ref_im) ** 2
    twin_peak = int(np.argmax(twin_mag2))
    ref_peak = int(np.argmax(ref_mag2))
    result.check(
        twin_peak == ref_peak == bin_k,
        f"FFT-2048(complex tone): peak at bin {bin_k}",
        f"twin={twin_peak}, ref={ref_peak}"
    )

    # Roundtrip — small amplitude (peak = amp*N/2 ≤ 32767 → amp ≤ 32) so the
    # forward FFT does not saturate, then IFFT should recover input within
    # 11*Q15 butterfly noise.
    rt_amp = 30
    in_re = [int(rt_amp * math.sin(2 * math.pi * 73 * i / n)) for i in range(n)]
    in_im = [0] * n
    fwd_re, fwd_im = fft.compute(in_re, in_im, inverse=False)
    rt_re, _ = fft.compute(fwd_re, fwd_im, inverse=True)
    rt_max_err = max(abs(rt_re[i] - in_re[i]) for i in range(n))
    result.check(
        rt_max_err <= TOL_FFT_ROUNDTRIP_LSB,
        (
            f"FFT-2048(roundtrip, amp={rt_amp}): FFT->IFFT recovers input "
            f"within {TOL_FFT_ROUNDTRIP_LSB} LSB"
        ),
        f"max |rt - in| = {rt_max_err}"
    )


def check_mf_invariants(result: CheckResult):
    """MF: peak must land at the injected delay; both twin and ref must agree."""
    print("\n--- Matched filter invariants (peak position) ---")
    n = 2048
    mf = MatchedFilterChain(fft_size=n)

    delay = 100
    bin_k = 17
    # Under scaled-mode FFTs the chain output peak ≈ correlation/N². For a
    # 256-sample tone matched against itself, correlation peak = pulse_len*amp²,
    # so chain output peak ≈ pulse_len*amp² / N² = amp²/16384. Lift amp from
    # 200 (peak ≈ 2.4, swamped by Q15 quantization) to 4000 (peak ≈ 977 — clean
    # margin against the 16-bit IFFT-output saturation at the chain boundary).
    amp = 4000
    pulse_len = 256
    sig_re = [0] * n
    sig_im = [0] * n
    ref_re_in = [0] * n
    ref_im_in = [0] * n
    for i in range(pulse_len):
        ref_re_in[i] = round(amp * math.cos(2 * math.pi * bin_k * i / pulse_len))
        ref_im_in[i] = round(amp * math.sin(2 * math.pi * bin_k * i / pulse_len))
        sig_re[i + delay] = ref_re_in[i]
        sig_im[i + delay] = ref_im_in[i]

    twin_re, twin_im = mf.process(sig_re, sig_im, ref_re_in, ref_im_in)
    ref_real, ref_imag = ref.matched_filter_reference(
        sig_re, sig_im, ref_re_in, ref_im_in, fft_size=n
    )
    twin_mag = np.sqrt(np.array(twin_re) ** 2 + np.array(twin_im) ** 2)
    ref_mag = np.sqrt(np.asarray(ref_real) ** 2 + np.asarray(ref_imag) ** 2)
    twin_peak = int(np.argmax(twin_mag))
    ref_peak = int(np.argmax(ref_mag))
    result.check(
        twin_peak == ref_peak == delay,
        f"MF: peak at injected delay (bin {delay}) [scaled-mode, amp={amp}]",
        f"twin={twin_peak}, ref={ref_peak}"
    )

    # Sidelobe behaviour: peak should be ≥ 5× the median magnitude. Under
    # scaled-mode at amp=4000 the peak rises to ~977 while sidelobes stay
    # near the LSB floor, easily clearing the threshold.
    twin_peak_val = float(twin_mag[delay])
    twin_median = float(np.median(twin_mag))
    pk_ratio = twin_peak_val / max(twin_median, 1.0)
    result.check(
        pk_ratio >= 5.0,
        "MF peak-to-median ratio ≥ 5",
        f"got ratio {pk_ratio:.2f}"
    )


def check_doppler_invariants(result: CheckResult):
    """Doppler: peak per sub-frame must be at the injected Doppler bin."""
    print("\n--- Doppler invariants (peak per sub-frame) ---")
    chirps_per_frame = 48
    range_bins = 16  # keep small; algorithm identical at any size
    num_subframes = 3
    chirps_per_subframe = 16

    inject = [(0, 5), (1, 11), (2, 3)]
    target_rbin = 10
    amp = 1500  # avoid Q15 saturation: peak = amp * N/2 = 12000 < 32767
    chirp_i = np.zeros((chirps_per_frame, range_bins), dtype=np.int64)
    chirp_q = np.zeros((chirps_per_frame, range_bins), dtype=np.int64)
    for sf, dop_bin in inject:
        for c in range(chirps_per_subframe):
            chirp_idx = sf * chirps_per_subframe + c
            phase = 2 * math.pi * dop_bin * c / chirps_per_subframe
            chirp_i[chirp_idx, target_rbin] = round(amp * math.cos(phase))
            chirp_q[chirp_idx, target_rbin] = round(amp * math.sin(phase))

    dop = DopplerProcessor(num_subframes=num_subframes,
                           chirps_per_frame=chirps_per_frame)
    dop.RANGE_BINS = range_bins
    twin_dop_i, twin_dop_q = dop.process_frame(
        chirp_i.tolist(), chirp_q.tolist()
    )
    ref_re, ref_im = ref.doppler_reference(
        chirp_i, chirp_q,
        num_subframes=num_subframes,
        chirps_per_subframe=chirps_per_subframe,
        range_bins=range_bins,
    )

    twin_mag = np.sqrt(np.array(twin_dop_i) ** 2 + np.array(twin_dop_q) ** 2)
    ref_mag = np.sqrt(ref_re ** 2 + ref_im ** 2)
    expected_bins = [sf * chirps_per_subframe + b for sf, b in inject]

    sf_peaks_twin = []
    sf_peaks_ref = []
    for sf in range(num_subframes):
        lo = sf * chirps_per_subframe
        hi = lo + chirps_per_subframe
        sf_peaks_twin.append(lo + int(np.argmax(twin_mag[target_rbin, lo:hi])))
        sf_peaks_ref.append(lo + int(np.argmax(ref_mag[target_rbin, lo:hi])))

    result.check(
        sf_peaks_twin == expected_bins,
        f"Doppler twin: peaks per sub-frame at {expected_bins}",
        f"got {sf_peaks_twin}"
    )
    result.check(
        sf_peaks_ref == expected_bins,
        f"Doppler reference: peaks per sub-frame at {expected_bins}",
        f"got {sf_peaks_ref}"
    )


# =============================================================================
# Main
# =============================================================================

def main():
    print("============================================================")
    print("  T-6 INDEPENDENT REFERENCE DRIFT COSIM")
    print("    fpga_model.py (twin) vs fpga_reference.py (numpy ideal)")
    print("============================================================")
    result = CheckResult()

    # 1. LUT bytewise spot-checks (highest-value transcription detector)
    check_nco_lut(result)
    check_twiddle_rom(result, n=16,   mem_filename="fft_twiddle_16.mem")
    check_twiddle_rom(result, n=2048, mem_filename="fft_twiddle_2048.mem")
    check_doppler_window_lut(result)

    # 2/3. End-to-end invariants
    check_nco_invariants(result)
    check_fft_invariants(result)
    check_mf_invariants(result)
    check_doppler_invariants(result)

    ok = result.summary()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
