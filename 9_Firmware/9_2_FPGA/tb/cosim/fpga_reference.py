#!/usr/bin/env python3
r"""
Independent floating-point reference for AERIS-10 signal processing chain.

Unlike fpga_model.py (which is a bit-exact PORT of the RTL — same NCO LUT,
same twiddle ROMs, same Q15 quantization), this module computes the
algorithm using ideal numpy/scipy primitives. It is the second leg of the
T-6 three-way triangulation:

    fpga_model.py (twin)  --bit-exact--  RTL simulation
              \                            /
               \                          /
                \-- within tolerance --> /
                  fpga_reference.py
                  (this file)

Drift signal interpretation:
  * twin == reference (within tol) and twin == RTL (bit-exact)
        -> chain healthy
  * twin == reference (within tol) but twin != RTL (bit-exact)
        -> RTL diverged from spec (real RTL bug)
  * twin != reference (outside tol) but twin == RTL (bit-exact)
        -> twin and RTL share a transcription error
           (e.g. wrong NCO_SINE_LUT entry, wrong twiddle ROM, wrong window
           coefficients). This is the bug class T-6 was opened to catch.

Coverage in this revision (highest transcription risk first):
  * NCO  — numpy.cos/sin vs the 64-entry quarter-wave NCO_SINE_LUT
  * FFT  — numpy.fft.fft/ifft vs Q15 twiddle ROM butterfly
  * MF   — numpy ifft(fft(sig) * conj(fft(ref))) vs RTL pipeline
  * Doppler — numpy.fft + ideal Hamming window vs Q15 window LUT + RTL FFT

Out of scope here (lower transcription risk, deferred):
  * CIC / FIR — coefficient files are derived from independent generators
  * Mixer / RangeBinDecimator — trivially correct in both
"""

from __future__ import annotations

import numpy as np


# =============================================================================
# NCO reference — ideal complex sinusoid
# =============================================================================

def nco_reference(num_samples: int, ftw: int, fs: float = 400e6,
                  phase_offset_deg: float = 0.0):
    """Ideal floating-point NCO output, scaled to match Q15 fpga_model.

    The fpga_model.NCO uses a 32-bit phase accumulator stepped by ftw, with
    a 64-entry quarter-wave cos LUT scaled to ±0x7FFF. This reference uses
    numpy.cos/sin directly with no LUT.

    Args:
        num_samples: number of samples to produce
        ftw: 32-bit unsigned phase increment per sample
        fs: sample rate (Hz)
        phase_offset_deg: phase offset (degrees), default 0

    Returns:
        (cos_q15, sin_q15) — float arrays of length num_samples, in Q15 scale
    """
    ftw = int(ftw) & 0xFFFFFFFF
    n = np.arange(num_samples, dtype=np.float64)
    # Phase accumulator semantics: phase[k] = (k * ftw) mod 2^32
    # Convert to radians: 2*pi * phase / 2^32
    phase_rad = 2.0 * np.pi * (n * ftw) / (1 << 32) + np.deg2rad(phase_offset_deg)
    cos_q15 = np.cos(phase_rad) * 32767.0
    sin_q15 = np.sin(phase_rad) * 32767.0
    return cos_q15, sin_q15


# =============================================================================
# FFT reference — numpy.fft.fft / ifft
# =============================================================================

def fft_reference(in_re, in_im, n: int = 2048, inverse: bool = False):
    """Ideal floating-point FFT.

    Scaling matches the RTL convention:
      forward: y[k] = sum_n x[n] * exp(-j*2*pi*k*n/N)            (no 1/N)
      inverse: y[n] = (1/N) * sum_k X[k] * exp(+j*2*pi*k*n/N)    (1/N applied)

    The RTL fft_engine implements >>>LOG2N before output saturation when
    inverse=1, which is the same 1/N. numpy.fft.ifft already includes the
    1/N factor, so we use it directly with no rescaling.

    Args:
        in_re/in_im: length-N int or float sequences
        n: FFT size (16, 1024, 2048)
        inverse: True for IFFT

    Returns:
        (out_re, out_im) — float arrays, no Q15 saturation
    """
    re = np.asarray(in_re, dtype=np.float64)
    im = np.asarray(in_im, dtype=np.float64)
    if len(re) != n or len(im) != n:
        raise ValueError(f"input length {len(re)} != N={n}")
    x = re + 1j * im
    y = np.fft.ifft(x) if inverse else np.fft.fft(x)
    return y.real.copy(), y.imag.copy()


# =============================================================================
# Matched filter reference — IFFT(FFT(sig) * conj(FFT(ref)))
# =============================================================================

def matched_filter_reference(sig_re, sig_im, ref_re, ref_im, fft_size: int = 2048):
    """Ideal range profile via FFT-domain matched filter.

    range_profile = IFFT( FFT(sig) * conj(FFT(ref)) )

    The RTL pipeline does the same operation but with Q15 twiddles, 32-bit
    accumulators, and Q15 round-and-saturate after the conjugate multiply.
    This reference is unquantized.

    Args:
        sig_re/im, ref_re/im: length-fft_size sequences (int or float)
        fft_size: matched filter FFT size (default 2048)

    Returns:
        (range_re, range_im) — float arrays, no Q15 saturation
    """
    sig_re = np.asarray(sig_re, dtype=np.float64)
    sig_im = np.asarray(sig_im, dtype=np.float64)
    ref_re = np.asarray(ref_re, dtype=np.float64)
    ref_im = np.asarray(ref_im, dtype=np.float64)
    s = sig_re + 1j * sig_im
    r = ref_re + 1j * ref_im
    S = np.fft.fft(s, n=fft_size)
    R = np.fft.fft(r, n=fft_size)
    P = S * np.conj(R)
    p = np.fft.ifft(P)
    return p.real.copy(), p.imag.copy()


# =============================================================================
# Doppler reference — Hamming-windowed per-sub-frame 16-pt FFT
# =============================================================================

def hamming_16_ideal():
    """Ideal Hamming(16) coefficients, scaled to Q15.

    fpga_model.HAMMING_WINDOW is a hard-coded Q15 LUT. If a coefficient was
    transcribed wrong, this catches it.

    Definition (matches scipy.signal.windows.hamming(16, sym=False) is
    DIFFERENT — that's a periodic Hamming. The RTL/fpga_model use the
    classic symmetric form: w[n] = 0.54 - 0.46*cos(2*pi*n/(N-1)).
    """
    n = np.arange(16, dtype=np.float64)
    w = 0.54 - 0.46 * np.cos(2.0 * np.pi * n / 15.0)
    return w * 32767.0


def doppler_reference(chirp_data_i, chirp_data_q,
                      num_subframes: int = 3,
                      chirps_per_subframe: int = 16,
                      range_bins: int = 512):
    """Ideal Doppler map using ideal Hamming + numpy.fft, no Q15 quantization.

    Args:
        chirp_data_i/q: 2D arrays [chirps_per_frame][range_bins], int or float
        num_subframes: number of independent 16-pt FFTs per range bin (default 3)
        chirps_per_subframe: 16
        range_bins: 512

    Returns:
        (doppler_map_re, doppler_map_im) — 2D float arrays
            shape [range_bins][num_subframes * chirps_per_subframe]
        Sub-frame s occupies output bins [s*16 .. s*16+15].
    """
    chirp_data_i = np.asarray(chirp_data_i, dtype=np.float64)
    chirp_data_q = np.asarray(chirp_data_q, dtype=np.float64)
    chirps_per_frame = num_subframes * chirps_per_subframe
    if chirp_data_i.shape != (chirps_per_frame, range_bins):
        raise ValueError(
            f"chirp_data_i shape {chirp_data_i.shape} != "
            f"({chirps_per_frame}, {range_bins})"
        )

    win = hamming_16_ideal()  # Q15-scaled
    total_bins = num_subframes * chirps_per_subframe
    out_re = np.zeros((range_bins, total_bins), dtype=np.float64)
    out_im = np.zeros((range_bins, total_bins), dtype=np.float64)

    for rbin in range(range_bins):
        for sf in range(num_subframes):
            start = sf * chirps_per_subframe
            stop = start + chirps_per_subframe
            offset = sf * chirps_per_subframe

            # Apply Hamming and divide by 32768 to undo the Q15 scaling so
            # the comparison is to ideal floating-point amplitudes (the RTL
            # rounds (data*win + 1<<14) >> 15 which is an approximate /32768).
            x_re = chirp_data_i[start:stop, rbin] * win / 32768.0
            x_im = chirp_data_q[start:stop, rbin] * win / 32768.0
            x = x_re + 1j * x_im

            X = np.fft.fft(x)
            out_re[rbin, offset:offset + chirps_per_subframe] = X.real
            out_im[rbin, offset:offset + chirps_per_subframe] = X.imag

    return out_re, out_im


# =============================================================================
# Self-test (sanity checks against numpy's own analytical answers)
# =============================================================================

def _self_test():
    """Quick sanity checks."""
    # NCO: at FTW = 0x4CCCCCCD, frequency = 0.3 * fs = 120 MHz at 400 MSPS.
    cos_q15, sin_q15 = nco_reference(8, 0x4CCCCCCD, fs=400e6)
    # First sample should be cos(0)=1, sin(0)=0 in Q15
    assert abs(cos_q15[0] - 32767.0) < 1.0, f"NCO[0].cos = {cos_q15[0]}"
    assert abs(sin_q15[0]) < 1.0, f"NCO[0].sin = {sin_q15[0]}"

    # FFT: impulse -> all bins = amplitude
    in_re = [1000] + [0] * 15
    in_im = [0] * 16
    out_re, out_im = fft_reference(in_re, in_im, n=16)
    for k in range(16):
        assert abs(out_re[k] - 1000.0) < 1e-9, f"FFT impulse bin {k}: {out_re[k]}"

    # Doppler: zero input -> zero output
    z_i = np.zeros((48, 512))
    z_q = np.zeros((48, 512))
    d_re, d_im = doppler_reference(z_i, z_q, num_subframes=3,
                                   chirps_per_subframe=16, range_bins=512)
    assert np.max(np.abs(d_re)) < 1e-9
    assert np.max(np.abs(d_im)) < 1e-9

    # Hamming: peak at center, edges match formula
    w = hamming_16_ideal()
    assert abs(w[0] - 0.08 * 32767.0) < 0.5
    assert abs(w[7] - (0.54 - 0.46 * np.cos(2 * np.pi * 7 / 15)) * 32767.0) < 0.5

    print("fpga_reference self-test: OK")


if __name__ == '__main__':
    _self_test()
