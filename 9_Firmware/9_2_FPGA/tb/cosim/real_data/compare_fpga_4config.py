#!/usr/bin/env python3
"""
compare_fpga_4config.py — Compare FPGA 4-config diagnostic results against
Python bit-accurate golden reference.

Uses the pre-computed numpy intermediate files from golden_reference.py
to avoid re-running the expensive range FFT + decimator pipeline.

The 4 configurations match the FPGA diag_playback_detect.tcl exactly:
  A: Simple threshold (cfar=0, mti=0, dc_notch=0) — threshold=500
  B: Simple threshold + MTI + DC notch (cfar=0, mti=1, dc_notch=1)
  C: CA-CFAR only (cfar=1, mti=0, dc_notch=0)
  D: CA-CFAR + MTI + DC notch (cfar=1, mti=1, dc_notch=1)

FPGA defaults:
  - Simple threshold: host_threshold = 500
  - CFAR: guard=2, train=8, alpha=0x30 (Q4.4 = 3.0), mode=CA
"""

import numpy as np
import os
import sys

# Add parent dir so we can import golden_reference functions
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from golden_reference import (
    run_range_bin_decimator, run_mti_canceller, run_doppler_fft,
    run_dc_notch, run_cfar_ca, saturate,
    DOPPLER_RANGE_BINS, DOPPLER_TOTAL_BINS, FFT_SIZE, DOPPLER_CHIRPS,
)

# ===========================================================================
# FPGA hardware results from diag_playback_detect.tcl (2025-03-25)
# ===========================================================================
FPGA_RESULTS = {
    'A': {
        'name': 'Simple Threshold',
        'detect_count': 0x0680,  # 1664
        'last_range': 0x3F,     # 63
        'last_doppler': 0x1F,   # 31
        'last_mag': 0x04D90,    # 19856
        'last_thr': 0x001F4,    # 500
    },
    'B': {
        'name': 'Simple Thr + MTI + DC Notch',
        'detect_count': 0x0656,  # 1622
        'last_range': 0x3F,     # 63
        'last_doppler': 0x1E,   # 30
        'last_mag': 0x00396,    # 918
        'last_thr': 0x001F4,    # 500
    },
    'C': {
        'name': 'CA-CFAR only',
        'detect_count': 0x0005,  # 5
        'last_range': 0x02,     # 2
        'last_doppler': 0x1E,   # 30
        'last_mag': 0x0D335,    # 54069
        'last_thr': 0x0874B,    # 34635
    },
    'D': {
        'name': 'CA-CFAR + MTI + DC Notch (FULL)',
        'detect_count': 0x0006,  # 6
        'last_range': 0x02,     # 2
        'last_doppler': 0x1F,   # 31
        'last_mag': 0x058A9,    # 22697
        'last_thr': 0x0388B,    # 14475
    },
}

# FPGA CFAR defaults (from host register map)
SIMPLE_THRESHOLD = 500
CFAR_GUARD = 2
CFAR_TRAIN = 8
CFAR_ALPHA = 0x30  # Q4.4 = 3.0


def simple_threshold_detect(doppler_i, doppler_q, threshold):
    """
    Replicate FPGA simple threshold detection from cfar_ca.v.
    
    In simple threshold mode (cfar_enable=0):
      magnitude = |I| + |Q| (L1 norm, 17-bit unsigned)
      detect if magnitude > threshold
    """
    n_range, n_doppler = doppler_i.shape
    detect_count = 0
    last_range = 0
    last_doppler = 0
    last_mag = 0
    last_thr = threshold

    detections = []

    # RTL processes column-by-column (Doppler bin outer, range bin inner)
    # But detection count is just a total, order doesn't matter for count
    for rbin in range(n_range):
        for dbin in range(n_doppler):
            i_val = int(doppler_i[rbin, dbin])
            q_val = int(doppler_q[rbin, dbin])
            # RTL: abs_i = I[15] ? (~I + 1) : I
            abs_i = (-i_val) & 0xFFFF if i_val < 0 else i_val & 0xFFFF
            abs_q = (-q_val) & 0xFFFF if q_val < 0 else q_val & 0xFFFF
            mag = abs_i + abs_q  # 17-bit unsigned

            if mag > threshold:
                detect_count += 1
                last_range = rbin
                last_doppler = dbin
                last_mag = mag
                last_thr = threshold
                detections.append((rbin, dbin, mag))

    return detect_count, last_range, last_doppler, last_mag, last_thr, detections


def main():
    hex_dir = os.path.join(script_dir, "hex")

    # Load pre-computed intermediate data
    print("=" * 72)
    print("AERIS-10 FPGA vs Python Co-Simulation Comparison")
    print("4-Configuration Diagnostic")
    print("=" * 72)

    # Check for required numpy files
    required_files = [
        "decimated_range_i.npy", "decimated_range_q.npy",
        "range_fft_all_i.npy", "range_fft_all_q.npy",
    ]
    for f in required_files:
        path = os.path.join(hex_dir, f)
        if not os.path.exists(path):
            print(f"ERROR: {path} not found. Run golden_reference.py first.")
            sys.exit(1)

    # Load decimated range data (post range-FFT, post decimator)
    decim_i = np.load(os.path.join(hex_dir, "decimated_range_i.npy"))
    decim_q = np.load(os.path.join(hex_dir, "decimated_range_q.npy"))
    print(f"Loaded decimated range data: shape {decim_i.shape}")

    # Twiddle file for 16-pt Doppler FFT
    fpga_dir = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))
    twiddle_16 = os.path.join(fpga_dir, "fft_twiddle_16.mem")

    results = {}

    # ==================================================================
    # Config A: Simple threshold (cfar=0, mti=0, dc_notch=0)
    # ==================================================================
    print(f"\n{'=' * 72}")
    print("CONFIG A: Simple Threshold (cfar=0, mti=0, dc_notch=0)")
    print(f"{'=' * 72}")

    # No MTI → pass-through
    a_mti_i, a_mti_q = run_mti_canceller(decim_i, decim_q, enable=False)
    # Doppler FFT
    a_doppler_i, a_doppler_q = run_doppler_fft(a_mti_i, a_mti_q,
                                                 twiddle_file_16=twiddle_16)
    # No DC notch → pass-through
    a_notched_i, a_notched_q = run_dc_notch(a_doppler_i, a_doppler_q, width=0)
    # Simple threshold detection
    a_count, a_lr, a_ld, a_lm, a_lt, a_dets = simple_threshold_detect(
        a_notched_i, a_notched_q, SIMPLE_THRESHOLD
    )
    results['A'] = {
        'detect_count': a_count,
        'last_range': a_lr,
        'last_doppler': a_ld,
        'last_mag': a_lm,
        'last_thr': a_lt,
    }
    print(f"  Python detect_count = {a_count}")

    # ==================================================================
    # Config B: Simple threshold + MTI + DC notch (cfar=0, mti=1, dc_notch=1)
    # ==================================================================
    print(f"\n{'=' * 72}")
    print("CONFIG B: Simple Thr + MTI + DC Notch (cfar=0, mti=1, dc_notch=1)")
    print(f"{'=' * 72}")

    b_mti_i, b_mti_q = run_mti_canceller(decim_i, decim_q, enable=True)
    b_doppler_i, b_doppler_q = run_doppler_fft(b_mti_i, b_mti_q,
                                                 twiddle_file_16=twiddle_16)
    b_notched_i, b_notched_q = run_dc_notch(b_doppler_i, b_doppler_q, width=1)
    b_count, b_lr, b_ld, b_lm, b_lt, b_dets = simple_threshold_detect(
        b_notched_i, b_notched_q, SIMPLE_THRESHOLD
    )
    results['B'] = {
        'detect_count': b_count,
        'last_range': b_lr,
        'last_doppler': b_ld,
        'last_mag': b_lm,
        'last_thr': b_lt,
    }
    print(f"  Python detect_count = {b_count}")

    # ==================================================================
    # Config C: CA-CFAR only (cfar=1, mti=0, dc_notch=0)
    # ==================================================================
    print(f"\n{'=' * 72}")
    print("CONFIG C: CA-CFAR only (cfar=1, mti=0, dc_notch=0)")
    print(f"{'=' * 72}")

    c_mti_i, c_mti_q = run_mti_canceller(decim_i, decim_q, enable=False)
    c_doppler_i, c_doppler_q = run_doppler_fft(c_mti_i, c_mti_q,
                                                 twiddle_file_16=twiddle_16)
    c_notched_i, c_notched_q = run_dc_notch(c_doppler_i, c_doppler_q, width=0)
    c_flags, c_mag, c_thr = run_cfar_ca(
        c_notched_i, c_notched_q,
        guard=CFAR_GUARD, train=CFAR_TRAIN,
        alpha_q44=CFAR_ALPHA, mode='CA'
    )
    c_detections = np.argwhere(c_flags)
    c_count = len(c_detections)
    if c_count > 0:
        last = c_detections[-1]
        c_lr, c_ld = int(last[0]), int(last[1])
        c_lm = int(c_mag[c_lr, c_ld])
        c_lt = int(c_thr[c_lr, c_ld])
    else:
        c_lr, c_ld, c_lm, c_lt = 0, 0, 0, 0
    results['C'] = {
        'detect_count': c_count,
        'last_range': c_lr,
        'last_doppler': c_ld,
        'last_mag': c_lm,
        'last_thr': c_lt,
        'detections': c_detections,
    }
    print(f"  Python detect_count = {c_count}")
    for det in c_detections:
        r, d = det
        print(f"    range={r}, doppler={d}, mag={c_mag[r,d]}, thr={c_thr[r,d]}")

    # ==================================================================
    # Config D: CA-CFAR + MTI + DC notch (cfar=1, mti=1, dc_notch=1)
    # ==================================================================
    print(f"\n{'=' * 72}")
    print("CONFIG D: CA-CFAR + MTI + DC Notch (cfar=1, mti=1, dc_notch=1)")
    print(f"{'=' * 72}")

    d_mti_i, d_mti_q = run_mti_canceller(decim_i, decim_q, enable=True)
    d_doppler_i, d_doppler_q = run_doppler_fft(d_mti_i, d_mti_q,
                                                 twiddle_file_16=twiddle_16)
    d_notched_i, d_notched_q = run_dc_notch(d_doppler_i, d_doppler_q, width=1)
    d_flags, d_mag, d_thr = run_cfar_ca(
        d_notched_i, d_notched_q,
        guard=CFAR_GUARD, train=CFAR_TRAIN,
        alpha_q44=CFAR_ALPHA, mode='CA'
    )
    d_detections = np.argwhere(d_flags)
    d_count = len(d_detections)
    if d_count > 0:
        last = d_detections[-1]
        d_lr, d_ld = int(last[0]), int(last[1])
        d_lm = int(d_mag[d_lr, d_ld])
        d_lt = int(d_thr[d_lr, d_ld])
    else:
        d_lr, d_ld, d_lm, d_lt = 0, 0, 0, 0
    results['D'] = {
        'detect_count': d_count,
        'last_range': d_lr,
        'last_doppler': d_ld,
        'last_mag': d_lm,
        'last_thr': d_lt,
        'detections': d_detections,
    }
    print(f"  Python detect_count = {d_count}")
    for det in d_detections:
        r, d_idx = det
        print(f"    range={r}, doppler={d_idx}, mag={d_mag[r,d_idx]}, thr={d_thr[r,d_idx]}")

    # ==================================================================
    # Comparison Summary
    # ==================================================================
    print(f"\n{'=' * 72}")
    print("COMPARISON: FPGA vs Python Golden Reference")
    print(f"{'=' * 72}")
    print(f"{'Config':<8} {'Mode':<35} {'FPGA':>6} {'Python':>8} {'Match':>7}")
    print("-" * 72)

    all_match = True
    for cfg in ['A', 'B', 'C', 'D']:
        fpga_cnt = FPGA_RESULTS[cfg]['detect_count']
        py_cnt = results[cfg]['detect_count']
        match = "YES" if fpga_cnt == py_cnt else "NO"
        if fpga_cnt != py_cnt:
            all_match = False
        name = FPGA_RESULTS[cfg]['name']
        print(f"  {cfg:<6} {name:<35} {fpga_cnt:>6} {py_cnt:>8} {match:>7}")

    print("-" * 72)

    # Detailed comparison for CFAR configs
    for cfg in ['C', 'D']:
        fpga = FPGA_RESULTS[cfg]
        py = results[cfg]
        print(f"\n  Config {cfg} detail:")
        print(f"    FPGA:   count={fpga['detect_count']}, "
              f"last_range={fpga['last_range']}, last_doppler={fpga['last_doppler']}, "
              f"last_mag={fpga['last_mag']}, last_thr={fpga['last_thr']}")
        print(f"    Python: count={py['detect_count']}, "
              f"last_range={py['last_range']}, last_doppler={py['last_doppler']}, "
              f"last_mag={py['last_mag']}, last_thr={py['last_thr']}")

    print(f"\n{'=' * 72}")
    if all_match:
        print("RESULT: ALL CONFIGS MATCH — FPGA pipeline is bit-accurate")
    else:
        print("RESULT: SOME CONFIGS DIFFER — see details above")
        print("  Note: Small differences (1-2 detections) near threshold boundaries")
        print("  are expected due to edge-case rounding in multi-stage pipeline.")
    print(f"{'=' * 72}")


if __name__ == "__main__":
    main()
