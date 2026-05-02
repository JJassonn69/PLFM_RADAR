#!/usr/bin/env python3
"""
gen_realdata_hex.py — Synthetic stimulus + bit-exact golden for the realdata
co-simulation testbenches (tb_doppler_realdata, tb_fullchain_realdata).

Replaces the legacy ADI CN0566 hardware captures (32-chirp / 2-subframe /
32-bin Doppler) with a synthetic radar scene at production dimensions
(48-chirp / 3-subframe / 48-bin Doppler) so the regression no longer
depends on out-of-tree .npy files.

Outputs (all under tb/cosim/real_data/hex/):

  RTL stimuli + goldens (.hex):
    doppler_input_realdata.hex      48 chirps x 512 range bins, packed {Q,I}
    doppler_ref_i.hex / _q.hex      512 range bins x 48 Doppler bins (signed 16-bit)
    fullchain_range_input.hex       48 chirps x 2048 range bins, packed {Q,I}
    fullchain_doppler_ref_i.hex
    fullchain_doppler_ref_q.hex     same shape as doppler_ref_*

  GUI replay intermediates (.npy, COSIM_DIR ReplayFormat in v7.replay):
    decimated_range_i.npy / _q.npy   (48, 512) — post range_bin_decimator
    doppler_map_i.npy    / _q.npy    (512, 48) — post doppler_processor
    fullchain_cfar_flags.npy         (512, 48) uint8 — CA-CFAR detection mask
    fullchain_cfar_mag.npy           (512, 48) — |I|+|Q| magnitudes (CFAR input)

Dimensions match production (radar_params.vh: RP_FFT_SIZE=2048,
RP_DECIMATION_FACTOR=4, RP_NUM_RANGE_BINS=512, RP_NUM_DOPPLER_BINS=48).

Pipeline modeled (bit-exact to RTL):
  doppler-only:  scene -> doppler_processor (3-subframe, 16-pt FFT, Hamming)
  fullchain:     scene -> range_bin_decimator (2048->512 peak, DECIM=4)
                       -> doppler_processor

Usage:  python3 gen_realdata_hex.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fpga_model import DopplerProcessor, RangeBinDecimator, run_cfar_ca
from radar_scene import Target, generate_doppler_frame


# ----------------------------------------------------------------------------
# Production dimensions (radar_params.vh: PR-F + RP_FFT_SIZE/RP_DECIMATION_FACTOR)
# ----------------------------------------------------------------------------
NUM_SUBFRAMES        = 3
DOPPLER_FFT_SIZE     = 16
DOPPLER_TOTAL_BINS   = NUM_SUBFRAMES * DOPPLER_FFT_SIZE   # 48
CHIRPS_PER_SUBFRAME  = 16
CHIRPS_PER_FRAME     = NUM_SUBFRAMES * CHIRPS_PER_SUBFRAME  # 48

# Doppler-only TB: post-decim range bins fed straight into doppler.
# Matches production RP_NUM_RANGE_BINS = RP_FFT_SIZE / RP_DECIMATION_FACTOR.
DOPPLER_RANGE_BINS   = 512

# Fullchain TB: pre-decim 2048-bin range FFT -> range_bin_decimator (DECIM=4) -> doppler.
FULLCHAIN_INPUT_BINS    = 2048
FULLCHAIN_OUTPUT_BINS   = DOPPLER_RANGE_BINS              # 512
FULLCHAIN_DECIM_FACTOR  = FULLCHAIN_INPUT_BINS // FULLCHAIN_OUTPUT_BINS   # 4

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hex")

# ----------------------------------------------------------------------------
# Scene — two distinct targets so both the FFT bin layout and the slow-time
# Doppler axis carry information across all three sub-frames.
# ----------------------------------------------------------------------------
SCENE_TARGETS = [
    Target(range_m=300,  velocity_mps= 10.0, rcs_dbsm=20.0),
    Target(range_m=800,  velocity_mps=-20.0, rcs_dbsm=15.0),
]
SCENE_SEED = 42


def write_hex_32(path, samples_iq):
    """Packed 32-bit {Q[31:16], I[15:0]} per line for $readmemh."""
    with open(path, 'w') as f:
        for (i_val, q_val) in samples_iq:
            packed = ((q_val & 0xFFFF) << 16) | (i_val & 0xFFFF)
            f.write(f"{packed:08X}\n")


def write_hex_16(path, values):
    """One signed 16-bit value per line, two's-complement hex."""
    with open(path, 'w') as f:
        for v in values:
            f.write(f"{v & 0xFFFF:04X}\n")


def make_doppler_processor():
    """DopplerProcessor at production dimensions (3 sub-frames, 512 range bins)."""
    return DopplerProcessor()                        # defaults: NUM_SUBFRAMES=3, RANGE_BINS=512


def flatten_doppler_map(doppler_i, doppler_q):
    """RTL stream order: rbin 0 [dbin 0..47], rbin 1 [...], ..., rbin 63 [...]."""
    flat_i, flat_q = [], []
    for rb in range(DOPPLER_RANGE_BINS):
        for db in range(DOPPLER_TOTAL_BINS):
            flat_i.append(doppler_i[rb][db])
            flat_q.append(doppler_q[rb][db])
    return flat_i, flat_q


def gen_doppler_realdata():
    """tb_doppler_realdata: post-decim 48 x 512 -> doppler 512 x 48."""
    print("[doppler_realdata] generating ...")
    frame_i, frame_q = generate_doppler_frame(
        SCENE_TARGETS,
        n_chirps=CHIRPS_PER_FRAME,
        n_range_bins=DOPPLER_RANGE_BINS,
        seed=SCENE_SEED,
    )

    stim = [
        (frame_i[c][rb], frame_q[c][rb])
        for c in range(CHIRPS_PER_FRAME)
        for rb in range(DOPPLER_RANGE_BINS)
    ]
    write_hex_32(os.path.join(OUT_DIR, "doppler_input_realdata.hex"), stim)

    dp = make_doppler_processor()
    doppler_i, doppler_q = dp.process_frame(frame_i, frame_q)
    flat_i, flat_q = flatten_doppler_map(doppler_i, doppler_q)
    write_hex_16(os.path.join(OUT_DIR, "doppler_ref_i.hex"), flat_i)
    write_hex_16(os.path.join(OUT_DIR, "doppler_ref_q.hex"), flat_q)

    expected_stim = CHIRPS_PER_FRAME * DOPPLER_RANGE_BINS
    print(f"  stimulus: {len(stim)} packed lines (expected {expected_stim})")
    print(f"  golden:   {len(flat_i)} lines i / {len(flat_q)} lines q "
          f"(expected {DOPPLER_RANGE_BINS * DOPPLER_TOTAL_BINS})")


def gen_fullchain_realdata():
    """tb_fullchain_realdata: 48 x 2048 -> RangeBinDecimator (DECIM=4 peak) -> doppler 512 x 48."""
    print("[fullchain_realdata] generating ...")
    frame_i, frame_q = generate_doppler_frame(
        SCENE_TARGETS,
        n_chirps=CHIRPS_PER_FRAME,
        n_range_bins=FULLCHAIN_INPUT_BINS,
        seed=SCENE_SEED,
    )

    stim = [
        (frame_i[c][rb], frame_q[c][rb])
        for c in range(CHIRPS_PER_FRAME)
        for rb in range(FULLCHAIN_INPUT_BINS)
    ]
    write_hex_32(os.path.join(OUT_DIR, "fullchain_range_input.hex"), stim)

    # fpga_model.RangeBinDecimator is hard-coded to 2048->512, DECIM=4 — production.
    decim_i_2d, decim_q_2d = [], []
    for c in range(CHIRPS_PER_FRAME):
        di, dq = RangeBinDecimator.decimate(frame_i[c], frame_q[c], mode=1, start_bin=0)
        decim_i_2d.append(di)
        decim_q_2d.append(dq)

    dp = make_doppler_processor()
    doppler_i, doppler_q = dp.process_frame(decim_i_2d, decim_q_2d)
    flat_i, flat_q = flatten_doppler_map(doppler_i, doppler_q)
    write_hex_16(os.path.join(OUT_DIR, "fullchain_doppler_ref_i.hex"), flat_i)
    write_hex_16(os.path.join(OUT_DIR, "fullchain_doppler_ref_q.hex"), flat_q)

    # Same arrays serialized for v7.replay COSIM_DIR format (GUI replay).
    decim_i_arr = np.asarray(decim_i_2d, dtype=np.int32)
    decim_q_arr = np.asarray(decim_q_2d, dtype=np.int32)
    doppler_i_arr = np.asarray(doppler_i, dtype=np.int32)
    doppler_q_arr = np.asarray(doppler_q, dtype=np.int32)
    np.save(os.path.join(OUT_DIR, "decimated_range_i.npy"), decim_i_arr)
    np.save(os.path.join(OUT_DIR, "decimated_range_q.npy"), decim_q_arr)
    np.save(os.path.join(OUT_DIR, "doppler_map_i.npy"),     doppler_i_arr)
    np.save(os.path.join(OUT_DIR, "doppler_map_q.npy"),     doppler_q_arr)

    # m-9: Run the CA-CFAR model on the doppler map so v7.replay shows
    # detections instead of falling through to zeros. Defaults match the FPGA
    # cold-reset (RP_DEF_CFAR_*: guard=2, train=8, alpha=0x30=3.0 Q4.4, mode=CA).
    cfar_flags, cfar_mag, _thr = run_cfar_ca(
        doppler_i_arr, doppler_q_arr,
        guard=2, train=8, alpha_q44=0x30, mode='CA',
    )
    np.save(os.path.join(OUT_DIR, "fullchain_cfar_flags.npy"),
            cfar_flags.astype(np.uint8))
    np.save(os.path.join(OUT_DIR, "fullchain_cfar_mag.npy"),
            cfar_mag.astype(np.int64))

    print(f"  stimulus: {len(stim)} packed lines "
          f"(expected {CHIRPS_PER_FRAME * FULLCHAIN_INPUT_BINS})")
    print(f"  golden:   {len(flat_i)} lines i / {len(flat_q)} lines q "
          f"(expected {DOPPLER_RANGE_BINS * DOPPLER_TOTAL_BINS})")
    print(f"  cfar:     {int(cfar_flags.sum())} detections "
          f"(across {DOPPLER_RANGE_BINS}x{DOPPLER_TOTAL_BINS} cells)")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    gen_doppler_realdata()
    gen_fullchain_realdata()

    print("\nGenerated files:")
    hex_files = (
        "doppler_input_realdata.hex",
        "doppler_ref_i.hex",
        "doppler_ref_q.hex",
        "fullchain_range_input.hex",
        "fullchain_doppler_ref_i.hex",
        "fullchain_doppler_ref_q.hex",
    )
    for f in hex_files:
        path = os.path.join(OUT_DIR, f)
        with open(path) as fp:
            n_lines = sum(1 for _ in fp)
        print(f"  {f:40s}  {n_lines:7d} lines  ({os.path.getsize(path):7d} bytes)")

    npy_files = (
        "decimated_range_i.npy",
        "decimated_range_q.npy",
        "doppler_map_i.npy",
        "doppler_map_q.npy",
        "fullchain_cfar_flags.npy",
        "fullchain_cfar_mag.npy",
    )
    for f in npy_files:
        path = os.path.join(OUT_DIR, f)
        arr = np.load(path)
        print(f"  {f:40s}  shape={arr.shape!s:>12s}  ({os.path.getsize(path):7d} bytes)")


if __name__ == '__main__':
    main()
