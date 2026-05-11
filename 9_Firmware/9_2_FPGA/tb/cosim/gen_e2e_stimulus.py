#!/usr/bin/env python3
"""
gen_e2e_stimulus.py — Deterministic single-target stimulus for the
PR-Z A6 end-to-end DSP-to-host integration test (tb_e2e_dsp_to_host.v).

Unlike gen_realdata_hex.py (which uses a 2-target scene), this generator
emits a single moving target at (range=100m, velocity=10 m/s) with -40 dBFS
Gaussian noise, sized so the doppler peak lands at a deterministic bin in
each of the 3 sub-frames AND clears the W=1 DC notch:

    f_doppler = 2 * v * fc / c = 700 Hz at fc=10.5 GHz
    sub-frame  PRI         bin = round(f_doppler * 16 * PRI)
       SHORT   175 us      round(1.96) = 2
       MEDIUM  161 us      round(1.80) = 2
       LONG    167 us      round(1.87) = 2

The target appears at the same in-sub-frame doppler bin = 2 in all three
sub-frames, which means after packing into the {sub_frame[1:0], bin[3:0]}
flat 48-bin axis the expected detections are at:

    sub-frame 0  doppler_bin 2   (cell  2)
    sub-frame 1  doppler_bin 2   (cell 18)
    sub-frame 2  doppler_bin 2   (cell 34)

Bin choice rationale: with host_dc_notch_width=1 the notch zeroes per-
subframe bins {0, 1, 15} (post the S-1 inclusive-comparator fix). bin 2
is OUTSIDE the notch, so the target survives — and assertion E4 can
prove the notch IS working by checking bin 0 = 0 / bin 2 != 0.

Range bin computation (post-decim, decim factor = 4 from 2048-pt MF output):
    range_bin = round(2 * R / c * fs / decim) = round(2*100/c * 400e6 / 4)
              = round(0.0667 * 100e6) = round(66.67) = 67

Outputs (under tb/cosim/e2e_data/):

    range_decim_packed.hex   24576 lines, 32-bit packed {Q[31:16], I[15:0]}
                             chirp-major order (chirp 0 bins 0..511, etc.)

The .hex format mirrors `doppler_input_realdata.hex` so the same
$readmemh + chirp-major scan in the RTL TB reads it without modification.

Why this stimulus matters for A6:
  * Single, mathematically predictable target -> every assertion in the
    chain (E1-E12 in the scope memo) has a hand-derivable expected value.
  * Non-folding velocity -> tests RTL Doppler axis correctness, NOT host CRT.
  * 3 sub-frames -> exercises full PR-F architecture (M-8 byte 2 packing).

Usage:
    python3 gen_e2e_stimulus.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

# Make sibling fpga_model / radar_scene importable.
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)


# ============================================================================
# Production dimensions (radar_params.vh + radar_scene.py)
# ============================================================================
NUM_SUBFRAMES        = 3
CHIRPS_PER_SUBFRAME  = 16
CHIRPS_PER_FRAME     = NUM_SUBFRAMES * CHIRPS_PER_SUBFRAME   # 48
RANGE_BINS           = 512
DOPPLER_FFT_SIZE     = 16
DOPPLER_TOTAL_BINS   = NUM_SUBFRAMES * DOPPLER_FFT_SIZE      # 48

# Per-sub-frame PRIs (radar_scene.py / radar_params.vh).
T_PRI_SHORT  = 175e-6
T_PRI_MEDIUM = 161e-6
T_PRI_LONG   = 167e-6
PRI_BY_SF    = (T_PRI_SHORT, T_PRI_MEDIUM, T_PRI_LONG)

# RF chain.
F_CARRIER = 10.5e9
C_LIGHT   = 3.0e8
FS_ADC    = 400e6
DECIM     = 4
RANGE_BIN_HZ = FS_ADC / DECIM           # 100 MHz post-decim sample rate

# Single target (constant across all chirps in the frame).
TARGET_RANGE_M    = 100.0
TARGET_VEL_MPS    = 10.0
TARGET_AMPLITUDE  = 16384                # ~50% full-scale 16-bit signed
NOISE_RMS_LSB     = 327                  # ~ -40 dBFS Gaussian against full-scale 32767
SCENE_SEED        = 4096                 # arbitrary; deterministic

# Host DC-notch width to apply when computing the expected USB frame
# (gen_e2e_expected.py replicates the S-1 inclusive-comparator notch).
HOST_DC_NOTCH_WIDTH = 1

# ============================================================================
# Target placement -> expected bin coordinates
# ============================================================================
# Range bin formula: round(2 * R / c * fs / decim). For R=100m, fs=400 MHz,
# decim=4 -> round(2 * 100 / 3e8 * 100e6) = round(66.667) = 67.
EXPECTED_RANGE_BIN = round(2.0 * TARGET_RANGE_M / C_LIGHT * RANGE_BIN_HZ)

# Per-sub-frame doppler bin (folding into 16-pt FFT). For our 5 m/s target
# this is intentionally non-folding -> 1 in all three sub-frames.
F_DOPPLER_HZ = 2.0 * TARGET_VEL_MPS * F_CARRIER / C_LIGHT
EXPECTED_DOPPLER_BIN_PER_SF = tuple(
    round(F_DOPPLER_HZ * DOPPLER_FFT_SIZE * pri) % DOPPLER_FFT_SIZE
    for pri in PRI_BY_SF
)
# Flat 48-bin doppler-axis expected cells (sub_frame << 4 | bin).
EXPECTED_DETECT_CELLS = tuple(
    (EXPECTED_RANGE_BIN, sf * DOPPLER_FFT_SIZE + dbin)
    for sf, dbin in enumerate(EXPECTED_DOPPLER_BIN_PER_SF)
)


# ============================================================================
# Stimulus synthesis
# ============================================================================

def _wrap_chirp_index_to_subframe(chirp_idx: int) -> tuple[int, int]:
    """Map global chirp index 0..47 to (sub_frame_id, in_subframe_index)."""
    sf = chirp_idx // CHIRPS_PER_SUBFRAME
    k_in_sf = chirp_idx % CHIRPS_PER_SUBFRAME
    return sf, k_in_sf


def _target_phase_rad(chirp_idx: int) -> float:
    """Slow-time phase of the target return at chirp `chirp_idx`.

    Phase resets per sub-frame (each sub-frame is its own coherent integration
    window — the PR-F doppler_processor does an independent 16-pt FFT per
    sub-frame). Across one sub-frame, phase advances by 2*pi*f_doppler*PRI per
    chirp.
    """
    sf, k_in_sf = _wrap_chirp_index_to_subframe(chirp_idx)
    pri = PRI_BY_SF[sf]
    return 2.0 * np.pi * F_DOPPLER_HZ * (k_in_sf * pri)


def generate_range_decim_frame(seed: int = SCENE_SEED) -> tuple[np.ndarray, np.ndarray]:
    """Build a deterministic post-decim frame.

    Returns:
        (frame_i, frame_q) — int16 arrays shape (CHIRPS_PER_FRAME, RANGE_BINS).
    """
    rng = np.random.default_rng(seed)
    frame_i = np.zeros((CHIRPS_PER_FRAME, RANGE_BINS), dtype=np.int32)
    frame_q = np.zeros((CHIRPS_PER_FRAME, RANGE_BINS), dtype=np.int32)

    for c in range(CHIRPS_PER_FRAME):
        # Background noise (independent per chirp / per range bin).
        noise_i = rng.normal(0.0, NOISE_RMS_LSB, RANGE_BINS).astype(np.int32)
        noise_q = rng.normal(0.0, NOISE_RMS_LSB, RANGE_BINS).astype(np.int32)
        frame_i[c, :] = noise_i
        frame_q[c, :] = noise_q

        # Target injection at the expected range bin.
        phi = _target_phase_rad(c)
        sig_i = round(TARGET_AMPLITUDE * np.cos(phi))
        sig_q = round(TARGET_AMPLITUDE * np.sin(phi))
        frame_i[c, EXPECTED_RANGE_BIN] += sig_i
        frame_q[c, EXPECTED_RANGE_BIN] += sig_q

    # Saturate to int16 — the post-decim domain is signed 16-bit.
    frame_i = np.clip(frame_i, -32768, 32767).astype(np.int16)
    frame_q = np.clip(frame_q, -32768, 32767).astype(np.int16)
    return frame_i, frame_q


# ============================================================================
# Hex emission
# ============================================================================

def write_packed_iq_hex(path: str, frame_i: np.ndarray, frame_q: np.ndarray) -> int:
    """Emit packed-32-bit {Q[31:16], I[15:0]} per line, chirp-major.

    Matches `doppler_input_realdata.hex` so the RTL TB's $readmemh + chirp-major
    scan can read it unchanged.
    """
    n = 0
    with open(path, 'w') as f:
        for c in range(CHIRPS_PER_FRAME):
            for rb in range(RANGE_BINS):
                i_val = int(frame_i[c, rb]) & 0xFFFF
                q_val = int(frame_q[c, rb]) & 0xFFFF
                packed = (q_val << 16) | i_val
                f.write(f"{packed:08X}\n")
                n += 1
    return n


def save_scene_npy(out_dir: str, frame_i: np.ndarray, frame_q: np.ndarray) -> None:
    """Save the int16 frame as .npy so gen_e2e_expected.py can re-load it
    without re-generating (keeps the two scripts deterministically aligned)."""
    np.save(os.path.join(out_dir, 'range_decim_i.npy'), frame_i)
    np.save(os.path.join(out_dir, 'range_decim_q.npy'), frame_q)


# ============================================================================
# Main
# ============================================================================

def main() -> int:
    out_dir = os.path.join(THIS_DIR, 'e2e_data')
    os.makedirs(out_dir, exist_ok=True)

    print("[A6 stimulus] generating deterministic single-target scene")
    print(f"  target:     range={TARGET_RANGE_M} m, vel={TARGET_VEL_MPS} m/s")
    print(f"              -> f_doppler = {F_DOPPLER_HZ:.1f} Hz")
    print(f"  expected:   range_bin = {EXPECTED_RANGE_BIN}")
    for sf, dbin in enumerate(EXPECTED_DOPPLER_BIN_PER_SF):
        print(f"              sub-frame {sf}: doppler_bin = {dbin} "
              f"(flat cell {sf*DOPPLER_FFT_SIZE + dbin})")

    frame_i, frame_q = generate_range_decim_frame()

    hex_path = os.path.join(out_dir, 'range_decim_packed.hex')
    n_lines = write_packed_iq_hex(hex_path, frame_i, frame_q)
    save_scene_npy(out_dir, frame_i, frame_q)

    expected_lines = CHIRPS_PER_FRAME * RANGE_BINS
    size_bytes = os.path.getsize(hex_path)
    print(f"\n  wrote: {hex_path}")
    print(f"         {n_lines} lines (expected {expected_lines}), "
          f"{size_bytes} bytes")
    print(f"  wrote: {out_dir}/range_decim_{{i,q}}.npy "
          f"shape={frame_i.shape}")

    if n_lines != expected_lines:
        print("  ERROR: line count mismatch", file=sys.stderr)
        return 1

    # Sanity: target peak should dominate at the expected range bin.
    peak_mag = np.abs(frame_i[:, EXPECTED_RANGE_BIN]).max() + \
               np.abs(frame_q[:, EXPECTED_RANGE_BIN]).max()
    bg_mag_typical = np.median(
        np.abs(frame_i[:, EXPECTED_RANGE_BIN - 5]) +
        np.abs(frame_q[:, EXPECTED_RANGE_BIN - 5])
    )
    snr_lsb_db = 20.0 * np.log10(peak_mag / max(bg_mag_typical, 1.0))
    print(f"\n  peak/noise ratio at bin {EXPECTED_RANGE_BIN}: {snr_lsb_db:.1f} dB")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
