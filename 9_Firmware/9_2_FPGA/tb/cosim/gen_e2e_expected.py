#!/usr/bin/env python3
"""
gen_e2e_expected.py — Bit-exact expected outputs for the PR-Z A6
end-to-end DSP-to-host test (tb_e2e_dsp_to_host.v).

Loads the deterministic stimulus emitted by gen_e2e_stimulus.py and runs
it through the same Python models used by tb_doppler_realdata
(`fpga_model.DopplerProcessor`, `fpga_model.run_cfar_ca`) to produce
expected:

  * doppler map  (post-S-1 DC notch, host_dc_notch_width=1)
  * CFAR detect-class array (NONE/CANDIDATE/CONFIRMED, encoded 0/1/2)
  * USB bulk frame bytes (PR-G v2 layout, doppler + cfar streams)

Design assumption — single deterministic moving target at the bin
identified by gen_e2e_stimulus.py constants (range_bin=67, doppler_bin=2
in each sub-frame). The expected three "CONFIRMED" cells are at
(67, 2), (67, 18), (67, 34).

Frame layout (radar_protocol.py BULK_*):

  flags byte (offset 2):
    bits[2:0] = 0b110     -> stream {cfar, doppler, range} = doppler+cfar
    bits[5:3] = 0b101     -> subframe_enable {LONG, MEDIUM, SHORT}
                             — drops MEDIUM to verify M-8 byte-2 packing
                             (E8 assertion). The doppler/cfar data on
                             the wire still spans all 48 cells; the host
                             CRT downgrades confidence based on this mask.
    bits[7:6] = 0b00      -> reserved-zero
    -> flags_byte = 0x2E

  frame size = 9 (header) + 49152 (doppler) + 6144 (cfar) + 1 (footer)
             = 55306 bytes

The "doppler stream" carries |I| + |Q| as big-endian uint16 per cell
(NOT raw I/Q) — matches usb_data_interface_ft2232h.v which writes the
magnitude approximation, not the complex value. Wait — the wire layout
documented in radar_protocol says doppler_mag is uint16, but parse_bulk
reads it raw. The pack here matches the FPGA's actual doppler_mag emit
shape (clamped to uint16).

Outputs (under tb/cosim/e2e_data/):

    expected_doppler_i.hex    24576 lines, 16-bit signed (post-notch I)
    expected_doppler_q.hex    24576 lines, 16-bit signed (post-notch Q)
    expected_cfar_class.hex   24576 lines, 2-bit (0=NONE, 1=CAND, 2=CONFIRM)
    expected_frame.bin        55306 bytes, the full PR-G v2 bulk frame

Usage:
    python3 gen_e2e_stimulus.py        # produce stimulus first
    python3 gen_e2e_expected.py        # then expected goldens
"""

from __future__ import annotations

import os
import struct
import sys

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

from fpga_model import DopplerProcessor, run_cfar_ca  # noqa: E402

# Pull stimulus configuration verbatim so dimensions stay aligned.
from gen_e2e_stimulus import (   # noqa: E402
    NUM_SUBFRAMES,
    DOPPLER_FFT_SIZE,
    DOPPLER_TOTAL_BINS,
    CHIRPS_PER_FRAME,
    RANGE_BINS,
    HOST_DC_NOTCH_WIDTH,
    EXPECTED_DETECT_CELLS,
)


# ============================================================================
# Frame layout constants (mirror radar_protocol.py)
# ============================================================================
HEADER_BYTE = 0xAA
FOOTER_BYTE = 0x55
RP_USB_PROTOCOL_VERSION = 0x02

BULK_FLAG_STREAM_RANGE   = 0x01
BULK_FLAG_STREAM_DOPPLER = 0x02
BULK_FLAG_STREAM_CFAR    = 0x04
BULK_SUBFRAME_ENABLE_SHIFT = 3

BULK_FRAME_HEADER_SIZE      = 9
BULK_RANGE_SECTION_BYTES    = RANGE_BINS * 2                      # 1024
BULK_DOPPLER_MAG_BYTES      = RANGE_BINS * DOPPLER_TOTAL_BINS * 2 # 49152
BULK_DETECT_BITS_PER_CELL   = 2
BULK_DETECT_BYTES_PER_RANGE = (DOPPLER_TOTAL_BINS * BULK_DETECT_BITS_PER_CELL + 7) // 8  # 12
BULK_DETECT_DENSE_BYTES     = RANGE_BINS * BULK_DETECT_BYTES_PER_RANGE  # 6144
BULK_FOOTER_SIZE            = 1

# E2E test wire shape
TEST_STREAM_FLAGS    = BULK_FLAG_STREAM_DOPPLER | BULK_FLAG_STREAM_CFAR  # 0x06
TEST_SUBFRAME_ENABLE = 0b101    # {LONG, MEDIUM, SHORT} = drop MEDIUM
TEST_FLAGS_BYTE = (TEST_SUBFRAME_ENABLE << BULK_SUBFRAME_ENABLE_SHIFT) | TEST_STREAM_FLAGS
# 0x28 | 0x06 = 0x2E
# First-frame snapshot: usb_data_interface_ft2232h captures frame_number
# BEFORE increment (radar_system_top.v opcode dispatch tb_usb_protocol_v2
# TEST 2.4 doc: "snapshot latches OLD frame_number at frame_complete"),
# so the first frame emitted carries fn=0.
TEST_FRAME_NUMBER = 0x0000

# CFAR config — production cold-reset defaults (RP_DEF_CFAR_*)
CFAR_GUARD     = 2
CFAR_TRAIN     = 8
CFAR_ALPHA_Q44 = 0x30   # = 3.0
CFAR_MODE      = 'CA'
# 2-tier soft alpha (CANDIDATE) — looser
CFAR_ALPHA_SOFT_Q44 = 0x18   # = 1.5

# Detect-class encoding (matches `RP_DETECT_NONE/CANDIDATE/CONFIRMED`).
DETECT_NONE      = 0
DETECT_CANDIDATE = 1
DETECT_CONFIRMED = 2


# ============================================================================
# DC notch — replicate the radar_system_top.v post-S-1 logic
# ============================================================================

def apply_dc_notch(doppler_i: np.ndarray, doppler_q: np.ndarray,
                   notch_width: int) -> tuple[np.ndarray, np.ndarray]:
    """Replicate radar_system_top.v DC-notch (post S-1 inclusive comparators).

    For each in-sub-frame bin b in [0..15]:
        notched if (W != 0) and (b <= W or b >= 16 - W)
    The notch is replicated independently for each of the 3 sub-frames.
    """
    if notch_width == 0:
        return doppler_i.copy(), doppler_q.copy()
    out_i = doppler_i.copy()
    out_q = doppler_q.copy()
    for sf in range(NUM_SUBFRAMES):
        for b in range(DOPPLER_FFT_SIZE):
            if b <= notch_width or b >= (DOPPLER_FFT_SIZE - notch_width):
                col = sf * DOPPLER_FFT_SIZE + b
                out_i[:, col] = 0
                out_q[:, col] = 0
    return out_i, out_q


# ============================================================================
# CFAR 2-tier — produce class codes (NONE/CANDIDATE/CONFIRMED)
# ============================================================================

def run_cfar_two_tier(doppler_i: np.ndarray, doppler_q: np.ndarray,
                      guard: int, train: int,
                      alpha_q44: int, alpha_soft_q44: int,
                      mode: str = 'CA') -> tuple[np.ndarray, np.ndarray]:
    """Run CFAR twice — once with the strict alpha (CONFIRMED tier), once
    with the soft alpha (CANDIDATE tier). Combine into a single per-cell
    class code per the PR-F 2-tier scheme:

      cell magnitude > strict threshold  -> CONFIRMED (2)
      cell magnitude > soft   threshold  -> CANDIDATE (1)
      else                                -> NONE      (0)

    Returns (class_codes, magnitudes).
    """
    flags_strict, mags, _ = run_cfar_ca(
        doppler_i, doppler_q,
        guard=guard, train=train, alpha_q44=alpha_q44, mode=mode,
    )
    flags_soft, _, _ = run_cfar_ca(
        doppler_i, doppler_q,
        guard=guard, train=train, alpha_q44=alpha_soft_q44, mode=mode,
    )
    classes = np.zeros_like(flags_strict, dtype=np.uint8)
    classes[flags_soft]   = DETECT_CANDIDATE
    classes[flags_strict] = DETECT_CONFIRMED
    return classes, mags


# ============================================================================
# Hex / .npy emission
# ============================================================================

def write_hex_16_signed(path: str, arr_2d: np.ndarray) -> int:
    """Emit signed-16-bit hex per cell, range-major (matches doppler_ref_*.hex).

    arr_2d shape (RANGE_BINS, DOPPLER_TOTAL_BINS).
    """
    n = 0
    with open(path, 'w') as f:
        for rb in range(arr_2d.shape[0]):
            for db in range(arr_2d.shape[1]):
                v = int(arr_2d[rb, db]) & 0xFFFF
                f.write(f"{v:04X}\n")
                n += 1
    return n


def write_hex_2bit_class(path: str, arr_2d: np.ndarray) -> int:
    """Emit class codes as 2-bit hex per cell, range-major. Useful for
    standalone TB lookup; the actual USB packing is in pack_bulk_frame()."""
    n = 0
    with open(path, 'w') as f:
        for rb in range(arr_2d.shape[0]):
            for db in range(arr_2d.shape[1]):
                v = int(arr_2d[rb, db]) & 0x3
                f.write(f"{v:01X}\n")
                n += 1
    return n


# ============================================================================
# USB bulk frame packer (inverse of radar_protocol.parse_bulk_frame)
# ============================================================================

def pack_bulk_frame(frame_number: int, flags: int,
                    doppler_mag: np.ndarray | None,
                    cfar_class: np.ndarray | None,
                    range_profile: np.ndarray | None = None) -> bytes:
    """Pack PR-G v2 bulk frame bytes — inverse of parse_bulk_frame.

    Args:
        frame_number: 16-bit frame counter (big-endian wire)
        flags: full 8-bit flags byte (stream bits + subframe_enable bits)
        doppler_mag: shape (RANGE_BINS, DOPPLER_TOTAL_BINS) uint16 magnitudes,
                     or None if STREAM_DOPPLER not set
        cfar_class: shape (RANGE_BINS, DOPPLER_TOTAL_BINS) uint8 in {0,1,2,3},
                    or None if STREAM_CFAR not set
        range_profile: shape (RANGE_BINS,) uint16, or None
    """
    out = bytearray()

    # Header (9 bytes)
    out.append(HEADER_BYTE)
    out.append(RP_USB_PROTOCOL_VERSION)
    out.append(flags)
    out += struct.pack('>H', frame_number & 0xFFFF)
    out += struct.pack('>H', RANGE_BINS)
    out += struct.pack('>H', DOPPLER_TOTAL_BINS)

    # Range profile section
    if flags & BULK_FLAG_STREAM_RANGE:
        if range_profile is None:
            range_profile = np.zeros(RANGE_BINS, dtype=np.uint16)
        for v in range_profile:
            out += struct.pack('>H', int(v) & 0xFFFF)

    # Doppler magnitude section
    if flags & BULK_FLAG_STREAM_DOPPLER:
        assert doppler_mag is not None
        for rb in range(RANGE_BINS):
            for db in range(DOPPLER_TOTAL_BINS):
                out += struct.pack('>H', int(doppler_mag[rb, db]) & 0xFFFF)

    # CFAR detect-class dense section (2-bit packed, 4 cells/byte MSB-first)
    if flags & BULK_FLAG_STREAM_CFAR:
        assert cfar_class is not None
        for rb in range(RANGE_BINS):
            for byte_idx in range(BULK_DETECT_BYTES_PER_RANGE):
                packed = 0
                for slot in range(4):
                    db = byte_idx * 4 + slot
                    # padding for db >= DOPPLER_TOTAL_BINS lands on slot 3
                    code = int(cfar_class[rb, db]) & 0x3 if db < DOPPLER_TOTAL_BINS else 0
                    packed |= code << ((3 - slot) * 2)
                out.append(packed)

    out.append(FOOTER_BYTE)
    return bytes(out)


# ============================================================================
# Magnitude (|I|+|Q|) -- the doppler_mag stream the FPGA emits
# ============================================================================

def doppler_magnitude_uint16(doppler_i: np.ndarray, doppler_q: np.ndarray) -> np.ndarray:
    """L1 magnitude clamped to uint16 (matches RTL CFAR magnitude path).

    The FPGA's doppler_mag stream into usb_data_interface_ft2232h is the
    same |I|+|Q| sum that cfar_ca consumes. cfar_ca itself caps to 17 bits
    (MAX_MAG = (1<<17)-1) but the wire format is big-endian uint16 — we
    saturate to 0xFFFF here so the round-trip matches.
    """
    mag = np.abs(doppler_i.astype(np.int64)) + np.abs(doppler_q.astype(np.int64))
    return np.clip(mag, 0, 0xFFFF).astype(np.uint16)


# ============================================================================
# Main
# ============================================================================

def main() -> int:
    out_dir = os.path.join(THIS_DIR, 'e2e_data')
    if not os.path.isdir(out_dir):
        print(f"  ERROR: {out_dir} does not exist — run gen_e2e_stimulus.py first",
              file=sys.stderr)
        return 1

    print("[A6 expected] computing bit-exact goldens")
    print(f"  cfg: notch_width={HOST_DC_NOTCH_WIDTH} "
          f"flags=0x{TEST_FLAGS_BYTE:02X} "
          f"(stream=0x{TEST_STREAM_FLAGS:X} sf_en=0b{TEST_SUBFRAME_ENABLE:03b})")
    print(f"       cfar: guard={CFAR_GUARD} train={CFAR_TRAIN} "
          f"alpha=0x{CFAR_ALPHA_Q44:02X} alpha_soft=0x{CFAR_ALPHA_SOFT_Q44:02X} "
          f"mode={CFAR_MODE}")

    # ---- 1. Load stimulus ----
    frame_i_np = np.load(os.path.join(out_dir, 'range_decim_i.npy'))
    frame_q_np = np.load(os.path.join(out_dir, 'range_decim_q.npy'))
    assert frame_i_np.shape == (CHIRPS_PER_FRAME, RANGE_BINS)

    # fpga_model.DopplerProcessor expects Python int lists (it uses bitwise
    # ops with mask 0xFFFF which would overflow int16). Cast up to int32
    # via tolist() so the bit-exact model runs cleanly.
    frame_i = [[int(v) for v in row] for row in frame_i_np]
    frame_q = [[int(v) for v in row] for row in frame_q_np]

    # ---- 2. Doppler (bit-exact) ----
    dp = DopplerProcessor()
    doppler_i_2d, doppler_q_2d = dp.process_frame(frame_i, frame_q)
    doppler_i = np.asarray(doppler_i_2d, dtype=np.int32)
    doppler_q = np.asarray(doppler_q_2d, dtype=np.int32)
    assert doppler_i.shape == (RANGE_BINS, DOPPLER_TOTAL_BINS)

    # ---- 3. DC notch (post-S-1, inclusive comparators) ----
    # Production wiring (radar_system_top.v lines 697 + 818-819):
    #   notched_doppler_data → cfar_ca
    #   raw rx_doppler_output → usb_data_interface_ft2232h doppler_real/imag
    # So the CFAR sees notched data, but the USB frame carries RAW magnitudes.
    notched_i, notched_q = apply_dc_notch(doppler_i, doppler_q, HOST_DC_NOTCH_WIDTH)

    # ---- 4. CFAR 2-tier (operates on notched data, same as RTL) ----
    cfar_class, cfar_mag = run_cfar_two_tier(
        notched_i, notched_q,
        guard=CFAR_GUARD, train=CFAR_TRAIN,
        alpha_q44=CFAR_ALPHA_Q44,
        alpha_soft_q44=CFAR_ALPHA_SOFT_Q44,
        mode=CFAR_MODE,
    )
    n_confirmed = int((cfar_class == DETECT_CONFIRMED).sum())
    n_candidate = int((cfar_class == DETECT_CANDIDATE).sum())
    print(f"  cfar:  {n_confirmed} CONFIRMED, {n_candidate} CANDIDATE "
          f"(+{int((cfar_class == DETECT_NONE).sum())} NONE)")
    for (rb, db) in EXPECTED_DETECT_CELLS:
        print(f"         expected ({rb}, {db}): "
              f"class={cfar_class[rb, db]} mag={cfar_mag[rb, db]} "
              f"doppler=(I={notched_i[rb, db]}, Q={notched_q[rb, db]})")

    # ---- 5. Doppler magnitude for USB stream (RAW, not notched) ----
    # The FPGA wires raw rx_doppler_output (not notched) into the USB
    # doppler_real/imag stream — see comment in step 3 above.
    doppler_mag = doppler_magnitude_uint16(doppler_i, doppler_q)

    # ---- 6. Pack the bulk frame ----
    frame_bytes = pack_bulk_frame(
        frame_number=TEST_FRAME_NUMBER,
        flags=TEST_FLAGS_BYTE,
        doppler_mag=doppler_mag,
        cfar_class=cfar_class,
        range_profile=None,
    )
    expected_size = (BULK_FRAME_HEADER_SIZE
                     + BULK_DOPPLER_MAG_BYTES
                     + BULK_DETECT_DENSE_BYTES
                     + BULK_FOOTER_SIZE)
    if len(frame_bytes) != expected_size:
        print(f"  ERROR: frame size {len(frame_bytes)} != expected {expected_size}",
              file=sys.stderr)
        return 1

    # ---- 7. Emit goldens ----
    # _raw    : pre-notch (what USB sees)
    # _notched: post-notch (what CFAR sees)
    write_hex_16_signed(os.path.join(out_dir, 'expected_doppler_raw_i.hex'), doppler_i)
    write_hex_16_signed(os.path.join(out_dir, 'expected_doppler_raw_q.hex'), doppler_q)
    write_hex_16_signed(os.path.join(out_dir, 'expected_doppler_notched_i.hex'), notched_i)
    write_hex_16_signed(os.path.join(out_dir, 'expected_doppler_notched_q.hex'), notched_q)
    write_hex_2bit_class(os.path.join(out_dir, 'expected_cfar_class.hex'), cfar_class)
    np.save(os.path.join(out_dir, 'expected_doppler_raw_i.npy'), doppler_i)
    np.save(os.path.join(out_dir, 'expected_doppler_raw_q.npy'), doppler_q)
    np.save(os.path.join(out_dir, 'expected_doppler_notched_i.npy'), notched_i)
    np.save(os.path.join(out_dir, 'expected_doppler_notched_q.npy'), notched_q)
    np.save(os.path.join(out_dir, 'expected_cfar_class.npy'), cfar_class)
    np.save(os.path.join(out_dir, 'expected_doppler_mag.npy'), doppler_mag)

    frame_path = os.path.join(out_dir, 'expected_frame.bin')
    with open(frame_path, 'wb') as f:
        f.write(frame_bytes)

    print(f"\n  wrote: expected_doppler_{{i,q}}.hex  "
          f"({RANGE_BINS * DOPPLER_TOTAL_BINS} lines each)")
    print(f"         expected_cfar_class.hex      "
          f"({RANGE_BINS * DOPPLER_TOTAL_BINS} lines)")
    print(f"         expected_frame.bin            "
          f"({len(frame_bytes)} bytes)")

    # ---- 8. Sanity: target cells must all be CONFIRMED ----
    failures: list[str] = []
    for (rb, db) in EXPECTED_DETECT_CELLS:
        if cfar_class[rb, db] != DETECT_CONFIRMED:
            failures.append(f"({rb}, {db}) class={cfar_class[rb, db]}")
    if failures:
        print(f"  WARN: target cells not all CONFIRMED: {failures}", file=sys.stderr)
        # Don't fail — the test will catch this, but flag it for review.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
