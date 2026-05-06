#!/usr/bin/env python3
"""
tb_e2e_dsp_to_host_parse.py — PR-Z A6 stage E12.

Reads `captured_frame.hex` (emitted by tb_e2e_dsp_to_host.v via $writememh,
one byte per line, 2-hex-digit format) and pipes it through
`radar_protocol.parse_bulk_frame`, asserting that:

  * the parser returns a valid RadarFrame dict (not None)
  * header fields match expected (E7, E8 are also asserted in the TB
    inline; this is a defense-in-depth re-check)
  * doppler_mag at the three target cells matches the Python golden
    `expected_doppler_mag.npy` (E9 — magnitude row endianness/byte ordering)
  * cfar_dense at target cells == CONFIRMED, at neighbor cells == NONE
    (E10 — detect map 2-bit packing)
  * the captured frame is byte-for-byte identical to expected_frame.bin
    (catches ANY layout drift the per-field assertions would miss)

Exit code 0 on success, 1 on failure (asserted by run_python_test in
run_regression.sh).
"""

from __future__ import annotations

import os
import sys

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, '..', '..', '..', '..'))
GUI_DIR = os.path.join(PROJECT_ROOT, '9_Firmware', '9_3_GUI')

sys.path.insert(0, GUI_DIR)
sys.path.insert(0, THIS_DIR)
from radar_protocol import (    # noqa: E402
    RadarProtocol,
    HEADER_BYTE,
    FOOTER_BYTE,
    NUM_RANGE_BINS,
    NUM_DOPPLER_BINS,
)


# Stimulus / expected frame parameters (must match gen_e2e_*.py).
TEST_FLAGS_BYTE     = 0x2E   # subframe_enable=0b101 + stream=doppler+cfar
EXPECTED_RANGE_BIN  = 67
EXPECTED_TARGETS    = ((67, 2), (67, 18), (67, 34))
NEIGHBOR_NONE_CELLS = ((60, 2), (75, 5), (200, 10))
DETECT_CONFIRMED    = 2
DETECT_NONE         = 0

# Frame-section offsets — must match radar_protocol BULK layout / pack_bulk_frame.
HEADER_BYTES         = 9
DOPPLER_MAG_BYTES    = NUM_RANGE_BINS * NUM_DOPPLER_BINS * 2     # 49152
DETECT_BYTES_PER_RNG = (NUM_DOPPLER_BINS * 2 + 7) // 8           # 12
CFAR_DENSE_BYTES     = NUM_RANGE_BINS * DETECT_BYTES_PER_RNG     # 6144
DOPPLER_OFFSET       = HEADER_BYTES                              # 9
CFAR_OFFSET          = DOPPLER_OFFSET + DOPPLER_MAG_BYTES        # 49161
FOOTER_OFFSET        = CFAR_OFFSET + CFAR_DENSE_BYTES            # 55305

# PR-AA fix: doppler_mag is now bit-exact against the Python golden — the
# WR_DOPPLER_DATA emit FSM advances mag_rd_addr at end of phase 0 (MSB) so
# BRAM has 2 cycles between addr-set and the next pair's MSB read. Strict
# zero-byte tolerance from here; any drift is a real regression.
DOPPLER_MAG_BYTE_DIFF_TOLERANCE = 0


# ============================================================================
# Output helpers
# ============================================================================

class TestState:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0
        self.total  = 0

    def check(self, name: str, cond: bool, detail: str = '') -> None:
        self.total += 1
        if cond:
            self.passed += 1
            return
        self.failed += 1
        msg = f"  [FAIL] {name}"
        if detail:
            msg += f"  ({detail})"
        print(msg)


# ============================================================================
# Captured-frame loader
# ============================================================================

def load_captured_frame_hex(path: str) -> bytes:
    """Read iverilog $writememh output (one byte per line, 2-hex-digit)."""
    out = bytearray()
    with open(path, 'r') as f:
        for line in f:
            tok = line.strip()
            if not tok or tok.startswith('//'):
                continue
            # $writememh sometimes emits address comments like "@0000ABCD";
            # skip them.
            if tok.startswith('@'):
                continue
            out.append(int(tok, 16) & 0xFF)
    return bytes(out)


# ============================================================================
# Main
# ============================================================================

def main() -> int:
    e2e_dir = os.path.join(THIS_DIR, 'e2e_data')
    captured_path = os.path.join(e2e_dir, 'captured_frame.hex')
    expected_path = os.path.join(e2e_dir, 'expected_frame.bin')

    if not os.path.isfile(captured_path):
        print(f"  ERROR: {captured_path} missing — run tb_e2e_dsp_to_host first",
              file=sys.stderr)
        return 1
    if not os.path.isfile(expected_path):
        print(f"  ERROR: {expected_path} missing — run gen_e2e_expected.py",
              file=sys.stderr)
        return 1

    print("============================================================")
    print("  PR-Z A6 stage E12 — Python parse round-trip")
    print("============================================================")

    captured = load_captured_frame_hex(captured_path)
    with open(expected_path, 'rb') as f:
        expected = f.read()

    print(f"  captured: {len(captured)} bytes")
    print(f"  expected: {len(expected)} bytes")

    state = TestState()

    # ---- Quick-look header sanity (also asserted in TB) ----
    state.check('E12.1: captured length == expected length',
                len(captured) == len(expected),
                f"captured={len(captured)} expected={len(expected)}")
    state.check('E12.2: byte0 == 0xAA (magic)', captured[0] == HEADER_BYTE,
                f"got 0x{captured[0]:02X}")
    state.check('E12.3: byte1 == 0x02 (version)', captured[1] == 0x02,
                f"got 0x{captured[1]:02X}")
    state.check('E12.4: byte2 == 0x2E (sf_en=0b101 + stream=0x06)',
                captured[2] == TEST_FLAGS_BYTE,
                f"got 0x{captured[2]:02X}")
    state.check('E12.5: last byte == 0x55 (footer)',
                captured[-1] == FOOTER_BYTE,
                f"got 0x{captured[-1]:02X}")

    # ---- Per-section compare against expected_frame.bin ----
    # E12.6 is split into 4 sub-checks so diffs are isolated:
    #   .a header (strict) .b doppler_mag (strict, post-PR-AA)
    #   .c cfar_dense (strict)  .d footer (strict)
    if len(captured) == len(expected):
        # .a header
        hdr_diff = sum(1 for i in range(HEADER_BYTES) if captured[i] != expected[i])
        state.check('E12.6.a: header bytes == expected (strict)',
                    hdr_diff == 0, f"{hdr_diff} differing bytes")

        # .b doppler_mag — strict bit-exact (PR-AA fix landed)
        dop_diffs = [i for i in range(DOPPLER_OFFSET, CFAR_OFFSET)
                     if captured[i] != expected[i]]
        state.check('E12.6.b: doppler_mag bytes == expected (strict)',
                    len(dop_diffs) <= DOPPLER_MAG_BYTE_DIFF_TOLERANCE,
                    f"{len(dop_diffs)} differing bytes; "
                    f"first 5 at {dop_diffs[:5]}")

        # .c cfar dense — strict bit-for-bit
        cfar_diffs = [i for i in range(CFAR_OFFSET, FOOTER_OFFSET)
                      if captured[i] != expected[i]]
        state.check('E12.6.c: cfar bytes == expected (strict)',
                    len(cfar_diffs) == 0,
                    f"{len(cfar_diffs)} differing bytes; "
                    f"first 5 at {cfar_diffs[:5]}")
        if cfar_diffs[:5]:
            for idx in cfar_diffs[:5]:
                print(f"        cfar [{idx}] cap=0x{captured[idx]:02X} "
                      f"exp=0x{expected[idx]:02X}")

        # .d footer
        foot_diff = 0 if captured[FOOTER_OFFSET] == expected[FOOTER_OFFSET] else 1
        state.check('E12.6.d: footer byte == expected (strict)',
                    foot_diff == 0,
                    f"got 0x{captured[FOOTER_OFFSET]:02X} "
                    f"vs 0x{expected[FOOTER_OFFSET]:02X}")

    # ---- Parse via radar_protocol.parse_bulk_frame (the real host parser) ----
    parsed = RadarProtocol.parse_bulk_frame(captured)
    state.check('E12.7: parse_bulk_frame returns non-None', parsed is not None)
    if parsed is None:
        print("  cannot continue — parse failed")
        return 1 if state.failed else 0

    state.check('E12.8: parsed.frame_size == captured length',
                parsed['frame_size'] == len(captured),
                f"parsed={parsed['frame_size']} captured={len(captured)}")
    state.check('E12.9: parsed.flags == 0x2E', parsed['flags'] == TEST_FLAGS_BYTE,
                f"got 0x{parsed['flags']:02X}")
    state.check('E12.10: parsed.subframe_enable == 0b101',
                parsed['subframe_enable'] == 0b101,
                f"got 0b{parsed['subframe_enable']:03b}")
    state.check('E12.11: parsed.n_range == 512', parsed['n_range'] == NUM_RANGE_BINS)
    state.check('E12.12: parsed.n_doppler == 48', parsed['n_doppler'] == NUM_DOPPLER_BINS)

    # ---- Doppler magnitude — E9 ----
    expected_mag = np.load(os.path.join(e2e_dir, 'expected_doppler_mag.npy'))
    doppler_mag = parsed['doppler_mag']
    state.check('E12.13: doppler_mag shape (512, 48)',
                doppler_mag is not None and doppler_mag.shape == (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
    if doppler_mag is not None:
        # Strict bit-exact post-PR-AA. Any drift fails.
        diff = np.abs(doppler_mag.astype(np.int64) - expected_mag.astype(np.int64))
        max_diff = int(diff.max())
        n_diff = int((diff > 0).sum())
        state.check('E12.14: doppler_mag bit-exact vs Python golden',
                    n_diff == 0,
                    f"max_diff={max_diff} ({n_diff} of {diff.size} cells differ)")

        # Specific target cells — magnitude at exact expected (rb,db) bin.
        for (rb, db) in EXPECTED_TARGETS:
            peak = int(doppler_mag[rb, db])
            state.check(f'E12.15.{rb}.{db}: doppler_mag[{rb},{db}] > 1000',
                        peak > 1000, f"got {peak}")

    # ---- CFAR dense — E10 ----
    cfar_dense = parsed['cfar_dense']
    state.check('E12.16: cfar_dense shape (512, 48)',
                cfar_dense is not None and cfar_dense.shape == (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
    if cfar_dense is not None:
        # All three target cells -> CONFIRMED
        for (rb, db) in EXPECTED_TARGETS:
            cls_v = int(cfar_dense[rb, db])
            state.check(f'E12.17.{rb}.{db}: cfar_dense[({rb}, {db})] == CONFIRMED',
                        cls_v == DETECT_CONFIRMED,
                        f"got class={cls_v}")
        # Neighbor cells -> NONE
        for (rb, db) in NEIGHBOR_NONE_CELLS:
            cls_v = int(cfar_dense[rb, db])
            state.check(f'E12.18.{rb}.{db}: cfar_dense[({rb}, {db})] == NONE',
                        cls_v == DETECT_NONE,
                        f"got class={cls_v}")
        # DC-notch implication: bin 0 of every range row -> NONE
        notched_bins = (0, 16, 32)  # bin 0 of each sub-frame
        notch_violations = 0
        for db in notched_bins:
            for rb in range(NUM_RANGE_BINS):
                if int(cfar_dense[rb, db]) != DETECT_NONE:
                    notch_violations += 1
        state.check('E12.19: all bin-0-per-subframe cells == NONE (DC notched)',
                    notch_violations == 0,
                    f"{notch_violations} cells out of {NUM_RANGE_BINS * 3} violate")

    # ---- Summary ----
    print()
    print("============================================================")
    print(f"  RESULTS: {state.passed} pass, {state.failed} fail / "
          f"{state.total} total")
    print("============================================================")
    if state.failed == 0:
        print("[OVERALL PASS]")
        return 0
    print(f"[OVERALL FAIL] {state.failed} assertion(s)")
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
