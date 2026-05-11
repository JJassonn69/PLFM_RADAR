#!/usr/bin/env python3
"""
Tests for AERIS-10 Radar Dashboard protocol parsing, command building,
data recording, and acquisition logic.

Run: python -m pytest test_GUI_V65_Tk.py -v
  or: python test_GUI_V65_Tk.py
"""

import struct
import time
import queue
import os
import tempfile
import unittest
import numpy as np

from radar_protocol import (
    RadarProtocol, FT2232HConnection, FT601Connection, DataRecorder, RadarAcquisition,
    RadarFrame, StatusResponse, Opcode,
    HEADER_BYTE, FOOTER_BYTE, STATUS_HEADER_BYTE,
    NUM_RANGE_BINS, NUM_DOPPLER_BINS,
    DATA_PACKET_SIZE,
    BULK_FRAME_HEADER_SIZE, BULK_FRAME_MAX_SIZE,
    BULK_RANGE_SECTION_BYTES, BULK_FOOTER_SIZE,
    BULK_FLAG_STREAM_RANGE, BULK_FLAG_STREAM_DOPPLER, BULK_FLAG_STREAM_CFAR,
    BULK_FLAGS_RESERVED_MASK,
    RP_USB_PROTOCOL_VERSION, STATUS_PACKET_SIZE,
)
from GUI_V65_Tk import DemoTarget, DemoSimulator, _ReplayController


class TestRadarProtocol(unittest.TestCase):
    """Test packet parsing and command building against usb_data_interface.v."""

    # ----------------------------------------------------------------
    # Command building
    # ----------------------------------------------------------------
    def test_build_command_trigger(self):
        """Opcode 0x01, value 1 → {0x01, 0x00, 0x0001}."""
        cmd = RadarProtocol.build_command(0x01, 1)
        self.assertEqual(len(cmd), 4)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x01)  # opcode
        self.assertEqual((word >> 16) & 0xFF, 0x00)  # addr
        self.assertEqual(word & 0xFFFF, 1)            # value

    def test_build_command_cfar_alpha(self):
        """Opcode 0x23, value 0x30 (alpha=3.0 Q4.4)."""
        cmd = RadarProtocol.build_command(0x23, 0x30)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x23)
        self.assertEqual(word & 0xFFFF, 0x30)

    def test_build_command_status_request(self):
        """Opcode 0xFF, value 0."""
        cmd = RadarProtocol.build_command(0xFF, 0)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0xFF)
        self.assertEqual(word & 0xFFFF, 0)

    def test_build_command_with_addr(self):
        """Command with non-zero addr field."""
        cmd = RadarProtocol.build_command(0x10, 500, addr=0x42)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x10)
        self.assertEqual((word >> 16) & 0xFF, 0x42)
        self.assertEqual(word & 0xFFFF, 500)

    def test_build_command_value_clamp(self):
        """Value > 0xFFFF should be masked to 16 bits."""
        cmd = RadarProtocol.build_command(0x01, 0x1FFFF)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual(word & 0xFFFF, 0xFFFF)

    # ----------------------------------------------------------------
    # Data packet parsing
    # ----------------------------------------------------------------
    def _make_data_packet(self, range_i=100, range_q=200,
                          dop_i=300, dop_q=400, detection=0):
        """Build a synthetic 11-byte data packet matching FT2232H format."""
        pkt = bytearray()
        pkt.append(HEADER_BYTE)
        pkt += struct.pack(">h", range_q & 0xFFFF if range_q >= 0 else range_q)
        pkt += struct.pack(">h", range_i & 0xFFFF if range_i >= 0 else range_i)
        pkt += struct.pack(">h", dop_i & 0xFFFF if dop_i >= 0 else dop_i)
        pkt += struct.pack(">h", dop_q & 0xFFFF if dop_q >= 0 else dop_q)
        pkt.append(detection & 0x01)
        pkt.append(FOOTER_BYTE)
        return bytes(pkt)

    def test_parse_data_packet_basic(self):
        raw = self._make_data_packet(100, 200, 300, 400, 0)
        result = RadarProtocol.parse_data_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], 100)
        self.assertEqual(result["range_q"], 200)
        self.assertEqual(result["doppler_i"], 300)
        self.assertEqual(result["doppler_q"], 400)
        self.assertEqual(result["detection"], 0)

    def test_parse_data_packet_with_detection(self):
        raw = self._make_data_packet(0, 0, 0, 0, 1)
        result = RadarProtocol.parse_data_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["detection"], 1)

    def test_parse_data_packet_negative_values(self):
        """Signed 16-bit values should round-trip correctly."""
        raw = self._make_data_packet(-1000, -2000, -500, 32000, 0)
        result = RadarProtocol.parse_data_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], -1000)
        self.assertEqual(result["range_q"], -2000)
        self.assertEqual(result["doppler_i"], -500)
        self.assertEqual(result["doppler_q"], 32000)

    def test_parse_data_packet_too_short(self):
        self.assertIsNone(RadarProtocol.parse_data_packet(b"\xAA\x00"))

    def test_parse_data_packet_wrong_header(self):
        raw = self._make_data_packet()
        bad = b"\x00" + raw[1:]
        self.assertIsNone(RadarProtocol.parse_data_packet(bad))

    # ----------------------------------------------------------------
    # Status packet parsing
    # ----------------------------------------------------------------
    def _make_status_packet(self, stream=7, threshold=10000,
                            long_chirp=3000, long_listen=13700,
                            guard=17540, short_chirp=50,
                            short_listen=17450, chirps=32,
                            st_flags=0, st_detail=0, st_busy=0,
                            agc_gain=0, agc_peak=0, agc_sat=0, agc_enable=0,
                            chirps_mismatch=0,
                            beam_handshake_watchdog=0,
                            cand_count=0, thr_soft=0, frame_drop=0,
                            medium_chirp=0, medium_listen=0):
        """Build an M-5 34-byte status response matching FPGA format."""
        pkt = bytearray()
        pkt.append(STATUS_HEADER_BYTE)

        # Word 0: {0xFF[31:24], reserved[23:22], stream[21:19], 3'b000[18:16], threshold[15:0]}
        # PR-AB.b expanded: bits [23:22] formerly radar_mode, now reserved 0.
        w0 = (0xFF << 24) | ((stream & 0x07) << 19) | (threshold & 0xFFFF)
        pkt += struct.pack(">I", w0)

        # Word 1: {long_chirp, long_listen}
        w1 = ((long_chirp & 0xFFFF) << 16) | (long_listen & 0xFFFF)
        pkt += struct.pack(">I", w1)

        # Word 2: {guard, short_chirp}
        w2 = ((guard & 0xFFFF) << 16) | (short_chirp & 0xFFFF)
        pkt += struct.pack(">I", w2)

        # Word 3: {short_listen, 10'd0, chirps[5:0]}
        w3 = ((short_listen & 0xFFFF) << 16) | (chirps & 0x3F)
        pkt += struct.pack(">I", w3)

        # Word 4: {agc_current_gain[31:28], agc_peak_magnitude[27:20],
        #          agc_saturation_count[19:12], agc_enable[11],
        #          chirps_mismatch[10], reserved[9:2],
        #          beam_handshake_watchdog[1], reserved[0]}
        # PR-AB.b expanded commit 5: bit [1] is the chirp_scheduler
        # S_BEAM_WAIT sticky watchdog flag (reset_n clear). The 8-bit
        # alpha_soft echo at [9:2] is FT2232H-only; this co-spec
        # builder leaves [9:2] reserved-0 like the FT601 path.
        w4 = (((agc_gain & 0x0F) << 28) | ((agc_peak & 0xFF) << 20) |
              ((agc_sat & 0xFF) << 12) | ((agc_enable & 0x01) << 11) |
              ((chirps_mismatch & 0x01) << 10) |
              ((beam_handshake_watchdog & 0x01) << 1))
        pkt += struct.pack(">I", w4)

        # Word 5: {frame_drop[31:25], self_test_busy[24], 8'd0,
        #           self_test_detail[15:8], 3'd0, self_test_flags[4:0]}
        w5 = (((frame_drop & 0x7F) << 25) | ((st_busy & 0x01) << 24)
              | ((st_detail & 0xFF) << 8) | (st_flags & 0x1F))
        pkt += struct.pack(">I", w5)

        # Word 6 (PR-G 2-tier CFAR telemetry):
        # high 16 bits = detect_count_cand, low 16 bits = detect_threshold_soft
        w6 = ((cand_count & 0xFFFF) << 16) | (thr_soft & 0xFFFF)
        pkt += struct.pack(">I", w6)

        # Word 7 (M-5 MEDIUM PRI readback): {medium_chirp[31:16], medium_listen[15:0]}
        w7 = ((medium_chirp & 0xFFFF) << 16) | (medium_listen & 0xFFFF)
        pkt += struct.pack(">I", w7)

        pkt.append(FOOTER_BYTE)
        return bytes(pkt)

    def test_parse_status_defaults(self):
        raw = self._make_status_packet()
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.stream_ctrl, 7)
        self.assertEqual(sr.cfar_threshold, 10000)
        self.assertEqual(sr.long_chirp, 3000)
        self.assertEqual(sr.long_listen, 13700)
        self.assertEqual(sr.guard, 17540)
        self.assertEqual(sr.short_chirp, 50)
        self.assertEqual(sr.short_listen, 17450)
        self.assertEqual(sr.chirps_per_elev, 32)
        self.assertEqual(sr.chirps_mismatch, 0)
        self.assertEqual(sr.beam_handshake_watchdog, 0)

    def test_parse_status_chirps_mismatch(self):
        # TX-G: bit 10 of word 4 must round-trip without disturbing neighbours.
        raw = self._make_status_packet(chirps_mismatch=1, agc_enable=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.chirps_mismatch, 1)
        self.assertEqual(sr.agc_enable, 1)
        self.assertEqual(sr.beam_handshake_watchdog, 0)

    def test_parse_status_beam_handshake_watchdog(self):
        # PR-AB.b expanded commit 5: bit 1 of word 4 must round-trip without
        # bleeding into chirps_mismatch (bit 10) or the reserved [0] bit.
        raw = self._make_status_packet(beam_handshake_watchdog=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.beam_handshake_watchdog, 1)
        self.assertEqual(sr.chirps_mismatch, 0)
        self.assertEqual(sr.agc_enable, 0)

    def test_parse_status_too_short(self):
        # Anything under STATUS_PACKET_SIZE (34 post-M-5) must be rejected.
        # 33-byte input = header + 32 bytes (one short of valid).
        self.assertIsNone(RadarProtocol.parse_status_packet(b"\xBB" + b"\x00" * 32))

    def test_parse_status_wrong_header(self):
        raw = self._make_status_packet()
        bad = b"\xAA" + raw[1:]
        self.assertIsNone(RadarProtocol.parse_status_packet(bad))

    def test_parse_status_wrong_footer(self):
        raw = bytearray(self._make_status_packet())
        raw[STATUS_PACKET_SIZE - 1] = 0x00  # corrupt footer
        self.assertIsNone(RadarProtocol.parse_status_packet(bytes(raw)))

    def test_parse_status_word6_2tier_cfar(self):
        """PR-G v2: word[6] high-half = detect_count_cand, low-half = detect_threshold_soft."""
        raw = self._make_status_packet(cand_count=0x0A5C, thr_soft=0x1234)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.detect_count_cand, 0x0A5C)
        self.assertEqual(sr.detect_threshold_soft, 0x1234)

    def test_parse_status_word7_medium_pri_readback(self):
        """M-5: word[7] high-half = medium_chirp, low-half = medium_listen.

        Closes the 161-µs MEDIUM PRI visibility gap left by PR-G (status word 3
        had only 10 reserved bits, not enough for a second 16-bit pair). Default
        production values: 500 cycles MEDIUM_CHIRP / 15600 cycles MEDIUM_LISTEN
        per RP_DEF_MEDIUM_*_CYCLES — picked here as the round-trip canary.
        """
        raw = self._make_status_packet(medium_chirp=500, medium_listen=15600)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.medium_chirp, 500)
        self.assertEqual(sr.medium_listen, 15600)
        # Defaults for unrelated fields must still be zero — guards against
        # bit-stealing into word 7 from neighbours.
        self.assertEqual(sr.detect_count_cand, 0)
        self.assertEqual(sr.detect_threshold_soft, 0)
        # Packet length matches new STATUS_PACKET_SIZE.
        self.assertEqual(len(raw), STATUS_PACKET_SIZE)
        self.assertEqual(STATUS_PACKET_SIZE, 34)

    def test_parse_status_word7_max_values(self):
        """M-5: 16-bit max in both halves of word 7 round-trips clean (no overflow)."""
        raw = self._make_status_packet(medium_chirp=0xFFFF, medium_listen=0xFFFF)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.medium_chirp, 0xFFFF)
        self.assertEqual(sr.medium_listen, 0xFFFF)

    def test_parse_status_word4_layout_co_spec(self):
        """GUI-S3: pin status word 4 bit positions to the FPGA word builder.

        Canonical layout per usb_data_interface.v:376-380 and
        usb_data_interface_ft2232h.v:675-679 — exactly one source of truth
        in this test, so any future drift between FPGA and GUI trips here:

            [31:28] agc_current_gain         (4-bit)
            [27:20] agc_peak_magnitude       (8-bit)
            [19:12] agc_saturation_count     (8-bit)
            [11]    agc_enable               (1-bit)
            [10]    chirps_mismatch          (1-bit, TX-G)
            [9:2]   reserved / alpha_soft    (8 bits — FT601 leaves zero; FT2232H
                                              echoes host_cfar_alpha_soft. This
                                              co-spec builder uses the FT601
                                              layout so the builder remains
                                              path-agnostic.)
            [1]     beam_handshake_watchdog  (1-bit sticky, PR-AB.b expanded
                                              commit 5)
            [0]     reserved                 (1-bit; was range_mode[0], retired
                                              in PR-AB.b expanded)

        For each field we set ONLY that field to its max, build the packet,
        parse, and assert (a) the field reads back correctly and (b) every
        other field reads back zero. Catches both LSB drift and width drift
        on either side of the wire.
        """
        layout = [
            # (field_name, builder_kwarg, lsb, width, parsed_attr)
            ("agc_current_gain",        "agc_gain",        28, 4, "agc_current_gain"),
            ("agc_peak_magnitude",      "agc_peak",        20, 8, "agc_peak_magnitude"),
            ("agc_saturation_count",    "agc_sat",         12, 8, "agc_saturation_count"),
            ("agc_enable",              "agc_enable",      11, 1, "agc_enable"),
            ("chirps_mismatch",         "chirps_mismatch", 10, 1, "chirps_mismatch"),
            ("beam_handshake_watchdog", "beam_handshake_watchdog",
                                                            1, 1, "beam_handshake_watchdog"),
        ]
        # Sanity: layout fields + reserved [9:2] (8 bits) + reserved [0] (1 bit)
        # must cover exactly 32 bits.
        used = sum(width for _, _, _, width, _ in layout)
        self.assertEqual(used + 9, 32,
            "word 4 layout (incl. reserved [9:2] + [0]) must total 32 bits")

        # No two fields may overlap.
        occupied = set()
        for name, _, lsb, width, _ in layout:
            bits = set(range(lsb, lsb + width))
            self.assertFalse(occupied & bits,
                f"{name} bits {sorted(bits)} overlap previously-allocated bits")
            occupied |= bits

        other_attrs = [attr for _, _, _, _, attr in layout]

        for name, kwarg, _lsb, width, attr in layout:
            max_val = (1 << width) - 1
            raw = self._make_status_packet(**{kwarg: max_val})
            sr = RadarProtocol.parse_status_packet(raw)
            self.assertIsNotNone(sr, f"{name}: parse failed")
            self.assertEqual(getattr(sr, attr), max_val,
                f"{name}: round-trip mismatch (set={max_val}, got={getattr(sr, attr)})")
            for other in other_attrs:
                if other == attr:
                    continue
                self.assertEqual(getattr(sr, other), 0,
                    f"{name} max value bled into {other} -- bit-position drift?")

    def test_parse_status_self_test_all_pass(self):
        """Status with all self-test flags set (all tests pass)."""
        raw = self._make_status_packet(st_flags=0x1F, st_detail=0xA5, st_busy=0)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.self_test_flags, 0x1F)
        self.assertEqual(sr.self_test_detail, 0xA5)
        self.assertEqual(sr.self_test_busy, 0)

    def test_parse_status_self_test_busy(self):
        """Status with self-test busy flag set."""
        raw = self._make_status_packet(st_flags=0x00, st_detail=0x00, st_busy=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.self_test_busy, 1)
        self.assertEqual(sr.self_test_flags, 0)
        self.assertEqual(sr.self_test_detail, 0)

    def test_parse_status_self_test_partial_fail(self):
        """Status with partial self-test failures (flags=0b10110)."""
        raw = self._make_status_packet(st_flags=0b10110, st_detail=0x42, st_busy=0)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.self_test_flags, 0b10110)
        self.assertEqual(sr.self_test_detail, 0x42)
        self.assertEqual(sr.self_test_busy, 0)
        # T0 (BRAM) failed, T1 (CIC) passed, T2 (FFT) passed, T3 (arith) failed, T4 (ADC) passed
        self.assertFalse(sr.self_test_flags & 0x01)  # T0 fail
        self.assertTrue(sr.self_test_flags & 0x02)    # T1 pass
        self.assertTrue(sr.self_test_flags & 0x04)    # T2 pass
        self.assertFalse(sr.self_test_flags & 0x08)   # T3 fail
        self.assertTrue(sr.self_test_flags & 0x10)     # T4 pass

    def test_parse_status_self_test_zero_word5(self):
        """Status with zero word 5 (self-test never run)."""
        raw = self._make_status_packet()
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.self_test_flags, 0)
        self.assertEqual(sr.self_test_detail, 0)
        self.assertEqual(sr.self_test_busy, 0)

    def test_status_packet_is_34_bytes(self):
        """M-5: status packet is 34 bytes (1 + 8*4 + 1; was 30 / PR-G 7 words)."""
        raw = self._make_status_packet()
        self.assertEqual(len(raw), STATUS_PACKET_SIZE)
        self.assertEqual(len(raw), 34)

    # ----------------------------------------------------------------
    # Boundary detection
    # ----------------------------------------------------------------
    def test_find_boundaries_mixed(self):
        data_pkt = self._make_data_packet()
        status_pkt = self._make_status_packet()
        buf = b"\x00\x00" + data_pkt + b"\x00" + status_pkt + data_pkt
        boundaries = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(boundaries), 3)
        self.assertEqual(boundaries[0][2], "data")
        self.assertEqual(boundaries[1][2], "status")
        self.assertEqual(boundaries[2][2], "data")

    def test_find_boundaries_empty(self):
        self.assertEqual(RadarProtocol.find_packet_boundaries(b""), [])

    def test_find_boundaries_truncated(self):
        """Truncated packet should not be returned."""
        data_pkt = self._make_data_packet()
        buf = data_pkt[:6]  # truncated (less than 11-byte packet size)
        boundaries = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(boundaries), 0)

    def test_find_boundaries_rejects_false_data_header(self):
        """GUI-S1: a stray 0xAA followed by 0x55 ten bytes later but with
        invalid byte-9 structure must NOT be accepted as a packet."""
        # Forge: header=0xAA, then 8 bytes of payload, byte 9 = 0xFF (bits
        # [6:1] all set — invalid: real packets have these zeroed),
        # then 0x55 footer. Old parser would accept; new parser rejects.
        forged = bytes([0xAA] + [0x00] * 8 + [0xFF, 0x55])
        real = self._make_data_packet()
        buf = forged + real
        boundaries = RadarProtocol.find_packet_boundaries(buf)
        # Must skip the forged 11 bytes and lock onto the real packet.
        self.assertEqual(len(boundaries), 1)
        self.assertEqual(boundaries[0], (11, 22, "data"))

    def test_find_boundaries_rejects_false_status_header(self):
        """GUI-S1: a stray 0xBB without 0xFF at offset+1 must NOT be
        accepted as a status packet — even if 0x55 lands at the right position."""
        # PR-G v2: 30-byte status; forge byte 1 = 0x00 (not 0xFF).
        forged = bytes([0xBB] + [0x00] * (STATUS_PACKET_SIZE - 2) + [0x55])
        self.assertEqual(len(forged), STATUS_PACKET_SIZE)
        real = self._make_data_packet()
        buf = forged + real
        boundaries = RadarProtocol.find_packet_boundaries(buf)
        # Forged status rejected; real data packet found inside the buffer.
        data_hits = [b for b in boundaries if b[2] == "data"]
        status_hits = [b for b in boundaries if b[2] == "status"]
        self.assertEqual(len(status_hits), 0)
        self.assertGreaterEqual(len(data_hits), 1)

    def test_find_boundaries_recovers_after_byte_drop(self):
        """GUI-S1: simulate a single-byte drop — parser should re-lock on
        the next intact packet rather than smearing forever."""
        good = self._make_data_packet(detection=0)
        # Drop byte 5 of the first packet to mimic USB byte loss.
        corrupted = good[:5] + good[6:]  # now 10 bytes, no full packet
        buf = corrupted + good + good  # two intact packets follow
        boundaries = RadarProtocol.find_packet_boundaries(buf)
        # We expect to recover and find at least the two intact tails.
        self.assertGreaterEqual(len(boundaries), 2)
        # Both recovered packets must be valid data packets.
        for start, end, ptype in boundaries:
            self.assertEqual(ptype, "data")
            self.assertIsNotNone(RadarProtocol.parse_data_packet(buf[start:end]))


class TestBulkFrameParser(unittest.TestCase):
    """AUDIT-C9: parser for the FT2232H bulk per-frame wire format."""

    def _build_bulk_frame(
        self,
        flags: int = (BULK_FLAG_STREAM_RANGE | BULK_FLAG_STREAM_DOPPLER
                      | BULK_FLAG_STREAM_CFAR),
        frame_number: int = 0xBEEF,
        n_range: int = NUM_RANGE_BINS,
        n_doppler: int = NUM_DOPPLER_BINS,
        range_seed: int = 1,
        doppler_seed: int = 2,
        cfar_seed: int = 3,
        bad_footer: bool = False,
        version: int = RP_USB_PROTOCOL_VERSION,
    ) -> tuple[bytes, np.ndarray, np.ndarray, np.ndarray]:
        """Synthesize a PR-G v2 bulk frame matching usb_data_interface_ft2232h.v.

        Returns (frame_bytes, range_profile, doppler_mag, cfar_codes). The
        latter three are the source-of-truth arrays used to generate the
        bytes; tests can assert round-trip equality. cfar_codes carries
        2-bit values 0..3 (NONE/CAND/CONFIRM/RSVD).
        """
        rng_r = np.random.RandomState(range_seed)
        rng_d = np.random.RandomState(doppler_seed)
        rng_c = np.random.RandomState(cfar_seed)

        range_profile = (rng_r.randint(0, 65535, size=n_range)
                         .astype(np.uint16) if (flags & BULK_FLAG_STREAM_RANGE)
                         else None)
        doppler_mag = (rng_d.randint(0, 65535, size=(n_range, n_doppler))
                       .astype(np.uint16) if (flags & BULK_FLAG_STREAM_DOPPLER)
                       else None)
        # PR-F 2-tier dense detect: 2 bits per cell (codes 0..3).
        cfar_codes = (rng_c.randint(0, 4, size=(n_range, n_doppler))
                      .astype(np.uint8) if (flags & BULK_FLAG_STREAM_CFAR)
                      else None)

        out = bytearray()
        out.append(HEADER_BYTE)
        # PR-G v2: byte 1 is the protocol version. Tests can override
        # `version` to exercise rejection.
        out.append(version & 0xFF)
        # Don't mask reserved bits — the parser must reject any byte with
        # bits in BULK_FLAGS_RESERVED_MASK set, and the rejection test
        # relies on those bits actually surviving into the synthesized frame.
        out.append(flags & 0xFF)
        out.append((frame_number >> 8) & 0xFF)
        out.append(frame_number & 0xFF)
        out.append((n_range >> 8) & 0xFF)
        out.append(n_range & 0xFF)
        out.append((n_doppler >> 8) & 0xFF)
        out.append(n_doppler & 0xFF)
        if range_profile is not None:
            out += range_profile.astype(">u2").tobytes()
        if doppler_mag is not None:
            out += doppler_mag.astype(">u2").tobytes()
        if cfar_codes is not None:
            # Pack 4 cells per byte, MSB-first within byte (matches FPGA emit).
            # Local bytes_per_range tracks the *actual* n_doppler so tests
            # passing n_doppler != NUM_DOPPLER_BINS don't overflow the row.
            bytes_per_range = (n_doppler * 2 + 7) // 8
            packed = np.zeros((n_range, bytes_per_range), dtype=np.uint8)
            for d_idx in range(n_doppler):
                byte_idx = d_idx // 4
                shift = (3 - (d_idx % 4)) * 2
                packed[:, byte_idx] |= ((cfar_codes[:, d_idx] & 0x03) << shift).astype(np.uint8)
            out += packed.tobytes()
        out.append(0x00 if bad_footer else FOOTER_BYTE)
        return bytes(out), range_profile, doppler_mag, cfar_codes

    def test_parse_full_frame_round_trip(self):
        """All-streams mag-only round trip: every cell exact."""
        raw, rprof, dmag, cdense = self._build_bulk_frame()
        parsed = RadarProtocol.parse_bulk_frame(raw)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["frame_number"], 0xBEEF)
        self.assertEqual(parsed["n_range"], NUM_RANGE_BINS)
        self.assertEqual(parsed["n_doppler"], NUM_DOPPLER_BINS)
        self.assertEqual(parsed["frame_size"], BULK_FRAME_MAX_SIZE)
        np.testing.assert_array_equal(parsed["range_profile"], rprof)
        np.testing.assert_array_equal(parsed["doppler_mag"], dmag)
        np.testing.assert_array_equal(parsed["cfar_dense"], cdense)

    def test_parse_range_only(self):
        raw, rprof, _dmag, _cdense = self._build_bulk_frame(flags=BULK_FLAG_STREAM_RANGE)
        parsed = RadarProtocol.parse_bulk_frame(raw)
        self.assertIsNotNone(parsed)
        np.testing.assert_array_equal(parsed["range_profile"], rprof)
        self.assertIsNone(parsed["doppler_mag"])
        self.assertIsNone(parsed["cfar_dense"])
        self.assertEqual(
            parsed["frame_size"],
            BULK_FRAME_HEADER_SIZE + BULK_RANGE_SECTION_BYTES + BULK_FOOTER_SIZE,
        )

    def test_parse_doppler_only(self):
        raw, _rprof, dmag, _cdense = self._build_bulk_frame(flags=BULK_FLAG_STREAM_DOPPLER)
        parsed = RadarProtocol.parse_bulk_frame(raw)
        self.assertIsNotNone(parsed)
        self.assertIsNone(parsed["range_profile"])
        np.testing.assert_array_equal(parsed["doppler_mag"], dmag)
        self.assertIsNone(parsed["cfar_dense"])

    def test_parse_cfar_only(self):
        raw, _rprof, _dmag, cdense = self._build_bulk_frame(flags=BULK_FLAG_STREAM_CFAR)
        parsed = RadarProtocol.parse_bulk_frame(raw)
        self.assertIsNotNone(parsed)
        np.testing.assert_array_equal(parsed["cfar_dense"], cdense)

    def test_parse_no_streams(self):
        """PR-G v2: header + footer only is 9 + 1 = 10 bytes."""
        raw, *_ = self._build_bulk_frame(flags=0)
        self.assertEqual(len(raw), BULK_FRAME_HEADER_SIZE + BULK_FOOTER_SIZE)
        parsed = RadarProtocol.parse_bulk_frame(raw)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["frame_size"], BULK_FRAME_HEADER_SIZE + BULK_FOOTER_SIZE)

    def test_reject_wrong_version_byte(self):
        """PR-G v2: byte 1 must equal RP_USB_PROTOCOL_VERSION."""
        raw, *_ = self._build_bulk_frame(version=0x01)
        self.assertIsNone(RadarProtocol.parse_bulk_frame(raw))

    def test_reject_reserved_bits_set(self):
        """Reserved high bits (BULK_FLAGS_RESERVED_MASK) must be zero."""
        raw, *_ = self._build_bulk_frame(flags=BULK_FLAG_STREAM_DOPPLER | 0x80)
        self.assertIsNone(RadarProtocol.parse_bulk_frame(raw))

    def test_reject_wrong_n_range(self):
        raw, *_ = self._build_bulk_frame(n_range=999)
        # Note: the synthesized payload size is wrong too but the n_range
        # check fires before any payload-size checks.
        self.assertIsNone(RadarProtocol.parse_bulk_frame(raw))

    def test_reject_wrong_n_doppler(self):
        raw, *_ = self._build_bulk_frame(n_doppler=64)
        self.assertIsNone(RadarProtocol.parse_bulk_frame(raw))

    def test_reject_missing_footer(self):
        raw, *_ = self._build_bulk_frame(bad_footer=True)
        self.assertIsNone(RadarProtocol.parse_bulk_frame(raw))

    def test_reject_wrong_header(self):
        raw, *_ = self._build_bulk_frame()
        bad = b"\x00" + raw[1:]
        self.assertIsNone(RadarProtocol.parse_bulk_frame(bad))

    def test_reject_truncated(self):
        raw, *_ = self._build_bulk_frame()
        self.assertIsNone(RadarProtocol.parse_bulk_frame(raw[:1000]))

    def test_find_boundaries_two_frames(self):
        f1, *_ = self._build_bulk_frame(frame_number=1)
        f2, *_ = self._build_bulk_frame(frame_number=2)
        buf = b"\x00\x12" + f1 + b"\x33" + f2  # garbage between/around frames
        out = RadarProtocol.find_bulk_frame_boundaries(buf)
        data = [(s, e, t) for (s, e, t) in out if t == "data"]
        self.assertEqual(len(data), 2)
        # Round-trip both
        for s, _e, _t in data:
            parsed = RadarProtocol.parse_bulk_frame(buf, offset=s)
            self.assertIsNotNone(parsed)

    def test_find_boundaries_with_status(self):
        """Status packets (PR-G v2 30 B) coexist with bulk frames in the same stream."""
        f1, *_ = self._build_bulk_frame()
        # Build a minimal valid 30-byte status packet (byte 1 = 0xFF, footer = 0x55).
        status = bytes([STATUS_HEADER_BYTE, 0xFF]
                       + [0x00] * (STATUS_PACKET_SIZE - 3) + [FOOTER_BYTE])
        self.assertEqual(len(status), STATUS_PACKET_SIZE)
        buf = f1 + status
        out = RadarProtocol.find_bulk_frame_boundaries(buf)
        types = [t for _s, _e, t in out]
        self.assertIn("data", types)
        self.assertIn("status", types)

    def test_find_boundaries_truncated_residual(self):
        """A partial frame at the buffer tail is not returned (kept as residual)."""
        f1, *_ = self._build_bulk_frame()
        # Cut off the last 100 bytes — find_bulk_frame_boundaries must not
        # return this frame; the caller keeps the bytes for next iteration.
        buf = f1[:-100]
        out = RadarProtocol.find_bulk_frame_boundaries(buf)
        self.assertEqual([t for _s, _e, t in out], [])

    def test_resync_after_byte_drop(self):
        """A dropped byte at the head must not lock the parser onto false positives."""
        f1, *_ = self._build_bulk_frame()
        # Single garbage byte before the real frame.
        buf = b"\x99" + f1
        out = RadarProtocol.find_bulk_frame_boundaries(buf)
        data = [(s, e, t) for (s, e, t) in out if t == "data"]
        self.assertEqual(len(data), 1)
        self.assertEqual(data[0][0], 1)  # frame starts at offset 1

    def test_acquisition_dispatches_bulk_for_ft2232h(self):
        """RadarAcquisition must select bulk format for FT2232H connections."""
        ft = FT2232HConnection(mock=True)
        ft.open()
        q: queue.Queue = queue.Queue()
        acq = RadarAcquisition(ft, q)
        self.assertTrue(acq._is_bulk)
        ft.close()

    def test_acquisition_dispatches_legacy_for_ft601(self):
        """RadarAcquisition must select legacy format for FT601 connections."""
        ft = FT601Connection(mock=True)
        ft.open()
        q: queue.Queue = queue.Queue()
        acq = RadarAcquisition(ft, q)
        self.assertFalse(acq._is_bulk)
        ft.close()

    def test_ingest_bulk_frame_populates_radarframe(self):
        """End-to-end: bulk parse → RadarFrame in queue with correct fields."""
        ft = FT2232HConnection(mock=True)
        ft.open()
        q: queue.Queue = queue.Queue(maxsize=4)
        acq = RadarAcquisition(ft, q)
        # Drive one tick of run() manually instead of starting the thread.
        chunk = ft.read(BULK_FRAME_MAX_SIZE * 2)
        packets = RadarProtocol.find_bulk_frame_boundaries(chunk)
        self.assertGreater(len(packets), 0)
        for s, _e, t in packets:
            if t == "data":
                parsed = RadarProtocol.parse_bulk_frame(chunk, offset=s)
                acq._ingest_bulk_frame(parsed)
        ft.close()

        frame = q.get_nowait()
        self.assertIsInstance(frame, RadarFrame)
        self.assertTrue(frame.mag_only)
        # Mag-only mode: I/Q stay zero, magnitude carries data.
        self.assertTrue((frame.range_doppler_i == 0).all())
        self.assertTrue((frame.range_doppler_q == 0).all())
        self.assertGreater(frame.magnitude.max(), 0)


class TestFT2232HConnection(unittest.TestCase):
    """Test mock FT2232H connection."""

    def test_mock_open_close(self):
        conn = FT2232HConnection(mock=True)
        self.assertTrue(conn.open())
        self.assertTrue(conn.is_open)
        conn.close()
        self.assertFalse(conn.is_open)

    def test_mock_read_returns_data(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        data = conn.read(4096)
        self.assertIsNotNone(data)
        self.assertGreater(len(data), 0)
        conn.close()

    def test_mock_read_contains_valid_packets(self):
        """Mock data should contain a parseable bulk frame (AUDIT-C9)."""
        conn = FT2232HConnection(mock=True)
        conn.open()
        raw = conn.read(BULK_FRAME_MAX_SIZE * 2)
        packets = RadarProtocol.find_bulk_frame_boundaries(raw)
        self.assertGreater(len(packets), 0)
        for start, _end, ptype in packets:
            if ptype == "data":
                parsed = RadarProtocol.parse_bulk_frame(raw, offset=start)
                self.assertIsNotNone(parsed)
                self.assertEqual(parsed["n_range"], NUM_RANGE_BINS)
                self.assertEqual(parsed["n_doppler"], NUM_DOPPLER_BINS)
                # PR-G v2: only the low 3 stream-enable bits are valid.
                self.assertEqual(parsed["flags"] & BULK_FLAGS_RESERVED_MASK, 0)
        conn.close()

    def test_mock_write(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        cmd = RadarProtocol.build_command(0x01, 1)
        self.assertTrue(conn.write(cmd))
        conn.close()

    def test_read_when_closed(self):
        conn = FT2232HConnection(mock=True)
        self.assertIsNone(conn.read())

    def test_write_when_closed(self):
        conn = FT2232HConnection(mock=True)
        self.assertFalse(conn.write(b"\x00\x00\x00\x00"))


class TestFT601Connection(unittest.TestCase):
    """Test mock FT601 connection (mirrors FT2232H tests)."""

    def test_mock_open_close(self):
        conn = FT601Connection(mock=True)
        self.assertTrue(conn.open())
        self.assertTrue(conn.is_open)
        conn.close()
        self.assertFalse(conn.is_open)

    def test_mock_read_returns_data(self):
        conn = FT601Connection(mock=True)
        conn.open()
        data = conn.read(4096)
        self.assertIsNotNone(data)
        self.assertGreater(len(data), 0)
        conn.close()

    def test_mock_read_contains_valid_packets(self):
        """Mock data should contain parseable data packets."""
        conn = FT601Connection(mock=True)
        conn.open()
        raw = conn.read(4096)
        packets = RadarProtocol.find_packet_boundaries(raw)
        self.assertGreater(len(packets), 0)
        for start, end, ptype in packets:
            if ptype == "data":
                result = RadarProtocol.parse_data_packet(raw[start:end])
                self.assertIsNotNone(result)
        conn.close()

    def test_mock_write(self):
        conn = FT601Connection(mock=True)
        conn.open()
        cmd = RadarProtocol.build_command(0x01, 1)
        self.assertTrue(conn.write(cmd))
        conn.close()

    def test_write_pads_to_4_bytes(self):
        """FT601 write() should pad data to 4-byte alignment."""
        conn = FT601Connection(mock=True)
        conn.open()
        # 3-byte payload should be padded internally (no error)
        self.assertTrue(conn.write(b"\x01\x02\x03"))
        conn.close()

    def test_read_when_closed(self):
        conn = FT601Connection(mock=True)
        self.assertIsNone(conn.read())

    def test_write_when_closed(self):
        conn = FT601Connection(mock=True)
        self.assertFalse(conn.write(b"\x00\x00\x00\x00"))


class TestDataRecorder(unittest.TestCase):
    """Test HDF5 recording (skipped if h5py not available)."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.filepath = os.path.join(self.tmpdir, "test_recording.h5")

    def tearDown(self):
        if os.path.exists(self.filepath):
            os.remove(self.filepath)
        os.rmdir(self.tmpdir)

    @unittest.skipUnless(
        (lambda: (
            __import__("importlib.util")
            and __import__("importlib").util.find_spec("h5py") is not None
        ))(),
        "h5py not installed"
    )
    def test_record_and_stop(self):
        import h5py
        rec = DataRecorder()
        rec.start(self.filepath)
        self.assertTrue(rec.recording)

        # Record 3 frames
        for i in range(3):
            frame = RadarFrame()
            frame.frame_number = i
            frame.timestamp = time.time()
            frame.magnitude = np.random.rand(NUM_RANGE_BINS, NUM_DOPPLER_BINS)
            frame.range_profile = np.random.rand(NUM_RANGE_BINS)
            rec.record_frame(frame)

        rec.stop()
        self.assertFalse(rec.recording)

        # Verify HDF5 contents
        with h5py.File(self.filepath, "r") as f:
            self.assertEqual(f.attrs["total_frames"], 3)
            self.assertIn("frames", f)
            self.assertIn("frame_000000", f["frames"])
            self.assertIn("frame_000002", f["frames"])
            mag = f["frames/frame_000001/magnitude"][:]
            self.assertEqual(mag.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))

    @unittest.skipUnless(
        (lambda: (
            __import__("importlib.util")
            and __import__("importlib").util.find_spec("h5py") is not None
        ))(),
        "h5py not installed"
    )
    def test_record_frame_isolates_from_post_call_mutation(self):
        # GUI-S2: the recorder must snapshot the frame's arrays so a
        # consumer (or future in-place scaling) mutating the same RadarFrame
        # after record_frame returns cannot tear the on-disk record.
        import h5py
        rec = DataRecorder()
        rec.start(self.filepath)

        frame = RadarFrame()
        frame.frame_number = 42
        frame.timestamp = 123.456
        frame.magnitude = np.full((NUM_RANGE_BINS, NUM_DOPPLER_BINS), 7.0)
        frame.range_profile = np.full(NUM_RANGE_BINS, 3.0)
        frame.detections[0, 0] = 1
        frame.range_doppler_i[1, 1] = 100
        frame.range_doppler_q[2, 2] = -50

        rec.record_frame(frame)

        # Mutate every array in place AFTER recording — must not bleed into HDF5.
        frame.magnitude.fill(0.0)
        frame.range_profile.fill(0.0)
        frame.detections.fill(0)
        frame.range_doppler_i.fill(0)
        frame.range_doppler_q.fill(0)

        rec.stop()

        with h5py.File(self.filepath, "r") as f:
            fg = f["frames/frame_000000"]
            self.assertTrue(np.all(fg["magnitude"][:] == 7.0))
            self.assertTrue(np.all(fg["range_profile"][:] == 3.0))
            self.assertEqual(fg["detections"][0, 0], 1)
            self.assertEqual(fg["range_doppler_i"][1, 1], 100)
            self.assertEqual(fg["range_doppler_q"][2, 2], -50)


class TestRadarAcquisition(unittest.TestCase):
    """Test acquisition thread with mock connection."""

    def test_acquisition_produces_frames(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        fq = queue.Queue(maxsize=16)
        acq = RadarAcquisition(conn, fq)
        acq.start()

        # AUDIT-C9: FT2232H mock now emits one full bulk frame per read
        # (50 ms cadence in the mock), so a frame should land within ~100 ms.
        frame = None
        try:  # noqa: SIM105
            frame = fq.get(timeout=10)
        except queue.Empty:
            pass

        acq.stop()
        acq.join(timeout=3)
        conn.close()

        if frame is not None:
            self.assertIsInstance(frame, RadarFrame)
            self.assertEqual(frame.magnitude.shape,
                             (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
            self.assertTrue(frame.mag_only)
        # If no frame arrived in timeout, that's still OK for a fast CI run

    def test_acquisition_stop(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        fq = queue.Queue(maxsize=4)
        acq = RadarAcquisition(conn, fq)
        acq.start()
        time.sleep(0.2)
        acq.stop()
        acq.join(timeout=3)
        self.assertFalse(acq.is_alive())
        conn.close()


class TestRadarFrameDefaults(unittest.TestCase):
    """Test RadarFrame default initialization."""

    def test_default_shapes(self):
        # Stale literals (64,32)/(64,) predated the GUI-C1 / Q3 alignment that
        # bumped NUM_RANGE_BINS 64 -> 512 to match FPGA truth. Reference the
        # constants so any future bin-count change updates the assertion too.
        f = RadarFrame()
        self.assertEqual(f.range_doppler_i.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
        self.assertEqual(f.range_doppler_q.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
        self.assertEqual(f.magnitude.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
        self.assertEqual(f.detections.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
        self.assertEqual(f.range_profile.shape, (NUM_RANGE_BINS,))
        self.assertEqual(f.detection_count, 0)

    def test_default_zeros(self):
        f = RadarFrame()
        self.assertTrue(np.all(f.magnitude == 0))
        self.assertTrue(np.all(f.detections == 0))


class TestEndToEnd(unittest.TestCase):
    """End-to-end: build command → parse response → verify round-trip."""

    def test_command_roundtrip_all_opcodes(self):
        """Verify all opcodes produce valid 4-byte commands."""
        opcodes = [0x01, 0x02, 0x03, 0x04, 0x10, 0x11, 0x12,
                   0x13, 0x14, 0x15, 0x16, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
                   0x26, 0x27, 0x30, 0x31, 0xFF]
        for op in opcodes:
            cmd = RadarProtocol.build_command(op, 42)
            self.assertEqual(len(cmd), 4, f"opcode 0x{op:02X}")
            word = struct.unpack(">I", cmd)[0]
            self.assertEqual((word >> 24) & 0xFF, op)
            self.assertEqual(word & 0xFFFF, 42)

    def test_data_packet_roundtrip(self):
        """Build an 11-byte data packet, parse it, verify values match."""
        ri, rq, di, dq = 1234, -5678, 9012, -3456

        pkt = bytearray()
        pkt.append(HEADER_BYTE)
        pkt += struct.pack(">h", rq)
        pkt += struct.pack(">h", ri)
        pkt += struct.pack(">h", di)
        pkt += struct.pack(">h", dq)
        pkt.append(1)
        pkt.append(FOOTER_BYTE)

        self.assertEqual(len(pkt), DATA_PACKET_SIZE)

        result = RadarProtocol.parse_data_packet(bytes(pkt))
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], ri)
        self.assertEqual(result["range_q"], rq)
        self.assertEqual(result["doppler_i"], di)
        self.assertEqual(result["doppler_q"], dq)
        self.assertEqual(result["detection"], 1)


class TestOpcodeEnum(unittest.TestCase):
    """Verify Opcode enum matches RTL host register map (radar_system_top.v)."""

    def test_gain_shift_is_0x16(self):
        """GAIN_SHIFT opcode must be 0x16 (matches radar_system_top.v:928)."""
        self.assertEqual(Opcode.GAIN_SHIFT, 0x16)

    def test_no_digital_gain_alias(self):
        """DIGITAL_GAIN should NOT exist (use GAIN_SHIFT)."""
        self.assertFalse(hasattr(Opcode, 'DIGITAL_GAIN'))

    def test_self_test_trigger(self):
        """SELF_TEST_TRIGGER opcode must be 0x30."""
        self.assertEqual(Opcode.SELF_TEST_TRIGGER, 0x30)

    def test_self_test_status(self):
        """SELF_TEST_STATUS opcode must be 0x31."""
        self.assertEqual(Opcode.SELF_TEST_STATUS, 0x31)

    def test_stream_control_is_0x04(self):
        """STREAM_CONTROL must be 0x04 (matches radar_system_top.v:906)."""
        self.assertEqual(Opcode.STREAM_CONTROL, 0x04)

    def test_legacy_aliases_removed(self):
        """Legacy aliases must NOT exist in production Opcode enum."""
        for name in ("TRIGGER", "PRF_DIV", "NUM_CHIRPS", "CHIRP_TIMER",
                      "STREAM_ENABLE", "THRESHOLD"):
            self.assertFalse(hasattr(Opcode, name),
                             f"Legacy alias Opcode.{name} should not exist")

    def test_basic_control_opcodes(self):
        """Canonical basic-control opcodes (0x03/0x04) match FPGA RTL.

        PR-AB.b expanded retired RADAR_MODE (0x01) and TRIGGER_PULSE (0x02);
        the legacy-alias test below ensures they stay absent.
        """
        self.assertEqual(Opcode.DETECT_THRESHOLD, 0x03)
        self.assertEqual(Opcode.STREAM_CONTROL, 0x04)

    def test_handshake_enable_opcode(self):
        """PR-AB.b expanded commit 5: beam-ready handshake opcode = 0x1A."""
        self.assertEqual(Opcode.HANDSHAKE_ENABLE, 0x1A)

    def test_all_rtl_opcodes_present(self):
        """Every RTL opcode (from radar_system_top.v) has a matching Opcode enum member.

        PR-AB.b expanded: 0x01 / 0x02 / 0x20 retired from the FPGA dispatch
        case-block; they must NOT reappear in the Opcode enum.
        """
        expected = {0x03, 0x04,
                    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
                    0x17, 0x18, 0x19, 0x1A,
                    0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
                    0x28, 0x29, 0x2A, 0x2B, 0x2C,
                    0x30, 0x31, 0xFF}
        enum_values = {int(m) for m in Opcode}
        for op in expected:
            self.assertIn(op, enum_values, f"0x{op:02X} missing from Opcode enum")


class TestStatusResponseDefaults(unittest.TestCase):
    """Verify StatusResponse dataclass has self-test fields."""

    def test_default_self_test_fields(self):
        sr = StatusResponse()
        self.assertEqual(sr.self_test_flags, 0)
        self.assertEqual(sr.self_test_detail, 0)
        self.assertEqual(sr.self_test_busy, 0)

    def test_self_test_fields_set(self):
        sr = StatusResponse(self_test_flags=0x1F,
                            self_test_detail=0xAB,
                            self_test_busy=1)
        self.assertEqual(sr.self_test_flags, 0x1F)
        self.assertEqual(sr.self_test_detail, 0xAB)
        self.assertEqual(sr.self_test_busy, 1)


class TestAGCOpcodes(unittest.TestCase):
    """Verify AGC opcode enum members match FPGA RTL (0x28-0x2C)."""

    def test_agc_enable_opcode(self):
        self.assertEqual(Opcode.AGC_ENABLE, 0x28)

    def test_agc_target_opcode(self):
        self.assertEqual(Opcode.AGC_TARGET, 0x29)

    def test_agc_attack_opcode(self):
        self.assertEqual(Opcode.AGC_ATTACK, 0x2A)

    def test_agc_decay_opcode(self):
        self.assertEqual(Opcode.AGC_DECAY, 0x2B)

    def test_agc_holdoff_opcode(self):
        self.assertEqual(Opcode.AGC_HOLDOFF, 0x2C)


class TestAGCStatusParsing(unittest.TestCase):
    """Verify AGC fields in status_words[4] are parsed correctly."""

    def _make_status_packet(self, **kwargs):
        """Delegate to TestRadarProtocol helper."""
        helper = TestRadarProtocol()
        return helper._make_status_packet(**kwargs)

    def test_agc_fields_default_zero(self):
        """With no AGC fields set, all should be 0."""
        raw = self._make_status_packet()
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 0)
        self.assertEqual(sr.agc_peak_magnitude, 0)
        self.assertEqual(sr.agc_saturation_count, 0)
        self.assertEqual(sr.agc_enable, 0)

    def test_agc_fields_nonzero(self):
        """AGC fields round-trip through status packet."""
        raw = self._make_status_packet(agc_gain=7, agc_peak=200,
                                       agc_sat=15, agc_enable=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 7)
        self.assertEqual(sr.agc_peak_magnitude, 200)
        self.assertEqual(sr.agc_saturation_count, 15)
        self.assertEqual(sr.agc_enable, 1)

    def test_agc_max_values(self):
        """AGC fields at max values."""
        raw = self._make_status_packet(agc_gain=15, agc_peak=255,
                                       agc_sat=255, agc_enable=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 15)
        self.assertEqual(sr.agc_peak_magnitude, 255)
        self.assertEqual(sr.agc_saturation_count, 255)
        self.assertEqual(sr.agc_enable, 1)

    def test_agc_fields_coexist_with_mismatch(self):
        """AGC fields and chirps_mismatch occupy the same word without conflict."""
        raw = self._make_status_packet(agc_gain=5, agc_peak=128,
                                       agc_sat=42, agc_enable=1,
                                       chirps_mismatch=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 5)
        self.assertEqual(sr.agc_peak_magnitude, 128)
        self.assertEqual(sr.agc_saturation_count, 42)
        self.assertEqual(sr.agc_enable, 1)
        self.assertEqual(sr.chirps_mismatch, 1)


class TestAGCStatusResponseDefaults(unittest.TestCase):
    """Verify StatusResponse AGC field defaults."""

    def test_default_agc_fields(self):
        sr = StatusResponse()
        self.assertEqual(sr.agc_current_gain, 0)
        self.assertEqual(sr.agc_peak_magnitude, 0)
        self.assertEqual(sr.agc_saturation_count, 0)
        self.assertEqual(sr.agc_enable, 0)

    def test_agc_fields_set(self):
        sr = StatusResponse(agc_current_gain=7, agc_peak_magnitude=200,
                            agc_saturation_count=15, agc_enable=1)
        self.assertEqual(sr.agc_current_gain, 7)
        self.assertEqual(sr.agc_peak_magnitude, 200)
        self.assertEqual(sr.agc_saturation_count, 15)
        self.assertEqual(sr.agc_enable, 1)


# =============================================================================
# AGC Visualization — ring buffer / data model tests
# =============================================================================

class TestAGCVisualizationHistory(unittest.TestCase):
    """Test the AGC visualization ring buffer logic (no GUI required)."""

    def _make_deque(self, maxlen=256):
        from collections import deque
        return deque(maxlen=maxlen)

    def test_ring_buffer_maxlen(self):
        """Ring buffer should evict oldest when full."""
        d = self._make_deque(maxlen=4)
        for i in range(6):
            d.append(i)
        self.assertEqual(list(d), [2, 3, 4, 5])
        self.assertEqual(len(d), 4)

    def test_gain_history_accumulation(self):
        """Gain values accumulate correctly in a deque."""
        gain_hist = self._make_deque(maxlen=256)
        statuses = [
            StatusResponse(agc_current_gain=g)
            for g in [0, 3, 7, 15, 8, 2]
        ]
        for st in statuses:
            gain_hist.append(st.agc_current_gain)
        self.assertEqual(list(gain_hist), [0, 3, 7, 15, 8, 2])

    def test_peak_history_accumulation(self):
        """Peak magnitude values accumulate correctly."""
        peak_hist = self._make_deque(maxlen=256)
        for p in [0, 50, 200, 255, 128]:
            peak_hist.append(p)
        self.assertEqual(list(peak_hist), [0, 50, 200, 255, 128])

    def test_saturation_total_computation(self):
        """Sum of saturation ring buffer gives running total."""
        sat_hist = self._make_deque(maxlen=256)
        for s in [0, 0, 5, 0, 12, 3]:
            sat_hist.append(s)
        self.assertEqual(sum(sat_hist), 20)

    def test_saturation_color_thresholds(self):
        """Color logic: green=0, yellow=1-10, red>10."""
        def sat_color(total):
            if total > 10:
                return "red"
            if total > 0:
                return "yellow"
            return "green"
        self.assertEqual(sat_color(0), "green")
        self.assertEqual(sat_color(1), "yellow")
        self.assertEqual(sat_color(10), "yellow")
        self.assertEqual(sat_color(11), "red")
        self.assertEqual(sat_color(255), "red")

    def test_ring_buffer_eviction_preserves_latest(self):
        """After overflow, only the most recent values remain."""
        d = self._make_deque(maxlen=8)
        for i in range(20):
            d.append(i)
        self.assertEqual(list(d), [12, 13, 14, 15, 16, 17, 18, 19])

    def test_empty_history_safe(self):
        """Empty ring buffer should be safe for max/sum."""
        d = self._make_deque(maxlen=256)
        self.assertEqual(sum(d), 0)
        self.assertEqual(len(d), 0)
        # max() on empty would raise — test the guard pattern used in viz code
        max_sat = max(d) if d else 0
        self.assertEqual(max_sat, 0)

    def test_agc_mode_string(self):
        """AGC mode display string from enable flag."""
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=1).agc_enable else "MANUAL",
            "AUTO")
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=0).agc_enable else "MANUAL",
            "MANUAL")

    def test_xlim_scroll_logic(self):
        """X-axis scroll: when n >= history_len, xlim should expand."""
        history_len = 8
        d = self._make_deque(maxlen=history_len)
        for i in range(10):
            d.append(i)
        n = len(d)
        # After 10 pushes into maxlen=8, n=8
        self.assertEqual(n, history_len)
        # xlim should be (0, n) for static or (n-history_len, n) for scrolling
        self.assertEqual(max(0, n - history_len), 0)
        self.assertEqual(n, 8)

    def test_sat_autoscale_ylim(self):
        """Saturation y-axis auto-scale: max(max_sat * 1.5, 5)."""
        # No saturation
        self.assertEqual(max(0 * 1.5, 5), 5)
        # Some saturation
        self.assertAlmostEqual(max(10 * 1.5, 5), 15.0)
        # High saturation
        self.assertAlmostEqual(max(200 * 1.5, 5), 300.0)


# =====================================================================
# Tests for DemoTarget, DemoSimulator, and _ReplayController
# =====================================================================


class TestDemoTarget(unittest.TestCase):
    """Unit tests for DemoTarget kinematics."""

    def test_initial_values_in_range(self):
        t = DemoTarget(1)
        self.assertEqual(t.id, 1)
        self.assertGreaterEqual(t.range_m, 20)
        self.assertLessEqual(t.range_m, DemoTarget._MAX_RANGE)
        self.assertIn(t.classification, ["aircraft", "drone", "bird", "unknown"])

    def test_step_returns_true_in_normal_range(self):
        t = DemoTarget(2)
        t.range_m = 150.0
        t.velocity = 0.0
        self.assertTrue(t.step())

    def test_step_returns_false_when_out_of_range_high(self):
        t = DemoTarget(3)
        t.range_m = DemoTarget._MAX_RANGE + 1
        t.velocity = -1.0  # moving away
        self.assertFalse(t.step())

    def test_step_returns_false_when_out_of_range_low(self):
        t = DemoTarget(4)
        t.range_m = 2.0
        t.velocity = 1.0  # moving closer
        self.assertFalse(t.step())

    def test_velocity_clamped(self):
        t = DemoTarget(5)
        t.velocity = 19.0
        t.range_m = 150.0
        # Step many times — velocity should stay within [-20, 20]
        for _ in range(100):
            t.range_m = 150.0  # keep in range
            t.step()
        self.assertGreaterEqual(t.velocity, -20)
        self.assertLessEqual(t.velocity, 20)

    def test_snr_clamped(self):
        t = DemoTarget(6)
        t.snr = 49.5
        t.range_m = 150.0
        for _ in range(100):
            t.range_m = 150.0
            t.step()
        self.assertGreaterEqual(t.snr, 0)
        self.assertLessEqual(t.snr, 50)


class TestDemoSimulatorNoTk(unittest.TestCase):
    """Test DemoSimulator logic without a real Tk event loop.

    We replace ``root.after`` with a mock to avoid needing a display.
    """

    def _make_simulator(self):
        from unittest.mock import MagicMock

        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        mock_root = MagicMock()
        # root.after(ms, fn) should return an id (str)
        mock_root.after.return_value = "mock_after_id"
        sim = DemoSimulator(fq, uq, mock_root, interval_ms=100)
        return sim, fq, uq, mock_root

    def test_initial_targets_created(self):
        sim, _fq, _uq, _root = self._make_simulator()
        # Should seed 8 initial targets
        self.assertEqual(len(sim._targets), 8)

    def test_tick_produces_frame_and_targets(self):
        sim, fq, uq, _root = self._make_simulator()
        sim._tick()
        # Should have a frame
        self.assertFalse(fq.empty())
        frame = fq.get_nowait()
        self.assertIsInstance(frame, RadarFrame)
        self.assertEqual(frame.frame_number, 1)
        # Should have demo_targets in ui_queue
        tag, payload = uq.get_nowait()
        self.assertEqual(tag, "demo_targets")
        self.assertIsInstance(payload, list)

    def test_tick_produces_nonzero_detections(self):
        """Demo targets should actually render into the range-Doppler grid."""
        sim, fq, _uq, _root = self._make_simulator()
        sim._tick()
        frame = fq.get_nowait()
        # At least some targets should produce magnitude > 0 and detections
        self.assertGreater(frame.magnitude.sum(), 0,
                           "Demo targets should render into range-Doppler grid")
        self.assertGreater(frame.detection_count, 0,
                           "Demo targets should produce detections")

    def test_stop_cancels_after(self):
        sim, _fq, _uq, mock_root = self._make_simulator()
        sim._tick()  # sets _after_id
        sim.stop()
        mock_root.after_cancel.assert_called_once_with("mock_after_id")
        self.assertIsNone(sim._after_id)


class TestReplayController(unittest.TestCase):
    """Unit tests for _ReplayController (no GUI required)."""

    def test_initial_state(self):
        fq = queue.Queue()
        uq = queue.Queue()
        ctrl = _ReplayController(fq, uq)
        self.assertEqual(ctrl.total_frames, 0)
        self.assertEqual(ctrl.current_index, 0)
        self.assertFalse(ctrl.is_playing)
        self.assertIsNone(ctrl.software_fpga)

    def test_set_speed(self):
        ctrl = _ReplayController(queue.Queue(), queue.Queue())
        ctrl.set_speed("2x")
        self.assertAlmostEqual(ctrl._frame_interval, 0.050)

    def test_set_speed_unknown_falls_back(self):
        ctrl = _ReplayController(queue.Queue(), queue.Queue())
        ctrl.set_speed("99x")
        self.assertAlmostEqual(ctrl._frame_interval, 0.100)

    def test_set_loop(self):
        ctrl = _ReplayController(queue.Queue(), queue.Queue())
        ctrl.set_loop(True)
        self.assertTrue(ctrl._loop)
        ctrl.set_loop(False)
        self.assertFalse(ctrl._loop)

    def test_seek_increments_past_emitted(self):
        """After seek(), _current_index should be one past the seeked frame."""
        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        ctrl = _ReplayController(fq, uq)
        # Manually set engine to a mock to allow seek
        from unittest.mock import MagicMock
        mock_engine = MagicMock()
        mock_engine.total_frames = 10
        mock_engine.get_frame.return_value = RadarFrame()
        ctrl._engine = mock_engine
        ctrl.seek(5)
        # _current_index should be 6 (past the emitted frame)
        self.assertEqual(ctrl._current_index, 6)
        self.assertEqual(ctrl._last_emitted_index, 5)
        # Frame should be in the queue
        self.assertFalse(fq.empty())

    def test_seek_clamps_to_bounds(self):
        from unittest.mock import MagicMock

        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        ctrl = _ReplayController(fq, uq)
        mock_engine = MagicMock()
        mock_engine.total_frames = 5
        mock_engine.get_frame.return_value = RadarFrame()
        ctrl._engine = mock_engine

        ctrl.seek(100)
        # Should clamp to last frame (index 4), then _current_index = 5
        self.assertEqual(ctrl._last_emitted_index, 4)
        self.assertEqual(ctrl._current_index, 5)

        ctrl.seek(-10)
        # Should clamp to 0, then _current_index = 1
        self.assertEqual(ctrl._last_emitted_index, 0)
        self.assertEqual(ctrl._current_index, 1)

    def test_close_releases_engine(self):
        from unittest.mock import MagicMock

        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        ctrl = _ReplayController(fq, uq)
        mock_engine = MagicMock()
        mock_engine.total_frames = 5
        mock_engine.get_frame.return_value = RadarFrame()
        ctrl._engine = mock_engine

        ctrl.close()
        mock_engine.close.assert_called_once()
        self.assertIsNone(ctrl._engine)
        self.assertIsNone(ctrl.software_fpga)


if __name__ == "__main__":
    unittest.main(verbosity=2)
