#!/usr/bin/env python3
"""
Tests for radar_protocol.py — v7c packet format.
=================================================
Covers:
  - build_command()
  - parse_data_packet() with v7c 24-byte packets
  - parse_status_packet() with v7c 40-byte packets
  - find_packet_boundaries() with mixed data/status/garbage
  - RadarAcquisition range-only mode frame assembly
  - FT601Connection._mock_read() generating valid v7c packets

All tests run locally without hardware (mock mode or synthetic buffers).
"""

import struct
import queue
import socket
import time
import threading
import unittest
import numpy as np

from radar_protocol import (
    RadarProtocol,
    RadarFrame,
    StatusResponse,
    FT601Connection,
    SocketConnection,
    RadarAcquisition,
    HEADER_BYTE,
    HEADER_DOPPLER,
    HEADER_CFAR,
    FOOTER_BYTE,
    STATUS_HEADER_BYTE,
    NUM_RANGE_BINS,
    NUM_DOPPLER_BINS,
    NUM_CELLS,
    Opcode,
    _to_signed16,
)


# ============================================================================
# Helpers — build synthetic v7c packets
# ============================================================================

def make_data_packet(range_i: int, range_q: int) -> bytes:
    """Build a synthetic v7c data packet (24 bytes, 6 x LE words)."""
    rword = (((range_q & 0xFFFF) << 16) | (range_i & 0xFFFF)) & 0xFFFFFFFF
    buf = bytearray(24)
    struct.pack_into("<I", buf, 0, 0x000000AA)       # header
    struct.pack_into("<I", buf, 4, rword)             # word 1: range data
    struct.pack_into("<I", buf, 8, (rword << 8) & 0xFFFFFFFF)   # word 2
    struct.pack_into("<I", buf, 12, (rword << 16) & 0xFFFFFFFF) # word 3
    struct.pack_into("<I", buf, 16, (rword << 24) & 0xFFFFFFFF) # word 4
    struct.pack_into("<I", buf, 20, 0x00000055)       # footer
    return bytes(buf)


def make_status_packet(
    threshold=500, stream_ctrl=0, radar_mode=0,
    long_chirp=3000, long_listen=13700,
    guard=17540, short_chirp=50,
    short_listen=17450, chirps_per_elev=32,
    range_mode=1,
    self_test_flags=0, self_test_detail=0, self_test_busy=0,
    dbg_wr_strobes=100, dbg_txe_blocks=20,
    dbg_pkt_starts=50, dbg_pkt_completions=50,
) -> bytes:
    """Build a synthetic v7c status packet (40 bytes, 10 x LE words)."""
    buf = bytearray(40)
    # Word 0: BB header
    struct.pack_into("<I", buf, 0, 0x000000BB)
    # Word 1: {0xFF, 3'b0, mode[1:0], 5'b0, stream[2:0], threshold[15:0]}
    w1 = (threshold & 0xFFFF) | ((stream_ctrl & 0x07) << 16) | ((radar_mode & 0x03) << 21)
    struct.pack_into("<I", buf, 4, w1)
    # Word 2: {long_chirp[31:16], long_listen[15:0]}
    w2 = (long_listen & 0xFFFF) | ((long_chirp & 0xFFFF) << 16)
    struct.pack_into("<I", buf, 8, w2)
    # Word 3: {guard[31:16], short_chirp[15:0]}
    w3 = (short_chirp & 0xFFFF) | ((guard & 0xFFFF) << 16)
    struct.pack_into("<I", buf, 12, w3)
    # Word 4: {short_listen[31:16], 10'd0, chirps_per_elev[5:0]}
    w4 = (chirps_per_elev & 0x3F) | ((short_listen & 0xFFFF) << 16)
    struct.pack_into("<I", buf, 16, w4)
    # Word 5: {30'd0, range_mode[1:0]}
    w5 = range_mode & 0x03
    struct.pack_into("<I", buf, 20, w5)
    # Word 6: {7'd0, self_test_busy, 8'd0, detail[7:0], 3'd0, flags[4:0]}
    w6 = (self_test_flags & 0x1F) | ((self_test_detail & 0xFF) << 8) | ((self_test_busy & 0x01) << 24)
    struct.pack_into("<I", buf, 24, w6)
    # Word 7: {dbg_wr_strobes[15:0], dbg_txe_blocks[15:0]}
    w7 = (dbg_txe_blocks & 0xFFFF) | ((dbg_wr_strobes & 0xFFFF) << 16)
    struct.pack_into("<I", buf, 28, w7)
    # Word 8: {dbg_pkt_starts[15:0], dbg_pkt_completions[15:0]}
    w8 = (dbg_pkt_completions & 0xFFFF) | ((dbg_pkt_starts & 0xFFFF) << 16)
    struct.pack_into("<I", buf, 32, w8)
    # Word 9: 55 footer
    struct.pack_into("<I", buf, 36, 0x00000055)
    return bytes(buf)


def make_doppler_packet(range_bin: int, doppler_bin: int,
                        doppler_i: int, doppler_q: int,
                        sub_frame: int = 0) -> bytes:
    """Build a synthetic v9 Doppler packet (16 bytes, 4 x LE words).

    Word 0: {24'h000000, 0xCC}
    Word 1: {range_bin[5:0], doppler_bin[4:0], sub_frame, 4'b0000, I[15:0]}
    Word 2: {Q[15:0], I[15:0]}
    Word 3: {24'h000000, 0x55}
    """
    buf = bytearray(16)
    struct.pack_into("<I", buf, 0, 0x000000CC)

    i_unsigned = doppler_i & 0xFFFF
    q_unsigned = doppler_q & 0xFFFF
    w1 = (i_unsigned
          | ((sub_frame & 0x01) << 20)
          | ((doppler_bin & 0x1F) << 21)
          | ((range_bin & 0x3F) << 26))
    struct.pack_into("<I", buf, 4, w1)

    w2 = ((q_unsigned << 16) | i_unsigned) & 0xFFFFFFFF
    struct.pack_into("<I", buf, 8, w2)

    struct.pack_into("<I", buf, 12, 0x00000055)
    return bytes(buf)


def make_cfar_packet(range_bin: int, doppler_bin: int,
                     magnitude: int, threshold: int,
                     detect_flag: int = 1) -> bytes:
    """Build a synthetic v9 CFAR detection packet (16 bytes, 4 x LE words).

    Word 0: {24'h000000, 0xDD}
    Word 1: {flag[31], range[30:25], doppler[24:20], 3'b000[19:17], magnitude[16:0]}
    Word 2: {15'b0[31:17], threshold[16:0]}
    Word 3: {24'h000000, 0x55}
    """
    buf = bytearray(16)
    struct.pack_into("<I", buf, 0, 0x000000DD)

    w1 = ((magnitude & 0x1FFFF)
          | ((doppler_bin & 0x1F) << 20)
          | ((range_bin & 0x3F) << 25)
          | ((detect_flag & 0x01) << 31))
    struct.pack_into("<I", buf, 4, w1)

    w2 = threshold & 0x1FFFF
    struct.pack_into("<I", buf, 8, w2)

    struct.pack_into("<I", buf, 12, 0x00000055)
    return bytes(buf)


# ============================================================================
# build_command()
# ============================================================================

class TestBuildCommand(unittest.TestCase):
    """Tests for RadarProtocol.build_command()."""

    def test_basic_command(self):
        cmd = RadarProtocol.build_command(0x04, 0x0001)
        self.assertEqual(len(cmd), 4)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x04)
        self.assertEqual(word & 0xFFFF, 0x0001)

    def test_status_request(self):
        cmd = RadarProtocol.build_command(0xFF, 0x0000)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0xFF)
        self.assertEqual(word & 0xFFFF, 0x0000)

    def test_threshold_command(self):
        cmd = RadarProtocol.build_command(Opcode.THRESHOLD, 0x1234)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x03)
        self.assertEqual(word & 0xFFFF, 0x1234)

    def test_addr_field(self):
        cmd = RadarProtocol.build_command(0x01, 0x0002, addr=0xAB)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x01)
        self.assertEqual((word >> 16) & 0xFF, 0xAB)
        self.assertEqual(word & 0xFFFF, 0x0002)

    def test_max_value(self):
        cmd = RadarProtocol.build_command(0xFF, 0xFFFF, addr=0xFF)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual(word, 0xFFFFFFFF)

    def test_stream_control_range_only(self):
        """stream_control=0x01 enables range-only streaming."""
        cmd = RadarProtocol.build_command(Opcode.STREAM_CONTROL, 0x01)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x04)
        self.assertEqual(word & 0xFFFF, 0x01)


# ============================================================================
# _to_signed16()
# ============================================================================

class TestToSigned16(unittest.TestCase):

    def test_positive(self):
        self.assertEqual(_to_signed16(100), 100)

    def test_zero(self):
        self.assertEqual(_to_signed16(0), 0)

    def test_max_positive(self):
        self.assertEqual(_to_signed16(0x7FFF), 32767)

    def test_negative_one(self):
        self.assertEqual(_to_signed16(0xFFFF), -1)

    def test_min_negative(self):
        self.assertEqual(_to_signed16(0x8000), -32768)

    def test_wraps_32bit(self):
        # Upper bits should be masked off
        self.assertEqual(_to_signed16(0x10064), 100)


# ============================================================================
# parse_data_packet()
# ============================================================================

class TestParseDataPacket(unittest.TestCase):
    """Tests for v7c 24-byte data packet parsing."""

    def test_basic_positive_values(self):
        pkt = make_data_packet(range_i=1000, range_q=500)
        result = RadarProtocol.parse_data_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], 1000)
        self.assertEqual(result["range_q"], 500)

    def test_negative_values(self):
        pkt = make_data_packet(range_i=-100, range_q=-200)
        result = RadarProtocol.parse_data_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], -100)
        self.assertEqual(result["range_q"], -200)

    def test_zero_values(self):
        pkt = make_data_packet(range_i=0, range_q=0)
        result = RadarProtocol.parse_data_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], 0)
        self.assertEqual(result["range_q"], 0)

    def test_max_positive(self):
        pkt = make_data_packet(range_i=32767, range_q=32767)
        result = RadarProtocol.parse_data_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], 32767)
        self.assertEqual(result["range_q"], 32767)

    def test_min_negative(self):
        pkt = make_data_packet(range_i=-32768, range_q=-32768)
        result = RadarProtocol.parse_data_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], -32768)
        self.assertEqual(result["range_q"], -32768)

    def test_range_value_word(self):
        """Verify range_value is the full 32-bit word."""
        pkt = make_data_packet(range_i=0x1234, range_q=0x5678)
        result = RadarProtocol.parse_data_packet(pkt)
        expected = ((0x5678 << 16) | 0x1234) & 0xFFFFFFFF
        self.assertEqual(result["range_value"], expected)

    def test_dev_wrapper_synthetic(self):
        """Match dev wrapper: range_profile_reg = {hb_counter[31:16], counter[15:0] ^ 0xA5A5}."""
        hb_counter = 0x00010000  # counter = 65536
        rword = ((hb_counter >> 16) << 16) | ((hb_counter & 0xFFFF) ^ 0xA5A5)
        # Build packet with this exact word
        buf = bytearray(24)
        struct.pack_into("<I", buf, 0, 0x000000AA)
        struct.pack_into("<I", buf, 4, rword)
        struct.pack_into("<I", buf, 8, (rword << 8) & 0xFFFFFFFF)
        struct.pack_into("<I", buf, 12, (rword << 16) & 0xFFFFFFFF)
        struct.pack_into("<I", buf, 16, (rword << 24) & 0xFFFFFFFF)
        struct.pack_into("<I", buf, 20, 0x00000055)
        result = RadarProtocol.parse_data_packet(bytes(buf))
        self.assertIsNotNone(result)
        self.assertEqual(result["range_value"], rword)

    def test_too_short(self):
        result = RadarProtocol.parse_data_packet(b"\xAA\x00\x00\x00" + b"\x00" * 10)
        self.assertIsNone(result)

    def test_bad_header(self):
        pkt = bytearray(make_data_packet(100, 200))
        pkt[0] = 0xCC  # corrupt header
        result = RadarProtocol.parse_data_packet(bytes(pkt))
        self.assertIsNone(result)

    def test_bad_footer(self):
        pkt = bytearray(make_data_packet(100, 200))
        pkt[20] = 0xCC  # corrupt footer byte
        result = RadarProtocol.parse_data_packet(bytes(pkt))
        self.assertIsNone(result)

    def test_extra_bytes_ignored(self):
        """Extra trailing bytes should not affect parsing."""
        pkt = make_data_packet(42, 84) + b"\xFF" * 100
        result = RadarProtocol.parse_data_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], 42)
        self.assertEqual(result["range_q"], 84)


# ============================================================================
# parse_status_packet()
# ============================================================================

class TestParseStatusPacket(unittest.TestCase):
    """Tests for v7c 40-byte status packet parsing."""

    def test_default_values(self):
        """Match the known dev-board defaults."""
        pkt = make_status_packet(
            threshold=0x1234, long_chirp=3000, long_listen=13700,
            guard=17540, short_chirp=50, short_listen=17450,
            chirps_per_elev=32, range_mode=1,
        )
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.cfar_threshold, 0x1234)
        self.assertEqual(sr.long_chirp, 3000)
        self.assertEqual(sr.long_listen, 13700)
        self.assertEqual(sr.guard, 17540)
        self.assertEqual(sr.short_chirp, 50)
        self.assertEqual(sr.short_listen, 17450)
        self.assertEqual(sr.chirps_per_elev, 32)
        self.assertEqual(sr.range_mode, 1)

    def test_radar_mode_and_stream_ctrl(self):
        pkt = make_status_packet(radar_mode=2, stream_ctrl=5, threshold=100)
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.radar_mode, 2)
        self.assertEqual(sr.stream_ctrl, 5)
        self.assertEqual(sr.cfar_threshold, 100)

    def test_self_test_fields(self):
        pkt = make_status_packet(
            self_test_flags=0x1F,  # all 5 pass
            self_test_detail=0xAD,
            self_test_busy=1,
        )
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.self_test_flags, 0x1F)
        self.assertEqual(sr.self_test_detail, 0xAD)
        self.assertEqual(sr.self_test_busy, 1)

    def test_debug_counters(self):
        pkt = make_status_packet(
            dbg_wr_strobes=3053, dbg_txe_blocks=600,
            dbg_pkt_starts=3053, dbg_pkt_completions=3053,
        )
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.dbg_wr_strobes, 3053)
        self.assertEqual(sr.dbg_txe_blocks, 600)
        self.assertEqual(sr.dbg_pkt_starts, 3053)
        self.assertEqual(sr.dbg_pkt_completions, 3053)

    def test_zero_values(self):
        pkt = make_status_packet(
            threshold=0, stream_ctrl=0, radar_mode=0,
            long_chirp=0, long_listen=0, guard=0, short_chirp=0,
            short_listen=0, chirps_per_elev=0, range_mode=0,
            self_test_flags=0, self_test_detail=0, self_test_busy=0,
            dbg_wr_strobes=0, dbg_txe_blocks=0,
            dbg_pkt_starts=0, dbg_pkt_completions=0,
        )
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.cfar_threshold, 0)
        self.assertEqual(sr.dbg_pkt_starts, 0)

    def test_too_short(self):
        pkt = b"\xBB\x00\x00\x00" + b"\x00" * 20  # only 24 bytes, need 40
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertIsNone(sr)

    def test_no_bb_header(self):
        pkt = b"\x00" * 40
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertIsNone(sr)

    def test_empty_input(self):
        sr = RadarProtocol.parse_status_packet(b"")
        self.assertIsNone(sr)

    def test_bb_offset_in_larger_buffer(self):
        """BB header not at start — parser should scan for it."""
        garbage = b"\x00\x01\x02\x03" * 4  # 16 bytes of garbage
        pkt = make_status_packet(threshold=999, range_mode=2)
        data = garbage + pkt
        sr = RadarProtocol.parse_status_packet(data)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.cfar_threshold, 999)
        self.assertEqual(sr.range_mode, 2)

    def test_chirps_per_elev_6bit_mask(self):
        """chirps_per_elev is masked to 6 bits (max 63)."""
        pkt = make_status_packet(chirps_per_elev=63)
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertEqual(sr.chirps_per_elev, 63)

    def test_range_mode_2bit_mask(self):
        """range_mode is masked to 2 bits."""
        pkt = make_status_packet(range_mode=3)  # 0b11 — max for 2-bit
        sr = RadarProtocol.parse_status_packet(pkt)
        self.assertEqual(sr.range_mode, 3)


# ============================================================================
# find_packet_boundaries()
# ============================================================================

class TestFindPacketBoundaries(unittest.TestCase):
    """Tests for scanning buffers for v7c data and status packets."""

    def test_single_data_packet(self):
        pkt = make_data_packet(100, 200)
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(len(result), 1)
        start, end, ptype = result[0]
        self.assertEqual(start, 0)
        self.assertEqual(end, 24)
        self.assertEqual(ptype, "data")

    def test_single_status_packet(self):
        pkt = make_status_packet()
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(len(result), 1)
        start, end, ptype = result[0]
        self.assertEqual(start, 0)
        self.assertEqual(end, 40)
        self.assertEqual(ptype, "status")

    def test_consecutive_data_packets(self):
        pkt = make_data_packet(10, 20) + make_data_packet(30, 40) + make_data_packet(50, 60)
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(len(result), 3)
        for i, (start, end, ptype) in enumerate(result):
            self.assertEqual(start, i * 24)
            self.assertEqual(end, (i + 1) * 24)
            self.assertEqual(ptype, "data")

    def test_data_then_status(self):
        buf = make_data_packet(100, 200) + make_status_packet(threshold=42)
        result = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0][2], "data")
        self.assertEqual(result[1][2], "status")
        self.assertEqual(result[0][0], 0)
        self.assertEqual(result[0][1], 24)
        self.assertEqual(result[1][0], 24)
        self.assertEqual(result[1][1], 64)

    def test_garbage_before_packet(self):
        garbage = bytes([0x12, 0x34, 0x56, 0x78])  # 4 bytes
        pkt = make_data_packet(10, 20)
        buf = garbage + pkt
        result = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], 4)  # starts after garbage
        self.assertEqual(result[0][1], 28)

    def test_garbage_between_packets(self):
        pkt1 = make_data_packet(10, 20)
        garbage = bytes([0x00, 0x11, 0x22, 0x33])
        pkt2 = make_data_packet(30, 40)
        buf = pkt1 + garbage + pkt2
        result = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0][0], 0)
        self.assertEqual(result[1][0], 28)

    def test_empty_buffer(self):
        result = RadarProtocol.find_packet_boundaries(b"")
        self.assertEqual(result, [])

    def test_short_buffer(self):
        result = RadarProtocol.find_packet_boundaries(b"\xAA\x00\x00")
        self.assertEqual(result, [])

    def test_incomplete_data_packet(self):
        """Buffer has header but not enough bytes for full packet."""
        pkt = make_data_packet(10, 20)[:20]  # truncated
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(result, [])

    def test_incomplete_status_packet(self):
        pkt = make_status_packet()[:36]  # truncated
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(result, [])

    def test_bad_footer_skipped(self):
        """Packet with header but corrupted footer should be skipped."""
        pkt = bytearray(make_data_packet(10, 20))
        pkt[20] = 0xCC  # corrupt footer
        result = RadarProtocol.find_packet_boundaries(bytes(pkt))
        self.assertEqual(result, [])

    def test_many_packets_performance(self):
        """Ensure scanning hundreds of packets is fast."""
        buf = b""
        n = 500
        for i in range(n):
            buf += make_data_packet(i, i * 2)
        result = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(result), n)

    def test_aa_byte_in_data_not_confused(self):
        """0xAA appearing in data payload should not create false boundaries."""
        # Build a packet where range_i = 0x00AA
        pkt = make_data_packet(range_i=0x00AA, range_q=0)
        result = RadarProtocol.find_packet_boundaries(pkt)
        # Should still find exactly 1 packet (the real one at offset 0)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], 0)


# ============================================================================
# FT601Connection._mock_read() — v7c format validation
# ============================================================================

class TestMockRead(unittest.TestCase):
    """Tests for mock mode generating valid v7c packets."""

    def setUp(self):
        self.conn = FT601Connection(mock=True)
        self.conn.open()

    def tearDown(self):
        self.conn.close()

    def test_returns_bytes(self):
        data = self.conn.read(1024)
        self.assertIsInstance(data, bytes)
        self.assertGreater(len(data), 0)

    def test_packet_size_multiple_of_24(self):
        data = self.conn.read(24 * 64)
        # Total length should be a multiple of 24
        self.assertEqual(len(data) % 24, 0)

    def test_packets_have_valid_framing(self):
        data = self.conn.read(24 * 10)
        packets = RadarProtocol.find_packet_boundaries(data)
        self.assertGreater(len(packets), 0)
        for start, end, ptype in packets:
            self.assertEqual(ptype, "data")
            self.assertEqual(end - start, 24)

    def test_all_packets_parseable(self):
        data = self.conn.read(24 * NUM_RANGE_BINS)
        packets = RadarProtocol.find_packet_boundaries(data)
        for start, end, ptype in packets:
            parsed = RadarProtocol.parse_data_packet(data[start:end])
            self.assertIsNotNone(parsed, f"Failed to parse packet at offset {start}")
            self.assertIn("range_i", parsed)
            self.assertIn("range_q", parsed)
            self.assertIn("range_value", parsed)

    def test_frame_wraps_at_num_range_bins(self):
        """After NUM_RANGE_BINS packets, mock should wrap sample index."""
        # _mock_read returns min(NUM_RANGE_BINS, size//24) packets per call
        # So read one full frame, then read another
        data1 = self.conn.read(24 * NUM_RANGE_BINS)
        self.assertEqual(len(data1), 24 * NUM_RANGE_BINS)
        data2 = self.conn.read(24 * NUM_RANGE_BINS)
        self.assertEqual(len(data2), 24 * NUM_RANGE_BINS)
        # Both reads should produce valid packets
        p1 = RadarProtocol.find_packet_boundaries(data1)
        p2 = RadarProtocol.find_packet_boundaries(data2)
        self.assertEqual(len(p1), NUM_RANGE_BINS)
        self.assertEqual(len(p2), NUM_RANGE_BINS)

    def test_static_scene_has_targets(self):
        """Static scene should have higher amplitudes near bins 20 and 40."""
        conn = FT601Connection(mock=True, moving_target=False)
        conn.open()
        data = conn.read(24 * NUM_RANGE_BINS)
        packets = RadarProtocol.find_packet_boundaries(data)
        magnitudes = []
        for start, end, _ in packets:
            parsed = RadarProtocol.parse_data_packet(data[start:end])
            if parsed:
                magnitudes.append(abs(parsed["range_i"]) + abs(parsed["range_q"]))
        conn.close()

        # Bins near 20 and 40 should be significantly above noise
        self.assertGreater(len(magnitudes), 40)
        noise_floor = np.median(magnitudes)
        # Check bin 20 region (indices 19-21)
        peak_20 = max(magnitudes[19:22])
        self.assertGreater(peak_20, noise_floor * 10,
                           f"Expected target near bin 20; peak={peak_20}, noise={noise_floor}")

    def test_moving_target_mode(self):
        """Moving target mode should also produce valid packets."""
        conn = FT601Connection(mock=True, moving_target=True)
        conn.open()
        data = conn.read(24 * NUM_RANGE_BINS)
        packets = RadarProtocol.find_packet_boundaries(data)
        self.assertEqual(len(packets), NUM_RANGE_BINS)
        conn.close()

    def test_mock_write_returns_true(self):
        cmd = RadarProtocol.build_command(0x04, 0x01)
        self.assertTrue(self.conn.write(cmd))

    def test_not_open_returns_none(self):
        conn = FT601Connection(mock=True)
        # Don't open
        self.assertIsNone(conn.read())
        self.assertFalse(conn.write(b"\x00\x00\x00\x00"))


# ============================================================================
# RadarAcquisition — range-only mode frame assembly
# ============================================================================

class TestRadarAcquisitionRangeOnly(unittest.TestCase):
    """Tests for RadarAcquisition in range_only=True mode."""

    def _run_acquisition(self, num_frames=2, timeout_s=5.0):
        """Helper: run acquisition in mock mode and collect frames."""
        conn = FT601Connection(mock=True, moving_target=False)
        conn.open()
        fq = queue.Queue(maxsize=10)
        acq = RadarAcquisition(conn, fq, range_only=True)
        acq.start()

        frames = []
        deadline = time.time() + timeout_s
        while len(frames) < num_frames and time.time() < deadline:
            try:
                frame = fq.get(timeout=0.5)
                frames.append(frame)
            except queue.Empty:
                pass

        acq.stop()
        acq.join(timeout=2.0)
        conn.close()
        return frames

    def test_produces_frames(self):
        frames = self._run_acquisition(num_frames=2)
        self.assertGreaterEqual(len(frames), 2, "Should produce at least 2 frames")

    def test_frame_has_correct_shape(self):
        frames = self._run_acquisition(num_frames=1)
        self.assertGreaterEqual(len(frames), 1)
        f = frames[0]
        self.assertIsInstance(f, RadarFrame)
        self.assertEqual(f.range_profile.shape, (NUM_RANGE_BINS,))
        self.assertEqual(f.magnitude.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))

    def test_range_profile_nonzero(self):
        """Range profile should have nonzero values (targets + noise)."""
        frames = self._run_acquisition(num_frames=1)
        self.assertGreaterEqual(len(frames), 1)
        rp = frames[0].range_profile
        self.assertGreater(np.max(rp), 0, "Range profile should have nonzero values")

    def test_frame_number_increments(self):
        frames = self._run_acquisition(num_frames=3)
        self.assertGreaterEqual(len(frames), 3)
        for i in range(1, len(frames)):
            self.assertGreater(frames[i].frame_number, frames[i - 1].frame_number)

    def test_timestamp_set(self):
        frames = self._run_acquisition(num_frames=1)
        self.assertGreaterEqual(len(frames), 1)
        self.assertGreater(frames[0].timestamp, 0)

    def test_magnitude_in_doppler_bin_0(self):
        """In range-only mode, magnitude should be in Doppler bin 0 only."""
        frames = self._run_acquisition(num_frames=1)
        self.assertGreaterEqual(len(frames), 1)
        mag = frames[0].magnitude
        # Doppler bins 1+ should be zero
        self.assertEqual(np.sum(mag[:, 1:]), 0.0,
                         "Doppler bins 1+ should be zero in range-only mode")
        # Doppler bin 0 should have data
        self.assertGreater(np.sum(mag[:, 0]), 0.0,
                           "Doppler bin 0 should have data in range-only mode")


# ============================================================================
# RadarAcquisition — residual buffer handling
# ============================================================================

class TestResidualBuffer(unittest.TestCase):
    """Tests for cross-read packet reassembly via _residual buffer."""

    def test_split_packet_across_reads(self):
        """Simulate a packet split across two USB reads."""
        conn = FT601Connection(mock=True)
        conn.open()
        fq = queue.Queue(maxsize=10)
        acq = RadarAcquisition(conn, fq, range_only=True)

        # Build two complete packets
        pkt1 = make_data_packet(100, 200)
        pkt2 = make_data_packet(300, 400)

        # Split: first read gets pkt1 + first 12 bytes of pkt2
        # second read gets remaining 12 bytes of pkt2
        combined = pkt1 + pkt2

        # Manually test the residual logic
        acq._residual = b""

        # First chunk: pkt1 complete + pkt2 partial
        chunk1 = combined[:36]  # 24 + 12
        raw = acq._residual + chunk1
        acq._residual = b""
        packets = RadarProtocol.find_packet_boundaries(raw)
        # Should find pkt1
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0][0], 0)
        self.assertEqual(packets[0][1], 24)
        # Residual should be the leftover 12 bytes
        last_end = packets[-1][1]
        acq._residual = raw[last_end:]
        self.assertEqual(len(acq._residual), 12)

        # Second chunk: remaining 12 bytes of pkt2
        chunk2 = combined[36:]
        raw2 = acq._residual + chunk2
        acq._residual = b""
        packets2 = RadarProtocol.find_packet_boundaries(raw2)
        # Should find pkt2 now
        self.assertEqual(len(packets2), 1)
        parsed = RadarProtocol.parse_data_packet(raw2[packets2[0][0]:packets2[0][1]])
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["range_i"], 300)
        self.assertEqual(parsed["range_q"], 400)

        conn.close()


# ============================================================================
# StatusResponse dataclass
# ============================================================================

class TestStatusResponse(unittest.TestCase):

    def test_defaults(self):
        sr = StatusResponse()
        self.assertEqual(sr.radar_mode, 0)
        self.assertEqual(sr.stream_ctrl, 0)
        self.assertEqual(sr.dbg_pkt_starts, 0)

    def test_all_fields_set(self):
        sr = StatusResponse(
            radar_mode=1, stream_ctrl=7, cfar_threshold=500,
            long_chirp=3000, long_listen=13700,
            guard=17540, short_chirp=50,
            short_listen=17450, chirps_per_elev=32,
            range_mode=1,
            self_test_flags=0x1F, self_test_detail=0, self_test_busy=0,
            dbg_wr_strobes=100, dbg_txe_blocks=20,
            dbg_pkt_starts=50, dbg_pkt_completions=50,
        )
        self.assertEqual(sr.radar_mode, 1)
        self.assertEqual(sr.long_chirp, 3000)
        self.assertEqual(sr.dbg_pkt_completions, 50)


# ============================================================================
# Opcode enum
# ============================================================================

class TestOpcode(unittest.TestCase):

    def test_stream_control_value(self):
        self.assertEqual(Opcode.STREAM_CONTROL, 0x04)

    def test_status_request_value(self):
        self.assertEqual(Opcode.STATUS_REQUEST, 0xFF)

    def test_self_test_trigger_value(self):
        self.assertEqual(Opcode.SELF_TEST_TRIGGER, 0x30)

    def test_all_opcodes_unique(self):
        values = [op.value for op in Opcode]
        self.assertEqual(len(values), len(set(values)),
                         "Opcode values must be unique")


# ============================================================================
# Packet size constants
# ============================================================================

class TestPacketConstants(unittest.TestCase):

    def test_data_packet_size(self):
        self.assertEqual(RadarProtocol.V7C_DATA_PACKET_SIZE, 24)

    def test_status_packet_size(self):
        self.assertEqual(RadarProtocol.V7C_STATUS_PACKET_SIZE, 40)

    def test_data_packet_matches(self):
        """Generated packet should match the declared size."""
        pkt = make_data_packet(0, 0)
        self.assertEqual(len(pkt), RadarProtocol.V7C_DATA_PACKET_SIZE)

    def test_status_packet_matches(self):
        pkt = make_status_packet()
        self.assertEqual(len(pkt), RadarProtocol.V7C_STATUS_PACKET_SIZE)


# ============================================================================
# SocketConnection tests (uses a local mock TCP server)
# ============================================================================

def _find_free_port():
    """Find a free TCP port on localhost."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class _MockStreamServer:
    """
    Minimal TCP server that mimics ft601_stream_server.py protocol.
    Sends length-prefixed data messages and receives length-prefixed commands.
    """

    def __init__(self, port):
        self.port = port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(("127.0.0.1", port))
        self.server_sock.listen(1)
        self.server_sock.settimeout(5.0)
        self.client_sock = None
        self.received_commands = []
        self._stop = threading.Event()
        self._thread = None

    def accept(self):
        self.client_sock, _ = self.server_sock.accept()
        self.client_sock.settimeout(2.0)

    def send_data(self, data: bytes):
        """Send a length-prefixed data message to the client."""
        hdr = struct.pack(">I", len(data))
        self.client_sock.sendall(hdr + data)

    def recv_command(self) -> bytes:
        """Receive one length-prefixed command from the client."""
        hdr = self.client_sock.recv(4)
        if len(hdr) < 4:
            return b""
        length = struct.unpack(">I", hdr)[0]
        if length == 0:
            return b""
        return self.client_sock.recv(length)

    def close(self):
        self._stop.set()
        if self.client_sock:
            try:
                self.client_sock.close()
            except Exception:
                pass
        self.server_sock.close()


class TestSocketConnection(unittest.TestCase):

    def setUp(self):
        self.port = _find_free_port()
        self.server = _MockStreamServer(self.port)
        # Accept in a thread so client can connect
        self._accept_thread = threading.Thread(target=self.server.accept,
                                                daemon=True)
        self._accept_thread.start()

    def tearDown(self):
        self.server.close()

    def _connect(self) -> SocketConnection:
        conn = SocketConnection(host="127.0.0.1", port=self.port)
        ok = conn.open()
        self._accept_thread.join(timeout=2)
        self.assertTrue(ok)
        self.assertTrue(conn.is_open)
        return conn

    def test_open_close(self):
        """SocketConnection can connect and disconnect."""
        conn = self._connect()
        self.assertTrue(conn.is_open)
        conn.close()
        self.assertFalse(conn.is_open)

    def test_open_fail_no_server(self):
        """open() returns False when no server is listening."""
        self.server.close()
        time.sleep(0.1)
        conn = SocketConnection(host="127.0.0.1", port=_find_free_port())
        ok = conn.open()
        self.assertFalse(ok)
        self.assertFalse(conn.is_open)

    def test_read_data_packet(self):
        """read() receives a framed data packet from the server."""
        conn = self._connect()
        # Server sends a v7c data packet
        pkt = make_data_packet(1000, 2000)
        self.server.send_data(pkt)
        time.sleep(0.05)
        data = conn.read()
        self.assertIsNotNone(data)
        self.assertEqual(data, pkt)
        conn.close()

    def test_read_multiple_packets(self):
        """read() returns packets one at a time (length-prefixed)."""
        conn = self._connect()
        pkt1 = make_data_packet(100, 200)
        pkt2 = make_data_packet(300, 400)
        self.server.send_data(pkt1)
        self.server.send_data(pkt2)
        time.sleep(0.05)
        d1 = conn.read()
        d2 = conn.read()
        self.assertEqual(d1, pkt1)
        self.assertEqual(d2, pkt2)
        conn.close()

    def test_write_command(self):
        """write() sends a length-prefixed command to the server."""
        conn = self._connect()
        cmd = RadarProtocol.build_command(Opcode.STREAM_CONTROL, 0x01)
        ok = conn.write(cmd)
        self.assertTrue(ok)
        time.sleep(0.05)
        received = self.server.recv_command()
        self.assertEqual(received, cmd)
        conn.close()

    def test_write_multiple_commands(self):
        """Multiple write() calls are received in order."""
        conn = self._connect()
        cmd1 = RadarProtocol.build_command(Opcode.STREAM_CONTROL, 0x01)
        cmd2 = RadarProtocol.build_command(Opcode.STATUS_REQUEST, 0x01)
        conn.write(cmd1)
        conn.write(cmd2)
        time.sleep(0.05)
        r1 = self.server.recv_command()
        r2 = self.server.recv_command()
        self.assertEqual(r1, cmd1)
        self.assertEqual(r2, cmd2)
        conn.close()

    def test_read_status_packet(self):
        """read() handles status packets (40 bytes)."""
        conn = self._connect()
        pkt = make_status_packet(threshold=0xABCD, long_chirp=5000)
        self.server.send_data(pkt)
        time.sleep(0.05)
        data = conn.read()
        self.assertIsNotNone(data)
        self.assertEqual(len(data), 40)
        status = RadarProtocol.parse_status_packet(data)
        self.assertIsNotNone(status)
        self.assertEqual(status.cfar_threshold, 0xABCD)
        self.assertEqual(status.long_chirp, 5000)
        conn.close()

    def test_read_timeout_returns_none(self):
        """read() returns None when no data is available (timeout)."""
        conn = self._connect()
        # Don't send anything — should timeout
        data = conn.read()
        self.assertIsNone(data)
        conn.close()

    def test_bidirectional_data_and_commands(self):
        """Full duplex: server sends data while client sends commands."""
        conn = self._connect()
        # Client sends a command
        cmd = RadarProtocol.build_command(Opcode.STREAM_CONTROL, 0x01)
        conn.write(cmd)
        # Server sends data
        pkt = make_data_packet(5000, 6000)
        self.server.send_data(pkt)
        time.sleep(0.05)
        # Client reads data
        data = conn.read()
        self.assertEqual(data, pkt)
        # Server reads command
        received = self.server.recv_command()
        self.assertEqual(received, cmd)
        conn.close()

    def test_large_burst(self):
        """Handle a burst of 64 packets (one full frame)."""
        conn = self._connect()
        # Server sends 64 data packets as one big message
        all_pkts = b""
        for i in range(NUM_RANGE_BINS):
            all_pkts += make_data_packet(i * 100, i * 50)
        self.server.send_data(all_pkts)
        time.sleep(0.1)
        data = conn.read()
        self.assertIsNotNone(data)
        self.assertEqual(len(data), NUM_RANGE_BINS * 24)
        # Verify we can parse all packets
        boundaries = RadarProtocol.find_packet_boundaries(data)
        self.assertEqual(len(boundaries), NUM_RANGE_BINS)
        conn.close()


# ============================================================================
# Test: Multi-frame waterfall accumulation (GUI display fix)
# ============================================================================

class TestMultiFrameWaterfall(unittest.TestCase):
    """
    Validates that when multiple RadarFrames are queued, all are accumulated
    into the waterfall — not just the last one. This was the root cause of
    the 'only one frame visible' bug in GUI_radar_dashboard_v2.py.

    Tests the core logic without requiring a tkinter display.
    """

    def test_all_frames_reach_waterfall(self):
        """All 32 frames from a BRAM playback appear in the waterfall."""
        from collections import deque
        WATERFALL_DEPTH = 128  # matches radar_protocol.py

        waterfall = deque(maxlen=WATERFALL_DEPTH)
        for _ in range(WATERFALL_DEPTH):
            waterfall.append(np.zeros(NUM_RANGE_BINS))

        frame_queue = queue.Queue(maxsize=64)

        # Simulate 32 frames arriving (one per chirp in BRAM playback)
        num_playback_frames = 32
        for chirp in range(num_playback_frames):
            frame = RadarFrame()
            # Each chirp has a distinct range profile
            frame.range_profile = np.full(NUM_RANGE_BINS, float(chirp + 1))
            frame.frame_number = chirp
            frame_queue.put_nowait(frame)

        # Simulate the FIXED _update_display logic:
        # Drain all frames, accumulate each into waterfall
        frames = []
        while True:
            try:
                frames.append(frame_queue.get_nowait())
            except queue.Empty:
                break

        for f in frames:
            waterfall.append(f.range_profile.copy())

        # Verify all 32 frames were collected
        self.assertEqual(len(frames), num_playback_frames)

        # Verify the waterfall has the data from all 32 frames
        # (last 32 entries should be the chirp data, rest zeros)
        wf_arr = np.array(list(waterfall))
        for i in range(num_playback_frames):
            row_idx = WATERFALL_DEPTH - num_playback_frames + i
            expected_val = float(i + 1)
            self.assertAlmostEqual(
                wf_arr[row_idx, 0], expected_val,
                msg=f"Waterfall row {row_idx} (chirp {i}) should be {expected_val}")

    def test_old_drain_loses_frames(self):
        """Demonstrate that the OLD logic (keep only last) loses waterfall data."""
        from collections import deque
        WATERFALL_DEPTH = 128

        waterfall = deque(maxlen=WATERFALL_DEPTH)
        for _ in range(WATERFALL_DEPTH):
            waterfall.append(np.zeros(NUM_RANGE_BINS))

        frame_queue = queue.Queue(maxsize=64)

        # Simulate 32 frames
        for chirp in range(32):
            frame = RadarFrame()
            frame.range_profile = np.full(NUM_RANGE_BINS, float(chirp + 1))
            frame_queue.put_nowait(frame)

        # OLD logic: drain queue, keep only last frame
        frame = None
        while True:
            try:
                frame = frame_queue.get_nowait()
            except queue.Empty:
                break

        # Only append the last frame to waterfall
        if frame is not None:
            waterfall.append(frame.range_profile.copy())

        # The waterfall should only have 1 non-zero row (the bug)
        wf_arr = np.array(list(waterfall))
        nonzero_rows = np.count_nonzero(np.sum(wf_arr, axis=1))
        self.assertEqual(nonzero_rows, 1,
                         "Old logic should only append 1 frame (the bug)")

    def test_incremental_updates(self):
        """Frames arriving across multiple update cycles all reach waterfall."""
        from collections import deque
        WATERFALL_DEPTH = 128

        waterfall = deque(maxlen=WATERFALL_DEPTH)
        for _ in range(WATERFALL_DEPTH):
            waterfall.append(np.zeros(NUM_RANGE_BINS))

        frame_queue = queue.Queue(maxsize=64)
        total_frames_seen = 0

        # Simulate 4 update cycles, each receiving 8 frames (32 total)
        for cycle in range(4):
            for i in range(8):
                chirp = cycle * 8 + i
                frame = RadarFrame()
                frame.range_profile = np.full(NUM_RANGE_BINS, float(chirp + 1))
                frame_queue.put_nowait(frame)

            # Fixed drain logic
            frames = []
            while True:
                try:
                    frames.append(frame_queue.get_nowait())
                except queue.Empty:
                    break

            for f in frames:
                waterfall.append(f.range_profile.copy())
            total_frames_seen += len(frames)

        self.assertEqual(total_frames_seen, 32)

        # Last 32 waterfall rows should have data
        wf_arr = np.array(list(waterfall))
        nonzero_rows = np.count_nonzero(np.sum(wf_arr, axis=1))
        self.assertEqual(nonzero_rows, 32)


# ============================================================================
# v9: parse_doppler_packet()
# ============================================================================

class TestParseDopplerPacket(unittest.TestCase):
    """Tests for v9 16-byte Doppler cell packet parsing."""

    def test_basic_values(self):
        pkt = make_doppler_packet(range_bin=10, doppler_bin=5,
                                  doppler_i=1000, doppler_q=500)
        result = RadarProtocol.parse_doppler_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_bin"], 10)
        self.assertEqual(result["doppler_bin"], 5)
        self.assertEqual(result["doppler_i"], 1000)
        self.assertEqual(result["doppler_q"], 500)
        self.assertEqual(result["sub_frame"], 0)

    def test_negative_iq(self):
        pkt = make_doppler_packet(range_bin=0, doppler_bin=0,
                                  doppler_i=-100, doppler_q=-200)
        result = RadarProtocol.parse_doppler_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["doppler_i"], -100)
        self.assertEqual(result["doppler_q"], -200)

    def test_max_bin_values(self):
        pkt = make_doppler_packet(range_bin=63, doppler_bin=31,
                                  doppler_i=32767, doppler_q=-32768,
                                  sub_frame=1)
        result = RadarProtocol.parse_doppler_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_bin"], 63)
        self.assertEqual(result["doppler_bin"], 31)
        self.assertEqual(result["doppler_i"], 32767)
        self.assertEqual(result["doppler_q"], -32768)
        self.assertEqual(result["sub_frame"], 1)

    def test_zero_values(self):
        pkt = make_doppler_packet(range_bin=0, doppler_bin=0,
                                  doppler_i=0, doppler_q=0)
        result = RadarProtocol.parse_doppler_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["doppler_i"], 0)
        self.assertEqual(result["doppler_q"], 0)

    def test_too_short(self):
        result = RadarProtocol.parse_doppler_packet(b"\xCC\x00\x00\x00" + b"\x00" * 8)
        self.assertIsNone(result)

    def test_bad_header(self):
        pkt = bytearray(make_doppler_packet(1, 1, 100, 200))
        pkt[0] = 0xAA  # wrong header
        result = RadarProtocol.parse_doppler_packet(bytes(pkt))
        self.assertIsNone(result)

    def test_bad_footer(self):
        pkt = bytearray(make_doppler_packet(1, 1, 100, 200))
        pkt[12] = 0xCC  # corrupt footer
        result = RadarProtocol.parse_doppler_packet(bytes(pkt))
        self.assertIsNone(result)


# ============================================================================
# v9: parse_cfar_packet()
# ============================================================================

class TestParseCfarPacket(unittest.TestCase):
    """Tests for v9 16-byte CFAR detection packet parsing."""

    def test_basic_detection(self):
        pkt = make_cfar_packet(range_bin=20, doppler_bin=10,
                               magnitude=5000, threshold=3000,
                               detect_flag=1)
        result = RadarProtocol.parse_cfar_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["detect_flag"], 1)
        self.assertEqual(result["range_bin"], 20)
        self.assertEqual(result["doppler_bin"], 10)
        self.assertEqual(result["magnitude"], 5000)
        self.assertEqual(result["threshold"], 3000)

    def test_max_values(self):
        # 17-bit magnitude max = 131071
        pkt = make_cfar_packet(range_bin=63, doppler_bin=31,
                               magnitude=131071, threshold=131071,
                               detect_flag=1)
        result = RadarProtocol.parse_cfar_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_bin"], 63)
        self.assertEqual(result["doppler_bin"], 31)
        self.assertEqual(result["magnitude"], 131071)
        self.assertEqual(result["threshold"], 131071)

    def test_zero_magnitude(self):
        pkt = make_cfar_packet(range_bin=0, doppler_bin=0,
                               magnitude=0, threshold=0,
                               detect_flag=1)
        result = RadarProtocol.parse_cfar_packet(pkt)
        self.assertIsNotNone(result)
        self.assertEqual(result["magnitude"], 0)
        self.assertEqual(result["threshold"], 0)

    def test_too_short(self):
        result = RadarProtocol.parse_cfar_packet(b"\xDD\x00\x00\x00" + b"\x00" * 8)
        self.assertIsNone(result)

    def test_bad_header(self):
        pkt = bytearray(make_cfar_packet(5, 5, 1000, 500))
        pkt[0] = 0xBB  # wrong header
        result = RadarProtocol.parse_cfar_packet(bytes(pkt))
        self.assertIsNone(result)

    def test_bad_footer(self):
        pkt = bytearray(make_cfar_packet(5, 5, 1000, 500))
        pkt[12] = 0xAA  # corrupt footer
        result = RadarProtocol.parse_cfar_packet(bytes(pkt))
        self.assertIsNone(result)


# ============================================================================
# v9: find_packet_boundaries() with mixed packet types
# ============================================================================

class TestFindPacketBoundariesV9(unittest.TestCase):
    """Tests for scanning buffers with all v9 packet types."""

    def test_single_doppler_packet(self):
        pkt = make_doppler_packet(5, 3, 100, 200)
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(len(result), 1)
        start, end, ptype = result[0]
        self.assertEqual(start, 0)
        self.assertEqual(end, 16)
        self.assertEqual(ptype, "doppler")

    def test_single_cfar_packet(self):
        pkt = make_cfar_packet(10, 8, 5000, 3000)
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(len(result), 1)
        start, end, ptype = result[0]
        self.assertEqual(start, 0)
        self.assertEqual(end, 16)
        self.assertEqual(ptype, "cfar")

    def test_mixed_all_types(self):
        """Buffer with range + doppler + cfar + status packets."""
        buf = (make_data_packet(100, 200)                      # 24 bytes
               + make_doppler_packet(5, 3, 100, 200)           # 16 bytes
               + make_cfar_packet(10, 8, 5000, 3000)           # 16 bytes
               + make_status_packet(threshold=42))             # 40 bytes
        result = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(result), 4)
        self.assertEqual(result[0][2], "data")
        self.assertEqual(result[1][2], "doppler")
        self.assertEqual(result[2][2], "cfar")
        self.assertEqual(result[3][2], "status")

    def test_consecutive_doppler_packets(self):
        buf = b""
        for i in range(10):
            buf += make_doppler_packet(i, i % 32, i * 100, i * 50)
        result = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(result), 10)
        for start, end, ptype in result:
            self.assertEqual(ptype, "doppler")
            self.assertEqual(end - start, 16)

    def test_range_then_doppler_burst(self):
        """Typical v9 pattern: range packets first, then Doppler burst."""
        buf = b""
        # 64 range packets (1 chirp)
        for rbin in range(NUM_RANGE_BINS):
            buf += make_data_packet(rbin * 10, rbin * 5)
        # 64 Doppler packets
        for rbin in range(NUM_RANGE_BINS):
            buf += make_doppler_packet(rbin, 0, 100, 50)
        result = RadarProtocol.find_packet_boundaries(buf)
        data_count = sum(1 for _, _, t in result if t == "data")
        doppler_count = sum(1 for _, _, t in result if t == "doppler")
        self.assertEqual(data_count, NUM_RANGE_BINS)
        self.assertEqual(doppler_count, NUM_RANGE_BINS)

    def test_garbage_between_doppler_packets(self):
        pkt1 = make_doppler_packet(0, 0, 100, 200)
        garbage = bytes([0x12, 0x34, 0x56, 0x78])
        pkt2 = make_doppler_packet(1, 1, 300, 400)
        buf = pkt1 + garbage + pkt2
        result = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0][2], "doppler")
        self.assertEqual(result[1][2], "doppler")

    def test_incomplete_doppler_packet(self):
        """Buffer has CC header but not enough bytes."""
        pkt = make_doppler_packet(0, 0, 100, 200)[:12]  # truncated
        result = RadarProtocol.find_packet_boundaries(pkt)
        self.assertEqual(result, [])

    def test_v9_packet_size_constants(self):
        self.assertEqual(RadarProtocol.V9_DOPPLER_PACKET_SIZE, 16)
        self.assertEqual(RadarProtocol.V9_CFAR_PACKET_SIZE, 16)


# ============================================================================
# v9: RadarAcquisition full pipeline frame assembly
# ============================================================================

class TestRadarAcquisitionV9(unittest.TestCase):
    """Tests for v9 full pipeline mode frame assembly."""

    def _build_v9_frame_data(self, include_range=True, include_cfar=True):
        """Build a complete v9 frame's worth of raw bytes.

        Returns raw bytes containing:
          - 2048 range packets (32 chirps x 64 bins) if include_range
          - 2048 Doppler packets (64 range x 32 doppler)
          - Some CFAR detection packets if include_cfar
              (interleaved before the last Doppler packet so they
               arrive before frame finalization)
        """
        buf = bytearray()

        # Range packets (32 chirps x 64 bins)
        if include_range:
            for chirp in range(32):
                for rbin in range(NUM_RANGE_BINS):
                    ri = (chirp * 100 + rbin) & 0x7FFF
                    rq = (chirp * 50 + rbin) & 0x7FFF
                    buf += make_data_packet(ri, rq)

        # CFAR detections (placed before Doppler packets so they arrive
        # before the frame is finalized by the 2048th Doppler packet)
        cfar_detections = []
        if include_cfar:
            cfar_detections = [(10, 5, 8000, 3000), (20, 15, 6000, 2500),
                               (30, 25, 9000, 4000)]
            for rbin, dbin, mag, thr in cfar_detections:
                buf += make_cfar_packet(rbin, dbin, mag, thr)

        # Doppler packets (64 range bins x 32 doppler bins)
        for rbin in range(NUM_RANGE_BINS):
            for dbin in range(NUM_DOPPLER_BINS):
                di = (rbin * 10 + dbin) & 0x7FFF
                dq = (rbin * 5 + dbin) & 0x7FFF
                buf += make_doppler_packet(rbin, dbin, di, dq)

        return bytes(buf)

    def test_v9_frame_assembly_with_all_types(self):
        """Feed range + doppler + cfar packets, verify frame assembly."""
        raw = self._build_v9_frame_data()
        fq = queue.Queue(maxsize=10)
        acq = RadarAcquisition(None, fq, range_only=False)

        # Parse and ingest all packets
        packets = RadarProtocol.find_packet_boundaries(raw)

        data_count = 0
        doppler_count = 0
        cfar_count = 0
        for start, end, ptype in packets:
            if ptype == "data":
                parsed = RadarProtocol.parse_data_packet(raw[start:end])
                if parsed:
                    acq._got_doppler = True  # force v9 mode
                    acq._ingest_v9_range(parsed)
                    data_count += 1
            elif ptype == "doppler":
                parsed = RadarProtocol.parse_doppler_packet(raw[start:end])
                if parsed:
                    acq._got_doppler = True
                    acq._ingest_v9_doppler(parsed)
                    doppler_count += 1
            elif ptype == "cfar":
                parsed = RadarProtocol.parse_cfar_packet(raw[start:end])
                if parsed:
                    acq._got_cfar = True
                    acq._ingest_v9_cfar(parsed)
                    cfar_count += 1

        self.assertEqual(data_count, 2048)    # 32 chirps x 64 bins
        self.assertEqual(doppler_count, 2048)  # 64 x 32
        self.assertEqual(cfar_count, 3)

        # Frame should have been finalized (doppler_count hit 2048)
        self.assertFalse(fq.empty(), "Frame should have been pushed to queue")
        frame = fq.get_nowait()
        self.assertIsInstance(frame, RadarFrame)

        # Check Doppler data was placed correctly
        # rbin=10, dbin=5: di = 10*10 + 5 = 105, dq = 10*5 + 5 = 55
        self.assertEqual(frame.range_doppler_i[10, 5], 105)
        self.assertEqual(frame.range_doppler_q[10, 5], 55)

        # Check magnitude
        expected_mag = abs(105) + abs(55)
        self.assertEqual(frame.magnitude[10, 5], expected_mag)

        # Check CFAR detections
        self.assertEqual(frame.detections[10, 5], 1)
        self.assertEqual(frame.detections[20, 15], 1)
        self.assertEqual(frame.detections[30, 25], 1)
        self.assertEqual(frame.detection_count, 3)

    def test_v9_doppler_only_mode(self):
        """Doppler-only (no range packets) still produces a frame."""
        raw = self._build_v9_frame_data(include_range=False, include_cfar=False)
        fq = queue.Queue(maxsize=10)
        acq = RadarAcquisition(None, fq, range_only=False)
        acq._got_doppler = True

        packets = RadarProtocol.find_packet_boundaries(raw)
        for start, end, ptype in packets:
            if ptype == "doppler":
                parsed = RadarProtocol.parse_doppler_packet(raw[start:end])
                if parsed:
                    acq._ingest_v9_doppler(parsed)

        self.assertFalse(fq.empty())
        frame = fq.get_nowait()

        # Range profile should be derived from magnitude (sum across Doppler)
        self.assertGreater(np.sum(frame.range_profile), 0)
        # No CFAR detections
        self.assertEqual(frame.detection_count, 0)

    def test_v9_range_profile_from_last_chirp(self):
        """Range profile comes from last chirp's 0xAA data."""
        fq = queue.Queue(maxsize=10)
        acq = RadarAcquisition(None, fq, range_only=False)
        acq._got_doppler = True

        # Feed 32 chirps of range data with distinct values
        for chirp in range(32):
            for rbin in range(NUM_RANGE_BINS):
                ri = chirp * 100 + rbin
                rq = chirp * 50 + rbin
                parsed = {"range_i": ri, "range_q": rq, "range_value": 0}
                acq._ingest_v9_range(parsed)

        # Now feed all Doppler packets to trigger frame finalization
        for rbin in range(NUM_RANGE_BINS):
            for dbin in range(NUM_DOPPLER_BINS):
                parsed = {"range_bin": rbin, "doppler_bin": dbin,
                          "doppler_i": 0, "doppler_q": 0, "sub_frame": 0}
                acq._ingest_v9_doppler(parsed)

        frame = fq.get_nowait()
        # Last chirp (chirp=31): ri = 31*100 + rbin, rq = 31*50 + rbin
        # Magnitude = |ri| + |rq| = (3100 + rbin) + (1550 + rbin) = 4650 + 2*rbin
        for rbin in range(NUM_RANGE_BINS):
            expected_mag = abs(31 * 100 + rbin) + abs(31 * 50 + rbin)
            self.assertAlmostEqual(frame.range_profile[rbin], expected_mag,
                                   msg=f"Range profile mismatch at bin {rbin}")

    def test_v9_frame_counter_increments(self):
        """Multiple v9 frames should have incrementing frame numbers."""
        fq = queue.Queue(maxsize=10)
        acq = RadarAcquisition(None, fq, range_only=False)
        acq._got_doppler = True

        for frame_idx in range(3):
            for rbin in range(NUM_RANGE_BINS):
                for dbin in range(NUM_DOPPLER_BINS):
                    parsed = {"range_bin": rbin, "doppler_bin": dbin,
                              "doppler_i": 0, "doppler_q": 0, "sub_frame": 0}
                    acq._ingest_v9_doppler(parsed)

        frames = []
        while not fq.empty():
            frames.append(fq.get_nowait())
        self.assertEqual(len(frames), 3)
        for i in range(len(frames)):
            self.assertEqual(frames[i].frame_number, i)


# ============================================================================
# v9: Packet constants
# ============================================================================

class TestV9PacketConstants(unittest.TestCase):

    def test_doppler_header(self):
        self.assertEqual(HEADER_DOPPLER, 0xCC)

    def test_cfar_header(self):
        self.assertEqual(HEADER_CFAR, 0xDD)

    def test_doppler_packet_size(self):
        pkt = make_doppler_packet(0, 0, 0, 0)
        self.assertEqual(len(pkt), RadarProtocol.V9_DOPPLER_PACKET_SIZE)

    def test_cfar_packet_size(self):
        pkt = make_cfar_packet(0, 0, 0, 0)
        self.assertEqual(len(pkt), RadarProtocol.V9_CFAR_PACKET_SIZE)


if __name__ == "__main__":
    unittest.main()
