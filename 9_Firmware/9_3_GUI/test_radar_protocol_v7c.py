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
import time
import unittest
import numpy as np

from radar_protocol import (
    RadarProtocol,
    RadarFrame,
    StatusResponse,
    FT601Connection,
    RadarAcquisition,
    HEADER_BYTE,
    FOOTER_BYTE,
    STATUS_HEADER_BYTE,
    NUM_RANGE_BINS,
    NUM_DOPPLER_BINS,
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


if __name__ == "__main__":
    unittest.main()
