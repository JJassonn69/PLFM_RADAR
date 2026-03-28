#!/usr/bin/env python3
"""
AERIS-10 Radar Protocol Layer
===============================
Pure-logic module for FT601 packet parsing and command building.
No GUI dependencies — safe to import from tests and headless scripts.

Matches usb_data_interface.v v9 packet format.

USB Packet Protocol (v9 — all transfers use BE=1111, 4-byte aligned):
  TX (FPGA→Host):
    Range packet  (24 bytes): [0xAA word] [range 4×4B] [0x55 word]
    Doppler packet(16 bytes): [0xCC word] [coords+I] [Q,I] [0x55 word]   (v9+)
    CFAR packet   (16 bytes): [0xDD word] [detect report] [thresh] [0x55] (v9+)
    Status packet (40 bytes): [0xBB word] [config 6×4B] [debug 2×4B] [0x55 word]
  RX (Host→FPGA):
    Command word:  {opcode[31:24], addr[23:16], value[15:0]}

  All words are little-endian on USB (DATA[7:0] first).
  Header/footer markers are in the LSB of the 32-bit word.
"""

import json
import os
import socket
import struct
import time
import threading
import queue
import logging
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict, Any
from enum import IntEnum
from collections import deque

import numpy as np

log = logging.getLogger("radar_protocol")

# ============================================================================
# Constants matching usb_data_interface.v
# ============================================================================

HEADER_BYTE = 0xAA
HEADER_DOPPLER = 0xCC    # v9: Doppler cell packet
HEADER_CFAR = 0xDD       # v9: CFAR detection packet
FOOTER_BYTE = 0x55
STATUS_HEADER_BYTE = 0xBB

NUM_RANGE_BINS = 64
NUM_DOPPLER_BINS = 32
NUM_CELLS = NUM_RANGE_BINS * NUM_DOPPLER_BINS  # 2048

WATERFALL_DEPTH = 64


class Opcode(IntEnum):
    """Host register opcodes — matches radar_system_top.v command decode exactly.

    RTL decode is at radar_system_top.v lines 811-853.  Keep this enum
    in sync whenever the Verilog case statement changes.

    Command word format: {opcode[31:24], addr[23:16], value[15:0]}
    """
    # Core control
    RADAR_MODE          = 0x01  # host_radar_mode [1:0]
    TRIGGER             = 0x02  # host_trigger_pulse (self-clearing)
    THRESHOLD           = 0x03  # host_detect_threshold [15:0]
    STREAM_CONTROL      = 0x04  # host_stream_control [2:0] (bit0=range, bit1=doppler, bit2=cfar)
    # Chirp timing configuration
    LONG_CHIRP          = 0x10  # host_long_chirp_cycles [15:0] (default 3000)
    LONG_LISTEN         = 0x11  # host_long_listen_cycles [15:0] (default 13700)
    GUARD               = 0x12  # host_guard_cycles [15:0] (default 17540)
    SHORT_CHIRP         = 0x13  # host_short_chirp_cycles [15:0] (default 50)
    SHORT_LISTEN        = 0x14  # host_short_listen_cycles [15:0] (default 17450)
    CHIRPS_PER_ELEV     = 0x15  # host_chirps_per_elev [5:0] (default 32)
    GAIN_SHIFT          = 0x16  # host_gain_shift [3:0] (bit3=dir, bit2:0=shift)
    # Range and detection configuration
    RANGE_MODE          = 0x20  # host_range_mode [1:0] (00=auto, 01=short, 10=long)
    CFAR_GUARD          = 0x21  # host_cfar_guard [3:0] (guard cells per side)
    CFAR_TRAIN          = 0x22  # host_cfar_train [4:0] (training cells per side)
    CFAR_ALPHA          = 0x23  # host_cfar_alpha [7:0] (Q4.4 threshold multiplier)
    CFAR_MODE           = 0x24  # host_cfar_mode [1:0] (00=CA, 01=GO, 10=SO)
    CFAR_ENABLE         = 0x25  # host_cfar_enable (1=CFAR, 0=simple threshold)
    MTI_ENABLE          = 0x26  # host_mti_enable (1=active, 0=pass-through)
    DC_NOTCH_WIDTH      = 0x27  # host_dc_notch_width [2:0] (0=off, 1..7 bins)
    # Board self-test
    SELF_TEST_TRIGGER   = 0x30  # host_self_test_trigger (self-clearing)
    SELF_TEST_STATUS    = 0x31  # host_status_request alias (triggers readback)
    # Status readback
    STATUS_REQUEST      = 0xFF  # host_status_request (triggers status packet)


# CFAR mode display names (index = mode value, matches cfar_ca.v)
CFAR_MODE_NAMES = ["CA-CFAR", "GO-CFAR", "SO-CFAR"]
CFAR_MODE_VALUES = {name: idx for idx, name in enumerate(CFAR_MODE_NAMES)}

# Parameter validation rules: opcode → (min, max, description)
# Matches RTL register widths in radar_system_top.v
PARAM_VALIDATION = {
    0x03: (0, 65535, "Threshold [16-bit]"),
    0x04: (0, 7, "Stream control [3-bit bitmask]"),
    0x16: (0, 15, "Gain shift [4-bit: bit3=dir, 2:0=shift]"),
    0x20: (0, 2, "Range mode [0=auto, 1=short, 2=long]"),
    0x21: (0, 15, "CFAR guard cells [4-bit]"),
    0x22: (0, 31, "CFAR training cells [5-bit]"),
    0x23: (0, 255, "CFAR alpha Q4.4 [8-bit]"),
    0x24: (0, 2, "CFAR mode [0=CA, 1=GO, 2=SO]"),
    0x27: (0, 7, "DC notch +/-width [3-bit]"),
}

# AERIS-10 radar configuration for physical axis labels.
# Used by dashboards when not in replay mode (which provides CN0566 config).
AERIS10_CONFIG = {
    "sample_rate": 400e6,        # Hz — AERIS-10 ADC sample rate
    "bandwidth": 500e6,          # Hz — chirp bandwidth
    "ramp_time": 300e-6,         # s  — chirp ramp time
    "center_freq": 10.5e9,       # Hz — X-band center frequency
    "fft_size": 1024,            # FFT length (range)
    "decimation": 16,            # peak decimation ratio
    "num_chirps": 32,            # chirps per Doppler frame
    "range_formula": "if",       # AERIS-10: range = c/(2*BW) * decimation per bin
}


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class RadarFrame:
    """One complete radar frame (64 range × 32 Doppler)."""
    timestamp: float = 0.0
    range_doppler_i: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.int16))
    range_doppler_q: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.int16))
    magnitude: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.float64))
    detections: np.ndarray = field(
        default_factory=lambda: np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.uint8))
    range_profile: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_RANGE_BINS, dtype=np.float64))
    detection_count: int = 0
    frame_number: int = 0


@dataclass
class StatusResponse:
    """Parsed status response from FPGA (10-word packet as of v7b/v7c)."""
    radar_mode: int = 0
    stream_ctrl: int = 0
    cfar_threshold: int = 0
    long_chirp: int = 0
    long_listen: int = 0
    guard: int = 0
    short_chirp: int = 0
    short_listen: int = 0
    chirps_per_elev: int = 0
    range_mode: int = 0
    # Self-test results (word 6)
    self_test_flags: int = 0     # 5-bit result flags [4:0]
    self_test_detail: int = 0    # 8-bit detail code [7:0]
    self_test_busy: int = 0      # 1-bit busy flag
    # Debug counters (words 7-8, added in v7b)
    dbg_wr_strobes: int = 0      # WR_N strobe count
    dbg_txe_blocks: int = 0      # TXE_N backpressure count
    dbg_pkt_starts: int = 0      # Packet start count
    dbg_pkt_completions: int = 0 # Packet completion count


# ============================================================================
# Protocol: Packet Parsing & Building
# ============================================================================

def _to_signed16(val: int) -> int:
    """Convert unsigned 16-bit integer to signed (two's complement)."""
    val = val & 0xFFFF
    return val - 0x10000 if val >= 0x8000 else val


class RadarProtocol:
    """
    Parse FPGA→Host packets and build Host→FPGA command words.
    Matches usb_data_interface.v packet format exactly.
    """

    @staticmethod
    def build_command(opcode: int, value: int, addr: int = 0) -> bytes:
        """
        Build a 32-bit command word: {opcode[31:24], addr[23:16], value[15:0]}.
        Returns 4 bytes, big-endian (MSB first as FT601 expects).
        """
        word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
        return struct.pack(">I", word)

    @staticmethod
    def parse_data_packet(raw: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse a single v7c data packet from the FPGA byte stream.
        Returns dict with keys: 'range_value' (uint32), or None if invalid.

        v7c packet format (range-only, stream_control=0x01):
          6 x 4 bytes = 24 bytes total, ALL transfers use BE=1111.
          FT601 byte lanes are little-endian.

          Word 0: {24'h000000, 0xAA}  = AA 00 00 00  (header)
          Word 1: range_profile_cap   = actual 32-bit range data
          Word 2: {range[23:0], 8'h00}  (shifted copy — ignore)
          Word 3: {range[15:0], 16'h00} (shifted copy — ignore)
          Word 4: {range[7:0], 24'h00}  (shifted copy — ignore)
          Word 5: {24'h000000, 0x55}  = 55 00 00 00  (footer)

        Each 4-byte word is read as little-endian uint32 via struct "<I".

        The dev wrapper generates synthetic range data:
          range_profile_reg = {hb_counter[31:16], hb_counter[15:0] ^ 16'hA5A5}
        So word 1 contains the full 32-bit value; words 2-4 are shifted.
        """
        if len(raw) < 24:
            return None

        # Parse all 6 words as little-endian uint32
        w0 = struct.unpack_from("<I", raw, 0)[0]
        w1 = struct.unpack_from("<I", raw, 4)[0]
        # w2, w3, w4 are shifted copies — skip
        w5 = struct.unpack_from("<I", raw, 20)[0]

        # Validate header and footer markers
        if (w0 & 0xFF) != HEADER_BYTE:
            return None
        if (w5 & 0xFF) != FOOTER_BYTE:
            return None

        # Word 1 is the real range data
        # In dev wrapper: {hb_counter[31:16], hb_counter[15:0] ^ 0xA5A5}
        # In real hardware: {range_q[15:0], range_i[15:0]}
        result = {
            "range_value": w1,
            "range_i": _to_signed16(w1 & 0xFFFF),
            "range_q": _to_signed16((w1 >> 16) & 0xFFFF),
        }
        return result

    @staticmethod
    def parse_doppler_packet(raw: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse a v9 Doppler cell packet (0xCC header, 16 bytes).

        Word 0: {24'h000000, 0xCC}      header
        Word 1: {range_bin[5:0], doppler_bin[4:0], sub_frame, 4'b0000, I[15:0]}
        Word 2: {Q[15:0], I[15:0]}      (I duplicated for easy parsing)
        Word 3: {24'h000000, 0x55}      footer

        All words are little-endian on USB (DATA[7:0] first).
        """
        if len(raw) < 16:
            return None

        w0 = struct.unpack_from("<I", raw, 0)[0]
        w1 = struct.unpack_from("<I", raw, 4)[0]
        w2 = struct.unpack_from("<I", raw, 8)[0]
        w3 = struct.unpack_from("<I", raw, 12)[0]

        if (w0 & 0xFF) != HEADER_DOPPLER:
            return None
        if (w3 & 0xFF) != FOOTER_BYTE:
            return None

        # Word 1: {range_bin[31:26], doppler_bin[25:21], sub_frame[20],
        #           4'b0000[19:16], I[15:0]}
        doppler_i = _to_signed16(w1 & 0xFFFF)
        sub_frame = (w1 >> 20) & 0x01
        doppler_bin = (w1 >> 21) & 0x1F
        range_bin = (w1 >> 26) & 0x3F

        # Word 2: {Q[31:16], I[15:0]}
        doppler_q = _to_signed16((w2 >> 16) & 0xFFFF)
        # I in lower 16 bits is a duplicate — we already have it from w1

        return {
            "range_bin": range_bin,
            "doppler_bin": doppler_bin,
            "sub_frame": sub_frame,
            "doppler_i": doppler_i,
            "doppler_q": doppler_q,
        }

    @staticmethod
    def parse_cfar_packet(raw: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse a v9 CFAR detection packet (0xDD header, 16 bytes).
        Only sent for cells where detect_flag=1.

        Word 0: {24'h000000, 0xDD}      header
        Word 1: {flag[31], range[30:25], doppler[24:20], 3'b000[19:17],
                  magnitude[16:0]}
        Word 2: {15'b0[31:17], threshold[16:0]}
        Word 3: {24'h000000, 0x55}      footer

        All words are little-endian on USB (DATA[7:0] first).
        """
        if len(raw) < 16:
            return None

        w0 = struct.unpack_from("<I", raw, 0)[0]
        w1 = struct.unpack_from("<I", raw, 4)[0]
        w2 = struct.unpack_from("<I", raw, 8)[0]
        w3 = struct.unpack_from("<I", raw, 12)[0]

        if (w0 & 0xFF) != HEADER_CFAR:
            return None
        if (w3 & 0xFF) != FOOTER_BYTE:
            return None

        # Word 1: {flag[31], range[30:25], doppler[24:20], 3'b000[19:17],
        #           magnitude[16:0]}
        magnitude = w1 & 0x1FFFF         # [16:0]
        doppler = (w1 >> 20) & 0x1F      # [24:20]
        range_bin = (w1 >> 25) & 0x3F    # [30:25]
        detect_flag = (w1 >> 31) & 0x01  # [31]

        # Word 2: {15'b0, threshold[16:0]}
        threshold = w2 & 0x1FFFF

        return {
            "detect_flag": detect_flag,
            "range_bin": range_bin,
            "doppler_bin": doppler,
            "magnitude": magnitude,
            "threshold": threshold,
        }

    @staticmethod
    def parse_status_packet(raw: bytes) -> Optional[StatusResponse]:
        """
        Parse a v7c status response packet.
        Format: 10 x 4 bytes = 40 bytes, all BE=1111, little-endian.
          Word 0: {24'h000000, 0xBB}  = BB header marker
          Words 1-6: config registers
          Word 7: {dbg_wr_strobes[15:0], dbg_txe_blocks[15:0]}
          Word 8: {dbg_pkt_starts[15:0], dbg_pkt_completions[15:0]}
          Word 9: {24'h000000, 0x55}  = footer

        Ported from diag_ft601_v7b.py parse_status_packet() which is proven.
        """
        if len(raw) < 4:
            return None

        # Parse all available 32-bit words (little-endian)
        words = []
        for i in range(0, len(raw) - 3, 4):
            w = struct.unpack_from("<I", raw, i)[0]
            words.append(w)

        if not words:
            return None

        # Look for BB header marker
        bb_idx = None
        for i, w in enumerate(words):
            if (w & 0xFF) == STATUS_HEADER_BYTE:
                bb_idx = i
                break

        if bb_idx is None:
            return None

        remaining = words[bb_idx:]
        if len(remaining) < 10:
            log.warning(f"Status packet too short: {len(remaining)} words "
                        f"after BB (expected 10)")
            return None

        # Verify footer
        footer_byte = remaining[9] & 0xFF
        if footer_byte != FOOTER_BYTE:
            log.warning(f"Status footer mismatch: 0x{footer_byte:02X} "
                        f"(expected 0x{FOOTER_BYTE:02X})")

        sr = StatusResponse()

        # Word 1: {0xFF, 3'b0, mode[1:0], 5'b0, stream[2:0], threshold[15:0]}
        w0 = remaining[1]
        sr.cfar_threshold = w0 & 0xFFFF
        sr.stream_ctrl = (w0 >> 16) & 0x07
        sr.radar_mode = (w0 >> 21) & 0x03

        # Word 2: {long_chirp[31:16], long_listen[15:0]}
        w1 = remaining[2]
        sr.long_listen = w1 & 0xFFFF
        sr.long_chirp = (w1 >> 16) & 0xFFFF

        # Word 3: {guard[31:16], short_chirp[15:0]}
        w2 = remaining[3]
        sr.short_chirp = w2 & 0xFFFF
        sr.guard = (w2 >> 16) & 0xFFFF

        # Word 4: {short_listen[31:16], 10'd0, chirps_per_elev[5:0]}
        w3 = remaining[4]
        sr.chirps_per_elev = w3 & 0x3F
        sr.short_listen = (w3 >> 16) & 0xFFFF

        # Word 5: {30'd0, range_mode[1:0]}
        w4 = remaining[5]
        sr.range_mode = w4 & 0x03

        # Word 6: {7'd0, self_test_busy, 8'd0, self_test_detail[7:0],
        #           3'd0, self_test_flags[4:0]}
        w5 = remaining[6]
        sr.self_test_flags = w5 & 0x1F
        sr.self_test_detail = (w5 >> 8) & 0xFF
        sr.self_test_busy = (w5 >> 24) & 0x01

        # Debug words (v7b+ instrumentation)
        # Word 7: {dbg_wr_strobes[15:0], dbg_txe_blocks[15:0]}
        w6 = remaining[7]
        sr.dbg_wr_strobes = (w6 >> 16) & 0xFFFF
        sr.dbg_txe_blocks = w6 & 0xFFFF

        # Word 8: {dbg_pkt_starts[15:0], dbg_pkt_completions[15:0]}
        w7 = remaining[8]
        sr.dbg_pkt_starts = (w7 >> 16) & 0xFFFF
        sr.dbg_pkt_completions = w7 & 0xFFFF

        return sr

    # Packet sizes (all BE=1111, 4-byte aligned)
    V7C_DATA_PACKET_SIZE = 24    # 6 x 4 bytes (range-only, 0xAA)
    V7C_STATUS_PACKET_SIZE = 40  # 10 x 4 bytes (status, 0xBB)
    V9_DOPPLER_PACKET_SIZE = 16  # 4 x 4 bytes (Doppler cell, 0xCC)
    V9_CFAR_PACKET_SIZE = 16     # 4 x 4 bytes (CFAR detection, 0xDD)

    @staticmethod
    def find_packet_boundaries(buf: bytes) -> List[Tuple[int, int, str]]:
        """
        Scan buffer for packet start markers (4-byte aligned words).

        Supported packet types:
          0xAA 00 00 00 → Range data   (24 bytes, v7c+)
          0xBB 00 00 00 → Status       (40 bytes, v7c+)
          0xCC 00 00 00 → Doppler cell (16 bytes, v9+)
          0xDD 00 00 00 → CFAR detect  (16 bytes, v9+)

        Returns list of (start_idx, expected_end_idx, packet_type).
        Only returns packets where the footer word is also valid.
        """
        packets = []
        i = 0
        buf_len = len(buf)
        while i + 3 < buf_len:
            b0 = buf[i]
            if buf[i+1] == 0 and buf[i+2] == 0 and buf[i+3] == 0:
                # Determine packet type and size from header byte
                if b0 == HEADER_BYTE:
                    pkt_size = RadarProtocol.V7C_DATA_PACKET_SIZE
                    pkt_type = "data"
                elif b0 == STATUS_HEADER_BYTE:
                    pkt_size = RadarProtocol.V7C_STATUS_PACKET_SIZE
                    pkt_type = "status"
                elif b0 == HEADER_DOPPLER:
                    pkt_size = RadarProtocol.V9_DOPPLER_PACKET_SIZE
                    pkt_type = "doppler"
                elif b0 == HEADER_CFAR:
                    pkt_size = RadarProtocol.V9_CFAR_PACKET_SIZE
                    pkt_type = "cfar"
                else:
                    i += 1
                    continue

                end = i + pkt_size
                if end <= buf_len:
                    # Verify footer word
                    if (buf[end-4] == FOOTER_BYTE and buf[end-3] == 0
                            and buf[end-2] == 0 and buf[end-1] == 0):
                        packets.append((i, end, pkt_type))
                        i = end
                        continue
                else:
                    break  # Not enough data for full packet
            i += 1  # Scan byte-by-byte to find next marker
        return packets


# ============================================================================
# FT601 USB Connection
# ============================================================================

# Optional ftd3xx import
try:
    import ftd3xx
    import ftd3xx._ftd3xx_linux as _ftd3xx_ll
    FTD3XX_AVAILABLE = True
except ImportError:
    FTD3XX_AVAILABLE = False
    _ftd3xx_ll = None

import ctypes

# FT601 pipe IDs for 245 Sync FIFO 1-channel mode
PIPE_OUT = 0x02  # Host → FPGA
PIPE_IN  = 0x82  # FPGA → Host


def _raw_write(handle, pipe: int, data: bytes, timeout_ms: int = 1000) -> int:
    """Low-level D3XX write using FT_WritePipeEx (proven in diag_ft601_v7b.py)."""
    buf = ctypes.create_string_buffer(data, len(data))
    xfer = ctypes.c_ulong(0)
    status = _ftd3xx_ll.FT_WritePipeEx(
        handle, ctypes.c_ubyte(pipe),
        buf, ctypes.c_ulong(len(data)),
        ctypes.byref(xfer), ctypes.c_ulong(timeout_ms))
    return xfer.value if status == 0 else -status


def _raw_read(handle, pipe: int, size: int, timeout_ms: int = 2000) -> bytes:
    """Low-level D3XX read using FT_ReadPipeEx (proven in diag_ft601_v7b.py)."""
    buf = ctypes.create_string_buffer(size)
    xfer = ctypes.c_ulong(0)
    status = _ftd3xx_ll.FT_ReadPipeEx(
        handle, ctypes.c_ubyte(pipe),
        buf, ctypes.c_ulong(size),
        ctypes.byref(xfer), ctypes.c_ulong(timeout_ms))
    return buf.raw[:xfer.value] if status == 0 else b""


class FT601Connection:
    """
    FT601 USB 3.0 FIFO bridge communication.
    Uses raw ctypes D3XX API (FT_ReadPipeEx/FT_WritePipeEx) for reliable
    USB transfers — the high-level readPipe/writePipe wrappers are unreliable.
    Supports mock mode for GUI development without hardware.
    """

    def __init__(self, mock: bool = True, moving_target: bool = False):
        self._mock = mock
        self._device = None
        self._handle = None  # raw ctypes handle for _raw_read/_raw_write
        self._lock = threading.Lock()
        self.is_open = False
        # Mock state
        self._mock_frame_num = 0
        self._mock_sample_idx = 0
        self._mock_rng = np.random.RandomState(42)
        # Moving target simulation
        self._moving_target = moving_target
        self._target_range_bin = 55  # Start far away
        self._target_velocity = -0.3  # Moving toward radar (negative = decreasing range)

    def open(self, device_index: int = 0) -> bool:
        if self._mock:
            self.is_open = True
            log.info("FT601 mock device opened (no hardware)")
            return True

        if not FTD3XX_AVAILABLE:
            log.error("ftd3xx not installed — cannot open real FT601 device")
            return False

        try:
            self._device = ftd3xx.create(device_index)
            if self._device is None:
                log.error("ftd3xx.create returned None")
                return False
            self._handle = self._device.handle
            # CRITICAL: Reset and re-initialize USB stream pipes.
            # Without this sequence, reads may timeout indefinitely after
            # FPGA reprogramming or USB reconnects (discovered 2026-03-28).
            self._init_pipes()
            # Flush any stale data from the IN pipe
            self._flush()
            self.is_open = True
            log.info(f"FT601 device {device_index} opened (raw D3XX API)")
            return True
        except Exception as e:
            log.error(f"FT601 open failed: {e}")
            return False

    def close(self):
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None
            self._handle = None
        self.is_open = False

    def _flush(self):
        """Aggressively flush the IN pipe to clear stale data."""
        if self._handle is None:
            return
        for _ in range(50):
            d = _raw_read(self._handle, PIPE_IN, 16384, timeout_ms=50)
            if not d:
                break
        try:
            self._device.flushPipe(PIPE_IN)
        except Exception:
            pass

    def _init_pipes(self):
        """Reset and re-initialize USB stream pipes.

        The FT601 D3XX driver can get into a state where reads timeout
        indefinitely (st=19) after FPGA reprogramming or USB cable
        reconnects.  The fix is to abort, flush, clear, and re-set the
        stream pipes before any transfers.  Discovered 2026-03-28.
        """
        if _ftd3xx_ll is None or self._handle is None:
            return
        h = self._handle
        zero = ctypes.c_ubyte(0)
        try:
            _ftd3xx_ll.FT_AbortPipe(h, ctypes.c_ubyte(PIPE_IN))
            _ftd3xx_ll.FT_AbortPipe(h, ctypes.c_ubyte(PIPE_OUT))
            _ftd3xx_ll.FT_FlushPipe(h, ctypes.c_ubyte(PIPE_IN))
            _ftd3xx_ll.FT_ClearStreamPipe(h, zero, zero, ctypes.c_ubyte(PIPE_IN))
            _ftd3xx_ll.FT_ClearStreamPipe(h, zero, zero, ctypes.c_ubyte(PIPE_OUT))
            _ftd3xx_ll.FT_SetStreamPipe(h, zero, zero, ctypes.c_ubyte(PIPE_IN),
                                         ctypes.c_ulong(65536))
            _ftd3xx_ll.FT_SetStreamPipe(h, zero, zero, ctypes.c_ubyte(PIPE_OUT),
                                         ctypes.c_ulong(4))
            log.info("FT601 stream pipes initialized (abort/flush/clear/set)")
        except Exception as e:
            log.warning(f"FT601 pipe init failed (non-fatal): {e}")

    def read(self, size: int = 16384) -> Optional[bytes]:
        """Read raw bytes from FT601. Returns None on error/timeout."""
        if not self.is_open:
            return None

        if self._mock:
            return self._mock_read(size)

        with self._lock:
            try:
                data = _raw_read(self._handle, PIPE_IN, size, timeout_ms=100)
                return data if data else None
            except Exception as e:
                log.error(f"FT601 read error: {e}")
                return None

    def write(self, data: bytes) -> bool:
        """Write raw bytes to FT601."""
        if not self.is_open:
            return False

        if self._mock:
            log.info(f"FT601 mock write: {data.hex()}")
            return True

        with self._lock:
            try:
                n = _raw_write(self._handle, PIPE_OUT, data, timeout_ms=1000)
                return n > 0
            except Exception as e:
                log.error(f"FT601 write error: {e}")
                return False

    def _mock_read(self, size: int) -> bytes:
        """
        Generate synthetic radar data packets in v7c format for testing.

        v7c format: 24 bytes per packet (6 x 4-byte words, all BE=1111).
        Each packet = one range bin (range-only streaming).

        In mock mode, we emit NUM_RANGE_BINS packets per frame (one per
        range bin), matching what the dev wrapper produces.

        Scene: two targets with noise floor
          Target A — stationary: range bin ~20
          Target B — moving:     range bin ~40

        With --moving-target: single target approaches from far range
        """
        time.sleep(0.01)  # Simulate USB latency

        # Update target position for moving target simulation
        if self._moving_target:
            self._target_range_bin += self._target_velocity
            # Wrap around: if target reaches 0, reset to far range
            if self._target_range_bin < 5:
                self._target_range_bin = 55
                log.info("Target wrapped: resetting to far range (bin 55)")
            elif int(self._target_range_bin) != int(self._target_range_bin - self._target_velocity):
                log.debug(f"Target at range bin {int(self._target_range_bin)}")

        num_packets = min(NUM_RANGE_BINS, size // 24)
        buf = bytearray(num_packets * 24)
        pos = 0

        for _ in range(num_packets):
            rbin = self._mock_sample_idx

            # Noise floor
            noise_i = int(self._mock_rng.normal(0, 30))
            noise_q = int(self._mock_rng.normal(0, 30))

            range_i = noise_i
            range_q = noise_q

            if self._moving_target:
                target_r = int(self._target_range_bin)
                if abs(rbin - target_r) <= 1:
                    range_i += 5000 + int(self._mock_rng.normal(0, 200))
                    range_q += 3000 + int(self._mock_rng.normal(0, 200))
            else:
                # Static scene: two fixed targets
                if abs(rbin - 20) <= 1:
                    range_i += 4000 + int(self._mock_rng.normal(0, 200))
                    range_q += 2000 + int(self._mock_rng.normal(0, 200))
                if abs(rbin - 40) <= 1:
                    range_i += 3000 + int(self._mock_rng.normal(0, 150))
                    range_q += 1500 + int(self._mock_rng.normal(0, 150))

            # Build 24-byte v7c packet (6 x 4-byte LE words)
            rword = (((range_q & 0xFFFF) << 16) | (range_i & 0xFFFF)) & 0xFFFFFFFF

            # Word 0: header {24'h0, 0xAA} -> LE bytes: AA 00 00 00
            struct.pack_into("<I", buf, pos, 0x000000AA); pos += 4
            # Word 1: range data (little-endian)
            struct.pack_into("<I", buf, pos, rword); pos += 4
            # Words 2-4: shifted copies (matching RTL behavior)
            struct.pack_into("<I", buf, pos, (rword << 8) & 0xFFFFFFFF); pos += 4
            struct.pack_into("<I", buf, pos, (rword << 16) & 0xFFFFFFFF); pos += 4
            struct.pack_into("<I", buf, pos, (rword << 24) & 0xFFFFFFFF); pos += 4
            # Word 5: footer {24'h0, 0x55} -> LE bytes: 55 00 00 00
            struct.pack_into("<I", buf, pos, 0x00000055); pos += 4

            self._mock_sample_idx += 1
            if self._mock_sample_idx >= NUM_RANGE_BINS:
                self._mock_sample_idx = 0
                self._mock_frame_num += 1

        return bytes(buf[:pos])


# ============================================================================
# Socket Connection — TCP bridge for remote FPGA streaming
# ============================================================================

class SocketConnection:
    """
    TCP socket connection to a remote ft601_stream_server.py instance.

    Same interface as FT601Connection (open/close/read/write/is_open)
    so the GUI and RadarAcquisition can use it interchangeably.

    Protocol matches ft601_stream_server.py:
      Server -> Client: [4 bytes: length (BE uint32)] [data bytes]
      Client -> Server: [4 bytes: length (BE uint32)] [command bytes]
    """

    def __init__(self, host: str = "localhost", port: int = 9000):
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self.is_open = False
        # Receive buffer for reassembling framed messages
        self._recv_buf = bytearray()

    def open(self, device_index: int = 0) -> bool:
        """Connect to remote stream server."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.IPPROTO_TCP,
                                  socket.TCP_NODELAY, 1)
            self._sock.settimeout(5.0)
            self._sock.connect((self._host, self._port))
            self._recv_buf = bytearray()
            self.is_open = True
            log.info(f"Connected to stream server {self._host}:{self._port}")
            return True
        except Exception as e:
            log.error(f"Socket connect failed: {e}")
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
            return False

    def close(self):
        """Disconnect from remote stream server."""
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self.is_open = False
        self._recv_buf = bytearray()

    def read(self, size: int = 16384) -> Optional[bytes]:
        """
        Read one framed message from the server.
        Returns the raw USB data bytes (same as FT601Connection.read).
        """
        if not self.is_open or self._sock is None:
            return None

        try:
            # Read length-prefixed message: [4 bytes len] [data]
            # First, ensure we have the 4-byte header
            while len(self._recv_buf) < 4:
                self._sock.settimeout(0.2)
                try:
                    chunk = self._sock.recv(65536)
                except socket.timeout:
                    return None
                if not chunk:
                    self.is_open = False
                    return None
                self._recv_buf.extend(chunk)

            # Parse length
            msg_len = struct.unpack_from(">I", self._recv_buf, 0)[0]

            if msg_len == 0:
                # Keepalive — consume header, return empty
                self._recv_buf = self._recv_buf[4:]
                return None

            # Read until we have header + full message
            total_needed = 4 + msg_len
            while len(self._recv_buf) < total_needed:
                self._sock.settimeout(0.5)
                try:
                    chunk = self._sock.recv(65536)
                except socket.timeout:
                    return None
                if not chunk:
                    self.is_open = False
                    return None
                self._recv_buf.extend(chunk)

            # Extract message
            data = bytes(self._recv_buf[4:total_needed])
            self._recv_buf = self._recv_buf[total_needed:]
            return data

        except (ConnectionResetError, BrokenPipeError):
            self.is_open = False
            return None
        except Exception as e:
            log.error(f"Socket read error: {e}")
            return None

    def write(self, data: bytes) -> bool:
        """
        Send a command to the remote server (forwarded to FT601).
        Uses length-prefixed framing matching the server protocol.
        """
        if not self.is_open or self._sock is None:
            return False

        with self._lock:
            try:
                hdr = struct.pack(">I", len(data))
                self._sock.sendall(hdr + data)
                return True
            except (BrokenPipeError, ConnectionResetError):
                self.is_open = False
                return False
            except Exception as e:
                log.error(f"Socket write error: {e}")
                return False


# ============================================================================
# Replay Connection — feed real .npy data through the dashboard
# ============================================================================

# Hardware-only opcodes that cannot be adjusted in replay mode
_HARDWARE_ONLY_OPCODES = {
    0x01,  # RADAR_MODE
    0x02,  # TRIGGER (self-clearing pulse)
    0x03,  # THRESHOLD (detect_threshold)
    0x04,  # STREAM_CONTROL
    0x10,  # LONG_CHIRP
    0x11,  # LONG_LISTEN
    0x12,  # GUARD
    0x13,  # SHORT_CHIRP
    0x14,  # SHORT_LISTEN
    0x15,  # CHIRPS_PER_ELEV
    0x16,  # GAIN_SHIFT
    0x20,  # RANGE_MODE
    0x30,  # SELF_TEST_TRIGGER
    0x31,  # SELF_TEST_STATUS
    0xFF,  # STATUS_REQUEST
}

# Replay-adjustable opcodes (re-run signal processing)
_REPLAY_ADJUSTABLE_OPCODES = {
    0x21,  # CFAR_GUARD
    0x22,  # CFAR_TRAIN
    0x23,  # CFAR_ALPHA
    0x24,  # CFAR_MODE
    0x25,  # CFAR_ENABLE
    0x26,  # MTI_ENABLE
    0x27,  # DC_NOTCH_WIDTH
}


def _saturate(val: int, bits: int) -> int:
    """Saturate signed value to fit in 'bits' width."""
    max_pos = (1 << (bits - 1)) - 1
    max_neg = -(1 << (bits - 1))
    return max(max_neg, min(max_pos, int(val)))


def _replay_mti(decim_i: np.ndarray, decim_q: np.ndarray,
                enable: bool) -> Tuple[np.ndarray, np.ndarray]:
    """Bit-accurate 2-pulse MTI canceller (matches mti_canceller.v)."""
    n_chirps, n_bins = decim_i.shape
    mti_i = np.zeros_like(decim_i)
    mti_q = np.zeros_like(decim_q)
    if not enable:
        return decim_i.copy(), decim_q.copy()
    for c in range(n_chirps):
        if c == 0:
            pass  # muted
        else:
            for r in range(n_bins):
                mti_i[c, r] = _saturate(int(decim_i[c, r]) - int(decim_i[c - 1, r]), 16)
                mti_q[c, r] = _saturate(int(decim_q[c, r]) - int(decim_q[c - 1, r]), 16)
    return mti_i, mti_q


def _replay_dc_notch(doppler_i: np.ndarray, doppler_q: np.ndarray,
                     width: int) -> Tuple[np.ndarray, np.ndarray]:
    """Bit-accurate DC notch filter (matches radar_system_top.v inline)."""
    out_i = doppler_i.copy()
    out_q = doppler_q.copy()
    if width == 0:
        return out_i, out_q
    n_doppler = doppler_i.shape[1]
    for dbin in range(n_doppler):
        if dbin < width or dbin > (n_doppler - 1 - width + 1):
            out_i[:, dbin] = 0
            out_q[:, dbin] = 0
    return out_i, out_q


def _replay_cfar(doppler_i: np.ndarray, doppler_q: np.ndarray,
                 guard: int, train: int, alpha_q44: int,
                 mode: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    Bit-accurate CA-CFAR detector (matches cfar_ca.v).
    Returns (detect_flags, magnitudes) both (64, 32).
    """
    ALPHA_FRAC_BITS = 4
    n_range, n_doppler = doppler_i.shape
    if train == 0:
        train = 1

    # Compute magnitudes: |I| + |Q| (17-bit unsigned L1 norm)
    magnitudes = np.zeros((n_range, n_doppler), dtype=np.int64)
    for r in range(n_range):
        for d in range(n_doppler):
            i_val = int(doppler_i[r, d])
            q_val = int(doppler_q[r, d])
            abs_i = (-i_val) & 0xFFFF if i_val < 0 else i_val & 0xFFFF
            abs_q = (-q_val) & 0xFFFF if q_val < 0 else q_val & 0xFFFF
            magnitudes[r, d] = abs_i + abs_q

    detect_flags = np.zeros((n_range, n_doppler), dtype=np.bool_)
    MAX_MAG = (1 << 17) - 1

    mode_names = {0: 'CA', 1: 'GO', 2: 'SO'}
    mode_str = mode_names.get(mode, 'CA')

    for dbin in range(n_doppler):
        col = magnitudes[:, dbin]
        for cut in range(n_range):
            lead_sum, lead_cnt = 0, 0
            for t in range(1, train + 1):
                idx = cut - guard - t
                if 0 <= idx < n_range:
                    lead_sum += int(col[idx])
                    lead_cnt += 1
            lag_sum, lag_cnt = 0, 0
            for t in range(1, train + 1):
                idx = cut + guard + t
                if 0 <= idx < n_range:
                    lag_sum += int(col[idx])
                    lag_cnt += 1

            if mode_str == 'CA':
                noise = lead_sum + lag_sum
            elif mode_str == 'GO':
                if lead_cnt > 0 and lag_cnt > 0:
                    noise = lead_sum if lead_sum * lag_cnt > lag_sum * lead_cnt else lag_sum
                else:
                    noise = lead_sum if lead_cnt > 0 else lag_sum
            elif mode_str == 'SO':
                if lead_cnt > 0 and lag_cnt > 0:
                    noise = lead_sum if lead_sum * lag_cnt < lag_sum * lead_cnt else lag_sum
                else:
                    noise = lead_sum if lead_cnt > 0 else lag_sum
            else:
                noise = lead_sum + lag_sum

            thr = min((alpha_q44 * noise) >> ALPHA_FRAC_BITS, MAX_MAG)
            if int(col[cut]) > thr:
                detect_flags[cut, dbin] = True

    return detect_flags, magnitudes


class ReplayConnection:
    """
    Loads pre-computed .npy arrays (from golden_reference.py co-sim output)
    and serves them as USB data packets to the dashboard, exercising the full
    parsing pipeline with real ADI CN0566 radar data.

    Signal processing parameters (CFAR guard/train/alpha/mode, MTI enable,
    DC notch width) can be adjusted at runtime via write() — the connection
    re-runs the bit-accurate processing pipeline and rebuilds packets.

    Required npy directory layout (e.g. tb/cosim/real_data/hex/):
      decimated_range_i.npy       (32, 64) int   — pre-Doppler range I
      decimated_range_q.npy       (32, 64) int   — pre-Doppler range Q
      doppler_map_i.npy           (64, 32) int   — Doppler I  (no MTI)
      doppler_map_q.npy           (64, 32) int   — Doppler Q  (no MTI)
      fullchain_mti_doppler_i.npy (64, 32) int   — Doppler I  (with MTI)
      fullchain_mti_doppler_q.npy (64, 32) int   — Doppler Q  (with MTI)
      fullchain_cfar_flags.npy    (64, 32) bool  — CFAR detections
      fullchain_cfar_mag.npy      (64, 32) int   — CFAR |I|+|Q| magnitude

    Optional sidecar:
      radar_config.json           — radar parameters for physical axis labels
        Keys: sample_rate, bandwidth, ramp_time, center_freq,
              fft_size, decimation, num_chirps
        If absent, defaults to ADI CN0566 parameters.
    """

    # Default radar config for ADI CN0566 phased-array data
    # range_per_bin: derived from (Fs/N_FFT) * c * T_ramp / (2 * BW) * decimation
    #   = (4e6/1024) * 3e8 * 300e-6 / (2*500e6) * 16 = 5.625 m
    # But peak decimation covers ALL 1024 FFT bins (complex IQ: bins 512-1023
    # are negative freq / aliased). Only first 32 output bins are physical range.
    # For display, we label the full 64-bin axis.
    CN0566_CONFIG = {
        "sample_rate": 4e6,         # Hz — baseband ADC sample rate
        "bandwidth": 500e6,         # Hz — chirp bandwidth
        "ramp_time": 300e-6,        # s  — chirp ramp time
        "center_freq": 9.9e9,       # Hz — carrier frequency
        "fft_size": 1024,           # FFT length (range)
        "decimation": 16,           # peak decimation ratio
        "num_chirps": 32,           # chirps per Doppler frame
        "range_formula": "baseband",  # use baseband range formula
    }

    def __init__(self, npy_dir: str, use_mti: bool = True,
                 replay_fps: float = 5.0):
        self._npy_dir = npy_dir
        self._use_mti = use_mti
        self._replay_fps = max(replay_fps, 0.1)
        self._lock = threading.Lock()
        self.is_open = False
        # Radar config for physical axis labels (set during open)
        self.radar_config: dict = dict(self.CN0566_CONFIG)
        self._packets: bytes = b""
        self._read_offset = 0
        self._frame_len = 0
        # Current signal-processing parameters
        self._mti_enable: bool = use_mti
        self._dc_notch_width: int = 2
        self._cfar_guard: int = 2
        self._cfar_train: int = 8
        self._cfar_alpha: int = 0x30
        self._cfar_mode: int = 0  # 0=CA, 1=GO, 2=SO
        self._cfar_enable: bool = True
        # Raw source arrays (loaded once, reprocessed on param change)
        self._dop_mti_i: Optional[np.ndarray] = None
        self._dop_mti_q: Optional[np.ndarray] = None
        self._dop_nomti_i: Optional[np.ndarray] = None
        self._dop_nomti_q: Optional[np.ndarray] = None
        self._range_i_vec: Optional[np.ndarray] = None
        self._range_q_vec: Optional[np.ndarray] = None
        # Rebuild flag
        self._needs_rebuild = False

    def open(self, device_index: int = 0) -> bool:
        try:
            self._load_arrays()
            self._packets = self._build_packets()
            self._frame_len = len(self._packets)
            self._read_offset = 0
            self.is_open = True
            log.info(f"Replay connection opened: {self._npy_dir} "
                     f"(MTI={'ON' if self._mti_enable else 'OFF'}, "
                     f"{self._frame_len} bytes/frame)")
            return True
        except Exception as e:
            log.error(f"Replay open failed: {e}")
            return False

    def close(self):
        self.is_open = False

    def read(self, size: int = 4096) -> Optional[bytes]:
        if not self.is_open:
            return None
        # Pace reads to target FPS (spread across ~64 reads per frame)
        time.sleep((1.0 / self._replay_fps) / (NUM_CELLS / 32))
        with self._lock:
            # If params changed, rebuild packets
            if self._needs_rebuild:
                self._packets = self._build_packets()
                self._frame_len = len(self._packets)
                self._read_offset = 0
                self._needs_rebuild = False
            end = self._read_offset + size
            if end <= self._frame_len:
                chunk = self._packets[self._read_offset:end]
                self._read_offset = end
            else:
                chunk = self._packets[self._read_offset:]
                self._read_offset = 0
            return chunk

    def write(self, data: bytes) -> bool:
        """
        Handle host commands in replay mode.
        Signal-processing params (CFAR, MTI, DC notch) trigger re-processing.
        Hardware-only params are silently ignored.
        """
        if len(data) < 4:
            return True
        word = struct.unpack(">I", data[:4])[0]
        opcode = (word >> 24) & 0xFF
        value = word & 0xFFFF

        if opcode in _REPLAY_ADJUSTABLE_OPCODES:
            changed = False
            with self._lock:
                if opcode == 0x21:  # CFAR_GUARD
                    if self._cfar_guard != value:
                        self._cfar_guard = value
                        changed = True
                elif opcode == 0x22:  # CFAR_TRAIN
                    if self._cfar_train != value:
                        self._cfar_train = value
                        changed = True
                elif opcode == 0x23:  # CFAR_ALPHA
                    if self._cfar_alpha != value:
                        self._cfar_alpha = value
                        changed = True
                elif opcode == 0x24:  # CFAR_MODE
                    if self._cfar_mode != value:
                        self._cfar_mode = value
                        changed = True
                elif opcode == 0x25:  # CFAR_ENABLE
                    new_en = bool(value)
                    if self._cfar_enable != new_en:
                        self._cfar_enable = new_en
                        changed = True
                elif opcode == 0x26:  # MTI_ENABLE
                    new_en = bool(value)
                    if self._mti_enable != new_en:
                        self._mti_enable = new_en
                        changed = True
                elif opcode == 0x27:  # DC_NOTCH_WIDTH
                    if self._dc_notch_width != value:
                        self._dc_notch_width = value
                        changed = True
                if changed:
                    self._needs_rebuild = True
            if changed:
                log.info(f"Replay param updated: opcode=0x{opcode:02X} "
                         f"value={value} — will re-process")
            else:
                log.debug(f"Replay param unchanged: opcode=0x{opcode:02X} "
                          f"value={value}")
        elif opcode in _HARDWARE_ONLY_OPCODES:
            log.debug(f"Replay: hardware-only opcode 0x{opcode:02X} "
                      f"(ignored in replay mode)")
        else:
            log.debug(f"Replay: unknown opcode 0x{opcode:02X} (ignored)")
        return True

    def _load_arrays(self):
        """Load source npy arrays once, plus optional radar_config.json."""
        npy = self._npy_dir

        # Load radar config sidecar if present
        config_path = os.path.join(npy, "radar_config.json")
        if os.path.isfile(config_path):
            try:
                with open(config_path, "r") as f:
                    user_cfg = json.load(f)
                self.radar_config.update(user_cfg)
                log.info(f"Loaded radar config from {config_path}")
            except Exception as e:
                log.warning(f"Failed to load radar_config.json: {e} "
                            f"(using CN0566 defaults)")
        else:
            log.info("No radar_config.json found, using CN0566 defaults")

        # MTI Doppler
        self._dop_mti_i = np.load(
            os.path.join(npy, "fullchain_mti_doppler_i.npy")).astype(np.int64)
        self._dop_mti_q = np.load(
            os.path.join(npy, "fullchain_mti_doppler_q.npy")).astype(np.int64)
        # Non-MTI Doppler
        self._dop_nomti_i = np.load(
            os.path.join(npy, "doppler_map_i.npy")).astype(np.int64)
        self._dop_nomti_q = np.load(
            os.path.join(npy, "doppler_map_q.npy")).astype(np.int64)
        # Range data
        try:
            range_i_all = np.load(
                os.path.join(npy, "decimated_range_i.npy")).astype(np.int64)
            range_q_all = np.load(
                os.path.join(npy, "decimated_range_q.npy")).astype(np.int64)
            self._range_i_vec = range_i_all[-1, :]  # last chirp
            self._range_q_vec = range_q_all[-1, :]
        except FileNotFoundError:
            self._range_i_vec = np.zeros(NUM_RANGE_BINS, dtype=np.int64)
            self._range_q_vec = np.zeros(NUM_RANGE_BINS, dtype=np.int64)

    def _build_packets(self) -> bytes:
        """Build a full frame of USB data packets from current params."""
        # Select Doppler data based on MTI
        if self._mti_enable:
            dop_i = self._dop_mti_i
            dop_q = self._dop_mti_q
        else:
            dop_i = self._dop_nomti_i
            dop_q = self._dop_nomti_q

        # Apply DC notch
        dop_i, dop_q = _replay_dc_notch(dop_i, dop_q, self._dc_notch_width)

        # Run CFAR
        if self._cfar_enable:
            det, _mag = _replay_cfar(
                dop_i, dop_q,
                guard=self._cfar_guard,
                train=self._cfar_train,
                alpha_q44=self._cfar_alpha,
                mode=self._cfar_mode,
            )
        else:
            det = np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=bool)

        det_count = int(det.sum())
        log.info(f"Replay: rebuilt {NUM_CELLS} packets "
                 f"(MTI={'ON' if self._mti_enable else 'OFF'}, "
                 f"DC_notch={self._dc_notch_width}, "
                 f"CFAR={'ON' if self._cfar_enable else 'OFF'} "
                 f"G={self._cfar_guard} T={self._cfar_train} "
                 f"a=0x{self._cfar_alpha:02X} m={self._cfar_mode}, "
                 f"{det_count} detections)")

        range_i = self._range_i_vec
        range_q = self._range_q_vec

        # Pre-allocate buffer (35 bytes per packet * 2048 cells)
        buf = bytearray(NUM_CELLS * 35)
        pos = 0
        for rbin in range(NUM_RANGE_BINS):
            ri = int(np.clip(range_i[rbin], -32768, 32767)) & 0xFFFF
            rq = int(np.clip(range_q[rbin], -32768, 32767)) & 0xFFFF
            rword = ((rq << 16) | ri) & 0xFFFFFFFF
            rw0 = struct.pack(">I", rword)
            rw1 = struct.pack(">I", (rword << 8) & 0xFFFFFFFF)
            rw2 = struct.pack(">I", (rword << 16) & 0xFFFFFFFF)
            rw3 = struct.pack(">I", (rword << 24) & 0xFFFFFFFF)
            for dbin in range(NUM_DOPPLER_BINS):
                di = int(np.clip(dop_i[rbin, dbin], -32768, 32767)) & 0xFFFF
                dq = int(np.clip(dop_q[rbin, dbin], -32768, 32767)) & 0xFFFF
                d = 1 if det[rbin, dbin] else 0

                dword = ((di << 16) | dq) & 0xFFFFFFFF

                buf[pos] = HEADER_BYTE
                pos += 1
                buf[pos:pos+4] = rw0; pos += 4
                buf[pos:pos+4] = rw1; pos += 4
                buf[pos:pos+4] = rw2; pos += 4
                buf[pos:pos+4] = rw3; pos += 4
                buf[pos:pos+4] = struct.pack(">I", dword); pos += 4
                buf[pos:pos+4] = struct.pack(">I", (dword << 8) & 0xFFFFFFFF); pos += 4
                buf[pos:pos+4] = struct.pack(">I", (dword << 16) & 0xFFFFFFFF); pos += 4
                buf[pos:pos+4] = struct.pack(">I", (dword << 24) & 0xFFFFFFFF); pos += 4
                buf[pos] = d; pos += 1
                buf[pos] = FOOTER_BYTE; pos += 1

        return bytes(buf)


# ============================================================================
# Data Recorder (HDF5)
# ============================================================================

try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False


class DataRecorder:
    """Record radar frames to HDF5 files for offline analysis."""

    def __init__(self):
        self._file = None
        self._grp = None
        self._frame_count = 0
        self._recording = False

    @property
    def recording(self) -> bool:
        return self._recording

    def start(self, filepath: str):
        if not HDF5_AVAILABLE:
            log.error("h5py not installed — HDF5 recording unavailable")
            return
        try:
            self._file = h5py.File(filepath, "w")
            self._file.attrs["creator"] = "AERIS-10 Radar Dashboard"
            self._file.attrs["start_time"] = time.time()
            self._file.attrs["range_bins"] = NUM_RANGE_BINS
            self._file.attrs["doppler_bins"] = NUM_DOPPLER_BINS

            self._grp = self._file.create_group("frames")
            self._frame_count = 0
            self._recording = True
            log.info(f"Recording started: {filepath}")
        except Exception as e:
            log.error(f"Failed to start recording: {e}")

    def record_frame(self, frame: RadarFrame):
        if not self._recording or self._file is None:
            return
        try:
            fg = self._grp.create_group(f"frame_{self._frame_count:06d}")
            fg.attrs["timestamp"] = frame.timestamp
            fg.attrs["frame_number"] = frame.frame_number
            fg.attrs["detection_count"] = frame.detection_count
            fg.create_dataset("magnitude", data=frame.magnitude, compression="gzip")
            fg.create_dataset("range_doppler_i", data=frame.range_doppler_i, compression="gzip")
            fg.create_dataset("range_doppler_q", data=frame.range_doppler_q, compression="gzip")
            fg.create_dataset("detections", data=frame.detections, compression="gzip")
            fg.create_dataset("range_profile", data=frame.range_profile, compression="gzip")
            self._frame_count += 1
        except Exception as e:
            log.error(f"Recording error: {e}")

    def stop(self):
        if self._file is not None:
            try:
                self._file.attrs["end_time"] = time.time()
                self._file.attrs["total_frames"] = self._frame_count
                self._file.close()
            except Exception:
                pass
            self._file = None
        self._recording = False
        log.info(f"Recording stopped ({self._frame_count} frames)")


# ============================================================================
# Radar Data Acquisition Thread
# ============================================================================

class RadarAcquisition(threading.Thread):
    """
    Background thread: reads from FT601, parses packets, assembles frames,
    and pushes complete frames to the display queue.

    Supports three modes:
    - range_only mode (stream_control=0x01): Each 0xAA packet = one range bin.
      Collects NUM_RANGE_BINS samples to form a frame (no Doppler from FPGA).
    - v9 full pipeline mode (stream_control=0x03 or 0x07): Independent
      0xAA range, 0xCC Doppler, and 0xDD CFAR packets. Doppler and CFAR
      packets carry range_bin/doppler_bin coordinates and are placed directly.
      Frame is finalized when all expected Doppler packets (2048) arrive,
      or on a timeout after the last range packet.
    - Mock/replay mode (full): Legacy sequential cell mode.
    """

    def __init__(self, connection, frame_queue: queue.Queue,
                 recorder: Optional[DataRecorder] = None,
                 status_callback=None,
                 range_only: bool = False):
        super().__init__(daemon=True)
        self.conn = connection
        self.frame_queue = frame_queue
        self.recorder = recorder
        self._status_callback = status_callback
        self._stop_event = threading.Event()
        self._frame = RadarFrame()
        self._sample_idx = 0
        self._frame_num = 0
        self._range_only = range_only
        # Buffer for incomplete data across reads
        self._residual = b""
        # v9 full pipeline counters (for frame assembly from independent streams)
        self._range_count = 0      # 0xAA packets received in current frame
        self._doppler_count = 0    # 0xCC packets received in current frame
        self._cfar_count = 0       # 0xDD packets received in current frame
        # When range_only=False (--v9 flag), assume v9 mode from the start.
        # This prevents the first USB batch (which may only contain 0xAA range
        # packets before any 0xCC Doppler arrive) from being misrouted to the
        # legacy _ingest_sample() handler, which would produce a bogus frame 0.
        self._got_doppler = not range_only  # True in v9 mode from the start
        self._got_cfar = False      # True once any 0xDD packet arrives
        # v9 deferred finalization: CFAR packets arrive after Doppler due to
        # FPGA priority arbiter (Range P1 > Doppler P2 > CFAR P3). We delay
        # frame finalization after all Doppler packets to collect CFAR.
        self._doppler_complete = False  # True when doppler_count >= NUM_CELLS
        self._doppler_complete_ts = 0.0  # time.time() when Doppler finished
        self._V9_CFAR_TIMEOUT = 0.2     # seconds to wait for CFAR after Doppler
        # Chirp tracking for range-only profile (last chirp's data)
        self._chirp_range_profile = np.zeros(NUM_RANGE_BINS, dtype=np.float64)
        self._range_bin_idx = 0    # sequential range bin within chirp

    def stop(self):
        self._stop_event.set()

    @property
    def _v9_mode(self) -> bool:
        """True if we're receiving v9 Doppler/CFAR packets (not mock/replay)."""
        return self._got_doppler or self._got_cfar

    def run(self):
        log.info(f"Acquisition thread started "
                 f"(range_only={self._range_only})")
        while not self._stop_event.is_set():
            raw = self.conn.read(16384)
            if raw is None or len(raw) == 0:
                # No data — check if v9 frame needs deferred finalization
                if self._doppler_complete:
                    elapsed = time.time() - self._doppler_complete_ts
                    if elapsed >= self._V9_CFAR_TIMEOUT:
                        self._finalize_v9_frame()
                time.sleep(0.01)
                continue

            # Prepend any leftover bytes from last read
            if self._residual:
                raw = self._residual + raw
                self._residual = b""

            packets = RadarProtocol.find_packet_boundaries(raw)

            # Save any trailing bytes that didn't form a complete packet
            if packets:
                last_end = packets[-1][1]
                if last_end < len(raw):
                    self._residual = raw[last_end:]
            else:
                # No packets found — check if buffer starts mid-packet
                # Keep last 39 bytes (max partial packet) for next read
                keep = min(len(raw), RadarProtocol.V7C_STATUS_PACKET_SIZE - 1)
                self._residual = raw[-keep:]

            cfar_in_batch = 0

            # Pre-scan: detect v9 mode from packet types in this batch.
            # This ensures _v9_mode is True before routing 0xAA range packets,
            # even if Doppler/CFAR packets appear later in the same USB read.
            if not self._v9_mode:
                for _, _, ptype in packets:
                    if ptype == "doppler":
                        self._got_doppler = True
                        break
                    elif ptype == "cfar":
                        self._got_cfar = True
                        break

            for start, end, ptype in packets:
                if ptype == "data":
                    parsed = RadarProtocol.parse_data_packet(raw[start:end])
                    if parsed is not None:
                        if self._range_only:
                            self._ingest_range_sample(parsed)
                        elif self._v9_mode:
                            self._ingest_v9_range(parsed)
                        else:
                            self._ingest_sample(parsed)
                elif ptype == "doppler":
                    parsed = RadarProtocol.parse_doppler_packet(raw[start:end])
                    if parsed is not None:
                        self._got_doppler = True
                        self._ingest_v9_doppler(parsed)
                elif ptype == "cfar":
                    parsed = RadarProtocol.parse_cfar_packet(raw[start:end])
                    if parsed is not None:
                        self._got_cfar = True
                        self._ingest_v9_cfar(parsed)
                        cfar_in_batch += 1
                elif ptype == "status":
                    status = RadarProtocol.parse_status_packet(raw[start:end])
                    if status is not None:
                        log.info(f"Status: mode={status.radar_mode} "
                                 f"stream={status.stream_ctrl} "
                                 f"pkt_starts={status.dbg_pkt_starts} "
                                 f"pkt_completions={status.dbg_pkt_completions}")
                        if status.self_test_busy or status.self_test_flags:
                            log.info(f"Self-test: busy={status.self_test_busy} "
                                     f"flags=0b{status.self_test_flags:05b} "
                                     f"detail=0x{status.self_test_detail:02X}")
                        if self._status_callback is not None:
                            try:
                                self._status_callback(status)
                            except Exception as e:
                                log.error(f"Status callback error: {e}")

            # v9 deferred finalization: after processing a batch, check if
            # Doppler is complete and no more CFAR packets arrived in this read.
            # This catches the common case where all CFAR packets arrive in
            # the same or next USB read after the last Doppler packet.
            if self._doppler_complete and cfar_in_batch == 0:
                self._finalize_v9_frame()

        log.info("Acquisition thread stopped")

    # -- Range-only mode (v7c/v8c, stream_control=0x01) --

    def _ingest_range_sample(self, sample: Dict):
        """
        Place a range-only sample into current frame.
        In range-only mode, each packet is one range bin.
        We accumulate NUM_RANGE_BINS samples to build one frame.
        The range profile is updated directly; the range-Doppler map
        gets the range value placed in Doppler bin 0.
        """
        rbin = self._sample_idx

        if rbin < NUM_RANGE_BINS:
            ri = sample["range_i"]
            rq = sample["range_q"]
            mag = abs(ri) + abs(rq)

            # Range profile directly from magnitude
            self._frame.range_profile[rbin] = mag

            # Also populate range-Doppler bin 0 for heatmap display
            self._frame.range_doppler_i[rbin, 0] = ri
            self._frame.range_doppler_q[rbin, 0] = rq
            self._frame.magnitude[rbin, 0] = mag

        self._sample_idx += 1

        if self._sample_idx >= NUM_RANGE_BINS:
            self._finalize_frame()

    # -- Mock/replay sequential mode (legacy) --

    def _ingest_sample(self, sample: Dict):
        """Place sample into current frame and emit when complete (mock/replay mode)."""
        rbin = self._sample_idx // NUM_DOPPLER_BINS
        dbin = self._sample_idx % NUM_DOPPLER_BINS

        if rbin < NUM_RANGE_BINS and dbin < NUM_DOPPLER_BINS:
            di = sample.get("doppler_i", 0)
            dq = sample.get("doppler_q", 0)
            self._frame.range_doppler_i[rbin, dbin] = di
            self._frame.range_doppler_q[rbin, dbin] = dq
            mag = abs(int(di)) + abs(int(dq))
            self._frame.magnitude[rbin, dbin] = mag
            if sample.get("detection", 0):
                self._frame.detections[rbin, dbin] = 1
                self._frame.detection_count += 1

        self._sample_idx += 1

        if self._sample_idx >= NUM_CELLS:
            self._finalize_frame()

    # -- v9 full pipeline mode (independent 0xAA/0xCC/0xDD packets) --

    def _ingest_v9_range(self, sample: Dict):
        """
        Ingest a v9 range packet (0xAA) in full pipeline mode.

        Range packets arrive as 32 chirps × 64 bins = 2048 packets.
        We track the last chirp's range profile for display, updating
        the range profile on each bin. The range-Doppler heatmap is
        populated from 0xCC Doppler packets instead.
        """
        rbin = self._range_bin_idx

        if rbin < NUM_RANGE_BINS:
            ri = sample["range_i"]
            rq = sample["range_q"]
            mag = abs(ri) + abs(rq)
            self._chirp_range_profile[rbin] = mag

        self._range_bin_idx += 1
        self._range_count += 1

        if self._range_bin_idx >= NUM_RANGE_BINS:
            self._range_bin_idx = 0  # Reset for next chirp
            # Copy current chirp's range profile (last chirp wins for display)
            self._frame.range_profile[:] = self._chirp_range_profile

    def _ingest_v9_doppler(self, sample: Dict):
        """
        Ingest a v9 Doppler packet (0xCC) in full pipeline mode.

        Each packet carries its own range_bin/doppler_bin coordinates,
        so we place it directly into the range-Doppler map.
        When all NUM_CELLS (2048) Doppler packets arrive, we set
        _doppler_complete to defer frame finalization — this allows
        CFAR packets (lower priority in FPGA arbiter) to arrive before
        the frame is sealed.
        """
        rbin = sample["range_bin"]
        dbin = sample["doppler_bin"]

        if 0 <= rbin < NUM_RANGE_BINS and 0 <= dbin < NUM_DOPPLER_BINS:
            di = sample["doppler_i"]
            dq = sample["doppler_q"]
            self._frame.range_doppler_i[rbin, dbin] = di
            self._frame.range_doppler_q[rbin, dbin] = dq
            mag = abs(int(di)) + abs(int(dq))
            self._frame.magnitude[rbin, dbin] = mag

        self._doppler_count += 1

        if self._doppler_count >= NUM_CELLS and not self._doppler_complete:
            # Mark Doppler as complete — defer finalization to collect CFAR
            self._doppler_complete = True
            self._doppler_complete_ts = time.time()

    def _ingest_v9_cfar(self, sample: Dict):
        """
        Ingest a v9 CFAR detection packet (0xDD) in full pipeline mode.

        Each packet carries range_bin/doppler_bin coordinates and the
        detection flag (always 1 since only detections are sent).
        """
        rbin = sample["range_bin"]
        dbin = sample["doppler_bin"]

        if 0 <= rbin < NUM_RANGE_BINS and 0 <= dbin < NUM_DOPPLER_BINS:
            if sample["detect_flag"]:
                self._frame.detections[rbin, dbin] = 1
                self._frame.detection_count += 1

        self._cfar_count += 1

    def _finalize_v9_frame(self):
        """
        Finalize a v9 frame after all Doppler packets received AND CFAR
        packets have been collected (deferred finalization).
        The range profile comes from accumulated 0xAA packets (last chirp),
        or from the magnitude map if range packets weren't streamed.
        """
        self._doppler_complete = False
        self._doppler_complete_ts = 0.0

        self._frame.timestamp = time.time()
        self._frame.frame_number = self._frame_num

        # If no range packets were received (doppler-only mode),
        # derive range profile from the Doppler magnitude map
        if self._range_count == 0:
            self._frame.range_profile = np.sum(self._frame.magnitude, axis=1)

        log.info(f"v9 frame {self._frame_num}: "
                 f"range={self._range_count} doppler={self._doppler_count} "
                 f"cfar={self._cfar_count} detections={self._frame.detection_count}")

        # Push to display queue (drop old if backed up)
        try:
            self.frame_queue.put_nowait(self._frame)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            self.frame_queue.put_nowait(self._frame)

        if self.recorder and self.recorder.recording:
            self.recorder.record_frame(self._frame)

        self._frame_num += 1
        self._frame = RadarFrame()
        self._range_count = 0
        self._doppler_count = 0
        self._cfar_count = 0
        self._range_bin_idx = 0
        self._chirp_range_profile[:] = 0

    def _finalize_frame(self):
        """Complete frame: compute range profile, push to queue, record."""
        self._frame.timestamp = time.time()
        self._frame.frame_number = self._frame_num

        if not self._range_only:
            # Range profile = sum of magnitude across Doppler bins
            self._frame.range_profile = np.sum(self._frame.magnitude, axis=1)

        # Push to display queue (drop old if backed up)
        try:
            self.frame_queue.put_nowait(self._frame)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            self.frame_queue.put_nowait(self._frame)

        if self.recorder and self.recorder.recording:
            self.recorder.record_frame(self._frame)

        self._frame_num += 1
        self._frame = RadarFrame()
        self._sample_idx = 0
