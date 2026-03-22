#!/usr/bin/env python3
"""
AERIS-10 Radar Protocol Layer
===============================
Pure-logic module for FT601 packet parsing and command building.
No GUI dependencies — safe to import from tests and headless scripts.

Matches usb_data_interface.v packet format exactly.

USB Packet Protocol:
  TX (FPGA→Host):
    Data packet:  [0xAA] [range 4×32b] [doppler 4×32b] [det 1B] [0x55]
    Status packet: [0xBB] [status 6×32b] [0x55]
  RX (Host→FPGA):
    Command word:  {opcode[31:24], addr[23:16], value[15:0]}
"""

import os
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
FOOTER_BYTE = 0x55
STATUS_HEADER_BYTE = 0xBB

NUM_RANGE_BINS = 64
NUM_DOPPLER_BINS = 32
NUM_CELLS = NUM_RANGE_BINS * NUM_DOPPLER_BINS  # 2048

WATERFALL_DEPTH = 64


class Opcode(IntEnum):
    """Host register opcodes (matches radar_system_top.v command decode)."""
    TRIGGER             = 0x01
    PRF_DIV             = 0x02
    NUM_CHIRPS          = 0x03
    CHIRP_TIMER         = 0x04
    STREAM_ENABLE       = 0x05
    GAIN_SHIFT          = 0x06
    THRESHOLD           = 0x10
    LONG_CHIRP          = 0x10
    LONG_LISTEN         = 0x11
    GUARD               = 0x12
    SHORT_CHIRP         = 0x13
    SHORT_LISTEN        = 0x14
    CHIRPS_PER_ELEV     = 0x15
    RANGE_MODE          = 0x20
    CFAR_GUARD          = 0x21
    CFAR_TRAIN          = 0x22
    CFAR_ALPHA          = 0x23
    CFAR_MODE           = 0x24
    CFAR_ENABLE         = 0x25
    MTI_ENABLE          = 0x26
    DC_NOTCH_WIDTH      = 0x27
    SELF_TEST_TRIGGER   = 0x30
    SELF_TEST_STATUS    = 0x31
    STATUS_REQUEST      = 0xFF


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
    """Parsed status response from FPGA (8-word packet as of Build 26)."""
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
    # Self-test results (word 5, added in Build 26)
    self_test_flags: int = 0     # 5-bit result flags [4:0]
    self_test_detail: int = 0    # 8-bit detail code [7:0]
    self_test_busy: int = 0      # 1-bit busy flag


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
        Parse a single data packet from the FPGA byte stream.
        Returns dict with keys: 'range_i', 'range_q', 'doppler_i', 'doppler_q',
        'detection', or None if invalid.

        Packet format (all streams enabled):
          [0xAA] [range 4×4B] [doppler 4×4B] [det 1B] [0x55]
          = 1 + 16 + 16 + 1 + 1 = 35 bytes

        With byte-enables, the FT601 delivers only valid bytes.
        Header/footer/detection use BE=0001 → 1 byte each.
        Range/doppler use BE=1111 → 4 bytes each × 4 transfers.

        In practice, the range data word 0 contains the full 32-bit value
        {range_q[15:0], range_i[15:0]}. Words 1–3 are shifted copies.
        Similarly, doppler word 0 = {doppler_real, doppler_imag}.
        """
        if len(raw) < 3:
            return None
        if raw[0] != HEADER_BYTE:
            return None

        result = {}
        pos = 1

        # Range data: 4 × 4 bytes, only word 0 matters
        if pos + 16 <= len(raw):
            range_word0 = struct.unpack_from(">I", raw, pos)[0]
            result["range_i"] = _to_signed16(range_word0 & 0xFFFF)
            result["range_q"] = _to_signed16((range_word0 >> 16) & 0xFFFF)
            pos += 16
        else:
            return None

        # Doppler data: 4 × 4 bytes, only word 0 matters
        # Word 0 layout: {doppler_real[31:16], doppler_imag[15:0]}
        if pos + 16 <= len(raw):
            dop_word0 = struct.unpack_from(">I", raw, pos)[0]
            result["doppler_q"] = _to_signed16(dop_word0 & 0xFFFF)
            result["doppler_i"] = _to_signed16((dop_word0 >> 16) & 0xFFFF)
            pos += 16
        else:
            return None

        # Detection: 1 byte
        if pos + 1 <= len(raw):
            result["detection"] = raw[pos] & 0x01
            pos += 1
        else:
            return None

        # Footer
        if pos < len(raw) and raw[pos] == FOOTER_BYTE:
            pos += 1

        return result

    @staticmethod
    def parse_status_packet(raw: bytes) -> Optional[StatusResponse]:
        """
        Parse a status response packet.
        Format: [0xBB] [6×4B status words] [0x55] = 1 + 24 + 1 = 26 bytes
        """
        if len(raw) < 26:
            return None
        if raw[0] != STATUS_HEADER_BYTE:
            return None

        words = []
        for i in range(6):
            w = struct.unpack_from(">I", raw, 1 + i * 4)[0]
            words.append(w)

        if raw[25] != FOOTER_BYTE:
            return None

        sr = StatusResponse()
        # Word 0: {0xFF, 3'b0, mode[1:0], 5'b0, stream[2:0], threshold[15:0]}
        sr.cfar_threshold = words[0] & 0xFFFF
        sr.stream_ctrl = (words[0] >> 16) & 0x07
        sr.radar_mode = (words[0] >> 21) & 0x03
        # Word 1: {long_chirp[31:16], long_listen[15:0]}
        sr.long_listen = words[1] & 0xFFFF
        sr.long_chirp = (words[1] >> 16) & 0xFFFF
        # Word 2: {guard[31:16], short_chirp[15:0]}
        sr.short_chirp = words[2] & 0xFFFF
        sr.guard = (words[2] >> 16) & 0xFFFF
        # Word 3: {short_listen[31:16], 10'd0, chirps_per_elev[5:0]}
        sr.chirps_per_elev = words[3] & 0x3F
        sr.short_listen = (words[3] >> 16) & 0xFFFF
        # Word 4: {30'd0, range_mode[1:0]}
        sr.range_mode = words[4] & 0x03
        # Word 5: {7'd0, self_test_busy, 8'd0, self_test_detail[7:0],
        #           3'd0, self_test_flags[4:0]}
        sr.self_test_flags = words[5] & 0x1F
        sr.self_test_detail = (words[5] >> 8) & 0xFF
        sr.self_test_busy = (words[5] >> 24) & 0x01
        return sr

    @staticmethod
    def find_packet_boundaries(buf: bytes) -> List[Tuple[int, int, str]]:
        """
        Scan buffer for packet start markers (0xAA data, 0xBB status).
        Returns list of (start_idx, expected_end_idx, packet_type).
        """
        packets = []
        i = 0
        while i < len(buf):
            if buf[i] == HEADER_BYTE:
                # Data packet: 35 bytes (all streams)
                end = i + 35
                if end <= len(buf):
                    packets.append((i, end, "data"))
                    i = end
                else:
                    break
            elif buf[i] == STATUS_HEADER_BYTE:
                # Status packet: 26 bytes (6 words + header + footer)
                end = i + 26
                if end <= len(buf):
                    packets.append((i, end, "status"))
                    i = end
                else:
                    break
            else:
                i += 1
        return packets


# ============================================================================
# FT601 USB Connection
# ============================================================================

# Optional ftd3xx import
try:
    import ftd3xx
    FTD3XX_AVAILABLE = True
except ImportError:
    FTD3XX_AVAILABLE = False


class FT601Connection:
    """
    FT601 USB 3.0 FIFO bridge communication.
    Supports ftd3xx (native D3XX) or mock mode.
    """

    def __init__(self, mock: bool = True):
        self._mock = mock
        self._device = None
        self._lock = threading.Lock()
        self.is_open = False
        # Mock state
        self._mock_frame_num = 0
        self._mock_rng = np.random.RandomState(42)

    def open(self, device_index: int = 0) -> bool:
        if self._mock:
            self.is_open = True
            log.info("FT601 mock device opened (no hardware)")
            return True

        if not FTD3XX_AVAILABLE:
            log.error("ftd3xx not installed — cannot open real FT601 device")
            return False

        try:
            self._device = ftd3xx.create(device_index, ftd3xx.CONFIGURATION_CHANNEL_0)
            if self._device is None:
                log.error("ftd3xx.create returned None")
                return False
            self.is_open = True
            log.info(f"FT601 device {device_index} opened")
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
        self.is_open = False

    def read(self, size: int = 4096) -> Optional[bytes]:
        """Read raw bytes from FT601. Returns None on error/timeout."""
        if not self.is_open:
            return None

        if self._mock:
            return self._mock_read(size)

        with self._lock:
            try:
                buf = self._device.readPipe(0x82, size, raw=True)
                return bytes(buf) if buf else None
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
                self._device.writePipe(0x02, data, len(data))
                return True
            except Exception as e:
                log.error(f"FT601 write error: {e}")
                return False

    def _mock_read(self, size: int) -> bytes:
        """
        Generate synthetic radar data packets for testing.
        Simulates a batch of packets with a target near range bin 20, Doppler bin 8.
        """
        time.sleep(0.01)  # Simulate USB latency
        self._mock_frame_num += 1

        buf = bytearray()
        num_packets = min(256, size // 35)
        for _ in range(num_packets):
            rbin = self._mock_rng.randint(0, NUM_RANGE_BINS)
            dbin = self._mock_rng.randint(0, NUM_DOPPLER_BINS)

            # Simulate range profile with a target at bin ~20 and noise
            range_i = int(self._mock_rng.normal(0, 100))
            range_q = int(self._mock_rng.normal(0, 100))
            if abs(rbin - 20) < 3:
                range_i += 5000
                range_q += 3000

            # Simulate Doppler with target at Doppler bin ~8
            dop_i = int(self._mock_rng.normal(0, 50))
            dop_q = int(self._mock_rng.normal(0, 50))
            if abs(rbin - 20) < 3 and abs(dbin - 8) < 2:
                dop_i += 8000
                dop_q += 4000

            detection = 1 if (abs(rbin - 20) < 2 and abs(dbin - 8) < 2) else 0

            # Build packet
            pkt = bytearray()
            pkt.append(HEADER_BYTE)

            rword = (((range_q & 0xFFFF) << 16) | (range_i & 0xFFFF)) & 0xFFFFFFFF
            pkt += struct.pack(">I", rword)
            pkt += struct.pack(">I", ((rword << 8) & 0xFFFFFFFF))
            pkt += struct.pack(">I", ((rword << 16) & 0xFFFFFFFF))
            pkt += struct.pack(">I", ((rword << 24) & 0xFFFFFFFF))

            dword = (((dop_i & 0xFFFF) << 16) | (dop_q & 0xFFFF)) & 0xFFFFFFFF
            pkt += struct.pack(">I", dword)
            pkt += struct.pack(">I", ((dword << 8) & 0xFFFFFFFF))
            pkt += struct.pack(">I", ((dword << 16) & 0xFFFFFFFF))
            pkt += struct.pack(">I", ((dword << 24) & 0xFFFFFFFF))

            pkt.append(detection & 0x01)
            pkt.append(FOOTER_BYTE)

            buf += pkt

        return bytes(buf)


# ============================================================================
# Replay Connection — feed real .npy data through the dashboard
# ============================================================================

# Hardware-only opcodes that cannot be adjusted in replay mode
_HARDWARE_ONLY_OPCODES = {
    0x01,  # TRIGGER
    0x02,  # PRF_DIV
    0x03,  # NUM_CHIRPS
    0x04,  # CHIRP_TIMER
    0x05,  # STREAM_ENABLE
    0x06,  # GAIN_SHIFT
    0x10,  # THRESHOLD / LONG_CHIRP
    0x11,  # LONG_LISTEN
    0x12,  # GUARD
    0x13,  # SHORT_CHIRP
    0x14,  # SHORT_LISTEN
    0x15,  # CHIRPS_PER_ELEV
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
    """

    def __init__(self, npy_dir: str, use_mti: bool = True,
                 replay_fps: float = 5.0):
        self._npy_dir = npy_dir
        self._use_mti = use_mti
        self._replay_fps = max(replay_fps, 0.1)
        self._lock = threading.Lock()
        self.is_open = False
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
        """Load source npy arrays once."""
        npy = self._npy_dir
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
    """

    def __init__(self, connection: FT601Connection, frame_queue: queue.Queue,
                 recorder: Optional[DataRecorder] = None,
                 status_callback=None):
        super().__init__(daemon=True)
        self.conn = connection
        self.frame_queue = frame_queue
        self.recorder = recorder
        self._status_callback = status_callback
        self._stop_event = threading.Event()
        self._frame = RadarFrame()
        self._sample_idx = 0
        self._frame_num = 0

    def stop(self):
        self._stop_event.set()

    def run(self):
        log.info("Acquisition thread started")
        while not self._stop_event.is_set():
            raw = self.conn.read(4096)
            if raw is None or len(raw) == 0:
                time.sleep(0.01)
                continue

            packets = RadarProtocol.find_packet_boundaries(raw)
            for start, end, ptype in packets:
                if ptype == "data":
                    parsed = RadarProtocol.parse_data_packet(raw[start:end])
                    if parsed is not None:
                        self._ingest_sample(parsed)
                elif ptype == "status":
                    status = RadarProtocol.parse_status_packet(raw[start:end])
                    if status is not None:
                        log.info(f"Status: mode={status.radar_mode} "
                                 f"stream={status.stream_ctrl}")
                        if status.self_test_busy or status.self_test_flags:
                            log.info(f"Self-test: busy={status.self_test_busy} "
                                     f"flags=0b{status.self_test_flags:05b} "
                                     f"detail=0x{status.self_test_detail:02X}")
                        if self._status_callback is not None:
                            try:
                                self._status_callback(status)
                            except Exception as e:
                                log.error(f"Status callback error: {e}")

        log.info("Acquisition thread stopped")

    def _ingest_sample(self, sample: Dict):
        """Place sample into current frame and emit when complete."""
        rbin = self._sample_idx // NUM_DOPPLER_BINS
        dbin = self._sample_idx % NUM_DOPPLER_BINS

        if rbin < NUM_RANGE_BINS and dbin < NUM_DOPPLER_BINS:
            self._frame.range_doppler_i[rbin, dbin] = sample["doppler_i"]
            self._frame.range_doppler_q[rbin, dbin] = sample["doppler_q"]
            mag = abs(int(sample["doppler_i"])) + abs(int(sample["doppler_q"]))
            self._frame.magnitude[rbin, dbin] = mag
            if sample.get("detection", 0):
                self._frame.detections[rbin, dbin] = 1
                self._frame.detection_count += 1

        self._sample_idx += 1

        if self._sample_idx >= NUM_CELLS:
            self._finalize_frame()

    def _finalize_frame(self):
        """Complete frame: compute range profile, push to queue, record."""
        self._frame.timestamp = time.time()
        self._frame.frame_number = self._frame_num
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
