#!/usr/bin/env python3
"""
AERIS-10 Radar Protocol Layer
===============================
Pure-logic module for USB packet parsing and command building.
No GUI dependencies — safe to import from tests and headless scripts.

USB transports + wire formats (these intentionally diverge):

  FT2232H USB 2.0 (50T production board, USB_MODE=1, default)
      Bulk per-frame format from `usb_data_interface_ft2232h.v`. One header
      + variable-length sections + footer per Doppler frame. The bulk format
      exists because USB 2.0's ~8 MB/s sustained ceiling cannot carry the
      production frame rate (~178 fps x 35849 B = 6.4 MB/s) at per-sample
      granularity. Wire layout:
          [0xAA][flags 1B][frame_num 2B][n_range 2B][n_doppler 2B]
          [range_profile 1024 B  if flags.stream_range]
          [doppler_mag   32768 B if flags.stream_doppler]   # mag_only=1 only
          [cfar_dense    2048 B  if flags.stream_cfar]      # sparse_det=0 only
          [0x55]
      Production FPGA today only emits mag_only=1 + dense-bitmap CFAR; the
      flag bits for full-I/Q (mag_only=0) and sparse-detection-list
      (sparse_det=1) are reserved for a future RTL extension and currently
      force-clamped to 1 and 0 respectively in `radar_system_top.v` opcode
      0x04 handler when USB_MODE=1.

  FT601 USB 3.0 (200T premium board, USB_MODE=0)
      Per-sample 11-byte legacy format from `usb_data_interface.v`. USB 3.0
      has ~50x the bandwidth headroom (~360 MB/s practical), so the lighter
      per-sample format is fine and offers easier resync after byte drops.
      Wire layout (per sample, 16384 samples per frame):
          [0xAA][range_q 2B][range_i 2B][dop_re 2B][dop_im 2B][det 1B][0x55]
          where det byte = {frame_start, 6'b0, cfar_detection}.
      Status (both transports): [0xBB][6x32b status words][0x55] = 26 B.

  RX (Host → FPGA, both transports)
      4 bytes per command: {opcode[7:0], addr[7:0], value[15:8], value[7:0]}.

The GUI parser handles both formats; `RadarAcquisition` dispatches on
connection type (FT2232HConnection → bulk; FT601Connection → legacy).
"""

import struct
import time
import threading
import queue
import logging
import contextlib
from dataclasses import dataclass, field
from typing import Any, ClassVar
from enum import IntEnum


import numpy as np

log = logging.getLogger("radar_protocol")

# ============================================================================
# Constants matching usb_data_interface.v
# ============================================================================

HEADER_BYTE = 0xAA
FOOTER_BYTE = 0x55
STATUS_HEADER_BYTE = 0xBB

# Packet sizes
DATA_PACKET_SIZE = 11               # 1 + 4 + 2 + 2 + 1 + 1
STATUS_PACKET_SIZE = 26              # 1 + 24 + 1

NUM_RANGE_BINS = 512
NUM_DOPPLER_BINS = 48                 # PR-F/PR-Q: 3 sub-frames * 16 (matches FPGA RP_NUM_DOPPLER_BINS)
NUM_CELLS = NUM_RANGE_BINS * NUM_DOPPLER_BINS  # 24576

WATERFALL_DEPTH = 64

# AUDIT-C9: FT2232H bulk-frame wire format constants. Mirrors
# usb_data_interface_ft2232h.v; if the RTL header changes, update both sides.
BULK_FRAME_HEADER_SIZE   = 8                       # AA + flags + fnum2 + nr2 + nd2
BULK_RANGE_SECTION_BYTES = NUM_RANGE_BINS * 2      # 512 x 2  = 1024
BULK_DOPPLER_MAG_BYTES   = NUM_CELLS * 2           # 16384 x 2 = 32768
BULK_DETECT_DENSE_BYTES  = NUM_CELLS // 8          # 16384 / 8 = 2048
BULK_FOOTER_SIZE         = 1
BULK_FRAME_MIN_SIZE      = BULK_FRAME_HEADER_SIZE + BULK_FOOTER_SIZE   # 9
BULK_FRAME_MAX_SIZE      = (BULK_FRAME_HEADER_SIZE + BULK_RANGE_SECTION_BYTES
                            + BULK_DOPPLER_MAG_BYTES + BULK_DETECT_DENSE_BYTES
                            + BULK_FOOTER_SIZE)                        # 35849

# Bulk-frame format flag bits (matches stream_ctrl_sync_1 layout in RTL).
BULK_FLAG_STREAM_RANGE   = 0x01
BULK_FLAG_STREAM_DOPPLER = 0x02
BULK_FLAG_STREAM_CFAR    = 0x04
BULK_FLAG_MAG_ONLY       = 0x08   # Forced 1 by RTL; full-I/Q write FSM not implemented.
BULK_FLAG_SPARSE_DET     = 0x10   # Forced 0 by RTL; sparse-list write FSM not implemented.


class Opcode(IntEnum):
    """Host register opcodes — must match radar_system_top.v case(usb_cmd_opcode).

    FPGA truth table (from radar_system_top.v lines 902-944):
        0x01  host_radar_mode        0x14  host_short_listen_cycles
        0x02  host_trigger_pulse     0x15  host_chirps_per_elev
        0x03  host_detect_threshold  0x16  host_gain_shift
        0x04  host_stream_control    0x20  host_range_mode
        0x10  host_long_chirp_cycles 0x21-0x27  CFAR / MTI / DC-notch
        0x11  host_long_listen_cycles 0x28-0x2C  AGC control
        0x12  host_guard_cycles      0x30  host_self_test_trigger
        0x13  host_short_chirp_cycles 0x31/0xFF  host_status_request
        0x33  host_adc_format       (AD9484 SCLK/DFS strap; AUDIT-C3)
        (0x32 reserved for the future S-25 adc_pwdn host-control fix)
    """
    # --- Basic control (0x01-0x04) ---
    RADAR_MODE          = 0x01  # 2-bit mode select
    TRIGGER_PULSE       = 0x02  # self-clearing one-shot trigger
    DETECT_THRESHOLD    = 0x03  # 16-bit detection threshold value
    STREAM_CONTROL      = 0x04  # 6-bit stream enable mask (FPGA: usb_cmd_value[5:0])

    # --- Digital gain (0x16) ---
    GAIN_SHIFT          = 0x16  # 4-bit digital gain shift

    # --- Chirp timing (0x10-0x15) ---
    LONG_CHIRP          = 0x10
    LONG_LISTEN         = 0x11
    GUARD               = 0x12
    SHORT_CHIRP         = 0x13
    SHORT_LISTEN        = 0x14
    CHIRPS_PER_ELEV     = 0x15

    # --- Signal processing (0x20-0x27) ---
    RANGE_MODE          = 0x20
    CFAR_GUARD          = 0x21
    CFAR_TRAIN          = 0x22
    CFAR_ALPHA          = 0x23
    CFAR_MODE           = 0x24
    CFAR_ENABLE         = 0x25
    MTI_ENABLE          = 0x26
    DC_NOTCH_WIDTH      = 0x27

    # --- AGC (0x28-0x2C) ---
    AGC_ENABLE          = 0x28
    AGC_TARGET          = 0x29
    AGC_ATTACK          = 0x2A
    AGC_DECAY           = 0x2B
    AGC_HOLDOFF         = 0x2C

    # --- Board self-test / status (0x30-0x31, 0xFF) ---
    SELF_TEST_TRIGGER   = 0x30
    SELF_TEST_STATUS    = 0x31
    STATUS_REQUEST      = 0xFF

    # --- AD9484 ADC sign-convention (0x33, AUDIT-C3) ---
    # 2'b00 = offset-binary (default; SJ1 jumper pins 1-2 bridged)
    # 2'b01 = two's-complement (SJ1 jumper pins 2-3 bridged)
    # AD9484 CSB is hard-tied HIGH on the Main Board (SPI unavailable);
    # this opcode lets the host adapt the DDC to the physical strap
    # without rebuilding the bitstream.
    # (Opcode 0x32 is reserved for the future AUDIT-S25 adc_pwdn fix.)
    ADC_FORMAT          = 0x33


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class RadarFrame:
    """One complete radar frame (64 range x 32 Doppler)."""
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
    # AUDIT-C9: True when this frame came from FT2232H bulk format with
    # mag_only=1 (the only mode FPGA emits today). I/Q arrays will be zero;
    # `magnitude` carries the per-cell Manhattan magnitude from the FPGA.
    mag_only: bool = False


@dataclass
class StatusResponse:
    """Parsed status response from FPGA (6-word / 26-byte packet)."""
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
    # AGC metrics (word 4, added for hybrid AGC)
    agc_current_gain: int = 0    # 4-bit current gain encoding [3:0]
    agc_peak_magnitude: int = 0  # 8-bit peak magnitude [7:0]
    agc_saturation_count: int = 0  # 8-bit saturation count [7:0]
    agc_enable: int = 0          # 1-bit AGC enable readback
    chirps_mismatch: int = 0     # TX-G: 1 if FPGA clamped/rejected host chirps_per_elev


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
        Returns 4 bytes, big-endian (MSB first).
        """
        word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
        return struct.pack(">I", word)

    @staticmethod
    def parse_data_packet(raw: bytes) -> dict[str, Any] | None:
        """
        Parse an 11-byte data packet from the FT2232H byte stream.
        Returns dict with keys: 'range_i', 'range_q', 'doppler_i', 'doppler_q',
        'detection', or None if invalid.

        Packet format (11 bytes):
          Byte 0:    0xAA (header)
          Bytes 1-2: range_q[15:0] MSB first
          Bytes 3-4: range_i[15:0] MSB first
          Bytes 5-6: doppler_real[15:0] MSB first
          Bytes 7-8: doppler_imag[15:0] MSB first
          Byte 9:    {7'b0, cfar_detection}
          Byte 10:   0x55 (footer)
        """
        if len(raw) < DATA_PACKET_SIZE:
            return None
        if raw[0] != HEADER_BYTE:
            return None
        if raw[10] != FOOTER_BYTE:
            return None

        range_q = _to_signed16(struct.unpack_from(">H", raw, 1)[0])
        range_i = _to_signed16(struct.unpack_from(">H", raw, 3)[0])
        doppler_i = _to_signed16(struct.unpack_from(">H", raw, 5)[0])
        doppler_q = _to_signed16(struct.unpack_from(">H", raw, 7)[0])
        det_byte = raw[9]
        detection = det_byte & 0x01
        frame_start = (det_byte >> 7) & 0x01

        return {
            "range_i": range_i,
            "range_q": range_q,
            "doppler_i": doppler_i,
            "doppler_q": doppler_q,
            "detection": detection,
            "frame_start": frame_start,
        }

    @staticmethod
    def parse_status_packet(raw: bytes) -> StatusResponse | None:
        """
        Parse a status response packet.
        Format: [0xBB] [6x4B status words] [0x55] = 1 + 24 + 1 = 26 bytes
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
        # Word 0: {0xFF[31:24], mode[23:22], stream[21:19], 3'b000[18:16], threshold[15:0]}
        sr.cfar_threshold = words[0] & 0xFFFF
        sr.stream_ctrl = (words[0] >> 19) & 0x07
        sr.radar_mode = (words[0] >> 22) & 0x03
        # Word 1: {long_chirp[31:16], long_listen[15:0]}
        sr.long_listen = words[1] & 0xFFFF
        sr.long_chirp = (words[1] >> 16) & 0xFFFF
        # Word 2: {guard[31:16], short_chirp[15:0]}
        sr.short_chirp = words[2] & 0xFFFF
        sr.guard = (words[2] >> 16) & 0xFFFF
        # Word 3: {short_listen[31:16], 10'd0, chirps_per_elev[5:0]}
        sr.chirps_per_elev = words[3] & 0x3F
        sr.short_listen = (words[3] >> 16) & 0xFFFF
        # Word 4 layout: gain[31:28] peak[27:20] sat[19:12] agc_en[11] mismatch[10] mode[1:0]
        sr.range_mode = words[4] & 0x03
        sr.chirps_mismatch = (words[4] >> 10) & 0x01
        sr.agc_enable = (words[4] >> 11) & 0x01
        sr.agc_saturation_count = (words[4] >> 12) & 0xFF
        sr.agc_peak_magnitude = (words[4] >> 20) & 0xFF
        sr.agc_current_gain = (words[4] >> 28) & 0x0F
        # Word 5: {7'd0, self_test_busy, 8'd0, self_test_detail[7:0],
        #           3'd0, self_test_flags[4:0]}
        sr.self_test_flags = words[5] & 0x1F
        sr.self_test_detail = (words[5] >> 8) & 0xFF
        sr.self_test_busy = (words[5] >> 24) & 0x01
        return sr

    @staticmethod
    def find_packet_boundaries(buf: bytes) -> list[tuple[int, int, str]]:
        """
        Scan buffer for packet start markers (0xAA data, 0xBB status).
        Returns list of (start_idx, expected_end_idx, packet_type).

        GUI-S1: in addition to header+footer, validate fixed structural
        bytes the FPGA always emits in known patterns. This rejects false
        starts where a payload byte happens to be 0xAA/0xBB and the byte
        DATA/STATUS_PACKET_SIZE later happens to be 0x55:
          - data byte 9   = {frame_start, 6'b0, cfar_detection} → bits[6:1]==0
          - status byte 1 = high byte of status_words[0]        → 0xFF
        Drops false-match probability from 1/256 to ~1/16384 (data) /
        ~1/65536 (status).
        """
        packets = []
        i = 0
        n = len(buf)
        while i < n:
            if buf[i] == HEADER_BYTE:
                end = i + DATA_PACKET_SIZE
                if end > n:
                    break  # partial packet at end — leave for residual
                if (buf[end - 1] == FOOTER_BYTE and
                        (buf[i + 9] & 0x7E) == 0):
                    packets.append((i, end, "data"))
                    i = end
                else:
                    i += 1  # structural mismatch — skip this false header
            elif buf[i] == STATUS_HEADER_BYTE:
                end = i + STATUS_PACKET_SIZE
                if end > n:
                    break  # partial status packet — leave for residual
                if (buf[end - 1] == FOOTER_BYTE and
                        buf[i + 1] == 0xFF):
                    packets.append((i, end, "status"))
                    i = end
                else:
                    i += 1
            else:
                i += 1
        return packets

    # ----------------------------------------------------------------
    # AUDIT-C9: FT2232H bulk-frame parsing (production board path)
    # ----------------------------------------------------------------
    @staticmethod
    def _bulk_frame_size_from_flags(flags: int) -> int:
        """Compute the on-wire size of a bulk frame from its flags byte.

        Tracks the FPGA write FSM in usb_data_interface_ft2232h.v: header
        (8 B) + per-stream payload + footer (1 B). The mag_only and
        sparse_det bits are documented in the wire format but the FPGA
        write FSM does not implement the alternate encodings yet — it
        always emits 32768 B mag and 2048 B dense bitmap. The host-side
        register handler in radar_system_top.v force-clamps these flags
        when USB_MODE=1, so any frame the parser sees in production will
        have mag_only=1 and sparse_det=0.
        """
        size = BULK_FRAME_HEADER_SIZE
        if flags & BULK_FLAG_STREAM_RANGE:
            size += BULK_RANGE_SECTION_BYTES
        if flags & BULK_FLAG_STREAM_DOPPLER:
            size += BULK_DOPPLER_MAG_BYTES
        if flags & BULK_FLAG_STREAM_CFAR:
            size += BULK_DETECT_DENSE_BYTES
        size += BULK_FOOTER_SIZE
        return size

    @staticmethod
    def parse_bulk_frame(raw: bytes, offset: int = 0) -> dict[str, Any] | None:
        """Parse one FT2232H bulk frame starting at `offset`.

        Returns a dict with keys: frame_number, flags, n_range, n_doppler,
        range_profile (np.ndarray | None), doppler_mag (np.ndarray | None,
        shape n_rangexn_doppler), cfar_dense (np.ndarray | None, shape
        n_rangexn_doppler, uint8 0/1), and frame_size (total bytes consumed
        including header + sections + footer). Returns None on any
        structural error (bad header/footer, wrong bin counts, reserved
        bits set, unimplemented flag combo).
        """
        n = len(raw)
        if n - offset < BULK_FRAME_MIN_SIZE:
            return None
        if raw[offset] != HEADER_BYTE:
            return None

        flags = raw[offset + 1]
        # Reserved high bits must be zero (FPGA emits {2'b00, 6-bit flags}).
        if flags & 0xC0:
            return None
        # Production FPGA only emits mag_only=1 + dense bitmap. Any other
        # encoding is either a corrupt frame or a future RTL revision the
        # parser hasn't been updated for; reject so the caller can resync.
        if (flags & BULK_FLAG_STREAM_DOPPLER) and not (flags & BULK_FLAG_MAG_ONLY):
            return None
        if (flags & BULK_FLAG_STREAM_CFAR) and (flags & BULK_FLAG_SPARSE_DET):
            return None

        frame_number = (raw[offset + 2] << 8) | raw[offset + 3]
        n_range      = (raw[offset + 4] << 8) | raw[offset + 5]
        n_doppler    = (raw[offset + 6] << 8) | raw[offset + 7]
        if n_range != NUM_RANGE_BINS or n_doppler != NUM_DOPPLER_BINS:
            return None

        size = RadarProtocol._bulk_frame_size_from_flags(flags)
        if n - offset < size:
            return None
        if raw[offset + size - 1] != FOOTER_BYTE:
            return None

        cursor = offset + BULK_FRAME_HEADER_SIZE
        range_profile = None
        doppler_mag = None
        cfar_dense = None

        if flags & BULK_FLAG_STREAM_RANGE:
            range_profile = np.frombuffer(
                raw, dtype=">u2", count=n_range, offset=cursor,
            ).astype(np.uint16, copy=True)
            cursor += BULK_RANGE_SECTION_BYTES

        if flags & BULK_FLAG_STREAM_DOPPLER:
            doppler_mag = np.frombuffer(
                raw, dtype=">u2", count=n_range * n_doppler, offset=cursor,
            ).astype(np.uint16, copy=True).reshape(n_range, n_doppler)
            cursor += BULK_DOPPLER_MAG_BYTES

        if flags & BULK_FLAG_STREAM_CFAR:
            packed = np.frombuffer(
                raw, dtype=np.uint8, count=BULK_DETECT_DENSE_BYTES, offset=cursor,
            )
            # Each byte holds 8 cells, MSB-first bit order (matches FPGA
            # WR_DETECT_DATA emission). np.unpackbits keeps that order.
            cfar_dense = np.unpackbits(packed).reshape(n_range, n_doppler)
            cursor += BULK_DETECT_DENSE_BYTES

        return {
            "frame_number":  frame_number,
            "flags":         flags,
            "n_range":       n_range,
            "n_doppler":     n_doppler,
            "range_profile": range_profile,
            "doppler_mag":   doppler_mag,
            "cfar_dense":    cfar_dense,
            "frame_size":    size,
        }

    @staticmethod
    def find_bulk_frame_boundaries(buf: bytes) -> list[tuple[int, int, str]]:
        """Scan a byte stream for FT2232H bulk frames and status packets.

        Status packets (0xBB header, 26 B) are unchanged between transports
        — the WR_STATUS_SEND state in usb_data_interface_ft2232h.v emits the
        same layout as the legacy FT601 path. Bulk data frames (0xAA header)
        are variable length per `_bulk_frame_size_from_flags`.

        Returns a list of (start, end, ptype) tuples like
        find_packet_boundaries, where ptype is "data" or "status". On a
        false header (any structural mismatch) the cursor advances by 1 and
        keeps scanning, mirroring the legacy parser's resync semantics.
        """
        out: list[tuple[int, int, str]] = []
        i = 0
        n = len(buf)
        while i < n:
            b = buf[i]
            if b == HEADER_BYTE:
                # Need at least the 8-byte header to compute the frame size.
                if n - i < BULK_FRAME_HEADER_SIZE:
                    break  # partial header — caller keeps as residual
                flags = buf[i + 1]
                # Quick reject before the more expensive size compute. The
                # full validation lives in parse_bulk_frame.
                if flags & 0xC0:
                    i += 1
                    continue
                size = RadarProtocol._bulk_frame_size_from_flags(flags)
                if n - i < size:
                    break  # partial frame — keep as residual
                # Validate footer + bin counts before accepting the boundary.
                if (buf[i + size - 1] == FOOTER_BYTE
                        and ((buf[i + 4] << 8) | buf[i + 5]) == NUM_RANGE_BINS
                        and ((buf[i + 6] << 8) | buf[i + 7]) == NUM_DOPPLER_BINS):
                    out.append((i, i + size, "data"))
                    i += size
                else:
                    i += 1
            elif b == STATUS_HEADER_BYTE:
                end = i + STATUS_PACKET_SIZE
                if end > n:
                    break
                if buf[end - 1] == FOOTER_BYTE and buf[i + 1] == 0xFF:
                    out.append((i, end, "status"))
                    i = end
                else:
                    i += 1
            else:
                i += 1
        return out


# ============================================================================
# FT2232H USB 2.0 Connection (pyftdi, 245 Synchronous FIFO)
# ============================================================================

# Optional pyftdi import
try:
    from pyftdi.ftdi import Ftdi, FtdiError
    PyFtdi = Ftdi
    PYFTDI_AVAILABLE = True
except ImportError:
    class FtdiError(Exception):
        """Fallback FTDI error type when pyftdi is unavailable."""

    PYFTDI_AVAILABLE = False


class FT2232HConnection:
    """
    FT2232H USB 2.0 Hi-Speed FIFO bridge communication.
    Uses pyftdi in 245 Synchronous FIFO mode (Channel A).
    VID:PID = 0x0403:0x6010 (FTDI default for FT2232H).
    """

    VID = 0x0403
    PID = 0x6010

    def __init__(self, mock: bool = True):
        self._mock = mock
        self._ftdi = None
        self._lock = threading.Lock()
        self.is_open = False
        # Mock state
        self._mock_frame_num = 0
        self._mock_rng = np.random.RandomState(42)

    def open(self, device_index: int = 0) -> bool:
        if self._mock:
            self.is_open = True
            log.info("FT2232H mock device opened (no hardware)")
            return True

        if not PYFTDI_AVAILABLE:
            log.error("pyftdi not installed — cannot open real FT2232H device")
            return False

        try:
            self._ftdi = PyFtdi()
            url = f"ftdi://0x{self.VID:04x}:0x{self.PID:04x}/{device_index + 1}"
            self._ftdi.open_from_url(url)
            # Configure for 245 Synchronous FIFO mode
            self._ftdi.set_bitmode(0xFF, PyFtdi.BitMode.SYNCFF)
            # Set USB transfer size for throughput
            self._ftdi.read_data_set_chunksize(65536)
            self._ftdi.write_data_set_chunksize(65536)
            # Purge buffers
            self._ftdi.purge_buffers()
            self.is_open = True
            log.info(f"FT2232H device opened: {url}")
            return True
        except FtdiError as e:
            log.error(f"FT2232H open failed: {e}")
            return False

    def close(self):
        if self._ftdi is not None:
            with contextlib.suppress(Exception):
                self._ftdi.close()
            self._ftdi = None
        self.is_open = False

    def read(self, size: int = 4096) -> bytes | None:
        """Read raw bytes from FT2232H. Returns None on error/timeout."""
        if not self.is_open:
            return None

        if self._mock:
            return self._mock_read(size)

        with self._lock:
            try:
                data = self._ftdi.read_data(size)
                return bytes(data) if data else None
            except FtdiError as e:
                log.error(f"FT2232H read error: {e}")
                return None

    def write(self, data: bytes) -> bool:
        """Write raw bytes to FT2232H (4-byte commands)."""
        if not self.is_open:
            return False

        if self._mock:
            log.info(f"FT2232H mock write: {data.hex()}")
            return True

        with self._lock:
            try:
                written = self._ftdi.write_data(data)
                return written == len(data)
            except FtdiError as e:
                log.error(f"FT2232H write error: {e}")
                return False

    def _mock_read(self, size: int) -> bytes:
        """Generate one synthetic FT2232H bulk frame per call.

        Mirrors `usb_data_interface_ft2232h.v` production behavior: mag-only
        Doppler section + dense-bitmap CFAR, all three streams enabled
        (matches `RP_STREAM_CTRL_DEFAULT = 6'b001_111`). A target is injected
        near range bin 20, Doppler bin 8 so dashboards have something to draw.
        """
        time.sleep(0.05)
        self._mock_frame_num += 1
        flags = (BULK_FLAG_STREAM_RANGE | BULK_FLAG_STREAM_DOPPLER
                 | BULK_FLAG_STREAM_CFAR | BULK_FLAG_MAG_ONLY)

        # Synthesize per-cell magnitudes once (vectorised).
        rbins = np.arange(NUM_RANGE_BINS).reshape(-1, 1)
        dbins = np.arange(NUM_DOPPLER_BINS).reshape(1, -1)
        noise = np.abs(self._mock_rng.normal(0, 50, size=(NUM_RANGE_BINS, NUM_DOPPLER_BINS)))
        target_mask = (np.abs(rbins - 20) < 3) & (np.abs(dbins - 8) < 2)
        mag = noise + target_mask * 12000.0
        mag_u16 = np.clip(mag, 0, 65535).astype(np.uint16)

        range_profile = np.clip(
            np.abs(self._mock_rng.normal(0, 100, size=NUM_RANGE_BINS))
            + (np.abs(rbins.flatten() - 20) < 3) * 8000,
            0, 65535,
        ).astype(np.uint16)

        det = (target_mask & (np.abs(dbins - 8) < 2) & (np.abs(rbins - 20) < 2)).astype(np.uint8)
        det_packed = np.packbits(det.flatten())  # MSB-first bit order matches FPGA

        buf = bytearray(BULK_FRAME_MAX_SIZE)
        buf[0] = HEADER_BYTE
        buf[1] = flags & 0x3F  # reserved high bits zero, matches RTL
        buf[2] = (self._mock_frame_num >> 8) & 0xFF
        buf[3] = self._mock_frame_num & 0xFF
        buf[4] = (NUM_RANGE_BINS >> 8) & 0xFF
        buf[5] = NUM_RANGE_BINS & 0xFF
        buf[6] = (NUM_DOPPLER_BINS >> 8) & 0xFF
        buf[7] = NUM_DOPPLER_BINS & 0xFF
        cursor = BULK_FRAME_HEADER_SIZE
        # Range profile (>u2 = big-endian uint16, matches FPGA MSB-first).
        buf[cursor:cursor + BULK_RANGE_SECTION_BYTES] = range_profile.astype(">u2").tobytes()
        cursor += BULK_RANGE_SECTION_BYTES
        buf[cursor:cursor + BULK_DOPPLER_MAG_BYTES] = mag_u16.astype(">u2").tobytes()
        cursor += BULK_DOPPLER_MAG_BYTES
        buf[cursor:cursor + BULK_DETECT_DENSE_BYTES] = det_packed.tobytes()
        cursor += BULK_DETECT_DENSE_BYTES
        buf[cursor] = FOOTER_BYTE

        # `size` is the host's read budget; emit at most one frame per call
        # (matches typical FT2232H driver semantics).
        return bytes(buf[:min(size, BULK_FRAME_MAX_SIZE)])


# ============================================================================
# FT601 USB 3.0 Connection (premium board only)
# ============================================================================

# Optional ftd3xx import (FTDI's proprietary driver for FT60x USB 3.0 chips).
# pyftdi does NOT support FT601 — it only handles USB 2.0 chips (FT232H, etc.)
try:
    import ftd3xx  # type: ignore[import-untyped]
    FTD3XX_AVAILABLE = True
    _Ftd3xxError: type = ftd3xx.FTD3XXError  # type: ignore[attr-defined]
except ImportError:
    FTD3XX_AVAILABLE = False
    _Ftd3xxError = OSError  # fallback for type-checking; never raised


class FT601Connection:
    """
    FT601 USB 3.0 SuperSpeed FIFO bridge — premium board only.

    The FT601 has a 32-bit data bus and runs at 100 MHz.
    VID:PID = 0x0403:0x6030 or 0x6031 (FTDI FT60x).

    Requires the ``ftd3xx`` library (``pip install ftd3xx`` on Windows,
    or ``libft60x`` on Linux). This is FTDI's proprietary USB 3.0 driver;
    ``pyftdi`` only supports USB 2.0 and will NOT work with FT601.

    Public contract matches FT2232HConnection so callers can swap freely.
    """

    VID = 0x0403
    PID_LIST: ClassVar[list[int]] = [0x6030, 0x6031]

    def __init__(self, mock: bool = True):
        self._mock = mock
        self._dev = None
        self._lock = threading.Lock()
        self.is_open = False
        # Mock state (reuses same synthetic data pattern)
        self._mock_frame_num = 0
        self._mock_rng = np.random.RandomState(42)

    def open(self, device_index: int = 0) -> bool:
        if self._mock:
            self.is_open = True
            log.info("FT601 mock device opened (no hardware)")
            return True

        if not FTD3XX_AVAILABLE:
            log.error(
                "ftd3xx library required for FT601 hardware — "
                "install with: pip install ftd3xx"
            )
            return False

        try:
            self._dev = ftd3xx.create(device_index, ftd3xx.OPEN_BY_INDEX)
            if self._dev is None:
                log.error("No FT601 device found at index %d", device_index)
                return False
            # Verify chip configuration — only reconfigure if needed.
            # setChipConfiguration triggers USB re-enumeration, which
            # invalidates the device handle and requires a re-open cycle.
            cfg = self._dev.getChipConfiguration()
            needs_reconfig = (
                cfg.FIFOMode != 0            # 245 FIFO mode
                or cfg.ChannelConfig != 0    # 1 channel, 32-bit
                or cfg.OptionalFeatureSupport != 0
            )
            if needs_reconfig:
                cfg.FIFOMode = 0
                cfg.ChannelConfig = 0
                cfg.OptionalFeatureSupport = 0
                self._dev.setChipConfiguration(cfg)
                # Device re-enumerates — close stale handle, wait, re-open
                self._dev.close()
                self._dev = None
                import time
                time.sleep(2.0)  # wait for USB re-enumeration
                self._dev = ftd3xx.create(device_index, ftd3xx.OPEN_BY_INDEX)
                if self._dev is None:
                    log.error("FT601 not found after reconfiguration")
                    return False
                log.info("FT601 reconfigured and re-opened (index %d)", device_index)
            self.is_open = True
            log.info("FT601 device opened (index %d)", device_index)
            return True
        except (OSError, _Ftd3xxError) as e:
            log.error("FT601 open failed: %s", e)
            self._dev = None
            return False

    def close(self):
        if self._dev is not None:
            with contextlib.suppress(Exception):
                self._dev.close()
            self._dev = None
        self.is_open = False

    def read(self, size: int = 4096) -> bytes | None:
        """Read raw bytes from FT601. Returns None on error/timeout."""
        if not self.is_open:
            return None

        if self._mock:
            return self._mock_read(size)

        with self._lock:
            try:
                data = self._dev.readPipe(0x82, size, raw=True)
                return bytes(data) if data else None
            except (OSError, _Ftd3xxError) as e:
                log.error("FT601 read error: %s", e)
                return None

    def write(self, data: bytes) -> bool:
        """Write raw bytes to FT601. Data must be 4-byte aligned for 32-bit bus."""
        if not self.is_open:
            return False

        if self._mock:
            log.info(f"FT601 mock write: {data.hex()}")
            return True

        # Pad to 4-byte alignment (FT601 32-bit bus requirement).
        # NOTE: Radar commands are already 4 bytes, so this should be a no-op.
        remainder = len(data) % 4
        if remainder:
            data = data + b"\x00" * (4 - remainder)

        with self._lock:
            try:
                written = self._dev.writePipe(0x02, data, raw=True)
                return written == len(data)
            except (OSError, _Ftd3xxError) as e:
                log.error("FT601 write error: %s", e)
                return False

    def _mock_read(self, size: int) -> bytes:
        """Generate synthetic radar packets (same pattern as FT2232H mock)."""
        time.sleep(0.05)
        self._mock_frame_num += 1

        buf = bytearray()
        num_packets = min(NUM_CELLS, size // DATA_PACKET_SIZE)
        start_idx = getattr(self, "_mock_seq_idx", 0)

        for n in range(num_packets):
            idx = (start_idx + n) % NUM_CELLS
            rbin = idx // NUM_DOPPLER_BINS
            dbin = idx % NUM_DOPPLER_BINS

            range_i = int(self._mock_rng.normal(0, 100))
            range_q = int(self._mock_rng.normal(0, 100))
            if abs(rbin - 20) < 3:
                range_i += 5000
                range_q += 3000

            dop_i = int(self._mock_rng.normal(0, 50))
            dop_q = int(self._mock_rng.normal(0, 50))
            if abs(rbin - 20) < 3 and abs(dbin - 8) < 2:
                dop_i += 8000
                dop_q += 4000

            detection = 1 if (abs(rbin - 20) < 2 and abs(dbin - 8) < 2) else 0

            pkt = bytearray()
            pkt.append(HEADER_BYTE)
            pkt += struct.pack(">h", np.clip(range_q, -32768, 32767))
            pkt += struct.pack(">h", np.clip(range_i, -32768, 32767))
            pkt += struct.pack(">h", np.clip(dop_i, -32768, 32767))
            pkt += struct.pack(">h", np.clip(dop_q, -32768, 32767))
            # Bit 7 = frame_start (sample_counter == 0), bit 0 = detection
            det_byte = (detection & 0x01) | (0x80 if idx == 0 else 0x00)
            pkt.append(det_byte)
            pkt.append(FOOTER_BYTE)

            buf += pkt

        self._mock_seq_idx = (start_idx + num_packets) % NUM_CELLS
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
        except (OSError, ValueError) as e:
            log.error(f"Failed to start recording: {e}")

    def record_frame(self, frame: RadarFrame):
        if not self._recording or self._file is None:
            return
        # GUI-S2: snapshot the arrays before handing them to h5py. The same
        # frame object is also queued for the display consumer, and h5py
        # releases the GIL during gzip compression — without this copy, any
        # in-place mutation by the consumer (or a future scaling/normalization
        # step) would tear the on-disk frame.
        try:
            mag  = np.asarray(frame.magnitude).copy()
            rdi  = np.asarray(frame.range_doppler_i).copy()
            rdq  = np.asarray(frame.range_doppler_q).copy()
            det  = np.asarray(frame.detections).copy()
            rprf = np.asarray(frame.range_profile).copy()
            fg = self._grp.create_group(f"frame_{self._frame_count:06d}")
            fg.attrs["timestamp"] = frame.timestamp
            fg.attrs["frame_number"] = frame.frame_number
            fg.attrs["detection_count"] = frame.detection_count
            fg.create_dataset("magnitude", data=mag, compression="gzip")
            fg.create_dataset("range_doppler_i", data=rdi, compression="gzip")
            fg.create_dataset("range_doppler_q", data=rdq, compression="gzip")
            fg.create_dataset("detections", data=det, compression="gzip")
            fg.create_dataset("range_profile", data=rprf, compression="gzip")
            self._frame_count += 1
        except (OSError, ValueError, TypeError) as e:
            log.error(f"Recording error: {e}")

    def stop(self):
        if self._file is not None:
            try:
                self._file.attrs["end_time"] = time.time()
                self._file.attrs["total_frames"] = self._frame_count
                self._file.close()
            except (OSError, ValueError, RuntimeError):
                pass
            self._file = None
        self._recording = False
        log.info(f"Recording stopped ({self._frame_count} frames)")


# ============================================================================
# Radar Data Acquisition Thread
# ============================================================================

class RadarAcquisition(threading.Thread):
    """Background thread: reads USB bytes, parses frames, queues them.

    Dispatches between two wire formats based on connection type:
      - FT2232HConnection -> bulk per-frame format (parses 35 KB frames in
        one shot via parse_bulk_frame; fills RadarFrame.magnitude directly).
      - FT601Connection   -> legacy 11-byte per-sample format (count-based
        sample placement via _ingest_sample, the original behavior).

    See module docstring for why both formats exist.
    """

    def __init__(self, connection, frame_queue: queue.Queue,
                 recorder: DataRecorder | None = None,
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
        # AUDIT-C9: dispatch on connection type. The bulk path skips the
        # per-sample state machine entirely.
        self._is_bulk = isinstance(connection, FT2232HConnection)
        self._read_chunk = (2 * BULK_FRAME_MAX_SIZE) if self._is_bulk else 4096

    def stop(self):
        self._stop_event.set()

    def run(self):
        log.info(
            "Acquisition thread started (%s wire format)",
            "FT2232H bulk" if self._is_bulk else "FT601 legacy 11-byte",
        )
        residual = b""
        while not self._stop_event.is_set():
            chunk = self.conn.read(self._read_chunk)
            if chunk is None or len(chunk) == 0:
                time.sleep(0.01)
                continue

            raw = residual + chunk
            if self._is_bulk:
                packets = RadarProtocol.find_bulk_frame_boundaries(raw)
                max_residual = BULK_FRAME_MAX_SIZE
            else:
                packets = RadarProtocol.find_packet_boundaries(raw)
                max_residual = 2 * max(DATA_PACKET_SIZE, STATUS_PACKET_SIZE)

            # Keep unparsed tail bytes for next iteration.
            if packets:
                last_end = packets[-1][1]
                residual = raw[last_end:]
            else:
                residual = raw[-max_residual:] if len(raw) > max_residual else raw

            for start, end, ptype in packets:
                if ptype == "data":
                    if self._is_bulk:
                        parsed = RadarProtocol.parse_bulk_frame(raw, offset=start)
                        if parsed is not None:
                            self._ingest_bulk_frame(parsed)
                    else:
                        sample = RadarProtocol.parse_data_packet(raw[start:end])
                        if sample is not None:
                            self._ingest_sample(sample)
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
                            except Exception as e:  # noqa: BLE001
                                log.error(f"Status callback error: {e}")

        log.info("Acquisition thread stopped")

    def _ingest_bulk_frame(self, parsed: dict):
        """Build a RadarFrame from one parsed bulk frame and emit it."""
        frame = RadarFrame()
        frame.timestamp = time.time()
        frame.frame_number = parsed["frame_number"]
        frame.mag_only = bool(parsed["flags"] & BULK_FLAG_MAG_ONLY)

        rprof = parsed["range_profile"]
        if rprof is not None:
            # Wire is uint16; RadarFrame.range_profile is float64.
            frame.range_profile[:] = rprof.astype(np.float64)

        dmag = parsed["doppler_mag"]
        if dmag is not None:
            frame.magnitude[:] = dmag.astype(np.float64)
            # I/Q arrays stay zero in mag-only mode (the only mode FPGA
            # emits today). Future RTL may populate them; for now flag is
            # the source of truth.

        cdense = parsed["cfar_dense"]
        if cdense is not None:
            frame.detections[:] = cdense.astype(np.uint8)
            frame.detection_count = int(cdense.sum())

        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                self.frame_queue.get_nowait()
            self.frame_queue.put_nowait(frame)

        if self.recorder and self.recorder.recording:
            self.recorder.record_frame(frame)

    def _ingest_sample(self, sample: dict):
        """Place sample into current frame and emit when complete."""
        # [GUI-C2 FIX] Use FPGA frame_start bit as the authoritative sync token.
        # If FPGA flags frame_start mid-stream (after a USB drop or any glitch),
        # finalize whatever we have and re-align to bin (0, 0). Without this the
        # count-only sync stays permanently misaligned after a single dropped byte.
        if sample.get("frame_start", 0) and self._sample_idx > 0:
            self._finalize_frame()  # resets _sample_idx to 0 and starts a new frame

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
            # [GUI-C4 FIX] FPGA emits the same range_i/range_q for all 32 Doppler
            # bins of a given range bin (it's the matched-filter range output,
            # repeated per Doppler cell). Accumulating across all 32 inflates
            # the profile 32x. Capture once per range bin at the first Doppler
            # cell instead.
            if dbin == 0:
                ri = int(sample.get("range_i", 0))
                rq = int(sample.get("range_q", 0))
                self._frame.range_profile[rbin] = abs(ri) + abs(rq)

        self._sample_idx += 1

        if self._sample_idx >= NUM_CELLS:
            self._finalize_frame()

    def _finalize_frame(self):
        """Complete frame: push to queue, record."""
        self._frame.timestamp = time.time()
        self._frame.frame_number = self._frame_num
        # range_profile is already accumulated from FPGA range_i/range_q
        # data in _ingest_sample(). No need to synthesize from doppler magnitude.

        # Push to display queue (drop old if backed up)
        try:
            self.frame_queue.put_nowait(self._frame)
        except queue.Full:
            with contextlib.suppress(queue.Empty):
                self.frame_queue.get_nowait()
            self.frame_queue.put_nowait(self._frame)

        if self.recorder and self.recorder.recording:
            self.recorder.record_frame(self._frame)

        self._frame_num += 1
        self._frame = RadarFrame()
        self._sample_idx = 0
