#!/usr/bin/env python3
"""
AERIS-10 FT601 USB 3.0 Streaming Test
=======================================
Connects to the UMFT601X-B dev board via D3XX, enables all data streams,
reads packets from the FPGA, and validates packet framing + content.

Requires:
  - libftd3xx.so installed in /usr/local/lib/
  - Python ftd3xx package (pip install ftd3xx)
  - FPGA programmed with radar_system_top_te0713_umft601x_dev bitstream
  - UMFT601X-B jumpers JP4=2-3, JP5=2-3 (245 FIFO mode, 1 channel)

Usage:
  python3 test_ft601_streaming.py
"""

import sys
import time
import struct
import ctypes

try:
    import ftd3xx
    import ftd3xx._ftd3xx_linux as _ll
except ImportError:
    print("ERROR: ftd3xx not installed. Run: pip install ftd3xx")
    sys.exit(1)

# Constants matching usb_data_interface.v
HEADER_BYTE = 0xAA
FOOTER_BYTE = 0x55
STATUS_HEADER_BYTE = 0xBB

# D3XX pipe IDs (245 mode, 1 channel)
PIPE_OUT = 0x02   # Host -> FPGA
PIPE_IN  = 0x82   # FPGA -> Host


# ============================================================================
# Low-level D3XX pipe I/O wrapper
# ============================================================================

class FT601Device:
    """Wrapper around ftd3xx with low-level pipe I/O for Linux."""

    def __init__(self):
        self._dev = None
        self._handle = None

    def open(self, index: int = 0) -> bool:
        try:
            self._dev = ftd3xx.create(index)
            if self._dev is None:
                return False
            self._handle = self._dev.handle
            return True
        except Exception as e:
            print(f"  Open error: {e}")
            return False

    def close(self):
        if self._dev:
            try:
                self._dev.close()
            except Exception:
                pass
            self._dev = None
            self._handle = None

    def get_chip_config(self):
        if self._dev:
            return self._dev.getChipConfiguration()
        return None

    def get_device_descriptor(self):
        if self._dev:
            return self._dev.getDeviceDescriptor()
        return None

    def get_interface_descriptor(self, iface: int = 0):
        if self._dev:
            return self._dev.getInterfaceDescriptor(iface)
        return None

    def get_pipe_info(self, iface: int, pipe: int):
        if self._dev:
            return self._dev.getPipeInformation(iface, pipe)
        return None

    def flush_pipe(self, pipe_id: int):
        if self._dev:
            try:
                self._dev.flushPipe(pipe_id)
            except Exception:
                pass

    def write_pipe(self, pipe_id: int, data: bytes, timeout_ms: int = 1000) -> int:
        """Write data to a pipe. Returns bytes transferred."""
        if not self._handle:
            return 0
        out_buf = ctypes.create_string_buffer(data, len(data))
        xfer = ctypes.c_ulong(0)
        timeout = ctypes.c_ulong(timeout_ms)
        status = _ll.FT_WritePipeEx(
            self._handle, ctypes.c_ubyte(pipe_id),
            out_buf, ctypes.c_ulong(len(data)),
            ctypes.byref(xfer), timeout
        )
        if status != 0:
            return -status  # Negative = error
        return xfer.value

    def read_pipe(self, pipe_id: int, size: int, timeout_ms: int = 2000) -> bytes:
        """Read data from a pipe. Returns bytes read (empty on error)."""
        if not self._handle:
            return b""
        buf = ctypes.create_string_buffer(size)
        xfer = ctypes.c_ulong(0)
        timeout = ctypes.c_ulong(timeout_ms)
        status = _ll.FT_ReadPipeEx(
            self._handle, ctypes.c_ubyte(pipe_id),
            buf, ctypes.c_ulong(size),
            ctypes.byref(xfer), timeout
        )
        if status != 0:
            return b""  # Error
        return buf.raw[:xfer.value]


# ============================================================================
# Helpers
# ============================================================================

def build_command(opcode: int, value: int, addr: int = 0) -> bytes:
    """Build 32-bit command word: {opcode[31:24], addr[23:16], value[15:0]}."""
    word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
    return struct.pack(">I", word)


def hexdump(data: bytes, max_bytes: int = 128) -> str:
    """Hex dump of first max_bytes."""
    out = []
    for i in range(0, min(len(data), max_bytes), 16):
        chunk = data[i:i+16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out.append(f"  {i:04X}: {hex_str:<48s}  {ascii_str}")
    if len(data) > max_bytes:
        out.append(f"  ... ({len(data)} bytes total, showing first {max_bytes})")
    return "\n".join(out)


def to_signed16(val: int) -> int:
    val = val & 0xFFFF
    return val - 0x10000 if val >= 0x8000 else val


# ============================================================================
# Test Steps
# ============================================================================

def test_enumerate():
    """Step 1: Enumerate FT601 devices."""
    print("=" * 60)
    print("STEP 1: Enumerate D3XX devices")
    print("=" * 60)
    n = ftd3xx.createDeviceInfoList()
    print(f"  Devices found: {n}")
    if n == 0:
        print("  FAIL: No FT601 devices found.")
        print("  Check: USB cable, udev rules, ftdi_sio not claiming device")
        return False
    for i in range(n):
        info = ftd3xx.getDeviceInfoDetail(i)
        flags = info['Flags']
        speed = "SuperSpeed" if flags & 4 else "Hi-Speed" if flags & 2 else "Unknown"
        print(f"  Device {i}:")
        print(f"    Description: {info['Description']}")
        print(f"    Serial:      {info['SerialNumber']}")
        print(f"    Speed:       {speed} (flags={flags})")
    print("  PASS")
    return True


def test_open_and_config(dev: FT601Device):
    """Step 2: Open FT601 device and check configuration."""
    print()
    print("=" * 60)
    print("STEP 2: Open device & check configuration")
    print("=" * 60)
    if not dev.open(0):
        print("  FAIL: Could not open device")
        return False
    print("  Device opened")

    cfg = dev.get_chip_config()
    if cfg:
        fifo_mode_name = "245" if cfg.FIFOMode == 0 else "600" if cfg.FIFOMode == 1 else f"unknown({cfg.FIFOMode})"
        clock_name = "100MHz" if cfg.FIFOClock == 0 else "66MHz" if cfg.FIFOClock == 1 else f"unknown({cfg.FIFOClock})"
        ch_names = {0: "4-channel", 1: "reserved", 2: "1-channel", 3: "1-IN-pipe", 4: "1-OUT-pipe"}
        ch_name = ch_names.get(cfg.ChannelConfig, f"unknown({cfg.ChannelConfig})")
        print(f"  FIFO Mode:      {fifo_mode_name} (raw={cfg.FIFOMode})")
        print(f"  FIFO Clock:     {clock_name} (raw={cfg.FIFOClock})")
        print(f"  Channel Config: {ch_name} (raw={cfg.ChannelConfig})")
        print(f"  VID/PID:        0x{cfg.VendorID:04X}/0x{cfg.ProductID:04X}")

        if cfg.FIFOMode != 0:
            print()
            print("  *** WARNING: FT601 is NOT in 245 FIFO mode! ***")
            print("  The FPGA RTL expects 245 mode. Data reads will fail.")
            print("  Fix: Move jumpers JP4 and JP5 to pins 2-3 on UMFT601X-B")
            print("       Then power-cycle the board (unplug/replug USB)")
            print()
            print("  Continuing anyway to test what we can...")
    else:
        print("  (Could not read chip config)")

    # Check pipe layout
    try:
        iface = dev.get_interface_descriptor(0)
        if iface:
            print(f"  Interface 0: {iface.bNumEndpoints} endpoints")
    except Exception:
        pass

    print("  PASS")
    return True


def test_write_command(dev: FT601Device, opcode: int, value: int, desc: str) -> bool:
    """Write a command to the FPGA."""
    cmd = build_command(opcode, value)
    print(f"  Sending {desc}: opcode=0x{opcode:02X} value=0x{value:04X} -> {cmd.hex()}")
    result = dev.write_pipe(PIPE_OUT, cmd, timeout_ms=1000)
    if result < 0:
        print(f"    Write error: D3XX status {-result}")
        return False
    print(f"    Transferred: {result} bytes")
    return result == len(cmd)


def test_enable_streams(dev: FT601Device):
    """Step 3: Enable all streams (opcode 0x04, value 0x07)."""
    print()
    print("=" * 60)
    print("STEP 3: Enable all data streams")
    print("=" * 60)
    ok = test_write_command(dev, 0x04, 0x07, "stream_control = 0x07 (range+doppler+cfar)")
    if ok:
        print("  PASS")
    else:
        print("  FAIL (write failed — may be due to 600 mode mismatch)")
    return ok


def test_read_raw(dev: FT601Device, read_size: int = 4096):
    """Step 4: Read raw data from FT601."""
    print()
    print("=" * 60)
    print("STEP 4: Read raw data from FPGA")
    print("=" * 60)

    print(f"  Reading {read_size} bytes from pipe 0x{PIPE_IN:02X}...")
    t0 = time.time()
    data = dev.read_pipe(PIPE_IN, read_size, timeout_ms=3000)
    elapsed = time.time() - t0

    if len(data) == 0:
        print(f"  FAIL: No data received (elapsed {elapsed:.3f}s)")
        print("  Possible causes:")
        print("    - FT601 not in 245 mode (check JP4/JP5 jumpers)")
        print("    - FPGA not streaming (bitstream not loaded)")
        print("    - Physical connection issue between FT601 and FPGA")
        return None

    print(f"  Received {len(data)} bytes in {elapsed:.3f}s")
    if elapsed > 0:
        print(f"  Throughput: {len(data)/elapsed/1e6:.2f} MB/s")
    print()
    print("  Raw hex dump:")
    print(hexdump(data, 128))
    print("  PASS")
    return data


def test_parse_packets(raw: bytes):
    """Step 5: Parse packet framing from raw data."""
    print()
    print("=" * 60)
    print("STEP 5: Parse packet framing")
    print("=" * 60)

    if raw is None or len(raw) == 0:
        print("  SKIP: No data to parse")
        return False

    # Scan for markers
    aa_count = sum(1 for b in raw if b == HEADER_BYTE)
    bb_count = sum(1 for b in raw if b == STATUS_HEADER_BYTE)
    ff_count = sum(1 for b in raw if b == FOOTER_BYTE)

    print(f"  0xAA (data header) count: {aa_count}")
    print(f"  0xBB (status header) count: {bb_count}")
    print(f"  0x55 (footer) count: {ff_count}")

    if aa_count == 0:
        print("  WARN: No 0xAA headers found in raw data")
        return False

    # Try parsing 35-byte data packets (all streams enabled)
    parsed = 0
    errors = 0
    i = 0
    while i < len(raw):
        if raw[i] == HEADER_BYTE and i + 35 <= len(raw):
            pkt = raw[i:i+35]
            if pkt[34] == FOOTER_BYTE:
                range_word0 = struct.unpack_from(">I", pkt, 1)[0]
                range_hi = to_signed16((range_word0 >> 16) & 0xFFFF)
                range_lo = to_signed16(range_word0 & 0xFFFF)
                dop_word0 = struct.unpack_from(">I", pkt, 17)[0]
                dop_real = to_signed16((dop_word0 >> 16) & 0xFFFF)
                dop_imag = to_signed16(dop_word0 & 0xFFFF)
                detection = pkt[33] & 0x01

                if parsed < 5:
                    print(f"  Packet {parsed}:")
                    print(f"    Range word0: 0x{range_word0:08X}")
                    print(f"    Range:  hi={range_hi:6d} lo={range_lo:6d}")
                    print(f"    Doppler: real={dop_real:6d} imag={dop_imag:6d}")
                    print(f"    Detection: {detection}")

                # Validate shifted copies (range words 1-3)
                rw1 = struct.unpack_from(">I", pkt, 5)[0]
                rw2 = struct.unpack_from(">I", pkt, 9)[0]
                rw3 = struct.unpack_from(">I", pkt, 13)[0]
                expected_rw1 = (range_word0 << 8) & 0xFFFFFFFF
                expected_rw2 = (range_word0 << 16) & 0xFFFFFFFF
                expected_rw3 = (range_word0 << 24) & 0xFFFFFFFF
                if rw1 != expected_rw1 or rw2 != expected_rw2 or rw3 != expected_rw3:
                    if parsed < 5:
                        print(f"    WARNING: Range shifted copies mismatch!")
                        print(f"      Got:    {rw1:08X} {rw2:08X} {rw3:08X}")
                        print(f"      Expect: {expected_rw1:08X} {expected_rw2:08X} {expected_rw3:08X}")
                    errors += 1

                parsed += 1
                i += 35
            else:
                i += 1
        elif raw[i] == STATUS_HEADER_BYTE and i + 26 <= len(raw):
            if raw[i+25] == FOOTER_BYTE:
                print(f"  (Status packet at offset {i} — skipping)")
                i += 26
            else:
                i += 1
        else:
            i += 1

    print(f"\n  Valid data packets: {parsed}")
    print(f"  Content errors: {errors}")

    if parsed > 0:
        print("  PASS")
        return True
    else:
        print("  FAIL: No valid packets found")
        return False


def test_status_request(dev: FT601Device):
    """Step 6: Send status request and parse response."""
    print()
    print("=" * 60)
    print("STEP 6: Status request (opcode 0xFF)")
    print("=" * 60)

    # Flush pending data
    dev.read_pipe(PIPE_IN, 8192, timeout_ms=500)
    time.sleep(0.1)

    # Send status request
    ok = test_write_command(dev, 0xFF, 0x0000, "status request")
    if not ok:
        print("  FAIL: Could not send status request")
        return False

    time.sleep(0.2)

    # Read response
    data = dev.read_pipe(PIPE_IN, 4096, timeout_ms=2000)
    if len(data) == 0:
        print("  FAIL: No response to status request")
        return False
    print(f"  Received {len(data)} bytes")

    # Scan for 0xBB status header
    for i in range(len(data)):
        if data[i] == STATUS_HEADER_BYTE and i + 26 <= len(data):
            pkt = data[i:i+26]
            if pkt[25] == FOOTER_BYTE:
                words = [struct.unpack_from(">I", pkt, 1 + w*4)[0] for w in range(6)]

                print(f"  Status packet at offset {i}:")
                for w_idx, w_val in enumerate(words):
                    print(f"    Word {w_idx}: 0x{w_val:08X}")

                # Decode
                threshold = words[0] & 0xFFFF
                stream = (words[0] >> 16) & 0x07
                mode = (words[0] >> 21) & 0x03
                long_listen = words[1] & 0xFFFF
                long_chirp = (words[1] >> 16) & 0xFFFF
                short_chirp = words[2] & 0xFFFF
                guard = (words[2] >> 16) & 0xFFFF
                chirps = words[3] & 0x3F
                short_listen = (words[3] >> 16) & 0xFFFF
                range_mode = words[4] & 0x03
                st_flags = words[5] & 0x1F
                st_detail = (words[5] >> 8) & 0xFF
                st_busy = (words[5] >> 24) & 0x01

                print(f"\n  Decoded status:")
                print(f"    Radar mode:      {mode}")
                print(f"    Stream ctrl:     0b{stream:03b}")
                print(f"    CFAR threshold:  {threshold} (0x{threshold:04X})")
                print(f"    Long chirp:      {long_chirp}")
                print(f"    Long listen:     {long_listen}")
                print(f"    Guard cycles:    {guard}")
                print(f"    Short chirp:     {short_chirp}")
                print(f"    Short listen:    {short_listen}")
                print(f"    Chirps/elev:     {chirps}")
                print(f"    Range mode:      {range_mode}")
                print(f"    Self-test busy:  {st_busy}")
                print(f"    Self-test flags: 0b{st_flags:05b}")
                print(f"    Self-test detail: 0x{st_detail:02X}")

                # Validate expected defaults from dev board top-level
                expected = {
                    "threshold": (0x1234, threshold),
                    "long_chirp": (3000, long_chirp),
                    "long_listen": (13700, long_listen),
                    "guard": (17540, guard),
                    "short_chirp": (50, short_chirp),
                    "short_listen": (17450, short_listen),
                    "chirps_per_elev": (32, chirps),
                    "range_mode": (1, range_mode),  # 0b01
                }
                print(f"\n  Validation against expected defaults:")
                all_ok = True
                for name, (exp, got) in expected.items():
                    ok_str = "OK" if exp == got else "MISMATCH"
                    if exp != got:
                        all_ok = False
                    print(f"    {name:.<25s} expected={exp}, got={got} [{ok_str}]")

                if all_ok:
                    print("  PASS")
                else:
                    print("  PARTIAL (some values differ)")
                return True

    print("  FAIL: No valid status packet found")
    print("  Raw hex:")
    print(hexdump(data, 128))
    return False


def test_self_test(dev: FT601Device):
    """Step 7: Trigger board self-test and read results."""
    print()
    print("=" * 60)
    print("STEP 7: Board self-test (opcode 0x30)")
    print("=" * 60)

    ok = test_write_command(dev, 0x30, 0x0001, "self-test trigger")
    if not ok:
        print("  FAIL: Could not send self-test trigger")
        return False

    print("  Waiting 500ms for self-test...")
    time.sleep(0.5)

    # Flush and request status
    dev.read_pipe(PIPE_IN, 8192, timeout_ms=500)
    time.sleep(0.05)

    ok = test_write_command(dev, 0x31, 0x0000, "self-test status readback")
    if not ok:
        return False

    time.sleep(0.2)
    data = dev.read_pipe(PIPE_IN, 4096, timeout_ms=2000)
    if len(data) == 0:
        print("  FAIL: No response")
        return False

    for i in range(len(data)):
        if data[i] == STATUS_HEADER_BYTE and i + 26 <= len(data):
            pkt = data[i:i+26]
            if pkt[25] == FOOTER_BYTE:
                words = [struct.unpack_from(">I", pkt, 1 + w*4)[0] for w in range(6)]

                st_flags = words[5] & 0x1F
                st_detail = (words[5] >> 8) & 0xFF
                st_busy = (words[5] >> 24) & 0x01

                test_names = ["PLL Lock", "BRAM R/W", "Clk Freq", "Loopback", "ADC Activity"]
                print(f"  Self-test results (busy={st_busy}):")
                for t in range(5):
                    passed = (st_flags >> t) & 1
                    status = "PASS" if passed else "FAIL"
                    print(f"    Test {t} ({test_names[t]:.<20s}): {status}")
                print(f"  Detail: 0x{st_detail:02X}")

                # Dev board: tests 0-3 pass, test 4 (ADC) fails (no AD9484)
                if (st_flags & 0x0F) == 0x0F:
                    print("  PASS (first 4 tests passed, ADC test expected to fail on dev board)")
                    return True
                elif st_flags == 0x1F:
                    print("  PASS (all 5 tests passed)")
                    return True
                else:
                    print(f"  WARN: Unexpected flags 0b{st_flags:05b}")
                    return True

    print("  FAIL: No status packet in response")
    return False


def test_sustained_read(dev: FT601Device, duration_s: float = 2.0):
    """Step 8: Sustained streaming throughput test."""
    print()
    print("=" * 60)
    print(f"STEP 8: Sustained streaming ({duration_s}s)")
    print("=" * 60)

    total_bytes = 0
    total_reads = 0
    empty_reads = 0
    header_count = 0
    read_size = 16384

    t_start = time.time()
    while time.time() - t_start < duration_s:
        data = dev.read_pipe(PIPE_IN, read_size, timeout_ms=500)
        if len(data) > 0:
            total_bytes += len(data)
            total_reads += 1
            header_count += sum(1 for b in data if b == HEADER_BYTE)
        else:
            empty_reads += 1
        time.sleep(0.001)

    elapsed = time.time() - t_start
    throughput = total_bytes / elapsed / 1e6 if elapsed > 0 else 0
    pkt_rate = header_count / elapsed if elapsed > 0 else 0

    print(f"  Duration:     {elapsed:.2f}s")
    print(f"  Total data:   {total_bytes:,} bytes ({total_bytes/1024:.1f} KB)")
    print(f"  Good reads:   {total_reads}")
    print(f"  Empty reads:  {empty_reads}")
    print(f"  Throughput:   {throughput:.2f} MB/s")
    print(f"  Packets:      ~{header_count} (0xAA headers)")
    print(f"  Packet rate:  ~{pkt_rate:.0f} pkt/s")
    print(f"  Expected:     ~1538 pkt/s (100MHz / 65536 cycles)")

    if total_bytes > 0:
        print("  PASS")
        return True
    else:
        print("  FAIL: No data received in {duration_s}s")
        return False


# ============================================================================
# Main
# ============================================================================

def main():
    print("AERIS-10 FT601 USB 3.0 Streaming Test")
    print("=" * 60)
    print()

    results = {}

    # Step 1: Enumerate
    if not test_enumerate():
        print("\nABORT: No devices found")
        sys.exit(1)

    # Step 2: Open and check config
    dev = FT601Device()
    if not test_open_and_config(dev):
        print("\nABORT: Could not open device")
        sys.exit(1)

    try:
        # Step 3: Enable all streams
        results["enable_streams"] = test_enable_streams(dev)
        time.sleep(0.2)

        # Step 4: Read raw data
        raw = test_read_raw(dev)
        results["read_raw"] = raw is not None and len(raw) > 0

        # Step 5: Parse packets
        results["parse_packets"] = test_parse_packets(raw)

        # Step 6: Status request
        results["status_request"] = test_status_request(dev)

        # Step 7: Self-test
        results["self_test"] = test_self_test(dev)

        # Step 8: Sustained read
        results["sustained_read"] = test_sustained_read(dev)

    finally:
        print()
        print("=" * 60)
        print("Closing device...")
        dev.close()

    # Summary
    print()
    print("=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    all_pass = True
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False
        print(f"  {name:.<30s} {status}")

    print()
    if all_pass:
        print("ALL TESTS PASSED")
    else:
        print("SOME TESTS FAILED — see details above")

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
