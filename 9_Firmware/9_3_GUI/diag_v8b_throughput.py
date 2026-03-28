#!/usr/bin/env python3
"""v8b throughput diagnostic - verify FIFO+handshake delivers ~2048 packets"""
import sys
import time
import ctypes
import struct

try:
    import ftd3xx
    import ftd3xx._ftd3xx_linux as _ll
except ImportError:
    print("ERROR: ftd3xx not installed")
    sys.exit(1)

PIPE_OUT = 0x02   # Host -> FPGA (commands)
PIPE_IN = 0x82    # FPGA -> Host (data/status)


def raw_write(handle, pipe, data, timeout_ms=1000):
    """Write raw bytes to pipe."""
    buf = ctypes.create_string_buffer(data, len(data))
    written = ctypes.c_ulong(0)
    status = _ll.FT_WritePipeEx(handle, ctypes.c_ubyte(pipe), buf, len(data), ctypes.byref(written), timeout_ms)
    if status != 0:
        raise RuntimeError(f"WritePipe failed: status={status}")
    return written.value


def raw_read(handle, pipe, size, timeout_ms=2000):
    """Read raw bytes from pipe."""
    buf = ctypes.create_string_buffer(size)
    read = ctypes.c_ulong(0)
    status = _ll.FT_ReadPipeEx(handle, ctypes.c_ubyte(pipe), buf, size, ctypes.byref(read), timeout_ms)
    if status != 0 and status != 6:
        raise RuntimeError(f"ReadPipe failed: status={status}")
    return buf.raw[:read.value]


def flush_pipe(handle, count=10):
    """Flush residual data from IN pipe."""
    for _ in range(count):
        try:
            d = raw_read(handle, PIPE_IN, 16384, timeout_ms=50)
            if not d:
                break
        except:
            break
    time.sleep(0.05)


def build_cmd(opcode, value=0):
    """Build 4-byte command packet (big-endian matching radar_protocol.py)."""
    return struct.pack(">I", (opcode << 24) | (0 << 16) | value)


def parse_data_packets(raw):
    """Parse raw data into individual 24-byte packets, return stats."""
    AA = 0xAA
    FF = 0x55
    packet_size = 24  # 6 words * 4 bytes
    num_packets = len(raw) // packet_size

    valid = 0
    invalid = 0
    frame_errors = 0
    first_range = None
    last_range = None
    range_values = []

    for i in range(num_packets):
        pkt = raw[i*packet_size:(i+1)*packet_size]
        hdr = pkt[0]
        ftr = pkt[20]
        rng = struct.unpack_from("<I", pkt, 4)[0]

        if hdr == AA and ftr == FF:
            valid += 1
            if first_range is None:
                first_range = rng
            last_range = rng
            range_values.append(rng)
        else:
            invalid += 1
            if hdr != AA:
                frame_errors += 1

    return {
        "total": num_packets,
        "valid": valid,
        "invalid": invalid,
        "frame_errors": frame_errors,
        "first_range": first_range,
        "last_range": last_range,
        "range_values": range_values,
    }


def main():
    print("=" * 60)
    print("v8b Throughput Diagnostic - FIFO + Handshake FSM")
    print("=" * 60)

    # Open FT601
    print("\nOpening FT601 device...")
    dev = ftd3xx.create(0)
    if dev is None:
        print("FT_Create failed!")
        sys.exit(1)
    speed = dev.getDeviceSpeed()
    print(f"Device speed: {speed} (3=SuperSpeed)")
    if speed != 3:
        dev.close()
        print("Not SuperSpeed!")
        sys.exit(1)

    handle = dev.handle
    print("FT601 SuperSpeed opened successfully")

    # Flush any stale data
    print("\nFlushing residual data...")
    flush_pipe(handle, 20)

    # 1. Send stream_control = 0x01 (enable range-only)
    print("\nSending stream_control = 0x01...")
    raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x01))

    # 2. Send trigger_pulse = 0x01 (start BRAM playback)
    print("Sending trigger_pulse = 0x01...")
    raw_write(handle, PIPE_OUT, build_cmd(0x02, 0x01))

    # 3. Read data for ~3 seconds
    print("\nReading data for 3 seconds...")
    all_data = bytearray()
    t_start = time.time()
    read_count = 0
    empty_reads = 0
    read_errors = 0

    while (time.time() - t_start) < 3.0:
        try:
            chunk = raw_read(handle, PIPE_IN, 65536, timeout_ms=100)
            if chunk and len(chunk) > 0:
                all_data.extend(chunk)
                read_count += 1
            else:
                empty_reads += 1
                time.sleep(0.001)
        except Exception as e:
            read_errors += 1
            if read_errors > 100:
                print(f"  Too many read errors: {e}")
                break

    elapsed = time.time() - t_start
    print(f"Read complete: {len(all_data)} bytes in {elapsed:.2f}s")
    print(f"  Reads: {read_count}, Empty: {empty_reads}, Errors: {read_errors}")
    if elapsed > 0:
        print(f"  Throughput: {len(all_data) / elapsed / 1e6:.2f} MB/s")

    # 4. Parse packets
    print("\nParsing data packets...")
    stats = parse_data_packets(all_data)
    print(f"  Total packets:  {stats['total']}")
    print(f"  Valid (AA/55):  {stats['valid']}")
    print(f"  Invalid:        {stats['invalid']}")
    print(f"  Frame errors:   {stats['frame_errors']}")

    if stats['first_range'] is not None:
        print(f"  First range:    0x{stats['first_range']:08X}")
        print(f"  Last range:     0x{stats['last_range']:08X}")
        print(f"  Unique values:  {len(set(stats['range_values']))}")

    # 5. Send stop
    print("\nSending stream_control = 0x00 (stop)...")
    raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x00))

    # 6. Read status to verify pkt_starts
    print("\nReading status packet...")
    raw_write(handle, PIPE_OUT, build_cmd(0xFF, 0x00))
    time.sleep(0.1)
    try:
        status_data = raw_read(handle, PIPE_IN, 65536, timeout_ms=2000)
        print(f"  Status bytes: {len(status_data)}")
        status_stats = parse_data_packets(status_data)
        print(f"  Status packets: {status_stats['valid']}")
        if status_stats['valid'] > 0 and status_stats['first_range'] is not None:
            word0 = status_stats['first_range']
            print(f"  Status word0: 0x{word0:08X}")
    except Exception as e:
        print(f"  Status read error: {e}")

    dev.close()

    # 7. Pass/Fail verdict
    print("\n" + "=" * 60)
    expected = 32 * 64  # 2048
    if stats['valid'] >= expected - 10:
        print(f"PASS: {stats['valid']} valid packets (expected ~{expected})")
        sys.exit(0)
    elif stats['valid'] > 2:
        print(f"PARTIAL: {stats['valid']} valid packets (expected ~{expected})")
        print("  FIFO improved delivery but not all packets came through")
        sys.exit(1)
    else:
        print(f"FAIL: only {stats['valid']} valid packets (expected ~{expected})")
        sys.exit(2)


if __name__ == "__main__":
    main()
