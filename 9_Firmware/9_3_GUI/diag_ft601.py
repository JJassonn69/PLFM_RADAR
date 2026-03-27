#!/usr/bin/env python3
"""
Quick FT601 diagnostic: flush, enable stream, read, hex dump.
Focuses on raw data inspection to debug packet framing.
"""
import sys
import time
import struct
import ctypes

try:
    import ftd3xx
    import ftd3xx._ftd3xx_linux as _ll
except ImportError:
    print("ERROR: ftd3xx not installed")
    sys.exit(1)

PIPE_OUT = 0x02
PIPE_IN  = 0x82

def raw_write(handle, pipe, data, timeout_ms=1000):
    buf = ctypes.create_string_buffer(data, len(data))
    xfer = ctypes.c_ulong(0)
    status = _ll.FT_WritePipeEx(handle, ctypes.c_ubyte(pipe),
                                buf, ctypes.c_ulong(len(data)),
                                ctypes.byref(xfer), ctypes.c_ulong(timeout_ms))
    return xfer.value if status == 0 else -status

def raw_read(handle, pipe, size, timeout_ms=2000):
    buf = ctypes.create_string_buffer(size)
    xfer = ctypes.c_ulong(0)
    status = _ll.FT_ReadPipeEx(handle, ctypes.c_ubyte(pipe),
                               buf, ctypes.c_ulong(size),
                               ctypes.byref(xfer), ctypes.c_ulong(timeout_ms))
    return buf.raw[:xfer.value] if status == 0 else b""

def hexdump(data, max_bytes=512):
    for i in range(0, min(len(data), max_bytes), 16):
        chunk = data[i:i+16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"  {i:04X}: {hex_str:<48s}  {ascii_str}")
    if len(data) > max_bytes:
        print(f"  ... ({len(data)} bytes total)")

def build_cmd(opcode, value, addr=0):
    word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
    return struct.pack(">I", word)

def main():
    dev = ftd3xx.create(0)
    if dev is None:
        print("Cannot open FT601")
        sys.exit(1)
    handle = dev.handle
    print("FT601 opened")

    # Step 1: Flush any stale data
    print("\n=== FLUSH ===")
    for i in range(5):
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=100)
        if len(d) > 0:
            print(f"  Flushed {len(d)} bytes")
        else:
            break
    try:
        dev.flushPipe(PIPE_IN)
    except:
        pass
    time.sleep(0.1)

    # Step 2: Disable all streams first
    print("\n=== DISABLE STREAMS ===")
    n = raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0000))
    print(f"  Wrote {n} bytes (stream_control=0x0000)")
    time.sleep(0.5)

    # Step 3: Flush again after disable
    print("\n=== FLUSH AFTER DISABLE ===")
    for i in range(5):
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=100)
        if len(d) > 0:
            print(f"  Flushed {len(d)} bytes")
        else:
            break
    try:
        dev.flushPipe(PIPE_IN)
    except:
        pass
    time.sleep(0.1)

    # Step 4: Read when streams are OFF — should get nothing
    print("\n=== READ WITH STREAMS OFF ===")
    d = raw_read(handle, PIPE_IN, 4096, timeout_ms=500)
    print(f"  Got {len(d)} bytes (expect 0)")
    if len(d) > 0:
        hexdump(d, 64)

    # Step 5: Enable range-only stream
    print("\n=== ENABLE RANGE STREAM ONLY ===")
    n = raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0001))
    print(f"  Wrote {n} bytes (stream_control=0x0001)")

    # Wait just enough for first packet (65536 cycles = 0.66ms at 100MHz)
    time.sleep(0.01)

    # Step 6: Read data
    print("\n=== READ WITH RANGE STREAM ===")
    d = raw_read(handle, PIPE_IN, 4096, timeout_ms=2000)
    print(f"  Got {len(d)} bytes")
    if len(d) > 0:
        hexdump(d, 256)
        # Count markers
        aa = sum(1 for b in d if b == 0xAA)
        bb = sum(1 for b in d if b == 0xBB)
        ff = sum(1 for b in d if b == 0x55)
        print(f"\n  0xAA count: {aa}")
        print(f"  0xBB count: {bb}")
        print(f"  0x55 count: {ff}")

        # Find AA positions
        positions = [i for i in range(len(d)) if d[i] == 0xAA]
        if positions:
            print(f"  0xAA positions: {positions[:20]}")
            # Show bytes around first AA
            for pos in positions[:3]:
                start = max(0, pos - 2)
                end = min(len(d), pos + 24)
                print(f"  Context around offset {pos}:")
                hexdump(d[start:end], 32)

    # Step 7: Enable ALL streams
    print("\n=== ENABLE ALL STREAMS ===")
    n = raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0007))
    print(f"  Wrote {n} bytes (stream_control=0x0007)")
    time.sleep(0.01)

    print("\n=== READ WITH ALL STREAMS ===")
    d = raw_read(handle, PIPE_IN, 8192, timeout_ms=2000)
    print(f"  Got {len(d)} bytes")
    if len(d) > 0:
        hexdump(d, 512)
        aa = sum(1 for b in d if b == 0xAA)
        print(f"\n  0xAA count: {aa}")
        positions = [i for i in range(len(d)) if d[i] == 0xAA]
        if positions:
            print(f"  0xAA positions: {positions[:20]}")

    # Step 8: Status request test
    print("\n=== STATUS REQUEST ===")
    # Flush first
    for i in range(5):
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=100)
        if len(d) == 0:
            break
    time.sleep(0.1)

    # Disable streams to stop data flow
    n = raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0000))
    print(f"  Disabled streams ({n} bytes)")
    time.sleep(0.5)

    # Flush again
    for i in range(5):
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=100)
        if len(d) == 0:
            break
    try:
        dev.flushPipe(PIPE_IN)
    except:
        pass
    time.sleep(0.1)

    # Send status request
    n = raw_write(handle, PIPE_OUT, build_cmd(0xFF, 0x0000))
    print(f"  Sent status request ({n} bytes)")
    time.sleep(0.2)

    d = raw_read(handle, PIPE_IN, 4096, timeout_ms=2000)
    print(f"  Got {len(d)} bytes")
    if len(d) > 0:
        hexdump(d, 256)
        bb_positions = [i for i in range(len(d)) if d[i] == 0xBB]
        print(f"  0xBB positions: {bb_positions}")
    else:
        print("  No data received!")

    dev.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
