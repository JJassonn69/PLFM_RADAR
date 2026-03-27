#!/usr/bin/env python3
"""
Rapid burst-read diagnostic: enables range stream, then reads rapidly in a loop.
"""
import sys
import time
import ctypes

try:
    import ftd3xx
    import ftd3xx._ftd3xx_linux as _ll
except ImportError:
    print("ERROR: ftd3xx not installed")
    sys.exit(1)

import struct

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

def build_cmd(opcode, value, addr=0):
    word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
    return struct.pack(">I", word)

def hexdump(data, max_bytes=256):
    for i in range(0, min(len(data), max_bytes), 16):
        chunk = data[i:i+16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        print(f"    {i:04X}: {hex_str}")

def main():
    dev = ftd3xx.create(0)
    if dev is None:
        print("Cannot open FT601")
        sys.exit(1)
    handle = dev.handle
    print("FT601 opened")

    # Disable + flush
    raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0000))
    time.sleep(0.5)
    for _ in range(20):
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=100)
        if not d:
            break
    try:
        dev.flushPipe(PIPE_IN)
    except:
        pass
    time.sleep(0.2)

    # Enable range only
    print("\nEnabling range-only stream...")
    raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0001))

    # Rapid burst reads for 1 second
    print("Reading rapidly for 1 second...\n")
    all_data = bytearray()
    read_sizes = []
    t_start = time.time()
    while time.time() - t_start < 1.0:
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=100)
        if d:
            read_sizes.append(len(d))
            all_data.extend(d)

    print(f"Total: {len(all_data)} bytes in {len(read_sizes)} reads")
    print(f"Read size distribution:")
    from collections import Counter
    for size, count in sorted(Counter(read_sizes).items()):
        print(f"  {size} bytes: {count} reads")

    print(f"\nFirst 512 bytes:")
    hexdump(bytes(all_data), 512)

    # Count markers
    aa = sum(1 for b in all_data if b == 0xAA)
    bb = sum(1 for b in all_data if b == 0xBB)
    x55 = sum(1 for b in all_data if b == 0x55)
    print(f"\n0xAA count: {aa}")
    print(f"0xBB count: {bb}")
    print(f"0x55 count: {x55}")

    # Find AA positions
    positions = [i for i in range(len(all_data)) if all_data[i] == 0xAA]
    if positions:
        print(f"0xAA positions (first 30): {positions[:30]}")
        # Compute deltas
        if len(positions) > 1:
            deltas = [positions[i+1] - positions[i] for i in range(min(len(positions)-1, 30))]
            print(f"0xAA deltas: {deltas}")
    
    # Find 0x55 positions
    pos55 = [i for i in range(len(all_data)) if all_data[i] == 0x55]
    if pos55:
        print(f"0x55 positions (first 30): {pos55[:30]}")

    dev.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
