#!/usr/bin/env python3
"""
v9c hardware diagnostic — Direct D3XX access (no stream server needed).
Run on remote server with: sudo /home/jason-stone/PLFM_RADAR_work/venv/bin/python3 v9b_hw_diagnostic.py

Sends commands to configure v9 full pipeline, triggers playback, counts packets.
v9c additions:
  - Reads CFAR debug counters from status_words[6] and status_words[7]
  - status_words[6] = {cfar_dbg_cells_processed[15:0], cfar_dbg_cols_completed[7:0], 8'd0}
  - status_words[7] = {cfar_detect_count[15:0], cfar_dbg_valid_count[15:0]}
"""
import ctypes
import struct
import time
import sys

import ftd3xx
import ftd3xx._ftd3xx_linux as _ll

# ── D3XX constants ──────────────────────────────────────────────────────────
PIPE_OUT = 0x02  # Host→Device
PIPE_IN  = 0x82  # FPGA → Host

# ── Helpers ─────────────────────────────────────────────────────────────────
def build_cmd(opcode, value):
    """Build big-endian command word: {opcode[31:24], addr[23:16], value[15:0]}"""
    return struct.pack(">I", (opcode << 24) | (opcode << 16) | value)

def init_pipes(handle):
    """Initialize FT601 pipes — use try/except for functions that may not be in Python wrapper."""
    zero = ctypes.c_ubyte(0)
    try:
        _ll.FT_AbortPipe(handle, ctypes.c_ubyte(PIPE_IN))
    except (AttributeError, Exception) as e:
        print(f"  FT_AbortPipe(IN): {e}")
    try:
        _ll.FT_AbortPipe(handle, ctypes.c_ubyte(PIPE_OUT))
    except (AttributeError, Exception) as e:
        print(f"  FT_AbortPipe(OUT): {e}")
    try:
        _ll.FT_FlushPipe(handle, ctypes.c_ubyte(PIPE_IN))
    except (AttributeError, Exception) as e:
        print(f"  FT_FlushPipe: {e}")
    try:
        _ll.FT_ClearStreamPipe(handle, zero, zero, ctypes.c_ubyte(PIPE_IN))
    except (AttributeError, Exception) as e:
        print(f"  FT_ClearStreamPipe(IN): {e}")
    try:
        _ll.FT_ClearStreamPipe(handle, zero, zero, ctypes.c_ubyte(PIPE_OUT))
    except (AttributeError, Exception) as e:
        print(f"  FT_ClearStreamPipe(OUT): {e}")
    try:
        _ll.FT_SetStreamPipe(handle, zero, zero, ctypes.c_ubyte(PIPE_IN), ctypes.c_ulong(65536))
    except (AttributeError, Exception) as e:
        print(f"  FT_SetStreamPipe(IN): {e}")
    try:
        _ll.FT_SetStreamPipe(handle, zero, zero, ctypes.c_ubyte(PIPE_OUT), ctypes.c_ulong(4))
    except (AttributeError, Exception) as e:
        print(f"  FT_SetStreamPipe(OUT): {e}")

def write_cmd(handle, data):
    buf = ctypes.create_string_buffer(data, len(data))
    xfer = ctypes.c_ulong(0)
    st = _ll.FT_WritePipeEx(
        handle, ctypes.c_ubyte(PIPE_OUT),
        buf, ctypes.c_ulong(len(data)),
        ctypes.byref(xfer), ctypes.c_ulong(1000))
    return st, xfer.value

def read_data(handle, size, timeout_ms=2000):
    buf = ctypes.create_string_buffer(size)
    xfer = ctypes.c_ulong(0)
    st = _ll.FT_ReadPipeEx(
        handle, ctypes.c_ubyte(PIPE_IN),
        buf, ctypes.c_ulong(size),
        ctypes.byref(xfer), ctypes.c_ulong(timeout_ms))
    return buf.raw[:xfer.value], st

# ── Main ────────────────────────────────────────────────────────────────────
def main():
    print("v9c Hardware Diagnostic")
    print("=" * 50)

    # Open device via ftd3xx package
    dev = ftd3xx.create(0)
    if dev is None:
        print("FAIL: ftd3xx.create returned None")
        return 1
    handle = dev.handle
    print(f"FT601 opened (handle={handle})")

    # Init pipes
    print("Initializing pipes...")
    init_pipes(handle)
    print("Pipes initialized")

    # Drain any stale data
    stale = 0
    for _ in range(50):
        data, s = read_data(handle, 65536, timeout_ms=100)
        if len(data) == 0:
            break
        stale += len(data)
    if stale:
        print(f"Drained {stale} stale bytes")
    else:
        print("No stale data")

    # Send configuration commands (big-endian)
    # Opcode 0x21 = CFAR_GUARD, value 2
    st, n = write_cmd(handle, build_cmd(0x21, 2))
    print(f"  CFAR_GUARD=2: st={st}, wrote={n}")
    time.sleep(0.05)

    # Opcode 0x22 = CFAR_TRAIN, value 8
    st, n = write_cmd(handle, build_cmd(0x22, 8))
    print(f"  CFAR_TRAIN=8: st={st}, wrote={n}")
    time.sleep(0.05)

    # Opcode 0x23 = CFAR_ALPHA, value 0x30 (3.0 in Q4.4)
    st, n = write_cmd(handle, build_cmd(0x23, 0x30))
    print(f"  CFAR_ALPHA=0x30 (3.0): st={st}, wrote={n}")
    time.sleep(0.05)

    # Opcode 0x24 = CFAR_MODE, value 0 (CA-CFAR)
    st, n = write_cmd(handle, build_cmd(0x24, 0))
    print(f"  CFAR_MODE=0 (CA): st={st}, wrote={n}")
    time.sleep(0.05)

    # Opcode 0x26 = MTI_ENABLE, value 0 (off for clean test)
    st, n = write_cmd(handle, build_cmd(0x26, 0))
    print(f"  MTI_ENABLE=0: st={st}, wrote={n}")
    time.sleep(0.05)

    # Opcode 0x27 = DC_NOTCH, value 0 (off for clean test)
    st, n = write_cmd(handle, build_cmd(0x27, 0))
    print(f"  DC_NOTCH=0: st={st}, wrote={n}")
    time.sleep(0.05)

    # Opcode 0x25 = CFAR_ENABLE, value 1
    st, n = write_cmd(handle, build_cmd(0x25, 1))
    print(f"  CFAR_ENABLE=1: st={st}, wrote={n}")
    time.sleep(0.05)

    # Opcode 0x04 = STREAM_CONTROL, value 0x07 (range+doppler+cfar)
    st, n = write_cmd(handle, build_cmd(0x04, 0x07))
    print(f"  STREAM_CONTROL=0x07: st={st}, wrote={n}")
    time.sleep(0.05)

    # Request status (opcode 0xFF)
    st, n = write_cmd(handle, build_cmd(0xFF, 0))
    print(f"  STATUS_REQUEST: st={st}, wrote={n}")
    time.sleep(0.5)

    # Read status response
    data, s = read_data(handle, 65536)
    if data:
        print(f"  Status response: {len(data)} bytes, first 8: {data[:8].hex()}")
    else:
        print(f"  No status response (st={s})")

    # Trigger playback (opcode 0x02)
    st, n = write_cmd(handle, build_cmd(0x02, 1))
    print(f"\nTriggered playback: st={st}, wrote={n}")
    print("Collecting packets...")

    # Collect ALL data
    all_data = bytearray()
    start = time.time()
    timeout = 30.0
    no_data_count = 0
    max_no_data = 200  # 200 * 50ms = 10s of silence → stop

    while time.time() - start < timeout:
        data, s = read_data(handle, 65536)
        if len(data) > 0:
            all_data.extend(data)
            no_data_count = 0
        else:
            no_data_count += 1
            if no_data_count > max_no_data:
                print(f"  No data for {max_no_data * 0.05:.1f}s, stopping")
                break
            time.sleep(0.05)

    elapsed = time.time() - start

    # ── Request status after playback to get CFAR debug counters ──────────
    print(f"\nRequesting post-playback status...")
    time.sleep(0.5)  # Let CFAR processing finish
    st, n = write_cmd(handle, build_cmd(0xFF, 0))
    print(f"  STATUS_REQUEST: st={st}, wrote={n}")
    time.sleep(0.5)

    status_data, s = read_data(handle, 65536)
    if status_data and len(status_data) >= 40:
        print(f"  Status response: {len(status_data)} bytes")
        # Parse status packet: 10 words × 4 bytes
        # Word 0: 0xBB header
        # Words 1-8: status_words[0..7]
        # Word 9: 0x55 footer
        for idx in range(0, min(len(status_data) - 3, 40), 4):
            w = struct.unpack_from("<I", status_data, idx)[0]
            print(f"    status[{idx//4}] = 0x{w:08X}")

        # Parse CFAR debug counters from status_words[6] (word index 7 in packet)
        # and status_words[7] (word index 8 in packet)
        if len(status_data) >= 36:
            w6 = struct.unpack_from("<I", status_data, 28)[0]  # status_words[6]
            w7 = struct.unpack_from("<I", status_data, 32)[0]  # status_words[7]

            cfar_cells = (w6 >> 16) & 0xFFFF
            cfar_cols  = (w6 >> 8)  & 0xFF
            cfar_detections = (w7 >> 16) & 0xFFFF
            cfar_valid_pulses = w7 & 0xFFFF

            print(f"\n  === CFAR Debug Counters (v9c) ===")
            print(f"    Cells processed:  {cfar_cells} (expected: 2048 = 64 range × 32 doppler)")
            print(f"    Columns completed: {cfar_cols} (expected: 32)")
            print(f"    Detect count:     {cfar_detections}")
            print(f"    Valid pulses:     {cfar_valid_pulses} (expected: 2048)")
    else:
        print(f"  No/short status response (len={len(status_data) if status_data else 0})")

    # Close device
    dev.close()

    # ── Analyze ──────────────────────────────────────────────────────────
    print(f"\n{'=' * 50}")
    print(f"  Raw Data Analysis ({elapsed:.1f}s)")
    print(f"{'=' * 50}")
    print(f"  Total bytes: {len(all_data):,}")

    if len(all_data) == 0:
        print("  NO DATA RECEIVED — check FPGA and USB connection")
        return 1

    # Scan for headers/footers by 4-byte aligned words
    range_count = 0
    doppler_count = 0
    cfar_count = 0
    status_count = 0
    footer_count = 0

    for i in range(0, len(all_data) - 3, 4):
        word = struct.unpack_from("<I", all_data, i)[0]
        if word == 0x000000AA:
            range_count += 1
        elif word == 0x000000CC:
            doppler_count += 1
        elif word == 0x000000DD:
            cfar_count += 1
        elif word == 0x000000BB:
            status_count += 1
        elif word == 0x00000055:
            footer_count += 1

    print(f"  Range   (0xAA) headers: {range_count}")
    print(f"  Doppler (0xCC) headers: {doppler_count}")
    print(f"  CFAR    (0xDD) headers: {cfar_count}")
    print(f"  Status  (0xBB) headers: {status_count}")
    print(f"  Footer  (0x55) count:   {footer_count}")
    print(f"{'=' * 50}")

    # Expected byte counts
    expected_range_bytes = range_count * 24
    expected_doppler_bytes = doppler_count * 16
    expected_cfar_bytes = cfar_count * 16
    expected_status_bytes = status_count * 40
    expected_total = expected_range_bytes + expected_doppler_bytes + expected_cfar_bytes + expected_status_bytes
    print(f"  Expected bytes: {expected_total:,} "
          f"(range={expected_range_bytes}, doppler={expected_doppler_bytes}, "
          f"cfar={expected_cfar_bytes}, status={expected_status_bytes})")

    # Validate
    ok = True
    if range_count < 2048:
        print(f"  WARNING: Expected 2048 range, got {range_count}")
        ok = False
    if doppler_count < 1000:
        print(f"  WARNING: Expected 1000+ Doppler, got {doppler_count}")
        ok = False
    if cfar_count < 100:
        print(f"  FAIL: Expected 100+ CFAR, got {cfar_count}")
        ok = False
    else:
        print(f"  CFAR count looks good: {cfar_count}")

    if ok:
        print(f"\n  PASS: v9c full pipeline working!")
    else:
        print(f"\n  PARTIAL: Some packet types missing")

    # Dump first few CFAR packets for inspection
    if cfar_count > 0:
        print(f"\nFirst {min(10, cfar_count)} CFAR packets:")
        cfar_found = 0
        for i in range(0, len(all_data) - 15, 4):
            word = struct.unpack_from("<I", all_data, i)[0]
            if word == 0x000000DD and cfar_found < 10:
                w1 = struct.unpack_from("<I", all_data, i+4)[0]
                w2 = struct.unpack_from("<I", all_data, i+8)[0]
                w3 = struct.unpack_from("<I", all_data, i+12)[0]
                # Parse: w1 = {flag, range[5:0], doppler[4:0], 3'b000, magnitude[16:0]}
                flag = (w1 >> 31) & 1
                rbin = (w1 >> 25) & 0x3F
                dbin = (w1 >> 20) & 0x1F
                mag  = w1 & 0x1FFFF
                # w2 = {15'b0, threshold[16:0]}
                thresh = w2 & 0x1FFFF
                print(f"  CFAR[{cfar_found}]: flag={flag} range={rbin} doppler={dbin} "
                      f"mag={mag} thresh={thresh} footer=0x{w3:08X}")
                cfar_found += 1

    # Save raw data for offline analysis
    outfile = "/tmp/v9c_capture.bin"
    with open(outfile, "wb") as f:
        f.write(all_data)
    print(f"\nRaw capture saved to {outfile}")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
