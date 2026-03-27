#!/usr/bin/env python3
"""
v7b Diagnostic: Debug instrumentation readback + streaming analysis.

Steps:
  1. Open FT601, flush everything
  2. Enable range-only streaming for 2 seconds
  3. Analyze streaming data (sizes, framing, patterns)
  4. Disable streaming, flush
  5. Send status request (0xFF)
  6. Read and parse expanded status packet (10 x 4-byte = 40 bytes)
     - Word 0: BB header marker
     - Words 1-6: Config registers
     - Word 7: {dbg_wr_strobes[15:0], dbg_txe_blocks[15:0]}
     - Word 8: {dbg_pkt_starts[15:0], dbg_pkt_completions[15:0]}
     - Word 9: 0x55 footer
  7. Print debug counter analysis
"""
import sys
import time
import ctypes
import struct
from collections import Counter

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

def build_cmd(opcode, value, addr=0):
    word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
    return struct.pack(">I", word)

def hexdump(data, max_bytes=256):
    for i in range(0, min(len(data), max_bytes), 16):
        chunk = data[i:i+16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        print(f"    {i:04X}: {hex_str}")

def flush_pipe(handle, dev):
    """Aggressively flush the IN pipe."""
    for _ in range(50):
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=50)
        if not d:
            break
    try:
        dev.flushPipe(PIPE_IN)
    except:
        pass

def parse_status_packet(data):
    """Parse expanded v7b status packet (10 x 4 bytes = 40 bytes).
    
    In v7b, all transfers use BE=1111, so each transfer is a full 32-bit word.
    FT601 little-endian byte lane mapping means:
      USB word DATA[31:0] arrives as bytes [D[7:0], D[15:8], D[23:16], D[31:24]]
    """
    if len(data) < 4:
        print(f"  Status packet too short: {len(data)} bytes")
        return None
    
    print(f"  Raw status data ({len(data)} bytes):")
    hexdump(data, 128)
    
    # Parse 32-bit words from little-endian byte stream
    words = []
    for i in range(0, len(data) - 3, 4):
        # FT601 byte lanes: DATA[7:0]=byte0, DATA[15:8]=byte1, etc.
        w = struct.unpack_from("<I", data, i)[0]
        words.append(w)
    
    print(f"\n  Parsed {len(words)} 32-bit words:")
    for i, w in enumerate(words):
        print(f"    Word {i}: 0x{w:08X}")
    
    if not words:
        return None
    
    # Look for BB header marker
    result = {}
    bb_idx = None
    for i, w in enumerate(words):
        if (w & 0xFF) == 0xBB or w == 0x000000BB:
            bb_idx = i
            break
    
    if bb_idx is None:
        print("  WARNING: No BB header marker found in status response")
        return None
    
    print(f"\n  BB header found at word index {bb_idx}")
    
    # Expected layout after BB: 8 data words + footer = 9 more words
    remaining = words[bb_idx:]
    if len(remaining) < 10:
        print(f"  WARNING: Only {len(remaining)} words after BB header (expected 10)")
        # Try to parse what we have
    
    # Parse config words
    if len(remaining) > 1:
        w0 = remaining[1]
        mode = (w0 >> 21) & 0x03
        stream = (w0 >> 16) & 0x07
        threshold = w0 & 0xFFFF
        result['mode'] = mode
        result['stream_ctrl'] = stream
        result['threshold'] = threshold
        print(f"  Config Word 0: mode={mode}, stream_ctrl=0b{stream:03b}, threshold={threshold}")
    
    if len(remaining) > 2:
        w1 = remaining[2]
        long_chirp = (w1 >> 16) & 0xFFFF
        long_listen = w1 & 0xFFFF
        print(f"  Config Word 1: long_chirp={long_chirp}, long_listen={long_listen}")
    
    if len(remaining) > 3:
        w2 = remaining[3]
        guard = (w2 >> 16) & 0xFFFF
        short_chirp = w2 & 0xFFFF
        print(f"  Config Word 2: guard={guard}, short_chirp={short_chirp}")
    
    if len(remaining) > 4:
        w3 = remaining[4]
        short_listen = (w3 >> 16) & 0xFFFF
        chirps_per_elev = w3 & 0x3F
        print(f"  Config Word 3: short_listen={short_listen}, chirps_per_elev={chirps_per_elev}")
    
    if len(remaining) > 5:
        w4 = remaining[5]
        range_mode = w4 & 0x03
        print(f"  Config Word 4: range_mode={range_mode}")
    
    if len(remaining) > 6:
        w5 = remaining[6]
        self_test_busy = (w5 >> 24) & 0x01
        self_test_detail = (w5 >> 8) & 0xFF
        self_test_flags = w5 & 0x1F
        print(f"  Config Word 5: self_test: busy={self_test_busy}, detail=0x{self_test_detail:02X}, flags=0b{self_test_flags:05b}")
    
    # DEBUG WORDS (v7b)
    if len(remaining) > 7:
        w6 = remaining[7]
        wr_strobes = (w6 >> 16) & 0xFFFF
        txe_blocks = w6 & 0xFFFF
        result['wr_strobes'] = wr_strobes
        result['txe_blocks'] = txe_blocks
        print(f"\n  === DEBUG WORD 6 ===")
        print(f"    dbg_wr_strobes  = {wr_strobes}")
        print(f"    dbg_txe_blocks  = {txe_blocks}")
    
    if len(remaining) > 8:
        w7 = remaining[8]
        pkt_starts = (w7 >> 16) & 0xFFFF
        pkt_completions = w7 & 0xFFFF
        result['pkt_starts'] = pkt_starts
        result['pkt_completions'] = pkt_completions
        print(f"  === DEBUG WORD 7 ===")
        print(f"    dbg_pkt_starts      = {pkt_starts}")
        print(f"    dbg_pkt_completions = {pkt_completions}")
    
    if len(remaining) > 9:
        footer_word = remaining[9]
        footer_byte = footer_word & 0xFF
        print(f"\n  Footer word: 0x{footer_word:08X} (byte=0x{footer_byte:02X}, expected 0x55)")
    
    return result


def main():
    dev = ftd3xx.create(0)
    if dev is None:
        print("Cannot open FT601")
        sys.exit(1)
    handle = dev.handle
    print("FT601 opened")

    # ===== STEP 1: Disable + flush =====
    print("\n=== Step 1: Disable streaming + flush ===")
    raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0000))
    time.sleep(0.5)
    flush_pipe(handle, dev)
    time.sleep(0.2)
    print("  Done.")

    # ===== STEP 2: Enable range-only streaming =====
    print("\n=== Step 2: Enable range-only streaming ===")
    raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0001))
    
    # ===== STEP 3: Burst read for 2 seconds =====
    print("\n=== Step 3: Burst read for 2 seconds ===")
    all_data = bytearray()
    read_sizes = []
    t_start = time.time()
    while time.time() - t_start < 2.0:
        d = raw_read(handle, PIPE_IN, 16384, timeout_ms=100)
        if d:
            read_sizes.append(len(d))
            all_data.extend(d)

    elapsed = time.time() - t_start
    print(f"  Total: {len(all_data)} bytes in {len(read_sizes)} reads ({elapsed:.2f}s)")
    if read_sizes:
        print(f"  Read size distribution:")
        for size, count in sorted(Counter(read_sizes).items()):
            print(f"    {size} bytes: {count} reads")
    
    if all_data:
        print(f"\n  First 256 bytes:")
        hexdump(bytes(all_data), 256)
        
        # Marker analysis
        aa_positions = [i for i in range(len(all_data)) if all_data[i] == 0xAA]
        x55_positions = [i for i in range(len(all_data)) if all_data[i] == 0x55]
        print(f"\n  0xAA header count: {len(aa_positions)}")
        print(f"  0x55 footer count: {len(x55_positions)}")
        if aa_positions:
            print(f"  0xAA first 20 positions: {aa_positions[:20]}")
            if len(aa_positions) > 1:
                deltas = [aa_positions[i+1] - aa_positions[i] for i in range(min(len(aa_positions)-1, 20))]
                print(f"  0xAA deltas: {deltas}")

    # ===== STEP 4: Disable streaming + flush =====
    print("\n=== Step 4: Disable streaming + flush ===")
    raw_write(handle, PIPE_OUT, build_cmd(0x04, 0x0000))
    time.sleep(0.5)
    flush_pipe(handle, dev)
    time.sleep(0.3)
    print("  Done.")

    # Verify flush is complete (should get 0 bytes)
    d = raw_read(handle, PIPE_IN, 16384, timeout_ms=200)
    print(f"  Post-flush read: {len(d)} bytes (expect 0)")

    # ===== STEP 5: Send status request =====
    print("\n=== Step 5: Send status request (0xFF) ===")
    n = raw_write(handle, PIPE_OUT, build_cmd(0xFF, 0x0000))
    print(f"  Write returned: {n} bytes")
    time.sleep(0.1)

    # ===== STEP 6: Read status response =====
    print("\n=== Step 6: Read status response ===")
    # v7b status packet: 10 x 4 bytes = 40 bytes
    # But FT601 may return more or less depending on how it buffers
    status_data = raw_read(handle, PIPE_IN, 4096, timeout_ms=2000)
    print(f"  Read {len(status_data)} bytes")
    
    if status_data:
        result = parse_status_packet(status_data)
        
        if result:
            # ===== STEP 7: Analysis =====
            print("\n=== Step 7: Debug Counter Analysis ===")
            wr = result.get('wr_strobes', 0)
            txe = result.get('txe_blocks', 0)
            starts = result.get('pkt_starts', 0)
            completions = result.get('pkt_completions', 0)
            
            if starts > 0:
                avg_writes_per_pkt = wr / starts
                print(f"  Avg WR_N strobes per packet start: {avg_writes_per_pkt:.1f}")
                print(f"    Expected for range-only: 6 (header + 4 range words + footer)")
                if avg_writes_per_pkt < 5:
                    print(f"    >>> FSM IS STALLING: only {avg_writes_per_pkt:.1f} writes per packet!")
                elif avg_writes_per_pkt >= 5.5:
                    print(f"    >>> Full packets are being sent (good)")
            
            if starts > 0 and completions > 0:
                completion_ratio = completions / starts
                print(f"  Packet completion ratio: {completions}/{starts} = {completion_ratio:.3f}")
                if completion_ratio < 0.9:
                    print(f"    >>> PACKETS ARE NOT COMPLETING!")
                else:
                    print(f"    >>> All packets completing (good)")
            
            if txe > 0:
                txe_ratio = txe / (txe + wr) if (txe + wr) > 0 else 0
                print(f"  TXE block ratio: {txe}/{txe + wr} = {txe_ratio:.3f}")
                print(f"    >>> FT601 IS APPLYING BACKPRESSURE ({txe} blocked cycles)")
            else:
                print(f"  TXE blocks: 0 (no FT601 backpressure)")
    else:
        print("  No status data received!")

    dev.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
