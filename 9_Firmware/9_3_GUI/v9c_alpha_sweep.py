#!/usr/bin/env python3
"""
v9c Alpha Sweep — Tests multiple CFAR alpha values on hardware to
diagnose whether the low detection count is an alpha calibration issue
vs a real threshold computation bug.

The CFAR module computes:
  threshold = (alpha_q44 * noise_sum) >> 4
For CA-CFAR with G=2, T=8 (16 total training cells):
  noise_sum ≈ 16 * avg_cell_magnitude
  threshold ≈ (alpha_q44 / 16) * noise_sum = alpha_q44 * avg_cell_magnitude

So alpha_q44=0x30 (3.0) gives threshold = 3 * noise_sum = 48 * avg (WAY too high)
   alpha_q44=0x05 (0.3125) gives threshold = 0.3125 * noise_sum = 5 * avg (reasonable)
   alpha_q44=0x08 (0.5) gives threshold = 0.5 * noise_sum = 8 * avg
   alpha_q44=0x10 (1.0) gives threshold = 1.0 * noise_sum = 16 * avg

Run on remote server with:
  sudo /home/jason-stone/PLFM_RADAR_work/venv/bin/python3 v9c_alpha_sweep.py
"""
import ctypes
import struct
import time
import sys

import ftd3xx
import ftd3xx._ftd3xx_linux as _ll

# ── D3XX constants ──────────────────────────────────────────────────────────
PIPE_OUT = 0x02
PIPE_IN  = 0x82

# ── Alpha values to test (Q4.4 format) ────────────────────────────────────
# Each tuple: (q44_value, human_readable_float, description)
ALPHA_SWEEP = [
    (0x03, "0.1875", "3x avg  — very sensitive"),
    (0x05, "0.3125", "5x avg  — standard Pfa~1e-4"),
    (0x08, "0.5000", "8x avg  — moderate"),
    (0x10, "1.0000", "16x avg — conservative"),
    (0x20, "2.0000", "32x avg — very conservative"),
    (0x30, "3.0000", "48x avg — current default (too high)"),
]

# ── Helpers ─────────────────────────────────────────────────────────────────
def build_cmd(opcode, value):
    """Build big-endian command word: {opcode[31:24], addr[23:16], value[15:0]}"""
    return struct.pack(">I", (opcode << 24) | (opcode << 16) | value)

def init_pipes(handle):
    zero = ctypes.c_ubyte(0)
    for fn_name, pipe in [("AbortPipe", PIPE_IN), ("AbortPipe", PIPE_OUT),
                          ("FlushPipe", PIPE_IN)]:
        try:
            getattr(_ll, f"FT_{fn_name}")(handle, ctypes.c_ubyte(pipe))
        except (AttributeError, Exception):
            pass
    for fn_name, pipe in [("ClearStreamPipe", PIPE_IN), ("ClearStreamPipe", PIPE_OUT)]:
        try:
            getattr(_ll, f"FT_{fn_name}")(handle, zero, zero, ctypes.c_ubyte(pipe))
        except (AttributeError, Exception):
            pass
    for pipe, sz in [(PIPE_IN, 65536), (PIPE_OUT, 4)]:
        try:
            _ll.FT_SetStreamPipe(handle, zero, zero, ctypes.c_ubyte(pipe), ctypes.c_ulong(sz))
        except (AttributeError, Exception):
            pass

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

def drain(handle, rounds=50, timeout_ms=100):
    """Drain any stale data from pipe."""
    total = 0
    for _ in range(rounds):
        data, _ = read_data(handle, 65536, timeout_ms=timeout_ms)
        if len(data) == 0:
            break
        total += len(data)
    return total

def run_one_alpha(handle, alpha_q44, alpha_str, desc, save_prefix=None):
    """Configure CFAR with given alpha, trigger playback, count detections.
    
    Returns (range_count, doppler_count, cfar_count, detect_count_from_status)
    """
    print(f"\n{'─' * 60}")
    print(f"  Alpha = 0x{alpha_q44:02X} ({alpha_str}) — {desc}")
    print(f"{'─' * 60}")

    # Drain stale data from previous run
    stale = drain(handle)
    if stale:
        print(f"  Drained {stale} stale bytes")

    # Configure CFAR
    for opcode, value, name in [
        (0x21, 2,          "CFAR_GUARD=2"),
        (0x22, 8,          "CFAR_TRAIN=8"),
        (0x23, alpha_q44,  f"CFAR_ALPHA=0x{alpha_q44:02X}"),
        (0x24, 0,          "CFAR_MODE=0 (CA)"),
        (0x26, 0,          "MTI_ENABLE=0"),
        (0x27, 0,          "DC_NOTCH=0"),
        (0x25, 1,          "CFAR_ENABLE=1"),
        (0x04, 0x07,       "STREAM=0x07 (all)"),
    ]:
        st, n = write_cmd(handle, build_cmd(opcode, value))
        if st != 0:
            print(f"  WARN: {name} write failed st={st}")
        time.sleep(0.03)

    # Brief pause to let config settle
    time.sleep(0.1)

    # Drain any status/stale packets
    drain(handle, rounds=10, timeout_ms=100)

    # Trigger playback
    st, n = write_cmd(handle, build_cmd(0x02, 1))
    if st != 0:
        print(f"  WARN: Trigger failed st={st}")

    # Collect data
    all_data = bytearray()
    start = time.time()
    no_data_count = 0

    while time.time() - start < 20.0:
        data, s = read_data(handle, 65536)
        if len(data) > 0:
            all_data.extend(data)
            no_data_count = 0
        else:
            no_data_count += 1
            if no_data_count > 100:  # 5s of silence
                break
            time.sleep(0.05)

    elapsed = time.time() - start

    # Request post-playback status for debug counters
    time.sleep(0.3)
    drain(handle, rounds=5, timeout_ms=50)
    write_cmd(handle, build_cmd(0xFF, 0))
    time.sleep(0.5)
    status_data, _ = read_data(handle, 65536)

    # Parse status
    detect_count_hw = -1
    cells_processed = -1
    valid_pulses = -1
    if status_data and len(status_data) >= 36:
        w6 = struct.unpack_from("<I", status_data, 28)[0]
        w7 = struct.unpack_from("<I", status_data, 32)[0]
        cells_processed = (w6 >> 16) & 0xFFFF
        detect_count_hw = (w7 >> 16) & 0xFFFF
        valid_pulses    = w7 & 0xFFFF

    # Count packet headers
    range_count = 0
    doppler_count = 0
    cfar_count = 0

    for i in range(0, len(all_data) - 3, 4):
        word = struct.unpack_from("<I", all_data, i)[0]
        if word == 0x000000AA:
            range_count += 1
        elif word == 0x000000CC:
            doppler_count += 1
        elif word == 0x000000DD:
            cfar_count += 1

    print(f"  Collected {len(all_data):,} bytes in {elapsed:.1f}s")
    print(f"  Range: {range_count}  Doppler: {doppler_count}  CFAR: {cfar_count}")
    print(f"  HW counters: cells={cells_processed} valid={valid_pulses} detect={detect_count_hw}")

    # Dump first few CFAR packets for this alpha
    if cfar_count > 0:
        cfar_found = 0
        for i in range(0, len(all_data) - 15, 4):
            word = struct.unpack_from("<I", all_data, i)[0]
            if word == 0x000000DD and cfar_found < 5:
                w1 = struct.unpack_from("<I", all_data, i+4)[0]
                w2 = struct.unpack_from("<I", all_data, i+8)[0]
                flag = (w1 >> 31) & 1
                rbin = (w1 >> 25) & 0x3F
                dbin = (w1 >> 20) & 0x1F
                mag  = w1 & 0x1FFFF
                thresh = w2 & 0x1FFFF
                print(f"    CFAR[{cfar_found}]: r={rbin} d={dbin} mag={mag} thr={thresh} flag={flag}")
                cfar_found += 1

    # Save raw capture for this alpha
    if save_prefix:
        outfile = f"/tmp/{save_prefix}_alpha_{alpha_q44:02x}.bin"
        with open(outfile, "wb") as f:
            f.write(all_data)
        print(f"  Saved to {outfile}")

    return range_count, doppler_count, cfar_count, detect_count_hw


def main():
    print("=" * 60)
    print("  v9c CFAR Alpha Sweep Test")
    print("  Tests multiple alpha values to diagnose threshold scaling")
    print("=" * 60)

    # Open device
    dev = ftd3xx.create(0)
    if dev is None:
        print("FAIL: ftd3xx.create returned None")
        return 1
    handle = dev.handle
    print(f"FT601 opened (handle={handle})")

    # Init pipes
    init_pipes(handle)
    print("Pipes initialized")

    # Initial drain
    stale = drain(handle)
    if stale:
        print(f"Drained {stale} initial stale bytes")

    # Run sweep
    results = []
    for alpha_q44, alpha_str, desc in ALPHA_SWEEP:
        rng, dop, cfar, hw_det = run_one_alpha(
            handle, alpha_q44, alpha_str, desc, save_prefix="v9c_sweep")
        results.append((alpha_q44, alpha_str, desc, rng, dop, cfar, hw_det))
        # Wait between runs for FPGA state to settle
        time.sleep(1.0)

    # Close device
    dev.close()

    # ── Summary ──────────────────────────────────────────────────────────
    print(f"\n{'=' * 70}")
    print(f"  ALPHA SWEEP RESULTS SUMMARY")
    print(f"{'=' * 70}")
    print(f"  {'Alpha':>7} {'Q4.4':>6} {'EffMult':>8} {'Range':>6} {'Dopp':>6} {'CFAR':>6} {'HWDet':>6}")
    print(f"  {'─'*7} {'─'*6} {'─'*8} {'─'*6} {'─'*6} {'─'*6} {'─'*6}")
    for alpha_q44, alpha_str, desc, rng, dop, cfar, hw_det in results:
        eff_mult = f"{alpha_q44}x"  # Effective = alpha_q44 * avg (since noise_sum ≈ 16*avg, >>4 gives alpha_q44*avg)
        print(f"  0x{alpha_q44:02X}    {alpha_str:>6}  {eff_mult:>8} {rng:>6} {dop:>6} {cfar:>6} {hw_det:>6}")

    print(f"\n  Interpretation:")
    print(f"  - threshold = (alpha_q44 * noise_sum) >> 4")
    print(f"  - For CA-CFAR with 16 training cells: noise_sum ≈ 16 * avg")
    print(f"  - So threshold ≈ alpha_q44 * avg_cell_magnitude")
    print(f"  - alpha_q44=0x05 → threshold ≈ 5x average (standard CFAR)")
    print(f"  - alpha_q44=0x30 → threshold ≈ 48x average (way too high)")
    print(f"{'=' * 70}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
