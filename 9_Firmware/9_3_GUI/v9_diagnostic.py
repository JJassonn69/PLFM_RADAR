#!/usr/bin/env python3
"""
v9 end-to-end diagnostic: connect through TCP tunnel, trigger playback
with stream_control=0x07, count packets by type.
"""
import sys
import time
import struct

sys.path.insert(0, ".")
from radar_protocol import (
    RadarProtocol, SocketConnection, Opcode,
    HEADER_BYTE, HEADER_DOPPLER, HEADER_CFAR, STATUS_HEADER_BYTE,
)

HOST = "localhost"
PORT = 9000

def main():
    conn = SocketConnection(host=HOST, port=PORT)
    print(f"Connecting to {HOST}:{PORT}...")
    if not conn.open():
        print("FAIL: could not connect")
        return 1

    print("Connected. Configuring v9 full pipeline...")

    # Enable CFAR
    cmd = RadarProtocol.build_command(Opcode.CFAR_ENABLE, 1)
    conn.write(cmd)
    print(f"  Sent CFAR_ENABLE=1")
    time.sleep(0.1)

    # Set stream_control=0x07 (range + doppler + cfar)
    cmd = RadarProtocol.build_command(Opcode.STREAM_CONTROL, 0x07)
    conn.write(cmd)
    print(f"  Sent STREAM_CONTROL=0x07")
    time.sleep(0.1)

    # Request status first
    cmd = RadarProtocol.build_command(0xFF, 0)
    conn.write(cmd)
    print(f"  Sent STATUS_REQUEST")
    time.sleep(0.5)

    # Read status response
    raw = conn.read(4096)
    if raw:
        status = RadarProtocol.parse_status_packet(raw)
        if status:
            print(f"  Status: mode={status.radar_mode} "
                  f"stream_ctrl=0x{status.stream_ctrl:02X}")
        else:
            print(f"  Got {len(raw)} bytes but no status packet parsed")
    else:
        print("  No status response (timeout)")

    # Trigger playback
    cmd = RadarProtocol.build_command(Opcode.TRIGGER, 1)
    conn.write(cmd)
    print(f"\nTriggered playback! Reading packets...")

    # Collect data for several seconds
    counts = {"data": 0, "doppler": 0, "cfar": 0, "status": 0, "unknown": 0}
    total_bytes = 0
    start = time.time()
    timeout = 30.0  # seconds — v9a: ~5400 packets with 2048-entry CFAR FIFO

    while time.time() - start < timeout:
        raw = conn.read(65536)
        if raw is None or len(raw) == 0:
            time.sleep(0.01)
            continue

        total_bytes += len(raw)
        packets = RadarProtocol.find_packet_boundaries(raw)

        for pstart, pend, ptype in packets:
            counts[ptype] = counts.get(ptype, 0) + 1

        # Stop early if we've received enough (2048+ range, 2048 doppler, 100+ cfar)
        if counts["doppler"] >= 2048 and counts["cfar"] >= 100:
            # Wait a bit more for remaining CFAR packets
            extra_start = time.time()
            while time.time() - extra_start < 5.0:
                raw = conn.read(65536)
                if raw:
                    total_bytes += len(raw)
                    for pstart, pend, ptype in RadarProtocol.find_packet_boundaries(raw):
                        counts[ptype] = counts.get(ptype, 0) + 1
                else:
                    time.sleep(0.01)
            break

    elapsed = time.time() - start

    print(f"\n{'='*50}")
    print(f"  v9 Diagnostic Results ({elapsed:.1f}s)")
    print(f"{'='*50}")
    print(f"  Total bytes received: {total_bytes:,}")
    print(f"  Range (0xAA) packets: {counts['data']}")
    print(f"  Doppler (0xCC) packets: {counts['doppler']}")
    print(f"  CFAR (0xDD) packets: {counts['cfar']}")
    print(f"  Status (0xBB) packets: {counts['status']}")
    print(f"  Unknown packets: {counts.get('unknown', 0)}")
    print(f"{'='*50}")

    # Validate expected counts
    ok = True
    if counts["data"] < 2048:
        print(f"  WARNING: Expected 2048 range packets, got {counts['data']}")
        ok = False
    if counts["doppler"] < 2048:
        print(f"  WARNING: Expected 2048 Doppler packets, got {counts['doppler']}")
        ok = False
    if counts["cfar"] < 100:
        print(f"  WARNING: Expected 100+ CFAR packets, got {counts['cfar']}")
        ok = False

    if ok:
        print(f"\n  PASS: v9 full pipeline working!")
    else:
        print(f"\n  PARTIAL: Some packet types missing (check stream_control)")

    conn.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
