#!/usr/bin/env python3
"""Diagnostic test: verify v8b throughput fix - expect ~2048 packets"""
import sys
sys.path.insert(0, '/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_3_GUI')
from radar_protocol import FT601Connection, PIPE_IN
import struct
import time

conn = FT601Connection(mock=False)
if not conn.open(0):
    print("Failed to open FT601")
    sys.exit(1)

time.sleep(0.5)

def send_command(opcode, addr, value):
    cmd = struct.pack(">I", (opcode << 24) | (addr << 16) | value)
    conn.write(cmd)
    time.sleep(0.01)

def read_packet():
    pkt = conn.read(64)
    if pkt and len(pkt) >= 24:
        return pkt
    return None

print("Flushing stale data...")
conn._flush()
time.sleep(0.2)

print("Enabling stream control...")
send_command(0x04, 0x00, 0x01)
time.sleep(0.1)

print("Triggering playback...")
send_command(0x02, 0x00, 0x01)
time.sleep(0.1)

print("Reading packets...")
packet_count = 0
start_time = time.time()
timeouts = 0

while time.time() - start_time < 5.0:
    pkt = read_packet()
    if pkt and len(pkt) >= 24:
        if pkt[0] == 0xAA:
            packet_count += 1
            if packet_count % 100 == 0:
                print(f"  Received {packet_count} packets...")
    else:
        timeouts += 1
        if timeouts > 100:
            break
        time.sleep(0.001)

print(f"\n=== RESULT ===")
print(f"Packets received: {packet_count}")
print(f"Expected: ~2048 (32 chirps x 64 bins)")
print(f"Timeouts: {timeouts}")
print(f"Status: {'SUCCESS - throughput fix working!' if packet_count > 1000 else 'FAIL - still bottlenecked'}")

conn.close()
