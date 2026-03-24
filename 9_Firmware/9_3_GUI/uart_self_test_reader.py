#!/usr/bin/env python3
"""
AERIS-10 UART Self-Test Reader

Reads self-test status packets from the FPGA over UART.
Designed to run on the remote server connected to the TE0701 carrier board.

Usage:
    python3 uart_self_test_reader.py [OPTIONS]

Options:
    --port PORT      Serial port (default: /dev/ttyUSB0)
    --baud BAUD      Baud rate (default: 115200)
    --request        Send 'S' to request status (otherwise just listen)
    --trigger        Send 'T' to trigger self-test then read result
    --loop           Continuously listen for packets
    --timeout SECS   Read timeout in seconds (default: 5)

Packet format (20 bytes):
    Byte  0-1:  0xA5 0x5A  (sync markers)
    Byte  2:    0x01        (packet type: self-test report)
    Byte  3:    result_flags[4:0] (bit per subsystem)
    Byte  4:    result_detail[7:0]
    Byte  5:    busy flag
    Byte  6-7:  version major.minor
    Byte  8-11: heartbeat counter (big-endian)
    Byte 12-16: ASCII P/F for BRAM, CIC, FFT, ARITH, ADC
    Byte 17-18: CR LF
    Byte 19:    checksum (XOR of bytes 0..18)
"""

import argparse
import sys
import time


PKT_LEN = 20
SYNC_0 = 0xA5
SYNC_1 = 0x5A

SUBSYSTEM_NAMES = ["BRAM", "CIC", "FFT", "ARITH", "ADC"]


def parse_packet(data):
    """Parse a 20-byte self-test status packet. Returns dict or None."""
    if len(data) < PKT_LEN:
        return None

    # Verify sync
    if data[0] != SYNC_0 or data[1] != SYNC_1:
        return None

    # Verify checksum
    xor_sum = 0
    for i in range(PKT_LEN - 1):
        xor_sum ^= data[i]
    if xor_sum != data[PKT_LEN - 1]:
        return {
            "error": f"Checksum mismatch: computed 0x{xor_sum:02X}, got 0x{data[PKT_LEN-1]:02X}",
            "raw": data.hex(),
        }

    pkt_type = data[2]
    flags = data[3] & 0x1F
    detail = data[4]
    busy = bool(data[5] & 0x01)
    ver_major = data[6]
    ver_minor = data[7]
    heartbeat = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11]

    subsystems = {}
    for i, name in enumerate(SUBSYSTEM_NAMES):
        bit_pass = bool(flags & (1 << i))
        ascii_char = chr(data[12 + i]) if 0x20 <= data[12 + i] <= 0x7E else "?"
        subsystems[name] = {"pass": bit_pass, "ascii": ascii_char}

    all_pass = flags == 0x1F

    return {
        "type": pkt_type,
        "flags": flags,
        "detail": detail,
        "busy": busy,
        "version": f"{ver_major}.{ver_minor}",
        "heartbeat": heartbeat,
        "subsystems": subsystems,
        "all_pass": all_pass,
        "raw": data.hex(),
    }


def print_packet(pkt):
    """Pretty-print a parsed packet."""
    if "error" in pkt:
        print(f"  ERROR: {pkt['error']}")
        print(f"  Raw: {pkt['raw']}")
        return

    status_str = "ALL PASS" if pkt["all_pass"] else "FAIL"
    print(f"  Status:    {status_str}")
    print(f"  Version:   {pkt['version']}")
    print(f"  Busy:      {pkt['busy']}")
    print(f"  Heartbeat: 0x{pkt['heartbeat']:08X} ({pkt['heartbeat']})")
    print(f"  Detail:    0x{pkt['detail']:02X}")
    print(f"  Flags:     0b{pkt['flags']:05b}")
    print()
    print("  Subsystem Results:")
    for name in SUBSYSTEM_NAMES:
        sub = pkt["subsystems"][name]
        mark = "PASS" if sub["pass"] else "FAIL"
        print(f"    {name:6s}: {mark} ({sub['ascii']})")
    print()


def read_packet(ser, timeout=5.0):
    """Read one packet from serial, syncing on 0xA5 0x5A header."""
    start_time = time.time()

    # Sync hunt: find 0xA5 0x5A
    state = 0  # 0=hunting, 1=got A5
    while time.time() - start_time < timeout:
        b = ser.read(1)
        if len(b) == 0:
            continue
        byte = b[0]
        if state == 0:
            if byte == SYNC_0:
                state = 1
        elif state == 1:
            if byte == SYNC_1:
                # Found sync, read remaining 18 bytes
                remaining = ser.read(PKT_LEN - 2)
                if len(remaining) == PKT_LEN - 2:
                    return bytes([SYNC_0, SYNC_1]) + remaining
                else:
                    print(f"  Warning: incomplete packet ({len(remaining)+2}/{PKT_LEN} bytes)")
                    return None
            elif byte == SYNC_0:
                state = 1  # Could be start of new sync
            else:
                state = 0

    return None


def main():
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="AERIS-10 UART Self-Test Reader")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--request", action="store_true", help="Send 'S' to request status")
    parser.add_argument("--trigger", action="store_true", help="Send 'T' to trigger self-test")
    parser.add_argument("--loop", action="store_true", help="Continuously listen")
    parser.add_argument("--timeout", type=float, default=5.0, help="Read timeout (seconds)")
    args = parser.parse_args()

    print(f"AERIS-10 UART Self-Test Reader")
    print(f"  Port: {args.port}")
    print(f"  Baud: {args.baud}")
    print()

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        sys.exit(1)

    try:
        # Flush any stale data
        ser.reset_input_buffer()

        if args.trigger:
            print("Sending self-test trigger ('T')...")
            ser.write(b"T")
            time.sleep(0.1)  # Let test run (~few ms in FPGA)
            print("Waiting for result...\n")

        if args.request:
            print("Requesting status ('S')...")
            ser.write(b"S")
            time.sleep(0.05)

        while True:
            data = read_packet(ser, timeout=args.timeout)
            if data is not None:
                pkt = parse_packet(data)
                if pkt:
                    print(f"--- Packet received ({len(data)} bytes) ---")
                    print_packet(pkt)
                else:
                    print("  Received data but failed to parse")
            else:
                if not args.loop:
                    print("  No packet received (timeout)")
                    break
                # In loop mode, silently retry

            if not args.loop:
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
