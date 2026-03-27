#!/usr/bin/env python3
"""
FT601 Stream Server — TCP bridge for remote FPGA data streaming.

Runs on the remote server (with FT601 hardware), opens the USB device,
and serves data over a TCP socket so a local GUI can visualize live
FPGA data through an SSH tunnel.

Architecture:
    [FPGA] --FT601/USB3--> [This Server :9000] --TCP--> [SSH tunnel]
                                                           |
    [Local Mac GUI] <-- localhost:9000 <-------------------+

Protocol (simple framing over TCP):
    FT601 reads  -> raw bytes forwarded to TCP client
    TCP client writes -> raw bytes forwarded to FT601 (commands)

Usage on remote server:
    sudo /home/jason-stone/PLFM_RADAR_work/venv/bin/python3 \\
        ft601_stream_server.py [--port 9000] [--device 0]

SSH tunnel from local Mac:
    ssh -i ~/.ssh/gpu_server_key -p 8765 -L 9000:localhost:9000 \\
        jason-stone@livepeerservice.ddns.net

Then on local Mac:
    python3 GUI_radar_dashboard_v2.py --remote localhost:9000
"""
import argparse
import logging
import select
import socket
import struct
import sys
import threading
import time

log = logging.getLogger("ft601_server")
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s",
                    datefmt="%H:%M:%S")

# ============================================================================
# FT601 D3XX low-level I/O (proven pattern from diag_ft601_v7b.py)
# ============================================================================

try:
    import ctypes
    import ftd3xx
    import ftd3xx._ftd3xx_linux as _ll
    FTD3XX_AVAILABLE = True
except ImportError:
    FTD3XX_AVAILABLE = False
    log.warning("ftd3xx not available — server will not function")

PIPE_OUT = 0x02  # Host -> FPGA
PIPE_IN  = 0x82  # FPGA -> Host


def raw_write(handle, pipe, data, timeout_ms=1000):
    buf = ctypes.create_string_buffer(data, len(data))
    xfer = ctypes.c_ulong(0)
    status = _ll.FT_WritePipeEx(handle, ctypes.c_ubyte(pipe),
                                buf, ctypes.c_ulong(len(data)),
                                ctypes.byref(xfer), ctypes.c_ulong(timeout_ms))
    return xfer.value if status == 0 else -status


def raw_read(handle, pipe, size, timeout_ms=100):
    buf = ctypes.create_string_buffer(size)
    xfer = ctypes.c_ulong(0)
    status = _ll.FT_ReadPipeEx(handle, ctypes.c_ubyte(pipe),
                                buf, ctypes.c_ulong(size),
                                ctypes.byref(xfer), ctypes.c_ulong(timeout_ms))
    return buf.raw[:xfer.value] if status == 0 else b""


def build_cmd(opcode, value, addr=0):
    word = ((opcode & 0xFF) << 24) | ((addr & 0xFF) << 16) | (value & 0xFFFF)
    return struct.pack(">I", word)


# ============================================================================
# Stream Server
# ============================================================================

class FT601StreamServer:
    """
    Single-client TCP server that bridges FT601 USB to a network socket.

    Two threads per client:
      - Reader thread: FT601 USB read -> TCP send (data stream)
      - Writer thread: TCP recv -> FT601 USB write (commands)

    Uses a simple length-prefixed protocol for commands (client -> server):
      [4 bytes: length (big-endian uint32)] [length bytes: command data]

    Data from FPGA (server -> client) is sent raw with length prefix:
      [4 bytes: length (big-endian uint32)] [length bytes: USB data]

    A zero-length message means "no data this cycle" (keepalive).
    """

    def __init__(self, port: int = 9000, device_index: int = 0):
        self._port = port
        self._device_index = device_index
        self._device = None
        self._handle = None
        self._server_sock = None
        self._running = False

    def _open_ft601(self) -> bool:
        if not FTD3XX_AVAILABLE:
            log.error("ftd3xx not available")
            return False

        try:
            self._device = ftd3xx.create(self._device_index)
            if self._device is None:
                log.error("ftd3xx.create returned None")
                return False
            self._handle = self._device.handle
            # Flush stale data
            for _ in range(50):
                d = raw_read(self._handle, PIPE_IN, 16384, timeout_ms=50)
                if not d:
                    break
            try:
                self._device.flushPipe(PIPE_IN)
            except Exception:
                pass
            log.info(f"FT601 device {self._device_index} opened")
            return True
        except Exception as e:
            log.error(f"FT601 open failed: {e}")
            return False

    def _close_ft601(self):
        if self._device is not None:
            # Stop streaming before close
            try:
                raw_write(self._handle, PIPE_OUT,
                          build_cmd(0x04, 0x00), timeout_ms=500)
            except Exception:
                pass
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None
            self._handle = None
            log.info("FT601 device closed")

    def run(self):
        if not self._open_ft601():
            sys.exit(1)

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(("0.0.0.0", self._port))
        self._server_sock.listen(1)
        self._running = True

        log.info(f"Listening on port {self._port}...")
        log.info("Waiting for client connection (start SSH tunnel + GUI)")

        try:
            while self._running:
                self._server_sock.settimeout(1.0)
                try:
                    client_sock, addr = self._server_sock.accept()
                except socket.timeout:
                    continue
                log.info(f"Client connected from {addr}")
                client_sock.setsockopt(socket.IPPROTO_TCP,
                                       socket.TCP_NODELAY, 1)
                self._handle_client(client_sock)
                log.info(f"Client {addr} disconnected")
        except KeyboardInterrupt:
            log.info("Shutting down...")
        finally:
            self._close_ft601()
            self._server_sock.close()

    def _handle_client(self, client_sock: socket.socket):
        """
        Handle one client session. Runs two threads:
          - usb_reader: FT601 -> client (data packets)
          - cmd_reader: client -> FT601 (command words)
        """
        stop_event = threading.Event()

        def usb_reader():
            """Read from FT601 and send to client with length prefix."""
            try:
                while not stop_event.is_set():
                    data = raw_read(self._handle, PIPE_IN, 16384,
                                    timeout_ms=100)
                    if data:
                        # Length-prefixed: [4 bytes len] [data]
                        hdr = struct.pack(">I", len(data))
                        try:
                            client_sock.sendall(hdr + data)
                        except (BrokenPipeError, ConnectionResetError):
                            stop_event.set()
                            break
            except Exception as e:
                if not stop_event.is_set():
                    log.error(f"USB reader error: {e}")
                stop_event.set()

        def cmd_reader():
            """Read length-prefixed commands from client and write to FT601."""
            try:
                while not stop_event.is_set():
                    # Read 4-byte length header
                    hdr = _recv_exact(client_sock, 4, stop_event)
                    if hdr is None:
                        stop_event.set()
                        break
                    length = struct.unpack(">I", hdr)[0]
                    if length == 0:
                        continue  # keepalive
                    if length > 65536:
                        log.warning(f"Command too large: {length} bytes")
                        stop_event.set()
                        break
                    cmd_data = _recv_exact(client_sock, length, stop_event)
                    if cmd_data is None:
                        stop_event.set()
                        break
                    # Forward to FT601
                    n = raw_write(self._handle, PIPE_OUT, cmd_data,
                                  timeout_ms=1000)
                    if n > 0:
                        log.debug(f"CMD forwarded: {cmd_data.hex()} "
                                  f"({n} bytes)")
                    else:
                        log.warning(f"CMD write failed: {cmd_data.hex()}")
            except Exception as e:
                if not stop_event.is_set():
                    log.error(f"CMD reader error: {e}")
                stop_event.set()

        reader_thread = threading.Thread(target=usb_reader, daemon=True)
        cmd_thread = threading.Thread(target=cmd_reader, daemon=True)
        reader_thread.start()
        cmd_thread.start()

        # Wait for either thread to stop
        while not stop_event.is_set():
            stop_event.wait(timeout=0.5)

        # Cleanup
        try:
            client_sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        client_sock.close()

        reader_thread.join(timeout=2)
        cmd_thread.join(timeout=2)


def _recv_exact(sock: socket.socket, n: int,
                stop_event: threading.Event) -> bytes:
    """Receive exactly n bytes from socket, or return None on error/close."""
    buf = bytearray()
    while len(buf) < n:
        if stop_event.is_set():
            return None
        try:
            sock.settimeout(0.5)
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None  # Connection closed
            buf.extend(chunk)
        except socket.timeout:
            continue
        except (ConnectionResetError, BrokenPipeError):
            return None
    return bytes(buf)


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="FT601 Stream Server — TCP bridge for remote FPGA data")
    parser.add_argument("--port", type=int, default=9000,
                        help="TCP port to listen on (default: 9000)")
    parser.add_argument("--device", type=int, default=0,
                        help="FT601 device index (default: 0)")
    args = parser.parse_args()

    server = FT601StreamServer(port=args.port, device_index=args.device)
    server.run()


if __name__ == "__main__":
    main()
