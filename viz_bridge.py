#!/usr/bin/env python3
"""Live bridge: motor digital twin (bldc_sim.py) -> web motor visualizer, over WebSocket.

    python bldc_sim.py motor_tmotor_u8ii_kv100.json      # the motor (drive it with any ESC, e.g. foc_esc.py)
    python viz_bridge.py                                 # then open http://127.0.0.1:8765

The bridge subscribes to the sim's probe stream (same mechanism as scope12.py), keeps the newest state
and recent phase-current waveform, and pushes ~30 JSON frames/s to every connected browser. It also serves
web/motor_visualizer.html. No third-party packages: a minimal RFC 6455 WebSocket server on asyncio.
Use the sim's own slow-motion keys ( , and . ) to slow the motor down for the 3-D view.
"""
import argparse
import asyncio
import base64
import hashlib
import json
import math
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scope_probe import unpack_block          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "web", "motor_visualizer.html")
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
WAVE_MAX = 600                                   # waveform samples per frame (decimated)


class Twin:
    """Receives SCP1 probe blocks from the sim and keeps what the browser needs."""

    def __init__(self, sim_addr, port, decim):
        self.sim_addr, self.decim = sim_addr, decim
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        except OSError:
            pass
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.state = None
        self.wave = []                           # (t, ia, ib, ic) since last frame
        self.last_rx = 0.0
        self.unwrapped = 0.0
        self._prev_th = None

    def heartbeat(self):
        try:
            self.sock.sendto(b"SUBS" + struct.pack("<H", self.decim), self.sim_addr)
        except OSError:
            pass

    def poll(self):
        while True:
            try:
                pkt, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                continue
            try:
                src, names, t, d = unpack_block(pkt)
            except ValueError:
                continue
            if src != "sim":
                continue
            ix = {n: k for k, n in enumerate(names)}
            need = ("ia[A]", "ib[A]", "ic[A]", "theta_e[rad]")
            if not all(n in ix for n in need):
                continue
            ia, ib, ic, th = (d[ix[n]] for n in need)
            for k in range(len(t)):                      # unwrap electrical angle -> continuous rotor angle
                a = float(th[k])
                if self._prev_th is not None:
                    da = (a - self._prev_th + math.pi) % (2 * math.pi) - math.pi
                    if t[k] < (self.state or {}).get("t", 0) - 1e-9:       # sim reset
                        da = 0.0
                    self.unwrapped += da
                self._prev_th = a
            step = max(1, len(t) // 40)
            self.wave.extend((float(t[k]), float(ia[k]), float(ib[k]), float(ic[k])) for k in range(0, len(t), step))
            if len(self.wave) > 4 * WAVE_MAX:
                self.wave = self.wave[-4 * WAVE_MAX:]

            def g(name, default=0.0):
                return float(d[ix[name], -1]) if name in ix else default
            self.state = dict(t=float(t[-1]), theta_e=g("theta_e[rad]"), theta_unwrapped=self.unwrapped,
                              rpm=g("rpm[rpm]"), ia=g("ia[A]"), ib=g("ib[A]"), ic=g("ic[A]"),
                              va=g("va[V]"), vb=g("vb[V]"), vc=g("vc[V]"), vn=g("vn[V]"),
                              torque=g("torque[Nm]"), vbus=g("vbus[V]"), ibus=g("ibus[A]"),
                              ibatt=g("ibatt[A]"), temp=g("temp[C]"), load=g("load[Nm]"),
                              hall=[int(g("hall_a")), int(g("hall_b")), int(g("hall_c"))],
                              gates=[round(g(n), 3) for n in ("AH", "AL", "BH", "BL", "CH", "CL")])
            self.last_rx = time.monotonic()

    def frame(self):
        if self.state is None:
            return json.dumps({"type": "status", "connected": False})
        w = self.wave[-WAVE_MAX:]
        self.wave = []
        return json.dumps({"type": "frame", "live": time.monotonic() - self.last_rx < 1.0, **self.state,
                           "wave": {"t": [x[0] for x in w], "ia": [round(x[1], 3) for x in w],
                                    "ib": [round(x[2], 3) for x in w], "ic": [round(x[3], 3) for x in w]}})


def ws_frame(text):
    data = text.encode()
    n = len(data)
    if n < 126:
        hdr = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        hdr = struct.pack("!BBH", 0x81, 126, n)
    else:
        hdr = struct.pack("!BBQ", 0x81, 127, n)
    return hdr + data


def page_html():
    """The page file is a body fragment (Artifact format); wrap it into a full document for local serving."""
    with open(PAGE, encoding="utf-8") as f:
        body = f.read()
    return ("<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">"
            "<style>html,body{margin:0;height:100%}[hidden]{display:none!important}</style></head><body>"
            + body + "</body></html>").encode("utf-8")


async def main_async(args):
    twin = Twin((args.sim_host, args.sim_port), args.udp_port, args.decim)
    clients = set()

    async def handle(reader, writer):
        try:
            head = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 5)
        except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, asyncio.TimeoutError):
            writer.close()
            return
        lines = head.decode("latin-1").split("\r\n")
        path = lines[0].split(" ")[1] if len(lines[0].split(" ")) > 1 else "/"
        hdrs = {k.strip().lower(): v.strip() for k, _, v in (ln.partition(":") for ln in lines[1:] if ":" in ln)}
        if path.startswith("/ws") and "sec-websocket-key" in hdrs:
            acc = base64.b64encode(hashlib.sha1((hdrs["sec-websocket-key"] + GUID).encode()).digest()).decode()
            writer.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                          f"Sec-WebSocket-Accept: {acc}\r\n\r\n").encode())
            await writer.drain()
            clients.add(writer)
            try:
                while True:                               # drain client frames until it closes
                    b = await reader.read(4096)
                    if not b or (b[0] & 0x0F) == 0x8:
                        break
            except (ConnectionError, asyncio.IncompleteReadError):
                pass
            finally:
                clients.discard(writer)
                writer.close()
            return
        if path in ("/", "/index.html", "/motor_visualizer.html"):
            try:
                body, ctype, code = page_html(), "text/html; charset=utf-8", "200 OK"
            except OSError:
                body, ctype, code = b"web/motor_visualizer.html not found", "text/plain", "404 Not Found"
        else:
            body, ctype, code = b"not found", "text/plain", "404 Not Found"
        writer.write(f"HTTP/1.1 {code}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n"
                     "Cache-Control: no-store\r\nConnection: close\r\n\r\n".encode() + body)
        await writer.drain()
        writer.close()

    server = await asyncio.start_server(handle, args.host, args.port)
    print(f"motor visualizer: http://{args.host}:{args.port}   (probing sim at {args.sim_host}:{args.sim_port})")
    last_hb = 0.0
    async with server:
        while True:
            now = time.monotonic()
            if now - last_hb > 0.5:
                last_hb = now
                twin.heartbeat()
            twin.poll()
            if clients:
                msg = ws_frame(twin.frame())
                for w in list(clients):
                    try:
                        w.write(msg)
                    except (ConnectionError, RuntimeError):
                        clients.discard(w)
            await asyncio.sleep(1 / 30)


def main():
    ap = argparse.ArgumentParser(description="Live bridge from bldc_sim.py to the web motor visualizer")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--sim-host", default="127.0.0.1")
    ap.add_argument("--sim-port", type=int, default=9000)
    ap.add_argument("--udp-port", type=int, default=9101, help="local UDP port for the probe stream")
    ap.add_argument("--decim", type=int, default=1)
    args = ap.parse_args()
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print(f"WARNING: serving on {args.host} - anyone on that network can open the visualizer.")
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
