"""Probe publisher for scope12.py (the 12-channel oscilloscope digital twin).

Any program (your ESC, the motor sim, a test script) streams named signals, time-stamped with
SIMULATION time, to the scope over UDP (default 127.0.0.1:9100). Signals from different programs
line up on the scope as long as they use the same time base - in lock-step, stamp ESC samples
with the motor's fb["t"].

    from scope.scope_probe import ScopeProbe
    probe = ScopeProbe("esc")                       # source name shown in the scope
    probe.sample(fb["t"], duty=d, sector=s, theta_est=th)   # call every control tick
    # units: put them in brackets in the name ->  probe.sample(t, **{"iq_ref[A]": 3.0})

Packet ("SCP1", little-endian):
    char[4] "SCP1" | u16 nsig | u16 nsamp | u16 names_len | u16 0
    names_len bytes: "source\\0sig1\\0sig2\\0...\\0" (UTF-8)
    f64 t[nsamp]
    f32 data[nsig][nsamp]      (signal-major)
A C/C++ ESC can send the same packet itself.
"""
import socket
import struct
import time

import numpy as np

DEFAULT_SCOPE_PORT = 9100
MAGIC = b"SCP1"
MAX_PACKET = 60000


def pack_block(source, names, t, data):
    """t: (n,) float64, data: (nsig, n) array-like. Returns bytes."""
    nb = ("\0".join([source] + list(names)) + "\0").encode("utf-8")
    t = np.ascontiguousarray(t, dtype="<f8")
    d = np.ascontiguousarray(data, dtype="<f4")
    return struct.pack("<4sHHHH", MAGIC, len(names), len(t), len(nb), 0) + nb + t.tobytes() + d.tobytes()


def unpack_block(pkt):
    """Parse and validate one SCP1 packet. Raises ValueError on anything malformed."""
    if len(pkt) < 12:
        raise ValueError("short packet")
    magic, nsig, n, nl, _ = struct.unpack_from("<4sHHHH", pkt, 0)
    if magic != MAGIC:
        raise ValueError("bad magic")
    if nsig == 0 or n == 0:
        raise ValueError("empty block")
    if len(pkt) != 12 + nl + 8 * n + 4 * nsig * n:
        raise ValueError("length mismatch")
    try:
        parts = pkt[12:12 + nl].decode("utf-8").split("\0")
    except UnicodeDecodeError:
        raise ValueError("bad names") from None
    if parts and parts[-1] == "":
        parts = parts[:-1]
    if len(parts) != nsig + 1 or not parts[0] or "/" in parts[0] or len(set(parts[1:])) != nsig \
            or any(not p for p in parts[1:]):
        raise ValueError("bad source/signal names")
    off = 12 + nl
    t = np.frombuffer(pkt, dtype="<f8", count=n, offset=off).astype(np.float64)
    off += 8 * n
    d = np.frombuffer(pkt, dtype="<f4", count=nsig * n, offset=off).reshape(nsig, n)
    if not np.all(np.isfinite(t)) or (n > 1 and np.any(np.diff(t) < 0)):
        raise ValueError("timestamps must be finite and non-decreasing")
    return parts[0], parts[1:], t, d


def block_len(nsig, want=256):
    return max(1, min(want, (MAX_PACKET - 1024) // (8 + 4 * max(1, nsig))))


class ScopeProbe:
    """Generic publisher: call sample(t, **signals) as often as you like; data goes out in blocks."""

    def __init__(self, source="esc", host="127.0.0.1", port=DEFAULT_SCOPE_PORT, block=256, max_latency_s=0.01):
        self.source = source
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.want = block
        self.max_latency = max_latency_s
        self.names = None
        self.rows = []
        self.t_last_flush = time.monotonic()

    def sample(self, t, **signals):
        """Publish one sample. Keep the same set of names every call - changing it starts a new
        record on the scope (the source's history is cleared)."""
        names = tuple(signals)
        if names != self.names:
            self.flush()
            self.names = names
            self.block = block_len(len(names), self.want)
        self.rows.append((t, *signals.values()))
        if len(self.rows) >= self.block or time.monotonic() - self.t_last_flush > self.max_latency:
            self.flush()

    def flush(self):
        self.t_last_flush = time.monotonic()
        if not self.rows:
            return
        a = np.asarray(self.rows, dtype=np.float64)
        self.rows = []
        try:
            self.sock.sendto(pack_block(self.source, self.names, a[:, 0], a[:, 1:].T), self.addr)
        except OSError:
            pass

    def close(self):
        self.flush()
        self.sock.close()


# --------------------------------------------------------------- motor probe
MOTOR_SIGNALS = ("ia[A]", "ib[A]", "ic[A]", "va[V]", "vb[V]", "vc[V]", "vn[V]", "rpm[rpm]",
                 "torque[Nm]", "vbus[V]", "ibus[A]", "ibatt[A]", "hall_a", "hall_b", "hall_c",
                 "AH", "AL", "BH", "BL", "CH", "CL", "theta_e[rad]",
                 "bemf_a[V]", "bemf_b[V]", "bemf_c[V]", "temp[C]", "load[Nm]")
_TWO_PI = 6.283185307179586
_RPM = 60.0 / _TWO_PI


class MotorProbe:
    """Fast capture of every BLDCMotor internal signal. Call capture() after each motor.step().
    `send` is a callable taking the packet bytes (see udp_sender())."""

    def __init__(self, motor, send, source="sim", block=256, decim=1):
        self.m = motor
        self.send = send
        self.source = source
        self.block = block_len(len(MOTOR_SIGNALS), block)
        self.decim = max(1, int(decim))
        self._k = 0
        self.rows = []

    def capture(self):
        self._k += 1
        if self._k < self.decim:
            return
        self._k = 0
        m = self.m
        i, vt, e, g = m.i, m.vt, m.e, m.gates
        h = m.hall_bits()
        self.rows.append((m.t, i[0], i[1], i[2], vt[0], vt[1], vt[2], m.vn, m.omega * _RPM, m.Te,
                          m.vbus, m.ibus, m.ibatt, h[0], h[1], h[2], g[0], g[1], g[2], g[3], g[4], g[5],
                          (m.pp * m.theta) % _TWO_PI, e[0], e[1], e[2], m.Tw, m.Tload))
        if len(self.rows) >= self.block:
            self.flush()

    def flush(self):
        if not self.rows:
            return
        a = np.asarray(self.rows, dtype=np.float64)
        self.rows = []
        self.send(pack_block(self.source, MOTOR_SIGNALS, a[:, 0], a[:, 1:].T))


def udp_sender(host="127.0.0.1", port=DEFAULT_SCOPE_PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(pkt):
        try:
            s.sendto(pkt, (host, port))
        except OSError:
            pass
    return send
