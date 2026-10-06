"""Scope core: acquisition memory, UDP receiver, trigger/decimation/measurement DSP and setup model.

No GUI dependencies - used by scope12.py and testable on its own.
"""
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass, asdict

import numpy as np

from scope_probe import unpack_block

NCH = 12
HDIV, VDIV_N = 10, 8                     # screen divisions
MAX_SOURCES = 16                         # protects memory from a misbehaving sender
MAX_SIGNALS = 256                        # per source


def seq125(lo, hi):
    out = []
    e = math.floor(math.log10(lo))
    while True:
        for m in (1, 2, 5):
            v = m * 10.0 ** e
            if v > hi * 1.0001:
                return out
            if v >= lo * 0.9999:
                out.append(float(f"{v:.3g}"))
        e += 1


VDIVS = seq125(1e-6, 1e5)
TDIVS = seq125(1e-7, 10.0)
_PFX = [(1e9, "G"), (1e6, "M"), (1e3, "k"), (1.0, ""), (1e-3, "m"), (1e-6, "µ"), (1e-9, "n"), (1e-12, "p")]


def eng(v, unit="", digits=4):
    if v is None or not np.isfinite(v):
        return "--"
    if v == 0:
        return f"0 {unit}".strip()
    a = abs(v)
    for f, p in _PFX:
        if a >= f * 0.99995:
            break
    s = f"{v / f:.{digits}g}"
    return f"{s} {p}{unit}".strip()


def ceil125(x, table=VDIVS):
    for v in table:
        if v >= x * 0.9999:
            return v
    return table[-1]


def nearest_idx(table, v):
    return int(np.argmin([abs(math.log(t / v)) if v > 0 else 1e9 for t in table]))


def split_unit(sig):
    if sig.endswith("]") and "[" in sig:
        k = sig.rindex("[")
        return sig[:k], sig[k + 1:-1]
    return sig, ""


# ============================================================ acquisition
class Source:
    """Per-source acquisition memory: linear buffer of 2*cap with compaction (always contiguous)."""

    def __init__(self, name, sigs, cap):
        self.name, self.sigs, self.cap = name, list(sigs), cap
        self.units = [split_unit(s)[1] for s in sigs]
        self.t = np.empty(2 * cap, dtype=np.float64)
        self.d = np.empty((len(sigs), 2 * cap), dtype=np.float32)
        self.w = 0
        self.rate = 0.0
        self.last_wall = 0.0
        self.total = 0
        self.gaps = 0                     # discontinuities in time (usually = UDP packets dropped)

    def append(self, t, d):
        n = len(t)
        if n == 0:
            return
        if self.w and t[0] < self.t[self.w - 1] - 1e-12:     # simulation was reset -> new record
            self.w = 0
        elif self.w and self.rate > 0 and t[0] - self.t[self.w - 1] > 3.0 / self.rate:
            self.gaps += 1
        if n > self.cap:
            t, d, n = t[-self.cap:], d[:, -self.cap:], self.cap
        if self.w + n > 2 * self.cap:
            keep = self.cap - n
            s = self.w - keep
            self.t[:keep] = self.t[s:self.w]
            self.d[:, :keep] = self.d[:, s:self.w]
            self.w = keep
        self.t[self.w:self.w + n] = t
        self.d[:, self.w:self.w + n] = d
        self.w += n
        self.total += n
        self.last_wall = time.monotonic()
        if n > 1 and t[-1] > t[0]:
            r = (n - 1) / (t[-1] - t[0])
            self.rate = r if self.rate == 0 else 0.8 * self.rate + 0.2 * r

    def view(self):
        s = max(0, self.w - self.cap)
        return self.t[s:self.w], self.d[:, s:self.w]


class Frozen:
    def __init__(self, src):
        t, d = src.view()
        self.name, self.sigs, self.units, self.rate = src.name, src.sigs, src.units, src.rate
        self.t, self.d = t.copy(), d.copy()
        self.last_wall, self.total, self.gaps = src.last_wall, src.total, src.gaps

    def view(self):
        return self.t, self.d


class Store:
    def lookup(self, key):
        if not key or "/" not in key:
            return None, -1
        sname, sig = key.split("/", 1)
        s = self.sources.get(sname)
        if s is None or sig not in s.sigs:
            return None, -1
        return s, s.sigs.index(sig)


class Acquisition(Store):
    def __init__(self, bind, port, depth, sim_targets, sim_decim):
        self.sources = {}
        self.lock = threading.Lock()
        self.version = 0
        self.depth = depth
        self.sim_targets = sim_targets
        self.sim_decim = sim_decim
        self.packets = 0
        self.errors = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
        except OSError:
            pass
        # the OS may grant less (Linux: net.core.rmem_max) - shown in the UI so drops are explainable
        self.rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        if hasattr(socket, "SIO_UDP_CONNRESET"):           # Windows: ignore ICMP port-unreachable
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.1)
        self.running = True
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        last_hb = 0.0
        while self.running:
            now = time.monotonic()
            if now - last_hb > 0.5:
                last_hb = now
                for tgt in self.sim_targets:
                    try:
                        self.sock.sendto(b"SUBS" + struct.pack("<H", self.sim_decim), tgt)
                    except OSError:
                        pass
            try:
                pkt, _ = self.sock.recvfrom(65535)
            except (socket.timeout, ConnectionResetError, OSError):
                continue
            try:
                name, sigs, t, d = unpack_block(pkt)
                if len(sigs) > MAX_SIGNALS:
                    raise ValueError("too many signals")
                with self.lock:
                    s = self.sources.get(name)
                    if s is None or s.sigs != sigs:
                        if s is None and len(self.sources) >= MAX_SOURCES:
                            raise ValueError("too many sources")
                        s = self.sources[name] = Source(name, sigs, self.depth)
                        self.version += 1
                    s.append(t, d)
                    self.packets += 1
            except Exception:                       # never let one bad packet stop acquisition
                self.errors += 1

    def clear(self):
        with self.lock:
            for s in self.sources.values():
                s.w = 0

    def signal_list(self):
        with self.lock:
            return [f"{n}/{g}" for n in sorted(self.sources) for g in self.sources[n].sigs]

    def stop(self):
        self.running = False
        self.th.join(timeout=1.0)
        self.sock.close()


class Snapshot(Store):
    def __init__(self, acq):
        self.lock = threading.Lock()
        with acq.lock:
            self.sources = {n: Frozen(s) for n, s in acq.sources.items()}


# ============================================================ DSP helpers
def find_trigger(t, v, level, edge, t_limit, search, after=-np.inf):
    i1 = int(np.searchsorted(t, t_limit, "right"))
    i0 = int(np.searchsorted(t, max(t_limit - search, after)))
    if i1 - i0 < 2:
        return None
    a, b = v[i0:i1 - 1], v[i0 + 1:i1]
    if edge == 0:
        m = (a < level) & (b >= level)
    elif edge == 1:
        m = (a > level) & (b <= level)
    else:
        m = ((a < level) & (b >= level)) | ((a > level) & (b <= level))
    k = np.flatnonzero(m)
    if len(k) == 0:
        return None
    j = i0 + int(k[-1])
    v0, v1 = float(v[j]), float(v[j + 1])
    fr = (level - v0) / (v1 - v0) if v1 != v0 else 0.0
    return float(t[j] + fr * (t[j + 1] - t[j]))


def peak_decimate(x, y, x0, x1, ncol):
    """Min/max per pixel column (like a DSO 'peak detect' display)."""
    n = len(x)
    if n <= 2 * ncol:
        return x, y
    edges = np.linspace(x0, x1, ncol + 1)
    idx = np.searchsorted(x, edges)
    idx[0], idx[-1] = 0, n
    st = idx[:-1][idx[1:] > idx[:-1]]
    mn = np.minimum.reduceat(y, st)
    mx = np.maximum.reduceat(y, st)
    xc = x[st]
    odd = (np.arange(len(st)) & 1).astype(bool)
    X = np.repeat(xc, 2)
    Y = np.empty(2 * len(st))
    Y[0::2] = np.where(odd, mx, mn)
    Y[1::2] = np.where(odd, mn, mx)
    return X, Y


def measure(t, v):
    if t is None or len(v) < 2:
        return None
    vmax, vmin = float(np.max(v)), float(np.min(v))
    pp = vmax - vmin
    # time-weighted mean/rms (samples may be non-uniform)
    dt = np.diff(t)
    T = t[-1] - t[0]
    if T > 0:
        vm = 0.5 * (v[1:] + v[:-1])
        mean = float(np.sum(vm * dt) / T)
        rms = float(math.sqrt(max(0.0, np.sum(0.5 * (v[1:] ** 2 + v[:-1] ** 2) * dt) / T)))
    else:
        mean, rms = float(np.mean(v)), float(np.sqrt(np.mean(v * v)))
    freq = duty = None
    if pp > 1e-12 * max(1.0, abs(vmax)):
        mid, h = 0.5 * (vmax + vmin), 0.1 * pp
        s = np.where(v > mid + h, 1, np.where(v < mid - h, -1, 0))
        idx = np.where(s != 0, np.arange(len(s)), 0)
        np.maximum.accumulate(idx, out=idx)
        st = s[idx]
        r = np.flatnonzero((st[1:] == 1) & (st[:-1] == -1)) + 1
        if len(r) >= 2:
            per = (t[r[-1]] - t[r[0]]) / (len(r) - 1)
            if per > 0:
                freq = 1.0 / per
                seg_t, seg_s = t[r[0]:r[-1] + 1], st[r[0]:r[-1] + 1]
                hi = np.sum(np.diff(seg_t) * (seg_s[:-1] == 1))
                duty = float(hi / (seg_t[-1] - seg_t[0]))
    return dict(max=vmax, min=vmin, pp=pp, mean=mean, rms=rms, freq=freq, duty=duty)


# ============================================================ model
@dataclass
class Ch:
    on: bool = False
    key: str = ""
    vdiv: float = 1.0
    pos: float = 0.0
    ofs: float = 0.0
    coup: str = "DC"
    digital: bool = False
    thr: float = 0.25
    autoset_pending: bool = False


def default_setup():
    chs = [Ch() for _ in range(NCH)]
    spec = [("sim/ia[A]", 2.0, False), ("sim/ib[A]", 2.0, False), ("sim/ic[A]", 2.0, False),
            ("sim/va[V]", -3.5, False), ("sim/vb[V]", -3.5, False), ("sim/vc[V]", -3.5, False),
            ("sim/vn[V]", -3.5, False), ("sim/hall_a", 0, True), ("sim/hall_b", 0, True),
            ("sim/hall_c", 0, True), ("sim/AH", 0, True), ("sim/AL", 0, True)]
    for c, (k, p, dig) in zip(chs, spec):
        c.on, c.key, c.pos, c.digital, c.autoset_pending = True, k, p, dig, not dig
    return dict(ch=[asdict(c) for c in chs], tdiv=1e-3, delay=0.0, trig_ch=7, edge=0, level=0.5,
                tmode=0, sel=0)


def sanitize_setup(st):
    """Coerce a (possibly hand-edited or old) setup dict into safe values; bad fields fall back."""
    d = default_setup()
    out = dict(d)
    chs = []
    raw = st.get("ch") if isinstance(st.get("ch"), list) else []
    for i in range(NCH):
        c = Ch(**d["ch"][i])
        src = raw[i] if i < len(raw) and isinstance(raw[i], dict) else {}
        for k, typ in (("on", bool), ("key", str), ("vdiv", float), ("pos", float), ("ofs", float),
                       ("coup", str), ("digital", bool), ("thr", float), ("autoset_pending", bool)):
            if k in src:
                try:
                    v = typ(src[k])
                    if typ is float and not math.isfinite(v):
                        continue
                    setattr(c, k, v)
                except (TypeError, ValueError):
                    pass
        c.vdiv = VDIVS[nearest_idx(VDIVS, c.vdiv)] if c.vdiv > 0 else 1.0
        c.pos = max(-VDIV_N / 2, min(VDIV_N / 2, c.pos))
        c.coup = c.coup if c.coup in ("DC", "AC", "GND") else "DC"
        chs.append(asdict(c))
    out["ch"] = chs

    def num(k, lo, hi, cast=float):
        try:
            v = cast(st.get(k, d[k]))
            return v if lo <= v <= hi else d[k]
        except (TypeError, ValueError):
            return d[k]
    out["tdiv"] = TDIVS[nearest_idx(TDIVS, num("tdiv", TDIVS[0], TDIVS[-1]))]
    out["delay"] = num("delay", -1e4, 1e4)
    out["trig_ch"] = num("trig_ch", 0, NCH - 1, int)
    out["edge"] = num("edge", 0, 2, int)
    out["level"] = num("level", -1e9, 1e9)
    out["tmode"] = num("tmode", 0, 1, int)          # never start in Single
    out["sel"] = num("sel", 0, NCH - 1, int)
    return out
