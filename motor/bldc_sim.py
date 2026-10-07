#!/usr/bin/env python3
"""BLDC motor digital twin - pygame UI + UDP interface for an external (simulated) ESC.

    python motor/bldc_sim.py [motor_config.json] [--port 9000] [--dt 10e-6] [--demo 0.5]

Physics + UDP server run in their own process (no GIL fight with the UI), pygame UI in the main one.

Keys:  1 = external ESC (UDP)   2 = built-in demo 6-step ESC   0 = all FETs off (coast)
       UP/DOWN demo duty        SPACE demo direction            R reset motor
       C reload config file     P pause                         [ ]  scope time base
       , .  slow-motion         ESC quit
"""
import argparse
import math
import multiprocessing as mp
import os
import queue
import random
import select
import socket
import struct
import sys
import time
from dataclasses import asdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from motor.bldc_model import CONFIG_DIR, config_path, BLDCMotor, MotorConfig, SECTOR_FWD, RADS2RPM, TWO_PI, PI
from motor.bldc_protocol import CMD_FMT, CMD_SIZE, REPLY_FMT, MODE_GATES, FLAG_RESET, DEFAULT_PORT
from scope.scope_probe import MotorProbe

NCH = 14            # history channels: ia ib ic va vb vc vn rpm Te vbus ibus ha hb hc
SAMPLE_DT = 20e-6   # scope sample period
HIST_S = 2.0
MAX_ADVANCE_US = 1e6   # lock-step: max sim time per packet (1 s) - a bad packet can't freeze the sim
MAX_SUBSCRIBERS = 4    # scopes allowed to receive the probe stream at once

# ---- UI -> sim control block (shared doubles)
C_MODE, C_DUTY, C_DIR, C_PAUSE, C_SPEED, C_RESET, C_LOADT, C_FAN, C_LINERT, C_VBAT, C_RUN = range(11)
NCTRL = 12
MODES = ("EXT", "INT", "OFF")
# ---- sim -> UI status block
(S_T, S_TH, S_OM, S_IA, S_IB, S_IC, S_VA, S_VB, S_VC, S_VN, S_IBUS, S_IBATT, S_VBUS, S_TE, S_TW,
 S_HALL, S_FAULT, S_RTF, S_PKTS, S_LINK, S_WIDX, S_DT, S_PORT, S_READY) = range(24)
NSTAT = 24


class SimHost:
    """Owns the motor; real-time or lock-step stepping; UDP server. Runs in the sim process."""

    def __init__(self, cfg, dt, port, bind, ctrl, stat, hist, cfg_q):
        self.motor = BLDCMotor(cfg)
        self.cfg = cfg
        self.dt = dt
        self.decim = max(1, round(SAMPLE_DT / dt))
        self.N = int(HIST_S / (self.decim * dt))
        self.buf = np.frombuffer(hist, dtype=np.float32, count=NCH * self.N).reshape(NCH, self.N)
        self.ctrl, self.stat, self.cfg_q = ctrl, stat, cfg_q
        self.widx = 0
        self._dec = 0
        self._ikey = None
        self._reset_seen = ctrl[C_RESET]
        self.last_ext = -1e9
        self.last_lock = -1e9
        self.pkts = 0
        self.rtf = 0.0
        self._coasted = False
        self.subs = {}                     # scope subscribers: addr -> (last heartbeat, decimation)
        self.probing = False
        self.probe = MotorProbe(self.motor, self._probe_send, source="sim")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)
        if hasattr(socket, "SIO_UDP_CONNRESET"):      # Windows: don't error on ICMP port-unreachable
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        stat[S_DT] = dt
        stat[S_PORT] = port

    # ------------------------------------------------------------ shared I/O
    def _pull_ctrl(self):
        c, cfg = self.ctrl, self.cfg
        cfg.load_torque_nm = c[C_LOADT]
        cfg.load_fan_k = c[C_FAN]
        cfg.load_inertia_kgm2 = c[C_LINERT]
        cfg.battery_voltage_v = c[C_VBAT]
        now = time.monotonic()
        for a in [a for a, (ts, _) in self.subs.items() if now - ts > 2.0]:
            del self.subs[a]
        self.probing = bool(self.subs)
        if self.probing:
            self.probe.decim = min(d for _, d in self.subs.values())
            self.probe.flush()
        else:
            self.probe.rows = []
        if c[C_RESET] != self._reset_seen:
            self._reset_seen = c[C_RESET]
            self._do_reset()
        while True:
            try:
                d = self.cfg_q.get_nowait()
            except queue.Empty:
                break
            try:
                MotorConfig(**d).validate()
            except (TypeError, ValueError) as ex:
                print("config rejected:", ex)
                continue
            for k, v in d.items():
                setattr(cfg, k, v)
            self.motor.rebuild()
            self._do_reset()

    def _probe_send(self, pkt):
        for addr in list(self.subs):
            try:
                self.sock.sendto(pkt, addr)
            except OSError:
                pass

    def _do_reset(self):
        if self.probing:
            self.probe.flush()
        self.motor.reset()
        self._ikey = None
        self.widx = 0
        self.buf[:] = 0.0

    def _push_stat(self):
        m, s = self.motor, self.stat
        now = time.monotonic()
        h = m.hall_bits()
        s[S_T] = m.t; s[S_TH] = m.theta; s[S_OM] = m.omega
        s[S_IA] = m.i[0]; s[S_IB] = m.i[1]; s[S_IC] = m.i[2]
        s[S_VA] = m.vt[0]; s[S_VB] = m.vt[1]; s[S_VC] = m.vt[2]; s[S_VN] = m.vn
        s[S_IBUS] = m.ibus; s[S_IBATT] = m.ibatt; s[S_VBUS] = m.vbus
        s[S_TE] = m.Te; s[S_TW] = m.Tw
        s[S_HALL] = h[0] | (h[1] << 1) | (h[2] << 2)
        s[S_FAULT] = m.fault_bits()
        s[S_RTF] = self.rtf
        s[S_PKTS] = self.pkts
        s[S_LINK] = 2 if now - self.last_lock < 0.25 else 1 if now - self.last_ext < 0.5 else 0
        s[S_WIDX] = self.widx
        s[S_READY] = 1

    # ------------------------------------------------------------- stepping
    def _internal_commutate(self, duty, direction):
        m = self.motor
        sct = m.hall_sector()
        key = (sct, duty, direction)
        if key == self._ikey:
            return
        self._ikey = key
        if duty <= 0.0:
            m.coast()
            return
        hi, lo = SECTOR_FWD[sct]
        if direction < 0:
            hi, lo = lo, hi
        du = [0.0, 0.0, 0.0]
        en = [False, False, False]
        du[hi] = duty
        en[hi] = en[lo] = True
        m.set_duty(du[0], du[1], du[2], en)

    def _advance(self, n):
        m, b, dt = self.motor, self.buf, self.dt
        mode = MODES[int(self.ctrl[C_MODE])]
        duty, direction = self.ctrl[C_DUTY], self.ctrl[C_DIR]
        if mode == "OFF":
            m.coast()
        N = self.N
        cap = self.probe.capture if self.probing else None
        for _ in range(n):
            if mode == "INT":
                self._internal_commutate(duty, direction)
            m.step(dt)
            if cap:
                cap()
            self._dec += 1
            if self._dec >= self.decim:
                self._dec = 0
                j = self.widx % N
                i, vt = m.i, m.vt
                b[0, j] = i[0]; b[1, j] = i[1]; b[2, j] = i[2]
                b[3, j] = vt[0]; b[4, j] = vt[1]; b[5, j] = vt[2]
                b[6, j] = m.vn; b[7, j] = m.omega * RADS2RPM; b[8, j] = m.Te
                b[9, j] = m.vbus; b[10, j] = m.ibus
                h = m.hall_bits()
                b[11, j] = h[0]; b[12, j] = h[1]; b[13, j] = h[2]
                self.widx += 1

    # ------------------------------------------------------------------ UDP
    def _reply(self, addr):
        m = self.motor
        h = m.hall_bits()
        sd = m.cfg.current_sensor_noise_a           # current-sensor noise as seen by the ESC
        nz = (lambda: random.gauss(0.0, sd)) if sd > 0 else (lambda: 0.0)
        pkt = struct.pack(REPLY_FMT, m.t, m.theta, m.omega, m.omega * RADS2RPM,
                          m.i[0] + nz(), m.i[1] + nz(), m.i[2] + nz(), m.ibus + nz(), m.ibatt, m.vbus,
                          m.vt[0], m.vt[1], m.vt[2], m.vn, m.Te, m.Tw,
                          h[0] | (h[1] << 1) | (h[2] << 2), m.fault_bits())
        try:
            self.sock.sendto(pkt, addr)
        except OSError:
            pass

    def _handle(self, data, addr):
        if data[:4] == b"SUBS":                                   # oscilloscope heartbeat
            decim = struct.unpack_from("<H", data, 4)[0] if len(data) >= 6 else 1
            if addr in self.subs or len(self.subs) < MAX_SUBSCRIBERS:
                self.subs[addr] = (time.monotonic(), max(1, decim))
                self.probing = True
            return
        if len(data) != CMD_SIZE:
            return
        mode, gates, flags, _, d0, d1, d2, load, adv = struct.unpack(CMD_FMT, data)
        if not all(math.isfinite(x) for x in (d0, d1, d2)) or not (math.isfinite(adv) or math.isnan(adv)):
            return                                                # malformed command - ignore
        self.last_ext = time.monotonic()
        self.pkts += 1
        m = self.motor
        if flags & FLAG_RESET:
            self._do_reset()
        if math.isfinite(load):
            self.ctrl[C_LOADT] = load
            self.cfg.load_torque_nm = load
        if MODES[int(self.ctrl[C_MODE])] == "EXT":
            self._coasted = False
            if mode == MODE_GATES:
                m.set_gates(gates & 1, gates & 2, gates & 4, gates & 8, gates & 16, gates & 32)
            else:
                m.set_duty(d0, d1, d2, (gates & 1, gates & 4, gates & 16))
        if adv > 0.0 and not self.ctrl[C_PAUSE]:
            adv = min(adv, MAX_ADVANCE_US)
            self._advance(max(1, int(round(adv * 1e-6 / self.dt))))
            self.last_lock = time.monotonic()
        self._reply(addr)

    def _drain(self):
        got = False
        while True:
            try:
                data, addr = self.sock.recvfrom(512)
            except BlockingIOError:
                return got
            except OSError:              # ICMP port-unreachable from a closed scope etc. - ignore
                continue
            got = True
            self._handle(data, addr)

    # ------------------------------------------------------------ main loop
    def run(self):
        m = self.motor
        wall0, sim0 = time.monotonic(), m.t
        rt_w, rt_s = wall0, sim0
        last_stat = 0.0
        while self.ctrl[C_RUN]:
            now = time.monotonic()
            if now - last_stat > 0.004:
                self._pull_ctrl()
                self._push_stat()
                last_stat = now
            if now - self.last_lock < 0.25:                       # lock-step client in charge of time
                r, _, _ = select.select([self.sock], [], [], 0.005)
                if r:
                    self._drain()
                wall0, sim0 = time.monotonic(), m.t
                rt_w, rt_s = wall0, sim0
                continue
            self._drain()
            if time.monotonic() - self.last_lock < 0.25:          # a lock-step packet just arrived:
                continue                                          # no free-running steps from now on
            if self.ctrl[C_PAUSE]:
                time.sleep(0.005)
                wall0, sim0 = time.monotonic(), m.t
                continue
            if MODES[int(self.ctrl[C_MODE])] == "EXT" and now - self.last_ext > 0.5 and not self._coasted:
                m.coast()                                          # ESC went silent -> gates float
                self._coasted = True
            speed = self.ctrl[C_SPEED]
            behind = sim0 + (now - wall0) * speed - m.t
            if behind > 0.05:                                      # can't keep up: drop real-time debt
                wall0, sim0 = now, m.t
                behind = 100 * self.dt
            n = min(int(behind / self.dt), 200)
            if n <= 0:
                time.sleep(0.0003)
                continue
            self._advance(n)
            if now - rt_w > 0.5:
                self.rtf = (m.t - rt_s) / (now - rt_w)
                rt_w, rt_s = now, m.t


def sim_process(cfg_dict, dt, port, bind, ctrl, stat, hist, cfg_q):
    host = SimHost(MotorConfig(**cfg_dict), dt, port, bind, ctrl, stat, hist, cfg_q)
    host.run()


# =================================================================== UI
BG = (16, 18, 24)
PANEL = (26, 29, 38)
GRID = (48, 52, 64)
TXT = (214, 218, 228)
DIM = (130, 136, 150)
CA, CB, CC = (255, 96, 96), (96, 224, 128), (96, 156, 255)
CN = (200, 200, 200)
CY = (255, 210, 80)
CP = (200, 120, 255)


def nice_range(lo, hi):
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return -1.0, 1.0
    if hi - lo < 1e-9:
        hi = lo + max(1e-3, abs(lo) * 0.1)
    span = hi - lo
    mag = 10 ** math.floor(math.log10(span))
    step = mag * (1 if span / mag < 2 else 2 if span / mag < 5 else 5) / 2
    return math.floor(lo / step) * step, math.ceil(hi / step) * step


class PlotState:
    def __init__(self):
        self.lo, self.hi = 0.0, 1e-3

    def update(self, lo, hi, zero):
        if zero:
            lo, hi = min(lo, 0.0), max(hi, 0.0)
        lo, hi = nice_range(lo, hi)
        self.lo = lo if lo < self.lo else self.lo + 0.08 * (lo - self.lo)
        self.hi = hi if hi > self.hi else self.hi + 0.08 * (hi - self.hi)
        return self.lo, self.hi


class Slider:
    def __init__(self, rect, label, vmin, vmax, idx, ctrl, fmt="{:.3f}", rnd=None):
        self.rect, self.label, self.vmin, self.vmax = rect, label, vmin, vmax
        self.idx, self.ctrl, self.fmt, self.rnd = idx, ctrl, fmt, rnd
        self.drag = False

    def _set(self, x):
        r = self.rect
        f = min(1.0, max(0.0, (x - r.x) / r.w))
        v = self.vmin + f * (self.vmax - self.vmin)
        self.ctrl[self.idx] = round(v, self.rnd) if self.rnd is not None else v

    def event(self, ev, pygame):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and self.rect.inflate(0, 14).collidepoint(ev.pos):
            self.drag = True
            self._set(ev.pos[0])
        elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            self.drag = False
        elif ev.type == pygame.MOUSEMOTION and self.drag:
            self._set(ev.pos[0])
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 3 and self.rect.inflate(0, 14).collidepoint(ev.pos):
            self.ctrl[self.idx] = 0.0 if self.vmin <= 0.0 <= self.vmax else self.vmin   # right-click = zero

    def draw(self, scr, font, pygame):
        r = self.rect
        v = self.ctrl[self.idx]
        f = (v - self.vmin) / (self.vmax - self.vmin) if self.vmax != self.vmin else 0
        f = min(1.0, max(0.0, f))
        pygame.draw.rect(scr, GRID, (r.x, r.centery - 2, r.w, 4), border_radius=2)
        pygame.draw.rect(scr, CY, (r.x, r.centery - 2, int(r.w * f), 4), border_radius=2)
        pygame.draw.circle(scr, CY, (r.x + int(r.w * f), r.centery), 7)
        scr.blit(font.render(f"{self.label}: {self.fmt.format(v)}", True, TXT), (r.x, r.y - 16))


def draw_plot(pygame, scr, font, rect, title, series, state, window_s, zero=True, fmt="{:.1f}"):
    x, y, w, h = rect
    pygame.draw.rect(scr, PANEL, rect)
    pygame.draw.rect(scr, GRID, rect, 1)
    if not series:
        scr.blit(font.render(title, True, DIM), (x + w // 2 - 60, y + 3))
        return
    lo = min(float(np.min(s[0])) for s in series)
    hi = max(float(np.max(s[0])) for s in series)
    lo, hi = state.update(lo, hi, zero)
    for k in range(5):
        gy = y + h - 1 - int(k / 4 * (h - 2))
        pygame.draw.line(scr, GRID, (x, gy), (x + w - 1, gy))
        scr.blit(font.render(fmt.format(lo + (hi - lo) * k / 4), True, DIM), (x + 3, gy + 1 if k == 4 else gy - 14))
    if lo < 0 < hi:
        zy = y + h - 1 - int((0 - lo) / (hi - lo) * (h - 2))
        pygame.draw.line(scr, (90, 96, 112), (x, zy), (x + w - 1, zy))
    scr.blit(font.render(title, True, TXT), (x + w // 2 - 60, y + 3))
    tx = x + 70
    for data, col, label in series:
        n = len(data)
        xs = x + 62 + np.arange(n) * ((w - 66) / max(1, n - 1))
        ys = y + h - 2 - (data - lo) / (hi - lo) * (h - 3)
        pygame.draw.lines(scr, col, False, list(zip(xs.tolist(), ys.tolist())), 1)
        lab = font.render(label, True, col)
        scr.blit(lab, (tx, y + 3))
        tx += lab.get_width() + 12
    scr.blit(font.render(f"{window_s * 1e3:g} ms", True, DIM), (x + w - 64, y + h - 16))


def draw_motor(pygame, scr, font, cfg, pp, st, cx, cy, R):
    slots = max(6, int(cfg.slots))
    imax = max(1e-3, cfg.rated_current_a)
    cols = (CA, CB, CC)
    cur = (st[S_IA], st[S_IB], st[S_IC])
    theta = st[S_TH]
    pygame.draw.circle(scr, (40, 44, 56), (cx, cy), int(R * 1.02))
    for s in range(slots):                      # stator teeth coloured by phase current
        a = TWO_PI * s / slots - PI / 2
        ph = (s % 6) // 2
        sign = 1 if s % 2 == 0 else -1
        k = 0.25 + 0.75 * min(1.0, abs(sign * cur[ph] / imax))
        col = tuple(int(c * k) for c in cols[ph])
        px, py = cx + R * 0.86 * math.cos(a), cy + R * 0.86 * math.sin(a)
        rr = max(5, int(R * 0.09))
        pygame.draw.circle(scr, col, (int(px), int(py)), rr)
        pygame.draw.circle(scr, (20, 22, 30), (int(px), int(py)), rr, 1)
    npole = 2 * pp
    r1, r2 = R * 0.46, R * 0.66
    for p in range(npole):                      # rotor magnets
        a0 = theta + TWO_PI * p / npole - PI / 2
        a1 = a0 + TWO_PI / npole * 0.92
        pts = [(cx + r2 * math.cos(a0 + (a1 - a0) * t / 4), cy + r2 * math.sin(a0 + (a1 - a0) * t / 4)) for t in range(5)]
        pts += [(cx + r1 * math.cos(a1 - (a1 - a0) * t / 4), cy + r1 * math.sin(a1 - (a1 - a0) * t / 4)) for t in range(5)]
        pygame.draw.polygon(scr, (200, 70, 70) if p % 2 == 0 else (70, 100, 200), pts)
    pygame.draw.circle(scr, (60, 64, 78), (cx, cy), int(R * 0.34))
    a = theta - PI / 2
    pygame.draw.line(scr, CY, (cx, cy), (cx + R * 0.34 * math.cos(a), cy + R * 0.34 * math.sin(a)), 3)
    hb = int(st[S_HALL])
    for k in range(3):                          # hall sensors
        hv = (hb >> k) & 1
        a = -PI / 2 + k * TWO_PI / 3 + PI / 6
        px, py = cx + R * 1.12 * math.cos(a), cy + R * 1.12 * math.sin(a)
        pygame.draw.rect(scr, (80, 230, 120) if hv else (60, 64, 76), (px - 9, py - 9, 18, 18), border_radius=3)
        scr.blit(font.render("ABC"[k], True, (10, 10, 10) if hv else DIM), (px - 4, py - 8))


def load_cfg(path):
    path = config_path(path)
    if path and os.path.exists(path):
        return MotorConfig.from_json(path)
    print(f"config {path!r} not found, using built-in defaults")
    return MotorConfig()


def main():
    ap = argparse.ArgumentParser(description="BLDC motor digital twin")
    ap.add_argument("config", nargs="?", default=os.path.join(CONFIG_DIR, "motor_config.json"))
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--dt", type=float, default=10e-6, help="physics step [s] (10us default; <=2.5us for gate-level PWM)")
    ap.add_argument("--demo", type=float, default=None, help="start with the built-in demo ESC at this duty (0..1)")
    ap.add_argument("--frames", type=int, default=0, help="exit after N frames (testing)")
    ap.add_argument("--screenshot", default=None, help="save PNG of the last frame (with --frames)")
    args = ap.parse_args()

    try:
        cfg = load_cfg(args.config)
    except (OSError, TypeError, ValueError) as ex:
        sys.exit(f"cannot load motor config {args.config!r}: {ex}")
    if args.bind not in ("127.0.0.1", "localhost", "::1"):
        print(f"WARNING: listening on {args.bind}:{args.port} - anyone on that network can drive the motor "
              "and request the probe stream. Use only on a trusted network.")
    ui_motor = BLDCMotor(cfg)          # only for derived constants (ke, pole pairs) in the UI
    decim = max(1, round(SAMPLE_DT / args.dt))
    sample_dt = decim * args.dt
    N = int(HIST_S / sample_dt)

    ctrl = mp.Array("d", NCTRL, lock=False)
    stat = mp.Array("d", NSTAT, lock=False)
    hist = mp.RawArray("f", NCH * N)
    cfg_q = mp.Queue()
    ctrl[C_MODE] = 1 if args.demo is not None else 0
    ctrl[C_DUTY] = max(0.0, min(1.0, args.demo or 0.0))
    ctrl[C_DIR] = 1.0
    ctrl[C_SPEED] = 1.0
    ctrl[C_RUN] = 1.0
    ctrl[C_LOADT] = cfg.load_torque_nm
    ctrl[C_FAN] = cfg.load_fan_k
    ctrl[C_LINERT] = cfg.load_inertia_kgm2
    ctrl[C_VBAT] = cfg.battery_voltage_v

    proc = mp.Process(target=sim_process, daemon=True,
                      args=(asdict(cfg), args.dt, args.port, args.bind, ctrl, stat, hist, cfg_q))
    proc.start()
    buf = np.frombuffer(hist, dtype=np.float32).reshape(NCH, N)

    import pygame
    pygame.init()
    W, H = 1280, 720
    scr = pygame.display.set_mode((W, H))
    pygame.display.set_caption("BLDC digital twin")
    font = pygame.font.SysFont("dejavusansmono,consolas,menlo,couriernew,monospace", 12)
    big = pygame.font.SysFont("dejavusansmono,consolas,menlo,couriernew,monospace", 15, bold=True)
    clock = pygame.time.Clock()
    windows = [0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 1.9]
    wi = 3
    states = [PlotState() for _ in range(5)]

    def mk_sliders():
        c = cfg
        tmax = max(0.01, 2.0 * ui_motor.ke0 * c.rated_current_a)        # ~ 1x rated torque
        w_nl = c.battery_voltage_v * c.kv_rpm_per_v / RADS2RPM
        kfan = tmax / (0.8 * w_nl) ** 2                                # full slider = rated torque @80% speed
        x, w, y0, sp = 20, 340, 566, 30
        return [
            Slider(pygame.Rect(x, y0, w, 12), "demo duty", 0.0, 1.0, C_DUTY, ctrl, "{:.2f}", 3),
            Slider(pygame.Rect(x, y0 + sp, w, 12), "load torque [N.m]", -tmax, tmax, C_LOADT, ctrl, "{:+.4f}"),
            Slider(pygame.Rect(x, y0 + 2 * sp, w, 12), "prop/fan k  T=k*w^2", 0.0, kfan, C_FAN, ctrl, "{:.2e}"),
            Slider(pygame.Rect(x, y0 + 3 * sp, w, 12), "load inertia [kg.m2]", 0.0, 50 * c.rotor_inertia_kgm2, C_LINERT, ctrl, "{:.2e}"),
            Slider(pygame.Rect(x, y0 + 4 * sp, w, 12), "battery [V]", 1.0, max(60.0, 2 * c.battery_voltage_v), C_VBAT, ctrl, "{:.1f}", 1),
        ]

    sliders = mk_sliders()
    frames = 0
    running = True
    t_fps, n_fps, fps = time.monotonic(), 0, 0.0
    while running:
        if not proc.is_alive():
            if proc.exitcode and proc.exitcode > 0:      # negative = killed by a signal (normal shutdown)
                print(f"simulation process exited with code {proc.exitcode} "
                      f"(is UDP port {args.port} already in use?)")
            break
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k == pygame.K_ESCAPE:
                    running = False
                elif k == pygame.K_1:
                    ctrl[C_MODE] = 0
                elif k == pygame.K_2:
                    ctrl[C_MODE] = 1
                elif k == pygame.K_0:
                    ctrl[C_MODE] = 2
                elif k == pygame.K_UP:
                    ctrl[C_DUTY] = min(1.0, round(ctrl[C_DUTY] + 0.02, 3))
                elif k == pygame.K_DOWN:
                    ctrl[C_DUTY] = max(0.0, round(ctrl[C_DUTY] - 0.02, 3))
                elif k == pygame.K_SPACE:
                    ctrl[C_DIR] = -ctrl[C_DIR]
                elif k == pygame.K_r:
                    ctrl[C_RESET] += 1
                elif k == pygame.K_p:
                    ctrl[C_PAUSE] = 0.0 if ctrl[C_PAUSE] else 1.0
                elif k == pygame.K_LEFTBRACKET:
                    wi = max(0, wi - 1)
                elif k == pygame.K_RIGHTBRACKET:
                    wi = min(len(windows) - 1, wi + 1)
                elif k == pygame.K_COMMA:
                    ctrl[C_SPEED] = max(1 / 1024, ctrl[C_SPEED] / 2)
                elif k == pygame.K_PERIOD:
                    ctrl[C_SPEED] = min(1.0, ctrl[C_SPEED] * 2)
                elif k == pygame.K_c:
                    try:
                        cfg = load_cfg(args.config)
                        ui_motor = BLDCMotor(cfg)
                        ctrl[C_LOADT], ctrl[C_FAN] = cfg.load_torque_nm, cfg.load_fan_k
                        ctrl[C_LINERT], ctrl[C_VBAT] = cfg.load_inertia_kgm2, cfg.battery_voltage_v
                        cfg_q.put(asdict(cfg))
                        sliders = mk_sliders()
                        states = [PlotState() for _ in range(5)]
                    except Exception as ex:
                        print("config reload failed:", ex)
            for s in sliders:
                s.event(ev, pygame)

        st = list(stat)
        scr.fill(BG)
        # ---- header
        mode = MODES[int(ctrl[C_MODE])]
        mode_txt = {"EXT": f"EXTERNAL ESC  (UDP {args.bind}:{args.port})",
                    "INT": f"DEMO 6-step ESC  ({'fwd' if ctrl[C_DIR] > 0 else 'rev'})",
                    "OFF": "ALL FETs OFF (coast)"}[mode]
        link = ("no ESC packets", "free-running", "LOCK-STEP")[int(st[S_LINK])]
        scr.blit(big.render(cfg.name, True, CY), (14, 8))
        scr.blit(font.render(mode_txt, True, TXT), (14, 28))
        scr.blit(font.render(f"t={st[S_T]:9.4f}s dt={args.dt*1e6:g}us RTF={st[S_RTF]:4.2f} {fps:3.0f}fps", True, DIM), (14, 44))
        scr.blit(font.render(f"link: {link}  pkts={int(st[S_PKTS])}" + ("   [PAUSED]" if ctrl[C_PAUSE] else ""), True,
                             (80, 230, 120) if st[S_LINK] else DIM), (14, 58))
        # ---- motor + readouts
        draw_motor(pygame, scr, font, cfg, ui_motor.pp, st, 190, 200, 100)
        rpm = st[S_OM] * RADS2RPM
        hb = int(st[S_HALL])
        rows = [
            ("RPM", f"{rpm:9.1f}", CY), ("Torque", f"{st[S_TE]:+8.4f}Nm", CY),
            ("Ia", f"{st[S_IA]:+8.2f} A", CA), ("Va", f"{st[S_VA]:7.2f} V", CA),
            ("Ib", f"{st[S_IB]:+8.2f} A", CB), ("Vb", f"{st[S_VB]:7.2f} V", CB),
            ("Ic", f"{st[S_IC]:+8.2f} A", CC), ("Vc", f"{st[S_VC]:7.2f} V", CC),
            ("I bus", f"{st[S_IBUS]:+8.2f} A", CP), ("Vn", f"{st[S_VN]:7.2f} V", CN),
            ("I batt", f"{st[S_IBATT]:+8.2f} A", CP), ("Vbus", f"{st[S_VBUS]:7.2f} V", CP),
            ("Winding", f"{st[S_TW]:7.1f} C", TXT), ("Hall ABC", f"{hb & 1}{(hb >> 1) & 1}{(hb >> 2) & 1}", TXT),
        ]
        yy = 330
        for r in range(0, len(rows), 2):
            for ci in range(2):
                lab, val, col = rows[r + ci]
                scr.blit(font.render(f"{lab:<9}{val}", True, col), (20 + ci * 185, yy))
            yy += 18
        flt = int(st[S_FAULT])
        msg = [n for b, n in ((1, "SHOOT-THROUGH"), (2, "OVER-TEMP"), (4, "BUS-OVERVOLT")) if flt & b]
        scr.blit(font.render("FAULT: " + (" ".join(msg) if msg else "none"), True, (255, 80, 80) if msg else DIM), (20, yy + 2))
        scr.blit(font.render("1 ext  2 demo  0 off | UP/DN duty | SPACE dir", True, DIM), (20, 482))
        scr.blit(font.render("R reset | P pause | [ ] timebase | C reload cfg", True, DIM), (20, 498))
        scr.blit(font.render(f", . slow-mo (x{ctrl[C_SPEED]:g}) | right-click slider = 0", True, DIM), (20, 514))
        for s in sliders:
            s.draw(scr, font, pygame)
        # ---- scopes
        win = windows[wi]
        widx = int(st[S_WIDX])
        n = min(int(win / sample_dt), widx, N)
        px, pw = 395, 875
        ph = (H - 12) // 5
        rects = [(px, 8 + k * ph, pw, ph - 6) for k in range(5)]
        if n >= 4:
            stride = max(1, n // 900)
            hd = buf[:, np.arange(widx - n, widx, stride) % N].copy()
            draw_plot(pygame, scr, font, rects[0], "phase current [A]", [(hd[0], CA, "Ia"), (hd[1], CB, "Ib"), (hd[2], CC, "Ic")], states[0], win)
            draw_plot(pygame, scr, font, rects[1], "terminal voltage [V]", [(hd[3], CA, "Va"), (hd[4], CB, "Vb"), (hd[5], CC, "Vc"), (hd[6], CN, "Vn")], states[1], win)
            draw_plot(pygame, scr, font, rects[2], "speed [rpm]", [(hd[7], CY, "rpm")], states[2], win, fmt="{:.0f}")
            draw_plot(pygame, scr, font, rects[3], "electromagnetic torque [N.m]", [(hd[8], CY, "Te")], states[3], win, fmt="{:.4f}")
            draw_plot(pygame, scr, font, rects[4], "DC link", [(hd[9], CP, "Vbus [V]"), (hd[10], (255, 150, 60), "Ibus [A]")], states[4], win)
        else:
            for k in range(5):
                draw_plot(pygame, scr, font, rects[k], "waiting for data...", [], states[k], win)
        pygame.display.flip()
        clock.tick(60)
        frames += 1
        n_fps += 1
        if time.monotonic() - t_fps > 1.0:
            fps = n_fps / (time.monotonic() - t_fps)
            t_fps, n_fps = time.monotonic(), 0
        if args.frames and frames >= args.frames:
            running = False
            if args.screenshot:
                pygame.image.save(scr, args.screenshot)

    ctrl[C_RUN] = 0.0
    proc.join(timeout=1.0)
    if proc.is_alive():
        proc.terminate()
    pygame.quit()


if __name__ == "__main__":
    mp.freeze_support()
    main()
