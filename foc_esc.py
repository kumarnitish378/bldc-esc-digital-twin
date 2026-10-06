#!/usr/bin/env python3
"""Field-Oriented Control (FOC) ESC for the BLDC digital twin.

The controller (`FOC`) is plain Python with no simulator imports, written the way it would run on an
MCU: one call per PWM period with the sampled phase currents, bus voltage and (optionally) the rotor
angle, returning three PWM duties.

  * Clarke/Park (amplitude invariant), d/q current PI with feed-forward decoupling and anti-windup
  * Space-vector PWM (min/max zero-sequence injection), voltage-circle limiting
  * One-PWM-period computation delay like real hardware (duty written to shadow registers), with
    1.5*Ts angle compensation in the inverse Park transform
  * Speed PI (outer loop, decimated), current-vector limit, separate regen (braking) limit
  * Field weakening (negative id from a voltage-margin integrator)
  * Angle sources: encoder (quantized, with automatic offset alignment) or SENSORLESS
    (Ortega non-linear flux observer + PLL, I/f open-loop start and smooth handover)
  * Motor parameter identification: R (DC injection), L (HF square-wave injection),
    flux linkage (open-loop spin)

Run it against the motor sim like an external ESC (lock-step UDP), publishing its internals to scope12:

    python bldc_sim.py motor_tmotor_u8ii_kv100.json
    python scope12.py                                # optional
    python foc_esc.py --rpm 3000 --seconds 4         # encoder FOC
    python foc_esc.py --sensorless --rpm 3000        # sensorless FOC
"""
import argparse
import math
import socket
import sys
from dataclasses import dataclass, field

TWO_PI = 2.0 * math.pi
SQRT3 = math.sqrt(3.0)
RPM = TWO_PI / 60.0


def wrap(a):
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % TWO_PI - math.pi


@dataclass
class FOCConfig:
    pole_pairs: int = 21
    R: float = 0.087                 # phase resistance as seen by the ESC (winding + FET) [ohm]
    L: float = 37e-6                 # phase inductance, Ld = Lq (surface magnets) [H]
    flux: float = 2.75e-3            # PM flux linkage, phase peak [V.s/rad electrical]
    J: float = 2.66e-3               # total inertia used to tune the speed loop [kg.m2]
    pwm_hz: float = 20000.0          # PWM = control frequency (one sample per period)
    current_bw_hz: float = 1000.0    # current-loop bandwidth
    speed_bw_hz: float = 6.0         # speed-loop bandwidth
    speed_div: int = 10              # speed loop runs every N current-loop periods
    i_max: float = 40.0              # current vector limit |idq| [A peak]
    i_regen_max: float = 20.0        # max braking (negative) iq [A]
    accel_limit_rpm_s: float = 4000  # speed-reference slew limit
    v_margin: float = 0.95           # usable fraction of the SVPWM linear range
    fw_enable: bool = True
    fw_id_min: float = -20.0         # field-weakening current limit [A]
    fw_gain: float = 400.0           # A/(V.s)
    sensorless: bool = False
    observer_gain: float = 0.0       # gamma; 0 = auto (2000 / flux^2)
    pll_bw_hz: float = 80.0
    if_current: float = 15.0         # I/f start current [A]
    if_accel_rpm_s: float = 600.0    # I/f ramp rate (mechanical rpm/s)
    handover_rpm: float = 450.0      # switch from I/f to observer at this speed
    handover_s: float = 0.05         # angle blend time at handover
    encoder_bits: int = 14           # absolute encoder resolution (mechanical)
    align_current: float = 15.0      # encoder offset alignment current
    align_s: float = 1.5             # ramp + hold time for alignment
    delay_comp: bool = True


@dataclass
class _PI:
    kp: float
    ki: float
    lo: float = -1e9
    hi: float = 1e9
    i: float = 0.0

    def step(self, e, ts, ff=0.0):
        u = self.kp * e + self.i + ff
        us = min(self.hi, max(self.lo, u))
        if u == us:
            self.i += self.ki * ts * e
        else:                                        # clamping anti-windup
            self.i = us - self.kp * e - ff
        return us


class FOC:
    """One FOC ESC. Call step() once per PWM period."""

    def __init__(self, cfg: FOCConfig):
        self.c = cfg
        self.ts = 1.0 / cfg.pwm_hz
        self.retune()
        self.reset()

    # ------------------------------------------------------------------ setup
    def retune(self):
        c = self.c
        wc = TWO_PI * c.current_bw_hz
        self.pi_d = _PI(c.L * wc, c.R * wc)
        self.pi_q = _PI(c.L * wc, c.R * wc)
        self.kt = 1.5 * c.pole_pairs * c.flux                    # N.m per A (iq, amplitude invariant)
        ws = TWO_PI * c.speed_bw_hz
        kp = c.J * ws / self.kt                                    # A per rad/s (mechanical)
        self.pi_w = _PI(kp, kp * ws / 4.0, -c.i_regen_max, c.i_max)
        wn = TWO_PI * c.pll_bw_hz
        self.pll_kp, self.pll_ki = 2.0 * 0.9 * wn, wn * wn
        self.gamma = c.observer_gain or 2000.0 / (c.flux * c.flux)

    def reset(self):
        self.mode = "off"            # off | current | speed | align | if | rotate_v | hold_d
        self.id_ref = self.iq_ref = 0.0
        self.rpm_ref = 0.0
        self._rpm_cmd = 0.0          # slew-limited speed reference
        self.theta = 0.0             # electrical angle of the d axis used for control
        self.omega = 0.0             # electrical speed estimate [rad/s]
        self.pll_theta = 0.0
        self.pll_omega = 0.0
        self.enc_offset = None       # encoder electrical offset, found by align()
        self.obs_x = [self.c.flux, 0.0]
        self.obs_theta = 0.0
        self.if_theta = 0.0
        self.if_omega = 0.0
        self.handover_t = None
        self.id_fw = 0.0
        self.vd = self.vq = 0.0
        self.v_abs = 0.0
        self.id = self.iq = 0.0
        self.pending = (0.5, 0.5, 0.5)   # duty computed last period, applied this period
        self.enabled = False
        self._v_applied = (0.0, 0.0)    # alpha/beta voltage the bridge applied during the period just ended
        self._v_out = (0.0, 0.0)        # voltage of the duty being handed to the bridge now
        self._v_next = (0.0, 0.0)       # voltage of the duty computed now (bridge applies it next period)
        self._k = 0
        self.t = 0.0
        self.sat = False
        self.fault = ""

    # ------------------------------------------------------------ commands
    def cmd_off(self):
        self.mode = "off"

    def cmd_current(self, iq, id=0.0):
        self.mode, self.iq_ref, self.id_ref = "current", iq, id

    def cmd_speed(self, rpm):
        if self.mode != "speed":
            self.pi_w.i = self.iq
            self._rpm_cmd = self.omega / self.c.pole_pairs / RPM
        self.mode, self.rpm_ref = "speed", rpm

    def cmd_align(self):
        self.mode, self._align_t0 = "align", self.t

    def cmd_if_start(self, rpm_target):
        """Sensorless start: open-loop rotating current (I/f), then hand over to the observer."""
        self.mode = "if"
        self.if_theta, self.if_omega = 0.0, 0.0
        self.handover_t = None
        self.rpm_ref = rpm_target
        self.obs_x = [self.c.flux, 0.0]

    # ---------------------------------------------------------- transforms
    @staticmethod
    def clarke(ia, ib, ic):
        return (2.0 * ia - ib - ic) / 3.0, (ib - ic) / SQRT3

    @staticmethod
    def park(a, b, th):
        c, s = math.cos(th), math.sin(th)
        return a * c + b * s, -a * s + b * c

    @staticmethod
    def ipark(d, q, th):
        c, s = math.cos(th), math.sin(th)
        return d * c - q * s, d * s + q * c

    @staticmethod
    def svpwm(va, vb, vbus):
        """alpha/beta voltage -> three duties with min/max (third-harmonic-like) injection."""
        a = va
        b = -0.5 * va + 0.5 * SQRT3 * vb
        c = -0.5 * va - 0.5 * SQRT3 * vb
        off = 0.5 * (max(a, b, c) + min(a, b, c))
        inv = 1.0 / max(vbus, 1e-3)
        return tuple(min(1.0, max(0.0, 0.5 + (x - off) * inv)) for x in (a, b, c))

    # ------------------------------------------------------------- angle
    def _pll(self, theta_meas):
        """Type-2 PLL. Predict to the sample instant first, then correct, so pll_theta is the angle AT
        the moment the currents were sampled (no one-period lead/lag)."""
        pred = self.pll_theta + self.ts * self.pll_omega
        e = wrap(theta_meas - pred)
        self.pll_omega += self.pll_ki * self.ts * e
        self.pll_theta = wrap(pred + self.ts * self.pll_kp * e)

    def _observer(self, ia, ib):
        """Ortega et al. non-linear flux observer (as used by VESC)."""
        c = self.c
        va, vb = self._v_applied
        ea, eb = self.obs_x[0] - c.L * ia, self.obs_x[1] - c.L * ib
        err = c.flux * c.flux - (ea * ea + eb * eb)
        g = 0.5 * self.gamma * err
        self.obs_x[0] += self.ts * (va - c.R * ia + g * ea)
        self.obs_x[1] += self.ts * (vb - c.R * ib + g * eb)
        ea, eb = self.obs_x[0] - c.L * ia, self.obs_x[1] - c.L * ib
        self.obs_theta = math.atan2(eb, ea)

    # ---------------------------------------------------------------- step
    def step(self, t, ia, ib, ic, vbus, theta_mech=None):
        """One PWM period. Returns ((da, db, dc), enabled) to apply during the NEXT period."""
        c = self.c
        ts = self.ts
        self.t = t
        out_now, en_now = self.pending, self.enabled
        # timeline: duty computed at call k-1 is returned at call k and applied during [t_k, t_k+1]
        self._v_applied = self._v_out
        self._v_out = self._v_next if en_now else (0.0, 0.0)
        ial, ibe = self.clarke(ia, ib, ic)

        # ---------- protection
        if abs(ia) > 1.6 * c.i_max or abs(ib) > 1.6 * c.i_max or abs(ic) > 1.6 * c.i_max:
            self.fault, self.mode = "overcurrent", "off"

        # ---------- angle / speed
        if theta_mech is not None:
            q = TWO_PI / (1 << c.encoder_bits)
            th_m = math.floor(theta_mech / q) * q            # encoder quantization
            raw = wrap(c.pole_pairs * th_m)
            if self.enc_offset is not None:
                self._pll(wrap(raw + self.enc_offset))
        if c.sensorless:
            self._observer(ial, ibe)
            if self.mode not in ("if", "off", "hold_d", "rotate_v", "align") or self.handover_t is not None:
                self._pll(self.obs_theta)
            elif self.mode == "if":
                self._pll(self.obs_theta)                    # keep the PLL locked for the handover

        mode = self.mode
        if mode == "off":
            self.enabled = False
            self.pending = (0.5, 0.5, 0.5)
            self._v_next = (0.0, 0.0)
            self.pi_d.i = self.pi_q.i = 0.0
            self.id = self.iq = 0.0
            return out_now, en_now

        if mode == "align":
            # ramp a d-axis current at a fixed angle; rotor's flux aligns with it
            tt = t - self._align_t0
            self.theta, self.omega = 0.0, 0.0
            self.id_ref = c.align_current * min(1.0, tt / (0.6 * c.align_s))
            self.iq_ref = 0.0
            if tt >= c.align_s and theta_mech is not None:
                self.enc_offset = wrap(0.0 - wrap(c.pole_pairs * theta_mech))
                self.pll_theta, self.pll_omega = 0.0, 0.0
                self.mode = "current"
                self.id_ref = self.iq_ref = 0.0
        elif mode == "if":
            # open-loop forced angle, current on q of the forced frame
            self.if_omega = min(self.if_omega + c.if_accel_rpm_s * RPM * c.pole_pairs * ts,
                                c.handover_rpm * RPM * c.pole_pairs * 1.05)
            self.if_theta = wrap(self.if_theta + self.if_omega * ts)
            self.id_ref, self.iq_ref = 0.0, c.if_current
            if self.handover_t is None and self.if_omega >= c.handover_rpm * RPM * c.pole_pairs:
                self.handover_t = t
            if self.handover_t is not None:
                w = min(1.0, (t - self.handover_t) / c.handover_s)
                self.theta = wrap(self.if_theta + w * wrap(self.pll_theta - self.if_theta))
                self.omega = self.if_omega + w * (self.pll_omega - self.if_omega)
                if w >= 1.0:                                  # observer in charge -> speed control
                    self.mode = "speed"
                    self.pi_w.i = c.if_current
                    self._rpm_cmd = self.pll_omega / c.pole_pairs / RPM
            else:
                self.theta, self.omega = self.if_theta, self.if_omega
        elif mode in ("hold_d", "rotate_v"):
            pass                                              # angle/voltage set by identification
        else:
            self.theta, self.omega = self.pll_theta, self.pll_omega

        # ---------- outer speed loop
        if self.mode == "speed":
            self._k += 1
            if self._k >= c.speed_div:
                self._k = 0
                dt = ts * c.speed_div
                step = c.accel_limit_rpm_s * dt
                self._rpm_cmd += max(-step, min(step, self.rpm_ref - self._rpm_cmd))
                w_err = (self._rpm_cmd - self.omega / c.pole_pairs / RPM) * RPM
                iq_hi = math.sqrt(max(0.0, c.i_max ** 2 - self.id_fw ** 2))
                self.pi_w.lo, self.pi_w.hi = -min(c.i_regen_max, iq_hi), iq_hi
                self.iq_ref = self.pi_w.step(w_err, dt)
            self.id_ref = self.id_fw

        # ---------- current loop
        self.id, self.iq = self.park(ial, ibe, self.theta)
        vmax = c.v_margin * vbus / SQRT3
        if self.mode == "rotate_v":
            vd, vq = self.vd, self.vq
        else:
            idr, iqr = self.id_ref, self.iq_ref
            mag = math.hypot(idr, iqr)
            if mag > c.i_max:
                idr, iqr = idr * c.i_max / mag, iqr * c.i_max / mag
            we = self.omega
            self.pi_d.lo, self.pi_d.hi = -vmax, vmax
            vd = self.pi_d.step(idr - self.id, ts, ff=-we * c.L * self.iq)
            vq_lim = math.sqrt(max(0.0, vmax * vmax - vd * vd))
            self.pi_q.lo, self.pi_q.hi = -vq_lim, vq_lim
            vq_unsat = self.pi_q.kp * (iqr - self.iq) + self.pi_q.i + we * (c.L * self.id + c.flux)
            vq = self.pi_q.step(iqr - self.iq, ts, ff=we * (c.L * self.id + c.flux))
            self.sat = abs(vq_unsat) > vq_lim
            if c.fw_enable and self.mode == "speed":
                excess = math.hypot(vd, vq_unsat) - 0.97 * vmax
                self.id_fw = min(0.0, max(c.fw_id_min, self.id_fw - c.fw_gain * ts * excess))
            self.vd, self.vq = vd, vq
        self.v_abs = math.hypot(vd, vq)

        # ---------- modulation (applied next period -> advance the angle)
        th_out = self.theta + (1.5 * self.omega * ts if c.delay_comp else 0.0)
        va, vb = self.ipark(vd, vq, th_out)
        self.pending = self.svpwm(va, vb, vbus)
        self.enabled = True
        da, db, dc = self.pending
        self._v_next = (vbus * (2 * da - db - dc) / 3.0, vbus * (db - dc) / SQRT3)
        return out_now, en_now

    # ----------------------------------------------------------- telemetry
    def rpm(self):
        return self.omega / self.c.pole_pairs / RPM


# ======================================================================= parameter identification
@dataclass
class IdentResult:
    R: float = 0.0
    L: float = 0.0
    flux: float = 0.0
    log: list = field(default_factory=list)


def identify(foc: FOC, plant_step, i_test=12.0):
    """Measure R, L and flux linkage like a real ESC's 'motor detection'.

    plant_step(duties, enabled) -> (t, ia, ib, ic, vbus, theta_mech) advances one PWM period.
    Uses only electrical measurements (no encoder).
    """
    c, ts = foc.c, foc.ts
    res = IdentResult()

    def run(n, collect=None):
        meas = plant_step(foc.pending, foc.enabled)
        for _ in range(n):
            out, en = foc.step(*meas[:5])
            meas = plant_step(out, en)
            if collect is not None:
                collect(meas)
        return meas

    # --- R: DC current on the d axis at a fixed angle (rotor aligns, no torque once aligned)
    foc.mode = "hold_d"
    foc.theta, foc.omega = 0.0, 0.0
    foc.id_ref, foc.iq_ref = i_test, 0.0
    run(int(1.5 / ts))                                   # align + settle (prop inertia)
    vs, cs = [], []
    run(int(0.2 / ts), lambda m: (vs.append(foc.vd), cs.append(foc.id)))
    res.R = sum(vs) / sum(cs)

    # --- L: square-wave voltage on the d axis around the DC point, L = (v - R i) / (di/dt)
    foc.mode = "rotate_v"
    v0 = res.R * i_test
    amp = 2.0                                            # V, gives a few A of ripple
    half = 4                                             # 4 periods per half-wave (2.5 kHz at 20 kHz PWM)
    last = plant_step(foc.pending, foc.enabled)
    vds, ids = [], []
    for k in range(160):
        foc.vd, foc.vq = v0 + (amp if (k // half) % 2 == 0 else -amp), 0.0
        vds.append(foc.vd)
        out, en = foc.step(*last[:5])
        last = plant_step(out, en)
        ids.append(foc.id)
    # ids[k] = current at the START of call k; period k-1 applied the voltage commanded at call k-2
    est = []
    for k in range(8, len(ids)):
        di = ids[k] - ids[k - 1]
        if abs(di) > 1e-3:
            est.append((vds[k - 2] - res.R * 0.5 * (ids[k] + ids[k - 1])) * ts / di)
    est.sort()
    res.L = est[len(est) // 2]

    # --- flux: open-loop rotating current (I/f), steady speed, |v - R i - jwL i| / w
    foc.c.R, foc.c.L = res.R, res.L
    foc.retune()
    foc.mode = "if"
    foc.if_theta, foc.if_omega, foc.handover_t = 0.0, 0.0, None
    saved = foc.c.sensorless, foc.c.handover_rpm
    foc.c.sensorless, foc.c.handover_rpm = False, 1e9
    target = 900.0 * RPM * c.pole_pairs
    acc = 400.0 * RPM * c.pole_pairs
    last = plant_step(foc.pending, foc.enabled)
    est = []
    t_end = 4.0
    n = int(t_end / ts)
    for k in range(n):
        foc.if_omega = min(foc.if_omega + acc * ts, target)
        foc.mode = "if"
        foc.c.if_accel_rpm_s = 0.0
        out, en = foc.step(*last[:5])
        last = plant_step(out, en)
        if k > n - int(0.5 / ts):
            w = foc.if_omega
            ed = foc.vd - res.R * foc.id + w * res.L * foc.iq
            eq = foc.vq - res.R * foc.iq - w * res.L * foc.id
            est.append(math.hypot(ed, eq) / w)
    res.flux = sum(est) / len(est)
    # ramp the open-loop speed back to zero so the motor is at standstill when detection ends
    while foc.if_omega > 0.0:
        foc.if_omega = max(0.0, foc.if_omega - acc * ts)
        out, en = foc.step(*last[:5])
        last = plant_step(out, en)
    foc.mode = "hold_d"
    foc.theta, foc.omega, foc.id_ref, foc.iq_ref = foc.if_theta, 0.0, i_test, 0.0
    for _ in range(int(0.5 / ts)):
        out, en = foc.step(*last[:5])
        last = plant_step(out, en)
    foc.c.sensorless, foc.c.handover_rpm = saved
    foc.c.if_accel_rpm_s = FOCConfig.if_accel_rpm_s
    foc.c.flux = res.flux
    foc.retune()
    foc.mode = "off"
    for _ in range(int(0.02 / ts)):
        out, en = foc.step(*last[:5])
        last = plant_step(out, en)
    return res


# ======================================================================= UDP runner (external ESC)
def main():
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from bldc_protocol import DEFAULT_PORT, MODE_DUTY, FLAG_RESET, REPLY_SIZE, pack_cmd, unpack_reply
    from scope_probe import ScopeProbe

    ap = argparse.ArgumentParser(description="FOC ESC talking to bldc_sim.py over UDP (lock-step)")
    ap.add_argument("--rpm", type=float, default=3000.0)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--sensorless", action="store_true")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()

    foc = FOC(FOCConfig(sensorless=args.sensorless))
    ts_us = foc.ts * 1e6
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect(("127.0.0.1", args.port))
    sock.settimeout(2.0)
    probe = ScopeProbe("foc")

    def exchange(duty, en, flags=0):
        sock.send(pack_cmd(MODE_DUTY, 0x15 if en else 0, flags=flags, duty=duty, advance_us=ts_us))
        return unpack_reply(sock.recv(REPLY_SIZE))

    try:
        fb = exchange((0.5, 0.5, 0.5), False, FLAG_RESET)
    except (socket.timeout, ConnectionError):
        sys.exit(f"no reply from the motor sim on 127.0.0.1:{args.port} - start: python bldc_sim.py "
                 "motor_tmotor_u8ii_kv100.json")
    t0 = fb["t"]
    if args.sensorless:
        foc.cmd_if_start(args.rpm)
    else:
        foc.cmd_align()
    started = False
    next_print = 0.0
    while fb["t"] - t0 < args.seconds:
        th = None if args.sensorless else fb["theta"]
        duty, en = foc.step(fb["t"], fb["ia"], fb["ib"], fb["ic"], fb["v_bus"], th)
        if not started and foc.mode == "current" and foc.enc_offset is not None:
            foc.cmd_speed(args.rpm)
            started = True
        th_true = wrap(21 * fb["theta"] - math.pi)
        probe.sample(fb["t"], **{"id[A]": foc.id, "iq[A]": foc.iq, "iq_ref[A]": foc.iq_ref,
                                 "id_ref[A]": foc.id_ref, "vd[V]": foc.vd, "vq[V]": foc.vq,
                                 "rpm_est[rpm]": foc.rpm(), "theta_ctrl[rad]": foc.theta,
                                 "theta_err[rad]": wrap(foc.theta - th_true)})
        try:
            fb = exchange(duty, en)
        except (socket.timeout, ConnectionError):
            sys.exit("motor sim stopped responding")
        t = fb["t"] - t0
        if t >= next_print:
            print(f"t={t:6.3f}s mode={foc.mode:7s} rpm={fb['rpm']:7.1f} est={foc.rpm():7.1f} "
                  f"id={foc.id:+6.2f} iq={foc.iq:+6.2f} Ibus={fb['i_bus']:+6.2f} Vbus={fb['v_bus']:5.2f}")
            next_print += 0.25
    probe.close()
    exchange((0.5, 0.5, 0.5), False)


if __name__ == "__main__":
    main()
