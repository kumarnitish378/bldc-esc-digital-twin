"""In-process test bench: FOC ESC + BLDC digital twin, one PWM period per exchange (same as UDP lock-step).

Used by foc_tests.py to produce the FOC test report. Records the ESC's view and the motor's ground truth.
"""
import math
import random

import numpy as np

from bldc_model import BLDCMotor, MotorConfig, RADS2RPM
from foc_esc import FOC, wrap


class Plant:
    def __init__(self, cfg: MotorConfig, ts, dt=5e-6, noise=True, seed=1):
        self.m = BLDCMotor(cfg)
        self.n = max(1, round(ts / dt))
        self.dt = ts / self.n
        self.noise = cfg.current_sensor_noise_a if noise else 0.0
        self.rng = random.Random(seed)
        self.avg = (0.0, 0.0, 0.0, 0.0)

    def step(self, duties, enabled):
        m = self.m
        if enabled:
            m.set_duty(*duties)
        else:
            m.coast()
        sb = sp = st = sl = 0.0
        for _ in range(self.n):
            m.step(self.dt)
            sb += m.ibatt
            sp += m.ibus
            st += m.Te
            sl += m.Tload
        # period averages = what a DC power meter / torque sensor reads (instantaneous values alias)
        k = 1.0 / self.n
        self.avg = (sb * k, sp * k, st * k, sl * k)
        g, s = self.rng.gauss, self.noise
        n = (g(0, s), g(0, s), g(0, s)) if s else (0.0, 0.0, 0.0)
        return m.t, m.i[0] + n[0], m.i[1] + n[1], m.i[2] + n[2], m.vbus, m.theta


FIELDS = ("t", "rpm", "rpm_est", "id", "iq", "id_ref", "iq_ref", "vd", "vq", "v_abs", "vmax", "theta_err",
          "Te", "Tload", "ibus", "ibatt", "vbus", "ia", "ib", "ic", "Tw", "rpm_ref", "mode")


class Bench:
    def __init__(self, cfg: MotorConfig, foc: FOC, encoder=True, dt=5e-6, noise=True):
        self.cfg, self.foc, self.encoder = cfg, foc, encoder
        self.plant = Plant(cfg, foc.ts, dt, noise)
        self.meas = self.plant.step(foc.pending, False)
        self.log = {k: [] for k in FIELDS}

    @property
    def motor(self):
        return self.plant.m

    def run(self, seconds, events=(), record_every=1):
        """events: list of (time_s, callable(bench)) fired once when sim time passes time_s."""
        foc, m, pp = self.foc, self.plant.m, self.plant.m.pp
        ev = sorted(events, key=lambda e: e[0])
        t_end = m.t + seconds
        k = 0
        L = self.log
        while m.t < t_end - 1e-12:
            while ev and m.t >= ev[0][0]:
                ev.pop(0)[1](self)
            t, ia, ib, ic, vbus, th = self.meas
            out, en = foc.step(t, ia, ib, ic, vbus, th if self.encoder else None)
            self.meas = self.plant.step(out, en)
            k += 1
            if k % record_every == 0:
                th_d = wrap(pp * th - math.pi)               # true d-axis angle AT THE SAMPLING INSTANT
                vals = (m.t, m.omega * RADS2RPM, foc.rpm(), foc.id, foc.iq, foc.id_ref, foc.iq_ref,
                        foc.vd, foc.vq, foc.v_abs, foc.c.v_margin * vbus / math.sqrt(3),
                        wrap(foc.theta - th_d), m.Te, self.plant.avg[3], self.plant.avg[1], self.plant.avg[0], m.vbus,
                        m.i[0], m.i[1], m.i[2], m.Tw, foc._rpm_cmd if foc.mode == "speed" else float("nan"),
                        foc.mode)
                for f, v in zip(FIELDS, vals):
                    L[f].append(v)
        return self

    def arrays(self):
        return {k: (np.array(v) if k != "mode" else v) for k, v in self.log.items()}

    def plant_step(self, duties, enabled):
        """For identify(): advance one period, return the measurement tuple."""
        self.meas = self.plant.step(duties, enabled)
        return self.meas


def new_bench(cfg_path="motor_tmotor_u8ii_kv100.json", foc_cfg=None, encoder=True, noise=True, **cfg_over):
    from foc_esc import FOCConfig
    cfg = MotorConfig.from_json(cfg_path)
    for k, v in cfg_over.items():
        setattr(cfg, k, v)
    foc = FOC(foc_cfg or FOCConfig())
    return Bench(cfg, foc, encoder=encoder, noise=noise)
