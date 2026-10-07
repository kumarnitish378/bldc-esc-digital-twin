import math

import numpy as np
import pytest

from esc.foc_bench import new_bench
from esc.foc_esc import FOC, FOCConfig, wrap

CFG = "motor_tmotor_u8ii_kv100.json"


def test_clarke_park_roundtrip():
    for th in np.linspace(-3, 3, 13):
        for ang in np.linspace(0, 6, 7):
            ia, ib, ic = (10 * math.cos(ang - k * 2 * math.pi / 3) for k in range(3))
            a, b = FOC.clarke(ia, ib, ic)
            assert math.hypot(a, b) == pytest.approx(10.0)            # amplitude invariant
            d, q = FOC.park(a, b, th)
            a2, b2 = FOC.ipark(d, q, th)
            assert (a2, b2) == pytest.approx((a, b))


def test_svpwm_reaches_linear_limit_without_clipping():
    vbus = 48.0
    vmax = vbus / math.sqrt(3)
    for ang in np.linspace(0, 2 * math.pi, 37):
        d = FOC.svpwm(vmax * math.cos(ang), vmax * math.sin(ang), vbus)
        assert min(d) >= -1e-9 and max(d) <= 1 + 1e-9
        a, b = FOC.clarke(*(vbus * x for x in d))                  # zero sequence cancels
        assert math.hypot(a, b) == pytest.approx(vmax, rel=1e-6)


def test_pll_tracks_without_lead_or_lag():
    f = FOC(FOCConfig())
    w = 2 * math.pi * 1000
    for k in range(4000):
        f._pll(wrap(w * k * f.ts))
    assert abs(wrap(f.pll_theta - w * 3999 * f.ts)) < 1e-6
    assert f.pll_omega == pytest.approx(w, rel=1e-6)


def test_current_step_fast_and_decoupled():
    b = new_bench(CFG, noise=False)
    b.foc.enc_offset = math.pi
    b.foc.cmd_current(0.0)
    b.run(0.02)                                                     # let the PLL lock first
    b.log = {k: [] for k in b.log}
    b.foc.cmd_current(10.0)
    b.run(0.003)
    a = b.arrays()
    assert a["iq"][-1] == pytest.approx(10.0, abs=0.2)
    assert np.max(np.abs(a["id"])) < 0.5
    assert np.argmax(a["iq"] > 9.0) * b.foc.ts < 0.0006                # < 0.6 ms to 90 %


def test_encoder_speed_control_holds_speed():
    b = new_bench(CFG, noise=False)
    b.foc.enc_offset = math.pi
    b.foc.cmd_speed(1500)
    b.run(1.2)
    a = b.arrays()
    assert a["rpm"][-1] == pytest.approx(1500, abs=2)
    assert abs(a["theta_err"][-1]) < 0.01
    assert a["Tload"][-1] == pytest.approx(1.8267e-5 * (1500 * math.pi / 30) ** 2, rel=0.02)


def test_sensorless_start_and_lock():
    b = new_bench(CFG, FOCConfig(sensorless=True), encoder=False, noise=False)
    b.foc.cmd_if_start(1200)
    b.run(1.6)
    a = b.arrays()
    assert a["mode"][-1] == "speed"
    assert a["rpm"][-1] == pytest.approx(1200, abs=10)
    assert abs(np.degrees(a["theta_err"][-1])) < 2.0
