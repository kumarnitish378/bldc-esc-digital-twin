import numpy as np
import pytest

from scope.scope_core import (NCH, VDIVS, Source, ceil125, default_setup, eng, find_trigger, measure,
                        peak_decimate, sanitize_setup, seq125)


def test_seq125():
    assert seq125(1e-3, 1) == [0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0]
    assert ceil125(0.72) == 1.0 and ceil125(0.2) == 0.2


def test_eng_format():
    assert eng(0.0005, "s") == "500 µs"
    assert eng(12000, "rpm") == "12 krpm"
    assert eng(float("nan")) == "--"


def test_trigger_is_interpolated_and_latest():
    t = np.arange(0, 0.01, 1e-5)
    v = np.sin(2 * np.pi * 1000 * t)                      # rising zero crossings every 1 ms
    tt = find_trigger(t, v, 0.0, 0, t[-1] - 0.002, 0.1)
    assert tt == pytest.approx(0.007, abs=1e-7)           # newest crossing before the limit, sub-sample exact
    assert find_trigger(t, v, 5.0, 0, t[-1], 0.1) is None  # level never reached
    tf = find_trigger(t, v, 0.0, 1, t[-1] - 0.002, 0.1)
    assert tf == pytest.approx(0.0075, abs=1e-7)          # falling edge


def test_trigger_respects_single_arm_time():
    t = np.arange(0, 0.01, 1e-5)
    v = np.sin(2 * np.pi * 1000 * t)
    assert find_trigger(t, v, 0.0, 0, 0.0095, 0.1, after=0.0095) is None


def test_peak_decimate_keeps_narrow_glitch():
    x = np.linspace(0, 1, 100_000)
    y = np.zeros_like(x)
    y[54_321] = 7.0                                       # one-sample glitch
    X, Y = peak_decimate(x, y, 0, 1, 500)
    assert len(X) <= 1000 and Y.max() == 7.0


def test_measure_pwm():
    t = np.arange(0, 0.02, 1e-6)
    v = ((t * 1000) % 1 < 0.3).astype(float)              # 1 kHz, 30 %
    m = measure(t, v)
    assert m["freq"] == pytest.approx(1000, rel=1e-3)
    assert m["duty"] == pytest.approx(0.3, abs=2e-3)
    assert m["pp"] == 1.0


def test_measure_sine_rms():
    t = np.arange(0, 0.1, 1e-5)
    m = measure(t, 2.0 * np.sin(2 * np.pi * 50 * t) + 1.0)
    assert m["rms"] == pytest.approx(np.sqrt(1 + 2), rel=1e-3)
    assert m["mean"] == pytest.approx(1.0, abs=1e-3)


def test_source_buffer_compaction_and_reset():
    s = Source("x", ["a"], cap=1000)
    for k in range(50):                                   # 5000 samples through a 1000-sample memory
        t = np.arange(k * 100, (k + 1) * 100) * 1e-3
        s.append(t, t[None, :].astype(np.float32))
    t, d = s.view()
    assert len(t) == 1000 and t[-1] == pytest.approx(4.999) and np.all(np.diff(t) > 0)
    np.testing.assert_allclose(d[0], t, rtol=1e-6)
    s.append(np.array([0.0, 0.001]), np.zeros((1, 2), np.float32))   # time went back: sim reset
    assert len(s.view()[0]) == 2


def test_source_counts_gaps():
    s = Source("x", ["a"], cap=1000)
    s.append(np.arange(0, 100) * 1e-3, np.zeros((1, 100), np.float32))
    s.append(np.arange(200, 300) * 1e-3, np.zeros((1, 100), np.float32))
    assert s.gaps == 1


def test_sanitize_setup_repairs_bad_values():
    st = sanitize_setup({"ch": [{"vdiv": 0, "pos": 99, "coup": "XX", "key": "sim/ia[A]"}], "tdiv": -1,
                         "trig_ch": 42, "tmode": 2, "edge": "?"})
    assert len(st["ch"]) == NCH
    c0 = st["ch"][0]
    assert c0["vdiv"] in VDIVS and c0["pos"] == 4.0 and c0["coup"] == "DC" and c0["key"] == "sim/ia[A]"
    d = default_setup()
    assert st["tdiv"] == d["tdiv"] and st["trig_ch"] == d["trig_ch"] and st["tmode"] == 0 and st["edge"] == 0
