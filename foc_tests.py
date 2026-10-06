#!/usr/bin/env python3
"""FOC ESC test campaign on the T-Motor U8 II KV100 digital twin.

    python foc_tests.py                 # all tests, ~3-5 min; writes docs/foc_report/ (PNG plots + results.json)

Every test is a closed-loop simulation: FOC ESC <-> motor + inverter + battery + G28x9.2 prop, one
exchange per 50 us PWM period, with current-sensor noise. The ESC only sees what a real ESC sees
(phase currents, bus voltage, encoder or nothing); ground truth is logged next to it.
"""
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bldc_model import BLDCMotor, MotorConfig, SECTOR_FWD, RADS2RPM          # noqa: E402
from foc_bench import new_bench                                               # noqa: E402
from foc_esc import FOCConfig, identify                                       # noqa: E402

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor_tmotor_u8ii_kv100.json")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs", "foc_report")
# T-Motor U8II KV100, G28x9.2 CF prop, 48 V: (throttle %, thrust g, torque N.m, current A, rpm, power W)
DATASHEET = [(40, 2243, 0.77, 3.50, 1896, 168), (50, 3231, 1.08, 6.00, 2268, 288), (60, 4201, 1.35, 8.90, 2581, 427),
             (70, 5153, 1.60, 12.20, 2858, 586), (80, 6171, 1.87, 16.20, 3122, 778), (90, 7329, 2.21, 21.40, 3379, 1027),
             (100, 8716, 2.73, 29.30, 3709, 1406)]
R = {}            # results


# ------------------------------------------------------------------------------------------------ plotting
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.dpi": 110, "savefig.dpi": 140, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold",
        "axes.edgecolor": "#c3c2b7", "axes.labelcolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e",
        "axes.grid": True, "grid.color": "#e8e7e1", "grid.linewidth": 0.8, "axes.spines.top": False,
        "axes.spines.right": False, "lines.linewidth": 1.6, "legend.frameon": False, "legend.fontsize": 8.5,
        "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "text.color": "#0b0b0b"})
    return plt


C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"      # categorical slots 1-4 (validated order)
# Te is logged instantaneously (for ripple); Tload, ibus, ibatt are PWM-period averages
GREY = "#8a8985"


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name))
    import matplotlib.pyplot as plt
    plt.close(fig)


def win(a, t0, t1):
    m = (a["t"] >= t0) & (a["t"] <= t1)
    return {k: (v[m] if not isinstance(v, list) else [x for x, keep in zip(v, m) if keep]) for k, v in a.items()}


def step_metrics(t, y, t0, y0, y1):
    """Rise 10-90 %, overshoot %, 2 % settling time for a step y0 -> y1 at t0."""
    span = y1 - y0
    n = (y - y0) / span
    after = t >= t0
    tt, nn = t[after] - t0, n[after]
    t10 = tt[np.argmax(nn >= 0.1)]
    t90 = tt[np.argmax(nn >= 0.9)]
    over = max(0.0, (np.max(nn) - 1.0) * 100)
    outside = np.flatnonzero(np.abs(nn - 1.0) > 0.02)
    settle = tt[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(tt) else 0.0
    return dict(rise_s=float(t90 - t10), overshoot_pct=float(over), settle_s=float(settle))


# ------------------------------------------------------------------------------------------------ tests
def bench_ready(sensorless=False, **over):
    """Bench with identified parameters and (encoder mode) an aligned encoder."""
    fc = FOCConfig(sensorless=sensorless, R=R["ident"]["R"], L=R["ident"]["L"], flux=R["ident"]["flux"])
    b = new_bench(CFG, fc, encoder=not sensorless, **over)
    return b


def t_identification():
    b = new_bench(CFG, FOCConfig())
    m = b.motor
    true_R = m.cfg.phase_resistance_ohm * (1 + m.cfg.copper_alpha * (m.cfg.ambient_c - 25)) + m.cfg.mosfet_rds_on_ohm
    true_L = m.cfg.phase_inductance_h
    true_flux = m.ke0 / m.pp
    r = identify(b.foc, b.plant_step)
    R["ident"] = dict(R=r.R, L=r.L, flux=r.flux, R_true=true_R, L_true=true_L, flux_true=true_flux,
                      R_err_pct=100 * (r.R / true_R - 1), L_err_pct=100 * (r.L / true_L - 1),
                      flux_err_pct=100 * (r.flux / true_flux - 1),
                      kv_from_flux=60 / (2 * math.pi) / (r.flux * m.pp * 3 * math.sqrt(3) / math.pi))
    # encoder alignment on the same (now stationary) motor
    b.foc.cmd_align()
    b.run(b.foc.c.align_s + 0.01)
    R["align"] = dict(offset_err_deg=math.degrees(abs(((b.foc.enc_offset - math.pi) + math.pi) % (2 * math.pi) - math.pi)))


def t_current_step():
    b = bench_ready()
    b.foc.enc_offset = math.pi
    b.foc.cmd_current(0.0)
    b.run(0.02)
    t0 = b.motor.t
    b.run(0.01, events=[(t0 + 0.002, lambda bb: bb.foc.cmd_current(10.0))])
    a = b.arrays()
    w = win(a, t0, t0 + 0.01)
    met = step_metrics(w["t"], w["iq"], t0 + 0.002, 0.0, 10.0)
    met["id_peak_coupling_A"] = float(np.max(np.abs(w["id"])))
    R["current_step_standstill"] = met
    # decoupling at speed: run at 3000 rpm, then step iq by +6 A in current mode
    b2 = bench_ready()
    b2.foc.enc_offset = math.pi
    b2.foc.cmd_speed(3000)
    b2.run(3.0)
    iq0 = float(np.mean(b2.arrays()["iq"][-200:]))
    t1 = b2.motor.t
    b2.foc.cmd_current(iq0)
    b2.run(0.01, events=[(t1 + 0.002, lambda bb: bb.foc.cmd_current(iq0 + 6.0))])
    a2 = b2.arrays()
    w2 = win(a2, t1, t1 + 0.01)
    met2 = step_metrics(w2["t"], w2["iq"], t1 + 0.002, iq0, iq0 + 6.0)
    met2["id_peak_coupling_A"] = float(np.max(np.abs(w2["id"])))
    met2["rpm"] = 3000
    R["current_step_3000rpm"] = met2
    plt = _plt()
    fig, ax = plt.subplots(1, 2, figsize=(9.5, 3.2), sharey=False)
    for k, (ww, tt0, title, base) in enumerate([(w, t0, "Standstill: iq 0 → 10 A", 0.0),
                                                 (w2, t1, "3000 rpm: iq +6 A (decoupling)", iq0)]):
        x = (ww["t"] - tt0 - 0.002) * 1e3
        ax[k].plot(x, ww["iq_ref"], color=GREY, lw=1.2, ls="--", label="iq ref")
        ax[k].plot(x, ww["iq"], color=C1, label="iq")
        ax[k].plot(x, ww["id"] + (base if False else 0), color=C2, label="id")
        ax[k].set_title(title)
        ax[k].set_xlabel("time from step [ms]")
        ax[k].set_ylabel("current [A]")
        ax[k].set_xlim(-0.5, 4)
        ax[k].legend(loc="center right")
    save(fig, "01_current_step.png")


def t_speed_step():
    b = bench_ready()
    b.foc.enc_offset = math.pi
    b.foc.cmd_speed(1500)
    b.run(3.0)
    t0 = b.motor.t
    b.run(2.0, events=[(t0 + 0.2, lambda bb: bb.foc.cmd_speed(3000))])
    a = win(b.arrays(), t0, t0 + 2.0)
    met = step_metrics(a["t"], a["rpm"], t0 + 0.2, 1500, 3000)
    met["iq_peak_A"] = float(np.max(a["iq"]))
    met["accel_limit_rpm_s"] = b.foc.c.accel_limit_rpm_s
    met["tracking_err_rpm_rms_during_ramp"] = float(np.sqrt(np.nanmean(
        (a["rpm"] - a["rpm_ref"])[(a["t"] > t0 + 0.25) & (a["t"] < t0 + 0.55)] ** 2)))
    R["speed_step"] = met
    plt = _plt()
    fig, ax = plt.subplots(2, 1, figsize=(7.5, 4.6), sharex=True)
    x = a["t"] - t0 - 0.2
    ax[0].plot(x, a["rpm_ref"], color=GREY, lw=1.2, ls="--", label="speed ref (slew limited)")
    ax[0].plot(x, a["rpm"], color=C1, label="speed (true)")
    ax[0].set_ylabel("speed [rpm]")
    ax[0].set_title("Speed step 1500 → 3000 rpm with G28×9.2 prop")
    ax[0].legend(loc="lower right")
    ax[1].plot(x, a["iq"], color=C1, label="iq")
    ax[1].plot(x, a["id"], color=C2, label="id")
    ax[1].set_ylabel("current [A]")
    ax[1].set_xlabel("time from step [s]")
    ax[1].legend(loc="upper right")
    save(fig, "02_speed_step.png")


def t_load_step():
    b = bench_ready()
    b.foc.enc_offset = math.pi
    b.foc.cmd_speed(2500)
    b.run(3.0)
    t0 = b.motor.t

    def gust(bb):
        bb.motor.cfg.load_torque_nm = 0.5
    b.run(1.5, events=[(t0 + 0.2, gust)])
    a = win(b.arrays(), t0, t0 + 1.5)
    after = a["t"] >= t0 + 0.2
    dip = 2500 - float(np.min(a["rpm"][after]))
    err = np.abs(a["rpm"] - 2500)
    bad = np.flatnonzero(after & (err > 10))
    rec = float(a["t"][bad[-1]] - t0 - 0.2) if len(bad) else 0.0
    R["load_step"] = dict(load_step_nm=0.5, speed_dip_rpm=dip, recovery_to_10rpm_s=rec,
                          iq_before=float(np.mean(a["iq"][~after][-200:])), iq_after=float(np.mean(a["iq"][-200:])))
    plt = _plt()
    fig, ax = plt.subplots(2, 1, figsize=(7.5, 4.4), sharex=True)
    x = a["t"] - t0 - 0.2
    ax[0].plot(x, a["rpm"], color=C1, label="speed")
    ax[0].axhline(2500, color=GREY, lw=1.0, ls="--", label="reference")
    ax[0].set_ylabel("speed [rpm]")
    ax[0].set_title("Load disturbance: +0.5 N·m step at 2500 rpm")
    ax[0].legend(loc="lower right")
    ax[1].plot(x, a["iq"], color=C1, label="iq")
    ax[1].set_ylabel("iq [A]")
    ax[1].set_xlabel("time from load step [s]")
    save(fig, "03_load_step.png")


def t_datasheet():
    rows = []
    b = bench_ready()
    b.foc.enc_offset = math.pi
    for thr, thrust, torque, cur, rpm, pw in DATASHEET[:-1]:
        b.foc.cmd_speed(rpm)
        b.run(2.0)
        a = b.arrays()
        n = int(0.25 / b.foc.ts)
        tl = float(np.mean(a["Tload"][-n:]))
        ib = float(np.mean(a["ibatt"][-n:]))
        vb = b.motor.cfg.battery_voltage_v
        irms = float(np.sqrt(np.mean(np.array(a["ia"][-n:]) ** 2)))
        rows.append(dict(throttle=thr, rpm=rpm, ds_torque=torque, sim_torque=tl, ds_current=cur, sim_current=ib,
                         ds_power=pw, sim_power=ib * vb, sim_iphase_rms=irms,
                         sim_eff=tl * rpm * RADS2RPM ** -1 / max(1e-9, ib * vb)))
    # 100 % point: maximum speed the ESC reaches at full modulation (no field weakening)
    b.foc.c.fw_enable = False
    b.foc.c.accel_limit_rpm_s = 1500
    b.foc.cmd_speed(6000)
    b.run(4.0)
    a = b.arrays()
    n = int(0.25 / b.foc.ts)
    tl = float(np.mean(a["Tload"][-n:]))
    ib = float(np.mean(a["ibatt"][-n:]))
    rpm_max = float(np.mean(a["rpm"][-n:]))
    thr, thrust, torque, cur, rpm, pw = DATASHEET[-1]
    rows.append(dict(throttle=100, rpm=rpm, sim_rpm_full=rpm_max, ds_torque=torque, sim_torque=tl, ds_current=cur,
                     sim_current=ib, ds_power=pw, sim_power=ib * 48.0,
                     sim_iphase_rms=float(np.sqrt(np.mean(np.array(a["ia"][-n:]) ** 2))),
                     sim_eff=tl * rpm_max / RADS2RPM / max(1e-9, ib * 48.0)))
    R["datasheet"] = rows
    plt = _plt()
    fig, ax = plt.subplots(1, 2, figsize=(9.5, 3.4))
    r = [x["rpm"] for x in rows]
    ax[0].plot(r, [x["ds_torque"] for x in rows], "o-", color=C1, ms=5, label="datasheet")
    ax[0].plot([x.get("sim_rpm_full", x["rpm"]) for x in rows], [x["sim_torque"] for x in rows], "s--", color=C2,
               ms=5, label="digital twin")
    ax[0].set_title("Prop torque vs speed")
    ax[0].set_xlabel("speed [rpm]")
    ax[0].set_ylabel("shaft torque [N·m]")
    ax[0].legend()
    ax[1].plot(r, [x["ds_current"] for x in rows], "o-", color=C1, ms=5, label="datasheet")
    ax[1].plot([x.get("sim_rpm_full", x["rpm"]) for x in rows], [x["sim_current"] for x in rows], "s--", color=C2,
               ms=5, label="digital twin (FOC)")
    ax[1].set_title("Battery current vs speed (48 V)")
    ax[1].set_xlabel("speed [rpm]")
    ax[1].set_ylabel("current [A]")
    ax[1].legend()
    save(fig, "04_datasheet_comparison.png")


def _six_step_run(duty, seconds=3.0, dt=5e-6):
    m = BLDCMotor(MotorConfig.from_json(CFG))
    log = {"t": [], "rpm": [], "Te": [], "ia": [], "ibatt": []}
    while m.t < seconds:
        hi, lo = SECTOR_FWD[m.hall_sector()]
        du = [0.0, 0.0, 0.0]
        en = [False, False, False]
        du[hi] = duty
        en[hi] = en[lo] = True
        m.set_duty(*du, enable=en)
        m.step(dt)
        if m.t > seconds - 0.1:
            log["t"].append(m.t)
            log["rpm"].append(m.omega * RADS2RPM)
            log["Te"].append(m.Te)
            log["ia"].append(m.i[0])
            log["ibatt"].append(m.ibatt)
    return {k: np.array(v) for k, v in log.items()}


def _thd(t, x, f0):
    """THD of x (uniformly sampled) relative to fundamental f0, harmonics 2..25."""
    dt = t[1] - t[0]
    n = int(round((int((t[-1] - t[0]) * f0) / f0) / dt))
    x = x[:n] - np.mean(x[:n])
    X = np.abs(np.fft.rfft(x * np.hanning(n)))
    f = np.fft.rfftfreq(n, dt)
    def amp(fk):
        i = np.argmin(np.abs(f - fk))
        return np.sqrt(np.sum(X[max(0, i - 2):i + 3] ** 2))
    a1 = amp(f0)
    return float(np.sqrt(sum(amp(k * f0) ** 2 for k in range(2, 26))) / a1 * 100)


def t_ripple():
    six = _six_step_run(0.62)
    rpm6 = float(np.mean(six["rpm"]))
    b = bench_ready()
    b.foc.enc_offset = math.pi
    b.foc.cmd_speed(rpm6)
    b.run(3.0)
    a = b.arrays()
    n = int(0.1 / b.foc.ts)
    foc = {k: np.asarray(a[k][-n:]) for k in ("t", "rpm", "Te", "ia", "ibatt")}
    fe = rpm6 / 60 * 21
    res = {}
    for name, d in (("six_step", six), ("foc", foc)):
        te = d["Te"]
        res[name] = dict(rpm=float(np.mean(d["rpm"])), torque_mean=float(np.mean(te)),
                         torque_ripple_pkpk_pct=float(np.ptp(te) / np.mean(te) * 100),
                         torque_ripple_rms_pct=float(np.std(te) / np.mean(te) * 100),
                         current_thd_pct=_thd(d["t"], d["ia"], fe), ibatt=float(np.mean(d["ibatt"])),
                         iphase_rms=float(np.sqrt(np.mean(d["ia"] ** 2))))
    R["ripple"] = res
    plt = _plt()
    fig, ax = plt.subplots(2, 1, figsize=(7.5, 4.8), sharex=True)
    per = 1 / fe
    for d, col, lab in ((six, C2, "6-step (hall)"), (foc, C1, "FOC")):
        x = (d["t"] - d["t"][0]) * 1e3
        m = x < 3 * per * 1e3
        ax[0].plot(x[m], d["Te"][m], color=col, label=lab)
        ax[1].plot(x[m], d["ia"][m], color=col, label=lab)
    ax[0].set_ylabel("torque [N·m]")
    ax[0].set_title(f"Same speed (~{rpm6:.0f} rpm) and prop load: 6-step vs FOC, 3 electrical periods")
    ax[0].legend(loc="lower right")
    ax[1].set_ylabel("phase A current [A]")
    ax[1].set_xlabel("time [ms]")
    ax[1].legend(loc="lower right")
    save(fig, "05_torque_ripple.png")


def t_sensorless():
    b = bench_ready(sensorless=True)
    b.foc.cmd_if_start(2500)
    b.run(3.5)
    a = b.arrays()
    ho = next((t for t, md in zip(a["t"], a["mode"]) if md == "speed"), None)
    start = dict(handover_time_s=float(ho) if ho else None, time_to_2500_s=None)
    reach = np.flatnonzero(np.abs(a["rpm"] - 2500) < 25)
    if len(reach):
        start["time_to_2500_s"] = float(a["t"][reach[0]])
    # angle error vs speed
    errs = []
    for rpm in (600, 1000, 2000, 3000, 3500):
        b.foc.cmd_speed(rpm)
        b.run(1.8)
        aa = b.arrays()
        n = int(0.2 / b.foc.ts)
        e = np.degrees(np.asarray(aa["theta_err"][-n:]))
        errs.append(dict(rpm=rpm, angle_err_mean_deg=float(np.mean(e)), angle_err_pkpk_deg=float(np.ptp(e)),
                         speed_err_rpm=float(np.mean(np.asarray(aa["rpm_est"][-n:]) - np.asarray(aa["rpm"][-n:])))))
    # sensorless load step at 2000 rpm
    b.foc.cmd_speed(2000)
    b.run(1.5)
    t0 = b.motor.t

    def gust(bb):
        bb.motor.cfg.load_torque_nm = 0.5
    b.run(1.0, events=[(t0 + 0.1, gust)])
    w = win(b.arrays(), t0, t0 + 1.0)
    R["sensorless"] = dict(startup=start, angle_vs_speed=errs,
                           load_step=dict(speed_dip_rpm=float(2000 - np.min(w["rpm"])),
                                          max_angle_err_deg=float(np.max(np.abs(np.degrees(w["theta_err"]))))),
                           observer_gain=b.foc.gamma, pll_bw_hz=b.foc.c.pll_bw_hz)
    plt = _plt()
    s = win(a, 0, 3.5)
    fig, ax = plt.subplots(2, 1, figsize=(7.5, 4.8), sharex=True)
    ax[0].plot(s["t"], s["rpm"], color=C1, label="true speed")
    ax[0].plot(s["t"], s["rpm_est"], color=C2, lw=1.2, ls="--", label="ESC estimate (I/f, then observer)")
    if ho:
        for axx in ax:
            axx.axvline(ho, color=GREY, lw=1.0, ls=":")
        ax[0].annotate("observer takes over", (ho, 450), xytext=(ho + 0.15, 1200), fontsize=8.5, color="#52514e",
                       arrowprops=dict(arrowstyle="-", color=GREY, lw=0.8))
    ax[0].set_ylabel("speed [rpm]")
    ax[0].set_title("Sensorless start from standstill to 2500 rpm (with prop)")
    ax[0].legend(loc="lower right")
    ax[1].plot(s["t"], np.degrees(s["theta_err"]), color=C1, lw=1.2)
    ax[1].set_ylabel("angle error [° elec]")
    ax[1].set_xlabel("time [s]")
    ax[1].set_ylim(-60, 60)
    save(fig, "06_sensorless_start.png")


def t_regen():
    b = bench_ready()
    b.foc.enc_offset = math.pi
    b.foc.c.accel_limit_rpm_s = 30000
    b.foc.cmd_speed(3500)
    b.run(3.5)
    t0 = b.motor.t
    b.run(1.2, events=[(t0 + 0.1, lambda bb: bb.foc.cmd_speed(1000))])
    a = win(b.arrays(), t0, t0 + 1.2)
    p = a["vbus"] * a["ibatt"]
    dt = np.diff(a["t"], prepend=a["t"][0])
    recovered = float(-np.sum(np.minimum(p, 0) * dt))
    R["regen"] = dict(from_rpm=3500, to_rpm=1000, decel_time_s=float(
        a["t"][np.argmax(a["rpm"] < 1050)] - t0 - 0.1), min_iq_A=float(np.min(a["iq"])),
        min_ibatt_A=float(np.min(a["ibatt"])), max_vbus_V=float(np.max(a["vbus"])), energy_recovered_J=recovered,
        regen_limit_A=b.foc.c.i_regen_max)
    plt = _plt()
    fig, ax = plt.subplots(3, 1, figsize=(7.5, 5.6), sharex=True)
    x = a["t"] - t0 - 0.1
    ax[0].plot(x, a["rpm"], color=C1)
    ax[0].set_ylabel("speed [rpm]")
    ax[0].set_title("Regenerative braking 3500 → 1000 rpm (regen limit 20 A)")
    ax[1].plot(x, a["ibatt"], color=C2)
    ax[1].axhline(0, color=GREY, lw=0.8)
    ax[1].set_ylabel("battery current [A]")
    ax[2].plot(x, a["vbus"], color=C3)
    ax[2].set_ylabel("bus voltage [V]")
    ax[2].set_xlabel("time from command [s]")
    save(fig, "07_regen_braking.png")


def t_locked_rotor():
    b = bench_ready(rotor_inertia_kgm2=1e6)          # rotor mechanically locked
    b.foc.enc_offset = math.pi
    b.foc.cmd_speed(3000)
    b.run(1.0)
    a = b.arrays()
    imax = float(np.max(np.sqrt(np.asarray(a["id"]) ** 2 + np.asarray(a["iq"]) ** 2)))
    ipk = float(np.max(np.abs(np.concatenate([a["ia"], a["ib"], a["ic"]]))))
    m = b.motor
    p_cu = 1.5 * b.foc.c.i_max ** 2 * m.cfg.phase_resistance_ohm
    R["locked_rotor"] = dict(i_limit_A=b.foc.c.i_max, max_current_vector_A=imax, max_phase_current_A=ipk,
                             copper_loss_W=p_cu, winding_heating_K_per_s=p_cu / m.cfg.thermal_capacitance_j_per_k,
                             time_to_150C_s=float(-m.cfg.thermal_capacitance_j_per_k * m.cfg.thermal_resistance_k_per_w *
                                                  math.log(1 - (150 - m.cfg.ambient_c) /
                                                           (p_cu * m.cfg.thermal_resistance_k_per_w)))
                             if p_cu * m.cfg.thermal_resistance_k_per_w > 150 - m.cfg.ambient_c else None)


def t_field_weakening():
    out = {}
    curves = {}
    for fw in (False, True):
        b = bench_ready(load_fan_k=0.0, load_inertia_kgm2=0.0)
        b.foc.enc_offset = math.pi
        b.foc.c.fw_enable = fw
        b.foc.c.accel_limit_rpm_s = 3000
        b.foc.c.J = b.motor.cfg.rotor_inertia_kgm2          # bare motor: re-tune the speed loop
        b.foc.c.speed_bw_hz = 15.0
        b.foc.retune()
        b.foc.cmd_speed(5500)
        b.run(3.0)
        a = b.arrays()
        n = int(0.2 / b.foc.ts)
        out["with_fw" if fw else "without_fw"] = dict(max_rpm=float(np.mean(a["rpm"][-n:])),
                                                      id_A=float(np.mean(a["id"][-n:])),
                                                      ibatt_A=float(np.mean(a["ibatt"][-n:])))
        curves[fw] = a
    R["field_weakening"] = out
    plt = _plt()
    fig, ax = plt.subplots(2, 1, figsize=(7.5, 4.6), sharex=True)
    for fw, col, lab in ((False, C2, "field weakening off"), (True, C1, "field weakening on")):
        a = curves[fw]
        ax[0].plot(a["t"], a["rpm"], color=col, label=lab)
        ax[1].plot(a["t"], a["id"], color=col, label=lab)
    ax[0].axhline(5500, color=GREY, lw=1.0, ls="--", label="reference 5500 rpm")
    ax[0].set_ylabel("speed [rpm]")
    ax[0].set_title("No prop, 48 V: speed above base speed")
    ax[0].legend(loc="lower right")
    ax[1].set_ylabel("id [A]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="lower left")
    save(fig, "08_field_weakening.png")


TESTS = [t_identification, t_current_step, t_speed_step, t_load_step, t_datasheet, t_ripple, t_sensorless, t_regen,
         t_locked_rotor, t_field_weakening]


def main():
    only = sys.argv[1:]
    w0 = time.time()
    for fn in TESTS:
        if only and fn.__name__ not in only and fn is not t_identification:
            continue
        t = time.time()
        fn()
        print(f"{fn.__name__:22s} {time.time() - t:6.1f}s")
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "results.json"), "w") as f:
        json.dump(R, f, indent=1)
    print(f"total {time.time() - w0:.0f}s -> {OUT}")


if __name__ == "__main__":
    main()
