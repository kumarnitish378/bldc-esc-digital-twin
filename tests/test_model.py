import math

import pytest

from bldc_model import BLDCMotor, MotorConfig, SECTOR_FWD, HALL_TO_SECTOR, RADS2RPM, D120

DT = 10e-6


def six_step(m, duty=1.0):
    hi, lo = SECTOR_FWD[m.hall_sector()]
    du = [0.0, 0.0, 0.0]
    en = [False, False, False]
    du[hi] = duty
    en[hi] = en[lo] = True
    m.set_duty(*du, enable=en)


def run(m, seconds, duty=1.0):
    while m.t < seconds:
        six_step(m, duty)
        m.step(DT)


def test_no_load_speed_matches_kv():
    m = BLDCMotor(MotorConfig(ideal_supply=True))
    run(m, 1.5)
    rpm = m.omega * RADS2RPM
    assert 0.97 * 12000 < rpm < 12000            # friction + Rds drop keep it just below Kv*V


def test_stall_current_is_v_over_2r():
    c = MotorConfig(ideal_supply=True, cogging_torque_nm=0.0, rotor_inertia_kgm2=1e3)   # locked rotor
    m = BLDCMotor(c)
    m.set_duty(1.0, 0.0, 0.0, (True, True, False))
    for _ in range(3000):
        m.step(DT)
    expect = 12.0 / (2 * (c.phase_resistance_ohm + c.mosfet_rds_on_ohm))
    assert m.i[0] == pytest.approx(expect, rel=0.02)
    assert m.i[1] == pytest.approx(-expect, rel=0.02)
    assert m.i[2] == 0.0


def test_currents_sum_to_zero_and_power_balance():
    m = BLDCMotor(MotorConfig(ideal_supply=True))
    run(m, 0.3, duty=0.6)
    for _ in range(500):
        six_step(m, 0.6)
        m.step(DT)
        assert abs(sum(m.i)) < 1e-9


def test_low_inductance_motor_is_stable():
    m = BLDCMotor(MotorConfig(phase_inductance_h=0.5e-6, phase_resistance_ohm=0.5, ideal_supply=True,
                              rotor_inertia_kgm2=1e3))
    m.set_duty(1.0, 0.0, 0.0, (True, True, False))
    for _ in range(500):
        m.step(DT)
    assert m.i[0] == pytest.approx(12.0 / (2 * 0.504), rel=0.02)


def test_regen_brakes_and_charges_battery():
    m = BLDCMotor(MotorConfig(cogging_torque_nm=0.0, load_inertia_kgm2=1e-4))
    m.omega = 11000 / RADS2RPM
    for _ in range(5000):
        six_step(m, 0.4)
        m.step(DT)
    assert m.omega * RADS2RPM < 10800
    assert m.ibatt < 0                                # current flows back into the battery


def test_regen_pumps_bus_without_battery_sink():
    m = BLDCMotor(MotorConfig(cogging_torque_nm=0.0, load_inertia_kgm2=1e-4, allow_regen_to_battery=False))
    m.omega = 11000 / RADS2RPM
    for _ in range(5000):
        six_step(m, 0.4)
        m.step(DT)
    assert m.vbus > 15.0
    assert m.fault_bits() & 4


def test_floating_phase_shows_bemf():
    m = BLDCMotor(MotorConfig(ideal_supply=True))
    run(m, 0.5, duty=0.5)
    m.coast()
    for _ in range(3000):                             # let freewheel currents die out
        m.step(DT)
    assert max(abs(x) for x in m.i) < 1e-9            # BEMF below bus: no rectification at 50 % speed
    for k in range(3):
        assert m.vt[k] == pytest.approx(m.vn + m.e[k], abs=1e-9)


def test_freewheel_diode_never_conducts_backwards():
    m = BLDCMotor(MotorConfig(ideal_supply=True))
    run(m, 0.4, duty=0.8)
    prev = list(m.i)
    for _ in range(20000):
        six_step(m, 0.8)
        m.step(DT)
        for k in range(3):
            en, _ = m.drive[k]
            if not en and m.i[k] != 0.0:
                # a floating phase can only keep conducting in the direction it already had
                assert prev[k] == 0.0 or math.copysign(1, m.i[k]) == math.copysign(1, prev[k])
        prev = list(m.i)


def test_hall_table_matches_sine_definition():
    m = BLDCMotor()
    for k in range(3600):
        m.theta = k / 3600 * 2 * math.pi + 1e-7
        th = m.pp * m.theta - m.hall_off
        ref = (int(math.sin(th) > 0), int(math.sin(th - D120) > 0), int(math.sin(th + D120) > 0))
        assert m.hall_bits() == ref
        assert HALL_TO_SECTOR[ref] == m.hall_sector()


def test_shoot_through_latches_fault():
    m = BLDCMotor()
    m.set_gates(1, 1, 0, 0, 0, 0)
    m.step(DT)
    assert m.fault_bits() & 1
    m.coast()
    assert m.fault_bits() & 1                         # latched until reset
    m.reset()
    assert not m.fault_bits() & 1


@pytest.mark.parametrize("field,value", [("battery_resistance_ohm", 0.0), ("phase_inductance_h", 0.0),
                                         ("kv_rpm_per_v", -1.0), ("pole_pairs", 0), ("bemf_shape", "square"),
                                         ("rotor_inertia_kgm2", float("nan"))])
def test_invalid_config_rejected(field, value):
    with pytest.raises(ValueError):
        BLDCMotor(MotorConfig(**{field: value}))


def test_ideal_supply_allows_zero_battery_resistance():
    BLDCMotor(MotorConfig(ideal_supply=True, battery_resistance_ohm=0.0))


def test_config_json_roundtrip(tmp_path):
    p = tmp_path / "m.json"
    MotorConfig(name="x", pole_pairs=4).to_json(p)
    assert MotorConfig.from_json(p).pole_pairs == 4
    p.write_text('{"not_a_field": 1}')
    with pytest.raises(ValueError):
        MotorConfig.from_json(p)


def test_sensor_noise_only_in_feedback():
    m = BLDCMotor(MotorConfig(current_sensor_noise_a=0.5, ideal_supply=True))
    m.set_duty(0.5, 0.0, 0.0, (True, True, False))
    for _ in range(100):
        m.step(DT)
    fb = [m.feedback()["ia"] for _ in range(200)]
    assert max(fb) - min(fb) > 0.5                     # noisy measurement
    assert m.feedback(noisy=False)["ia"] == m.i[0]     # ground truth untouched
