"""Example: your ESC code and the motor in ONE Python process, gate-level PWM, with scope probes.

    python scope/scope12.py --no-sim         # terminal 1 (optional) - oscilloscope
    python examples/example_inprocess_pwm.py    # terminal 2

Your ESC logic runs once per tick (dt). It sets the six gate signals, then calls motor.step(dt).
This is the most accurate coupling (deterministic, real 20 kHz PWM with dead time, ripple visible).
Every motor signal is streamed to the scope as source "sim", the ESC internals as source "esc".
Zoom the scope to 10-20 us/div and trigger on sim/AH to see dead time and current ripple.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from motor.bldc_model import BLDCMotor, MotorConfig, HALL_TO_SECTOR, SECTOR_FWD
from scope.scope_probe import MotorProbe, ScopeProbe, udp_sender

DT = 2.5e-6                 # tick
PWM_TICKS = 20              # 20 ticks * 2.5us = 50us -> 20 kHz
DEAD_TICKS = 1              # dead time = 1 tick = 2.5 us
T_END = 1.5

motor = BLDCMotor(MotorConfig.from_json("motor_config.json"))
scope = "--no-scope" not in sys.argv
if scope:
    mprobe = MotorProbe(motor, udp_sender(), source="sim")
    eprobe = ScopeProbe("esc")


class SimpleESC:
    """6-step hall-commutated, complementary PWM with dead time, open-loop duty."""
    def __init__(self):
        self.cnt = 0
        self.duty = 0.0
        self.sector = -1

    def tick(self, hall, i_bus):
        self.sector = HALL_TO_SECTOR.get(hall, -1)
        if self.sector < 0 or self.duty <= 0:
            return (0, 0, 0, 0, 0, 0)
        hi, lo = SECTOR_FWD[self.sector]
        self.cnt = (self.cnt + 1) % PWM_TICKS
        on = int(self.duty * PWM_TICKS)
        hs_on = self.cnt < on - DEAD_TICKS            # high side PWM
        ls_on = self.cnt >= on + DEAD_TICKS           # low side complementary (sync rectification)
        g = [0] * 6                                    # AH AL BH BL CH CL
        g[2 * hi] = 1 if hs_on else 0
        g[2 * hi + 1] = 1 if ls_on else 0
        g[2 * lo + 1] = 1                              # low phase: low-side always on
        return tuple(g)


esc = SimpleESC()
next_print = 0.0
while motor.t < T_END:
    esc.duty = min(0.7, motor.t / 0.3 * 0.7)           # ramp duty 0 -> 0.7 in 0.3 s
    motor.cfg.load_torque_nm = 0.03 if motor.t > 0.8 else 0.0   # load step
    fb_hall = motor.hall_bits()                        # <- feedback available to your ESC code
    t_cmd = motor.t
    motor.set_gates(*esc.tick(fb_hall, motor.ibus))
    if scope:
        eprobe.sample(t_cmd, duty=esc.duty, sector=esc.sector, pwm_cnt=esc.cnt)
    motor.step(DT)
    if scope:
        mprobe.capture()
    if motor.t >= next_print:
        fb = motor.feedback()
        print(f"t={fb['t']:.3f}s duty={esc.duty:.2f} rpm={fb['rpm']:8.1f} "
              f"Ia={fb['ia']:+6.2f} Ib={fb['ib']:+6.2f} Ic={fb['ic']:+6.2f} Ibus={fb['i_bus']:+6.2f} Vbus={fb['v_bus']:5.2f}")
        next_print += 0.1
if scope:
    mprobe.flush()
    eprobe.close()
