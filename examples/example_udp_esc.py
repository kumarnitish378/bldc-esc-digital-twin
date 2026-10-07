"""Example: ESC in a separate program talking to bldc_sim.py over UDP (lock-step), with scope probes.

    python motor/bldc_sim.py            # terminal 1 (motor)
    python scope/scope12.py             # terminal 2 (oscilloscope - optional; probes the motor automatically)
    python examples/example_udp_esc.py     # terminal 3 (this ESC)

Each loop = one ESC control tick of 100 us (10 kHz). The sim advances exactly 100 us of motor time per
packet, so results are deterministic regardless of PC speed. Uses the averaged-duty inverter mode
(use MODE_GATES with a small advance_us, or example_inprocess_pwm.py, for gate-level PWM).

The ESC's own internal variables are published to the scope as source "esc" (duty, sector, ...), time-
stamped with MOTOR time, so on the scope they line up exactly with the motor's currents and voltages.
"""
import os, socket, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from motor.bldc_protocol import (DEFAULT_PORT, MODE_GATES, MODE_DUTY, FLAG_RESET, REPLY_SIZE,
                           pack_cmd, unpack_reply)
from motor.bldc_model import HALL_TO_SECTOR, SECTOR_FWD
from scope.scope_probe import ScopeProbe

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(("127.0.0.1", DEFAULT_PORT))
sock.settimeout(2.0)
probe = ScopeProbe("esc")                   # -> scope12.py on UDP 9100 (harmless if no scope running)

TICK_US = 100.0
RUN_S = float(sys.argv[sys.argv.index("--seconds") + 1]) if "--seconds" in sys.argv else 3.0
sock.send(pack_cmd(MODE_GATES, 0, flags=FLAG_RESET, load_nm=0.0, advance_us=TICK_US))  # reset, no load
try:
    fb = unpack_reply(sock.recv(REPLY_SIZE))
except (socket.timeout, ConnectionError):
    sys.exit(f"no reply from the motor sim on 127.0.0.1:{DEFAULT_PORT} - start it first: python motor/bldc_sim.py")
t0 = fb["t"]
next_print = 0.0
while True:
    t = fb["t"] - t0
    # ---------------- your ESC algorithm: feedback (fb) in -> gates/duties out ----------------
    duty = min(0.8, t / 0.5 * 0.8)                          # soft start
    load = 0.03 if t > 1.5 else 0.0                         # test case: load step at 1.5 s
    sector = HALL_TO_SECTOR.get(fb["hall"], -1)
    d = [0.0, 0.0, 0.0]
    gates = 0
    hi = lo = -1
    if sector >= 0 and duty > 0:
        hi, lo = SECTOR_FWD[sector]
        d[hi] = duty
        gates = (1 << (2 * hi)) | (1 << (2 * lo))           # enable bits (AH/BH/CH positions)
    # -------------------------------------------------------------------------------------------
    probe.sample(fb["t"], duty=duty, sector=sector, phase_hi=hi, phase_lo=lo,
                 **{"rpm_meas[rpm]": fb["rpm"], "load_cmd[Nm]": load})
    try:
        sock.send(pack_cmd(MODE_DUTY, gates, duty=tuple(d), load_nm=load, advance_us=TICK_US))
        fb = unpack_reply(sock.recv(REPLY_SIZE))
    except (socket.timeout, ConnectionError):
        sys.exit("motor sim stopped responding")
    if t >= next_print:
        print(f"t={t:.3f}s rpm={fb['rpm']:8.1f} Ia={fb['ia']:+6.2f} Ib={fb['ib']:+6.2f} "
              f"Ic={fb['ic']:+6.2f} Ibus={fb['i_bus']:+6.2f} Vbus={fb['v_bus']:5.2f} hall={fb['hall']}")
        next_print += 0.25
    if t > RUN_S:
        break
probe.close()
sock.send(pack_cmd(MODE_GATES, 0, load_nm=0.0, advance_us=0))     # coast
