"""Physics model of a BLDC motor + 3-phase inverter + DC bus (no pygame needed).

Modelled:
  * Star-connected windings, per-phase R and L (L = self - mutual), floating neutral
  * Trapezoidal / sinusoidal / blended back-EMF, aligned to the rotor angle
  * Torque from power balance  Te = sum(e_k * i_k) / omega  (exact, includes ripple)
  * Cogging torque, viscous + Coulomb friction, load (const / Coulomb / viscous / fan), inertias
  * Inverter: MOSFET Rds_on, body-diode freewheeling, Hi-Z phases (diode clamps to rails),
    open-phase terminal voltage = Vn + e  (usable for sensorless BEMF zero-crossing)
  * DC bus: battery + internal resistance + bus capacitor -> voltage sag, regen pumps the bus up
  * Winding thermal model (copper R rises, magnet flux drops with temperature)
  * Optional inductance saturation
  * Hall sensors (offset configurable)

Sign convention: phase current is positive INTO the motor from the inverter node.
Back-EMF f(theta_e) is aligned with sin(): phase A flat-top +1 for 30..150 deg electrical.
"""
from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, asdict, fields

PI = math.pi
TWO_PI = 2.0 * PI
D120 = TWO_PI / 3.0
RADS2RPM = 60.0 / TWO_PI

# hall sector (0..5, hall edges at the commutation points) -> (high phase, low phase), forward torque
SECTOR_FWD = [(0, 1), (0, 2), (1, 2), (1, 0), (2, 0), (2, 1)]
# same thing keyed by hall state (Ha,Hb,Hc) for ESC code that reads hall bits
_HALL_OF_SECTOR = ((1, 0, 1), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1))
HALL_TO_SECTOR = {(1, 0, 1): 0, (1, 0, 0): 1, (1, 1, 0): 2, (0, 1, 0): 3, (0, 1, 1): 4, (0, 0, 1): 5}


@dataclass
class MotorConfig:
    name: str = "2212 1000KV outrunner (12V)"
    # ---- machine ----
    pole_pairs: int = 7
    slots: int = 12                      # only used for cogging period
    kv_rpm_per_v: float = 1000.0         # no-load rpm per volt of DC bus (6-step, ideal)
    phase_resistance_ohm: float = 0.080  # per phase (winding), at 25 C
    phase_inductance_h: float = 35e-6    # per phase, L - M
    saturation_current_a: float = 0.0    # 0 = no saturation; else L = L0/(1+(I/Isat)^2)
    rated_current_a: float = 15.0        # only used to scale UI sliders
    bemf_shape: str = "trapezoid"        # "trapezoid" | "sine" | "blend"
    bemf_blend: float = 0.0              # 0 = trapezoid, 1 = sine (when bemf_shape == "blend")
    cogging_torque_nm: float = 0.0015    # peak
    rotor_inertia_kgm2: float = 6e-6
    viscous_friction_nms: float = 1e-6
    coulomb_friction_nm: float = 4e-4
    hall_offset_deg: float = 30.0        # 30 = hall edges exactly at ideal 6-step commutation points
    # ---- thermal ----
    thermal_resistance_k_per_w: float = 6.0
    thermal_capacitance_j_per_k: float = 12.0
    ambient_c: float = 25.0
    copper_alpha: float = 0.00393
    magnet_tempco_per_k: float = -0.0010
    # ---- supply ----
    battery_voltage_v: float = 12.0
    battery_resistance_ohm: float = 0.02
    bus_capacitance_f: float = 470e-6
    ideal_supply: bool = False           # True = rigid Vbus, no sag / regen pump
    allow_regen_to_battery: bool = True  # False = bench PSU that cannot sink current (bus pumps up)
    # ---- inverter ----
    mosfet_rds_on_ohm: float = 0.004
    diode_vf_v: float = 0.7
    # ---- load (also changeable live) ----
    load_inertia_kgm2: float = 0.0
    load_torque_nm: float = 0.0          # signed constant (e.g. hanging weight)
    load_coulomb_nm: float = 0.0         # opposes motion
    load_viscous_nms: float = 0.0
    load_fan_k: float = 0.0              # T = k * w * |w|   (propeller / fan)
    # ---- sensors ----
    current_sensor_noise_a: float = 0.0  # gaussian noise added in feedback()

    @staticmethod
    def from_json(path):
        with open(path) as f:
            raw = json.load(f)
        names = {f.name for f in fields(MotorConfig)}
        unknown = set(raw) - names
        if unknown:
            raise ValueError(f"unknown config keys: {sorted(unknown)}")
        return MotorConfig(**raw).validate()

    def validate(self):
        """Raise ValueError with a clear message for physically impossible / numerically unsafe values."""
        must_pos = ["kv_rpm_per_v", "phase_resistance_ohm", "phase_inductance_h", "rotor_inertia_kgm2",
                    "thermal_resistance_k_per_w", "thermal_capacitance_j_per_k", "battery_voltage_v"]
        if not self.ideal_supply:
            must_pos += ["battery_resistance_ohm", "bus_capacitance_f"]
        bad = [k for k in must_pos if not (math.isfinite(getattr(self, k)) and getattr(self, k) > 0)]
        must_nonneg = ["saturation_current_a", "cogging_torque_nm", "viscous_friction_nms", "coulomb_friction_nm",
                       "mosfet_rds_on_ohm", "diode_vf_v", "load_inertia_kgm2", "load_coulomb_nm",
                       "load_viscous_nms", "load_fan_k", "current_sensor_noise_a", "slots"]
        bad += [k for k in must_nonneg if not (math.isfinite(getattr(self, k)) and getattr(self, k) >= 0)]
        if int(self.pole_pairs) < 1:
            bad.append("pole_pairs")
        if self.bemf_shape.lower() not in ("trapezoid", "sine", "blend"):
            bad.append("bemf_shape")
        if bad:
            raise ValueError(f"invalid motor config value(s): {', '.join(bad)}")
        return self

    def to_json(self, path):
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)


def _lcm(a, b):
    return a * b // math.gcd(a, b)


def _trap(x):
    """Trapezoid, +-1 flat 120 deg, 60 deg ramps, zero crossings at 0/180 deg."""
    x %= TWO_PI
    k = 6.0 / PI
    if x < PI / 6:
        return x * k
    if x < 5 * PI / 6:
        return 1.0
    if x < 7 * PI / 6:
        return 1.0 - (x - 5 * PI / 6) * k
    if x < 11 * PI / 6:
        return -1.0
    return -1.0 + (x - 11 * PI / 6) * k


def ll_window_factor(blend):
    """Average line-line BEMF over a 6-step conduction window (60 deg centred on its peak), per unit of
    phase-peak BEMF. 2.0 for trapezoid, 3*sqrt(3)/pi = 1.654 for sine. This is what makes the DC-bus
    no-load speed equal Kv * Vbus for any BEMF shape (the usual way Kv is measured)."""
    n = 3600
    ell = [_shape(TWO_PI * k / n, blend) - _shape(TWO_PI * k / n - D120, blend) for k in range(n)]
    w = n // 6                                   # 60 degree window, best position (ideal commutation)
    run = sum(ell[:w])
    best = run
    for k in range(n):
        run += ell[(k + w) % n] - ell[k]
        best = max(best, run)
    return best / w


def _shape(x, blend):
    if blend <= 0.0:
        return _trap(x)
    if blend >= 1.0:
        return math.sin(x)
    return (1.0 - blend) * _trap(x) + blend * math.sin(x)


class BLDCMotor:
    def __init__(self, cfg: MotorConfig | None = None):
        self.cfg = cfg if cfg is not None else MotorConfig()
        self.rebuild()
        self.reset()

    # ------------------------------------------------------------------ setup
    def rebuild(self):
        """Call after changing cfg fields that feed derived constants (kv, poles, shape, hall)."""
        c = self.cfg.validate()
        self.pp = int(c.pole_pairs)
        self.n_cog = _lcm(int(c.slots), 2 * self.pp) if c.slots > 0 else 0
        s = c.bemf_shape.lower()
        self.blend = 0.0 if s == "trapezoid" else 1.0 if s == "sine" else min(1.0, max(0.0, c.bemf_blend))
        # phase-peak BEMF constant [V.s/rad mech] such that 6-step no-load speed = Kv * Vbus
        self.ke0 = 1.0 / (ll_window_factor(self.blend) * c.kv_rpm_per_v * TWO_PI / 60.0)
        self.hall_off = math.radians(c.hall_offset_deg)

    def reset(self, theta=0.0):
        c = self.cfg
        self.t = 0.0
        self.i = [0.0, 0.0, 0.0]
        self.omega = 0.0
        self.theta = theta % TWO_PI          # mechanical, wrapped
        self.theta_unwrapped = theta
        self.Tw = c.ambient_c
        self.vbus = c.battery_voltage_v
        self.drive = [(False, 0.0)] * 3      # per phase: (enabled, high-side fraction 0..1)
        self.gates = (0.0,) * 6              # AH AL BH BL CH CL as applied (duty mode: AH=d, AL=1-d)
        self.shoot = False
        self.shoot_latched = False
        self.vt = [0.0, 0.0, 0.0]            # terminal voltages (to battery negative)
        self.vn = 0.0
        self.ibus = 0.0
        self.ibatt = 0.0
        self.Te = 0.0
        self.Tload = 0.0
        self.e = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------ inverter I/O
    def set_gates(self, ah, al, bh, bl, ch, cl):
        """Real gate signals. Both FETs of a leg off -> that phase is Hi-Z (diodes still conduct)."""
        d = []
        shoot = False
        for h, l in ((ah, al), (bh, bl), (ch, cl)):
            if h and l:
                shoot = True
                d.append((True, 0.5))
            elif h:
                d.append((True, 1.0))
            elif l:
                d.append((True, 0.0))
            else:
                d.append((False, 0.0))
        self.drive = d
        self.gates = (1.0 if ah else 0.0, 1.0 if al else 0.0, 1.0 if bh else 0.0,
                      1.0 if bl else 0.0, 1.0 if ch else 0.0, 1.0 if cl else 0.0)
        self.shoot = shoot
        if shoot:
            self.shoot_latched = True

    def set_duty(self, da, db, dc, enable=(True, True, True)):
        """Averaged complementary half-bridge: node voltage = duty * Vbus. enable=False -> Hi-Z."""
        self.drive = [(bool(enable[k]), min(1.0, max(0.0, dd))) for k, dd in enumerate((da, db, dc))]
        g = []
        for en, fr in self.drive:
            g += (fr, 1.0 - fr) if en else (0.0, 0.0)
        self.gates = tuple(g)
        self.shoot = False

    def coast(self):
        self.drive = [(False, 0.0)] * 3
        self.gates = (0.0,) * 6
        self.shoot = False

    # ------------------------------------------------------------------- step
    def step(self, dt):
        c = self.cfg
        Vb = self.vbus
        om = self.omega
        i = self.i
        dT = self.Tw - 25.0
        Rph = c.phase_resistance_ohm * (1.0 + c.copper_alpha * dT)
        rds = c.mosfet_rds_on_ohm
        R = Rph + rds
        ke = self.ke0 * (1.0 + c.magnet_tempco_per_k * dT)
        L = c.phase_inductance_h
        isat = c.saturation_current_a
        if isat > 0.0:
            im = max(abs(i[0]), abs(i[1]), abs(i[2]))
            L = L / (1.0 + (im / isat) ** 2)
        vd = c.diode_vf_v

        the = self.pp * self.theta
        bl = self.blend
        f0 = _shape(the, bl)
        f1 = _shape(the - D120, bl)
        f2 = _shape(the + D120, bl)
        kw = ke * om
        e = (kw * f0, kw * f1, kw * f2)

        # ---- which phases are connected to something, and at what node voltage
        conn = [False, False, False]
        act = [False, False, False]      # actively driven by a FET
        node = [0.0, 0.0, 0.0]
        frac = [0.0, 0.0, 0.0]           # high-side fraction (for bus current)
        dio = [0, 0, 0]                  # conducting body diode: +1 low-side (i>0), -1 high-side (i<0)
        drive = self.drive
        for k in (0, 1, 2):
            en, fr = drive[k]
            if en:
                conn[k] = act[k] = True
                frac[k] = fr
                node[k] = fr * Vb
            else:
                ik = i[k]
                if ik > 0.0:             # current into motor: low-side body diode
                    conn[k] = True
                    node[k] = -vd
                    dio[k] = 1
                elif ik < 0.0:           # current out of motor: high-side body diode
                    conn[k] = True
                    node[k] = Vb + vd
                    frac[k] = 1.0
                    dio[k] = -1

        m = 0
        s = 0.0
        for k in (0, 1, 2):
            if conn[k]:
                m += 1
                s += node[k] - e[k]
        # open (Hi-Z, zero current) phases: do their terminal voltages forward-bias a diode?
        for _ in range(2):
            changed = False
            if m == 0:
                khi = 0
                klo = 0
                for k in (1, 2):
                    if e[k] > e[khi]:
                        khi = k
                    if e[k] < e[klo]:
                        klo = k
                if e[khi] - e[klo] > Vb + 2.0 * vd:     # uncontrolled rectifier
                    conn[khi] = True
                    node[khi] = Vb + vd
                    frac[khi] = 1.0
                    dio[khi] = -1
                    conn[klo] = True
                    node[klo] = -vd
                    dio[klo] = 1
                    changed = True
            else:
                vn_try = s / m
                for k in (0, 1, 2):
                    if not conn[k]:
                        vt = vn_try + e[k]
                        if vt > Vb + vd:
                            conn[k] = True
                            node[k] = Vb + vd
                            frac[k] = 1.0
                            dio[k] = -1
                            changed = True
                        elif vt < -vd:
                            conn[k] = True
                            node[k] = -vd
                            dio[k] = 1
                            changed = True
            if not changed:
                break
            m = 0
            s = 0.0
            for k in (0, 1, 2):
                if conn[k]:
                    m += 1
                    s += node[k] - e[k]
        vn = s / m if m else 0.5 * Vb

        # ---- phase currents (sum of connected currents is forced to zero: isolated star point)
        ni = [0.0, 0.0, 0.0]
        if m:
            # backward Euler in R (stable for any L/R vs dt), explicit in the source voltages
            a = 1.0 / (1.0 + dt * R / L)
            g = dt / L
            tot = 0.0
            for k in (0, 1, 2):
                if conn[k]:
                    x = (i[k] + g * (node[k] - vn - e[k])) * a
                    ni[k] = x
                    tot += x
            mean = tot / m
            crossed = False
            for k in (0, 1, 2):
                if conn[k]:
                    x = ni[k] - mean
                    if not act[k] and dio[k] * x <= 0.0:
                        x = 0.0              # diode would be reverse-biased: it blocks
                        conn[k] = False
                        crossed = True
                    ni[k] = x
            if crossed:
                m2 = 0
                tot = 0.0
                for k in (0, 1, 2):
                    if conn[k]:
                        m2 += 1
                        tot += ni[k]
                if m2:
                    mean = tot / m2
                    for k in (0, 1, 2):
                        if conn[k]:
                            ni[k] -= mean

        # ---- torque and mechanics
        Te = ke * (f0 * ni[0] + f1 * ni[1] + f2 * ni[2])
        Tcog = c.cogging_torque_nm * math.sin(self.n_cog * self.theta) if self.n_cog else 0.0
        sg = om * 2.0
        sg = 1.0 if sg > 1.0 else -1.0 if sg < -1.0 else sg      # smooth sign, 0.5 rad/s band
        Tl = (c.load_torque_nm + c.load_coulomb_nm * sg + c.load_viscous_nms * om
              + c.load_fan_k * om * abs(om))
        Tf = c.coulomb_friction_nm * sg + c.viscous_friction_nms * om
        J = c.rotor_inertia_kgm2 + c.load_inertia_kgm2
        om += dt * (Te - Tcog - Tf - Tl) / J
        d_th = dt * om
        self.theta_unwrapped += d_th
        self.theta = (self.theta + d_th) % TWO_PI
        self.omega = om

        # ---- thermal
        P = Rph * (ni[0] * ni[0] + ni[1] * ni[1] + ni[2] * ni[2])
        self.Tw += dt * (P - (self.Tw - c.ambient_c) / c.thermal_resistance_k_per_w) / c.thermal_capacitance_j_per_k

        # ---- DC bus
        ib = 0.0
        for k in (0, 1, 2):
            if conn[k]:
                ib += frac[k] * ni[k]
        if self.shoot:
            for k in (0, 1, 2):
                en, fr = drive[k]
                if en and fr == 0.5:
                    ib += Vb / (4.0 * rds + 1e-3)
        vbat = c.battery_voltage_v
        if c.ideal_supply:
            Vn = vbat
            ibatt = ib
        else:
            Rb = c.battery_resistance_ohm
            C = c.bus_capacitance_f
            Vn = (Vb + dt / C * (vbat / Rb - ib)) / (1.0 + dt / (Rb * C))
            if Vn > vbat and not c.allow_regen_to_battery:
                Vn = max(Vb - dt * ib / C, vbat)
                ibatt = 0.0
            else:
                ibatt = (vbat - Vn) / Rb
            if Vn < 0.05:
                Vn = 0.05
        self.vbus = Vn
        self.ibus = ib
        self.ibatt = ibatt

        # ---- terminal voltages (what an ESC ADC would see)
        vt = self.vt
        for k in (0, 1, 2):
            if act[k]:
                vt[k] = node[k] - rds * ni[k]
            elif conn[k]:
                vt[k] = node[k]
            else:
                vt[k] = vn + e[k]
        self.vn = vn
        self.i = ni
        self.Te = Te
        self.Tload = Tl
        self.e = e
        self.t += dt

    # ---------------------------------------------------------------- outputs
    def hall_bits(self):
        return _HALL_OF_SECTOR[self.hall_sector()]

    def hall_sector(self):
        th = (self.pp * self.theta - self.hall_off) % TWO_PI
        return min(5, int(th * 3.0 / PI))

    def fault_bits(self):
        c = self.cfg
        return ((1 if self.shoot_latched else 0) | (2 if self.Tw > 150.0 else 0)
                | (4 if self.vbus > 1.3 * c.battery_voltage_v else 0))

    def feedback(self, noisy=True):
        """Everything the ESC could measure (plus a few ground-truth internals)."""
        i = self.i
        n = self.cfg.current_sensor_noise_a if noisy else 0.0
        g = random.gauss
        return {
            "t": self.t,
            "theta": self.theta,
            "theta_elec": (self.pp * self.theta) % TWO_PI,
            "omega": self.omega,
            "rpm": self.omega * RADS2RPM,
            "ia": i[0] + (g(0, n) if n else 0.0),
            "ib": i[1] + (g(0, n) if n else 0.0),
            "ic": i[2] + (g(0, n) if n else 0.0),
            "i_bus": self.ibus + (g(0, n) if n else 0.0),
            "i_batt": self.ibatt,
            "v_bus": self.vbus,
            "va": self.vt[0], "vb": self.vt[1], "vc": self.vt[2], "vn": self.vn,
            "torque": self.Te,
            "temp": self.Tw,
            "hall": self.hall_bits(),
            "fault": self.fault_bits(),
            "bemf": self.e,                   # ground truth, not measurable on a real motor
        }
