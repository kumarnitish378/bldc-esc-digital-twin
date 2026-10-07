"""UDP protocol shared by the simulator (bldc_sim.py) and your ESC program.

COMMAND  (ESC -> motor sim), little-endian, 24 bytes:  "<4B5f"
    mode        u8   0 = GATES  : `gates` bitmask = real gate signals (AH,AL,BH,BL,CH,CL)
                     1 = DUTY   : averaged half-bridge model. duty[A,B,C] 0..1,
                                  `gates` bits AH/BH/CH (0x01,0x04,0x10) = phase enabled,
                                  disabled phase = Hi-Z (floating, diode clamps still act)
    gates       u8   bit0 AH, bit1 AL, bit2 BH, bit3 BL, bit4 CH, bit5 CL
    flags       u8   bit0 = reset motor state
    pad         u8
    duty_a/b/c  f32  used in mode 1
    load_nm     f32  external load torque override (NaN = leave as set in the UI)
    advance_us  f32  >0 : LOCKSTEP - sim advances exactly this much sim-time, then replies
                     0  : free-running (real-time) - sim replies with current state

REPLY (motor sim -> ESC), little-endian, 72 bytes: "<d15fBBxx"
    t, theta_mech(rad), omega(rad/s), rpm, ia, ib, ic, i_bus, i_batt, v_bus,
    va, vb, vc (terminal voltages wrt battery negative), vn (star point),
    torque_e (N.m), temp_winding (C), hall bits (bit0 Ha, bit1 Hb, bit2 Hc), fault bits
    fault: bit0 shoot-through, bit1 winding over-temp (>150C), bit2 bus over-voltage
"""
import struct

DEFAULT_PORT = 9000
CMD_FMT = "<4B5f"
REPLY_FMT = "<d15fBBxx"
CMD_SIZE = struct.calcsize(CMD_FMT)
REPLY_SIZE = struct.calcsize(REPLY_FMT)

MODE_GATES, MODE_DUTY = 0, 1
FLAG_RESET = 1
G_AH, G_AL, G_BH, G_BL, G_CH, G_CL = 1, 2, 4, 8, 16, 32

REPLY_FIELDS = ("t", "theta", "omega", "rpm", "ia", "ib", "ic", "i_bus", "i_batt",
                "v_bus", "va", "vb", "vc", "vn", "torque", "temp")


def pack_cmd(mode=MODE_GATES, gates=0, flags=0, duty=(0.0, 0.0, 0.0),
             load_nm=float("nan"), advance_us=0.0):
    return struct.pack(CMD_FMT, mode, gates, flags, 0, duty[0], duty[1], duty[2],
                       load_nm, advance_us)


def unpack_reply(data):
    v = struct.unpack(REPLY_FMT, data)
    d = dict(zip(REPLY_FIELDS, v[:16]))
    hb = v[16]
    d["hall"] = (hb & 1, (hb >> 1) & 1, (hb >> 2) & 1)
    d["fault"] = v[17]
    return d
