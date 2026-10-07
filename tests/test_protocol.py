import math
import struct

import numpy as np
import pytest

from motor.bldc_protocol import (CMD_SIZE, REPLY_FMT, REPLY_SIZE, MODE_DUTY, pack_cmd, unpack_reply)
from scope.scope_probe import MOTOR_SIGNALS, MotorProbe, pack_block, unpack_block
from motor.bldc_model import BLDCMotor


def test_command_size_and_defaults():
    pkt = pack_cmd(MODE_DUTY, 0x05, duty=(0.5, 0.0, 0.0), advance_us=50)
    assert len(pkt) == CMD_SIZE == 24
    f = struct.unpack("<4B5f", pkt)
    assert f[0] == MODE_DUTY and f[1] == 5 and math.isnan(f[7]) and f[8] == 50.0


def test_reply_roundtrip():
    vals = [1.5] + [float(i) for i in range(15)] + [0b101, 3]
    d = unpack_reply(struct.pack(REPLY_FMT, *vals))
    assert len(struct.pack(REPLY_FMT, *vals)) == REPLY_SIZE == 72
    assert d["t"] == 1.5 and d["rpm"] == 2.0 and d["hall"] == (1, 0, 1) and d["fault"] == 3


def test_block_roundtrip():
    t = np.linspace(0, 1e-3, 100)
    data = np.vstack([np.sin(t), np.cos(t)])
    src, names, t2, d2 = unpack_block(pack_block("esc", ["a[A]", "b"], t, data))
    assert src == "esc" and names == ["a[A]", "b"]
    np.testing.assert_array_equal(t2, t)
    np.testing.assert_allclose(d2, data, rtol=1e-6)


@pytest.mark.parametrize("mutate", [
    lambda p: p[:-1],                                    # truncated
    lambda p: b"XXXX" + p[4:],                           # bad magic
    lambda p: p[:4] + struct.pack("<H", 3) + p[6:],      # nsig doesn't match names / length
    lambda p: b"",
])
def test_malformed_blocks_rejected(mutate):
    pkt = pack_block("esc", ["a", "b"], np.arange(4.0), np.zeros((2, 4)))
    with pytest.raises(ValueError):
        unpack_block(mutate(pkt))


def test_non_monotonic_time_rejected():
    with pytest.raises(ValueError):
        unpack_block(pack_block("esc", ["a"], np.array([0.0, 2.0, 1.0]), np.zeros((1, 3))))


def test_duplicate_or_slash_names_rejected():
    with pytest.raises(ValueError):
        unpack_block(pack_block("esc", ["a", "a"], np.arange(2.0), np.zeros((2, 2))))
    with pytest.raises(ValueError):
        unpack_block(pack_block("a/b", ["x"], np.arange(2.0), np.zeros((1, 2))))


def test_motor_probe_packets_parse():
    m = BLDCMotor()
    sent = []
    p = MotorProbe(m, sent.append, block=50)
    m.set_duty(0.5, 0.0, 0.0, (True, True, False))
    for _ in range(120):
        m.step(1e-5)
        p.capture()
    p.flush()
    assert len(sent) == 3
    src, names, t, d = unpack_block(sent[0])
    assert src == "sim" and names == list(MOTOR_SIGNALS) and d.shape == (len(MOTOR_SIGNALS), 50)
    assert d[names.index("AH"), 0] == 0.5 and d[names.index("BL"), 0] == 1.0
