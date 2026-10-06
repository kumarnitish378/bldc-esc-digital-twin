"""Runs the real SimHost UDP server in a thread and talks to it like an ESC would."""
import multiprocessing as mp
import queue
import socket
import struct
import threading
import time

import pytest

import bldc_sim as bs
from bldc_model import MotorConfig
from bldc_protocol import CMD_FMT, MODE_DUTY, REPLY_SIZE, FLAG_RESET, pack_cmd, unpack_reply


@pytest.fixture
def sim():
    port = _free_port()
    ctrl = mp.Array("d", bs.NCTRL, lock=False)
    stat = mp.Array("d", bs.NSTAT, lock=False)
    dt = 10e-6
    n = int(bs.HIST_S / (max(1, round(bs.SAMPLE_DT / dt)) * dt))
    hist = mp.RawArray("f", bs.NCH * n)
    ctrl[bs.C_SPEED] = ctrl[bs.C_RUN] = ctrl[bs.C_DIR] = 1.0
    ctrl[bs.C_VBAT] = 12.0
    host = bs.SimHost(MotorConfig(), dt, port, "127.0.0.1", ctrl, stat, hist, queue.Queue())
    th = threading.Thread(target=host.run, daemon=True)
    th.start()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("127.0.0.1", port))
    s.settimeout(3.0)
    yield host, s
    ctrl[bs.C_RUN] = 0.0
    th.join(2.0)
    host.sock.close()


def _free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_lockstep_advances_exactly(sim):
    host, s = sim
    s.send(pack_cmd(flags=FLAG_RESET, advance_us=100))
    t0 = unpack_reply(s.recv(REPLY_SIZE))["t"]
    for _ in range(10):
        s.send(pack_cmd(MODE_DUTY, 0x05, duty=(0.5, 0, 0), advance_us=100))
        fb = unpack_reply(s.recv(REPLY_SIZE))
    assert fb["t"] - t0 == pytest.approx(1e-3, abs=1e-9)
    assert fb["ia"] > 0 and fb["ib"] < 0


def test_hostile_commands_do_not_kill_sim(sim):
    host, s = sim
    inf, nan = float("inf"), float("nan")
    for bad in (struct.pack(CMD_FMT, 1, 5, 0, 0, nan, 0, 0, nan, 100),    # NaN duty
                struct.pack(CMD_FMT, 1, 5, 0, 0, 0.5, 0, 0, inf, 100),    # inf load -> ignored
                struct.pack(CMD_FMT, 0, 0, 0, 0, 0, 0, 0, nan, inf),      # inf advance
                b"garbage", b"SUBS" + b"\xff" * 100):
        s.send(bad)
    time.sleep(0.2)
    assert host.cfg.load_torque_nm == 0.0 or abs(host.cfg.load_torque_nm) < 1e30
    t_before = host.motor.t
    s.send(pack_cmd(advance_us=1e12))                                    # clamped to 1 s of sim time
    fb = unpack_reply(s.recv(REPLY_SIZE))
    assert fb["t"] - t_before <= 1.0 + 1e-6


def test_scope_subscription_is_capped(sim):
    host, s = sim
    socks = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(bs.MAX_SUBSCRIBERS + 3)]
    for c in socks:
        c.sendto(b"SUBS\x01\x00", ("127.0.0.1", host.sock.getsockname()[1]))
    time.sleep(0.3)
    assert len(host.subs) == bs.MAX_SUBSCRIBERS
    for c in socks:
        c.close()
