import json
import struct

import numpy as np

import viz.viz_bridge as vb
from scope.scope_probe import MOTOR_SIGNALS, pack_block


def test_ws_frame_lengths():
    for n in (5, 200, 70000):
        f = vb.ws_frame("x" * n)
        assert f[0] == 0x81
        if n < 126:
            assert f[1] == n
        elif n < 65536:
            assert f[1] == 126 and struct.unpack("!H", f[2:4])[0] == n
        else:
            assert f[1] == 127 and struct.unpack("!Q", f[2:10])[0] == n


def test_page_is_wrapped_into_full_document():
    html = vb.page_html().decode()
    assert html.startswith("<!doctype html>") and "<title>BLDC Motor Twin</title>" in html and html.endswith("</html>")


def test_twin_parses_probe_stream_and_unwraps_angle():
    tw = vb.Twin(("127.0.0.1", 9), 0, 1)
    n = 200
    t = np.arange(n) * 1e-5
    th = (np.arange(n) * 0.2) % (2 * np.pi)                    # wraps several times
    data = np.zeros((len(MOTOR_SIGNALS), n))
    data[MOTOR_SIGNALS.index("theta_e[rad]")] = th
    data[MOTOR_SIGNALS.index("ia[A]")] = 1.5
    data[MOTOR_SIGNALS.index("rpm[rpm]")] = 1234
    tw.sock.sendto(pack_block("sim", MOTOR_SIGNALS, t, data), tw.sock.getsockname())
    import time
    time.sleep(0.05)
    tw.poll()
    f = json.loads(tw.frame())
    assert f["type"] == "frame" and f["rpm"] == 1234 and f["ia"] == 1.5
    assert abs(f["theta_unwrapped"] - 0.2 * (n - 1)) < 1e-4    # continuous, not wrapped
    assert len(f["wave"]["t"]) > 0
    tw.sock.close()
