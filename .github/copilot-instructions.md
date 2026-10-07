# Copilot instructions

## Build, test, and lint

There is no separate build step. CI runs on Python 3.10-3.13 and installs `numpy`, `pytest`, and `pyflakes`.

```bash
python -m pip install numpy pytest pyflakes
python -m pyflakes */*.py
python -m pytest -q
```

Run one test file or an individual test with pytest selectors:

```bash
python -m pytest -q tests/test_model.py
python -m pytest -q tests/test_model.py::test_no_load_speed_matches_kv
```

The GUI applications additionally need the packages in `requirements.txt` (`pygame`, `pyqtgraph`, and `PyQt6`, along with NumPy). The test suite is headless; the simulator server tests do not launch the GUI.

## Architecture

Code is grouped into packages (`motor/`, `esc/`, `scope/`, `viz/`) plus `examples/`, `validation/`, `configs/` and `tests/`. Run scripts from the repo root; modules import each other package-qualified (`from motor.bldc_model import ...`), and entry scripts put the repo root on `sys.path`.


- `motor/bldc_model.py` is the GUI-independent motor, inverter, and DC-bus physics. `MotorConfig` defines the model and validates configuration; the JSON files in `configs/` are example motor parameter sets (`config_path()` resolves bare file names there).
- `motor/bldc_sim.py` runs `SimHost` in a separate process from its Pygame UI. The host owns the motor and exposes it to external ESC processes over UDP. `motor/bldc_protocol.py` is the shared command/reply contract; lock-step commands advance exact simulation time and are used for deterministic control and tests.
- `esc/foc_esc.py` contains the FOC controller independently of the simulator. Its UDP runner uses the shared protocol, while `esc/foc_bench.py` couples the same controller to the motor in-process for repeatable tests and campaigns (`validation/foc_tests.py`).
- `scope/scope_probe.py` defines the SCP1 UDP telemetry format and publishers. `scope/scope_core.py` handles GUI-independent acquisition, buffering, triggering, and measurements; `scope/scope12.py` provides the Qt/pyqtgraph UI. Probe timestamps are simulation time so ESC and motor traces align.
- `viz/viz_bridge.py` consumes the motor's SCP1 stream and serves `viz/web/motor_visualizer.html`, forwarding live state to browser clients over WebSocket. The page also includes its own built-in twin. For browser inspection, `.vscode/mcp.json` configures Playwright MCP; start `python viz/viz_bridge.py` and use `http://127.0.0.1:8765`.
- `tests/` covers model physics, protocol and probe packets, scope DSP, UDP behavior, FOC, and the visualizer bridge. `.github/workflows/tests.yml` is the CI source of truth for lint and test commands.

## Repository-specific conventions

- Preserve the physics sign convention: phase current is positive into the motor; bus current is positive when drawn from the bus. Check `motor/bldc_model.py` and README's UDP section before changing electrical or torque calculations.
- Keep the UDP wire format and sizes synchronized across `motor/bldc_protocol.py`, `motor/bldc_sim.py`, the ESC runner, and protocol tests. Lock-step `advance_us` is the deterministic simulation clock; do not substitute wall-clock timing in control or probe timestamps.
- Probe blocks use a fixed set of signal names per source, with units encoded in brackets (for example, `ia[A]`). Coordinate changes to motor telemetry with `MOTOR_SIGNALS`, consumers such as the scope and visualizer, and packet tests.
- Motor parameter changes belong in `MotorConfig` and its validation, with JSON configs using those dataclass field names. Rebuild the motor after changing fields that affect derived constants.
- Keep numerical/control logic importable without UI dependencies: physics is in `motor/bldc_model.py`, FOC control is in `esc/foc_esc.py`, and scope acquisition/DSP is in `scope/scope_core.py`.
