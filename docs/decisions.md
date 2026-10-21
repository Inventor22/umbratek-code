# Decisions

Decisions made during the overnight run of 2026-10-21, while the owner was asleep and couldn't be asked. Each has the
alternatives considered and why this one won. Later entries may revise earlier ones; they say so.

## D1. Layout: `tools/` for the simulator and capture tools, `tests/` for the suites, `data/captures/` for raw data

- **Decision:** `tools/utra_sim.py`, `tools/capture/{capture.py, line_check.mjs, settings_probe.py}`, the suites and
  their single runner in `tests/` (`tests/run_all.py`), and the capture copied unchanged (SHA-256 checked) to
  `data/captures/2026-10-21/`.
- **Alternatives:** everything under `scripts/` (mixes things that drive the real arm with test fixtures); a `sim/`
  top-level folder (one more top-level name for one file).
- **Why:** `scripts/` stays what runs the arm. The simulator is a tool the owner may run by hand too
  (`python tools/utra_sim.py` then `python scripts/jog_server.py 127.0.0.1`), so it isn't buried in `tests/`.

## D2. The oracle environment comes from conda-forge, in `.venv-oracle/`

- **Decision:** one conda environment (Python 3.12, Pinocchio 4.1, NumPy, SciPy, OpenCV), git-ignored; the runner
  activates it (prepends its `Library\bin` folders to PATH) for oracle suites.
- **Alternatives:** a pip venv (tried first: `pin` has no Windows wheels for any Python version, and building it from
  source fails); skipping Pinocchio (loses the one independent kinematics implementation); WSL (another OS to set up).
- **Why:** it's the supported way to get Pinocchio on Windows. Without activation NumPy's `linalg` crashes on
  Windows (a delay-loaded LAPACK DLL), found and fixed while bringing the collision oracle in.

## D3. Simulators on separate loopback addresses, so suites run side by side

- **Decision:** `utra_sim.py --host 127.0.0.N`; `run_all.py` gives suite *k* 127.0.0.(11+*k*) and panel port 8800+*k*.
  The simulator binds with `SO_EXCLUSIVEADDRUSE` on Windows.
- **Alternatives:** a port offset (the SDK hard-codes port 502, so the server and `arm.py` can't follow it); running
  suites one after another (about four minutes longer per run).
- **Why:** Windows routes all of 127/8 to loopback and lets each address bind 502. `SO_REUSEADDR` (what the
  simulator used) lets a second socket on Windows bind the same address and port and steal connections, which would
  make parallel runs flaky.

## D4. The JavaScript suites' inputs are written from the Python model at run time

- **Decision:** `tests/js_tests.py` writes `ik_tests.mjs`'s config and `xcheck.mjs`'s cases from `utra_model` and
  `jog_server` each run, instead of copying the earlier session's `ik_config.json` and `xcheck.json`.
- **Alternatives:** commit the JSON files (they go stale silently whenever the model or the limits change).
- **Why:** the browser code must be tested against the model the server serves today.

## D5. Not brought in: the model-building scripts

- **Decision:** `fit_capsules.py`, `build_cart*.py`, `sw_export*.py` and the earlier probes stay in the earlier
  session's folder.
- **Why:** they made `collision.json` and the cart URDF (described in each model's `SOURCE.txt`); the brief asked for
  the simulator, the test suites and the capture tools. They need SolidWorks and aren't tests.
