# Tests, the simulator and the capture tools

Everything here runs offline, against `tools/utra_sim.py`, a stand-in for the NTRO controller. Nothing in this folder
connects to the real controller except the capture tools in `tools/capture/`, which are for use with the owner present.

```
python tests/run_all.py            # every suite, three at a time; one line each, full output in tests/logs/
python tests/run_all.py --quick    # the same with smaller samples in the slow oracle suites
python tests/run_all.py panel      # one suite (python tests/run_all.py --list names them)
python tests/run_all.py --python C:\Python312\python.exe   # run the code under test on another Python
```

`run_all.py` gives each suite that drives the simulator its own loopback address (127.0.0.11, 127.0.0.12, ...: the
SDK always connects to port 502) and its own jog panel port, so suites run side by side. A suite passes when its
script exits 0; each prints PASS or FAIL per check and a count at the end.

## The oracle environment

The jog server and the simulator use only Python's standard library. The test oracles, which check our results
against independent implementations, use NumPy, SciPy, OpenCV and [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
Pinocchio has no Windows wheels on PyPI, so the oracle environment comes from conda-forge, in `.venv-oracle/` (git
ignores it):

```
conda create -p .venv-oracle -c conda-forge --override-channels python=3.12 pinocchio numpy scipy opencv -y
```

`run_all.py` finds it and activates it for the oracle suites (on Windows NumPy's linear algebra needs the
environment's `Library\bin` on PATH). Without it, those suites report SKIP and the run fails. To run an oracle
script by hand, use `conda run -p .venv-oracle python tests/<script>.py`.

## Suites

| Suite | Script | What it checks |
| ----- | ------ | -------------- |
| cli | `cli_tests.py` | `scripts/arm.py` against the simulator with injected faults (sticky errors, E-stop, ignored moves, NaN joints, ...), and its argument checks. |
| inproc | `inproc_tests.py` | `arm.py` interrupted in-process: Ctrl+C mid-move and while preparing, a stop with the SDK's lock held, a stop with no controller. |
| panel | `panel_tests.py` | `scripts/jog_server.py` against the simulator: jogs, planned moves, collision refusals, speed caps, Wi-Fi stalls, watchdogs, faults, E-stop, the three.js download. |
| js | `js_tests.py` | Under Node, `ik_tests.mjs` (the panel's inverse kinematics) and `xcheck.mjs` (the panel's collision check reaches the server's verdicts). Their inputs are written from `utra_model.py` at run time. |
| collide | `collide_tests.py` | Oracle (NumPy): the collision geometry's closest points against dense sampling. |
| paths | `path_tests.py` | `check_path` against dense sampling along random moves. Slow in pure Python, so it samples 6 moves (2 with `--quick`). |
| capture | `capture_tests.py` | The real arm's capture (`data/captures/2026-10-21`): our forward kinematics against the controller's pose in every one of its 2292 status frames and the quoted joint/pose pairs, the streams' timing and config, and `tools/capture/capture.py analyze` reproducing the committed summary. |
| sim | `sim_tests.py` | The simulator's motion model replayed against the capture's 43 joint moves, 24 straight lines, STOP and enable shift; then the running simulator: 10 Hz state, port 30002 refused, the config stream on 30003, the start delay, STOP, the queue count, straight lines through a pause, faults, the tool offset, payload and sensitivity registers, GPIO, freedrive and collision detection. |
| spine | `spine_tests.py` | Planned and learned steps and the brain interface (`docs/brain-interface.md`): the protocol's and the safety shell's checks in-process (the tool point's speed cap judged by sampling); then the jog server against the simulator with the stub brain as a separate process (a taught, planned, taught program end to end, sampled clear of a keep-out zone) and brains the test plays on its own port: a path through a zone, the cart or past a limit, stale, malformed, oversized, slow and late replies, each refused with nothing sent; learned skills inside and beyond their region, out of time, with a failing check, holding still (10 Hz at most); STOP while the brain thinks and while its path runs; pause and resume; and programs that need a brain that isn't there. Its brain listens on the suite's panel port + 1000. |

## The simulator

```
python tools/utra_sim.py                 # serves 127.0.0.1:502, the status stream on :30001 and config on :30003
python scripts/jog_server.py 127.0.0.1   # then open http://127.0.0.1:8765
```

`python tools/utra_sim.py --help` lists its options and the faults it can inject; `--host` picks another loopback
address. How it moves is fitted to the capture (`python tools/capture/fit_sim.py` shows the fit; `docs/decisions.md`
D6 to D12 say what's measured and what's assumed). Its control port, 30099, lets tests act as a hand, a machine's
signals or the E-stop; see `Arm.control` in `tools/utra_sim.py`.

## The capture tools

`tools/capture/capture.py` recorded `data/captures/2026-10-21` through the running jog panel, with the owner present;
`settings_probe.py` read the controller's settings and the joint servos' registers (read-only, motors off);
`line_check.mjs` checked the straight tool lines first with the panel's inverse kinematics and collision check. Only
`capture.py analyze` is used offline, by the capture suite.
