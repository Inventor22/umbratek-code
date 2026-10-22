# Tests, the simulator and the capture tools

Everything here runs offline, against `tools/utra_sim.py`, a stand-in for the NTRO controller. Nothing in this folder
connects to the real controller except the capture tools in `tools/capture/`, which are for use with the owner present.

```
python tests/run_all.py            # every suite, three at a time; one line each, full output in tests/logs/
python tests/run_all.py --quick    # the same with smaller samples in the slow oracle suites
python tests/run_all.py panel      # one suite (python tests/run_all.py --list names them)
python tests/run_all.py --python C:\Python312\python.exe   # run the code under test on another Python
python tests/run_all.py --base 111 # beside another checkout's run: addresses from 127.0.0.111, panel ports from 8900
```

`run_all.py` gives each suite that drives the simulator its own loopback address (127.0.0.11, 127.0.0.12, ...: the
SDK always connects to port 502) and its own jog panel port, so suites run side by side. A second checkout running
its suites at the same time on this PC would take the same addresses; give one of them `--base 111` (127.0.0.111 up,
panel ports from 8900). A suite passes when its script exits 0; each prints PASS or FAIL per check and a count at the
end.

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

## The brain's environment

The twin's suite runs in the brain's own environment, `brain/.venv` (MuJoCo and NumPy; `brain/README.md` says how to
make it, on Windows or in WSL2). `run_all.py` uses it, or the Python named by `UMBRATEK_BRAIN_PYTHON`; without either,
the twin suite reports SKIP and the run fails. By hand: `brain\.venv\Scripts\python tests\twin_tests.py`.

## The planner's environment

The planner's suite runs in WSL2, in the planner's environment (`~/.venvs/umbratek-planner`: cuRobo and PyTorch;
`brain/README.md` says how to make it), with the twin in the brain's WSL2 environment (`~/.venvs/umbratek-brain`).
From Windows, `run_all.py` runs it through `wsl.exe` in WSL's default distribution (`UMBRATEK_PLANNER_WSL` names
another), passing the suite's address, ports and log folder by `WSLENV`; in Linux it runs it directly.
`UMBRATEK_PLANNER_PYTHON` names another Python inside WSL2. Without the environment the suite reports SKIP and the run
fails. The twin serves the controller's port 502, so WSL2 must let unprivileged processes have it
(`net.ipv4.ip_unprivileged_port_start=0`, `brain/README.md`). By hand, inside WSL2 from the repo's root:
`~/.venvs/umbratek-planner/bin/python tests/planner_tests.py`.

`tests/simrig.py` can serve any suite's simulator from the twin instead (`UTRA_TEST_TWIN=1`): `python -m brain.twin
serve` hosts `tools/utra_sim.py` in its own process, on the same ports, so the suite's commands reach the same
controller model with the twin following it.

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
| twin | `twin_tests.py` | The twin (`brain/twin`, in the brain's environment): its model against the spine's kinematics, collision shapes, cart, floor and limits, and a cell's tool, frames, zones and targets; obstacles and physics; the cameras' intrinsics, depth and true poses; its coupling to the simulator, on a virtual clock and live through the SDK and HTTP; and both captures replayed, every figure against its stated tolerance (`brain/README.md`). |
| planner | `planner_tests.py` | The planner (`brain/planner`, in WSL2): cuRobo's model of the arm against the spine's (kinematics, spheres covering every capsule and the base, the self-collision pairs, limits); the world (the twin's shapes, spheres and capsules in the spine's check, perception's map and its bound); 48 random blocked moves around a crate, a pole, a ball and a tube, each accepted by the spine's own shell and sampled clear of the obstacle, with the planning times; keep-out zones, starts near something, goals inside something; a fake perception map planned around; then the twin, the planner brain and the jog server as processes: taught, planned, taught programs around the twin's obstacles, out and back, moved and changed, with no contact in the twin and every plan accepted; and a brain blind to the obstacles, whose accepted straight path the twin sees hit the crate. Its brain listens on the suite's panel port + 1000. |

## The simulator

```
python tools/utra_sim.py                 # serves 127.0.0.1:502, the status stream on :30001 and config on :30003
python scripts/jog_server.py 127.0.0.1   # then open http://127.0.0.1:8765
```

`python tools/utra_sim.py --help` lists its options and the faults it can inject; `--host` picks another loopback
address. How it moves is fitted to the capture (`python tools/capture/fit_sim.py` shows the fit; `docs/decisions.md`
D6 to D12 say what's measured and what's assumed). Its control port, 30099, lets tests act as a hand, a machine's
signals or the E-stop; see `Arm.control` in `tools/utra_sim.py`. It keeps time with a high-resolution clock and timer
(D50), so its 10 Hz frames leave 100 ms apart on Windows too. The twin (`python -m brain.twin serve`) serves the same
ports from its own process, with the simulator inside it.

## The capture tools

`tools/capture/capture.py` recorded `data/captures/2026-10-21` through the running jog panel, with the owner present;
`settings_probe.py` read the controller's settings and the joint servos' registers (read-only, motors off);
`line_check.mjs` checked the straight tool lines first with the panel's inverse kinematics and collision check. Only
`capture.py analyze` is used offline, by the capture suite.
