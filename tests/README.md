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
the twin suite reports SKIP and the run fails. By hand: `brain\.venv\Scripts\python tests\twin_tests.py`, optionally
naming sections (`cadence viewer_follows`). It opens no window unless `TWIN_TEST_WINDOW=1` is set (on Windows): then
it also starts `serve --viewer` and closes the viewer's window as its close button does.

`tests/simrig.py` can serve any suite's simulator from the twin instead (`UTRA_TEST_TWIN=1`): `python -m brain.twin
serve` hosts `tools/utra_sim.py` in its own process, on the same ports, so the suite's commands reach the same
controller model with the twin following it.

## Suites

| Suite | Script | What it checks |
| ----- | ------ | -------------- |
| cli | `cli_tests.py` | `scripts/arm.py` against the simulator with injected faults (sticky errors, E-stop, ignored moves, NaN joints, ...), and its argument checks. |
| inproc | `inproc_tests.py` | `arm.py` interrupted in-process: Ctrl+C mid-move and while preparing, a stop with the SDK's lock held, a stop with no controller. |
| panel | `panel_tests.py` | `scripts/jog_server.py` against the simulator: jogs, planned moves, collision refusals, speed caps, Wi-Fi stalls, watchdogs, faults, E-stop, the three.js download; reduced speed (each jog and planned move's speed sampled along it against 250 mm/s at the flange and a tool's point, one also measured in the simulator) and hold-to-move (refusals, the hold reported, a key let go and a hold lapsing mid-move, timed against the simulator, freedrive untouched); a launcher's token (`UMBRATEK_PANEL_TOKEN`) and `simulated`. |
| js | `js_tests.py` | Under Node, `ik_tests.mjs` (the panel's inverse kinematics) and `xcheck.mjs` (the panel's collision check reaches the server's verdicts, and its gaps to 1e-9 m, on random moves, many of them starting with part of the arm inside a zone, where the gap is signed, and on moves from just outside a guarded zone to less than 1 mm into it, which the same box unguarded lets through: all refused, the 1 mm allowance stopping at a closed zone's face). Their inputs are written from `utra_model.py` at run time. |
| collide | `collide_tests.py` | Oracle (NumPy): the collision geometry's closest points against dense sampling, and its signed gap (minus the depth inside a box or cylinder) against NumPy's own signed distance. |
| paths | `path_tests.py` | `check_path` against dense sampling along random moves, and along short moves starting inside a zone, which may move out but not further in. Slow in pure Python, so it samples 6 moves (2 with `--quick`). The tending suite checks that a move checked in parts (a line's samples, a path's segments) has one allowance for parts already close. |
| capture | `capture_tests.py` | The real arm's capture (`data/captures/2026-10-21`): our forward kinematics against the controller's pose in every one of its 2292 status frames and the quoted joint/pose pairs, the streams' timing and config, and `tools/capture/capture.py analyze` reproducing the committed summary. |
| sim | `sim_tests.py` | The simulator's motion model replayed against the capture's 43 joint moves, 24 straight lines, STOP and enable shift; then the running simulator: 10 Hz state, port 30002 refused, the config stream on 30003, the start delay, STOP, the queue count, straight lines through a pause, faults, the tool offset, payload and sensitivity registers, GPIO, freedrive and collision detection. |
| spine | `spine_tests.py` | Planned and learned steps and the brain interface (`docs/brain-interface.md`): the protocol's and the safety shell's checks in-process (the tool point's speed cap judged by sampling); then the jog server against the simulator with the stub brain as a separate process (a taught, planned, taught program end to end, sampled clear of a keep-out zone) and brains the test plays on its own port: a path through a zone, the cart or past a limit, stale, malformed, oversized, slow and late replies, each refused with nothing sent; learned skills inside and beyond their region, out of time, with a failing check, holding still (10 Hz at most); STOP while the brain thinks and while its path runs; pause and resume; programs that need a brain that isn't there; and three runs of one program started within one second, each with a log of its own (`runner.new_log`, also raced from eight threads). Then reduced speed and hold-to-move in runs: the caps on joint moves and lines against dense sampling (random moves, the brain's path, lines that turn); a program marked full speed run at reduced speed, every move it sent sampled and the run measured in the simulator; runs refused without a hold (dry runs needing none), paused when the hold lapses and carried on when it's held again, and a move to a target stopped when the key is let go. Its brain listens on the suite's panel port + 1000. |
| twin | `twin_tests.py` | The twin (`brain/twin`, in the brain's environment): its model against the spine's kinematics, collision shapes, cart, floor and limits, and a cell's tool, frames, zones and targets; obstacles and physics; the cameras' intrinsics, depth and true poses; its coupling to the simulator, on a virtual clock and live through the SDK and HTTP; every HTTP server here (the jog panel's, the stub brain's and the twin's) binding without a reverse DNS lookup, each made in its own environment with `socket`'s lookups recorded, and no module but `scripts/loopback_http.py` using `http.server`'s own server classes; camera frames from its camera process (ready as the twin starts, and after each rebuild), pixel for pixel its own, and its loop's 10 ms cadence and the controller's 10 Hz status held while frames are pulled (against idle seconds interleaved with them), and frames pulled as the arm sweeps each taken between their request and their reply; the viewer following it from its own process, read-only, through a move, obstacles added, moved and removed, and the twin restarting, and the twin carrying on when a viewer ends; and both captures replayed, every figure against its stated tolerance (`brain/README.md`). |
| tending | `tending_tests.py` | The tending basics. In-process: patterns' slots against a grid worked out in the test, their checks (corners in one frame, pitches, rows and columns not alike, layers), counters kept apart from the cell, grippers' and interlocks' checks, touch-up and Undo with the targets' log, the new steps' checks and sentences, and an old cell read unchanged; and a part within the margin of a closed guarded zone coming up to 1 mm closer, but not into it (a joint move, a line and a brain's path, let through against the same box unguarded). Against the simulator, whose control port plays a gripper (its inputs following its output after its time, a part between its jaws or not, jammed): Grasp, Close and Open from the page and in programs, confirmed by inputs or a set time, a missing part and a jammed gripper failing, and the payload on the controller switched to the tool and the part and back (and, with `--payload-stops`, the arm made ready again when the write stops it, but never after a STOP since the grip began, even one just before it or while that Enable runs, nor over a controller error, which stays: with `--payload-error`, the run faults); the page's gripper keys refused while a Move is under way, and no run or jog while one works; a payload due when freedrive ends with STOP sent at the next Enable; a 3 x 4 tray visited slot by slot by its counter (each slot's centre within 1 mm, as the simulator's joints put the tool point), the counter used up, kept through a restart, reset, set and counted on a copy in a dry run; a machine's zone guarded by its door's interlock, refused before anything is sent while the door is shut (a line, a run, the page's Move, the brain's plan), entered while it's open, and a run stopped and paused within a second of the door shutting mid-move, then resumed; with the tool already in the machine and its door shut, Resume refused and the run kept paused, a later step further in failing, the page's Move and tool jog further in refused, and a Move back out and a tool jog sliding out of the machine's side refused too, then the arm guided out by hand in freedrive, a learned skill and safe home's joint move stopped as the door shuts, and safe home refused with it shut; with the tool 0.3 mm above the machine and its door shut, a jog, a Move, a program's line and the brain's plan into it refused (a dry run assuming the door), and a jog up to its face and back allowed; a tool jog refused, not sent unchecked, with J6 past 180°; in-process, the Enable after a payload write giving way to a STOP pressed as the gripper closes, as the payload is written, or 50 ms before the grip, and (against the simulator) as that Enable checks the joints or switches the motors on, and not resetting an error that appears as it begins; the page's watchdog stopping a Move into the zone at once when hold-to-move is let go, and within 0.5 s when its interlocks can't be read, while a request holds the SDK; Retry catching failures and not faults, Fallback, calling the owner, safe home backing 40 mm out along the tool (or as far as a shelf allows) then home, Run from into a fallback's If it fails; touch-up and Undo through the API; and an old cell and program loading and running unchanged, the API keeping its fields. The stub brain listens on the suite's panel port + 1000. |

## The simulator

```
python tools/utra_sim.py                 # serves 127.0.0.1:502, the status stream on :30001 and config on :30003
python scripts/jog_server.py 127.0.0.1   # then open http://127.0.0.1:8765
```

`python tools/utra_sim.py --help` lists its options and the faults it can inject; `--host` picks another loopback
address. How it moves is fitted to the capture (`python tools/capture/fit_sim.py` shows the fit; `docs/decisions.md`
D6 to D12 say what's measured and what's assumed). Its control port, 30099, lets tests act as a hand, a machine's
signals or the E-stop, and plays a gripper on an output (`{"cmd": "gripper", ...}`: its closed, opened and on-a-part
inputs follow the output after the gripper's time, with a part between its jaws or not, `{"cmd": "part", ...}`, or
jammed); see `Arm.control` in `tools/utra_sim.py`. `--payload-stops` makes writing the payload stop the arm, as writing
the teach sensitivity does (whether the real controller does is unverified), and `--payload-error CODE` raises that
controller error as it's written, as a collision tripping during a grasp would. It keeps time with a high-resolution
clock and timer (D50), so its 10 Hz frames leave 100 ms apart on Windows too. The twin (`python -m brain.twin serve`)
serves the same ports from its own process, with the simulator inside it.

## The capture tools

`tools/capture/capture.py` recorded `data/captures/2026-10-21` through the running jog panel, with the owner present;
`settings_probe.py` read the controller's settings and the joint servos' registers (read-only, motors off);
`line_check.mjs` checked the straight tool lines first with the panel's inverse kinematics and collision check. Only
`capture.py analyze` is used offline, by the capture suite.
