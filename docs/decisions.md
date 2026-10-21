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

## D6. The simulator's motion model is fitted to the capture

- **Decision:** moves follow a jerk-limited ("double S") profile with the speed and acceleration sent, capped at the
  controller's configured acceleration (200 deg/s^2, 200 mm/s^2), at its configured jerk (24 rad/s^3, 2000 mm/s^3),
  starting 20 ms (joint moves) or 10 ms (lines) after the command arrives. Queued moves run one after another, each
  from rest. `tools/capture/fit_sim.py` does the fit and prints the evidence.
- **Alternatives:** a trapezoid (fits worse: RMS 0.215 deg against 0.109 over the 43 joint moves, best with a 70 ms
  delay); a trapezoid or double S through a first-order servo lag (no better at any lag from 10 to 60 ms); blending
  queued lines into one move (RMS 12 mm against 0.26 mm: the capture's 50 mm clicks each started and ended at rest,
  peaking near 90 mm/s, which is what a 50 mm double S at 100 mm/s, 200 mm/s^2 and 2000 mm/s^3 gives).
- **Why:** with these, the simulator predicts the capture's joint angles with a median error of 0.034 deg and its
  straight lines with 0.08 mm; and a 20 ms start, a jerk-limited creep to 0.02 deg and 10 Hz frames reproduce the
  measured "about 110 ms from command to visible motion" (the simulator gives a median of 94 to 109 ms).

## D7. STOP in the simulator replays the one measured stop

- **Decision:** on STOP every joint carries on for 6.39 ms at its current speed (J1 rested 0.192 deg past where it
  was when STOP was sent, at 30 deg/s), swings back and settles along J1's measured transient, scaled by each joint's
  overrun, within 0.5 s. State reads 4 at once.
- **Alternatives:** an instant halt (the old simulator; misses the 0.33 deg swing back that the next frame showed); a
  fitted second-order servo model (one 10 Hz measurement can't pin down its two parameters).
- **Why:** it reproduces the measurement exactly (the suite checks this, though the model was built from it). Other
  joints, speeds and payloads are scaled guesses: unverified until the morning tests.
- **Correction, logged here:** a first pass took the overrun as 7.05 ms from a slip in arithmetic (30 deg/s x 0.7 ms
  taken as 0.002 deg); the suite's replay caught it.

## D8. The enable shift is one measured vector

- **Decision:** every enable shifts the joints by (-0.0016, 0.0002, 0.0554, -0.0080, -0.0007, 0.0015) deg, as the
  capture's second enable did (0.46 mm at the tool; the first enable shifted it 0.53 mm, measured more coarsely).
- **Why and limits:** both measurements were at the same pose. J3 carries the most gravity torque, so the shift is
  probably pose- and payload-dependent: unverified.

## D9. The status frame's queue count leaves out one command

- **Decision:** the simulator reports the queue minus the one command "in the planner".
- **Why:** every frame of the capture read 0 queued, including those with a second line click queued behind a moving
  one. Whether it would read 1 with three commands (as the simulator does) is unverified, so nothing in the
  featureset may rely on the count: the runner sends one move at a time and watches state and position instead.

## D10. The simulator starts with the real arm's configuration

- **Decision:** 200 deg/s^2 and 200 mm/s^2 maximum accelerations, the measured jerks, no tool, both sensitivities 0,
  as captured. It used to start with 500 deg/s^2 and 3000 mm/s^2. The one panel test that needs others now passes
  them (`--joint-maxacc 300 --tcp-maxacc 3000`).

## D11. What the simulator models without evidence, and how

Each is marked unverified in `tools/utra_sim.py` and gets a morning test:
- **Pause and resume** ramp to a halt along the path in 0.25 s and back (the SDK documents pause, 3, and ready, 0).
- **Freedrive** (motion mode 2) lets the control port's "hand" move the joints if teach sensitivity is 1 to 5; at 0,
  as captured, the arm holds. The SDK documents 1 to 5 only, so the server sets a level before entering freedrive.
- **Collision detection:** a control-port "contact" past a threshold set by collision sensitivity (1 to 5) stops the
  arm with error 31 ("tau collision limit"). Thresholds are made up; error 22 ("Mechanical collision limit") is the
  other candidate.
- **Speed fault:** error 24 when a joint passes 84% of its servo's speed limit, which is where the real J3 faulted.
- **Joint limits** (error 23) from Umbratek's xacro; **unreachable lines** (error 21) before moving.
- **Tool offset:** the reported pose is the flange's pose composed with the offset (x, y, z in mm, then roll, pitch,
  yaw as Rz Ry Rx) in the flange frame. The featureset keeps the controller's offset at zero anyway (see D14).
- **GPIO:** 16 digital inputs and outputs each on the tool and the controller, no analog, read and written with the
  SDK's own calls.

## D12. A control port for tests, on the simulator only

- **Decision:** HOST:30099 takes JSON lines: read the whole state, move the arm by hand in freedrive, set inputs, press
  the E-stop, bump the arm, inject a fault.
- **Alternatives:** command-line fault flags only (can't act mid-test, as a hand or a machine's signal does).
- **Why:** teaching, I/O waits and recovery can't be tested without something outside the controller acting. Nothing
  in `scripts/` may use it.

## D13. Build order: kinematics, frames, moves, programs, teaching

- **Decision:** as `docs/capabilities.md` section 7: server-side kinematics (K1); tools, frames, calibration and
  keep-out zones (F1 to F4); straight and relative moves with supervision (M2 to M5); the program model, runner, event
  log and I/O steps (P1 to P5, S2, A1); freedrive and recording targets (T1, T2).
- **Alternatives:** teaching first (it would have nothing to record into); I/O after teaching (every job needs I/O
  steps, and they cost little once the runner exists).
- **Why:** each needs the ones before it; the brief's expected order holds, with I/O folded into the runner.

## D14. The tool point is applied on the PC; the controller's tool offset stays zero

- **Decision:** tools (tool point, payload, shape) live in `cell/cell.json`. Targets are tool-point poses; the server
  converts them to flange poses (target x inverse(tool)) and sends flange poses. The controller's offset stays zero,
  so its reported pose stays the flange's, which our kinematics matches in every captured frame. The payload is sent
  to the controller (`set_tcp_load`), since its dynamics and collision detection need it.
- **Alternatives:** set the controller's offset to the active tool (`set_tcp_offset`): its lines would then be
  straight for the tool point even while turning, but the offset's convention is undocumented and unmeasured, and the
  reported pose's meaning would change with it.
- **Why:** everything stays on measured ground. A translation-only line of the flange is a straight line of the tool
  point; lines that also turn are split by the planner into segments short enough that the tool point stays within
  0.1 mm of its line (checked). Revisit after the morning test of the offset's convention.

## D15. The cell is stored as JSON in `cell/`, one file for the cell and one per program

- **Decision:** `cell/cell.json` (tools, frames, targets, zones) and `cell/programs/<name>.json`, written atomically
  (a temporary file, then a rename), each with a revision number; run logs in `cell/logs/`. `--cell DIR` picks
  another folder. Git ignores `cell/`.
- **Alternatives:** SQLite (opaque to the owner, who may want to read or diff a program); one file for everything
  (every program edit would rewrite the cell).
- **Why:** readable, diffable, and the owner can put it under version control if he likes. The revision number stops
  two browser tabs from overwriting each other.

## D16. Poses are stored with roll, pitch and yaw in degrees

- **Decision:** `{"xyz": [mm], "rpy": [deg]}`, R = Rz(yaw) Ry(pitch) Rx(roll), as the controller reports and the panel
  shows. Quaternions only inside interpolation.
- **Alternatives:** quaternions (RAPID's choice; unreadable to the owner), rotation vectors.
- **Why:** the owner reads these numbers in the panel and the controller's own displays; the conversions are checked
  against SciPy.

## D17 to D19: from the study of RAPID, UR, PLCopen and behaviour trees

Recorded with their reasons in `docs/capabilities.md` section 4: fine points only, no fly-by zones or blends (D17);
a UR-like tree of steps with PLCopen's step outcomes and Stop/Halt distinction (D18); behaviour trees' step results
and Retry/Fallback adopted for later, their reactive ticking rejected (D19).
