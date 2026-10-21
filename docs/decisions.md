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

## D20. Pause is a STOP, and Resume re-plans from wherever the arm is

- **Decision:** Pause stops the arm with STOP (the measured stop, D7). Resume re-enables the motors if the arm is
  stopped and waits for a status frame from after that, so the plan includes the enable shift (about 0.5 mm, D8). If
  the interrupted step was a straight line, it first moves back onto the line ("regain"), then plans the rest of the
  step from there.
- **Alternatives:** the controller's own pause and resume (`set_motion_status(3)`, then 0), which would keep the
  queued move and its path. Its behaviour on this firmware is unmeasured, and a paused controller can't take the
  jogs that the owner may want between Pause and Resume.
- **Why:** STOP is the one stop that was measured. Re-planning from the measured pose makes jogging while paused safe
  and keeps every move checked. The cost is the enable shift and a short regain move on each resume.

## D21. A faulted run must be reset; a stopped one needn't be

- **Decision:** a run that faults (a step that can't be planned, the arm off its path, a controller error, an input
  that never comes) stays FAULTED until Reset, and Run is refused until then. Stopped and finished runs can be run
  again directly.
- **Alternatives:** start over on Run, as with a stopped run.
- **Why:** PLCopen's ErrorStop: a fault needs acknowledging, so a program isn't restarted into the same problem by a
  reflex click. Stop is the operator's own choice and needs no acknowledgement.

## D22. Program runs carry on without the page; moves started from the page don't

- **Decision:** a program run is supervised by the server (its executor watches every move against its plan, the
  status's freshness and the controller's errors) and ignores the page. A move started from the page (Move, or a
  manual move through `/api/moveto`) stops if the page goes quiet for 1.5 s, as jogs always have.
- **Alternatives:** both stop with the page (rules out unattended runs, the point of the featureset); neither does
  (a closed laptop lid could leave a hand-started move running).
- **Why:** the brief asks for programs that run "without the page open and with their own supervision"; a move
  started by hand is watched by the person who started it, through the page.

## D23. Tool jogs are planned and checked as straight lines

- **Decision:** a tool step (X, Y, Z, roll, pitch, yaw) is now planned as the straight line the controller will drive
  and checked against the arm, cart, floor, the tool in use and the keep-out zones. A step out of reach or through a
  wrist singularity is refused before it's sent.
- **Alternatives:** leave tool steps unchecked, as before (the controller worked out their joint angles, so the panel
  couldn't check them).
- **Why:** with zones and tools in the cell, an unchecked step could drive the tool into a fixture. A behaviour
  change, in README.

## D24. Calibration accuracy is judged statistically

- **Decision:** with simulated touch noise, the frame and tool fits are judged on their RMS error, against what the
  noise predicts (for example, the frame origin's RMS within 15% of sigma times the square root of 3, which it was:
  0.172 against 0.173 mm), plus agreement with SciPy's fit of the same noisy points. Without noise they must be
  exact (1e-9).
- **Alternatives:** a worst-case bound per trial (the first version; it failed on rare but legitimate noise draws).
- **Why:** a fit can't beat its data; the right test is that it uses the data as well as an independent fit does.

## D25. The new panel sections are built from the desk's own parts

- **Decision:** Program, Teach and Cell use only the existing vocabulary (DESIGN.md): charcoal keys, sunk wells, detented
  faders, flat lamps, joined mode keys, the state window's tones. A step's state is its lamp, never a coloured side
  stripe. Freedrive is held on like Enable, with an amber bar and lamp. The cell is drawn in the 3D view in neutral
  tones, never the gizmo's axis colours. Faults stay in the dark red tint; only STOP and the bridge's alarm window are
  solid red.
- **Alternatives:** a conventional web admin layout (cards, tables, checkboxes, modals), which DESIGN.md and the
  design review both reject.
- **How it was checked:** two screenshot rounds at desktop, 900 and 420 px against the simulator; a design review in
  a separate context (its eight material fixes all applied and verified in a second pass); the design detector,
  whose remaining notes are values DESIGN.md already uses for the same jobs. The design skill's question round was
  skipped because the brief says not to ask; the brief and DESIGN.md decided instead.

## D26. A target's key stages a plan; straight moves to targets run as program steps

- **Decision:** a target row's Plan key stages a joint move to the target (shadow arm, path check, plan bar), which
  Move sends, as a drag does.
- **Alternatives:** Joint and Line keys that move at once (the first version; it broke the panel's Stage-then-Move
  rule); staging straight lines too (the plan bar and Move only do joint moves, and the 3D view's path is a joint
  path).
- **Why:** nothing on the page moves the arm on a single click except a jog step. A straight move to a target is a
  program step (Step runs one).

## D27. Space types in text fields

- **Decision:** Space still stops the arm anywhere, except while typing in a text field, where it types a space.
  STOP, Esc and the hardware E-stop still stop.
- **Alternatives:** keep Space as STOP everywhere (no spaces in comments or halt messages); a different stop key.
- **Why:** the new sections have text fields; a halt message needs spaces. Esc remains a one-key stop in a field.

## D28. A target used by a program can't be deleted

- **Decision:** deleting a target that any saved program moves to is refused, naming the programs.
- **Alternatives:** delete it and let those programs fail their check at the next run or save; delete their steps
  too.
- **Why:** a program that silently loses a step is worse than a refused delete; the owner removes the step first.

## D29. Program editing in the page covers the top level and repeats

- **Decision:** the page adds, moves and deletes steps at the top level and inside repeats; ifs, their branches and
  calls are written in the program's JSON (`scripts/runner.py` documents the format) and shown in the list.
- **Alternatives:** a full tree editor.
- **Why:** the jobs' programs are mostly flat sequences with I/O waits; a tree editor is a lot of UI for little use
  tonight. The runner and its tests cover every step type.

## D30. The morning's tools listen, or need the panel closed

- **Decision:** `tools/capture/tau_check.py` only listens (status and config streams, alongside the panel);
  `tools/capture/sensitivity.py` sets collision and teach sensitivity over the command connection, with the panel
  closed.
- **Alternatives:** settings in the panel's API.
- **Why:** collision sensitivity is a safety setting whose levels are unknown; it gets a page control only once the
  morning's test says what the levels do.

## D31. Override starts at the server's value, 100%

- **Decision:** the Override fader shows the server's override, which starts at 100% when the server starts. Programs
  carry their own (slow) default speeds, and straight lines are capped at 250 mm/s unless the program says
  `"fullSpeed": true`.
- **Alternatives:** start at a slow override every time, as the jog speed faders do.
- **Why:** a program's speeds are chosen when it's written; the override is for trying a program slower. The
  reduced-speed cap is the standing guard. The test plan runs new programs at 20% first (the fader's
  detents are 10% apart).

## D32. Straight lines arrive within 1.0 mm and 0.3°

Decided on the arm, 2026-10-21, before the program tests.
- **Decision:** the executor counts a straight line as arrived within 1.0 mm and 0.3° of its goal (it was 0.2 mm and
  0.1°).
- **Alternatives:** keep the tight limit and retry the last millimetre; a limit per move.
- **Why:** the capture's 24 lines came to rest a median 0.21 mm (at most 0.47 mm and 0.125°) from their goals, so
  the old limit would have faulted most real lines. `tests/capture_tests.py` checks the tolerance against them.

## D33. Enable measures the joints from the server's own status stream

- **Decision:** Enable's checks (`arm.preflight`, `arm.verified_targets`) take the measured joints from a frame of
  the server's own 10 Hz stream that arrived after the check began, not from a new status connection.
- **Alternatives:** retry the connection; hold a second status connection open.
- **Why:** the controller serves at most five clients on port 30001 (Umbratek's SDK readme) and frees closed ones
  slowly; on the arm, Enable's extra connections filled the slots and it failed with "the controller closed the
  status connection". The simulator's `--report-clients` and the feature test scenario_status_slots reproduce it.

## D34. Freedrive follows Umbratek's order: settings, then ready, then mode 2

- **Decision:** to enter freedrive the server writes the teach sensitivity only if it differs from the controller's,
  makes the arm ready through Enable's checked sequence if it's stopped, then selects motion mode 2, and counts
  freedrive as on only once a later status frame shows mode 2, not stopped, no error. Freedrive is refused from
  anything but a still, ready arm with every motor on.
- **Alternatives:** (a) set_motion_mode(2) after the sensitivity, as overnight; (b) Umbratek's sequence for position
  mode (mode, enable, ready) after a STOP, as tried this morning. Both failed on the arm.
- **Why:** step by step on the arm (`tools/capture/teach_probe.py`, logged in
  `data/captures/2026-10-21-tests/teach_probe.jsonl`): writing the teach sensitivity stops the arm; mode 2 is ignored
  while it's stopped, which is why (a) and (b) failed; from ready, mode 2 is taken at once and the arm reports
  state 1. Umbratek's current SDK readme gives the same order (load, offset, mounting and sensitivity; enable; ready;
  mode), and the controller's own dashboard refuses "Joint Tech" while the arm is stopped.

## D35. Teach sensitivity is 90, 100 or 110 %, 90 by default, and fixed during freedrive

- **Decision:** the panel offers 90, 100 and 110 %, default 90, and the server refuses a change while in freedrive
  (leave, then enter again).
- **Alternatives:** 1 to 5 (the pinned SDK's docstring); 100 (Umbratek's stated default); 0 (the controller's value
  at power-on, which neither SDK documents); changing it live.
- **Why:** Umbratek's current SDK documents 90 to 110 %. On the arm, 0 and 90 moved easily and stopped where the hand
  let go (within 0.07 mm over 18 s, hands off); at 100 and 110 the shoulder and elbow were lighter, but a raised elbow
  sank back a few degrees; base and wrist joints felt the same at all four. For placing points, holding where
  released matters most. The levels 1 to 5 were never tried in the right order, so they stay unknown. The
  sensitivity write stops the arm, so a live change would end freedrive anyway.

## D36. Leaving freedrive switches the motors off; Enable then runs, unless nobody may be there

- **Decision:** leaving freedrive selects mode 0, which on this controller switches the motors off onto their brakes
  (J3 settles about 0.45°, 0.43 measured), then runs Enable's checked sequence. When freedrive times out after 10
  minutes, the motors stay off. STOP in freedrive sends STOP (the arm holds rigid, still in mode 2) and then mode 0,
  so it too ends with the motors off.
- **Alternatives:** STOP and stay in mode 2, holding (what a later "ready" does then is untested, and might free the
  arm unexpectedly); leave the motors off every time (one more hold of Enable for the user).
- **Why:** it's the controller's own exit, measured on the arm. Re-enabling on the user's explicit request, with
  Enable's no-jump checks, keeps teach-then-test quick. A timeout may mean nobody is at the arm, so it doesn't
  re-energise anything. Targets recorded in freedrive are taken before the settle, so they're unaffected.

## D37. Enable checks that the controller is in position mode afterwards

- **Decision:** Enable fails, and stops the arm, if a status frame from after it reports any motion mode but 0.
- **Alternatives:** trust the sequence.
- **Why:** after this morning, mode changes are known to be refused silently in some states. A ready arm in the wrong
  mode, free or rigid, is what the panel must never assume away.

## D38. Test sessions use the controller on the home network, by cable

- **Decision:** for sessions at the arm, the controller is cabled to the router (192.168.1.43 here) and the server
  runs against that address.
- **Alternatives:** the controller's own Wi-Fi (NTRO_43, 192.168.2.1), as overnight.
- **Why:** on Wi-Fi the status stream dropped out for about 3 s every 33 s (the afternoon's recording, 12:09 to
  13:15); two drop-outs caused SDK reply timeouts (code -3), one of which also lost a STOP's reply ("STOP NOT
  CONFIRMED", cleared with the E-stop). On the cable: no gaps at all, 100 frames in every 10 s. The cost: on the home network the controller's command port and Umbratek's dashboard are open to every
  device there, with no password.

## D39. A joint past its limit is recovered by hand, one brake at a time

- **Decision:** a documented procedure, not a panel feature: switch off only that joint's motor
  (`set_motion_enable(n, 0)`, numbering from 1, and check the enable mask shows just that joint off), release only its
  brake for a bounded time, with a guard that re-engages it on the wrong direction, and turn it back by hand inside
  the limit; then freedrive for the rest.
- **Alternatives:** Umbratek's newer limit switch (LIMIT_FUN, register 0x49), which this firmware doesn't answer;
  servo commands over the RS485 pass-through; asking Umbratek.
- **Why:** wrist 1 turned by hand in freedrive to 358° faulted (error 23, "Mechanical angle limit"). From then on
  every "ready" raised it again, freedrive was refused while it was flagged, and a move back was rejected. The
  procedure brought it back on the arm. It needs a person holding the joint and is rare, so it stays out of the
  panel; the README describes it.
