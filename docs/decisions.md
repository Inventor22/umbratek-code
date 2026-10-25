# Decisions

Decisions made during the overnight run of 2026-10-21, while the owner was asleep and couldn't be asked. Each has the
alternatives considered and why this one won. Later entries may revise earlier ones; they say so.

The machine-tending specification (`docs/capabilities.md`), the map contract (`docs/world-map.md`), the planner
(`brain/planner/`) and perception (`brain/perception/`), with their suites and the twin's obstacle clip, are no longer
in this repo. The entries that cite them, D60 to D79 among them, are kept as they were written.

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
- **Later, 2026-10-23:** at the owner's request, the model-building scripts are in `tools/model/` after all, with the
  two drafts they replaced, so the cart model and the capsules can be made again: that folder was their only copy.
  Run again, they reproduce the committed files exactly (`tools/model/README.md`). The earlier probes stay out.

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
  change, in the README (since moved to `docs/jog-panel.md`, D80).

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
  panel; `docs/troubleshooting.md` describes it (the README did, until D80).

## D40 to D49: the spine and the brain

Decided on 2026-10-21 (evening), building the spine's side of the spine, brain and twin architecture the owner
approved: planned and learned steps, the interface to the brain (`docs/brain-interface.md`) and its safety shell.

## D40. The brain proposes; the spine checks all of it and runs it as its own

- **Decision:** the brain is a separate process that only answers the spine's questions: a path for a planned move, the
  next burst of a learned skill. The safety shell (`scripts/safety.py`) checks each answer whole, and what passes
  becomes an ordinary plan that the existing executor supervises, segment by segment. Nothing from the brain reaches
  the controller any other way.
- **Alternatives:** let the brain drive the controller itself (its own SDK connection: the controller serves only one,
  and there'd be two safety models); let the brain stream setpoints the spine relays (no time to check them, and the
  10 Hz status couldn't supervise them).
- **Why:** one safety model, one connection, one supervisor. The brain can then be as clever, and as wrong, as it
  likes: a buggy brain is refused, not obeyed (tested with one whose world model lost the keep-out zones).

## D41. The brain is a server on this PC; the spine asks it, one question at a time

- **Decision:** the brain serves HTTP on a loopback address (default `http://127.0.0.1:8770`); the spine is its only
  client. The spine accepts only loopback addresses for it.
- **Alternatives:** the spine as the server, the brain connecting to it (the brief's first sketch: "the brain reaches
  the spine over localhost"); a message queue or gRPC (dependencies the spine can't have); shared memory.
- **Why:** WSL2's default (NAT) networking forwards Windows' `localhost` to servers in WSL2, but WSL2 can reach a
  Windows server only at the host's own address, not `localhost`. With the brain as the server, a brain in WSL2
  bound to its own `127.0.0.1` is reached from Windows at `localhost` (checked: hello in 12 ms), with no
  `.wslconfig` changes and nothing listening beyond loopback. It also means the brain can't speak unless asked, and
  the spine opens no new port. HTTP and JSON are in both standard libraries and readable in a log. The link has no
  authentication, hence loopback only; any local process could pose as the brain, which is why every reply is
  treated as untrusted (D43).

## D42. Protocol version 1: four messages, tied to their request and state

- **Decision:** `hello`, `plan`, `act` and `end` (`scripts/brain_protocol.py`). Every message carries the version, its
  type and the spine's id; replies to `plan` and `act` echo the sequence number of the status frame they were worked
  out from. Replies of another version, to another request (by id) or from another state are refused as stale; so is
  anything malformed, any number that isn't finite (Python's json would accept NaN, Infinity and 1e999), and anything
  over 1 MiB. Unknown fields are ignored, so fields can be added within version 1. Each question has a deadline (plan
  10 s, `--brain-timeout`; act 2 s; connecting 1 s).
- **Alternatives:** timestamps instead of sequence numbers (the PC's and WSL2's clocks differ, and Windows'
  monotonic clock ticks in 15.6 ms); a free-form reply (nothing to check against).
- **Why:** small enough to implement in any language from one document, strict enough that a stale or garbled
  reply can't pass as a fresh one. A refused connection to a closed port on Windows takes about 2 s (it retries the
  SYN), so connecting has its own 1 s limit: on loopback a listening brain takes the connection at once.

## D43. The shell checks a proposal whole, refuses it with every reason, and never runs part of it

- **Decision:** before anything is sent: where it starts and ends (within 0.01° of the arm and the goal), joint limits
  at every waypoint, the continuous collision check on every segment (as every move gets), speed caps, a burst's
  length and region, and just before it runs, freshness (the status under 1 s old, the arm within 0.1° of where the
  brain was told it was, the cell's revision unchanged). Every failed check is a reason; any reason refuses the whole
  proposal, faults the run (stopping the arm, as other planning failures do) and is logged with the first collision
  for the 3D view. A refusal isn't retried.
- **Alternatives:** trim a path to its good prefix (runs part of what was refused); ask the brain again after a refusal
  (a brain that proposes something unsafe is a bug; retrying hides it, and could loop); report the first reason only.
- **Why:** "never partially execute a refused proposal" is simplest to guarantee by checking first and sending after.
  All the reasons at once make a buggy brain quick to diagnose.

## D44. A brain's path runs as fine-point joint moves, one per segment

- **Decision:** each segment of a path (two consecutive waypoints) is one joint move, from rest to rest, sent once the
  arm has arrived from the last and judged from where that one ended. Paths are capped at 64 waypoints and bursts at
  10 more after their start.
- **Alternatives:** blended moves or the controller's timed joint setpoints (`moveto_servo_joint`) for smooth motion.
- **Why:** D17 stands: a blended corner's path is undefined, so it can't be checked, and the executor can only supervise
  a move it can predict. A joint move between two waypoints is exactly the straight joint-space segment the
  collision check covers. The cost is a stop at every waypoint, so a brain should send few; cuRobo's dense
  trajectories will need thinning (wave 2). Revisit once blends are measured.

## D45. What the brain proposes is held to 250 mm/s at the tool point

- **Decision:** besides each joint's rated speed and the step's speed (and the brain's, if lower), every segment of a
  brain's path or burst is slowed until the tool point can't pass 250 mm/s (1000 mm/s with `"fullSpeed": true`):
  `planner.Planner.joint`'s new `tool_speed`, a bound from each joint's share of the turn times the tool point's
  distance from its axis, taken at 17 points along the segment.
- **Alternatives:** no cap for joint moves, as taught joint moves have none; refusing fast proposals.
- **Why:** ISO 10218-1's reduced speed is the program runner's standing guard for straight lines (D31); a path the
  brain chose through the cell deserves at least the same, and speed is a cap, not a reason to refuse. The bound
  can't be exceeded by any combination of the joints' turns; sampled independently, the tool point stayed under it
  (135 mm/s at most on the test path).

## D46. A planned move's goal is the spine's, as taught; dry runs ask the brain too

- **Decision:** the spine works out where a planned move ends: a joint target's joints, or a tool target (with its
  offset, if the step has one) solved from the joints it was taught at, so in the taught configuration. The brain
  plans only the path, which must end exactly there. A dry run asks the brain for each planned path, from where the
  run would be, and checks it as a run would; a learned step is assumed to end where it began.
- **Alternatives:** let the brain choose the goal's joints (a different configuration could fail the next taught
  step's line); a dry run that skips planned moves (it couldn't say a path exists, or how long it takes).
- **Why:** the next taught step starts where the planned one ends, as taught. A learned step's end can't be known
  without running it.

## D47. Learned steps: a region, short bursts, at most 10 Hz, a time limit and a check

- **Decision:** a learned step names its skill and the target it works near; the tool point must stay within `within`
  mm of it (default 100, at most 500), sampled every 0.1° of turn. A burst is at most 10 waypoints after its start,
  15° of turn for any joint and 50 mm of tool point travel. The skill is asked for its next burst once the last has
  run and a newer status frame shows the result, never sooner than 0.1 s after the last request (timed with
  `perf_counter`: `monotonic()` ticks in 15.6 ms on Windows). It may also hold still (up to 1 s), finish or give up.
  The step has a time limit (default 60 s); when the skill says it's done, the step's optional check (an input) must
  read true. The brain is told how each episode ended.
- **Alternatives:** trust the skill's "done"; let a policy stream continuously (unsupervisable at 10 Hz, D44); no
  region (collisions are checked anyway, but a learned skill wandering the whole cell is not what "the variable part"
  means).
- **Why:** the brain only proposes; the spine decides, including whether the job got done. The region keeps a
  learned skill local to its taught target, so a recalibrated frame moves it with the target.

## D48. STOP wins

- **Decision:** a STOP abandons a question to the brain at once (its reply, if it comes, is dropped: the next question
  has a new id), the executor checks for a STOP or pause before each segment, and a run that's been stopped ends as
  stopped whatever else went wrong meanwhile (a failure the STOP itself caused used to read as a fault). Pause
  abandons the question too, and resume asks again, from wherever the arm is.
- **Alternatives:** wait out the brain's deadline (up to 10 s of "stopping"); treat a late reply as valid if it still
  fits.
- **Why:** STOP stops the arm through the controller connection whatever the runner is doing; the run's state should
  say so promptly and nothing from before the STOP should move it afterwards. A move reaching the controller in the
  same instant as a STOP is discarded by it, as it's stopped (the simulator models this; on the arm it's
  [unverified]).

## D49. The stub brain is standard-library Python, reusing the spine's model

- **Decision:** `brain/stub/stub_brain.py` speaks the interface with `scripts/brain_protocol.py` and plans with the
  spine's own model and planner: straight there if clear, else up and over (the tool point raised 60 to 300 mm above
  the start and goal), else random via points, within 60% of its deadline. One skill, `seek`, spirals the tool point
  out until an input goes high. `--ignore-zones` plays a buggy brain. The tests (`tests/spine_tests.py`, suite `spine`)
  run it as a separate process for the end-to-end program, and play misbehaving brains in-process with its answers.
- **Alternatives:** a stub in the brain's own environment (NumPy and friends: its tests wouldn't run under the normal
  interpreter, and the twin session owns that environment); a fake inside the spine (wouldn't cross a process
  boundary).
- **Why:** the stub exists to prove the interface and the shell, not to plan well. Using the spine's own collision
  model, it agrees with the shell whenever it's told the truth, so every refusal in the tests is one a buggy brain
  earned.

## D50 to D59: the twin

Decided on 2026-10-21 (evening), building the twin of the spine, brain and twin architecture the owner approved: a
simulated arm and cell that accepts the same commands as the real arm (`brain/README.md`). D50 is the simulator's
timing, which the twin is built on.

## D50. The simulator keeps time with a high-resolution clock and timer

- **Decision:** `tools/utra_sim.py` reads time from `time.perf_counter()` and, on Windows before Python 3.11, sleeps on
  a high-resolution waitable timer (`CreateWaitableTimerExW` with `CREATE_WAITABLE_TIMER_HIGH_RESOLUTION`, through
  ctypes); elsewhere `time.sleep` already is one. Its status and config streams keep an absolute 100 ms schedule,
  starting it afresh rather than sending a burst if the process was starved for more than a period. The sim suite
  times the frames it receives with `perf_counter` too.
- **Alternatives:** `timeBeginPeriod(1)` (1 ms granularity at best, for the whole process); spinning through the last
  milliseconds (a busy core per stream, with three suites at once); requiring Python 3.13, whose `monotonic()` is
  high-resolution on Windows (the repo supports 3.8 up).
- **Why:** on Python 3.9 under Windows, `time.monotonic()` and `time.sleep()` both follow the 15.6 ms system tick,
  so frames came 63 to 140 ms apart and the suite's "frame gaps 85 to 115 ms" check failed under load; and every
  angle the simulator reported was worked out at a time up to 15.6 ms stale (0.5 deg at 30 deg/s), which the twin,
  sampling the simulator, would have inherited. On this PC, ten seconds of frames timed at the receiver: before,
  77 to 125 ms apart (5th to 95th percentile 92.7 to 111.2) idle, and a 5th percentile of 78 ms under load; after,
  99.4 to 100.5 ms idle and 96.3 to 105.1 ms with 20 busy processes on 16 threads. On Linux and macOS nothing
  changes. The command-to-motion latency the suite measures went from a median of 109 to 100 ms, inside D6's 94 to
  109: the coarse tick had been starting moves up to 15.6 ms late.

## D51. The twin is MuJoCo, in the brain's own environment

- **Decision:** MuJoCo 3.16 (CPU physics, OpenGL rendering) with NumPy, pinned in `brain/requirements.txt`, in
  `brain/.venv` (Python 3.12) on Windows and a venv on the Linux disk in WSL2; the spine stays standard-library
  Python. `tests/run_all.py` runs the twin's suite with that environment and fails a run without it, as it does for
  the oracle environment (D2).
- **Alternatives:** NVIDIA's Newton 1.x (GPU physics on Warp, ray-traced tiled cameras, URDF, MJCF and USD import,
  Windows and Linux); Isaac Sim (out of scope: it needs 16 GB of VRAM, the RTX 3080 Ti has 12).
- **Why:** the twin is one arm in one cell, run in real time beside a 10 Hz controller, with a few cameras. MuJoCo
  does that on the CPU, deterministically, renders colour, depth and segmentation through OpenGL with little VRAM
  (5 ms for a 640 x 480 RGB-D frame), installs as one wheel, and runs the same on Windows (GLFW) and in WSL2 (EGL,
  or WSLg's display), all checked by the suite on both. Newton's strength is thousands of environments in parallel
  on the GPU, for training learned skills; its cameras need CUDA, and its API has moved quickly (1.0 in March 2026,
  1.7 by October). Newton reads MJCF and uses MuJoCo Warp as a solver, so the generated model (`python -m brain.twin
  export`) is the way to it when learned skills need parallel simulation. That route isn't tried yet.

## D52. The twin's model is generated from the repo's sources every time it loads

- **Decision:** `brain/twin/model.py` writes the MJCF in memory on every load, from: `scripts/utra_model.py`'s reading
  of Umbratek's URDF (joint transforms and axes) and the URDF's meshes; the collision capsules of `collision.json`
  and the cart URDF's boxes, where `utra_model` puts them; the floor at the top of the cart model's floor box; the
  joint limits `tools/utra_sim.py` enforces; and, given a cell folder, its tool, keep-out zones, frames and targets
  through `scripts/cell.py`. Nothing generated is committed. Numbers are written as Python's shortest round-trip
  text, so the links sit where `utra_model` puts them to 1e-15 m. Geom groups 0 to 2 (the world, the arm, the tool)
  are what cameras see; 3 (collision shapes), 4 (zones) and 5 (frames and targets) are for the viewer. The arm's
  inertia comes from its capsules at water's density: the URDF's masses are placeholders (1e-5 kg), and the arm's
  joints are set from the controller (D53), so its inertia only matters as something that pushes.
- **Alternatives:** commit a generated MJCF (two copies of the cell and the arm to keep in step); MuJoCo's own URDF
  import (loses the capsules, the cart's labelled boxes and the cell, and needs the mesh paths rewritten).
- **Why:** one source of truth. The twin can't drift from the spine's collision model, and recalibrating a cell frame
  moves the twin's zones, targets and obstacles with it.

## D53. The twin's arm follows `tools/utra_sim.py`, and the twin serves the controller's ports

- **Decision:** each step, the twin sets its arm's joints to the simulator's (`Arm.joints_at(t)`). `python -m
  brain.twin serve` hosts the simulator in the twin's process (`utra_sim.serve()` and `run()`, split out of its
  `main()`) on the usual ports, so the jog panel, `arm.py`, the SDK and the brain drive it as they drive the
  controller. Replays and tests run the same `Arm` on a virtual clock (`Arm(a, now=...)`), fed the register writes
  the SDK sends, encoded by the SDK's own `hex_data`. Free obstacles are stepped with the arm moving steadily between
  samples, so it pushes them; the physics never moves the arm.
- **Alternatives:** position actuators tracking the simulator (a lag and overshoot of the twin's own, unmeasured);
  the twin as a separate process polling the simulator's control port (a second clock and network jitter between
  them); a motion model of the twin's own (a second model of the arm to fit and keep in step).
- **Why:** the simulator is the model fitted to the real arm. The twin adds geometry, physics and cameras around it,
  no motion of its own: its joints are the simulator's exactly (0 rad apart at every step on the virtual clock, and,
  live, on the joint path the status stream reports).

## D54. The twin is verified by replaying both captures, against stated tolerances; what that found

- **Decision:** `brain/twin/replay.py` sends both captures' commands to the simulator again, as the jog panel and the
  program runner built them (from the arm's measured state, which the controller's target registers mirror), and
  compares every status frame with the twin's joints and its tool pose. Each move's start is also fitted within
  +/-150 ms, and the offsets are reported and bounded: they're what the captures can't time (commands crossed Wi-Fi
  in 12 to 45 ms; the run logs stamp a move once the controller has replied). A replay restarts from the measured
  joints at each Enable; a STOP is checked with the stopped move's start fitted to the frames before it; frames that
  Wi-Fi delivered in a burst after a stall are left out (their arrival isn't when they were sent). Every figure and
  its tolerance, with the reason for it, is in `verify()`; `brain/README.md` has the table. All 25 pass.
- **Found, and pinned by the suite as known limits:**
  - Turns. A line that mostly turns the tool is paced by `ROT_MM_PER_RAD` (1 rad as 100 mm), marked unverified in
    D11. The arm turned 10 deg in about 1.2 s at 10 mm/s and 0.85 s at 25 mm/s; the simulator takes 1.9 and 0.92 s.
    The tool's position is right (0.26 mm); the joints lag mid-turn by up to 3.4 deg. One model fits both speeds:
    the turn paced by the controller's own configured rotational limits (4 rad/s^2, 8 rad/s^3, from its config
    stream) at a speed of v/50 rad/s for v mm/s, which gives 1.19 and 0.89 s. Not adopted yet: it rests on four
    moves of one size. Measure turns of 5, 30 and 90 deg at 10, 25 and 100 mm/s, then refit.
  - STOP at another speed. D7 scales the overrun with speed (6.39 ms of travel, from the one stop at 30 deg/s). MT-2
    stopped at 10 deg/s and ran on 0.168 deg, against the model's 0.064. Two stops (0.192 deg at 30 deg/s, 0.168 at
    10) look more like a near-constant distance than a constant time; two points aren't a model.
  - The controller itself. In 6 frames of the slow Taught run its reported pose and joints were a cycle apart (0.7
    to 0.9 mm at 10 mm/s), and long lines ran about 0.25% faster than the speed sent (52.02 s against 52.15 s over
    520 mm).
- **Alternatives:** compare at the logged times only (the captures' timing then dominates: 0.24 deg at the 95th
  percentile for the first capture's joint moves, against 0.07 with the timing fitted); refit the simulator now (it
  changes the controller model under every spine suite, on thin evidence).
- **Why:** it separates what the model gets wrong from what the captures can't time, and states each tolerance with
  its reason, so a change to the simulator or the twin that moves a figure shows up with an explanation to check.

## D55. Simulated cameras: ideal RGB-D in OpenCV's conventions, also served over HTTP

- **Decision:** a camera has pinhole intrinsics in pixels (any fx, fy, cx, cy; MuJoCo's own camera intrinsics, whose
  principal point is an offset from the image centre with the opposite sign to OpenCV's), a mount (the cell, a cell
  frame, or an arm link) and a pose in OpenCV's frame. A frame has colour, z-depth in metres (0 for no data,
  including outside near to far), segmentation by geom, K and the camera's true pose, and saves as PNGs (depth as
  uint16 millimetres) with a JSON file. Multisampling is off, so depth and segmentation are exact at every pixel.
  While `serve` runs, `GET /state`, `GET /cameras/<name>.npz` and `POST /obstacles` and `/cameras` answer on the
  twin's loopback address, port 30100; requests with an Origin header, and POSTs that aren't JSON, are refused.
- **Alternatives:** cameras set only by field of view (square pixels, centred: can't match a calibrated camera);
  sensor noise now (it belongs in perception's tests, with the real camera's figures); a socket protocol of the
  twin's own (curl and a browser couldn't look); ROS (not on this PC).
- **Why:** perception code, nvblox included, expects OpenCV's frame, depth along z in metres, and uint16 millimetre
  depth images; wave 2 builds against these and swaps in the real camera's driver later. The suite checks the
  conventions rather than trusting them: known points project within 0.12 px through an off-centre K with fx != fy,
  and depth back-projects onto known surfaces within 0.1 mm.

## D56. Obstacles: fixed ones are mocap bodies, free ones have a free joint; scenes use the cell's conventions

- **Decision:** a fixed obstacle is a MuJoCo mocap body, moved by setting its pose, without a rebuild; a free one has
  a free joint, a mass and a box's inertia, falls, and is pushed by the arm. Adding or removing one rebuilds the
  model and keeps the arm's and the free obstacles' state. Scene files give sizes in millimetres and poses as
  `{"xyz": [mm], "rpy": [deg]}` in the base frame or a cell frame, as the cell does; a mesh obstacle collides as its
  convex hull.
- **Alternatives:** obstacles as zones in the cell (zones are rules for the spine's check, not things a camera can
  see or the arm can push); rebuilding the model for every move of an obstacle.
- **Why:** perception and planning need obstacles the cameras see and the arm can hit, placed the way the owner
  places everything else in the cell.

## D60 to D69: the planner

Decided on 2026-10-21 (night), building wave 2's planner: a brain that plans planned moves with cuRobo on the GPU, in
WSL2, around what the cell holds (`brain/README.md`, "The planner").

## D60. cuRoboV2 at a pinned commit, on PyTorch 2.12.1 with CUDA 13, in its own WSL2 venv

- **Decision:** `nvidia-curobo` from cuRobo's main branch at `78fd485` (2026-09-10, after the v0.8.0 release), with
  its `cu13` extra; PyTorch 2.12.1+cu130; Ubuntu 24.04's Python 3.12; in `~/.venvs/umbratek-planner`, beside the
  twin's venv; all pinned in `brain/planner/requirements.txt`, which installs into a fresh venv from scratch (checked).
- **Alternatives:** cuRobo v0.7.8 (the v1 API most examples use, but under NVIDIA's non-commercial licence, and its
  extensions need `nvcc` and a CUDA toolkit to build); the v0.8.0 release (without main's later fixes, among them a
  graph-planner bug that let colliding configurations into its roadmap, and attached geometry shared wrongly between
  its solvers); CUDA 12.8's wheels; PyTorch 2.14 (newer than cuRobo's latest commit); one venv with the twin's.
- **Why:** cuRoboV2 is Apache 2.0, and compiles its kernels when they're first used (cuda.core's NVRTC and Warp), so
  WSL2 needs no CUDA toolkit: the Windows driver (616.64, CUDA 13.4) is enough. The 3080 Ti (compute 8.6) is
  supported by CUDA 13. PyTorch 2.12.1 is a patch release from before cuRobo's last commits. Its own venv, because
  two sessions installing into one at once can break it. The first start compiles kernels (about 70 s), cached in
  `~/.cache/warp`; after that cuRobo loads in about 2.5 s and warms up in about 12 s, and holds about 0.7 GB of the
  GPU.

## D61. cuRobo's arm is generated from the repo's sources, its spheres covering the spine's capsules

- **Decision:** `brain/planner/robot.py` writes Umbratek's URDF again with the planning limits (`planner.DRAG_LIMITS_DEG`
  within `utra_sim.JOINT_LIMITS_RAD`), the rated joint speeds, an effort range (Umbratek's is 0, which cuRobo refuses)
  and absolute mesh paths, into `~/.cache/umbratek-planner`; nothing generated is committed. Each capsule of
  `collision.json`, where `utra_model` places it, becomes spheres along its core at most 0.4 radii apart, each of
  radius sqrt(r^2 + (s/2)^2) for their spacing s, so their union covers the capsule (growing it by 1.0 mm at most).
  The base's cylinder becomes a ring of 18 spheres of 30 mm, lowest point 1 mm above the mounting face, which with the
  shoulder's spheres covers its side above 6 mm. cuRobo compares exactly the pairs of links the spine's check compares
  (`utra_model.ArmModel.pairs`, its skip list included). A fixed link on the flange has 64 sphere slots, filled for
  each request from the active tool's capsules.
- **Alternatives:** cuRobo's sphere fitting to the meshes (a second collision model, to keep in step with the
  capsules the spine checks); one sphere per capsule (far bigger than the capsule, or not covering it); cuRobo's own
  self-collision defaults; the tool baked into the model (it changes with the cell).
- **Why:** one source of truth, as the twin's (D52). Spheres that cover the capsules make cuRobo at least as cautious
  as the spine, so what it finds the spine's check passes, first time. Checked by the suite: its flange within
  0.0002 mm of the spine's at 300 random poses (float32), 6800 points on the capsules all inside their spheres, the
  base's side covered, the same pairs.

## D62. cuRobo keeps 12 mm: the spine's margin and 2 mm more

- **Decision:** spheres are grown by the spine's margin (10 mm, `utra_model.MARGIN_M`) and 2 mm more for the world;
  each link's self-collision padding takes back all but half of that, so two links keep 12 mm between them, not
  24. cuRobo's world leaves out the cart parts its fixed spheres would always touch, grown so: the mounting plate
  (the shoulder's capsule stands 10.4 mm above it by design) and a cap screw under it. The spine's own check, which
  every move passes before it's sent (D63), checks them exactly.
- **Alternatives:** the spine's 10 mm exactly (cuRobo checks samples of its spheres, so its paths would often fail the
  spine's continuous check by a fraction of a millimetre); a larger margin (fewer tight paths).
- **Why:** the spine's check is the gate, so cuRobo's margin only needs to make its paths pass it at the first try:
  in the suite the least gap to an obstacle on a planned path is 12.1 mm.

## D63. Straight if it can be; else cuRobo's graph search, its path cut into few moves, each through the spine's check

- **Decision:** a planned move is straight if the spine's own continuous check passes with the world's obstacles
  added. Otherwise cuRobo's graph search (straight joint moves between the nodes of a roadmap it checks), its
  trajectory optimiser on every other attempt, up to 8 attempts in 80% of the spine's deadline. The route is then cut
  greedily: from each kept waypoint, the farthest point along it that one straight joint move reaches, the candidates
  screened together on the GPU (cuRobo's feasibility of samples no sphere moves more than 2 mm between), the move kept
  confirmed by `utra_model.check_path` with the obstacles added. If cuRobo finds nothing, the stub's search (up and
  over, random via points) against the same world. A path whose moves don't all pass the check is never sent.
- **Alternatives:** cuRobo's `plan_cspace` (trajectory optimisation, three time-optimal refinements: 1.3 s a plan;
  0.33 s without the refinements), whose smoothness is lost anyway once the path is cut into moves the controller runs
  from rest to rest (D44); sending cuRobo's dense trajectory (up to 64 waypoints, each a stop); trusting cuRobo's
  verdict (it doesn't know the spine's slack rule or the exact capsules).
- **Why:** fewer segments run faster, and a graph path is straight moves already: 2.2 segments on average around the
  suite's obstacles. The spine's own check as the gate means the spine accepts what the planner sends: 48 of 48 in the
  suite, and every plan in the twin. Planning took 0.86 s median and 1.23 s at most beside the other suites (0.69 and
  1.08 s alone), against the spine's 10 s deadline: on average 0.09 s for the straight check, 0.29 s in cuRobo, 0.45 s
  cutting (3.2 moves through the check, about 0.1 s each in pure Python, the cost of using the spine's own code).

## D64. A start or goal inside cuRobo's margin: a short move away first

- **Decision:** when cuRobo finds the start (or goal) too near something, the planner moves the tool point 10 to
  150 mm straight away from the nearest obstacle (from the spine's check's closest points), or up, or both, turned the
  same way; keeps the first such pose cuRobo finds clear and the spine's check passes the move to; and plans from (or
  to) there.
- **Alternatives:** shrinking cuRobo's spheres for such plans (paths near the margin everywhere, which then fail the
  check); refusing (the spine lets a move start nearer than the margin if it moves away, `utra_model.SLACK_M`, and a
  taught pose can be that near).
- **Why:** taught poses are often near things (a part on a table, the jaw of a vise). Checked: a start 10.8 mm from a
  wall plans through a short move away.

## D65. What the planner plans around: the request, the twin's truth, or perception's map

- **Decision:** `--world request|twin[=URL]|map[=URL]`. Always the tool and keep-out zones of each request, and the arm
  and cart from the repo. `twin` adds the twin's true obstacles, read from its `GET /state` for every plan; the twin's
  `/state` now reports each obstacle's shape, size and mesh (`obstacleShapes`), as it reported only poses before. A
  mesh is its convex hull, as the twin collides it. `map` adds perception's map (`docs/world-map.md`): its ESDF goes to
  cuRobo less half a voxel, since cuRobo interpolates between voxel centres, where a lower bound can overstate the
  clearance; the spine's check is joined by the map's own, each capsule's core sampled every 4 mm along it and along
  the move. Each sample is judged first by a quick lower bound that holds everywhere (the ESDF at the nearest voxel
  centre less the distance to it; beyond the grid, through the point's projection onto the grid's box, as every
  obstacle is inside it), which can be 35 mm below the truth; where that would refuse, by the distance to the obstacle
  voxels themselves, taken as cubes (those where the ESDF is negative, in a k-d tree: the nearest cube among the 32
  nearest centres, exact near an obstacle's surface and a lower bound everywhere). The spine's slack rule (a move may
  start nearer than the margin and move away) is measured from an upper bound on the start's distance and checked
  with lower bounds along the move, so the slack is never overstated.
  A map more than 2 s old, or whose arm isn't where the plan starts (after waiting for a newer frame), fails the plan.
  Any source that can't answer fails the plan, saying why.
- **Alternatives:** the planner reading the twin's scene file (misses obstacles placed over HTTP, and where free ones
  have been pushed); obstacles from the request (the interface has no field for them; the spine doesn't know them);
  treating perception's unknown voxels as solid (everything outside the cameras' view would block); ignoring a
  source that fails (a plan blind to what's there); the quick bound alone (the first version: with perception's own
  map, every path cuRobo found past a crate was refused, as the arm's start was 36 mm from the crate's voxels and the
  bound said 15 mm).
- **Why:** before perception exists, the twin's truth tests planning against obstacles the spine doesn't know about,
  and the same code path takes perception's map. Checked with a fake perception server that serves the contract (a
  crate seen as 2 cm voxels is planned around, 49.6 mm from the true crate), and with perception's own map (D68). A
  review found the first version of the
  map's check fail open in two ways (the slack measured from a loose lower bound, and points just beyond the grid read
  as clear); the suite now checks the rule against exact distances to the voxels on random moves over a slab (101 of
  120 accepted, none nearer than the rule allows; the first version let through moves from 6.0 to 3.1 mm), and the
  distance to the cubes against brute force.

## D66. The planner's brain serves the stub's skill, and says it doesn't plan until it can

- **Decision:** `python -m brain.planner serve` listens at once, on 8770 by default as the stub does, and answers
  `hello` with `"planner": false` (and "warming up") until cuRobo is loaded and warmed up. Learned steps go to the
  stub's scripted `seek`. `GET /v1/stats` lists recent plans and their timings. hello never waits for a plan.
- **Alternatives:** listening only once ready (the panel would show no brain for 15 s, or two minutes the first time);
  a brain with no skills (programs with learned steps wouldn't start with it).
- **Why:** it replaces the stub where it's run, and the spine won't start a run with planned steps until it plans.

## D67. The planner's suite runs in WSL2, through wsl.exe from the runner on Windows

- **Decision:** suite `planner` of `tests/run_all.py`: from Windows, `wsl.exe --cd <repo> -e sh -c 'exec
  ~/.venvs/umbratek-planner/bin/python tests/planner_tests.py'`, the suite's address, ports and log folder passed by
  `WSLENV` (the folder translated to its `/mnt/c` path); in Linux, directly; reported SKIP without the environment.
  Inside WSL2 it runs the twin in the twin's venv on the suite's own loopback address (so port 502 needs
  `net.ipv4.ip_unprivileged_port_start=0`, persisted on this PC in `/etc/sysctl.d/99-umbratek.conf`), the planner's
  brain on the panel port + 1000, and the jog server in the planner's Python. `tests/simrig.py` gains
  `start_sim(twin=True)` and `UTRA_TEST_TWIN=1`, which serve any suite's controller ports from the twin, and looks
  for the twin's environment on its own platform only (in WSL2, a Windows venv under `/mnt/c` would run on Windows).
- **Alternatives:** a planner suite run only by hand in WSL2; the spine's processes on Windows and the planner in WSL2
  (only `127.0.0.1` crosses into WSL2, and its ports are the owner's; the suites use other loopback addresses).
- **Why:** one command runs every suite, and inside WSL2 every address is free. The boundary is crossed by hand
  instead, on a port of its own (D68).
- **Later, 2026-10-22:** with a VPN on, WSL2's DNS doesn't answer reverse lookups of loopback addresses other than
  `127.0.0.1`, and gives up after 10 s. Python's `http.server` makes one when it binds (`socket.getfqdn`, only to keep
  the name), so the twin's HTTP port opened 11 s after it started, past the twin suite's wait, and perception's map
  11 s after it did. Every HTTP server (the jog panel, the stub, the planner, the twin, perception) is now built on
  `scripts/loopback_http.py`, `http.server`'s `ThreadingHTTPServer` without that lookup. The planner's suite makes
  each in its own environment with `socket`'s lookups recorded instead of made, and fails if one asks; and it fails if
  any other module of `scripts/`, `tools/` or `brain/` uses `http.server`'s own server classes.

## D68. How the planner is validated: random moves, the twin end to end, the camera's chain, a control, the boundary

- **Decision:** in-process, 48 random moves above a table whose straight way a crate, pole, ball or tube blocks, each
  path put through the spine's own shell (`safety.check_path`) and its clearance sampled every 0.5 deg; then the twin,
  the planner's brain and the jog server as separate processes running taught, planned, taught programs out and back
  around the twin's obstacles (moved, and changed for a pole, a ball, a tube), the twin's contacts sampled every
  20 ms; then the same program with a brain blind to the obstacles (`--world request`). Then the chain, the camera
  the only source: perception calibrates the twin's overhead camera by its own procedure (views of the board on the
  flange; `tests/perception_tests.py calibration_fixed`), serves its map of what that camera sees, and the planner
  plans on it (`--world map`) around a crate on the example scene's table, out and back; the crate is moved in the
  twin and the next plan must be made on the map that shows the move; then the blind brain as the control. The twin
  carries the active tool, built from the cell given before it starts, so its contacts include the tool's. By hand,
  the spine and the simulator on Windows and the planner in WSL2 on `127.0.0.1:8790`.
- **Why:** the requirement is that the spine accepts the plans and the twin reports no contact; the control shows
  the obstacle was in the way (the spine accepts the blind brain's straight path, and the twin sees the arm hit the
  crate), so it's the planner that avoids it. In the chain, the crate is no keep-out zone and no brain is told of it:
  only the camera can. The results are in `brain/README.md`.

## D69. Known limits

- The real arm hasn't run a planned path from this brain; that's for a session with the owner present, as is a real
  camera (perception's map has only seen the twin's ideal depth here).
- The map counts unknown space as empty (`docs/world-map.md`). With one camera, the faces of an object turned away
  from it are known only within perception's hidden band (15 cm behind what it sees): a tall object's far side, lower
  down, is unknown, and a path may pass through it. The chain's crate (180 mm, under a camera looking down) is covered
  where the arm goes; more cameras, or the occupancy's unknown state, are the way to be more cautious.
- The map is read once per plan; what moves after that isn't planned around until the next plan.
- cuRobo's covering of the base grows it by up to 20 mm at mid-height, so a move with the wrist near the base may need
  the stub's search, or find nothing where the spine's check alone would pass.
- A path is cut along cuRobo's route; it isn't the fewest segments possible. Waypoints anywhere (not only on the
  route) could cut more, at the cost of more checks. Around the chain's crate, the graph search goes over its far side
  (the tool 236 mm up), where a shorter way around exists.
- The twin and the planner read the same `/state`; a free obstacle pushed after the plan is made isn't re-planned
  around (the spine's freshness checks are about the arm and the cell, not obstacles).

## D70 to D79: perception

Decided on 2026-10-22, building perception for the brain (`brain/perception/`): an obstacle map from depth cameras,
for the planner, and the cameras' calibration to the arm (`brain/README.md`, "Perception"). The map's contract is
`docs/world-map.md`; the planner session accepted it unchanged.

## D70. The map is perception's own, on the CPU: a dense voxel grid with evidence, and an ESDF from it on request

- **Decision:** one dense grid (2 cm voxels over the arm's reach, 1.67 million of them) holding each voxel's evidence
  as log-odds. A frame adds hits where its points land and misses where its camera sees past a voxel's centre (each
  voxel is projected into the image once per camera pose, so a frame costs a gather, not a ray per pixel; with a lens's
  distortion, only voxels whose direction is within the image's field of view, as its own pixels' rays bound it, since
  the distortion polynomial folds directions far off the axis back into the image); the signed distance field comes
  from SciPy's exact Euclidean distance transform of the occupancy, once per revision, when asked. NumPy, SciPy and
  OpenCV only, in an environment of its own.
- **Alternatives:** nvblox through nvblox_torch (GPU TSDF and ESDF, what cuRobo's examples use); cuRoboV2's own GPU
  mapper (`curobo/_src/perception/mapper`, block-sparse TSDF and ESDF in Warp); a TSDF of our own; OctoMap.
- **Why:** both GPU mappers live in the planner's process and its torch and CUDA versions, while the map has to cross
  processes with a contract both sides can test alone (the coordinator's brief), and the 12 GB GPU is shared. On this
  PC's CPU a 640 x 480 frame integrates in 70 to 100 ms and the ESDF takes 0.2 s, ample for one camera at 5 Hz and a
  planner that asks once a step. It runs, and is tested, on Windows and in WSL2 alike. Evidence over frames, rather
  than a TSDF's averaged surface, gives the occupancy the planner and a person can read directly, and clears an
  obstacle that has moved. Should a larger or finer map be needed, cuRoboV2's mapper can be fed the same frames.

## D71. The map is served over HTTP from WSL2, on 127.0.0.1:8780, as JSON and .npz

- **Decision:** perception serves `GET /v1/map` (a description), `GET /v1/map.npz` (occupancy and ESDF arrays), `GET
  /v1/map/occupied.json` and `POST /v1/reset`, with a long poll (`after`, `wait`); loopback only, refusing web pages
  as the jog panel and the twin do. Version 1, fields added within it, as the brain interface does (D42).
- **Alternatives:** shared memory or files (one machine, but no freshness, no versioning, awkward across WSL2's
  boundary); ZeroMQ or gRPC (dependencies for both ends); ROS (not on this PC).
- **Why:** WSL2 can't reach Windows' `localhost`, so perception runs beside the planner in WSL2, and Windows still
  reaches it at `127.0.0.1` (D41). HTTP and NumPy's .npz are in both sides' environments and readable with curl. The
  arrays are about 5 MB with a float16 ESDF, 0.03 to 0.3 s a request on loopback.

## D72. Four states: free, occupied, unknown, and hidden space assumed solid up to 15 cm behind what was seen

- **Decision:** a voxel up to `hiddenM` (0.15 m) behind a surface a camera saw (not the arm's, the cart's or the
  floor's) is "hidden" and counts as an obstacle in the ESDF, unless a camera has seen it empty within the last 3
  frames; the arm being there ends it. Unknown space counts as empty, and the occupancy says which is which.
- **Alternatives:** unknown as empty everywhere (nvblox's default); unknown as solid everywhere.
- **Why:** a fixed camera sees the front of things. With unknown as empty, a planner could take the arm behind a box,
  through its unseen back half; with unknown as solid, the arm's own shadow and everything outside the view would be
  obstacles, and the arm would start every plan in collision. Hidden space covers the backs of objects (an object's
  depth along the view up to 15 cm) without either. The cost: objects look up to 15 cm deeper than they are, away
  from the camera.

## D73. The ESDF is a lower bound on clearance, in cuRoboV2's layout and sign

- **Decision:** outside obstacles, the distance between a voxel's centre and the nearest obstacle voxel's centre, less
  half a voxel's diagonal; inside, minus the distance to the nearest free voxel's centre plus half a voxel; metres,
  negative inside, arrays `[x][y][z]` in C order over a grid of even counts, so cuRoboV2's `VoxelGrid` (posed at the
  grid's centre) takes it as it is. Grids whose counts cuRoboV2 would misread (it truncates float32 dims / voxel) are
  refused.
- **Alternatives:** distances between centres (overstates clearance by up to 17 mm at 2 cm); positive inside (cuRobo
  0.7's sign).
- **Why:** a sphere the ESDF calls clear then touches no obstacle voxel, and every seen surface is inside one: in the
  twin it was never more than the distance to what was seen (by 3.2 mm at least). The planner subtracts its own half
  voxel for interpolation. The layout and sign were read from cuRoboV2 at the commit the planner uses (78fd485) and
  the suite re-implements its lookup against the published map.

## D74. The arm is left out by its capsules at the status frames around each frame; the base, cart and floor by utra_model

- **Decision:** joints come from the controller's 10 Hz status stream (port 30001, only listening). For each frame,
  the status frames from the last before it to the first after it bracket where the arm was; if they differ, the arm is
  left out at poses between them, close enough together that no capsule moves 12.5 mm between two. Points within the
  capsules grown by `padM` (25 mm) plus three times the depth's noise are dropped, and voxels within `padM` aren't
  updated by the frame. A frame is skipped unless the status frames around it, each with the one before it, all
  arrived on the stream's 10 Hz cadence (they carry no time of their own, so after a stall the burst of held-up frames
  would pass for fresh ones, and the first frame after a burst can still be late), as is one
  where the arm moved more than 30 degrees, or one from a camera on the flange taken while the arm moved at all. The
  base, the cart (20 mm) and the floor (20 mm) are left out the same way from `utra_model`'s shapes, and the active
  tool's capsules from the cell.
- **Alternatives:** the joints the twin renders with (exact, but a real camera has no such thing); rendering the arm's
  meshes into a mask image (MoveIt's self-filter: needs a renderer, and the meshes rather than the shapes the spine and
  planner use); segmenting the arm from the image.
- **Why:** the capsules are fitted around the arm's meshes (in the twin, every point the cameras saw of the arm was
  inside them), and they're the spine's own model. Bracketing covers the time between a frame and the status frames:
  in the twin, the arm sweeping at up to 60 deg/s left nothing in the map, where joints 0.5 s stale (a Wi-Fi stall,
  D54's captures had one of 3 s) left 410 voxels of it, hence the skip. Keeping what was known within the pad means an
  obstacle beside the arm, seen before the arm came, stays in the map.

## D75. Evidence and margins: two frames to show, four to clear, three sigma of a D455's noise

- **Decision:** a voxel is hit when at least 2 of a frame's points land in it; log-odds +0.85 a hit, -0.4 a miss,
  between -1.5 and 2.5, occupied above 1.0. Misses and the arm's and floor's margins allow three standard deviations
  of the depth's noise, modelled as sigma = 1 mm + 2 mm/m^2 times the distance squared (about 5.5 mm at 1.5 m), and
  pixels at depth discontinuities (where stereo cameras return "flying pixels" between near and far) are dropped.
- **Alternatives:** one frame to show (fast, but a single noisy frame makes an obstacle); averaging depth over frames
  first.
- **Why:** an obstacle shows within 0.4 s at 5 frames a second (0.6 s where the space was seen empty), and one that
  has moved clears in 0.8 s; obstacles appear faster than they go. With `noise.py`'s model of a D455's flaws, and with
  twice its noise, the twin's maps had no voxel more than 5 cm off an obstacle and none of the arm.

## D76. Calibration: a ChArUco board, hand-eye with both unknowns refined on every corner, views captured passively

- **Decision:** a ChArUco board (6 x 4 squares of 60 mm, DICT_4X4_50) on the flange for a fixed camera, or fixed in
  the cell for one on the flange. OpenCV's hand-eye solvers (Park, Tsai, Horaud, Daniilidis: the best fit) start it;
  the camera's pose and the board's are then refined together by least squares on every corner's reprojection error
  (a soft-L1 loss), views that don't fit are dropped and named, and the uncertainty comes from the fit. Views are
  saved by `calibrate capture` each time the arm rests at a new pose: a taught program or the jog panel moves it. The
  camera's intrinsics are its own.
- **Alternatives:** a board at a measured spot (its pose would have to be measured to the accuracy wanted); a sphere
  on the tool found in depth (a depth camera's bias goes straight into the pose); registering the arm's meshes to the
  depth (no fiducial at all, but less accurate, and nothing to check it against yet).
- **Why:** nothing needs measuring: the board's mount is solved too. ChArUco corners are known by id, so a board the
  arm partly hides still counts. In the twin a fixed camera came out within 0.5 mm and 0.03 deg of its true pose
  (tolerance 2 mm and 0.2 deg), a view with wrong joints was dropped, and joints read 0.02 deg off cost 1.3 mm.
  Perception never moves the arm: the brain only proposes (D40).

## D77. The board in the twin is two obstacles moved with the flange; the twin is unchanged

- **Decision:** the board's black parts are a mesh (`board.write_stl`, generated from OpenCV's own board image, so
  exact) over a white plate, placed as two ordinary obstacles and moved with the flange by whoever drives the twin (the
  tests). The camera's true pose is used only to choose poses where the board faces it, as a person would by looking.
- **Alternatives:** textures in the twin's model (a change to the twin, which wave 2's other session also touches);
  drawing the board into the twin's images (the calibration wouldn't see the twin's renderer).
- **Why:** the twin's renders are what a real camera's images stand in for, aliased edges and all (corners found to
  0.4 px RMS), and the twin needed no change.

## D78. Cameras plug in through a seam that carries only what a real camera gives

- **Decision:** a camera source returns depth in metres aligned to colour, the colour image, its intrinsics and
  distortion, and when it was taken. The twin's cameras are read over HTTP with their true pose and segmentation
  dropped; saved frames replay from a folder. No RealSense support until there's a camera to test it on.
- **Alternatives:** a RealSense source written now, untested; using the twin's true pose until a camera arrives.
- **Why:** everything above the seam already runs as it will with a real camera, including its calibration, and the
  margins assume a D455's noise. Driver code that has never seen the device would be the part most likely wrong.

## D80 to D89: the README and its pictures

Decided on 2026-10-22, shortening the README into a landing page with pictures of the software at work
(`tools/media/README.md`).

## D80. The README is a landing page; its sections moved, word for word, to pages in `docs/`

- **Decision:** the README says what this is, shows it, gives a quick start (the simulator first) and keeps the
  safety notes; everything else moved unchanged to `docs/connecting.md`, `docs/jog-panel.md`, `docs/programs.md`,
  `docs/python.md`, `docs/troubleshooting.md` and `docs/repo-layout.md`, with only their pointers to each other
  rewritten. Pointers into the old sections (`scripts/arm.py`'s messages, `DESIGN.md`, D23, D39, the specification)
  name the new pages.
- **Alternatives:** trimming the README in place; a documentation site (MkDocs and the like).
- **Why:** at 33,000 characters with no pictures, the README buried what the software does. Moving the text
  unchanged loses nothing and reviews as a move; plain Markdown pages need no tooling and read the same on GitHub.

## D81. The pictures are filmed from the real software against the simulator and the twin, by committed scripts

- **Decision:** the panel's GIFs are the real page and jog server, driven by Playwright with real mouse moves, against
  `tools/utra_sim.py` (freedrive's hand is its control port's); the obstacle clip is the real chain (the twin,
  perception, the planner and the jog server) recorded in WSL2 and drawn again from a scene camera. The scripts and
  their environment are in `tools/media/`; each GIF is 960 pixels wide at 12.5 frames a second, under 3 MB.
- **Alternatives:** screen recordings made by hand (can't be made again when the panel changes); mock-ups; filming the
  real arm (needs the owner, and can't be scripted).
- **Why:** the pictures show what the software does, and one command makes them again after a change. Nothing
  touches the real arm: every process runs on loopback addresses of its own, clear of the owner's.

## D90 to D99: the twin's viewer and its cadence

Decided on 2026-10-22 (night). Watching the twin in MuJoCo's viewer through WSLg, the owner found it "super slow.
Just a handful of FPS", with "odd balls surrounding the geometry", and chose: "Make the twin's viewer smooth: a native
Windows viewer, camera rendering off the main loop, and closing it no longer stops the twin". Newton was set aside:
physics isn't the bottleneck (the twin follows `tools/utra_sim.py`), and in WSL2 its viewer would take the same WSLg
path.

## D90. The viewer is a process of its own that follows the twin over HTTP, read-only; on Windows, natively

- **Decision:** `python -m brain.twin view --follow URL` opens MuJoCo's own viewer (its passive mode) in a process of
  its own. It takes the model from the twin, `GET /model`: the generated MJCF and its meshes, which it checks against
  the twin's fingerprint of them (a SHA-256 of both) and compiles. Then it follows the twin's state as the twin streams
  it, `GET /pose/stream`: a JSON line for every step of the twin's 10 ms loop, with the arm's joints and every
  obstacle's pose, fixed or free. When the fingerprint in the stream changes (the twin rebuilt its model: an obstacle
  or a camera added or removed), it loads the new model into the same window and keeps the view. It makes GET requests
  only. `serve --viewer` starts it as a child process. Closing it ends that process only; Ctrl+C stops the twin. In
  the chain the twin runs headless in WSL2, and the viewer natively on Windows, from `brain\.venv`, through WSL2's
  localhost forwarding.
- **Alternatives:** the viewer in the twin's process, as it was (through WSLg, a few frames a second; closing it
  ended the twin's loop and with it the simulated controller; and its rendering would share the twin's GIL, D91);
  building the model on the viewer's side from the same scene and cell (the scene's paths are WSL2's, obstacles added
  over HTTP would have to be described in full, and builds on Windows and Linux differ in the last digit of a few
  numbers, from each C library's sin and cos, so a fingerprint couldn't confirm them the same); polling `GET /state`
  (it runs on the loop and works out contacts and zone clearances: a hundred times a second would load the loop);
  Newton's viewer (above).
- **Why:** measured on this PC, the twin in WSL2 and the viewer on Windows: all 100 states a second arrive (with
  Nagle's algorithm off on the twin's HTTP connections; with it on, 71 did), and MuJoCo's own counter reads 144 frames
  a second, the screen's refresh rate with vertical sync on, while perception pulls frames and the arm moves. A frame
  of the window costs about 4 ms on the RTX 3080 Ti. Closing the window as its close button does ends the viewer with
  exit code 0, and the twin's controller, loop and HTTP carry on. The model shown is the twin's, byte for byte, and
  the viewer can't change the twin: whatever is changed in its window (a joint slider, a perturbation) changes only
  its own copy of the state.

## D91. Camera frames are rendered in a process of their own, from snapshots of the loop

- **Decision:** every step, the twin's loop publishes a snapshot of its state: copies of the joints, the free
  obstacles and the mocap poses, with the build they belong to. A frame request waits for the first snapshot after it
  arrived and hands it to the camera process (Python's multiprocessing, spawned). That process starts with the twin
  and gets ready in the background: it's sent the model and compiles it, makes OpenGL contexts and an MjData of its
  own, and renders each camera once; it does the same for each new build. For a frame it renders the snapshot's state,
  encodes it (`.npz` or PNG) and sends back the bytes. One frame at a time; a process that dies, or doesn't answer
  within 10 s, is replaced at the next frame. On Linux it keeps to the upper half of the CPUs. `/pose`, its stream
  and `/model` are answered from snapshots too; `/state` and the POSTs still run on the loop, between its steps.
  `GET /loop` reports the loop's cadence.
- **Alternatives:** a render thread with its own context in the twin's process (MuJoCo holds Python's GIL while it
  loads a model onto the GPU, in `MjrContext`: 45 ms on Windows and 180 to 380 ms in WSL2, at the first frame and
  after every rebuild, stalling the loop and the simulated controller's threads); rendering on the loop less often
  (each frame would still hold it up 83 ms in WSL2); cheaper frames, without shadows or reflections (52 down to 11 ms
  for the colour in WSL2, but the images perception is built against would change); confining the twin's loop to
  some CPUs instead of the renderer (measured: it doesn't help, the renderer's threads still land on the loop's CPUs).
- **Why:** rendered on the loop, as they were, perception's frames (5 a second) held it up about 40% of each second
  in WSL2, 83 ms a frame. In an experiment with that arrangement, 95% of ticks came within 77.3 ms of the one before
  in WSL2 (the worst 113.3) and within 18.8 ms on Windows (the worst 41.6). Now, with idle seconds interleaved for
  comparison (`tests/twin_tests.py`; `brain/README.md` has the table), 99% of ticks come within 10.5 ms of the one
  before under perception's load and with frames pulled back to back from two clients, on Windows and in WSL2, none
  late, and the controller's status stays 99.6 to 100.4 ms apart (5th to 95th percentile). Across the boundary, with
  perception itself pulling frames from the twin in WSL2: 2000 ticks in 20 s, 99% within 10.15 ms of the one before,
  the worst 11.47. The frames are pixel for pixel what `Twin.render` gives. The CPUs: in WSL2 the GPU driver's work
  delays other threads' wake-ups on the CPUs it runs on. With frames rendered back to back, a 10 ms loop in another
  process had 99% of its ticks within 16.8 ms and the worst at 20.8; with the renderer kept to 8 of the 16 CPUs, 10.1
  and 10.4 ms, the renderer making 64 frames in 6 s instead of 68. Kept to 4, the loop did no better and the renderer
  made 57: perception's requests took 114 ms (median) against main's 92, in an A/B of the same sweep, where with 8
  they took 99 and 104 ms against main's 92 and 97 (two runs each). That A/B swept the arm over the table, J1 +/-35 deg
  at 45 deg/s, with perception mapping the overhead camera, then parked it clear: the maps 5 s later were the same
  with both twins (no voxel in space the camera sees through, none occupied more than 3 cm from a true surface), and
  in every frame the arm was among the poses perception left out. Getting ready as the twin starts: started with the
  first frame asked for instead, the process made perception's first frames wait for it, so in the planner suite's
  chain, under the full run's load, the crate showed in the map 7.3 s after perception served it, against 1.2 s
  before. Ready beforehand, the first frame takes 69 ms on Windows and 128 ms in WSL2, and so does the first after a
  rebuild; in that chain the crate now shows 0.6 s after perception serves its map.
- **Known limits:** rebuilding the model (an obstacle or a camera added or removed) still runs on the loop. It pauses
  the twin's following for about 50 ms on Windows and 170 to 200 ms in WSL2, once per change; the simulated
  controller doesn't pause (the build releases the GIL), and moving an obstacle doesn't rebuild. Other processes' GPU
  work in WSL2 isn't confined by this: while another twin rendered its viewer through WSLg beside it, a twin's idle
  loop had 99% of its ticks within 21 ms.

## D92. The viewer's markers are sized for the arm (`statistic meansize`), and the viewer keeps the model's look

- **Decision:** the generated model sets `<statistic meansize="0.05">`, so the viewer draws centres of mass (key M)
  as balls 40 mm across, and joint axes and frames 50 mm long. The viewer keeps the model's visual settings (a 4096
  shadow map, reflections).
- **Alternatives:** the compiler's own meansize, 0.40 m from the extents of the cart and the floor (the "odd balls":
  centres of mass drawn 320 mm across, joined up by auto connect, key A); lighter viewer defaults (a smaller shadow
  map).
- **Why:** meansize scales only what the viewer draws to explain the model; cameras draw none of it (they render with
  MuJoCo's default options, which leave it all out). Natively the shadow map costs nothing measurable: 4.4 and 3.7 ms
  a frame at the window's size with a 4096 and a 2048 map, reading the pixels back included, which the window doesn't
  do. The window keeps up with the screen, so nothing was taken away.

## D100 to D104: reduced speed, hold-to-move and the launcher

Decided on 2026-10-23, building what the owner's modes app asks of the spine. The app (Survey, Teach, Rehearse, Run and
Review) starts the jog server itself and serves its own web app to a tablet and this PC, forwarding to the spine over
loopback; its teaching mode is "reduced speed and a hold-to-move key". The requirement: the spine stays the only thing
that moves the arm, and "the safety rules must be enforced here in the spine, not in a web page" or in the app.

## D100. The manual-motion policy is the spine's own

- **Decision:** reduced speed and hold-to-move are settings of the jog server: off unless `--reduced-speed` and
  `--hold-to-move` start it so, changed by `POST /api/manual`, reported by `GET /api/status` (`manual`), and enforced
  where moves are made: the jog and planned-move paths, the spine's planner (`scripts/planner.py`) that every program
  step, move to a target and brain's proposal is planned with, and the runner's supervision. A change is refused while
  the arm moves (a jog or planned move under way, or a run that isn't paused). With both off, every speed sent and
  every message is as before.
- **Alternatives:** enforcing them in the modes app or its web page (neither moves the arm, and anything reaching the
  spine another way would get round them); a setting per request (any client could ask for full speed); changing them
  mid-move (a move planned under one policy would run under the other).
- **Why:** the spine is the one thing every move goes through, so a rule kept there can't be got round, and the
  checks stay small enough to test here against the simulator. The app only asks.

## D101. Reduced speed: a joint move is capped by a bound on its lever arms

- **Decision:** with reduced speed on, every joint move (a jog step, a planned pose, a program's joint move, each
  segment of the brain's paths and bursts) is sent no faster than keeps both the flange and the tool point in use under
  250 mm/s: `planner.lever_bound`, the sum of each joint's share of the turn times the point's distance from its axis
  (joints turning together add up), taken where no joint turns more than 1° between samples, plus the most that sum
  can grow between them (a point's distance from an axis changes no faster than the joints beyond it turn, times its
  reach: at most the links from the shoulder, 1145 mm, and the tool's offset). The speed is lowered, never refused,
  and the message says why ("..., keeping the tool point and flange under 250 mm/s (reduced speed)"). A program's
  `"fullSpeed": true` is overridden. Without the arm model, every joint is taken to swing the tool 1150 mm out.
- **Alternatives:** D45's bound as it was (the tool point only, at 17 samples, so not a bound between them); the page's
  redline (one joint at a time from the pose now, and only a warning); refusing moves in the red (the requirement was
  to slow them); one cap from the arm's full reach (about 12 °/s for every joint, even the wrist's).
- **Why:** by the triangle inequality the sum bounds the speed of any point the turns carry, and the growth term makes
  it hold between samples, so the cap holds for the actual pose and the joints that turn. Tested in-process, 46 random
  moves at their caps, sampled at 400 points, never passed 250 mm/s; with one joint turning the bound is exact but for
  the 0.1 °/s rounding (248.9 mm/s at least). When several turn it's up to about 40% cautious, as their contributions
  seldom line up: a planned move of J1, J2 and J6 was capped at 11.1 °/s, which carried the tool point at 177.8 mm/s
  at most (177.5 measured in the simulator).

## D102. Reduced speed: a straight move, and how fast a turn is taken to go

- **Decision:** a tool step or a program's straight line is sent at 250 mm/s at most, the controller's line speed
  being the flange's, and slower when it turns. Each segment is taken to last at least its length, or its turn at
  25 mm a radian, over the speed sent, while a point r mm from the flange travels at most the length plus the turn
  times r; the speed sent keeps that under 250 mm/s (`planner.reduced_line_speed`). So a 5° turn of a 200 mm tool,
  asked for at 1000 mm/s, is sent at 31.2 mm/s, and a slide at 250.
- **Alternatives:** the simulator's pace for a turn, 1 rad as 100 mm (D11, unverified); the arm's fitted pace, 1 rad as
  50 mm (D54, from four turns of one size); bounding the flange only (a long tool's point would sweep past 250 mm/s
  on a turn).
- **Why:** how the controller paces a turn isn't known. The arm turned faster than the simulator assumes, and D54's
  fit rests on four turns, so the bound allows for turns twice as fast as fitted; slides, which don't turn, keep the
  full 250 mm/s. Measuring turns of 5, 30 and 90° at 10, 25 and 100 mm/s (D54) would let it be relaxed. Known limits:
  as for the redline, only the tool point and the flange's centre are bounded, not what sticks out further; and the
  controller's own speed error (long lines ran 0.25% fast, D54) isn't allowed for.

## D103. Hold-to-move is a hold renewed every 0.1 s, which the server watches

- **Decision:** `POST /api/hold {"ms": N}` holds for N ms from now (100 to 1000, 400 by default) and `{"ms": 0}` lets
  go. With hold-to-move on and no current hold, jogs, planned moves, moves to targets and starting, resuming or
  stepping a run are refused, saying why; a dry run needs none, and Enable, STOP, freedrive, the cell's API and
  pausing or stopping a run work regardless. When the hold lapses or is let go while the arm moves, a jog or planned
  move is stopped by the watchdog as STOP does, within its 0.1 s cadence (a let-go wakes it at once); a program's run
  pauses (its supervision checks every 0.05 s), and Resume with the key held again re-plans from where the arm
  stopped; a move to a target stops, as it does when the page goes quiet. Freedrive is never stopped by it. The hold
  is the server's, not a client's: while it's held, any client with the token may move the arm.
- **Alternatives:** a hold per client (behind the modes app the server sees one client, and the app holds for the
  device that asked); stopping a run outright (the requirement: holding again and resuming carries on); pausing a move
  to a target (the page has no Resume for one); a hold-to-move key on this page (with a mouse, one key can't be held
  while another is clicked, and the requirement left it optional).
- **Why:** an enabling device must stop the arm both when it's let go and when its link fails, and a lease does both:
  a client that crashes or loses its Wi-Fi stops renewing, and the arm stops half a second later. Measured against the
  simulator: let go, a move stopped 1 to 16 ms after the server had the release; a lapse stopped one 0.42 to 0.51 s
  after the last renewal, which held for 0.4 s.

## D104. The launcher's token, and "simulated"

- **Decision:** if `UMBRATEK_PANEL_TOKEN` is set when the server starts, it's the page's token, so the program that
  started the server can call its POST API too. A value shorter than 16 characters, or with anything but URL-safe
  base64's letters, digits, `-` and `_` (so no `=` padding either), stops the server before it connects to the
  controller, with exit code 2 and a message that says what's wanted without echoing the value; the token is never
  printed.
  `GET /api/status` says `"simulated": true` when the controller's address is this PC's own: 127.0.0.0/8 or ::1
  (IPv4-mapped too), or localhost when it resolves to those alone. Any other address or name is reported live.
- **Alternatives:** a token file, or the token printed for the launcher to read (a file outlives the server, and a
  console may be logged); the launcher saying whether it started the simulator (it could be wrong; the address can't);
  resolving any name (a name for this PC is still a name).
- **Why:** the token still keeps other web pages and programs out while letting in the one that started the server,
  and checking it first makes a misconfigured launcher fail at once. Only the simulator and the twin can serve on this
  PC's own addresses, so "simulated" errs towards LIVE: a LIVE banner over the simulator costs some caution, the other
  way round could cost the arm.

## D105 to D124: tending basics

Decided on 2026-10-25, building what any machine-tending job needs before one is programmed: grippers that confirm a
grasp and switch the payload, grids of slots with a counter, interlocks that guard a machine's zone, recovery (Retry,
Fallback, safe home and calling the owner) and touch-up. They're general program and cell features; no job's program
is written here. The requirements: everything stays checked by the spine as before (dry runs, collision checks against
the zones and the tool, supervision and STOP), safe home's moves and whatever recovery and interlocks do included; the
HTTP API keeps its endpoints and fields; old cells and programs load unchanged.

A review of the work found a move from inside a guarded zone unchecked, and gaps in the payload's handling, the page's
watchdog and tool jogs. D120 to D124 and the revisions marked in D105, D106, D110, D111, D114 and D118 are its fixes
(D124 from a second look at them), decided the same day. A review of those fixes then revised D105, D106, D110 and
D121 again: an Enable the server runs on its own could still undo a STOP, and the page could slide the arm through a
closed zone while backing it out.

## D105. A gripper is bound to outputs, and optionally to inputs; without one to confirm, it takes a set time

- **Decision:** a gripper is a cell object (`grippers`): the outputs, with their values, that close it (one, or up to
  four, as a double-acting valve needs), those that open it, how long it takes (0.5 s by default), how long an input
  may take to confirm it (3 s), and three optional inputs: closed, opened, and closed on a part. A program's gripper
  step opens, closes or grasps: it sets the outputs, then waits for the input that confirms it, failing the step past
  the timeout, or, with no such input, waits the set time. The rows in the Cell section do the same, refused while a
  run is under way. STOP leaves the outputs as they are, so what's held stays held. A gripper a program uses can't be
  deleted.
- **Alternatives:** writing a gripper as set, wait and if steps, as before (nothing confirms it in one place, nothing
  names it, and the payload can't follow it); making it part of a tool (a tool may carry two grippers, one to take a
  finished part out and one holding the next blank, and tools and their API stay as they were).
- **Why:** the only gripper there is, the Chess Gripper, has no sensor, and how the controller will drive it isn't
  settled (an Arduino behind a tool output is one option). Bound to an output and a set time it works whatever drives
  it, and grippers with sensors confirm. Unverified until a gripper is on the arm: the Chess Gripper's time, and its
  drive.
- **Revised after review:** the rows' Open, Close and Grasp are refused while a jog or Move from the page is under way,
  and while one works (up to its timeout) no run starts, nor a jog, Move or freedrive; one works at a time. Before,
  only a run under way refused them, so a grasp during a jog wrote the payload mid-move, and a run could start while
  the gripper was still closing. Tested: a grasp during a Move is refused with the payload untouched, and a run and a
  jog are refused while a gripper takes 2 s to close. And, since the fixes were reviewed, a STOP pressed during a grip
  from the page (or from Foreman, through `/api/gripper`) is never undone by the Enable that may follow it (D106).

## D106. A grasp checks for a part where there's an input, and switches the payload to the tool and the part

- **Decision:** a grasp closes the gripper as Close does, then reads its "closed on a part" input, if it has one, and
  fails the step unless it reads true within 0.3 s; without that input, the close is trusted. Then the payload sent to
  the controller (`set_tcp_load`, D14) becomes the tool's and the part's together: their mass, and their centre of
  gravity from the flange, the part's given on the step in mm from the tool point, along the tool's axes (`"part":
  {"mass", "cog"}`). Opening the gripper sends the tool's alone; closing it without a grasp changes nothing. The server
  keeps what each gripper holds (`held`, in the status). In freedrive the payload waits, and is sent as freedrive ends.
  If writing it stops a ready arm, as writing the teach sensitivity does (D34), Enable's checked sequence makes the arm
  ready again, and the message says so.
- **Alternatives:** a payload step of its own (a program could grasp and forget it, and the grasp is when the part's
  weight arrives); switching on every close (closing an empty gripper, to pass through a gap say, would add a part
  that isn't there); a part's centre of gravity from the flange (it's known from where the part is held, the tool
  point).
- **Why:** the controller's dynamics and collision detection need the payload (D14), and a part changes it.
  Unverified: whether writing the payload stops the real arm (the simulator's `--payload-stops` plays the case where
  it does, and both cases are tested), and how much the payload's accuracy matters to the arm. Known limits: what the
  grippers hold is kept in memory, so after the server restarts they hold nothing, while the controller keeps the
  last payload written (the server writes none as it starts): a part held then is still counted until the next grasp,
  release, or Use of a tool. A grasp from the page doesn't know the part's mass.
- **Revised after review:** Enable makes the arm ready again after the write only if the status frame after it shows
  no controller error, no STOP has come since the write, and the run has no stop pending. An error then is a fault
  (the run faults, and the error is left for a person, where Enable would have reset it: a collision tripping during a
  grasp would have been swallowed); after a STOP, or with a stop pending, the arm stays stopped. A pause pending is
  no reason to leave it stopped: the run then pauses between steps, which leaves the arm ready, and a Resume would
  otherwise find it stopped and fault. A payload due when freedrive ends without sending it (STOP ends freedrive too)
  goes at the next Enable, before the motors come on, and Enable is refused if the controller refuses it; the arm
  can't move before then. Tested: the simulator's `--payload-error` raises a controller error as the payload is
  written, and the run faults with it unreset; the STOP, stop and pause cases in-process; a grasp in freedrive ended
  by STOP reaches the controller at the next Enable.
- **Revised again, after the fixes were reviewed:** that Enable could still undo a STOP. The gate read the stop count
  only as the payload was written, after a STOP in the grip's last moments had already counted, and judged the arm
  ready on a status frame up to 0.1 s older than that STOP; Enable itself never looked again, so a STOP while it
  switched the motors on was undone by its "ready"; and Enable reset any controller error that came after the one frame
  the gate read. Now:
  - the grip takes the stop count as it begins (`Panel.grip`), and every `stop()` counts as it begins and as it ends;
  - the arm must be seen ready on a status frame that came after the grip began and at least 0.3 s after any STOP ended
    (`STOP_SEEN_S`), with no STOP since the grip began and none under way (`apply_payload`);
  - the Enable runs as the server's own (`enable(auto=...)`): it gives way to any STOP since the grip began, checked
    again just before the motors are switched on, and after: a STOP that came while they were switched on is made
    again, before a jog or move can start; and it refuses a controller error rather than resets it, leaving
    `prepare_for_motion`'s own reset out (`reset=False`), so the grip fails with the error, which stays for a person.
  This holds for grips from the page and Foreman as for a run's. Tested: in-process, with STOP emulated as the gripper
  closes, as the payload is written, and 50 ms before the grip with no status frame since (the arm stays stopped
  each time); against the simulator, with a real STOP pressed as that Enable checks the joints, and as it switches the
  motors on, and a controller error appearing as it begins (the arm stays stopped, the error kept). Those checks fail
  on the code before this revision.

## D107. A pattern is taught by three targets, and its slots fill the parallelogram between them

- **Decision:** a pattern (`patterns`) names three tool targets taught in one frame with one tool: its first slot, the
  last slot of its first row and the last slot of its first column (the first and one other, for a single row or
  column), with its rows, columns and layers (stacked a set pitch apart along the frame's z, negative to stack
  downwards). Slot n, counted from 1 along the first row, then the next row, then the next layer, is the first slot's
  position plus its column times the row's step (the row's last slot less the first, over the columns less one), plus
  its row times the column's step, plus its layer times the layer pitch. It takes the first slot's orientation, and is
  solved from the corners' taught joints interpolated the same way, so it keeps their configuration. The pattern holds
  the targets' names, not copies, so touching one up or recalibrating its frame moves the grid. Each pattern is checked
  as it's saved: slots at least 1 mm apart, rows and columns 20° to 160° apart, a layer pitch when there's more than
  one layer, at most 100 a side and 10,000 slots. A corner can't be deleted while a pattern uses it.
- **Alternatives:** pitches typed in (a tray's are nominal; teaching its corners measures the tray as it sits); four
  corners, interpolated bilinearly (a tray isn't a trapezoid, and a fourth corner is one more to teach and keep
  consistent); each slot's orientation interpolated from the corners' (a tray's pockets all face one way, and
  interpolating three orientations isn't well defined); copies of the corners' poses (touching up a corner would leave
  the grid where it was).
- **Why:** three points fix a flat grid whose rows and columns needn't be square (a tray set down skewed), and the
  corners are ordinary targets, taught and touched up as any are. Checked: 200 random grids, every slot against a
  ground truth built with numpy, within 3.8e-13 mm (`tests/feature_oracle_tests.py`); a 3 x 4 tray in the simulator,
  visited slot by slot, each slot within 1 mm of where the simulator's joints put the tool point (`tending`).
  Unverified: a 3 x 4 tray taught at three corners on the arm, the pointer within 0.5 mm of each pocket's centre.

## D108. A pattern's counter is kept apart from the cell, in `cell/counters.json`, and moved on by a step of its own

- **Decision:** a pattern's counter is the next slot to use, from 1; one past the last means used up. Counters are kept
  in `cell/counters.json`, written whole and atomically at each change, so they carry on after a restart, and they're
  not part of the cell's revision. A move to a pattern without a slot goes to the slot its counter stands at, and fails
  if the pattern is used up. A count step moves the counter on by one, or sets it (`"to": n`, from 1 to one past the
  last); Reset on the page puts it back to 1. A dry run counts on a copy. Deleting a pattern drops its counter;
  changing one keeps it. `GET /api/status` reports them (`counters`).
- **Alternatives:** counters in `cell.json` (every count would bump the cell's revision, and an edit made on the page a
  moment earlier would be refused: the run and the operator would fight); counters in memory (a restart would start a
  half-full tray again from slot 1); a move that counts itself on (a failed pick would skip its slot, and a retried
  place would use the next; with a step of its own, the program counts once the part is confirmed).
- **Why:** how a counter persists was ours to decide, and a counter is the run's state, not the cell's. Tested: kept
  through a restart, reset, set, used up (the move fails), and counted on a copy in a dry run.

## D109. An interlock is a named condition on an input, and guards a zone

- **Decision:** an interlock (`interlocks`) is an input and the value it holds at (a machine's door open:
  `controller.2` reads 1). A keep-out zone may be guarded by up to four (`"interlocks"` on the zone): the arm may enter
  it while they all hold, and it's a keep-out zone while any doesn't. An interlock that guards a zone can't be deleted.
  The page shows each interlock's lamp, lit green while it holds, read once a second.
- **Alternatives:** waits and ifs before the moves into a machine, as today (a program can leave one out, and jogs, Move
  and the brain's paths never see them); conditions on steps (each step into the machine would carry them, and moves
  that aren't steps would go unchecked); a zone's own unnamed input (a name says what's wrong in a refusal, and one
  interlock can guard several zones).
- **Why:** "checked before every move into a zone the condition guards": tying the condition to the zone makes "into
  the zone" the collision check's question, which every move is already asked, whatever made it. An old cell's zones
  have no interlocks and stay keep-out zones. Known limit: an interlock isn't a safety function. It's read through the
  controller's ordinary inputs, a tenth of a second apart (D111), and a machine's own guarding protects people.

## D110. A move is planned against every zone; a guarded zone opens only for a move that needs it, read just before

- **Decision:** every move is planned with every zone closed first. If a guarded zone blocks it, that zone's
  interlocks are read: if they all hold, it's planned again with the zone open; otherwise it's refused before anything
  is sent, saying which interlock doesn't hold and what its input reads. The same goes for program steps, moves to
  targets and slots, safe home, and the page's jogs and Move (the 3D view's own check uses the interlocks as last
  read). The brain is told only of the closed zones; the shell checks its paths against those, and the zones a path
  enters are read again just before it runs. A plan carries the guarded zones it enters. A dry run assumes the
  interlocks hold, as it assumes inputs arrive, and logs each zone it assumed open.
- **Alternatives:** reading every interlock before every move (moves nowhere near a machine would read inputs too, and
  fail when they can't); opening zones from a reading kept by a poll (up to a poll old when the move starts); telling
  the brain the interlocks (its interface would change, for something the spine decides anyway).
- **Why:** a move that never comes near a guarded zone is planned and checked exactly as before, with nothing more
  read, and one that does reads its interlocks at the last moment before it's sent. Since the collision check decides,
  everything checked against zones is covered, dry runs and the page's check included. A move out of a zone needs no
  interlock: the check already lets the arm move away from what it's within the margin of.
- **Revised after review:** "blocks it" missed a move from inside a guarded zone, which the check never saw going
  further in (D120); and "a move out of a zone needs no interlock" no longer holds for a guarded zone. A move that
  starts in a guarded zone needs its interlocks whatever it does there, and nothing moves the arm in one whose
  interlocks don't hold, the page's jogs and Move included (D121).

## D111. A move into a guarded zone is watched: if an interlock stops holding, the arm stops and the run pauses

- **Decision:** while a move into a guarded zone runs, its interlocks are read every 0.1 s (`INTERLOCK_POLL_S`) along
  with the run's other checks. If one stops holding, or can't be read, the arm stops as for Pause: a program's run
  pauses and calls the owner (D115), and a move from the page stops. Resume plans the rest again, reading them again,
  and is refused while one doesn't hold.
- **Alternatives:** checking only before the move (a door shut mid-move would go unnoticed); faulting the run (a door
  shutting is a machine doing its job, and the run can carry on once it's open again).
- **Why:** measured in the simulator, with the door's input dropped during a slow move into the zone: the run had
  stopped the arm and paused 0.14 s later (the tending suite's bound is 1 s). Known limit: as D109 says, it isn't a
  safety function.
- **Revised after review:** a move from inside a guarded zone is watched too (D121), and "Resume ... is refused" is now
  what happens: a run paused this way that's resumed while the interlock still doesn't hold plans the rest again, is
  refused, and stays paused, saying so, with nothing sent (before, a resumed move from inside went on in, unwatched).
  The interlocks are read without holding up the other reasons to stop (D122).

## D112. A step that fails is caught by Retry and Fallback; a fault never is

- **Decision:** a step *fails* when it can't do its job, with the arm at rest: it can't be planned or reached, the
  brain has no path, a wait or a gripper's input times out, a grasp finds no part, a pattern is used up, an interlock
  keeps a zone closed. A *fault* is the controller reporting an error or losing status, the arm leaving its path or not
  arriving, the brain's proposal refused, or a bug. A Retry or Fallback catches failures inside it, and carries on only
  if the arm is still ready (fresh status, no error, motors on, not in freedrive); otherwise the run faults. A failure
  nothing catches stops the arm and faults the run, as every failure did before; the log says `step.failed` before
  `run.faulted`. STOP wins over both.
- **Alternatives:** catching faults too (an arm off its path, or a controller error, needs a person, not another try);
  catching by kind, written in the program (more to write and get wrong; it can come when a job needs it).
- **Why:** a failure happens before anything is sent or once a move has arrived, so the arm is where the program
  thinks; a fault leaves that in doubt. Programs without Retry or Fallback run as before.

## D113. Retry runs its steps again from the first; a Fallback's "If it fails" steps are numbered on from its own

- **Decision:** `retry {"tries": n, "steps"}` (3 by default, at most 100) runs its steps, and when one fails, runs them
  again from the first, up to n tries in all; then the failure goes on out. `fallback {"steps", "else"}` runs its
  steps, and when one fails, its else steps. The else steps' paths are numbered on from the steps' (under a fallback at
  4 with three steps, the else steps are 4.4 on), so every step has one path, and Run from one of them goes straight
  there. Each try is planned afresh from wherever the arm is: nothing of the failed step is resumed.
- **Alternatives:** retrying only the step that failed (a missed pick needs its approach again: what's inside the
  retry says what's tried again); else paths of their own form (the page and Foreman would have to learn it); numbering
  else from 1, as an if's branches are (two steps would share a path, as an if's still do).
- **Why:** the build line asks for Retry(n) and Fallback; a step's path is how the page, the log and Foreman point at
  it, so it had to stay one number per level.

## D114. Safe home backs out along the tool, as far as it's clear, then makes a checked joint move

- **Decision:** `home {"target", "retract": mm, "speed"?}`: a straight line from where the tool point is, back along the
  tool's own z (the way it went in) by the step's distance (50 mm by default, 0 to 500), then a joint move to the
  target, each planned and checked as any move is, guarded zones included. If the whole line can't be planned, the arm
  backs out as far as it's clear (the line's clear part, if it's 1 mm or more), and the log says how far and what
  stopped it; then the joint move's own check decides whether home can be reached. A pause during either carries on
  with the rest. On the page, Go home, beside the run keys, does it as a move from the page (stopping if the page goes
  quiet), to the target and distance chosen there.
- **Alternatives:** straight up, along the base's z (a tool reaching into a vise, under a gantry or across a printer's
  bed may have gone in sideways; along the tool is the way it came); back to the last approach target (the program
  would have to know it, and a failure can come anywhere); a joint move straight home (it may drag the tool, or the
  part, through what it's in); refusing when the line is blocked (the arm would stay inside the machine).
- **Why:** the build line: retract along the tool, then a checked joint move home. How it picks its retract was ours to
  decide. Tested in the simulator: 40 mm back along the tool (the line sent ended 0.00 mm from its goal), then home;
  with a shelf over the wrist, 14 of the 60 mm asked for, saying why, then home.
- **Revised after review:** with the arm in a guarded zone, safe home's moves need its interlocks and are watched, as
  any move from inside one is; while they don't hold, safe home doesn't move the arm (D121), as its joint move could
  drag the tool through the machine. Tested: from inside the machine, its joint move stops within 0.12 s of the door
  shutting, and with the door shut it's refused, nothing sent.

## D115. Calling the owner: a call kept by the server and shown on the page until it's seen

- **Decision:** a `notify` step calls the owner with its message, and the run carries on; a run that faults calls the
  owner too (unless it's a dry run or a move from the page), as does an interlock that pauses a run. A call has an id,
  a time, its message, program, step, and why (a step, a fault or an interlock). It's written to the run's log, kept by
  the server (the last 50) and listed by `GET /api/status` (`calls`) until it's seen; `GET /api/calls` lists them all,
  and `POST /api/calls/ack` marks one, or all, as seen. The Program section's Calls list shows them with Seen keys, and
  the message line says each new one. A dry run only logs what it would have called.
- **Alternatives:** a phone notification now (an account, a service and a secret in the spine; the build line says
  later); a halt (it pauses the run, which a call needn't).
- **Why:** one list that the page, Foreman and a notifier later can all read, with nothing leaving this PC. Known
  limit: calls are kept in memory, so a restart clears them; the run logs keep them.

## D116. Touch-up re-teaches a target in place, says what changed, and keeps the old value

- **Decision:** Touch up, on a target's row, re-teaches it where the arm is: a tool target keeps its frame and tool
  (which must be the tool in use) and takes the tool point's pose in that frame, with the joints as its seed; a joint
  target takes the joints. The message says how far it moved, along its frame's x, y and z, how far it turned, and
  which programs and patterns change with it; the row says so until it's taught again; the 3D view rings where it was,
  with a line to where it is. Undo puts the previous value back (one level). Every change to a target (taught, taught
  again, touched up, undone, deleted) is added to `cell/logs/targets.jsonl` with its old and new values. A touch-up is
  refused while a program moves the arm, and allowed while one is paused.
- **Alternatives:** teaching it again, as before (nothing showed what changed, and the old value was gone); showing the
  change and asking before saving (a second press every time, where Undo costs only a mistake's); undoing to any
  earlier value (the log has them all; one level covers the slip).
- **Why:** the build line: re-teach in place, show the change, keep the old value in the log; how it shows the change
  was ours to decide. A paused run is where touch-up is wanted: stopped at a target that's a little off.

## D117. The page: tending's controls go in the Program and Cell sections, within DESIGN.md

- **Decision:** built with the design skill within DESIGN.md's system, as the other sections were (D25). The Program
  section gains the Calls list, a Safe home row (Go home, with its target and distance), and the new steps in its
  composer; a retry's and a fallback's steps are edited in place, and an empty one's row, or If it fails, is selected
  to add steps into it. The Cell section gains Grippers (Open, Close, Grasp, with what their inputs read as a cue),
  Patterns (Plan, Reset, Edit, with the counter's slot), Interlocks (each with its lamp), a zone's Open while, and a
  target's Touch up and Undo. The 3D view draws a guarded zone with dashed edges, without its fill while it's open,
  patterns' slots as small grey dots, and a touched-up target's old place as a ring. A gripper has no lamp: colour on
  the desk reports state (the Backlight Rule), and an output set isn't a gripper confirmed. The page reads the I/O once
  a second while the cell has interlocks or grippers.
- **Alternatives:** a section of its own for tending (the steps belong to programs, and grippers, patterns and
  interlocks to the cell, where the rest of their kind are).
- **How it was checked:** screenshots at 1440 and 420 px against the simulator, and, driven by a browser, a retry and
  a fallback composed (If it fails included), a gripper opened, an interlock's lamp lit as its input was set, a
  touch-up and its Undo, and a pattern's Plan, with nothing in the console but the expected refusals while the rig
  restarted. The design detector found nothing new but one value DESIGN.md already uses (#8f8b83, the scale's ink).
  The design skill's question round was skipped: the brief left these decisions to this work, as D25's did.

## D118. The simulator plays a gripper on its control port, and can stop when the payload is written

- **Decision:** `{"cmd": "gripper", "output": "tool.N", "close": 1, "seconds": s, "closed"?, "opened"?, "part"?,
  "jammed"?}` puts a gripper on an output: once the output has stayed at its closing value for its time, its closed
  input reads 1, and its part input too while a part is between its jaws (`{"cmd": "part", "present": ...}`); at the
  other value, its opened input; a jammed one never gets there; `"remove": true` takes it away. `--payload-stops` makes
  writing the payload stop the arm, as writing the teach sensitivity does.
- **Alternatives:** a gripper faked inside the server (the tests would check the server against itself); one in the
  twin (later, once the real gripper's drive is known).
- **Why:** the tests drive grippers through the same SDK reads and writes the arm will see, as machines' signals are
  already set through the control port.
- **Revised after review:** `--payload-error CODE` also raises that controller error as the payload is written, as a
  collision tripping during a grasp would, for D106's revised Enable.

## D119. Old cells, programs and the API keep working

- **Decision:** the cell's new collections (`grippers`, `interlocks`, `patterns`) read as empty when a cell has none,
  and the cell's version stays 1; a zone without interlocks is a keep-out zone as before. Every step, endpoint and
  field keeps its meaning: new endpoints and fields are added (in the status, `held`, `counters` and `calls`;
  `/api/plan` and `/api/moveto` also take a `pattern`), and none is changed or taken away. A failure nothing catches
  faults the run, as before. The previous server, given a cell from this one, takes a guarded zone for a keep-out
  zone, and refuses a program with the new steps rather than run it.
- **Alternatives:** a version 2 cell (the previous server would refuse the whole cell, where only its new parts are
  new).
- **Why:** Foreman drives this API, and the cells and programs there are must load unchanged. Tested: an old cell and
  program load and run unchanged and the API keeps its fields (`tending`'s compatibility scenario), and the other
  suites pass as before.

## D120. The gap to the cart, the floor and the zones is signed: inside one, it's minus the depth

- **Decision:** a capsule's gap to a box or a cylinder (the cart, the floor, the cell's zones) is the signed distance
  between them: how far apart they are outside, and, when the capsule's core reaches inside, minus the depth of its
  deepest point (its distance to the nearest point of the surface), less the capsule's radius as before. The check's
  rule for parts already within the margin, that they may move apart or 1 mm closer at most, so now means, for a part
  already inside, no more than 1 mm further in. The lower bound the check skips pairs with uses the same signed
  distance (it's 1-Lipschitz, so still a bound). `scripts/utra_model.py` has it (`Obstacle.segment_gap`), the page's
  `scripts/web/collide.js` line for line, and so the brain's shell and the stub brain, which check through the same
  model. A hit that takes a part further into what it's in says so ("the tool would go further into the machine zone").
- **Alternatives:** the unsigned gap, as before (inside, it was minus the capsule's radius however deep, so a move
  further in was never seen: from inside a zone, the arm could be driven on through it); measuring how much of the
  capsule is inside (costlier, and no better for the case below).
- **Why:** the review found that a move further into a zone the arm was already in went unchecked, and, the zone being
  guarded, unwatched. Checked: the signed gap against an independent NumPy signed distance on 400 random segments per
  shape, 263 of them reaching inside, never above the densely sampled minimum by more than 1e-9 m
  (`tests/collide_tests.py`); `collide.js` against `utra_model.py` on 240 random paths, 39 of them starting with part
  of the arm inside a zone and 33 of those refused for going further in, to 1e-15 m (`tests/js_tests.py`); paths from
  inside a zone against dense sampling, none let through further in (`tests/path_tests.py`). `check_path` is no
  slower: 40 paths took 2.9 s against 3.7 s before, as the distance outside is now worked out from the shape's own
  coordinates. Known limit: depth is measured to the nearest face, and a box can't tell a machine's door from its
  walls. A tool hanging into a zone from above is deepest by its distance to the top face, so it can move further into
  the machine sideways without going deeper by this measure. D121 covers that for guarded zones.

## D121. A move that starts in a guarded zone needs its interlocks; while they don't hold, nothing moves the arm there

- **Decision:** a guarded zone the arm is in at a move's start (any part of it overlapping the zone: `Panel.zones_in`)
  counts as entered by the move, whatever the move does there. Its interlocks are read; if they all hold, the zone is
  open for the move and the move is watched (D111), even one leaving it; a dry run assumes they'll hold. If they
  don't, nothing moves the arm there, whatever asks: a program's steps, moves to targets and slots, safe home, the
  brain's paths and bursts (the brain isn't asked), and the page's jogs and Move are refused (`planner.ZoneClosed`, a
  failure as D112 has it), saying so. A run that an interlock paused, resumed while it still doesn't hold, plans the
  rest again, is refused, and stays paused, saying so, with nothing sent. The arm stays put until the interlocks hold
  again (the door opened), or a person guides it out by hand, in freedrive, which the server doesn't steer. The page's
  own check (`arm_view.js`) previews any plan from inside a closed zone as blocked.
- **Revised again, after the fixes were reviewed:** at first a jog or Move from the page could back the arm out,
  checked with the zone closed, so never further in (D120), and ending outside it. But "further in" is the depth to
  the nearest face, so a move at a constant depth passed: a tool hanging into the zone from above could slide sideways
  through it and out of a side, or a curved Move leave the zone and come back in as deep as it began (a part's
  allowance only follows it out once it's 10 mm clear, D124). The review suggested requiring a steady retreat instead,
  back along the tool's axis, or with no point inside moving towards the box's middle on any axis. Neither tells which
  way is out: in the tests' machine the tool went in sideways, through its -y face, pointing down, so retreating along
  the tool would take it up through the zone's top and refuse the way it came; and from the middle of a box, every
  direction is away from the middle, out through the far face included. And with the door's interlock not holding,
  the door may be shut on the arm, where backing out the way it came drags it through the door. So the page is
  refused too.
- **Alternatives:** letting any move back out that never goes further in (the first version, above); a retreat rule
  (above); a zone's door face, taught, so that "out" is known (more to teach, and a door that's shut is still in the
  way); retracing the path the arm went in by (the server knows it only for a run's last move).
- **Why:** the door shutting on the arm is the machine saying it's not safe in there, and only a person can see which
  way is out. Tested against the simulator, with a tool hanging 32 mm into the machine (its depth pinned by the zone's
  top face) for each case the reviews listed: Resume with the door still shut (refused, nothing sent), a later step
  further in (fails, nothing sent), a Move and a tool jog further in, a Move straight back out the way it came, and a
  tool jog sliding the tool out of the machine's side at its depth (all refused, nothing sent), a learned skill working
  in the machine (stopped 0.09 s after the door shut, no more bursts), and safe home's joint move from inside (stopped
  0.12 s after the door shut; refused with it shut); then freedrive, the arm guided out by hand, and the page jogging
  it again with the door still shut. The checks of the review's cases fail on the code before each revision.

## D122. Watching a move's interlocks never holds up the other reasons to stop

- **Decision:** the page's watchdog acts on the hold-to-move key let go, a quiet page, stale status and a controller
  error first, and reads the interlocks last. It, and a run's supervision, waits at most 0.05 s for the SDK
  (`WATCH_LOCK_WAIT_S`), which a request may hold for seconds while the status stream stalls; if the interlocks can't
  be read within 0.5 s of the last time they were (`GUARD_CONFIRM_S`), the arm stops, as when one doesn't hold.
  (`Panel.watch_guards`.)
- **Alternatives:** reading them first and waiting as long as it takes (as before: letting go of hold-to-move, a quiet
  page or a stale status could wait seconds behind a request); a thread of its own reading them (more to coordinate,
  for the same bound); stopping whenever the SDK is busy (a jog's planning would stop moves into zones).
- **Why:** the review found the release of hold-to-move waiting behind the SDK's lock. Measured in-process against the
  simulator, with a test thread holding the SDK as a stalled request would: letting go stopped a Move into a guarded
  zone 0.015 s later; with the key held, the Move stopped 0.44 s after the SDK was taken, its interlocks unread.
  Before, neither stopped until the SDK was free.

## D123. A tool jog that can't be checked isn't sent

- **Decision:** a tool jog is planned from the joints its start is solved at, from where the arm is. If that can't be
  solved within the planning limits (a joint jogged past them, J6 beyond 180° say), the jog is refused while the arm
  is still, saying which joint is where, and held ("Holding", nothing sent) while the arm is still moving. Before, it
  was sent unchecked.
- **Alternatives:** refusing only near a guarded zone (an unchecked step can hit the cart, a keep-out zone or the
  arm itself as well); widening the planning limits (they keep plans within the faders' range).
- **Why:** the review found that after J6 was jogged past 180°, every tool jog went unchecked, into a closed machine
  included. Tested: with J6 at 191°, a tool jog into the machine is refused and nothing sent.

## D124. A move checked in parts has one allowance for parts already close; once they're clear, the margin holds again

- **Decision:** a part already within the margin of something where a move begins may come at most 1 mm closer (or
  further in) over the whole move. A straight line is checked sample by sample (every 2 mm or 1°), and a path from the
  brain segment by segment: each part is now checked with the allowance worked out once, at the move's first pose
  (`ArmModel.start_limits`), passed to every part (`check_path`'s `limits`), in the planner's lines, the shell's
  paths, `zones_entered` and the stub brain alike. And once a pair is seen back beyond the margin during a move, the
  margin holds for it again, so a move can't back out of a tight spot and come back closer than the margin.
- **Alternatives:** each part working out its own allowance, as before (a line from 6 mm off anything could close in
  1 mm a sample: a 50 mm tool jog at 15° to a closed zone's face ended 6.9 mm inside it, without the zone being seen,
  so without its interlocks being read, and a brain's path the same, 1 mm a segment); one allowance for the whole move
  without the margin coming back (a line could leave a tight spot and come back down to its start's gap, which its
  samples used to catch).
- **Why:** found by a second review of the fixes for D120 and D121, as the rule that a move from the page backing out of
  a closed zone must end outside it (since withdrawn: D121) left the arm within the margin of the zone, where this
  creep began; but it holds for any move begun close to anything. Tested in-process: the 50 mm line and a brain's path
  closing in 0.9 mm a segment from 6 mm off the machine zone are both refused once 1 mm closer (both were let through
  before), a path backing out to 18 mm and coming back to 9 mm is held to the margin, and `collide.js`, which has the
  same rule for a whole move, still agrees with `utra_model.py`.
