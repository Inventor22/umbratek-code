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
  the move, against a lower bound that holds everywhere (the ESDF at the nearest voxel centre less the distance to it;
  beyond the grid, through the point's projection onto the grid's box, as every obstacle is inside it). The spine's
  slack rule (a move may start nearer than the margin and move away) is given only where an upper bound shows the start
  surely is that near, and measured from that upper bound, as the map's bounds are loose by up to about two voxels.
  A map more than 2 s old, or whose arm isn't where the plan starts (after waiting for a newer frame), fails the plan.
  Any source that can't answer fails the plan, saying why.
- **Alternatives:** the planner reading the twin's scene file (misses obstacles placed over HTTP, and where free ones
  have been pushed); obstacles from the request (the interface has no field for them; the spine doesn't know them);
  treating perception's unknown voxels as solid (everything outside the cameras' view would block); ignoring a
  source that fails (a plan blind to what's there).
- **Why:** before perception exists, the twin's truth tests planning against obstacles the spine doesn't know about,
  and the same code path takes perception's map. Checked with a fake perception server that serves the contract: a
  crate seen as 2 cm voxels is planned around, 58.7 mm from the true crate. A review found the first version of the
  map's check fail open in two ways (the slack measured from a loose lower bound, and points just beyond the grid read
  as clear); the suite now checks the rule against exact distances to the voxels on random moves over a slab (69 of
  120 accepted, none nearer than the rule allows; the first version let through moves from 6.0 to 3.1 mm).

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

## D68. How the planner is validated: random moves, the twin end to end, a control, and the boundary by hand

- **Decision:** in-process, 48 random moves above a table whose straight way a crate, pole, ball or tube blocks, each
  path put through the spine's own shell (`safety.check_path`) and its clearance sampled every 0.5 deg; then the twin,
  the planner's brain and the jog server as separate processes running taught, planned, taught programs out and back
  around the twin's obstacles (moved, and changed for a pole, a ball, a tube), the twin's contacts sampled every
  20 ms; then the same program with a brain blind to the obstacles (`--world request`). By hand, the spine and the
  simulator on Windows and the planner in WSL2 on `127.0.0.1:8790`.
- **Why:** the requirement is that the spine accepts the plans and the twin reports no contact; the control shows
  the obstacle was in the way (the spine accepts the blind brain's straight path, and the twin sees the arm hit the
  crate), so it's the planner that avoids it. The results are in `brain/README.md`.

## D69. Known limits

- The real arm hasn't run a planned path from this brain; that's for a session with the owner present.
- Perception's map is tested with a fake server built from the contract, not perception's own yet. Map plans take
  longer (1.6 to 2.3 s), most of it in the map's check, which samples in NumPy. Its bounds are loose by up to about two
  voxels, so a move that starts within that of a mapped obstacle is refused unless it keeps the margin by the lower
  bound: safe, but it may find no path where the true distances would allow one.
- cuRobo's covering of the base grows it by up to 20 mm at mid-height, so a move with the wrist near the base may need
  the stub's search, or find nothing where the spine's check alone would pass.
- A path is cut along cuRobo's route; it isn't the fewest segments possible. Waypoints anywhere (not only on the
  route) could cut more, at the cost of more checks.
- The twin and the planner read the same `/state`; a free obstacle pushed after the plan is made isn't re-planned
  around (the spine's freshness checks are about the arm and the cell, not obstacles).