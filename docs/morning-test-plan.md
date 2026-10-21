# Morning test plan: the featureset on the real arm

For the owner, at the arm, at reduced speed. It confirms what was built overnight on the simulator, and settles what
the 2026-10-21 capture left open: the units of the joints' "tau" readings, how freedrive feels at each teach
sensitivity, and what each collision sensitivity level does. Run the tests in order; each assumes the ones before
it passed. About 2 to 2.5 hours.

Every result goes in the sheet at the end and in `data/captures/2026-10-21-tests/` (the commands below write there).

**Run on 2026-10-21, 11:00 to 15:00,** with the owner at the arm: MT-1, 2, 3, 7 and 9 passed, after fixes the arm
made necessary (decisions D32 to D39); MT-8 is next; MT-4, 5, 6 and 10 wait for their equipment. The sheet at the
end has the numbers. From 13:15 the controller was cabled to the home network (192.168.1.43), which the plan below
now recommends (D38).
Anything that surprises you: press STOP (or the hardware E-stop), write down what happened, and skip to the next
test that doesn't depend on it.

## Before you start

**Bring:** a 1.000 kg mass you can hang from the flange (a bag of rice or a water bottle, weighed on a kitchen
scale, with a short loop of string); a spring or luggage scale reading 0 to 50 N (or 0 to 5 kg); a block of soft foam
about 10 cm across; a pointer for the flange (a long bolt or a pen held firmly, sticking out about 100 mm); a fixed
spike or cone on the table (a nail through a block of wood); a flat plate or block with square edges to use as a
fixture; a steel rule; tape.

**Safety, throughout:**
- The hardware E-stop within reach of your free hand, the cart's casters locked, the area within 1.2 m of the
  arm clear.
- Speeds: joint moves at 10 °/s (the joint speed fader's default), tool jogs at 20 mm/s, programs at their
  default speeds (50 mm/s lines, 20 °/s joint moves) with the Override at 20% for the first run of anything new,
  50% after.
- Stand clear of the arm while it runs a program; your hands near it only in freedrive or with the motors off.
- The jog panel and the tools below share the controller's single command connection: `tau_check.py` only
  listens and runs alongside the panel; `sensitivity.py` and `settings_probe.py` need the panel closed (Ctrl+C in
  its window).

**Start:**
1. Power the controller on. Best, cable it to your router and use its address there (192.168.1.43 on 2026-10-21;
   D38); otherwise join `NTRO_43` and use 192.168.2.1. Then `python scripts/arm.py status <address>` (read only).
   Start `python tools/capture/record.py data/captures/<folder> --ip <address>` in its own window: it only
   listens, and the analysis below reads its recording.
2. Make the day's folder: `mkdir data\captures\2026-10-21-tests`.
3. `python tools/capture/settings_probe.py --dir data/captures/2026-10-21-tests` (read only, panel not running yet).
4. `python scripts/jog_server.py 192.168.2.1` and open http://127.0.0.1:8765. Its tools, frames, targets,
   programs and run logs go in `cell/` (git ignores it).

## MT-1. Still, listening only: the status and config streams

**Purpose:** confirm the 10 Hz stream and the resting noise as captured, and whether the controller kept its
sensitivities (0 and 0) and payload (zero) through the power cycle.

**Steps:** with the motors still off, then again after holding Enable:
`python tools/capture/tau_check.py record data/captures/2026-10-21-tests/rest-off.json --seconds 10`, and
`... rest-on.json --seconds 10`.

**Expected:** 95 to 105 frames in 10 s (the capture: 10.01 Hz); joint spread under 0.005°; at the capture's start
pose (36.1, 28.5, 138.8, 108.4, 45.8, 64.6), tau near J2 5.5 and J3 -25.9 (sd 0.08 and 0.2), elsewhere whatever it
is now; the config line: collision sensitivity 0, teach sensitivity 0, payload [0, 0, 0, 0], tool offset zero.

**Pass:** the frame count and spread are within those ranges. **Record:** tau and the settings in the sheet; if
the sensitivities aren't 0, they persisted from an earlier session.

**Exercises:** `tools/capture/tau_check.py` (record), the simulator's 10 Hz stream and rest tau (`tools/utra_sim.py`
`REST_TAU`), docs/capabilities.md section 1.

## MT-2. Enable and STOP, against the simulator's model

**Purpose:** confirm the two behaviours the simulator was fitted to, at today's reduced speed.

**Steps:**
1. Note the tool rows (X, Y, Z), hold Enable, and note them again.
2. Set joint step 15° and speed 10 °/s; click J1 + once and press STOP while it moves. Note J1's readout when
   STOP is pressed (the message line) and where it settles.
3. Hold Enable to carry on.

**Expected:** Enable moves the tool 0.4 to 0.6 mm (the capture: 0.46 and 0.53 mm; the simulator applies 0.456).
STOP at 10 °/s: J1 runs on about 0.07° (6.4 ms at 10 °/s, plus the brief settle the capture showed) and is still
within 0.3 s.

**Pass:** shift 0.2 to 1.0 mm; overrun under 0.3°. **Exercises:** `Panel.enable`, `Panel.stop`
(`scripts/jog_server.py`); the simulator's `ENABLE_SHIFT_DEG` and STOP model (`tools/utra_sim.py`), fitted by
`tools/capture/fit_sim.py`.

## MT-3. Programs on the server: dry run, run, pause, step, stop, faults

**Purpose:** confirm the runner, supervision and planning on the arm. Nothing is mounted on the flange; the tool in
use is the bare flange.

**Steps:**
1. Teach by jogging: in Teach, record **home** as Joints (In: base). Jog the tool to a clear spot about 300 mm
   above the table, record **a** (Tool pose, In: base). With the tool step at 50 mm, click +Y twice and record
   **b** (100 mm from a). Set the turn step to 15°, click Yaw + twice and record **b_turned** (b turned 30° about
   z, same point). Check each in the Cell section's Targets list.
2. In Program, create **check**, and add in order (Add at the end): Joint move to home; Straight move to a;
   Straight move to b, speed 20; Straight move to b_turned; Straight move to a, Back 30; Joint move to home.
3. **Dry run.** Expected: DRY RUN, then PLANNED with an estimate; the arm doesn't move (joints within 0.01°).
4. Override 20%. **Step** through all six steps, pressing Step for each; watch the arm before each press. The
   b to b_turned step only turns the tool: hand on STOP, since how this controller paces a turn on a straight
   move is unknown (the planner assumes 1 rad takes as long as 100 mm).
5. Override 50%, **Run**. During the a to b line, **Pause**; then **Resume**.
6. Run again, and **Stop run** during a line.
7. Add a keep-out zone across the a to b line (Cell, Keep-out zones: in base, centred halfway between a and b,
   whose x, y and z the Targets list shows; size 40 x 40 x 200 mm), then **Run**.
8. Delete the zone.

**Expected:**
- Step 4: each step stops where it should, with the hollow amber lamp on the step that's done. The run log's
  "done in" time for each straight line is within 15% of its "about ... s" (20% speed). Note the b to b_turned
  step's time; that settles the turn's pacing.
- Step 5: PAUSING, then PAUSED, with the arm stopped (the bridge says STOPPED; Resume makes it ready again
  itself). On Resume the motors re-enable (a 0.5 mm shift). The log shows "back to the path from about 0.5 mm
  off", then the line carries on to b. The run ends DONE at home.
- Step 6: STOPPED within a second; the arm halts within about 1 mm.
- Step 7: FAULTED before anything moves, naming the zone; the 3D view shows the red shadow arm, the contact mark
  and the zone in red. Reset clears it.

**Pass:** all of the above, and no unexpected fault. Paths stay within the executor's 2 mm line tolerance (a
deviation would fault the run). **Record:** the dry-run estimate, the real time, each line's time against its
estimate, and the turn step's time.

**Exercises:** `scripts/runner.py` (start, step, pause, resume, stop, reset, override, dry run, event log),
`scripts/executor.py` (sending and supervising moves, regaining the line after a pause), `scripts/planner.py`
(lines, offsets along the tool, zones, duration estimates), `scripts/web/featureset.js`. Simulator tests:
`tests/feature_tests.py` (scenario_runner, scenario_controls, scenario_faults).

## MT-4. What "tau" measures

**Purpose:** find tau's units by loading the arm with a known mass and comparing the change in tau with the torque
each joint must add (the arm's own masses aren't modelled, so this can't be done offline).

**Steps:**
1. Bring the arm to the capture's start pose: in Teach, record it as a joint target **tau_pose** if you're there,
   or plan it by dragging (36.1, 28.5, 138.8, 108.4, 45.8, 64.6). The tool in use must be the bare flange (payload
   zero).
2. Nothing on the flange: `python tools/capture/tau_check.py record data/captures/2026-10-21-tests/tau-empty.json`.
3. Hang the 1.000 kg mass from the flange's centre on a short string, let it stop swinging, and measure from the
   flange face to where the string is tied (L mm, along the tool's z): `... record ...tau-1kg.json`.
4. Take it off: `... record ...tau-empty2.json`.
5. `python tools/capture/tau_check.py compare data/captures/2026-10-21-tests/tau-empty.json data/captures/2026-10-21-tests/tau-1kg.json --mass 1.0 --at 0,0,L`
6. Optional, a second check: hook the spring scale to the flange and pull horizontally along the base's +X with
   20 N (hold it steady), recording `tau-pull.json`; compare with `--force 20,0,0`.

**Expected:** at this pose 1 kg needs J2 +2.37, J3 -4.53 and J4 +0.56 N m (the tool's numbers, checked against
Pinocchio). The ratios of tau change to those torques agree within 15% across J2, J3 and J4, giving tau's scale
(units per N m at the joint) and sign. Empty and empty2 agree within twice the noise.

**Pass:** a consistent ratio (the tool says so). If J2 (120:1 gearbox) disagrees with J3 and J4 (100:1), tau is
measured at the motor side; note both ratios. **Record:** the compare output, in the sheet and the folder.

**Exercises:** `tools/capture/tau_check.py` (`holding_torques`, `compare`), checked against Pinocchio's generalized
gravity in `tests/oracle_tests.py`. Unblocks capability S4 (docs/capabilities.md).

## MT-5. A tool point by touching a spike (4-point)

**Purpose:** confirm tool-point calibration on the arm.

**Steps:**
1. Mount the pointer firmly. Measure its tip from the flange face with the rule (L mm).
2. In Cell, Tools: name it **pointer**, mass about 0 (or its mass), and Save tool. Don't Use it yet.
3. Jog the tip onto the spike's point (tool step 1 mm, then 0.1 mm; speed 20 mm/s) and press Touch. Repeat from
   at least 4 more orientations, turning the wrist 20° or more each time (Roll, Pitch and Yaw keys, then
   re-centre with X, Y, Z).
4. Fit. Then Use it.
5. Repeat steps 3 and 4 once (Clear first).

**Expected:** the fit puts the point about (0, 0, L) mm with an RMS under 0.5 mm; the two fits agree within 0.5 mm.

**Pass:** |z − L| < 2 mm (the rule's accuracy), x and y under 2 mm, RMS < 0.5 mm, repeat within 0.5 mm.
**Exercises:** `cell.tool_from_flange_poses`, `/api/cell/tool/touch` and `/calibrate`; the simulator test
recovers a known point within 0.05 mm (`tests/feature_tests.py` scenario_calibration), and
`tests/feature_oracle_tests.py` checks the fit against SciPy with noise.

## MT-6. A fixture frame by touching three points, and targets that move with it

**Purpose:** confirm frames, targets stored in them, and re-mapping on recalibration.

**Steps:**
1. Tape the plate to the table. In Cell, Frames: name **plate**, In base. Jog the pointer's tip to the plate's
   near-left corner, press Origin; to a point 100 mm or more along its front edge, press +X; to a point on its
   surface towards the back, press XY. Fit.
2. Jog the tip 20 mm above a mark on the plate and record **over_mark** (Tool pose, In: plate).
3. Move away, then Plan on over_mark, Move. Note where the tip ends up against the mark.
4. Slide the plate exactly 50 mm along its edge (use the rule against a taped stop) and turn it about 10°.
   Re-touch plate (all three points) and Fit. The message says over_mark moved with it.
5. Plan on over_mark again, Move.

**Expected:** step 3 puts the tip 20 mm above the mark within 1 mm sideways; step 4 reports over_mark moved about
50 mm (and turned about 10°); step 5 puts the tip over the mark again within 1 mm.

**Pass:** both landings within 1 mm sideways and 1 mm in height. **Exercises:** `cell.frame_from_points`,
`Cell.calibrate_frame`, `Cell._remap`, `stageJoints` and Move (`scripts/jog_panel.html`); simulator and oracle
tests: scenario_calibration (frame within 0.05 mm, targets re-mapped) and `tests/feature_oracle_tests.py`.

## MT-7. Freedrive at each teach sensitivity

**Purpose:** confirm freedrive on the arm, and how it feels at each sensitivity the panel offers: 90, 100 and 110 %
(Umbratek's current SDK; D35).

**Done on 2026-10-21** with `tools/capture/teach_probe.py` (one controller write per run, the panel closed), after
both of the server's first attempts failed: see D34 and the sheet. The probe established the order the server now
uses, and the feel at 0, 90, 100 and 110. What's left is to repeat it from the panel with the fixed server.

**Steps,** for 90, 100 and 110, one hand near the forearm, the other on the E-stop:
1. Hold Enable (the arm READY), set Sensitivity, then hold **Hold for freedrive** until the amber bar fills. The
   server writes the sensitivity (the controller stops the arm for that), makes it ready again with Enable's
   checks, then selects freedrive. The key reads "Freedrive on" and the top window FREEDRIVE.
2. Hands off, close to the arm, for 10 s.
3. Guide the flange about 100 mm, raise the elbow, and let go. Optionally, pull with the spring scale to find the
   force that starts it moving.
4. Press the key to leave: the motors switch off (J3 settles about 0.45° onto its brake), then on again with
   Enable's checks. The arm stays where it is.
5. Once, in freedrive, press STOP: the arm goes rigid, then its motors switch off. Hold Enable to carry on.

**Don't** turn a joint by hand past about ±350°: the controller faults (error 23) at its limit (wrist 1 did at 358°),
and getting it back takes the procedure in D39.

**Expected (as found with the probe):** freedrive on within 0.2 s of the mode write; hands off, the tool within
0.1 mm over 10 s; 90 stops where released; 100 and 110 lighter at the shoulder and elbow, the elbow sinking back a
few degrees after a raise.

**Pass:** each level enters and leaves cleanly from the panel, and at least one guides comfortably and stops where
released. **Exercises:** `Panel.freedrive` and `_leave_freedrive` (`scripts/jog_server.py`), `in_freedrive`, Enable's
mode check, the 10-minute limit, the freedrive key and Sensitivity fader (`scripts/web/featureset.js`); simulator
tests: sim_tests live_registers (the controller's behaviour) and feature_tests scenario_teach (the server's).

## MT-8. Teaching a program by freedrive, then running it

**Purpose:** the whole teach-then-run loop on the arm.

**Steps:**
1. Create the program **taught**, then select nothing (new steps go at the end).
2. In Teach, set Then add to Straight move, In: plate. Freedrive at 90 %. Guide the pointer over three marks on
   the plate (20 mm above each) and press Record here at each (p1, p2, p3); leave freedrive (the motors switch off
   and on again; the recorded targets are from before that).
3. Dry run, then Run at 20%, then at 50%.

**Expected:** three targets in plate and three straight moves in **taught**; the runs pass over each mark within
1 mm, matching where you taught them.

**Pass:** within 1 mm; no faults. **Exercises:** `teach_target` (`scripts/jog_server.py`), the record row and mode
keys, the runner; simulator test: scenario_teach (recorded target equals Pinocchio's within 0.001 mm).

## MT-9. A run with the page closed

**Purpose:** confirm a program runs with its own supervision, without the page.

**Steps:** Run **check** at 50%, close the browser tab during the first line, wait for the arm to finish, then
reopen the page.

**Expected:** the run carries on to DONE; the reopened page shows DONE and the whole log. (A Plan-and-Move from the
page, by contrast, stops if the page goes quiet.)

**Pass:** DONE, with every step in the log; the run's file is in `cell/logs/<date>/`.
**Exercises:** `Runner` (its own thread), `Panel._watch` (skips runner moves); simulator test: scenario_page.

## MT-10. Collision sensitivity levels

**Purpose:** find what the controller's collision detection does at each level. It's off (0) now. The panel doesn't
rely on it (it plans clear of the cell instead), but it's the backstop for things it doesn't know about.

**Steps:**
1. Quit the panel. `python tools/capture/sensitivity.py --collision 1`, then restart the panel and hold Enable.
2. **Still arm:** push the flange horizontally with the spring scale, slowly up to 30 N, and let go. Note whether,
   and at what force, the controller faults (the state window shows FAULT with the error number).
3. **Moving arm:** stage J1 +20° at 10 °/s (drag the ring or the fader cap), hold the foam block where the forearm
   will meet it, and press Move. Note whether it faults, the error number (31 "tau collision limit" or 22
   "Mechanical collision limit"), and roughly how hard it pressed (the scale between the foam and your hand, if
   you can).
4. Hold Enable to recover, and repeat steps 1 to 3 for levels 2 to 5.
5. At the level that seems right, re-run **check** at 50% twice: no false trips.
6. Finally, set the level you want (or 0 if none is reliable) with sensitivity.py.

**Expected:** higher levels trip at smaller forces; the controller stops and reports a fault, and the panel shows
it and needs Enable.

**Pass:** a level that trips on gentle contact (under about 50 N) and never on normal moves; or a clear record that
none does. **Record:** each level's trip force (still and moving), the error number, and whether a normal run
false-tripped. Then pick the level and note why.

**Exercises:** `tools/capture/sensitivity.py`; the panel's fault handling (`describe`, the alarm state window), the
executor faulting a run on a controller error (`tests/feature_tests.py` scenario_faults); the simulator's contact
model (error 31 above a per-level threshold, `CONTACT_THRESHOLDS`, a guess until this test).

## MT-11 (optional). Digital I/O

Only if something is wired to the tool's or the controller's inputs and outputs. Add steps to a program: Set
tool.0 to 1, Wait 1 s, Set tool.0 to 0; Wait for input tool.0 = 1 (timeout 10 s); then Run it, close the input by
hand, and check the output with a meter or the tool's LED. **Expected:** the output follows the steps, the wait
ends when the input closes, and with nothing closed the run faults after 10 s, naming the input.
**Exercises:** `Panel.write_output`, `Panel.read_input`, the runner's I/O steps (`tests/feature_tests.py`
scenario_runner).

## MT-12 (optional). The controller's own tool offset convention

The panel keeps the controller's tool offset at zero and applies tools itself (docs/decisions.md D14), so this only
matters if that changes. With the panel closed, in Python:
`import sys; sys.path.insert(0, "scripts"); import arm; a = arm.open_arm("192.168.2.1"); a.set_tcp_offset([0, 0, 100, 0, 0, 0])`,
then `python scripts/arm.py status 192.168.2.1` in another window; the reported pose should be 100 mm out along the
flange's z. Then `a.set_tcp_offset([0, 0, 0, 0, 0, 0])`, check the status shows the flange again, and quit Python
(Ctrl+C if it doesn't exit). **Exercises:** the simulator's
assumption (flange frame, mm and rpy), `tests/sim_tests.py` live_registers.

## Wrap-up

1. `python tools/capture/sensitivity.py` (the panel closed) and `settings_probe.py --dir data/captures/2026-10-21-tests`
   to record the settings as you leave them.
2. Fill in the sheet below; save it with the folder.

## Results sheet

| Test | Result (numbers) | Pass? | Notes |
| ---- | ---------------- | ----- | ----- |
| MT-1 frames in 10 s, spread, tau at rest, sensitivities, payload | 100 frames / 10 s on the cable (Wi-Fi: 1 to 3 s stalls about every 100 s); joints held within 0.005°; tau exactly 0 with the motors off; at rest J1 -5.13, J2 +0.10, J3 -23.94, J4 +2.27, J5 +0.80, J6 -0.82; collision and teach sensitivity 0, payload and offset 0 after power-on | Pass | tau is servo-side; holding values depend on how the arm got there (J2 5.5 last night at the same pose) |
| MT-2 enable shift (mm), STOP overrun (°) | enable 0.468 mm (J3 +0.057°; simulator 0.456); STOP from J1 at -10 °/s: 0.167° past (16.6 ms), stopped in the next frame, settled within 0.3 s | Pass | J3 gains about 0.055° at every "ready"; the simulator's fixed-time STOP model doesn't scale with speed (capture: 6.4 ms at 30 °/s) |
| MT-3 dry-run estimate vs real (s); line times vs estimates; turn step time; pause regain offset (mm); stop; zone fault | estimate 9.8 s; at 20%: b line 25.09 planned / 25.12 s, back to a 10.19 / 10.28, home 2.03 / 2.06, 10° turn 1.9 / 1.44; at 50%: 18.66 s; pause regain 0.511 mm, rest of line 15.33 / 15.29 s; within 0.40 mm of the line while moving; stop 39 ms; zone fault at step 3, 94 mm along the line | Pass | after D32 (arrival tolerance) and D33 (status slots); two Wi-Fi faults led to D38; the controller paces pure turns faster than the planner assumes |
| MT-4 tau ratios J2, J3, J4; scale and sign; empty vs empty2 | not run | | needs the 1 kg mass |
| MT-5 tool point (mm), RMS, repeat difference | not run | | needs the pointer and spike |
| MT-6 landing before and after moving the plate (mm) | not run | | needs the plate |
| MT-7 freedrive: drift, feel, by sensitivity | with the probe: 0, 90, 100 and 110 all entered; hands off within 0.07 mm over 18.6 s (0), 0.29 mm / 22 s and 0.49 mm / 54 s (100); guided at up to 290 mm/s with no error; STOP rigid in mode 2; leaving: motors off, J3 +0.43° | Pass (probe); panel re-test pending | 0 and 90 stop where released; 100 and 110 lighter at shoulder and elbow, which sinks back a few degrees; wrist 1 turned to 358° faulted (error 23), recovered by D39 |
| MT-8 taught passes over marks (mm) | | | |
| MT-9 page closed: DONE, log complete | DONE in 18.88 s, all 6 steps logged (133132-check.jsonl) | Pass | |
| MT-10 level 1 to 5: trip force still / moving (N), error code, false trips | not run | | needs the spring scale and foam |
| MT-11 I/O | not run | | |
| MT-12 tool offset convention | not run | | |
