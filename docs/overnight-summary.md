# Overnight summary: the machine-tending featureset, 2026-10-21

Built on branch `inventor22/machine-tending-featureset`, against the simulator only: nothing connected to the
controller (192.168.2.1), the jog-server terminal and `main` were left alone, and the main checkout was only read from
(the capture was copied out of it).

**In short.** The simulator, the tests and the capture tools are in the repo, with one runner (`python
tests/run_all.py`, 11 suites, all passing). The simulator now behaves like the capture: 10 Hz state, the command
delay, the enable shift and the measured STOP. `docs/capabilities.md` specifies the featureset from four tending jobs
and puts it in build order. Its foundations are built on the server, in the simulator and in the panel: tool and work
frames with calibration and re-mapping, keep-out zones, straight-line and relative moves (all collision-checked), a
program model with a server-side runner (start, pause, resume, stop, step, override, dry run, faults, event log), and
teaching by freedrive. `docs/morning-test-plan.md` tests it on the arm and settles what the capture left open: tau's
units, freedrive at each teach sensitivity, and each collision sensitivity level.

## On the arm, the afternoon of 2026-10-21

With you at the arm, 11:00 to 15:30, the controller cabled to the home router from 13:15. Results are in the test
plan's sheet; the data, with a timeline, in `data/captures/2026-10-21-tests/`.
- **Passed:** MT-1 (the streams at rest; tau is exactly 0 with the motors off, so it's a servo-side quantity), MT-2
  (enable shift 0.468 mm against the simulator's 0.456; STOP 0.167° past at 10 °/s), MT-3 (programs: dry run, step,
  run, pause and resume, stop, a keep-out zone fault; line times within 1% of the plan), MT-7 (freedrive), MT-8
  (three points taught by freedrive, replayed at 20% and 50% within 0.26 mm) and MT-9 (a run with the page closed).
- **The arm found four bugs, each fixed with a simulator test that reproduces it:**
  - Line arrival was checked at 0.2 mm, but real lines rest up to 0.47 mm out (D32; ee09263).
  - Enable opened extra status connections and ran out of the controller's five (D33; ee09263).
  - Freedrive was entered in the wrong order. The controller stops the arm when the sensitivity is written and
    ignores mode 2 while stopped (D34 to D37; cb22aab).
  - The recorder missed a power cut, because a dead controller never closes the connection (cb22aab).
- **Learned:**
  - Leaving freedrive switches the motors off, and J3 settles about 0.45° onto its brake.
  - Teach sensitivity is 90 to 110 % in Umbratek's current SDK, and 90 holds where released (D35).
  - Joints are numbered from 1 for enable.
  - The controller paces a pure turn faster than the planner assumes (a 10° turn took 1.44 s against 1.9).
  - STOP's overrun isn't a fixed time (16.6 ms at 10 °/s against 6.4 at 30).
  - Wi-Fi drops the stream for 3 s every 33 s (D38).
- **An incident:** wrist 1 was turned by hand to its limit (358°) and faulted. Recovering it took releasing that one
  brake and turning the joint by hand, now documented (D39, `docs/troubleshooting.md`). It went the wrong way first,
  by over a turn, to 762°. No damage showed (J5 and J6 kept answering), but have the wrist's cabling checked if
  anything misbehaves.
- **Open:** whether the status stream reports measured or commanded positions while moving. At 50% the tool sat on
  the line to 0.0001 mm, too perfect to be measured. Until it's settled, path straightness from the stream isn't
  evidence of the physical path. Arrival at rest is, since the brake settle and enable shift show in it.

## What was built

| Commit | What |
| ------ | ---- |
| 1137f86, cb4882f | The simulator (`tools/utra_sim.py`), the suites (`tests/`, one runner `run_all.py`), the capture tools (`tools/capture/`) and the 2026-10-21 capture (`data/captures/`, byte for byte), with relative paths. The capture became regression fixtures (`tests/capture_tests.py`). |
| feee832 | The simulator fitted to the capture: a jerk-limited profile, start delays, the queue count, STOP's 6.39 ms overrun and settle, the enable shift, port 30002 refused, the config stream, registers, GPIO, freedrive, collision detection, faults, and a control port for tests (D6 to D12). |
| cd6f3f1 | `docs/capabilities.md`: the controller's facts; the four jobs broken into steps; the capabilities they share; RAPID, UR, PLCopen and behaviour trees adopted or rejected; each capability with what the controller provides, what we build, acceptance and dependencies; what 10 Hz rules out; build order. |
| 5fa78a4 | K1: server-side kinematics (`scripts/kinematics.py`), checked against Pinocchio and SciPy. |
| da948bc | On the server: the cell (`cell.py`: tools, frames, targets, zones, calibration, re-mapping), planning (`planner.py`: joint and straight moves, offsets, zones, singularities, durations), supervised execution (`executor.py`), the program model and runner (`runner.py`), freedrive and teaching (`jog_server.py`), and their API. |
| 19efb0b | In the panel: the Program, Teach and Cell sections (`scripts/web/featureset.js`), designed to DESIGN.md and reviewed; faulted steps drawn in the 3D view; the morning's tools (`tau_check.py`, `sensitivity.py`); a simulator bug fixed. |
| f095c64 | `docs/morning-test-plan.md`. |
| f950aa6 | Tests for the program model and the runner's states; the spec's acceptance text corrected to what the tests check; decisions D20 to D31. |

## How each claim was verified

Every check below is in the suites (`python tests/run_all.py`); the judge is outside the code under test.

| Claim | Judge | Result |
| ----- | ----- | ------ |
| Our forward kinematics is the controller's | the capture's 2292 frames (1146 moving) | within 0.001 mm and 0.001 deg |
| ... and Pinocchio's, from the same URDF | Pinocchio | 1000 joint sets within 1e-9 m and 1e-9 rad |
| Inverse kinematics lands where asked | Pinocchio | 99% or more of 366 reachable poses from a seed within 6 deg, within 0.01 mm and 0.006 deg |
| Roll, pitch, yaw, rotation logs, slerp | SciPy | within 1e-9 |
| The double-S profile | exact integration of its jerk | within 1e-9 of the distance; limits never exceeded |
| The simulator moves like the arm | replaying the capture's commands | 43 joint moves: median error under 0.05 deg (0.034 in the fit); 24 lines: median under 0.15 mm; STOP: J1 rests 0.19 to 0.21 deg on, as measured; enable shift 0.456 mm (measured 0.46 and 0.53); command to first motion median 94 ms (captured 111, within 40); 10 Hz; port 30002 refused |
| Tool points | Pinocchio frames at the same placement | 20 tools x 50 joint sets within 1e-9 m |
| Three-point frames | ground truth, SciPy's Kabsch fit | exact (1e-9); with 0.1 mm noise, origin RMS 0.172 mm against 0.173 predicted, orientation RMS 0.10 deg |
| Four-point tool fits | ground truth, SciPy least squares | exact (1e-6 mm); with 0.1 mm noise RMS 0.11 mm, equal to SciPy within 1e-6 mm |
| Calibration end to end | the simulator, touching with the pointer | frame within 0.05 mm and 0.01 deg; tool point within 0.05 mm |
| Recalibrating a frame moves its targets and zones, and only those | NumPy | within 1e-9; out-of-reach targets flagged |
| Keep-out zones and the tool's shape are never missed or over-reported | dense sampling (NumPy) | 34 random moves near random zones: none missed, none false |
| The page's collision check agrees with the server's | `xcheck.mjs` against the server, with a tool and zones | same verdicts |
| Straight lines are straight | Pinocchio at every planned sample; the simulator while running | planned within 0.05 mm (27 lines, 3 refused); executed, 152 samples within 0.1 mm, ending on target within 0.1 mm |
| Offsets (along the tool, in a frame) | SciPy composition; the simulator | within 1e-12; an approach 50 mm back ends 50.00 mm away |
| Override and the reduced-speed cap | the simulator's log of what was sent | 500 mm/s at 50% is sent at 250; at 100% a program not marked fullSpeed is still capped at 250 |
| A dry run moves nothing and estimates the run | the simulator's log; a real run | no writes; estimate 10.8 s against 12.2 s less 0.5 s of supervision (within 10%) |
| The runner's controls and states | the simulator | pause mid-line stops on the line (0.1 mm), resume re-enables, regains the line and finishes on it; step runs one step; stop; each control refused with a reason where it doesn't apply; runs carry on with the page closed; manual moves don't |
| Faults | injected in the simulator | controller error mid-move, the arm knocked 3 deg off its path, an input timeout (2.0 to 2.6 s), a zone in the way (refused before moving, with the hit drawn in the 3D view); a faulted run needs Reset |
| The program model | the server's API | malformed programs refused with the step's path; a full round trip changes only the revision; stale saves refused; a server killed mid-run leaves whole log lines |
| Teaching by freedrive | Pinocchio | the recorded target equals the forward kinematics of the hand's joints, in the frame, within 0.001 mm; the taught program runs onto it within 0.1 mm |
| tau_check's expected torques | Pinocchio's generalized gravity | within 1e-6 N m over 40 poses and loads; recovers a synthetic scale within 2% |
| The capture tools still reproduce the capture's analysis | `capture.py analyze` | summary identical to `summary.json` |
| The panel's new sections | a browser against the simulator; a separate design review | screenshots at 1600, 900 and 420 px; the review's eight material fixes applied and confirmed in its second pass; every element the module uses exists once in the page (panel suite) |

Suites at the last run (after the afternoon's fixes): cli, inproc, collide, js and paths (pass/fail), panel 194/194,
oracle 17/17, geometry 24/24, capture 19/19, sim 53/53, feature 91/91; 11 of 11. (feature's dry-run timing check
missed once, 12% against 10%, with three suites sharing the CPU; it passes on its own.)

## What is still unverified: needs the arm

Each is in the morning test plan (MT-n):
- **tau's units** (MT-4). The URDF has no masses, so offline it can't be done; `tau_check.py` is ready.
- ~~Freedrive~~: done on the arm (MT-7, D34 and D35). Still unknown: the pinned SDK's levels 1 to 5, and what
  making the arm ready while it's stopped in mode 2 does (the simulator assumes freedrive again; the server never
  does it).
- **Collision sensitivity:** what each level does, and whether it reports error 31 or 22 (MT-10). The simulator's
  thresholds are guesses.
- ~~Pause on the arm~~: done (MT-3; regain 0.511 mm).
- **How the controller paces a turn on a straight line**: measured faster than the planner's 1 rad per 100 mm (a
  10° turn in 1.44 s against 1.9); not yet modelled.
- **Calibration repeatability** with a real pointer and fixture (MT-5, MT-6). Landing on taught points: done (MT-8).
- **GPIO**: line numbering and wiring (MT-11). The simulator assumes 16 digital lines each on the tool and the
  controller.
- **The controller's tool offset convention** (MT-12). Not used (D14).
- Not tested in the plan, and not relied on: whether the enable shift depends on pose; STOP's settle for other joints
  and speeds; the queue count with more than one move pending; how the controller handles singular or unreachable
  lines (the planner refuses them before sending); the controller's own pause; queued register writes.

## Decisions

All 39 are in `docs/decisions.md`, each with its alternatives:
- **Layout and tests:** D1 to D5.
- **The simulator's model:** D6 to D12.
- **Build order:** D13.
- **The tool point on the PC, the controller's offset at zero:** D14.
- **The cell as JSON with revisions:** D15.
- **Roll, pitch and yaw in degrees:** D16.
- **From RAPID, UR, PLCopen and behaviour trees:** D17 to D19: fine points only, a UR-like step tree with PLCopen
  outcomes, behaviour-tree results for later.
- **The runner:** pause as STOP plus re-plan (D20); faults need Reset (D21); runs don't need the page, manual moves do
  (D22).
- **Moves:** tool jogs checked as lines (D23).
- **Testing:** statistical calibration checks (D24).
- **The panel:** built from the desk's own parts (D25); target keys stage a plan (D26); Space types in text fields
  (D27); targets in use can't be deleted (D28); the page edits top-level steps and repeats (D29); the morning tools
  listen or need the panel closed (D30); Override starts at 100% (D31).
- **From the arm, that afternoon:** line arrival at 1.0 mm (D32); Enable reads the server's own stream (D33);
  freedrive's order, sensitivity, exit and Enable's mode check (D34 to D37); the cable for test sessions (D38);
  recovering a joint past its limit (D39).

## Found and fixed along the way

- **The simulator** left a finished hand move short of its goal when the control port, not the status loop, noticed
  the end. A test that failed only under full load found it.
- **Two tests** read the arm's state before a pause's STOP had shown or settled; they now wait for it.
- **STOP's overrun:** a first pass took it as 7.05 ms through an arithmetic slip; a test caught it (6.39 ms, D7).
- **Earlier oracle-test flaws** were fixed: a stale Pinocchio data object, and a sampled integration that couldn't
  judge the profile.
- **`capabilities.md`** promised more than the tests checked (an OpenCV check that doesn't exist, tighter tolerances);
  it now says what is tested.

## Known issues, not fixed

- **A pre-existing layout gap.** Between 861 and about 920 px wide, the bridge's minimum widths overflow by about
  20 px. Your app side panel (about 1018 px) fits. Moving the two-row breakpoint from 860 to about 940 px would fix
  it.
- **The design detector** leaves 63 advisory notes. They are colour values DESIGN.md's shadow vocabulary and state
  tones already use, and two older font sizes; I didn't suppress them.
- **I/O isn't shown in the panel** (only at `/api/io`). A program's "full speed" flag and its default speeds are
  edited in its JSON.
- **The design review** was run by a general-purpose agent with the design skill's reviewer instructions, because the
  skill's own reviewer agent isn't available in this app.

## Questions for you

1. After MT-4, should the next step be using tau (S4, checking a grasp by the payload) or grippers (A2)?
2. ~~Which teach sensitivity should freedrive default to?~~ I chose 90 % (it held where you let go; D35). Say if
   you'd rather have 100.
3. Collision detection: leave it off (0) and rely on the planner's checks, or set the level MT-10 finds? The panel
   doesn't manage it.
4. Is a 250 mm/s cap on program lines (unless marked full speed) what you want? Should Override start at 25% each
   time the server starts, like the jog speeds?
5. Next in the page: ifs and calls, or patterns for trays (F5)?
6. What I/O will the first job use (a printer door switch, a gripper on tool.0)? Lines and names would make the I/O
   test concrete.
7. Should the cell and programs (`cell/`, git-ignored now) be committed?
8. Is a firmware update that serves port 30002 (100 Hz) possible? It lifts most of the limits in capabilities.md
   section 6.
9. Fix the 900 px bridge overflow?
10. Unattended runs need the server to survive a dropped command connection (the controller reset an idle one after
    about 12 minutes this morning, and a power cut drops it). Build reconnection and a keepalive next?
11. Should the panel warn as a joint nears its limit in freedrive (wrist 1 faulted at 358°)?
12. To settle whether the stream reports the commanded path while moving: may I use a dial indicator against a
    straightedge, or the tool tip along a ruler, in a later session?
13. The controller is on your home network with no password on its ports. Keep it there, or go back to its Wi-Fi
    between sessions?

## Trying it

- **Tests:** `python tests/run_all.py`; `tests/README.md` sets up the oracle environment.
- **The panel against the simulator:** `python tools/utra_sim.py --host 127.0.0.31`, then
  `python scripts/jog_server.py 127.0.0.31 --port 8831 --cell <a scratch folder>`, and open
  http://127.0.0.1:8831.
