# umbratek-code

Code and experiments for controlling an [Umbratek UTRA 1000 6-DOF robotic arm](https://www.umbratek.com/store/robot/utra/utra-6-1000) with Python.

| Path | What it is |
| ---- | ---------- |
| `ut_sdk_python/` | Umbratek's Python SDK (git submodule), pinned to the March 2022 version the notebook was run against. |
| `scripts/arm.py` | Command-line helper: find the arm on the network, read its status, jog a joint, home, stop. |
| `scripts/jog_server.py`, `scripts/jog_panel.html` | Jog panel: a web page on this PC with a 3D view of the arm on its cart, for moving it one exact step at a time or to a pose planned by dragging joints; and for teaching, editing and running programs, with the server running and supervising them. |
| `scripts/web/` | The jog panel's 3D view (`arm_view.js`), its collision check (`collide.js`, the same check the server makes), its inverse kinematics (`ik.js`) and its Program, Teach and Cell sections (`featureset.js`). |
| `scripts/cell.py`, `planner.py`, `executor.py`, `runner.py`, `kinematics.py`, `motion_profile.py` | The featureset on the server: the cell (tools, frames, targets, zones), straight-line and joint planning with collision checks, executing a plan under supervision, programs and their runner, and the kinematics they share. |
| `scripts/vendor/` | Not in git: [three.js](https://threejs.org) 0.186.1 (MIT), which draws the 3D view. The jog panel server downloads it the first time it runs and checks every file against SHA-256 hashes pinned in `jog_server.py`. |
| `scripts/model/utra6_1000/`, `scripts/utra_model.py` | Umbratek's URDF and meshes for the UTRA 6-1000 (BSD 2-Clause, from [UmbraTek/ut_arm_ros](https://github.com/UmbraTek/ut_arm_ros)), collision capsules fitted to them, and the kinematics and collision checks the jog panel uses. |
| `scripts/model/cart/` | The aluminium-extrusion cart the arm stands on, as a URDF in the arm's base frame, built from the SolidWorks assembly in [Inventor22/umbratek-cad](https://github.com/Inventor22/umbratek-cad) (see its `SOURCE.txt`). |
| `notebooks/0 UTRA Tests.ipynb` | SDK motion demos: joint point-to-point, linear, blended linear, arcs, servo. |
| `arduino/Chess Gripper/` | Servo calibration sketch for the chess-piece gripper (servo on pin 9, alternates 25° and 120°). |
| `tools/utra_sim.py` | A stand-in for the NTRO controller, for trying the jog panel and `arm.py` offline and for the tests. |
| `tools/capture/` | The tools that recorded the arm's status streams, settings and servo registers (with the owner present), and the morning test plan's tools: `tau_check.py` (what the joints' "tau" readings measure), `sensitivity.py` (the controller's collision and teach sensitivity), `record.py` (records the status and config streams, only listening, alongside the panel) and `teach_probe.py` (one controller write per run with read-backs, which worked out freedrive on 2026-10-21). |
| `data/captures/` | What the real arm measured: raw streams, commands, settings and the analysis, one folder per capture. |
| `tests/` | The test suites and their runner, `python tests/run_all.py` (see `tests/README.md`). |
| `docs/` | The machine-tending featureset's specification, decisions, test plans and reports. |

CAD for the mounting plate and gripper, plus the UTRA quick start guide PDF, live in [Inventor22/umbratek-cad](https://github.com/Inventor22/umbratek-cad).

After cloning, fetch the SDK:

```
git submodule update --init
```

## Connecting to the arm

The arm is driven by an **NTRO series controller**, the box with the power button and E-stop.

1. **Wire it up.** Arm cable to *Robot Connector*. External E-stop switch (or the shorting plug from the box) to *Expansion E-STOP*. The two Wi-Fi antennas go on the top-left and top-right posts; the middle post is for the optional wireless E-stop. Power cable to *Power Supply Socket*.
2. **Boot.** Press the power button once and wait about 25 seconds. The indicator breathes quickly (1 s cycle) while booting and slowly (3 s cycle) when ready. A fast 0.3 s blink means an error: check that the E-stop isn't pressed and the cables are seated.
3. **Network.** Use either:
   - **Wi-Fi:** join the controller's `NTRO_xxx` network. The default password is `12345678`. This repo's controller broadcasts `NTRO_43`, sits at `192.168.2.1` and gives the PC a `192.168.2.x` address. It offers no internet gateway, so the PC keeps its normal connection for internet traffic.
   - **Ethernet:** run a cable from the controller's RJ45 port to the PC, with the PC's adapter set to obtain an IP address automatically.
   - **Your router (best for long sessions):** cable the controller's RJ45 port to your router and use its address there (`192.168.1.43` for this repo's controller). Its Wi-Fi stalled for 1 to 3 s about every 100 s on 2026-10-21, enough to time out commands and once a STOP's reply; on the cable the status stream never missed a frame. The catch: on your home network the controller's command port and Umbratek Studio are open to every device, with no password.
4. **Umbratek Studio (optional).** Browse to the controller's address (`http://192.168.2.1` over Wi-Fi, or the address printed on the label on its interface panel) and click *Connect* for Umbratek's web UI.

### Using Umbratek Studio

Any browser on the controller's network works, including a phone or tablet joined to `NTRO_43`. Clicking *connect* only opens a session; it doesn't enable or move the arm. Then:

- **Live Control** shows the tool position and joint angles, with jog panels for joints (left) and for the tool in X/Y/Z and roll/pitch/yaw (right).
  - **Set the speed first.** The *Vel* boxes default to 100 °/s and 100 mm/s; something like 10 °/s and 20 mm/s is easier to watch and stop.
  - **A short click on −/+** moves one step: 1° per joint click, 10 mm or 5° per tool click. The gear icons change the step.
  - **Holding a −/+ button keeps the joint moving** until you let go.
  - **Dragging a slider or typing a joint value** sends the arm straight to that value at the panel speed.
  - **Zero Position** stands the arm straight up, about 1.1 m tall, and **Init Position** goes to a pose saved in Setting. Clear the arm's reach before using either.
  - **Clear Error** does the same as `scripts/arm.py reset`. **Joint Tech** is a hand-guiding mode for dragging the arm by hand.
- The red **E-STOP** button in the top bar stops motion in software and clears queued commands. The hardware E-stop is the one that cuts the arm's power.
- **Blocky** builds programs from blocks (moves, speeds, gripper, GPIO, loops). **Setting** holds motion parameters (TCP payload and offset, mount direction, speeds), GPIO, versions and servo temperatures under *Info*, and network, reboot and shutdown under *System*. Leave the calibration and upgrade buttons under *Info* alone.
- Don't move the arm from Studio and from Python at the same time.

## Jog panel

Studio's Live Control computes each click from the last pose it displayed, so quick clicks can do nothing, and holding a button sends a far-away target that only stops when the release arrives. The jog panel replaces it for manual moves:

```
python scripts/jog_server.py 192.168.2.1
```

Then open http://127.0.0.1:8765 in a browser on this PC.

The first time it starts, the server downloads three.js, which draws the 3D view: about 2 MB from jsDelivr, or from unpkg if jsDelivr can't be reached. So that first start needs internet; after that the panel works offline. It checks the files against hashes pinned in `jog_server.py` every time it starts, and serves only that checked copy, so the page still loads scripts from this PC alone. If the download fails, the panel works without the 3D view, and starting the server again with internet fetches it.

- **Hold Enable** until the green bar fills. It clears a latched error (for example after releasing the E-stop), switches the motors on and makes the arm ready, with the same checks as `scripts/arm.py jog`.
- **3D view:** the arm drawn from Umbratek's URDF and meshes, standing on its cart (from the cart's SolidWorks model) above the floor, following the measured joint angles. Drag empty space to turn the view, right-drag to pan and scroll to zoom; *Reset view* frames the arm and cart again.
- **Planning a pose:** each joint has a coloured ring in the 3D view, matching the tape on its strip. Drag a ring and an amber shadow arm shows the new pose, with the tool's path drawn in; release, and that joint joins the plan. Drag more rings to build up the pose: the rings ride on the shadow, so each drag carries on from the plan. The white cap on each strip's meter works the same way: drag it and drop it on an angle. Hold Shift to snap to 5°; press Esc during a drag to cancel it. Nothing moves yet.
- **Moving the tool instead:** press **Tool** above the 3D view (and **Joints** to go back). A gizmo appears at the flange, coloured as in other 3D tools (X red, Y green, Z blue): drag an arrow to slide the tool along X, Y or Z (the arm's base frame, as on the controller and the tool rows), in 1 mm steps, or 10 mm with Shift. Drag a ring to turn the tool about that axis while the flange stays put, in 0.5° steps, or 5° with Shift. The panel works out the joint angles as you drag (inverse kinematics, from the pose before, so the arm keeps its elbow and wrist the way they are). The shadow arm, the strips and the tool rows show the result, and releasing stages every joint it turns. If the arm can't reach the pose, or a joint would pass its limit, the drag stops at the last pose it can reach and says why. The plan, the collision check and Move work just as they do for rings, and you can mix the two to build a pose. If a staged pose leaves the gizmo near the edge of the view or under the plan bar, the view re-frames to keep it within reach (unless you've turned or zoomed it yourself). Move turns all the joints together, so the tool reaches the planned pose, but it doesn't travel in a straight line on the way; the path is drawn in the view. The panel remembers which mode you used.
- **Move:** the plan bar along the bottom of the 3D view lists the planned joints (× takes one out) and says what Move will do. **Move** sends every planned joint together as one move at the joint speed you've picked; the shadow stays until the arm settles. **Clear**, or Esc, drops the plan without moving. Plans stay within ±180° (±165° for J3), and the panel takes a new plan only once the arm has settled.
- **Collisions are checked before anything moves.** The panel checks the whole path from the arm to the plan, not just the end, against the arm itself, its cart and the floor, and won't move if any part would come within 10 mm of another. The shadow, path and readout turn red, a red mark shows where it first comes too close, the cart part it would hit is outlined in red, and the plan bar names the parts ("the forearm would come within 10 mm of a cart brace"). The server checks every plan again before sending it. The arm's links are covered by capsules fitted to Umbratek's meshes, so the check errs on the cautious side by up to a few centimetres at the motor housings. Whatever is mounted on the flange is modelled only as the shape you give the tool in use (Cell, below). If the arm already sits within 10 mm of something, it may move away but not closer.
- **Joints:** each joint has a strip. **+** and **−** move it one step. Its meter shows the actual angle (solid bar) and, while it moves, the commanded angle (hollow bar). The amber lights beside the meter count how far the command is ahead of the arm, four lights per step up to the three-step limit, and go out as the arm catches up.
- **Tool:** moves the tool in X/Y/Z or turns it in roll/pitch/yaw. With a plan staged, each row shows in grey where the plan would put the tool.
- **Step and speed:** set them on the small faders at the top of each section; each snaps to the values the arm accepts. Joint steps run 0.1–15° at 1–120 °/s; tool steps run 0.1–50 mm, turns 0.1–15°, at 1–1000 mm/s (the UTRA 6-1000's published top speed, and Umbratek Studio's limit). Each joint move is capped at the top speeds of the joints it turns: 84 °/s for J1 and J2, 114.5 °/s for J3 and 119.1 °/s for the wrists. Those are 80% of the limits the joints' own servos report (220 rad/s at the motor through 120:1 gearboxes for J1 and J2, 250 and 260 rad/s through 100:1 for the rest, so 105, 143 and 149 °/s), because the controller faulted with error 24, a speed limit, on a J3 move at 120 °/s. Umbratek's ROS model of the arm claims faster joints (up to 172 °/s) than these servos allow. Set above those caps, the joint speed fader prints which joints top out lower, and the message line says when a move was capped. Moves ramp up to speed in about 0.3 s, but never harder than the maximum acceleration configured in the controller, which the panel prints when it starts. On this arm that's 200 °/s² for the joints and 200 mm/s² for the tool, so short steps never reach the top speeds: a 5° step peaks near 30 °/s, a 15° step near 55 °/s and a 50 mm step near 100 mm/s. The top speeds show on long planned moves. Speed is a deliberate choice: the speed faders go back to 10 °/s and 20 mm/s every time the panel opens (the steps are remembered), and at the top speeds the tool can move faster than a metre a second. Lock the cart's casters.
- **Redline:** like a speedometer, each speed fader turns red past the speeds that could carry the flange faster than 250 mm/s, the limit the ISO 10218 robot safety standard sets for jogging by hand. The red zone prints where it starts ("250", or "17" on joint speed). With the cap in the red the value turns red, and the message line says how fast the flange could go and which joint would take it there. It says so again on every step in the red, and when you leave it. Red speeds still work; the redline is a warning. On tool speed it's fixed at 250 mm/s, a step of its own. On joint speed it follows the arm: it's the slowest speed at which turning one joint alone would swing the flange past 250 mm/s from where the arm is now. That's usually the shoulder, around 25 °/s with the arm folded and 14 °/s stretched out. A staged plan has a line of its own, often lower, because joints that turn together add up; the fader shows whichever is lower, and the plan bar's note turns red with the figure when the plan passes its own. The panel assumes full reach until it knows the pose. Anything mounted on the flange sticks out further, and moves faster.
- **Clicking and holding:** one click moves exactly one step from the last commanded position, so quick clicks add up. Holding a key repeats the step four times a second, and the commanded position never runs more than three steps ahead of the arm, so a hold covers about four steps a second whatever the speed: pick a bigger step to cover more ground. Joint steps get the same collision check, one step at a time. Tool steps are planned as the straight line the controller will drive and checked the same way, with the tool in use and the keep-out zones; a step out of reach or through a wrist singularity is refused before it's sent.
- **STOP** or Space stops the arm and clears queued moves (and any program run, and freedrive); so does Esc, once there's no drag or plan to back out of (and always while the arm moves). Space types a space while you're typing in a text field; STOP and Esc still stop. Hold Enable to carry on.

Over Wi-Fi the controller's status stream stalls for 1 to 3 seconds every half minute to two minutes (a cable to your router avoids it). The panel rides this out: the link light turns grey and shows how old the status is, a click waits for fresh status before it's sent, and NO SIGNAL appears only after 5 seconds of silence.

The panel stops the arm if the page stops talking to it mid-move (a closed tab or a sleeping PC), if the controller's status stays quiet for more than 4.5 seconds, or if the controller reports an error. It listens on this PC only and ignores requests from other web pages. Keep the hardware E-stop within reach; it's still the real safety.

The controller serves one command connection (port 502) at a time and closes any other at once. While the jog panel runs it holds that connection, so its STOP goes over it, and `scripts/arm.py` (including `arm.py stop`) and the notebook can't connect until you quit the panel.

## Programs, teaching and the cell

Below the joints and the tool, the panel has three more sections for machine-tending work: **Program**, **Teach** and **Cell**. The server keeps everything in a folder (`cell/` in the repo, which git ignores; pick another with `--cell DIR`) and runs programs itself, so a run carries on with the page closed and is supervised by the server, not the page. `docs/capabilities.md` specifies the featureset and `docs/decisions.md` records why it works the way it does.

- **The cell** (`cell/cell.json`): *tools*, *frames*, *targets* and *keep-out zones*. Every change is saved at once with a new revision; a page holding an older revision is refused rather than overwriting someone else's change.
  - A **tool** is its tool point (mm from the flange), its payload (kg, and centre of gravity in mm) and its shape: a capsule from the flange towards the tool point, which joins the collision check. **Use** makes it the tool in use and sends its payload to the controller. The controller's own tool offset stays zero: the panel applies the tool point itself (`docs/decisions.md`, D14). To find a tool point you can't measure, name the tool, rest its point on one fixed spike from four or more well-turned angles and press **Touch** at each, then **Fit**: the panel solves for the point and reports how well the touches agree (RMS).
  - A **frame** is a fixture's position and orientation (in the base, or in another frame). Name it, touch its origin, a point along its +X edge and a point on its +Y side with the tool point, then **Fit**. Targets and zones are stored in their frame, so recalibrating a frame after moving the fixture moves every target in it: the message says how many moved, and flags any now out of reach or in another arm configuration.
  - A **target** is a tool-point pose in a frame (with the tool and the joint angles it was taught with, which keep the arm's elbow and wrist the way they were), or plain joint angles. **Plan** beside a target stages a joint move to it, just as dragging does: the shadow arm and the plan bar show it, and nothing moves until you press **Move**. Straight moves to targets run as program steps (**Step** runs one at a time).
  - A **keep-out zone** is a box in a frame: no part of the arm or tool may come within 10 mm of it. The 3D view draws zones as faint boxes, frames as a short light line along their x axis with their name, and targets as dots.
- **Programs** (`cell/programs/<name>.json`) are lists of steps: joint and straight moves to targets (optionally backed off along the tool, at their own speed), waits (for a time, or for an input with a timeout), setting or pulsing an output, halts (pause with a message for the operator) and comments. Repeats, ifs and calls to other programs are written in the JSON (see `scripts/runner.py`) and shown in the list; the page edits the top level and repeats. Click a step to select it: new steps and recorded targets go after it. A target used by a program can't be deleted.
- **Running:** **Run** runs from the top, **Run from** on a step runs from there, **Step** runs one step and waits, **Dry run** plans every move and checks it for collisions without moving anything or setting outputs (a wait for an input assumes it arrives; an if reads the input as it is) and says how long the run would take. **Pause** stops the arm; **Resume** re-plans the interrupted step from wherever the arm is, first moving back onto the straight line it was on, if it was on one. **Stop run** ends it. A step that can't be done (out of reach, blocked, the arm knocked off its path, an input that never came, a controller error) stops the arm and *faults* the run; when a move would have collided, the 3D view shows where, as it does for a blocked plan. Fix the cause and press **Reset**. Every move is planned and collision-checked before it's sent, and watched against its plan while it runs. Programs run at their own speeds (by default 50 mm/s for lines, 20 °/s for joint moves) times the **Override** (10 to 100%); straight lines are capped at 250 mm/s unless the program's JSON says `"fullSpeed": true`. Jogging is refused while a program moves the arm and allowed while it's paused.
- **The log:** the Program section lists the run's events; every run is also written to `cell/logs/<date>/<time>-<program>.jsonl`.
- **Teaching:** hold **Hold for freedrive** until the amber bar fills, guide the arm by hand, and press **Record here** at each pose: as a tool pose in the chosen frame or as joint angles, and with **Then add** set to a straight or joint move, a move to it is added to the program after the selected step. Freedrive starts only from READY. Press the key again to leave: the controller switches the motors off onto their brakes (the arm settles about half a millimetre; recorded targets are from before that), and the panel switches them back on with Enable's checks. STOP in freedrive holds the arm rigid, then switches the motors off; freedrive also ends on its own after 10 minutes, motors off. **Sensitivity** is the controller's teach sensitivity, 90, 100 or 110 % (set it before entering; writing it stops the arm). On this arm 90, the default, stops where you let go; 100 and 110 make the shoulder and elbow lighter, but a raised elbow sinks back a few degrees. Don't turn a joint by hand past about ±350 degrees: the controller faults at its limit (see Troubleshooting).

What the 10 Hz status allows, and what it rules out (no stopping on contact, for one), is in `docs/capabilities.md`.

The server's API (JSON; every POST needs the page's token, so only the page can call it): `GET /api/cell`, `/api/programs`, `/api/program?name=`, `/api/run`, `/api/io`; `POST /api/cell/tool`, `/api/cell/tool/activate`, `/api/cell/tool/delete`, `/api/cell/tool/touch`, `/api/cell/tool/touch/clear`, `/api/cell/tool/calibrate`, `/api/cell/frame`, `/api/cell/frame/delete`, `/api/cell/frame/touch`, `/api/cell/frame/calibrate`, `/api/cell/target`, `/api/cell/target/delete`, `/api/cell/zone`, `/api/cell/zone/delete`, `/api/program`, `/api/program/delete`, `/api/run/start`, `/api/run/pause`, `/api/run/resume`, `/api/run/step`, `/api/run/stop`, `/api/run/reset`, `/api/run/override`, `/api/moveto`, `/api/plan`, `/api/io/set` and `/api/freedrive`. `scripts/jog_server.py` (`api_get`, `api_post`) documents their bodies.

## Controlling it from Python

The SDK talks to the controller over TCP: commands on port 502, status reports on ports 30001 (10 Hz), 30002 (100 Hz) and 30003 (configuration). It needs only the Python standard library (Python 3.8 or later).

```
python scripts/arm.py find                        # scan this PC's networks for the controller
python scripts/arm.py status 192.168.2.1          # identity, state, errors, joint angles, TCP pose
python scripts/arm.py watch 192.168.2.1           # live status (Ctrl+C to quit)
python scripts/arm.py reset 192.168.2.1           # clear a latched error, e.g. after releasing the E-stop
python scripts/arm.py jog 192.168.2.1 -j 6 -d 5   # rotate joint 6 (the wrist) by +5 degrees
python scripts/arm.py home 192.168.2.1            # move to the controller's home pose
python scripts/arm.py stop 192.168.2.1            # stop motion and clear queued commands
```

Use the address `find` reports if yours differs. `find`, `status` and `watch` only read, and `reset` only clears errors: it doesn't enable the motors or move the arm. Before `jog` or `home` moves anything, the script:

- refuses to start, without changing anything on the controller, if the arm is moving, paused or has queued commands, if its joints aren't answering, if it can't read the measured joint angles from the status report, or if the controller's joint targets differ from the measured angles (enabling the motors could then make the arm jump);
- asks for confirmation unless you pass `--yes`, then repeats those checks and makes sure the arm hasn't moved since it showed you the plan;
- resets errors, enables the motors and checks that the targets still match the measured angles;
- follows the move on the status report, and stops the arm on Ctrl+C, a controller error, an early pause or stop, a lost report or a timeout;
- checks that the arm ended where it was sent.

If anything fails after the script starts changing the controller's state, it stops the arm and waits for the controller to confirm the stop. If no confirmation arrives, it tells you to press the hardware E-stop.

Speeds default to 10 deg/s (`--speed`, at most 60). A single jog must be between 0.1 and 15 degrees; raise the upper limit with `--max-step` (at most 90).

To script motions yourself, use the SDK directly, as the notebook does:

```python
import math, sys
sys.path.append("ut_sdk_python/utapi")
from utra.utra_api_tcp import UtraApiTcp

deg = math.pi / 180
arm = UtraApiTcp("192.168.2.1")
arm.reset_err()
arm.set_motion_mode(0)       # position control
arm.set_motion_enable(8, 1)  # enable all joints
arm.set_motion_status(0)     # ready to accept motion
arm.moveto_joint_p2p([0, -30 * deg, 50 * deg, -10 * deg, 90 * deg, 0], 30 * deg, 3, 0)
```

Joint angles are in radians, and Cartesian poses `[x, y, z, rx, ry, rz]` are in millimetres and radians. The controller queues motion commands (up to 1024) and runs them in order; `arm.set_motion_status(4)` stops the arm and clears the queue. The SDK's [UTRA programming guide](https://github.com/UmbraTek/ut_sdk_python/blob/master/utapi/utra/readme.md) explains the motion types, states and reports.

Newer upstream SDK releases move the imports under `utapi.` (for example `from utapi.utra.utra_api_tcp import UtraApiTcp`) and add APIs. The notebook and `scripts/arm.py` use the pinned version. The notebook's cells connect to `192.168.1.43`, the address the controller had when they were written; change it to `192.168.2.1` to run them over the controller's Wi-Fi.

## Troubleshooting

- **`find` reports no arm.** Work through the checklist it prints. You can also try the address on the controller's label directly: `python scripts/arm.py status <ip>`.
- **The power button blinks quickly instead of fading slowly, and `status` shows error `1 (ESTOP_SWITCH)`.** An emergency stop is active. The arm's joints then have no power, so `Servos` shows `comm [-3 -3 -3 -3 -3 -3]` and the joint angles read as placeholder zeros. Release the front E-stop (twist it) and any external E-stop, and make sure the rear *Expansion E-STOP* socket has its switch or the shorting plug from the box. The controller keeps the error until it's reset, so then run `python scripts/arm.py reset 192.168.2.1`. It should report no errors with every joint answering, and the button goes back to fading slowly.
- **`reset` reports error `10 (Servo error)` or joints that still don't answer.** Check that the arm cable is seated in *Robot Connector*; the *Robot Power Indicator* beside it should be lit. Then restart the controller with its power button and run `reset` again.
- **The Ethernet adapter only gets a `169.254.x.x` address.** No DHCP server answered. Give the adapter a static address in the same subnet as the address on the controller's label, for example `192.168.1.200/24` for a controller at `192.168.1.43`.
- **The arm's network uses the same subnet as your home network** (both `192.168.1.x`, for example), so Windows may send arm traffic out the wrong adapter. `find` warns when this PC has two addresses in one subnet. Disconnect the other network while using the arm, or add a host route through the adapter connected to the arm from an administrator PowerShell, replacing the address and adapter name with yours:

  ```
  New-NetRoute -DestinationPrefix 192.168.1.43/32 -InterfaceAlias "Wi-Fi" -NextHop 0.0.0.0
  ```

- **Error `23 (Mechanical angle limit)` comes back at every Enable, after a joint was turned by hand too far** (wrist 1 faulted at 358 degrees). Moves and freedrive are refused while it's flagged, and this firmware can't switch the limit off. With someone holding that joint and a hand on the E-stop: switch off only its motor (`set_motion_enable(n, 0)`, joints numbered from 1; check the enable mask shows just that joint off), release only its brake (`set_brake_enable(n, 1)`) for a few seconds, turn it back a few degrees inside the limit, and re-engage the brake (`set_brake_enable(n, 0)`). Then Enable, freedrive, and turn it the rest of the way. Watch the angle while turning: on 2026-10-21 the first try went the wrong way by over a full turn. `docs/decisions.md` (D39) has the details.
- **`status` shows another non-zero error code.** `status` and `watch` print the error's name, taken from the error table in the controller's own Studio web app. Fix the cause, then clear the error with `reset`. `jog`, `home` and the notebook demos also reset errors before moving; if the error comes back, the script stops and says so.
- **Umbratek's documentation site** (umbratek.com and its wiki) was unreachable as of October 2026. The quick start guide is in [umbratek-cad](https://github.com/Inventor22/umbratek-cad), and the SDK's programming guide is linked above.

## Safety

- Keep the E-stop within reach whenever the motors are enabled.
- Clear the arm's whole reach before running `home` or the notebook demos. They move several joints through large arcs at up to about 30 deg/s.
- `jog` and `home` leave the motors enabled, holding position. Use Umbratek Studio to disable them, or press the E-stop, which cuts power to the arm; after releasing the E-stop, run `reset`.
