# umbratek-code

Code and experiments for controlling an [Umbratek UTRA 1000 6-DOF robotic arm](https://www.umbratek.com/store/robot/utra/utra-6-1000) with Python.

| Path | What it is |
| ---- | ---------- |
| `ut_sdk_python/` | Umbratek's Python SDK (git submodule), pinned to the March 2022 version the notebook was run against. |
| `scripts/arm.py` | Command-line helper: find the arm on the network, read its status, jog a joint, home, stop. |
| `scripts/jog_server.py`, `scripts/jog_panel.html` | Jog panel: a web page on this PC with a 3D view of the arm on its cart, for moving it one exact step at a time or to a pose planned by dragging joints. |
| `scripts/web/` | The jog panel's 3D view (`arm_view.js`), its collision check (`collide.js`, the same check the server makes) and its inverse kinematics (`ik.js`). |
| `scripts/vendor/` | Not in git: [three.js](https://threejs.org) 0.186.1 (MIT), which draws the 3D view. The jog panel server downloads it the first time it runs and checks every file against SHA-256 hashes pinned in `jog_server.py`. |
| `scripts/model/utra6_1000/`, `scripts/utra_model.py` | Umbratek's URDF and meshes for the UTRA 6-1000 (BSD 2-Clause, from [UmbraTek/ut_arm_ros](https://github.com/UmbraTek/ut_arm_ros)), collision capsules fitted to them, and the kinematics and collision checks the jog panel uses. |
| `scripts/model/cart/` | The aluminium-extrusion cart the arm stands on, as a URDF in the arm's base frame, built from the SolidWorks assembly in [Inventor22/umbratek-cad](https://github.com/Inventor22/umbratek-cad) (see its `SOURCE.txt`). |
| `notebooks/0 UTRA Tests.ipynb` | SDK motion demos: joint point-to-point, linear, blended linear, arcs, servo. |
| `arduino/Chess Gripper/` | Servo calibration sketch for the chess-piece gripper (servo on pin 9, alternates 25° and 120°). |

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
- **Collisions are checked before anything moves.** The panel checks the whole path from the arm to the plan, not just the end, against the arm itself, its cart and the floor, and won't move if any part would come within 10 mm of another. The shadow, path and readout turn red, a red mark shows where it first comes too close, the cart part it would hit is outlined in red, and the plan bar names the parts ("the forearm would come within 10 mm of a cart brace"). The server checks every plan again before sending it. The arm's links are covered by capsules fitted to Umbratek's meshes, so the check errs on the cautious side by up to a few centimetres at the motor housings. Anything mounted on the flange isn't modelled. If the arm already sits within 10 mm of something, it may move away but not closer.
- **Joints:** each joint has a strip. **+** and **−** move it one step. Its meter shows the actual angle (solid bar) and, while it moves, the commanded angle (hollow bar). The amber lights beside the meter count how far the command is ahead of the arm, four lights per step up to the three-step limit, and go out as the arm catches up.
- **Tool:** moves the tool in X/Y/Z or turns it in roll/pitch/yaw. With a plan staged, each row shows in grey where the plan would put the tool.
- **Step and speed:** set them on the small faders at the top of each section; each snaps to the values the arm accepts. Joint steps run 0.1–15° at 1–120 °/s; tool steps run 0.1–50 mm, turns 0.1–15°, at 1–1000 mm/s (the UTRA 6-1000's published top speed, and Umbratek Studio's limit). Each joint move is capped at the top speeds of the joints it turns: 84 °/s for J1 and J2, 114.5 °/s for J3 and 119.1 °/s for the wrists. Those are 80% of the limits the joints' own servos report (220 rad/s at the motor through 120:1 gearboxes for J1 and J2, 250 and 260 rad/s through 100:1 for the rest, so 105, 143 and 149 °/s), because the controller faulted with error 24, a speed limit, on a J3 move at 120 °/s. Umbratek's ROS model of the arm claims faster joints (up to 172 °/s) than these servos allow. Set above those caps, the joint speed fader prints which joints top out lower, and the message line says when a move was capped. Moves ramp up to speed in about 0.3 s, but never harder than the maximum acceleration configured in the controller, which the panel prints when it starts. On this arm that's 200 °/s² for the joints and 200 mm/s² for the tool, so short steps never reach the top speeds: a 5° step peaks near 30 °/s, a 15° step near 55 °/s and a 50 mm step near 100 mm/s. The top speeds show on long planned moves. Speed is a deliberate choice: the speed faders go back to 10 °/s and 20 mm/s every time the panel opens (the steps are remembered), and at the top speeds the tool can move faster than a metre a second. Lock the cart's casters.
- **Redline:** like a speedometer, each speed fader turns red past the speeds that could carry the flange faster than 250 mm/s, the limit the ISO 10218 robot safety standard sets for jogging by hand. The red zone prints where it starts ("250", or "17" on joint speed). With the cap in the red the value turns red, and the message line says how fast the flange could go and which joint would take it there. It says so again on every step in the red, and when you leave it. Red speeds still work; the redline is a warning. On tool speed it's fixed at 250 mm/s, a step of its own. On joint speed it follows the arm: it's the slowest speed at which turning one joint alone would swing the flange past 250 mm/s from where the arm is now. That's usually the shoulder, around 25 °/s with the arm folded and 14 °/s stretched out. A staged plan has a line of its own, often lower, because joints that turn together add up; the fader shows whichever is lower, and the plan bar's note turns red with the figure when the plan passes its own. The panel assumes full reach until it knows the pose. Anything mounted on the flange sticks out further, and moves faster.
- **Clicking and holding:** one click moves exactly one step from the last commanded position, so quick clicks add up. Holding a key repeats the step four times a second, and the commanded position never runs more than three steps ahead of the arm, so a hold covers about four steps a second whatever the speed: pick a bigger step to cover more ground. Joint steps get the same collision check, one step at a time; tool steps don't, because the controller works out their joint angles.
- **STOP** or Space stops the arm and clears queued moves; so does Esc, once there's no drag or plan to back out of (and always while the arm moves). Hold Enable to carry on.

Over Wi-Fi the controller's status stream stalls for about 3 seconds every half minute or so. The panel rides this out: the link light turns grey and shows how old the status is, a click waits for fresh status before it's sent, and NO SIGNAL appears only after 5 seconds of silence.

The panel stops the arm if the page stops talking to it mid-move (a closed tab or a sleeping PC), if the controller's status stays quiet for more than 4.5 seconds, or if the controller reports an error. It listens on this PC only and ignores requests from other web pages. Keep the hardware E-stop within reach; it's still the real safety.

The controller serves one command connection (port 502) at a time and closes any other at once. While the jog panel runs it holds that connection, so its STOP goes over it, and `scripts/arm.py` (including `arm.py stop`) and the notebook can't connect until you quit the panel.

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

- **`status` shows another non-zero error code.** `status` and `watch` print the error's name, taken from the error table in the controller's own Studio web app. Fix the cause, then clear the error with `reset`. `jog`, `home` and the notebook demos also reset errors before moving; if the error comes back, the script stops and says so.
- **Umbratek's documentation site** (umbratek.com and its wiki) was unreachable as of October 2026. The quick start guide is in [umbratek-cad](https://github.com/Inventor22/umbratek-cad), and the SDK's programming guide is linked above.

## Safety

- Keep the E-stop within reach whenever the motors are enabled.
- Clear the arm's whole reach before running `home` or the notebook demos. They move several joints through large arcs at up to about 30 deg/s.
- `jog` and `home` leave the motors enabled, holding position. Use Umbratek Studio to disable them, or press the E-stop, which cuts power to the arm; after releasing the E-stop, run `reset`.
