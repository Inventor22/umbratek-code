# The brain

The arm's software is split three ways:

- **The spine** is the code in `scripts/`: standard-library Python, the jog panel, programs and the safety shell
  every motion passes through (collision checks, speed caps, supervision).
- **The brain** is a separate process that proposes motion to the spine (`scripts/`), which checks everything it
  proposes before anything moves. The interface is `docs/brain-interface.md`. The brain will hold perception, a world
  model, a motion planner and learned skills, with their heavy dependencies in an environment of its own; it never
  drives the controller itself.
- **The twin**, in `brain/twin/`, is a simulated arm and cell that accepts the same commands as the real arm, so every
  planner and policy can run in simulation first, unchanged.

This folder holds a stub brain that speaks the interface (standard-library Python, no environment needed), the brain's
environment, and the twin. `docs/decisions.md` says why they are the way they are: D40 to D49 for the interface and
the stub, D50 to D56 for the twin, D90 to D92 for its viewer and its cadence.

## Stub brain (`brain/stub/`)

`stub_brain.py` is a stand-in brain that speaks the interface, so planned and learned steps run end to end before the
real brain exists. It's standard-library Python (3.8 or later) and needs no environment of its own: it imports the
spine's protocol, arm model and planner from `scripts/`.

```
python brain/stub/stub_brain.py                 # serves http://127.0.0.1:8770, where the jog server looks by default
python tools/utra_sim.py                        # the simulated arm, in another window
python scripts/jog_server.py 127.0.0.1          # then open http://127.0.0.1:8765
```

It runs the same in WSL2 (`python3 brain/stub/stub_brain.py` from the repo under `/mnt/c/...`): WSL2 forwards Windows'
`localhost` to it, so the jog server on Windows reaches it at `http://127.0.0.1:8770` unchanged.

- **Planned moves:** it plans a path from where the arm is to the goal the spine sets, against the spine's model of the
  arm and its cart plus the tool and keep-out zones the spine sends: straight there if that's clear, else up and over
  (through poses with the tool point 60 to 300 mm above the start and the goal), else through random via points. It
  gives up, saying so, when 60% of the spine's deadline has gone.
- **One learned skill, `seek`:** a scripted stand-in for a policy. It spirals the tool point out from where the step
  began, a few millimetres a burst, keeping its orientation, until an input goes high. Params: `input` (default
  `tool.1`), `stepMm` (8), `pitchMm` (10 per turn), `radiusMm` (40) and `doneAfter` (a number of bursts, to say it's
  done regardless).
- **`--ignore-zones`** plays a buggy brain whose world model has lost the cell's keep-out zones: its paths go straight
  through them, and the spine refuses them. `--port`, `--host` (loopback by default; it warns otherwise, as the
  interface has no authentication), `--seed` (for the random via points) and `--quiet` (no line per request).

`tests/spine_tests.py` (the `spine` suite of `tests/run_all.py`) runs it as a separate process against the simulator,
and plays misbehaving brains with its answers in-process.

## The environment

Python 3.12 with MuJoCo's and NumPy's wheels, pinned in `requirements.txt`. On Windows, from the repo's root (the
twin's viewer runs from this one, natively, even when the twin runs in WSL2):

```
py -3.12 -m venv brain\.venv
brain\.venv\Scripts\python -m pip install -r brain\requirements.txt
```

In WSL2 (Ubuntu 24.04), keep the environment on the Linux disk (a venv under `/mnt/c` is slow), and run from the
repo's root wherever it is:

```
python3 -m venv ~/.venvs/umbratek-brain
~/.venvs/umbratek-brain/bin/python -m pip install -r brain/requirements.txt
sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0
```

The last line is for `serve`: the controller's command port is 502, and Linux keeps ports below 1024 for root. It
lasts until WSL restarts; to keep it, put `net.ipv4.ip_unprivileged_port_start=0` in `/etc/sysctl.d/99-umbratek.conf`.
(`wsl -u root sysctl -w ...` from Windows does the same without a password.) Without a display, the twin renders
through EGL on the GPU (it sets `MUJOCO_GL=egl` itself); `MUJOCO_GL=osmesa` renders in software instead, if EGL
isn't there.

WSL2 forwards Windows' `localhost` into WSL2, not the other way (the spine's D41 builds on this). So when the brain
runs in WSL2, run the twin there too, on `--host 127.0.0.1`: the brain reaches its HTTP endpoint directly, and on
Windows the jog panel, `scripts/arm.py` and the spine reach its controller ports at `127.0.0.1` as if it were local
(checked: `python scripts/arm.py status 127.0.0.1` on Windows reads the twin served from WSL2). Only `127.0.0.1` is
forwarded, not the other loopback addresses the test suites use, and only on ports Windows isn't using itself: while
the simulator (`tools/utra_sim.py`) holds 502, 30001, 30003 and 30099 on Windows, Windows' `127.0.0.1` reaches it,
not the twin, so stop it first.

With a VPN on, WSL2's DNS may not answer a reverse lookup of a loopback address other than `127.0.0.1`, and gives up
after 10 s. Python's `http.server` makes one when it binds, so every HTTP server here (the jog panel's, the stub
brain's and the twin's) is built on `scripts/loopback_http.py`, which doesn't: before it, the twin's HTTP port opened
11 s after it started on `127.0.0.53`. The twin's suite checks that none asks (`tests/twin_tests.py`, `servers`).

`tests/run_all.py` runs the twin's suite with `brain/.venv` (or the Python named by `UMBRATEK_BRAIN_PYTHON`, for
example the WSL2 venv); without one, the suite is reported SKIP and the run fails, as the oracle suites do.

## The twin

A MuJoCo model of the arm on its cart, in the cell, whose joints follow `tools/utra_sim.py`:

- **One source of truth.** The model is generated every time it's loaded, from the repo's own files: the arm's
  kinematics and meshes from Umbratek's URDF (`scripts/model/utra6_1000/`, read by `scripts/utra_model.py`), the
  collision capsules fitted to them, the cart's meshes and collision boxes and the floor (`scripts/model/cart/`),
  the joint limits the controller enforces (`tools/utra_sim.py`), and, given a cell folder, its tool, frames,
  keep-out zones and targets (through `scripts/cell.py`). Its links sit exactly where the spine's kinematics put them
  (checked to 1e-15 m at 300 random poses).
- **The same commands as the real arm.** `serve` runs the controller's stand-in, `tools/utra_sim.py`, inside the
  twin's process, on the same ports (UTRC on 502, the 10 Hz status on 30001, config on 30003, its control port on
  30099), and the twin's arm follows it. The jog panel, `scripts/arm.py`, the SDK and the brain drive it exactly as
  they would drive the real controller. Its motion is the simulator's, fitted to the real arm (decisions D6 to D12):
  10 Hz state, about 110 ms from command to visible motion, position commands only.
- **Obstacles** can be placed in a scene file, from Python or over HTTP: boxes, spheres, cylinders, capsules or
  meshes, in the base frame or a cell frame. Fixed ones stay where they're put (and can be moved); free ones fall
  and are pushed by the arm. The arm is driven by the controller, so in the physics it pushes things but isn't pushed.
- **Contacts and zones.** The twin reports what the arm or tool touches (part, obstacle, depth) and how far it is
  from each keep-out zone. Nothing it finds stops the arm: the controller's own collision detection is off on this
  arm (sensitivity 0), and the spine checks moves before they're sent.
- **Cameras**: colour, depth and segmentation from any pose, fixed in the cell or on the arm, with known intrinsics
  and the camera's true pose, for perception to be built against before a real camera arrives. They're rendered in a
  process of their own, so the twin's 10 ms loop and the simulated controller never wait for a frame.
- **A viewer of its own.** `python -m brain.twin view` shows a running twin in MuJoCo's viewer from a separate
  process, read-only: natively on Windows while the twin runs in WSL2, at the screen's refresh rate. Closing it never
  stops the twin.

### Running it

From the repo's root, with the environment above (`python` meaning `brain\.venv\Scripts\python` on Windows):

```
python -m brain.twin serve --scene brain/twin/scenes/example.json    # the simulated controller, with the twin
python -m brain.twin view                                             # in another shell: the viewer, following it
python scripts/jog_server.py 127.0.0.1                                # in another shell: the jog panel, against it
```

`serve` takes `tools/utra_sim.py`'s options too (`--joints`, fault injection, ...), `--host` for another loopback
address, `--cell DIR` for a cell folder (the jog panel's is `cell/`), and `--viewer` to start the viewer as well (a
process of its own). It prints what the arm touches as it happens. It runs until Ctrl+C in its shell, or until its
process is ended; closing the viewer doesn't stop it.

The viewer, `python -m brain.twin view [--follow URL]` (`http://127.0.0.1:30100` by default), is MuJoCo's own
viewer on the model the twin built: it takes the twin's generated MJCF and meshes over HTTP, so it's the same model
exactly, obstacles added over HTTP included. It follows the twin's state as the twin streams it, every step of its
loop: the arm's joints and where every obstacle is, fixed or free. When the twin rebuilds its model (an obstacle or a
camera added or removed), the viewer loads the new one into the same window and keeps the view. Its group keys show
the collision shapes, zones, and frames and targets (groups 3, 4 and 5); M, A and J show centres of mass, connections
and joints, sized for the arm. It's read-only: it only ever reads the twin (`GET /model` and `GET /pose/stream`),
and whatever is changed in its window (a joint slider, a perturbation) changes only what it shows. If the twin stops,
the window stays, says so, and follows the twin again when it's back. `view --check 5` follows the twin for 5 s
without a window and says what arrived: whether the twin can be followed from where it runs.

```
python -m brain.twin snapshot --scene brain/twin/scenes/example.json --joints 0,10,100,-20,90,0 --out frames
python -m brain.twin replay      # both captures through the simulator and the twin, every figure against its tolerance
python -m brain.twin export --scene brain/twin/scenes/example.json out    # the MJCF and meshes, to open elsewhere
python -m brain.twin check       # the environment, and a test frame
```

While `serve` runs, other processes reach the twin over HTTP on the same address, port 30100 (clear of the brain's
own port, 8770, and the spine suite's). It answers this PC only and refuses requests from web pages, as the jog panel
does.

| Request | What it does |
| ------- | ------------ |
| `GET /state` | Time, joints, flange and tool pose, the controller's state, contacts, zone clearances, obstacles (`obstacles`: where each is; `obstacleShapes`: its shape, size, whether it's fixed, and its mesh file), and every camera's intrinsics and true pose. |
| `GET /cameras/<name>.npz` | A frame: `rgb`, `depth` (m), `segmentation`, `K`, `T_base_camera` and `meta` (JSON). |
| `GET /cameras/<name>/rgb.png`, `/depth.png` | The colour image; depth as 16-bit millimetres. |
| `GET /pose` | The arm's joints (rad) and where every obstacle is (`[x, y, z]` in m and a quaternion `[w, x, y, z]`, base frame), with the model's revision and fingerprint, and the loop's step number. |
| `GET /pose/stream` | The same, one JSON line for every step of the loop (100 a second; `?hz=N` for at most N a second), until the client goes: what the viewer follows. |
| `GET /model` | The generated model as a zip: `twin.xml`, its meshes (`assets/`) and `layout.json` (its revision and fingerprint, and the scene it was built from). |
| `GET /loop` | The loop's cadence over the last 10 s (`?seconds=S`): how many ticks, the gaps between them and the work in each, in ms (`&raw=1`: every gap); the camera process's pid, and the revision of the model it has ready (`camerasReady`). |
| `POST /obstacles` | `{"add": {obstacle}}`, `{"move": {"name", "pose", "frame"}}` or `{"remove": "name"}`. |
| `POST /cameras` | `{"add": {camera}}`, `{"update": {"name", ...}}` or `{"remove": "name"}`. |

Every step, the twin's loop publishes a snapshot of its state. A camera frame is rendered from the first snapshot
after its request arrives, in the camera process; `/pose`, its stream and `/model` are answered from the newest. None
of them waits for the loop or makes it wait. `/state` and the POSTs run on the loop, between its steps.

From Python, without a server:

```python
from brain.twin import Twin, Scene, Obstacle, CameraSpec, Controller
twin = Twin(Scene.load("brain/twin/scenes/example.json"))
ctrl = Controller([0, -30, 50, -10, 90, 0])        # tools/utra_sim.py on a virtual clock
ctrl.enable()
ctrl.moveto_joint_p2p(goal_rad, speed_rad_s, acc_rad_s2)   # the SDK's calls, as the bytes it sends
for t in times:
    ctrl.advance(t)
    twin.follow(t, ctrl.joints(t))                 # free obstacles step along; the arm pushes them
frame = twin.render("overhead")                    # frame.rgb, .depth, .K, .T_base_camera, .points()
```

### Scenes

A scene file is JSON: `cell` (a cell folder, relative to the file), `obstacles` and `cameras`. It uses the cell's
conventions: millimetres, and poses as `{"xyz": [mm], "rpy": [deg]}` with R = Rz(yaw) Ry(pitch) Rx(roll), in the
arm's base frame (z up, origin at the centre of the base's mounting face) or, with `"frame"`, in a cell frame.
`brain/twin/scenes/example.json` is an example; its table is illustrative, not measured.

- An **obstacle** is `{"name", "shape", "size", "pose", "frame", "fixed", "mass", "rgba", "mesh"}`. Sizes in mm: a
  box's `[x, y, z]` (full lengths), a sphere's `[radius]`, a cylinder's or capsule's `[radius, length]` along its z
  axis, a mesh's `[scale]` (1 for a mesh in mm). Free obstacles (`"fixed": false`) need a `mass` in kg. A mesh
  obstacle collides as its convex hull.
- A **camera** is `{"name", "width", "height", "fx", "fy", "cx", "cy", "mount", "pose", "near", "far"}`. `mount` is
  `base`, a cell frame, `flange`, or a link (`link_1` to `link_6`). `pose` is the camera's optical frame in the mount,
  as `{"xyz", "rpy"}`, or, for a fixed camera, `{"eye": [mm], "target": [mm], "up": [x, y, z]}`.

### Camera frames

As a depth camera's driver and most perception code have them:

- The camera frame is OpenCV's: x right, y down, z along the view. `T_base_camera` is base from camera (4 x 4,
  metres): the camera's true pose, wherever it's mounted.
- `K` is the pinhole matrix in pixels, pixel centres at whole numbers; any fx, fy, cx and cy; no distortion. The
  default, 640 x 480 at fx = fy = 615 with the principal point centred, is about a RealSense D435's colour stream.
- Depth is the distance along the camera's z axis (not along the ray), in metres, aligned pixel for pixel with the
  colour image; 0 means no data (nothing there, or outside `near` to `far`, 0.1 to 6 m by default). Saved, it's a
  16-bit PNG in millimetres (depth scale 0.001), as a RealSense writes it.
- Segmentation holds each pixel's MuJoCo geom id (-1 for none), and `labels` names them (`"arm: wrist 1"`,
  `"obstacle: part"`, ...).
- The camera is ideal: no noise, no holes at edges, no exposure, and no anti-aliasing (every pixel is exactly what's
  at its centre). Real sensors' noise belongs in the tests of what reads the frames. Keep-out zones, frames and
  targets aren't rendered; the arm, tool, cart, floor and obstacles are.

A 640 x 480 frame takes about 5 ms (colour and depth) or 16 ms (with segmentation) on the RTX 3080 Ti, and about
75 ms with segmentation in WSL2. Over HTTP, compressing included, an `.npz` frame takes about 70 ms on Windows and
120 ms in WSL2. Frames are rendered in a process of their own, the camera process. It starts with the twin and gets
ready in the background (the model compiled, its OpenGL contexts made, each camera rendered once: about 1 s on
Windows, 2 s in WSL2), and again after each rebuild. It renders from a snapshot of the twin taken after the request
arrives, and its frames are pixel for pixel what `Twin.render` gives in the twin's own process. That's for the twin's
10 ms loop: rendered on the loop, as they were, a frame held it up 83 ms in WSL2 (`docs/decisions.md` D91).

### How it's verified

`tests/twin_tests.py` (`python tests/run_all.py twin`) checks the model against the spine's own kinematics and
collision shapes, the cell's parts, obstacles and physics, the cameras (known points land within 0.12 px of where K
and the pose put them; depth back-projects onto known surfaces to within 0.1 mm), the coupling (the simulator's
joints, exactly, from the SDK's bytes), a live run through the SDK and HTTP, every HTTP server here binding without a
reverse DNS lookup, the camera process's frames against the twin's own, the loop's cadence while frames are pulled,
the viewer, and the replays.

The cadence is checked against the loop's own record (`GET /loop`), with seconds of camera load interleaved with idle
seconds, 6 of each, so the idle ones show what the computer itself does meanwhile. A tick is late when it comes more
than 15 ms after the one before. Under load, the 99th percentile may be at most 2 ms worse than idle (or 15 ms), and
late ticks beyond the idle seconds' share may number at most one for every two frames pulled. Rendered on the loop,
as before, every frame made a tick late. Each figure is the suite's, run on Windows and, by hand, in WSL2:

| What | Windows | WSL2 |
| ---- | ------- | ---------- |
| The loop under a perception client's load: one camera's `.npz` (with segmentation) 5 times a second | 99% of ticks within 10.41 ms of the one before, 0 of 593 late, the worst 10.6 ms (idle: 10.49, 0, 10.8) | 10.14 ms, 0 of 593, 10.4 ms (idle: 10.15, 0, 10.7) |
| ... frames one after another from two clients (`.npz` and PNG) | 165 frames in 6 s; 10.47 ms, 0 of 629, 10.9 ms | 87 frames; 10.15 ms, 0 of 695, 10.3 ms |
| ... the camera process starting with the twin and getting ready | ready in about 1 s; the worst tick meanwhile 10.6 ms; the first frame then 69 ms | 1.7 s; 12.4 ms; 112 ms |
| ... an obstacle added (the model rebuilt): the next frame | 77 ms, the obstacle in it | 107 ms |
| The simulated controller's status meanwhile | 5th to 95th percentile 99.6 to 100.4 ms apart, all 98.5 to 101.3 | 99.8 to 100.2, all 99.4 to 100.7 |
| For comparison: the loop with frames rendered on it, 5 a second (as before; an experiment) | 95% within 18.8 ms, worst 41.6 | 95% within 77.3 ms, worst 113.3 |

The viewer is checked without a window: it follows a joint move at about 100 states a second, each on the
controller's path, and settles on the twin's joints exactly; it takes the twin's new model when an obstacle is added
or removed, shows one moved without a new model, only ever makes `GET /model` and `GET /pose/stream` requests, and,
when the twin stops, says so and follows it again when it's back. A viewer's process ended while it followed leaves
the twin running: its controller's status at 10 Hz, its loop, HTTP, and a move it then takes. Frames pulled 5 times
a second while the arm sweeps J1 from -35 to 35 deg are each taken between their request and their reply, so the
joints read around a request say where the arm was in its frame: the requests take 65 ms (median) on Windows and
100 ms in WSL2. With `TWIN_TEST_WINDOW=1` on Windows, the suite also opens the real window (`serve --viewer`) and
closes it as its close button does: the viewer exits with 0 and the twin carries on.

By hand across the boundary (the twin and a perception client in WSL2 on `127.0.0.1`, the viewer on Windows), the
viewer gets all 100 states a second and MuJoCo's own counter reads 144 frames a second, the screen's refresh rate,
while the client pulls frames and the arm moves. Over 20 s, the twin's loop ticked 2000 times, 99% within 10.11 ms of
the one before and the worst 10.21 ms without the client, and 10.15 and 11.47 ms with it.

The replays send both captures' commands to the simulator again, as the jog panel and the program runner built
them, and compare every status frame with the twin's joints and its tool pose:

| Capture | What | Result | Tolerance |
| ------- | ---- | ------ | --------- |
| both | The twin's kinematics of each frame's joints against the pose the controller reported | 0.00005 mm in all 2292 frames; all but 15 of 116,559 | 0.001 mm (in 6 frames the controller's own pose and joints were a cycle apart, 0.9 mm) |
| 2026-10-21 | At rest | median 0.002 deg, 95% within 0.039 | 0.01, 0.06 deg: the real joints settle within 0.05 deg and the simulator doesn't model the settle |
| 2026-10-21 | Joint moves, each move's timing fitted | median 0.019 deg, 95% within 0.074 | 0.03, 0.1 deg |
| 2026-10-21 | ... at the logged times | median 0.048 deg, 95% within 0.24 | 0.06, 0.6 deg: commands took 12 to 45 ms to cross the controller's Wi-Fi |
| 2026-10-21 | Straight lines, timing fitted: tool position | median 0.22 mm, 95% within 0.58 | 0.3, 0.7 mm: 7 ms of frame jitter at 90 mm/s |
| 2026-10-21 | STOP from 30 deg/s, the stopped move's start fitted | 0.018 deg | 0.03 deg |
| 2026-10-21-tests | 35 program moves: joint moves, timing fitted | median 0.015 deg, 95% within 0.04 | 0.03, 0.06 deg |
| 2026-10-21-tests | Straight lines, timing fitted: tool position | median 0.19 mm, 95% within 0.46, worst 1.4 | 0.3, 0.6, 2 mm |

Each move's fitted timing offset is reported too: within 25 ms in the first capture (Wi-Fi latency), and about 20 ms
early in the second, whose run logs stamp a move once the controller has replied. Frames the controller's Wi-Fi
delivered in a burst after a 3 s stall (99 of them) are left out: their arrival times aren't when they were sent.

The replays also found two places where the simulator, and so the twin, isn't the arm yet (D54). Both are pinned by
the suite so a better model shows up:

- **Turns.** A straight line that mostly turns the tool is paced by a guess (`ROT_MM_PER_RAD`, 1 rad counted as
  100 mm): the arm turned 10 degrees in about 1.2 s at 10 mm/s and 0.85 s at 25 mm/s, the simulator in 1.9 and
  0.92 s. The tool's position is right (within 0.26 mm); the joints lag mid-turn by up to 3.4 degrees.
- **STOP at other speeds.** The STOP model is built from the one stop at 30 deg/s. At 10 deg/s (MT-2), the arm ran
  on 0.17 degrees after STOP, the model 0.06.
