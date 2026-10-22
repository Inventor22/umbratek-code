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
the stub, D50 to D56 for the twin.

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

Python 3.12 with MuJoCo's and NumPy's wheels, pinned in `requirements.txt`. On Windows, from the repo's root:

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
forwarded, not the other loopback addresses the test suites use.

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
  and the camera's true pose, for perception to be built against before a real camera arrives.

### Running it

From the repo's root, with the environment above (`python` meaning `brain\.venv\Scripts\python` on Windows):

```
python -m brain.twin serve --scene brain/twin/scenes/example.json    # the simulated controller, with the twin
python scripts/jog_server.py 127.0.0.1                                # in another shell: the jog panel, against it
```

`serve` takes `tools/utra_sim.py`'s options too (`--joints`, fault injection, ...), `--host` for another loopback
address, `--cell DIR` for a cell folder (the jog panel's is `cell/`), and `--viewer` to watch the scene in MuJoCo's
viewer (its group keys show the collision shapes, zones, and frames and targets: groups 3, 4 and 5). It prints what
the arm touches as it happens.

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
| `POST /obstacles` | `{"add": {obstacle}}`, `{"move": {"name", "pose", "frame"}}` or `{"remove": "name"}`. |
| `POST /cameras` | `{"add": {camera}}`, `{"update": {"name", ...}}` or `{"remove": "name"}`. |

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
  at its centre). Real sensors' noise belongs in perception's tests. Keep-out zones, frames and targets aren't
  rendered; the arm, tool, cart, floor and obstacles are.

A 640 x 480 frame takes about 5 ms (colour and depth) or 16 ms (with segmentation) on the RTX 3080 Ti.

### How it's verified

`tests/twin_tests.py` (`python tests/run_all.py twin`) checks the model against the spine's own kinematics and
collision shapes, the cell's parts, obstacles and physics, the cameras (known points land within 0.12 px of where K
and the pose put them; depth back-projects onto known surfaces to within 0.1 mm), the coupling (the simulator's
joints, exactly, from the SDK's bytes), a live run through the SDK and HTTP, and the replays.

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

## The planner (`brain/planner/`)

A brain that plans planned moves with NVIDIA's cuRobo (cuRoboV2) on the GPU, in WSL2. It serves the interface
(`docs/brain-interface.md`) itself, so the spine on Windows reaches it at `http://127.0.0.1:<port>` as it reaches the
stub, through WSL2's localhost forwarding. The stub stays the fallback that needs nothing installed.
`docs/decisions.md` D60 to D69 say why it is the way it is.

A planned move, from where the arm is to the goal the spine set:

1. **Straight there**, if the spine's own continuous check (`utra_model.check_path`, with the world's obstacles added)
   passes: one segment is the fastest the controller runs.
2. **Otherwise cuRobo**: its graph search (a roadmap of collision-free straight joint moves), or its trajectory
   optimiser when that finds nothing, around the cart, the floor, the keep-out zones and the obstacles, with the arm
   and tool as spheres covering their capsules, grown by 12 mm (the spine's 10 plus 2). A start or goal nearer than
   that to something (the spine lets a move start that close if it moves away) gets a short move away first.
3. **Cut to a few joint moves**: the controller stops at every waypoint (D44), so from each kept waypoint the planner
   takes the farthest point along cuRobo's path that one straight joint move reaches. It screens the candidates
   together on the GPU, and the move it keeps has passed the spine's check with the obstacles added.
4. If cuRobo finds nothing in time, the stub's search (up and over, random via points) against the same world.

Every move it sends has passed the spine's check, so the spine accepts it. The spine doesn't know about the obstacles,
so keeping the margin from them is the planner's job, and that same check does it.

- **One model of the arm.** cuRobo's arm is generated when the planner starts, from the repo's own files: Umbratek's
  URDF with the planning limits (the spine's, within the controller's) and the rated speeds; spheres covering every
  capsule of `collision.json` and the base's cylinder; exactly the pairs of links the spine's check compares; and a
  link on the flange for the active tool's capsules, which come with each request. Its kinematics match the spine's
  to 0.0002 mm.
- **What it plans around** (`--world`): `request` (the default): the tool and keep-out zones each request carries,
  with the arm and its cart from the repo. `twin[=URL]`: as well, the twin's true obstacles, read from its `GET /state`
  for every plan (default `http://127.0.0.1:30100`). `map[=URL]`: as well, perception's obstacle map
  (`docs/world-map.md`, default `http://127.0.0.1:8780`); its ESDF goes to cuRobo grown by half a voxel, and every
  move is checked against a bound that holds between voxel centres too. A source that doesn't answer fails the plan,
  saying why; it's never ignored.
- **Learned steps** go to the stub's scripted skill, `seek`, so programs with learned steps run with this brain too.
- **Starting.** It listens at once and says it doesn't plan (`hello`) until cuRobo is loaded and warmed up: about
  2.5 s and 12 s. The first start on a PC compiles cuRobo's kernels, a minute or two, cached in `~/.cache/warp`.
  It holds about 0.7 GB of the GPU.
- `GET /v1/stats` lists its recent plans and how long each took.

### The planner's environment

In WSL2 (Ubuntu 24.04): its Python 3.12, PyTorch 2.12.1 with CUDA 13 and cuRoboV2 at a pinned commit, in a venv of
its own beside the twin's (`brain/planner/requirements.txt`), from the repo's root:

```
python3 -m venv ~/.venvs/umbratek-planner
~/.venvs/umbratek-planner/bin/python -m pip install -r brain/planner/requirements.txt
```

No CUDA toolkit is needed: cuRobo compiles its kernels when they're first used, and the Windows driver provides CUDA
inside WSL2 (don't install a Linux NVIDIA driver there). Planning around the twin needs the twin's environment in WSL2
too, and port 502 for it (above).

### Running it

From the repo's root in WSL2 (`python` meaning `~/.venvs/umbratek-planner/bin/python`):

```
python -m brain.planner check                  # the environment, cuRobo's model against the spine's, three timed plans
python -m brain.planner serve                  # http://127.0.0.1:8770, where the jog server looks by default
python -m brain.planner serve --world twin     # around the twin's obstacles, with python -m brain.twin serve running
```

On Windows, `python scripts/jog_server.py <controller>` then finds it at `http://127.0.0.1:8770`; on another port
(`serve --port 8790`), give the jog server `--brain http://127.0.0.1:8790`. Only `127.0.0.1` crosses from Windows into
WSL2, so a twin the planner plans around runs in WSL2 too, where the planner reaches its HTTP endpoint.

### How it's verified

`tests/planner_tests.py` (`python tests/run_all.py planner`, which runs it in WSL2 through `wsl.exe`) checks cuRobo's
model against the spine's (kinematics, 6800 points on the capsules all inside their spheres, the base's ring, the
self-collision pairs, the limits), the world sources (the twin's shapes, spheres and capsules in the spine's check, the
map's bound against the true distance at 4000 points), and then plans:

| What | Result |
| ---- | ------ |
| 48 random blocked moves above a table, around a crate, a pole, a ball and a tube | all planned, by graph search, in 2.2 segments on average (at most 4); every one accepted by the spine's shell (`safety.check_path`); 12.1 mm from the obstacles and the table at the least, sampled every 0.5 deg |
| Their planning times | median 0.69 s, 90% within 0.94 s, the worst 1.08 s: on average 0.07 s checking the straight way, 0.25 s in cuRobo, 0.37 s cutting its path (3.2 moves through the spine's check) |
| A keep-out zone the spine sent, a start 11 mm from a wall, a mesh obstacle, a fake perception map | planned and accepted; a goal inside an obstacle, or past a limit, gets no path, saying why |
| The twin, the planner brain and the jog server as processes: taught, planned, taught around the twin's crate, out and back, the crate moved, then a pole, a ball, a tube | 6 runs done, every plan accepted, no contact in the twin (sampled 178 to 351 times a run), the tool on its target to 0.001 mm |
| Their planning times | the brain 1.16 s median, 1.55 s at most (with reading the twin's state); the spine, from starting the planned step to accepting its path, 1.19 s median and 1.63 s at most, against its 10 s deadline |
| The control: the same program with a brain blind to the crate (`--world request`) | the spine accepts its straight path, and the twin sees the arm hit the crate |

Across the boundary, by hand: the spine and the simulator on Windows, the planner in WSL2 on `127.0.0.1:8790`, the
spine suite's taught, planned, taught program around its keep-out zone, out and back six times: all done, nothing
refused, the brain 0.67 to 1.19 s, the spine from starting the step to accepting the path 1.23 to 1.90 s.