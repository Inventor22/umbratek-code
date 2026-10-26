# What's where

What each folder and file in this repo is.

| Path | What it is |
| ---- | ---------- |
| `ut_sdk_python/` | Umbratek's Python SDK (git submodule), pinned to the March 2022 version the notebook was run against. |
| `scripts/arm.py` | Command-line helper: find the arm on the network, read its status, jog a joint, home, stop. |
| `scripts/jog_server.py`, `scripts/jog_panel.html` | Jog panel: a web page on this PC with a 3D view of the arm on its cart, for moving it one exact step at a time or to a pose planned by dragging joints; and for teaching, editing and running programs, with the server running and supervising them. |
| `scripts/web/` | The jog panel's 3D view (`arm_view.js`), its collision check (`collide.js`, the same check the server makes), its inverse kinematics (`ik.js`) and its Program, Teach and Cell sections (`featureset.js`). |
| `scripts/cell.py`, `planner.py`, `executor.py`, `runner.py`, `kinematics.py`, `motion_profile.py` | The featureset on the server: the cell (tools, grippers, frames, targets, patterns of slots and their counters, interlocks, zones), straight-line and joint planning with collision checks, executing a plan under supervision, programs and their runner (Retry, Fallback, safe home and calling the owner among its steps), and the kinematics they share. |
| `scripts/brain_protocol.py`, `brain_link.py`, `safety.py` | The spine's side of the brain: the interface (`docs/brain-interface.md`), the link that asks the brain, and the safety shell that checks everything the brain proposes before anything moves. |
| `scripts/loopback_http.py` | The HTTP server class every server here is built on (the jog panel's, the stub brain's, the twin's): Python's own, without the reverse DNS lookup of its address when it binds, which takes 10 s in WSL2 with a VPN on. |
| `brain/` | The brain: a separate process that proposes motion for planned and learned steps. `brain/stub/` is a stand-in that speaks the interface (`brain/README.md`). |
| `brain/twin/` | The twin: a MuJoCo model of the arm on its cart in the cell, generated from this repo's models and cell, whose joints follow `tools/utra_sim.py`. It serves the controller's ports, renders RGB-D cameras with known intrinsics and true poses, and takes obstacles. It runs in the brain's own environment (`brain/README.md`). |
| `scripts/vendor/` | Not in git: [three.js](https://threejs.org) 0.186.1 (MIT), which draws the 3D view. The jog panel server downloads it the first time it runs and checks every file against SHA-256 hashes pinned in `jog_server.py`. |
| `scripts/model/utra6_1000/`, `scripts/utra_model.py` | Umbratek's URDF and meshes for the UTRA 6-1000 (BSD 2-Clause, from [UmbraTek/ut_arm_ros](https://github.com/UmbraTek/ut_arm_ros)), collision capsules fitted to them, and the kinematics and collision checks the jog panel uses. |
| `scripts/model/cart/` | The aluminium-extrusion cart the arm stands on, as a URDF in the arm's base frame, built from the SolidWorks assembly in [Inventor22/umbratek-cad](https://github.com/Inventor22/umbratek-cad) (see its `SOURCE.txt`). |
| `notebooks/0 UTRA Tests.ipynb` | SDK motion demos: joint point-to-point, linear, blended linear, arcs, servo. |
| `arduino/Chess Gripper/` | Servo calibration sketch for the chess-piece gripper (servo on pin 9, alternates 25° and 120°). |
| `tools/utra_sim.py` | A stand-in for the NTRO controller, for trying the jog panel and `arm.py` offline and for the tests; its control port plays a hand, a machine's signals, the E-stop and a gripper. |
| `tools/capture/` | The tools that recorded the arm's status streams, settings and servo registers (with the owner present), and the morning test plan's tools: `tau_check.py` (what the joints' "tau" readings measure), `sensitivity.py` (the controller's collision and teach sensitivity), `record.py` (records the status and config streams, only listening, alongside the panel) and `teach_probe.py` (one controller write per run with read-backs, which worked out freedrive on 2026-10-21). |
| `tools/media/` | The scripts that film the README's pictures from the real panel against the simulator, so they can be made again when the panel changes (`tools/media/README.md`). Their environment (Playwright, Pillow, ffmpeg) is their own. |
| `tools/model/` | The one-off scripts that made the cart model from the SolidWorks assembly (they need SolidWorks) and fitted the arm's collision capsules, kept so both can be made again (`tools/model/README.md`). |
| `data/captures/` | What the real arm measured: raw streams, commands, settings and the analysis, one folder per capture. |
| `tests/` | The test suites and their runner, `python tests/run_all.py` (see `tests/README.md`). |
| `docs/` | The guides: connecting to the arm (`connecting.md`), the jog panel (`jog-panel.md`), programs, teaching and the cell (`programs.md`), Python (`python.md`), troubleshooting (`troubleshooting.md`) and this page. Decisions, test plans and reports. The README's pictures, in `docs/media/`. |

CAD for the mounting plate and gripper, plus the UTRA quick start guide PDF, live in [Inventor22/umbratek-cad](https://github.com/Inventor22/umbratek-cad).

After cloning, fetch the SDK:

```
git submodule update --init
```
