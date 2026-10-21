# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Users

The arm's owner, a single maker working at a desktop PC beside an Umbratek UTRA 6-1000 arm on a table, with mouse and keyboard, the controller's E-stop within reach, watching the arm more than the screen.

## Product Purpose

Code and tools for driving the arm from this PC: find the controller, read its state, and move it safely from Python. The jog panel replaces Umbratek Studio's Live Control for manual jogging. Studio computes each click from the last displayed pose, so repeated clicks are silently dropped, and its press-and-hold sends a far-away target that only stops when the release arrives.

## Positioning

A click moves the arm by exactly one bounded step from the last commanded goal, checked against the controller's measured state; clicks are never lost or doubled. Dragging joints (their rings in the 3D view, or their fader caps), or the tool itself (a gizmo at the flange, the joints following by inverse kinematics), stages a pose, shown as a shadow arm on the arm's cart; nothing moves until Move sends every staged joint together, and then only if on the way no part of the arm would come within 10 mm of itself, its cart or the floor. Nothing the page sends can run the arm away.

## Operating Context

- The NTRO controller sits at `192.168.2.1` on its own Wi-Fi (`NTRO_43`); this PC joins it with a USB Wi-Fi adapter.
- The arm stands on an aluminium-extrusion cart (in the owner's CAD, Inventor22/umbratek-cad), with the NTRO controller on a plank at the foot of its column; the arm's base is 730 mm above the floor.
- The tool often works within about 10 cm of the table surface.
- The physical E-stop on the controller cuts arm power; afterwards the controller needs a reset.
- Umbratek Studio (served by the controller), `scripts/arm.py` and the SDK notebook can all command the arm too, but the controller serves one command connection (port 502) at a time: while the jog panel runs, the scripts can't connect, and every stop goes over the panel's own connection.

## Capabilities and Constraints

- Python 3.8+ standard library only, on top of the Umbratek SDK pinned as a git submodule (2022). The jog panel's 3D view uses three.js in the browser; it isn't kept in the repo: the server downloads it on its first run and checks it against pinned hashes.
- Controller firmware dated 2021-12: no forward-kinematics query, and the joint "target" register mirrors the measured angles. The panel computes forward kinematics itself from Umbratek's URDF, which matches the controller's reported tool pose.
- Status arrives at 10 Hz on TCP port 30001; commands go to port 502. Over Wi-Fi the status stream stalls for about 3 s every half minute or so.
- The joints' own servos limit them to 220 rad/s at the motor (J1, J2, through 120:1 gearboxes), 250 (J3) and 260 (the wrists, through 100:1): 105, 143 and 149 °/s at the joints. The controller faulted (error 24, a speed limit) with J3 at 120 °/s, so the panel caps each joint at 80% of its servo's limit (84, 114.5 and 119.1 °/s) and the joint speed fader stops at 120 °/s. Umbratek's ROS model rates the joints faster (2.0, 2.0, 2.1 and 3.0 rad/s) than these servos allow. Umbratek publishes 1000 mm/s as the UTRA 6-1000's top tool speed (Studio's limit too), and the tool speed fader stops there.
- ISO 10218-1 limits jogging by hand to 250 mm/s at the tool. The panel's speed faders print a redline there, like a speedometer (on joint speed it follows the arm's pose), but don't enforce it.
- The controller is configured for at most 200 °/s² at the joints and 200 mm/s² at the tool (the panel reads this when it starts and never asks for more), so short steps peak well below the top speeds.
- The jog panel listens on `127.0.0.1` only; it's used from this desktop. Phone or tablet use would need network exposure and a PIN, and isn't planned.

## Product Principles

1. The physical E-stop is the real safety; software stops are a convenience that is either confirmed or loudly flagged.
2. Refuse rather than guess: when the arm's state is unknown, don't move.
3. One input, one bounded motion: a click is one step; Move sends exactly the pose the shadow arm showed, within the joint limits and clear of the arm itself, its cart and the floor.
4. Slow by default; speed is a deliberate choice, up to what the arm's servos can do and never beyond, and the faders say in red when the tool could pass what's safe by hand.
