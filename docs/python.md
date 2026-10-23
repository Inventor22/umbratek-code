# Controlling the arm from Python

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
