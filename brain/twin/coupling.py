"""The twin's arm follows the controller's stand-in, tools/utra_sim.py, whose motion model is fitted to the real arm's
capture (docs/decisions.md D6 to D12): the twin's joints are what the simulator, and so the controller, would do with
the same commands. Nothing here models motion of its own.

Two ways to run it:
- Host: the twin's process serves the simulator's ports (UTRC on HOST:502, the 10 Hz status on :30001, config on
  :30003, its control port on :30099), so the jog panel, scripts/arm.py, the SDK and the brain drive it exactly as
  they would drive the real controller, in real time: 10 Hz state, about 110 ms from command to visible motion, position
  commands only.
- Controller: the simulator's Arm on a virtual clock, for replays and tests. Commands go in as the UTRC register
  writes the SDK sends (encoded by the SDK's own hex_data), and time runs as fast as the computer allows.
"""
import math
import threading

from . import repo

TICK_S = 0.005  # how often utra_sim.run advances the arm


class VirtualClock:
    def __init__(self, t=0.0):
        self.t = float(t)

    def __call__(self):
        return self.t


class Controller:
    """tools/utra_sim.py's Arm on a virtual clock. `args` are utra_sim's command-line options (fault injection,
    accelerations, ...). quiet silences the simulator's log of every command (for the whole process: it's
    utra_sim's module-level log)."""

    def __init__(self, joints_deg, args=(), t0=0.0, quiet=True):
        self.sim = repo.spine("utra_sim")
        if quiet:
            self.sim.log = _silent
        self.hex = repo.spine("common.hex_data")
        self.reg = repo.spine("base.arm_reg").ARM_REG(self.sim.AXIS)
        a = self.sim.make_parser().parse_args([f"--joints={','.join(f'{v:.9f}' for v in joints_deg)}", *args])
        self.clock = VirtualClock(t0)
        self.arm = self.sim.Arm(a, now=self.clock)
        self.ticked = t0

    @property
    def now(self):
        return self.clock.t

    def advance(self, t):
        """Run the controller up to time t (s), ticking every 5 ms as the simulator does in real time."""
        while self.ticked + TICK_S <= t + 1e-12:
            self.ticked += TICK_S
            self.clock.t = self.ticked
            self.arm.tick(self.ticked, TICK_S)
        self.clock.t = max(self.clock.t, t)

    def joints(self, t=None):
        """Joint angles (rad) at time t (default: now), as the status frame would report them."""
        with self.arm.lock:
            return self.arm.joints_at(self.clock.t if t is None else t)

    def pose(self):
        """The pose the status frame reports now: [x, y, z (mm), roll, pitch, yaw (rad)]."""
        with self.arm.lock:
            return self.arm.pose(self.arm.joints_at(self.clock.t))

    def target_pose(self):
        """The controller's tool target register (0x50), which mirrors the measured joints."""
        with self.arm.lock:
            return self.arm.pose(self.arm.target_with(self.arm.joints_at(self.clock.t)))

    @property
    def busy(self):
        return self.arm.status == 1 or self.arm.queued() > 0 or bool(self.arm.queue)

    # -- the SDK's calls, as the register writes it sends ----------------------------------------------------------
    def write(self, reg, data):
        return self.arm.handle(1, reg[0], data)

    def reset_err(self):
        return self.write(self.reg.RESET_ERR, self.hex.int8_to_bytes_big(self.reg.RESET_ERR[0], 1))  # as the SDK

    def set_motion_mode(self, mode):
        return self.write(self.reg.MOTION_MDOE, self.hex.int8_to_bytes_big(int(mode), 1))

    def set_motion_enable(self, axis, en):
        return self.write(self.reg.MOTION_ENABLE, self.hex.int8_to_bytes_big([int(axis), int(en)], 2))

    def set_motion_status(self, state):
        return self.write(self.reg.MOTION_STATUS, self.hex.int8_to_bytes_big(int(state), 1))

    def moveto_joint_p2p(self, joints, speed, acc, mvtime=0.0):
        return self.write(self.reg.MOVEJ_P2P, self.hex.fp32_to_bytes_big([*joints, speed, acc, mvtime], 9))

    def moveto_cartesian_line(self, pose, speed, acc, mvtime=0.0):
        return self.write(self.reg.MOVET_LINE, self.hex.fp32_to_bytes_big([*pose, speed, acc, mvtime], 9))

    def enable(self):
        """What Enable sends (scripts/arm.py, the jog panel): clear errors, position mode, motors on, ready."""
        self.reset_err()
        self.set_motion_mode(0)
        self.set_motion_enable(8, 1)
        self.set_motion_status(0)

    def stop(self):
        self.set_motion_status(4)


class Host:
    """tools/utra_sim.py served from this process, in real time, on `host` (a loopback address). Raises OSError if
    its ports are taken (another simulator, or the twin already running there)."""

    def __init__(self, host="127.0.0.1", args=()):
        self.sim = repo.spine("utra_sim")
        self.args = self.sim.make_parser().parse_args(["--host", host, *args])
        self.arm = self.sim.serve(self.args)
        self.thread = threading.Thread(target=self.sim.run, args=(self.arm, self.args), name="utra_sim", daemon=True)
        self.thread.start()

    def sample(self):
        """(time s, joints rad, state): the arm now, on the simulator's own clock."""
        with self.arm.lock:
            t = self.arm.now()
            q = self.arm.joints_at(t)
            return t, q, {"status": self.arm.status, "mode": self.arm.mode, "enable": self.arm.enable,
                          "err": self.arm.err, "moving": self.arm.current is not None}


def degrees(q):
    return [math.degrees(v) for v in q]


def _silent(msg):
    pass
