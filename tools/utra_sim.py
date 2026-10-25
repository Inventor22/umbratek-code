#!/usr/bin/env python3
"""A stand-in for the UTRA 6-1000's NTRO controller, for trying scripts/arm.py and the jog panel offline and for the
tests.

    python tools/utra_sim.py [--host 127.0.0.N] [fault options]     (--help lists them)

It speaks UTRC on HOST:502 (one command connection at a time, as the controller does), streams the 10 Hz status
report on HOST:30001 (to any number of clients) and the config report on HOST:30003, and, like this arm's firmware
(2021-12-18), refuses the SDK's 100 Hz port, 30002. HOST is 127.0.0.1 unless --host picks another loopback address,
so several simulators can run side by side.

How it moves comes from what the real arm did on 2026-10-21 (data/captures/2026-10-21; tools/capture/fit_sim.py
fits the model to it):
  - Each move starts START_DELAY_JOINT_S (or START_DELAY_LINE_S, for straight lines) after its command arrives and
    follows a jerk-limited ("double S") profile with the speed and acceleration asked for, capped at the controller's
    configured acceleration, and its configured jerk.
    Joint moves turn every joint in step; straight-line moves keep the tool on the line, orientation turning
    proportionally. Commands queue and run one after another, each from rest, as the capture's back-to-back line
    clicks did. Over the capture's 43 joint moves this predicts the measured angles with a median error of 0.034 deg;
    its straight lines, 0.08 mm.
  - The pose it reports is the forward kinematics of the measured joints (scripts/utra_model.py), which matches the
    real controller's report in every one of the capture's 2292 frames, then the tool offset, if one is set.
  - STOP halts at once with the measured transient: the joints carry on STOP_OVERRUN_S at speed, swing back, and settle
    within 0.4 s (STOP_TRANSIENT, from J1 stopped at 30 deg/s).
  - Switching the motors on shifts the joints by ENABLE_SHIFT_DEG, as measured (0.46 mm at the tool); switching them off
    (or the E-stop) lets them settle onto their brakes by BRAKE_SETTLE_DEG (J3 about 0.45 deg, measured that day).
  - The status frame's queued count leaves out the one command waiting in the planner: the capture's frames showed 0
    with a second click queued behind a moving one.
  - Freedrive (motion mode 2, "joint teaching"), as the real arm did it in the tests that afternoon
    (data/captures/2026-10-21-tests/teach_probe.jsonl): writing the teach sensitivity stops the arm; mode 2 is taken
    only from ready with every motor on (it's ignored while stopped), and the arm then reports state 1 while the
    control port's "hand" moves it at any sensitivity; with an error flagged the mode is taken but the arm holds. STOP
    holds it rigid, still in mode 2; leaving mode 2, free or stopped, switches the motors off onto their brakes.
    Pushing a joint to its limit by hand gives error 23 with the motors off, as wrist 1 did at 358 deg. Single joints
    are enabled by number, from 1. Unverified: making the arm ready while it's stopped in mode 2 (here: freedrive
    again).
Unverified on the real arm, and modelled as plausibly as the SDK allows: pause and resume (a ramp to a halt along the
path and back), collision detection (a control-port "contact" past a threshold set by the collision sensitivity raises
error 31), the joint speed fault (error 24 past 84% of a joint servo's limit, where the real J3 faulted), joint limits
(error 23; wrist 1's fired between 355.9 and 358.0 deg against the xacro's 358.9), unreachable lines (error 21), the
tool offset's convention, and GPIO (16 digital inputs and outputs on the tool and the controller, no analog). The "tau"
readings are the capture's mean values at rest plus a term in acceleration: illustrative, not a model.

The control port (HOST:30099, JSON lines, the simulator's alone) lets tests act on the arm: move it by hand in
freedrive, set inputs, read outputs, press the E-stop, bump it, or read its whole state; and emulate a gripper on an
output, whose inputs (closed, opened, closed on a part) follow it after its time, with a part between its jaws or not,
or jammed. See control().

Time comes from a high-resolution clock and timer (clock, sleep; see D50), so frames leave 100 ms apart and report the
arm where it is at that instant, on Windows as elsewhere. serve() and run() let another process host the simulator
(the twin, brain/twin, does); an Arm can also run on a virtual clock (Arm(a, now=...)), ticked by its owner.
"""
import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ut_sdk_python", "utapi"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import crc16  # noqa: E402
import motion_profile  # noqa: E402
import utra_model  # noqa: E402

AXIS = 6
CONTROL_PORT = 30099
NAMES = {0x0C: "RESET_ERR", 0x20: "MOTION_MODE", 0x21: "MOTION_ENABLE", 0x22: "BRAKE_ENABLE", 0x25: "MOTION_STATUS",
         0x26: "CMD_NUM", 0x30: "MOVET_LINE", 0x31: "MOVET_LINEB", 0x38: "MOVEJ_P2P", 0x3A: "MOVEJ_HOME",
         0x3B: "MOVE_SLEEP", 0x3D: "MOVES_JOINT", 0x40: "TCP_JERK", 0x41: "TCP_MAXACC", 0x42: "JOINT_JERK",
         0x43: "JOINT_MAXACC", 0x44: "TCP_OFFSET", 0x45: "LOAD_PARAM", 0x46: "GRAVITY_DIR", 0x47: "COLLIS_SENS",
         0x48: "TEACH_SENS", 0x61: "UTRC_INT32_NOW"}

# From the capture (see the module's docstring and tools/capture/fit_sim.py).
START_DELAY_JOINT_S = 0.020  # joint moves: fit over the capture's 43 joint moves
START_DELAY_LINE_S = 0.010  # straight lines: fit over its 24 lines
STOP_OVERRUN_S = 0.00639  # 0.192 deg past the stop point at 30 deg/s
# The settle after STOP: (seconds after the stop, offset from the final angle in units of the overrun distance), J1's
# frames after the capture's STOP.
STOP_TRANSIENT = ((0.0, -1.0), (0.0992, -1.7222), (0.1993, -0.1183), (0.2991, -0.0172), (0.3991, -0.001),
                  (0.4991, 0.0))
ENABLE_SHIFT_DEG = (-0.0016, 0.0002, 0.0554, -0.0080, -0.0007, 0.0015)
# Joints settling onto their brakes when their motors switch off, measured 2026-10-21 at folded poses like the
# capture's: J3 +0.43 deg leaving freedrive, +0.46 on a power cut, +0.49 on the E-stop; J2 +0.02. (The sign follows
# gravity, so other poses may settle the other way.)
BRAKE_SETTLE_DEG = (0.0, 0.02, 0.45, 0.0, 0.0, 0.0)
FULL_MASK = (1 << AXIS) - 1
REST_TAU = (-2.56, 5.52, -25.95, 0.87, -0.07, 1.69)  # the capture's mean "tau" at rest
TAU_PER_ACC = 0.15  # illustrative: J1 read about +/-15 accelerating at 100 deg/s^2
# Faults, unverified: joint limits from Umbratek's xacro, and the speed past which a joint faults (error 24): 84% of
# its servo's limit, which is where the real J3 faulted (120.3 deg/s against 143.2).
JOINT_LIMITS_RAD = ((-2 * math.pi + 0.02, 2 * math.pi - 0.02),) * 2 + ((-2.8811, 2.8811),) + \
                   ((-2 * math.pi + 0.02, 2 * math.pi - 0.02),) * 3
SERVO_LIMIT_DEG_S = (105.0, 105.0, 143.2, 149.0, 149.0, 149.0)
FAULT_SPEED_DEG_S = tuple(0.84 * v for v in SERVO_LIMIT_DEG_S)
ROT_MM_PER_RAD = 100.0  # unverified: how a line move's turn counts against its speed (1 rad as 100 mm)
PAUSE_RAMP_S = 0.25  # unverified: pause and resume ramp along the path
CONTACT_THRESHOLDS = (None, 40.0, 30.0, 20.0, 12.0, 6.0)  # unverified: contact needed to trip each sensitivity
LINE_SAMPLE_MM = 1.0


def log(msg):
    print(f"[sim {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ---- time: a high-resolution clock and timer -----------------------------------------------------------------
# The status stream is paced at 10 Hz and every reported angle is worked out from the clock, so both need better
# than the ~15.6 ms system tick that time.monotonic() (before Python 3.13) and time.sleep() (before 3.11) follow on
# Windows: with them, frames came 63 to 140 ms apart, and an angle could be read up to a tick late (0.5 deg at
# 30 deg/s). perf_counter() is monotonic and high-resolution everywhere (QueryPerformanceCounter on Windows,
# CLOCK_MONOTONIC on Linux). On Windows before 3.11, sleeps wait on a high-resolution waitable timer, as 3.11's own
# time.sleep() does; elsewhere time.sleep() already is one (docs/decisions.md, D50).
clock = time.perf_counter


def _sleeper():
    if sys.platform != "win32" or sys.version_info >= (3, 11):
        return time.sleep
    import ctypes
    from ctypes import wintypes
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateWaitableTimerExW.restype = wintypes.HANDLE
    kernel32.CreateWaitableTimerExW.argtypes = (wintypes.LPVOID, wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD)
    kernel32.SetWaitableTimer.restype = wintypes.BOOL
    kernel32.SetWaitableTimer.argtypes = (wintypes.HANDLE, ctypes.POINTER(ctypes.c_longlong), wintypes.LONG,
                                          wintypes.LPVOID, wintypes.LPVOID, wintypes.BOOL)
    kernel32.WaitForSingleObject.restype = wintypes.DWORD
    kernel32.WaitForSingleObject.argtypes = (wintypes.HANDLE, wintypes.DWORD)
    high_resolution, all_access, infinite = 0x00000002, 0x001F0003, 0xFFFFFFFF
    timers, timers_lock = [], threading.Lock()  # one timer per sleeping thread, reused

    def sleep(seconds):
        if seconds <= 0:
            time.sleep(0)
            return
        with timers_lock:
            timer = timers.pop() if timers else kernel32.CreateWaitableTimerExW(None, None, high_resolution, all_access)
        if not timer:  # no high-resolution timers before Windows 10 1803: the system tick it is
            time.sleep(seconds)
            return
        try:
            due = ctypes.c_longlong(-max(1, round(seconds * 1e7)))  # relative, in 100 ns units
            if kernel32.SetWaitableTimer(timer, ctypes.byref(due), 0, None, None, False):
                kernel32.WaitForSingleObject(timer, infinite)
            else:
                time.sleep(seconds)
        finally:
            with timers_lock:
                timers.append(timer)

    return sleep


sleep = _sleeper()


def sleep_until(deadline):
    left = deadline - clock()
    if left > 0:
        sleep(left)


def next_tick(tick, period):
    """Sleeps until `tick` + `period` and returns that time: an absolute schedule, so periods don't drift. If the
    process was starved of CPU for more than a period, the schedule starts afresh rather than catching up in a
    burst of frames."""
    tick += period
    now = clock()
    if tick < now - period:
        tick = now
    sleep_until(tick)
    return tick


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def io_line(name):
    """"tool.N" or "controller.N" as the controller's (line, bit): the tool is line 2, the controller line 3."""
    line, _, bit = str(name).partition(".")
    if line not in ("tool", "controller") or not bit.isdigit() or int(bit) > 15:
        raise ValueError(f"not a digital line: {name!r} (tool.0 to tool.15, controller.0 to controller.15)")
    return (2 if line == "tool" else 3), int(bit)


def vars_text(request):
    return ", ".join(f"{k} {v}" for k, v in request.items() if k != "cmd")


# ---- kinematics: the model's forward kinematics, and an inverse of its own -----------------------------------

MODEL = utra_model.ArmModel()


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rot_rpy(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), the controller's convention."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def pose_matrix(pose):
    """4x4 transform from [x, y, z (mm), roll, pitch, yaw (rad)]."""
    R = rot_rpy(*pose[3:6])
    return [R[0] + [pose[0] / 1000.0], R[1] + [pose[1] / 1000.0], R[2] + [pose[2] / 1000.0], [0.0, 0.0, 0.0, 1.0]]


def matrix_pose(T):
    pitch = math.asin(max(-1.0, min(1.0, -T[2][0])))
    if abs(T[2][0]) > 1 - 1e-9:
        roll, yaw = 0.0, math.atan2(-T[0][1], T[1][1])
    else:
        roll, yaw = math.atan2(T[2][1], T[2][2]), math.atan2(T[1][0], T[0][0])
    return [T[0][3] * 1000.0, T[1][3] * 1000.0, T[2][3] * 1000.0, roll, pitch, yaw]


def flange(q):
    """Flange transform for joints q (radians)."""
    return MODEL.frames([math.degrees(v) for v in q])[-1]


def rot_log(R):
    """Rotation vector of R."""
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0))
    angle = math.acos(c)
    v = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
    if angle < 1e-9:
        return [x / 2.0 for x in v]
    if math.pi - angle < 1e-6:
        k = max(range(3), key=lambda i: R[i][i])
        axis = [(R[i][k] + (1.0 if i == k else 0.0)) / 2.0 for i in range(3)]
        n = math.sqrt(sum(a * a for a in axis)) or 1.0
        return [a / n * angle for a in axis]
    s = angle / (2.0 * math.sin(angle))
    return [x * s for x in v]


def rot_exp(w):
    angle = math.sqrt(sum(x * x for x in w))
    if angle < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z = (v / angle for v in w)
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [[c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t]]


def mat3(T):
    return [row[:3] for row in T[:3]]


def mul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose3(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def pose_error(T_target, T):
    """[dx, dy, dz (m), wx, wy, wz (rad)]: what turns T into T_target, in the base frame."""
    w = rot_log(mul3(mat3(T_target), transpose3(mat3(T))))
    return [T_target[i][3] - T[i][3] for i in range(3)] + w


def solve6(A, b):
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-15:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def ik(T_target, seed, iterations=30):
    """Joints (radians) putting the flange at T_target, by Newton steps with a numerical Jacobian from `seed`; None
    when it doesn't converge (out of reach, or at a singularity)."""
    q = list(seed)
    for _ in range(iterations):
        e = pose_error(T_target, flange(q))
        if math.sqrt(sum(x * x for x in e[:3])) < 1e-7 and math.sqrt(sum(x * x for x in e[3:])) < 1e-7:
            return q
        J = []
        for j in range(AXIS):
            h = 1e-6
            dq = list(q)
            dq[j] += h
            J.append([v / h for v in pose_error(flange(dq), flange(q))])
        JT = J  # rows are joints: J^T
        A = [[sum(JT[r][k] * JT[c][k] for k in range(6)) + (1e-9 if r == c else 0.0) for c in range(AXIS)]
             for r in range(AXIS)]
        b = [sum(JT[r][k] * e[k] for k in range(6)) for r in range(AXIS)]
        step = solve6(A, b)
        if step is None:
            return None
        biggest = max(abs(s) for s in step)
        if biggest > 0.2:
            step = [s * 0.2 / biggest for s in step]
        q = [v + s for v, s in zip(q, step)]
    return None


# ---- motions ---------------------------------------------------------------------------------------------------

class Motion:
    """One queued command, from rest to rest. joints(tau) is where it puts the joints tau seconds into its profile."""

    def __init__(self, kind, label, start, speed, acc, jerk, distance):
        self.kind, self.label, self.start = kind, label, list(start)
        self.profile = motion_profile.double_s(distance, speed, acc, jerk) if distance > 0 else None
        self.duration = self.profile.duration if self.profile else 0.0

    def fraction(self, tau):
        if self.profile is None or self.profile.distance <= 0:
            return 1.0
        return self.profile.position(tau) / self.profile.distance

    def joints(self, tau):
        raise NotImplementedError


class JointMotion(Motion):
    def __init__(self, label, start, goal, speed, acc, jerk):
        self.goal = list(goal)
        lead = max(abs(g - s) for g, s in zip(goal, start))
        super().__init__("joint", label, start, speed, acc, jerk, lead)

    def joints(self, tau):
        f = self.fraction(tau)
        return [s + (g - s) * f for s, g in zip(self.start, self.goal)]


class LineMotion(Motion):
    """A straight line of the tool point (the flange with the tool offset), orientation turning proportionally. The
    joint path is solved in LINE_SAMPLE_MM steps when the move starts and interpolated between them."""

    def __init__(self, label, start, offset, goal_pose, speed, acc, jerk):
        self.offset = offset
        T0 = mat_mul(flange(start), offset)
        T1 = pose_matrix(goal_pose)
        self.p0, self.p1 = [T0[i][3] for i in range(3)], [T1[i][3] for i in range(3)]
        self.R0 = mat3(T0)
        self.w = rot_log(mul3(transpose3(self.R0), mat3(T1)))  # turn, in the start frame
        length_mm = math.dist(self.p0, self.p1) * 1000.0
        turn = math.sqrt(sum(x * x for x in self.w))
        super().__init__("line", label, start, speed, acc, jerk, max(length_mm, turn * ROT_MM_PER_RAD))
        self.samples = None

    def pose_at(self, f):
        R = mul3(self.R0, rot_exp([x * f for x in self.w]))
        p = [a + (b - a) * f for a, b in zip(self.p0, self.p1)]
        return [R[0] + [p[0]], R[1] + [p[1]], R[2] + [p[2]], [0.0, 0.0, 0.0, 1.0]]

    def solve(self, inv_offset):
        """The joint path, or the fraction along the line where there's no solution."""
        n = max(1, int(math.ceil((self.profile.distance if self.profile else 0.0) / LINE_SAMPLE_MM)))
        q, out = list(self.start), [list(self.start)]
        for k in range(1, n + 1):
            q = ik(mat_mul(self.pose_at(k / n), inv_offset), q)
            if q is None:
                return k / n
            out.append(q)
        self.samples = out
        return None

    def joints(self, tau):
        if not self.samples:
            return list(self.start)
        x = self.fraction(tau) * (len(self.samples) - 1)
        i = min(int(x), len(self.samples) - 2) if len(self.samples) > 1 else 0
        if len(self.samples) == 1:
            return list(self.samples[0])
        u = x - i
        return [a + (b - a) * u for a, b in zip(self.samples[i], self.samples[i + 1])]


class SleepMotion(Motion):
    def __init__(self, label, start, seconds):
        super().__init__("sleep", label, start, 1.0, 1.0, 0.0, 0.0)
        self.duration = max(0.0, seconds)

    def joints(self, tau):
        return list(self.start)


class ServoMotion(Motion):
    """moveto_servo_joint: up to three joint targets, each reached linearly in its own time (unverified)."""

    def __init__(self, label, start, frames):
        super().__init__("servo", label, start, 1.0, 1.0, 0.0, 0.0)
        self.frames, self.duration = frames, sum(t for _, t in frames)

    def joints(self, tau):
        q = list(self.start)
        for goal, t in self.frames:
            if tau <= t and t > 0:
                return [a + (b - a) * tau / t for a, b in zip(q, goal)]
            tau -= t
            q = list(goal)
        return q


def offset_matrix(offset):
    return pose_matrix(offset[:3] + offset[3:6])


def invert(T):
    R = mat3(T)
    Rt = transpose3(R)
    p = [T[i][3] for i in range(3)]
    t = [-sum(Rt[i][k] * p[k] for k in range(3)) for i in range(3)]
    return [Rt[0] + [t[0]], Rt[1] + [t[1]], Rt[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


# ---- the controller ------------------------------------------------------------------------------------------

class Arm:
    def __init__(self, a, now=None):
        self.a = a
        self.now = now or clock  # the time source: the high-resolution clock, or a twin's virtual one
        self.lock = threading.RLock()
        self.uuid = b"SIMUTRA6100000001"
        self.sw = b"V1.0.SIM  20211218  "
        self.hw = b"NTRO-SIM  HW1.0     "
        self.mode, self.enable, self.brake = 0, 0, 0
        self.err, self.warn = a.error, 0
        self.status = 4  # stopped at power-on
        self.phantom_queued = a.queued  # a queue count with nothing behind it
        self.queue = []  # Motions waiting
        self.current = None  # the running Motion
        self.clock = None  # (wall time at tau = 0, rate) for the running motion; see progress()
        self.tau0 = 0.0
        self.pause = None  # (kind "down"/"up", started, tau at start) while ramping
        self.q = [math.radians(d) for d in a.joints]
        self.transient = None  # (started, final joints, overrun per joint) after a STOP
        self.hand = None  # (started, start joints, goal joints, seconds, past a limit) while moved by hand in freedrive
        self.received = 0.0  # when the last motion command arrived
        self.ready_at = None  # when the arm was last set ready
        self.report_connections, self.nan_joints = 0, False
        self.started = self.now()
        self.config = {"tcp_jerk": a.tcp_jerk, "tcp_maxacc": a.tcp_maxacc, "rot_maxacc": 4.0, "rot_jerk": 8.0,
                       "joint_jerk": math.radians(a.joint_jerk), "joint_maxacc": math.radians(a.joint_maxacc),
                       "tcp_offset": [0.0] * 6, "payload": [0.0] * 4, "gravity": [0.0, 0.0, -1.0],
                       "collis_sens": a.collision_sens, "teach_sens": a.teach_sens}
        self.inputs = {2: 0, 3: 0}  # digital inputs on the tool (line 2) and the controller (line 3)
        self.outputs = {2: 0, 3: 0}
        # Grippers emulated on the control port ("gripper"): (line, bit) of the output that drives each -> how it
        # moves and the inputs it sets; and whether a part sits between the jaws ("part").
        self.grippers = {}
        self.part = False
        self.velocity = [0.0] * AXIS
        self.accel = [0.0] * AXIS
        self.nudge = [0.0] * AXIS

    # -- state ----------------------------------------------------------------------------------------------
    def queued(self):
        """The queue as the status frame and CMD_NUM report it: not counting the one command in the planner."""
        return max(0, len(self.queue) - 1) + self.phantom_queued

    def tool(self):
        return offset_matrix(self.config["tcp_offset"])

    def pose(self, q=None):
        q = self.q if q is None else q
        return matrix_pose(mat_mul(flange(q), self.tool()))

    def target(self):
        """Mirrors the measured joints, like the real controller, plus injected faults."""
        t = list(self.q)
        t[0] += math.radians(self.a.target_offset)
        t[0] += math.radians(self.a.enable_jump if self.enable else self.a.stale_target)
        return t

    def tau(self):
        return [REST_TAU[j] + TAU_PER_ACC * math.degrees(self.accel[j]) for j in range(AXIS)]

    # -- motion -----------------------------------------------------------------------------------------------
    def progress(self, now):
        """tau: how far the running motion is through its profile, through any pause ramp."""
        if self.current is None:
            return 0.0
        if self.pause is None:
            return self.tau0 + max(0.0, now - self.clock)
        kind, started, tau_start = self.pause
        dt = max(0.0, now - started)
        if kind == "down":
            dt = min(dt, PAUSE_RAMP_S)
            return tau_start + dt - dt * dt / (2 * PAUSE_RAMP_S)
        if kind == "held":
            return tau_start
        if dt < PAUSE_RAMP_S:  # "up"
            return tau_start + dt * dt / (2 * PAUSE_RAMP_S)
        return tau_start + PAUSE_RAMP_S / 2 + (dt - PAUSE_RAMP_S)

    def joints_at(self, now):
        if self.transient is not None:
            started, final, overrun = self.transient
            t = now - started
            for (t0, v0), (t1, v1) in zip(STOP_TRANSIENT, STOP_TRANSIENT[1:]):
                if t <= t1:
                    k = v0 + (v1 - v0) * (max(t, t0) - t0) / (t1 - t0)
                    return [f + k * o for f, o in zip(final, overrun)]
            self.transient = None
            return list(final)
        if self.hand is not None:
            started, start, goal, seconds, past_limit = self.hand
            f = min(1.0, (now - started) / seconds) if seconds > 0 else 1.0
            f = f * f * (3 - 2 * f)
            q = [s + (g - s) * f for s, g in zip(start, goal)]
            if f >= 1.0:  # done: the arm stays where the hand left it, whichever call notices first
                self.hand = None
                self.q = list(q)
                if past_limit:  # as on the arm (2026-10-21): J4 turned by hand to 358 deg gave error 23, motors off
                    self.motors_off(FULL_MASK)
                    self.mode, self.err, self.status = 0, 23, 2
                    log("moved by hand to a joint limit -> error 23, motors off, out of freedrive")
                    return list(self.q)
            return q
        if self.current is None:
            return list(self.q)
        return self.current.joints(self.progress(now))

    def finish(self, now):
        self.q = self.current.joints(self.current.duration)
        log(f"motion complete -> {'next' if self.queue else 'ready'}")
        self.current, self.pause = None, None
        self.nan_joints = self.a.nan_after_move
        if not self.queue:
            self.status = 2

    def begin(self, now, start_at):
        m = self.queue.pop(0)
        m.start = list(self.q)
        if isinstance(m, JointMotion):
            m.__init__(m.label, self.q, m.goal, m.speed, m.acc, m.jerk)
        elif isinstance(m, LineMotion):
            m.__init__(m.label, self.q, self.tool(), m.goal_pose, m.speed, m.acc, m.jerk)
            missing = m.solve(invert(self.tool()))
            if missing is not None:
                self.fault(21, f"{m.label}: no inverse kinematics solution {missing * 100:.0f}% along the line", now)
                return
        self.current, self.clock, self.tau0, self.pause = m, start_at, 0.0, None
        self.status = 1

    def motors_off(self, bits, now=None):
        """Switch off the motors in `bits`: the arm stops, and those joints settle onto their brakes."""
        if self.current is not None and now is not None:
            self.q = self.joints_at(now)
        off = self.enable & bits
        self.enable &= ~bits
        self.q = [v + (math.radians(BRAKE_SETTLE_DEG[j]) if off >> j & 1 else 0.0) for j, v in enumerate(self.q)]
        self.current, self.pause, self.hand, self.transient = None, None, None, None
        self.queue.clear()
        self.status = 4
        if off:
            log(f"motors off (mask {off:06b}): those joints settle onto their brakes")

    def fault(self, code, why, now, transient=False):
        if transient:
            self.halt(now)
        else:
            q = self.joints_at(now)
            self.q, self.transient = q, None
            self.current, self.pause = None, None
        self.err, self.status = code, 4
        self.queue.clear()
        log(f"{why} -> error {code}")

    def halt(self, now):
        """STOP: the joints carry on STOP_OVERRUN_S at speed, then settle back (the measured transient)."""
        if self.current is None:
            return  # nothing moving (a transient already under way carries on)
        q = self.joints_at(now)
        overrun = [v * STOP_OVERRUN_S for v in self.velocity] if self.status == 1 else [0.0] * AXIS
        final = [a + o for a, o in zip(q, overrun)]
        if any(abs(o) > 1e-9 for o in overrun):
            self.transient = (now, final, overrun)
        self.q = final
        self.current, self.pause = None, None

    def tick(self, now, dt):
        with self.lock:
            self._grippers(now)
            q_before = list(self.q) if self.current is None else self.joints_at(now - dt)
            if self.hand is not None:
                self.q = self.joints_at(now)
            if self.current is None:
                if self.queue and self.status in (0, 1, 2) and not self.err:
                    delay = START_DELAY_LINE_S if isinstance(self.queue[0], LineMotion) else START_DELAY_JOINT_S
                    self.begin(now, max(now, self.received + delay))
                elif self.status == 0 and not self.queue and self.ready_at and now - self.ready_at > 0.15:
                    self.status = 2  # as the capture's frames went from 0 to 2 once the arm was ready
                self._kinematics(q_before, now, dt)
                return
            if self.status not in (1, 3):
                return
            moving_for = now - self.clock
            if self.a.error_after and moving_for > self.a.error_after:
                self.err, self.status, self.current, self.pause = 9, 4, None, None
                self.queue.clear()
                log("injected error 9 mid-move -> stopped")
                return
            if self.a.pause_after and moving_for > self.a.pause_after and self.status == 1:
                self.status, self.pause = 3, ("held", now, self.progress(now))
                self.a.pause_after = 0.0
                log("injected pause mid-move")
                return
            tau = self.progress(now)
            if self.pause is not None and self.pause[0] == "down" and now - self.pause[1] >= PAUSE_RAMP_S:
                self.pause = ("held", now, tau)
            elif self.pause is not None and self.pause[0] == "up" and now - self.pause[1] >= PAUSE_RAMP_S:
                self.tau0, self.clock, self.pause = tau, now, None
            if (self.pause is None or self.pause[0] == "up") and tau >= self.current.duration:
                self.finish(now)
                self._kinematics(q_before, now, dt)
                return
            self.q = self.current.joints(tau)
            self._kinematics(q_before, now, dt)
            for j in range(AXIS):
                if abs(math.degrees(self.velocity[j])) > FAULT_SPEED_DEG_S[j]:
                    self.fault(24, f"J{j + 1} at {abs(math.degrees(self.velocity[j])):.1f} deg/s, past its servo's limit", now)
                    return

    def _kinematics(self, q_before, now, dt):
        velocity = [(a - b) / dt for a, b in zip(self.joints_at(now), q_before)] if dt > 0 else [0.0] * AXIS
        self.accel = [(v - w) / dt for v, w in zip(velocity, self.velocity)] if dt > 0 else [0.0] * AXIS
        self.velocity = velocity

    def _grippers(self, now):
        """Each emulated gripper follows its output: once the output has stayed at its closing value for the gripper's
        time, its "closed" input reads 1, and its "part" input too if a part is there (and while it stays); once it has
        stayed at the other value that long, its "opened" input reads 1. While it moves, neither does, and a jammed
        gripper never gets there. Inputs are active high."""
        for (line, bit), g in self.grippers.items():
            closing = ((self.outputs[line] >> bit) & 1) == g["close"]
            if closing != g["closing"]:
                g["closing"], g["since"] = closing, now
            there = not g["jammed"] and now - g["since"] >= g["seconds"]
            for key, value in (("closed", there and closing), ("opened", there and not closing),
                               ("part", there and closing and self.part)):
                if g[key] is not None:
                    at, n = g[key]
                    self.inputs[at] = (self.inputs[at] | (1 << n)) if value else (self.inputs[at] & ~(1 << n))

    def accept(self, motion, shown, speed_shown, acc_shown):
        log(f"{motion.label}: goal=[{' '.join(f'{g:.2f}' for g in shown)}] speed={speed_shown:.1f} acc={acc_shown:.1f} "
            f"(status={self.status} enable=0x{self.enable:x} err={self.err} queued={self.queued()})")
        if self.a.ignore_moves or self.status not in (0, 1, 2) or not self.enable or self.err or self.mode not in (0, 1):
            log(f"{motion.label}: discarded")
            return
        self.queue.append(motion)
        self.received = self.now()

    def joint_move(self, label, goal, speed, acc):
        for j, g in enumerate(goal):
            low, high = JOINT_LIMITS_RAD[j]
            if not low <= g <= high:
                log(f"{label}: J{j + 1} target {math.degrees(g):.1f} deg past its limit")
                if self.enable and not self.err and self.status in (0, 1, 2) and self.mode == 0:
                    self.fault(23, f"{label}: joint limit", self.now())
                return
        acc = min(acc, self.config["joint_maxacc"]) if acc > 0 else self.config["joint_maxacc"]
        m = JointMotion(label, self.q, goal, speed, acc, self.config["joint_jerk"])
        m.speed, m.acc, m.jerk = speed, acc, self.config["joint_jerk"]
        self.accept(m, [math.degrees(g) for g in goal], math.degrees(speed), math.degrees(acc))

    def line_move(self, label, pose, speed, acc):
        acc = min(acc, self.config["tcp_maxacc"]) if acc > 0 else self.config["tcp_maxacc"]
        shown = list(pose[:3]) + [math.degrees(v) for v in pose[3:6]]
        if self.a.ik_fail:
            log(f"{label}: goal=[{' '.join(f'{g:.2f}' for g in shown)}] speed={speed:.1f} acc={acc:.1f}")
            if self.enable and not self.err:
                self.fault(21, f"{label}: no inverse kinematics solution", self.now())
            return
        m = LineMotion(label, self.q, self.tool(), pose, speed, acc, self.config["tcp_jerk"])
        m.goal_pose, m.speed, m.acc, m.jerk = list(pose), speed, acc, self.config["tcp_jerk"]
        self.accept(m, shown, speed, acc)

    # -- the UTRC registers -----------------------------------------------------------------------------------
    def handle(self, rw, cmd, data):
        with self.lock:
            now = self.now()
            if self.hand is not None:
                self.q = self.joints_at(now)
            if rw == 0:
                return self.read(cmd, data, now)
            log(f"WRITE {NAMES.get(cmd, hex(cmd))} data={data.hex()}")
            return self.write(cmd, data, now)

    def read(self, cmd, data, now):
        comm = [self.a.servo_comm & 0xFF] * AXIS
        c = self.config
        q = self.joints_at(now)
        reads = {0x01: lambda: self.uuid, 0x02: lambda: self.sw, 0x03: lambda: self.hw,
                 0x04: lambda: bytes([AXIS]), 0x20: lambda: bytes([self.mode]),
                 0x21: lambda: struct.pack(">i", self.enable), 0x22: lambda: struct.pack(">i", self.brake),
                 0x23: lambda: bytes([self.err, self.warn]), 0x24: lambda: bytes(comm + [0] * AXIS),
                 0x25: lambda: bytes([self.status]), 0x26: lambda: struct.pack(">i", self.queued()),
                 0x40: lambda: struct.pack(">f", c["tcp_jerk"]), 0x41: lambda: struct.pack(">f", c["tcp_maxacc"]),
                 0x42: lambda: struct.pack(">f", c["joint_jerk"]), 0x43: lambda: struct.pack(">f", c["joint_maxacc"]),
                 0x44: lambda: struct.pack(">6f", *c["tcp_offset"]), 0x45: lambda: struct.pack(">4f", *c["payload"]),
                 0x46: lambda: struct.pack(">3f", *c["gravity"]), 0x47: lambda: bytes([c["collis_sens"]]),
                 0x48: lambda: bytes([c["teach_sens"]]),
                 0x50: lambda: struct.pack(">6f", *self.pose(self.target_with(q))),
                 0x51: lambda: struct.pack(f">{AXIS}f", *self.target_with(q)),
                 0x6E: lambda: self.gpio_reply(data, self.inputs), 0x6F: lambda: self.gpio_reply(data, self.outputs)}
        return reads[cmd]() if cmd in reads else None

    def target_with(self, q):
        saved, self.q = self.q, q
        try:
            return self.target()
        finally:
            self.q = saved

    def gpio_reply(self, data, bank):
        line = data[0] if data else 0
        if line not in bank:
            return None
        return bytes([0]) + struct.pack(">I", 0) + struct.pack(">I", bank[line]) + bytes([0])

    def write(self, cmd, data, now):
        if cmd == 0x0C:
            if not self.a.sticky_error:
                self.err = 0
            return b""
        if cmd == 0x20:
            # Joint teaching (freedrive) as the real arm did it on 2026-10-21 (tools/capture/teach_probe.py, logged in
            # data/captures/2026-10-21-tests/teach_probe.jsonl), which is Umbratek's documented order: motors on and
            # ready, then mode 2.
            mode = data[0]
            if mode == 2 and self.mode != 2:
                if self.current is not None or self.status in (1, 3):
                    log("teach mode refused while moving")
                elif self.status == 4:
                    log("teach mode ignored: the arm is stopped")  # as both of the server's first attempts found
                elif self.err:
                    self.mode = 2  # taken, but the arm holds: freedrive needs a clear error (seen with error 23)
                    log("motion mode 2 with an error: the arm holds")
                elif self.enable != FULL_MASK:
                    log("teach mode ignored: motors off")  # unverified: Umbratek's dashboard asks for all of them on
                else:
                    self.mode, self.status = 2, 1  # the real arm reports freedrive as moving (state 1)
                    log("motion mode 0 -> 2: freedrive")
            elif mode != 2 and self.mode == 2:
                # Leaving teach mode, whether free or stopped, switches the motors off onto the brakes.
                self.mode = mode
                self.motors_off(FULL_MASK, now)
                log(f"motion mode 2 -> {mode}: motors off")
            elif mode != self.mode:
                log(f"motion mode {self.mode} -> {mode}")
                self.mode = mode
            return b""
        if cmd == 0x21:
            if not self.a.enable_fails:
                axis, on = data[0], data[1]
                bits = 1 << (axis - 1) if 1 <= axis <= AXIS else FULL_MASK  # joints count from 1 (2026-10-21)
                if on:
                    was = self.enable
                    self.enable |= bits
                    self.q = [v + (math.radians(d) if bits >> j & 1 else 0.0)
                              for j, (v, d) in enumerate(zip(self.q, ENABLE_SHIFT_DEG))]
                    if self.enable & ~was:
                        log("motors on")
                else:
                    self.motors_off(bits, now)
            return b""
        if cmd == 0x22:
            return b""
        if cmd == 0x25:
            v = data[0]
            if v == 0 and not self.err:
                if self.mode == 2:
                    if self.status == 4 and self.enable == FULL_MASK:  # unverified on the arm: back to freedrive
                        self.status = 1
                        log("made ready in mode 2: freedrive again")
                elif self.status == 3 and self.current is not None:  # resume along the path
                    self.pause = ("up", now, self.progress(now))
                    self.status = 1
                    log("resumed")
                else:
                    self.status = 1 if self.current is not None else 0
                    self.ready_at = now
            elif v == 3 and self.status == 1 and self.current is not None:
                self.status, self.pause = 3, ("down", now, self.progress(now))
                log("pausing")
            elif v == 4 and self.a.ignore_stop:
                log("stop ignored (fault injection)")
            elif v == 4:
                self.halt(now)
                self.status, self.phantom_queued, self.hand = 4, 0, None  # stopped: the arm holds, even in freedrive
                self.queue.clear()
            return b""
        if cmd == 0x26:
            return b""
        if cmd in (0x30, 0x31):
            vals = struct.unpack(">10f" if cmd == 0x31 else ">9f", data)
            self.line_move(NAMES[cmd], list(vals[:6]), vals[6], vals[7])
            return b"\x00\x00\x00\x00"
        if cmd == 0x38:
            vals = struct.unpack(f">{AXIS + 3}f", data)
            self.joint_move("MOVEJ_P2P", list(vals[:AXIS]), vals[AXIS], vals[AXIS + 1])
            return b"\x00\x00\x00\x00"
        if cmd == 0x3A:
            vals = struct.unpack(">3f", data)
            self.joint_move("MOVEJ_HOME", [0.0] * AXIS, vals[0], vals[1])
            return b"\x00\x00\x00\x00"
        if cmd == 0x3B:
            seconds = struct.unpack(">f", data)[0]
            self.accept(SleepMotion("MOVE_SLEEP", self.q, seconds), [seconds], 0.0, 0.0)
            return b"\x00\x00\x00\x00"
        if cmd == 0x3D:
            count = struct.unpack(">I", data[:4])[0]
            vals = struct.unpack(f">{count * (AXIS + 1)}f", data[4:4 + 4 * count * (AXIS + 1)])
            frames = [(list(vals[k * (AXIS + 1):k * (AXIS + 1) + AXIS]), vals[k * (AXIS + 1) + AXIS]) for k in range(count)]
            self.accept(ServoMotion("MOVES_JOINT", self.q, frames), [math.degrees(v) for v in frames[-1][0]], 0.0, 0.0)
            return b"\x00\x00\x00\x00"
        floats = {0x40: ("tcp_jerk", 1), 0x41: ("tcp_maxacc", 1), 0x42: ("joint_jerk", 1), 0x43: ("joint_maxacc", 1),
                  0x44: ("tcp_offset", 6), 0x45: ("payload", 4), 0x46: ("gravity", 3)}
        if cmd in floats:
            name, n = floats[cmd]
            vals = list(struct.unpack(f">{n}f", data[:4 * n]))
            self.config[name] = vals[0] if n == 1 else vals
            if cmd == 0x45 and self.a.payload_stops and self.status != 4:  # --payload-stops: as the teach sensitivity
                self.halt(now)
                self.status, self.phantom_queued, self.hand = 4, 0, None
                self.queue.clear()
                log("payload set: stopped (--payload-stops)")
            if cmd == 0x45 and self.a.payload_error:  # --payload-error: a fault arriving as it's written
                self.fault(self.a.payload_error, "payload set (--payload-error)", now, transient=True)
            return b"\x00\x00\x00\x00" if n == 1 else b""
        if cmd in (0x47, 0x48):
            self.config["collis_sens" if cmd == 0x47 else "teach_sens"] = data[0]
            if cmd == 0x48 and self.status != 4:  # as on the arm (2026-10-21): the write stops it
                self.halt(now)
                self.status, self.phantom_queued, self.hand = 4, 0, None
                self.queue.clear()
                log("teach sensitivity set: stopped")
            return b""
        if cmd == 0x61:
            line, _, reg = data[0], data[1], data[2]
            value = struct.unpack(">I", data[3:7])[0]
            if line in self.outputs and reg == 0x13:
                mask, bits = value >> 16, value & 0xFFFF
                self.outputs[line] = (self.outputs[line] & ~mask) | (bits & mask)
                log(f"{'tool' if line == 2 else 'controller'} outputs -> {self.outputs[line]:016b}")
                return b"\x00"
            return None
        return None

    # -- the control port (the simulator's own; the real controller has nothing like it) --------------------------
    def control(self, request):
        """{"cmd": "state"} | {"cmd": "hand", "joints": [deg]*6, "seconds": s} | {"cmd": "input", "line": "tool" or
        "controller", "bit": 0-15, "value": 0 or 1} | {"cmd": "estop"} | {"cmd": "contact", "torque": x} |
        {"cmd": "fault", "err": code} | {"cmd": "nudge", "joints": [deg]*6} |
        {"cmd": "gripper", "output": "tool.N", "close": 1, "seconds": s, "closed": "tool.N"?, "opened": "tool.N"?,
         "part": "tool.N"?, "jammed": false} (a gripper on that output, its inputs following it; "remove": true takes
        it away) | {"cmd": "part", "present": true or false} (a part between the jaws, or not)"""
        with self.lock:
            now = self.now()
            what = request.get("cmd")
            if what == "state":
                q = self.joints_at(now)
                return {"ok": True, "joints": [math.degrees(v) for v in q], "pose": self.pose(q), "mode": self.mode,
                        "enable": self.enable, "status": self.status, "err": self.err, "queued": len(self.queue),
                        "moving": self.current is not None, "config": self.config,
                        "inputs": {"tool": self.inputs[2], "controller": self.inputs[3]},
                        "outputs": {"tool": self.outputs[2], "controller": self.outputs[3]},
                        "grippers": {f"{'tool' if line == 2 else 'controller'}.{bit}": {
                            "closing": g["closing"], "there": not g["jammed"] and now - g["since"] >= g["seconds"],
                            "jammed": g["jammed"]} for (line, bit), g in self.grippers.items()},
                        "part": self.part}
            if what == "gripper":
                line, bit = io_line(request["output"])
                if request.get("remove"):
                    self.grippers.pop((line, bit), None)
                    return {"ok": True}
                close, seconds = int(request.get("close", 1)), float(request.get("seconds", 0.3))
                self.grippers[(line, bit)] = {
                    "close": close, "seconds": seconds, "jammed": bool(request.get("jammed")),
                    "closed": io_line(request["closed"]) if request.get("closed") else None,
                    "opened": io_line(request["opened"]) if request.get("opened") else None,
                    "part": io_line(request["part"]) if request.get("part") else None,
                    "closing": ((self.outputs[line] >> bit) & 1) == close, "since": now - seconds}
                self._grippers(now)
                log(f"a gripper on {request['output']} ({vars_text(request)})")
                return {"ok": True}
            if what == "part":
                self.part = bool(request.get("present"))
                self._grippers(now)
                log(f"a part {'is' if self.part else 'is not'} between the jaws")
                return {"ok": True}
            if what == "hand":
                goal = [math.radians(v) for v in request["joints"]]
                if self.mode != 2 or self.status != 1 or self.enable != FULL_MASK or self.err:
                    return {"ok": False, "why": "not in freedrive (motion mode 2, reported moving, motors on, no error)"}
                clamped = [max(lo, min(hi, g)) for g, (lo, hi) in zip(goal, JOINT_LIMITS_RAD)]
                self.q = self.joints_at(now)
                self.hand = (now, list(self.q), clamped, float(request.get("seconds", 1.0)), clamped != goal)
                log(f"moved by hand towards [{' '.join(f'{math.degrees(g):.2f}' for g in clamped)}]"
                    + (" (to a joint limit)" if clamped != goal else ""))
                return {"ok": True}
            if what == "input":
                line = 2 if request.get("line") == "tool" else 3
                bit = int(request["bit"])
                self.inputs[line] = (self.inputs[line] | (1 << bit)) if request.get("value") else (self.inputs[line] & ~(1 << bit))
                log(f"{'tool' if line == 2 else 'controller'} input {bit} -> {1 if request.get('value') else 0}")
                return {"ok": True}
            if what == "estop":
                self.estop(now)
                return {"ok": True}
            if what == "contact":
                sens = self.config["collis_sens"]
                limit = CONTACT_THRESHOLDS[sens] if 0 <= sens < len(CONTACT_THRESHOLDS) else None
                if self.current is not None and self.status == 1 and limit is not None and request.get("torque", 0) >= limit:
                    self.fault(31, f"contact {request.get('torque')} past sensitivity {sens}'s {limit}", now, transient=True)
                    return {"ok": True, "tripped": True}
                log(f"contact {request.get('torque')} not detected (sensitivity {sens})")
                return {"ok": True, "tripped": False}
            if what == "fault":
                self.fault(int(request["err"]), "injected fault", now)
                return {"ok": True}
            if what == "nudge":  # the arm knocked off its path: every reported angle shifts, and stays shifted
                delta = [math.radians(float(v)) for v in request["joints"]]
                self.nudge = [a + b for a, b in zip(self.nudge, delta)]
                self.q = [a + b for a, b in zip(self.q, delta)]
                if self.current is not None:
                    self.current.start = [a + b for a, b in zip(self.current.start, delta)]
                    if isinstance(self.current, JointMotion):
                        self.current.goal = [a + b for a, b in zip(self.current.goal, delta)]
                    elif isinstance(self.current, LineMotion) and self.current.samples:
                        self.current.samples = [[a + b for a, b in zip(s, delta)] for s in self.current.samples]
                log(f"nudged by [{' '.join(f'{v:.2f}' for v in request['joints'])}] deg")
                return {"ok": True}
            return {"ok": False, "why": f"unknown command {what!r}"}

    def estop(self, now):
        self.q = self.joints_at(now)
        self.transient, self.hand = None, None
        self.motors_off(FULL_MASK)
        self.err, self.status, self.current, self.pause = 1, 4, None, None
        self.queue.clear()
        log("hardware E-stop pressed (fault injection)")


# ---- the network side ----------------------------------------------------------------------------------------

def serve_cmd(conn, arm):
    buf = b""
    with conn:
        while True:
            try:
                chunk = conn.recv(1024)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while True:
                i = buf.find(b"\xAA\x55")
                if i < 0:
                    buf = buf[-1:]
                    break
                buf = buf[i:]
                if len(buf) < 3 or len(buf) < (buf[2] & 0x7F) + 5:
                    break
                n = (buf[2] & 0x7F) + 5
                frame, buf = buf[:n], buf[n:]
                if frame[-2:] != crc16.crc_modbus(frame[:-2]):
                    log("bad CRC, dropped")
                    continue
                rw, cmd = frame[3] >> 7, frame[3] & 0x7F
                reply = arm.handle(rw, cmd, frame[4:-2])
                if reply is None:
                    log(f"unhandled rw={rw} cmd=0x{cmd:02x}")
                    continue
                head = bytes([0x55, 0xAA, ((1 if arm.err else 0) << 7) | (len(reply) + 1), (rw << 7) | cmd])
                try:
                    conn.sendall(head + reply + crc16.crc_modbus(head + reply))
                except OSError:
                    return


def serve_report(conn, arm):
    with arm.lock:
        arm.report_connections += 1
        if arm.report_connections == arm.a.drift_on_connect:
            arm.q[0] += math.radians(5)
            log(f"injected +5 deg drift on J1 at report connection {arm.report_connections}")
    next_at = clock()
    with conn:
        try:
            while True:
                if arm.a.stall_every:
                    phase = (arm.now() - arm.started) % arm.a.stall_every
                    if phase >= arm.a.stall_every - arm.a.stall_for:  # the controller's Wi-Fi stall
                        sleep(0.02)
                        next_at = clock()
                        continue
                with arm.lock:
                    now = arm.now()
                    q = arm.joints_at(now)
                    pose = arm.pose(q)
                    joints = [math.nan] * AXIS if arm.nan_joints else q
                    frame = struct.pack("<HBBBIIBBH", 41 + 8 * AXIS, AXIS, arm.status, arm.mode, arm.brake,
                                        arm.enable, arm.err, arm.warn, arm.queued())
                    frame += struct.pack(f"<{AXIS}f6f{AXIS}f", *joints, *pose, *arm.tau())
                conn.sendall(frame)
                next_at = next_tick(next_at, 0.1)
        except OSError:
            pass


def serve_config(conn, arm):
    next_at = clock()
    with conn:
        try:
            while True:
                with arm.lock:
                    c = arm.config
                    frame = struct.pack("<H19fBB", 80, c["tcp_maxacc"], c["tcp_jerk"], c["rot_maxacc"], c["rot_jerk"],
                                        c["joint_maxacc"], c["joint_jerk"], *c["tcp_offset"], *c["payload"],
                                        *c["gravity"], c["collis_sens"], c["teach_sens"])
                conn.sendall(frame)
                next_at = next_tick(next_at, 0.1)
        except OSError:
            pass


def serve_control(conn, arm):
    buf = b""
    with conn:
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        reply = arm.control(json.loads(line))
                    except Exception as e:  # a bad request; say so
                        reply = {"ok": False, "why": repr(e)}
                    conn.sendall(json.dumps(reply).encode() + b"\n")
        except OSError:
            pass


def bind(host, port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Exclusive on Windows, where SO_REUSEADDR would let a second simulator share the port and steal connections.
    srv.setsockopt(socket.SOL_SOCKET, getattr(socket, "SO_EXCLUSIVEADDRUSE", socket.SO_REUSEADDR), 1)
    srv.bind((host, port))
    srv.listen(8)
    return srv


def listen(srv, handler, arm, single=False, limit=0):
    port = srv.getsockname()[1]
    serving = None
    clients = []
    while True:
        conn, _ = srv.accept()
        if single and serving is not None and serving.is_alive():
            serving.join(0.2)  # a client that has just closed (a reachability probe, say) may still be finishing
        if single and serving is not None and serving.is_alive():
            # Like the real controller: one command connection at a time; any other is closed at once.
            log(f"closed a second connection on :{port}")
            conn.close()
            continue
        clients = [t for t in clients if t.is_alive()]
        if limit and len(clients) >= limit:
            # Like the real controller's status port, which serves only a few clients (2026-10-21).
            log(f"closed connection {len(clients) + 1} on :{port} (serving {limit} at most)")
            conn.close()
            continue
        serving = threading.Thread(target=handler, args=(conn, arm), daemon=True)
        serving.start()
        clients.append(serving)


def make_parser():
    p = argparse.ArgumentParser(description="A stand-in for the UTRA arm's NTRO controller.")
    p.add_argument("--host", default="127.0.0.1", help="loopback address to serve on (default 127.0.0.1)")
    p.add_argument("--joints", type=lambda s: [float(v) for v in s.split(",")], default=[0, -30, 50, -10, 90, 0],
                   help="starting joint angles, deg, comma-separated")
    p.add_argument("--joint-maxacc", type=float, default=200.0, help="configured joint max acceleration, deg/s^2 (200)")
    p.add_argument("--tcp-maxacc", type=float, default=200.0, help="configured tool max acceleration, mm/s^2 (200)")
    p.add_argument("--joint-jerk", type=float, default=math.degrees(24.0), help="configured joint jerk, deg/s^3")
    p.add_argument("--tcp-jerk", type=float, default=2000.0, help="configured tool jerk, mm/s^3 (2000)")
    p.add_argument("--collision-sens", type=int, default=0, help="collision sensitivity at start, 0-5 (0, as captured)")
    p.add_argument("--teach-sens", type=int, default=0, help="teach sensitivity at start (0, as the controller boots)")
    p.add_argument("--multi-client", action="store_true",
                   help="serve several command connections at once (the real controller serves one)")
    p.add_argument("--no-report", action="store_true", help="do not serve port 30001")
    p.add_argument("--report-clients", type=int, default=0,
                   help="serve at most this many status (30001) clients at once and close any more (0: no limit)")
    faults = p.add_argument_group("faults to inject")
    faults.add_argument("--error", type=int, default=0, help="initial controller error code")
    faults.add_argument("--sticky-error", action="store_true", help="RESET_ERR does not clear the error")
    faults.add_argument("--target-offset", type=float, default=0.0, help="J1 target-vs-measured offset, always (deg)")
    faults.add_argument("--stale-target", type=float, default=0.0, help="J1 target offset only while disabled (deg)")
    faults.add_argument("--enable-jump", type=float, default=0.0, help="J1 target offset only while enabled (deg)")
    faults.add_argument("--enable-fails", action="store_true", help="enable command has no effect")
    faults.add_argument("--servo-comm", type=int, default=0, help="servo comm code reported for every joint")
    faults.add_argument("--queued", type=int, default=0, help="initial phantom queued command count")
    faults.add_argument("--ignore-moves", action="store_true", help="discard motion commands")
    faults.add_argument("--ik-fail", action="store_true", help="reject tool moves with error 21")
    faults.add_argument("--pause-after", type=float, default=0.0, help="pause N s into a move")
    faults.add_argument("--error-after", type=float, default=0.0, help="raise error 9 N s into a move")
    faults.add_argument("--drift-on-connect", type=int, default=0, help="move J1 +5 deg when the Nth report connection opens")
    faults.add_argument("--nan-after-move", action="store_true", help="report NaN joints once a move completes")
    faults.add_argument("--ignore-stop", action="store_true", help="ignore MOTION_STATUS=4 (stop) writes")
    faults.add_argument("--estop-after", type=float, default=0.0, help="press the hardware E-stop N s after start")
    faults.add_argument("--stall-every", type=float, default=0.0, help="pause the status report every N s...")
    faults.add_argument("--stall-for", type=float, default=3.1, help="...for this long, like the real controller over Wi-Fi")
    faults.add_argument("--payload-stops", action="store_true",
                        help="writing the payload stops the arm, as writing the teach sensitivity does (unverified on "
                             "the arm for the payload)")
    faults.add_argument("--payload-error", type=int, default=0,
                        help="writing the payload also raises this controller error, as a collision tripping during a "
                             "grasp would")
    return p


def serve(a, now=None):
    """Binds the simulator's ports on a.host (OSError if one is taken), serves them from daemon threads and returns
    the Arm. Something must then advance it with arm.tick(), as run() does. `now` is the arm's time source (the
    high-resolution clock unless given)."""
    arm = Arm(a, now)
    ports = [(502, serve_cmd, not a.multi_client, 0)]
    if not a.no_report:
        ports.append((30001, serve_report, False, a.report_clients))
    ports += [(30003, serve_config, False, 0), (CONTROL_PORT, serve_control, False, 0)]
    bound = [(bind(a.host, port), handler, single, limit) for port, handler, single, limit in ports]
    for srv, handler, single, limit in bound:
        threading.Thread(target=listen, args=(srv, handler, arm, single, limit), daemon=True).start()
    log(f"listening on {a.host}:502{'' if a.no_report else ', :30001'}, :30003 and control :{CONTROL_PORT} ({vars(a)})")
    return arm


def run(arm, a):
    """Advances the arm every 5 ms, for ever."""
    started = last = arm.now()
    while True:
        sleep(0.005)
        now = arm.now()
        arm.tick(now, now - last)
        last = now
        if a.estop_after and now - started > a.estop_after and arm.err != 1:
            with arm.lock:
                arm.estop(now)
            a.estop_after = 0.0


def main(argv=None):
    a = make_parser().parse_args(argv)
    run(serve(a), a)


if __name__ == "__main__":
    main()
