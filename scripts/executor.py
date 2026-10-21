"""Runs a planned move on the arm and follows it until it arrives, stopping the arm if it doesn't go as planned.

Each segment of a plan is sent alone and followed on the 10 Hz status: the arm must stay near its planned path (within
PATH_TOLERANCE_DEG of the joint-space line for joint moves, PATH_TOLERANCE_MM of the tool point's line for straight
lines), arrive (state ready and within ARRIVAL_DEG of the goal, or ARRIVAL_MM of the tool point's goal), and do so
within the plan's estimate x 1.5 + 2 s. A controller error, an unexpected stop or pause, status older than
LOST_S, or a request to abort (pause or stop) stops the arm. Status older than FRESH_S holds judgement until it's
fresh again (the Wi-Fi link stalls for about 3 s now and then). The controller's frame queue count isn't trusted: the
capture's frames always read 0. Standard library only.
"""
from __future__ import annotations

import math
import time

import kinematics as kin

PATH_TOLERANCE_DEG = 1.0
PATH_TOLERANCE_MM = 2.0
ARRIVAL_DEG = 0.1
# The real arm settles within 0.05 deg per joint, which leaves the tool up to 0.47 mm and 0.13 deg from where a line
# was sent (capture 2026-10-21: median 0.21 mm over 24 lines; joint moves up to 0.44 mm and 0.10 deg). About twice that.
ARRIVAL_MM, ARRIVAL_TURN_DEG = 1.0, 0.3
START_TOLERANCE_DEG = 0.5  # the arm must be this close to where the plan starts
FRESH_S = 1.0
LOST_S = 4.5
NOT_STARTED_S = 1.5  # a move that hasn't stirred the arm by now was ignored
POLL_S = 0.05
OK_CODES = (0, -4)


class MoveFailed(Exception):
    """The move didn't go as planned; the arm has been stopped (stopped is False if the stop wasn't confirmed)."""

    def __init__(self, reason, stopped=True):
        super().__init__(reason)
        self.reason, self.stopped = reason, stopped


class Interrupted(Exception):
    """A pause or stop was asked for mid-move; the arm has been stopped."""

    def __init__(self, reason, stopped=True):
        super().__init__(reason)
        self.reason, self.stopped = reason, stopped


def joint_deviation(q, a, g):
    """How far (deg, worst joint) joints q are from the joint-space segment a -> g."""
    d = [y - x for x, y in zip(a, g)]
    dd = sum(v * v for v in d)
    s = 0.0 if dd < 1e-12 else max(0.0, min(1.0, sum((qi - ai) * di for qi, ai, di in zip(q, a, d)) / dd))
    return max(abs(qi - (ai + s * di)) for qi, ai, di in zip(q, a, d))


def line_deviation(p, a, b):
    """Distance (mm) from point p to the segment a -> b."""
    d = [y - x for x, y in zip(a, b)]
    dd = sum(v * v for v in d)
    s = 0.0 if dd < 1e-12 else max(0.0, min(1.0, sum((pi - ai) * di for pi, ai, di in zip(p, a, d)) / dd))
    return math.dist(p, [ai + s * di for ai, di in zip(a, d)])


class Executor:
    """Sends plans over `panel`'s SDK connection (one conversation at a time, under panel.lock) and follows them on
    panel's live status. `panel` provides: arm (the SDK), lock, frame and frame_at (the latest status frame and when
    it came), mask (all joints' enable bits), stop(cause) -> (confirmed, text), and model (for the tool point)."""

    def __init__(self, panel, sleep=time.sleep):
        self.panel = panel
        self.sleep = sleep
        self.current = None  # (plan, segment index) while running, for the status

    def _frame(self):
        frame, at = self.panel.frame, self.panel.frame_at
        return frame, (time.monotonic() - at) if frame is not None else math.inf

    def _stop(self, reason, error=MoveFailed):
        confirmed, _ = self.panel.stop(reason)
        return error(reason, stopped=confirmed)

    def _fresh(self, timeout=4.0):
        deadline = time.monotonic() + timeout
        while True:
            frame, age = self._frame()
            if frame is not None and age <= FRESH_S:
                return frame
            if time.monotonic() > deadline:
                raise MoveFailed("No live status from the controller, so not moving.", stopped=True)
            self.sleep(POLL_S)

    def check_ready(self, plan):
        frame = self._fresh()
        if frame["err"]:
            raise MoveFailed(f"The controller reports error {frame['err']}; not moving.")
        if frame["enable"] & self.panel.mask != self.panel.mask:
            raise MoveFailed("The motors are off; not moving.")
        if frame["status"] not in (0, 2):
            raise MoveFailed(f"The arm isn't ready (state {frame['status']}); not moving.")
        q = [math.degrees(v) for v in frame["joints"][:6]]
        gap = max(abs(a - b) for a, b in zip(q, plan.start))
        if gap > START_TOLERANCE_DEG:
            raise MoveFailed(f"The arm is {gap:.1f}\u00b0 from where the plan starts; plan it again.", stopped=True)
        return frame

    def execute(self, plan, should_abort=lambda: None, on_event=lambda kind, **detail: None):
        """Run `plan`. Returns when the arm has arrived. Raises MoveFailed or Interrupted, having stopped the arm."""
        if not plan.segments:
            return
        self.check_ready(plan)
        try:
            self._execute(plan, should_abort, on_event)
        finally:
            self.current = None

    def _execute(self, plan, should_abort, on_event):
        if plan.kind == "line":
            t0, target = plan.poses
            line_a, line_b = kin.position(t0), kin.position(target)
        k = kin.Kinematics(self.panel.model)
        n = len(plan.segments)
        for i, segment in enumerate(plan.segments):
            self.current = (plan, i)
            if plan.kind == "joint":
                goal, speed, acc = segment
                start = plan.start
                send = lambda: self.panel.arm.moveto_joint_p2p([math.radians(v) for v in goal], math.radians(speed),
                                                               math.radians(acc), 0)
                seg_goal_pose = None
            else:
                flange, speed, acc = segment
                start = None
                send = lambda: self.panel.arm.moveto_cartesian_line(flange, speed, acc, 0)
                seg_goal_pose = kin.mul(kin.from_controller_pose(flange), plan.tool)
            with self.panel.lock:
                ret = send()
            sent = time.monotonic()
            if ret not in OK_CODES:
                raise self._stop(f"the controller rejected the move (code {ret})")
            on_event("move.sent", segment=i + 1, of=n, speed=speed)
            budget = plan.duration * 1.5 + 2.0
            seen_moving, first = False, None
            while True:
                self.sleep(POLL_S)
                why = should_abort()
                if why:
                    raise self._stop(why, Interrupted)
                frame, age = self._frame()
                now = time.monotonic()
                if age > LOST_S:
                    raise self._stop("the controller's status went quiet")
                if age > FRESH_S:
                    continue
                if frame["err"]:
                    raise self._stop(f"the controller reported error {frame['err']}")
                if frame["enable"] & self.panel.mask != self.panel.mask or frame["status"] in (3, 4):
                    state = {3: "paused", 4: "stopped"}.get(frame["status"], "switched off")
                    raise self._stop(f"the arm {state} on its own (STOP, E-stop or a fault)")
                q = [math.degrees(v) for v in frame["joints"][:6]]
                if not all(math.isfinite(v) for v in q):
                    raise self._stop("the controller reported unusable joint angles")
                if first is None:
                    first = q
                if frame["status"] == 1:
                    seen_moving = True
                if self.panel.frame_at > sent + 0.1:  # a frame from after the command
                    if plan.kind == "joint":
                        dev = joint_deviation(q, start, goal)
                        if dev > PATH_TOLERANCE_DEG:
                            raise self._stop(f"the arm left its path by {dev:.1f}\u00b0")
                    else:
                        tcp = kin.position(k.tool_pose(q, plan.tool))
                        dev = line_deviation(tcp, line_a, line_b)
                        if dev > PATH_TOLERANCE_MM:
                            raise self._stop(f"the tool left its line by {dev:.1f} mm")
                if plan.kind == "joint":
                    arrived = max(abs(a - b) for a, b in zip(q, goal)) <= ARRIVAL_DEG
                else:
                    gap_mm, gap_deg = kin.pose_distance(k.tool_pose(q, plan.tool), seg_goal_pose)
                    arrived = gap_mm <= ARRIVAL_MM and gap_deg <= ARRIVAL_TURN_DEG
                if arrived and frame["status"] in (0, 2) and (seen_moving or now - sent > 0.6):
                    on_event("move.arrived", segment=i + 1, of=n, seconds=round(now - sent, 2))
                    break
                if not seen_moving and now - sent > NOT_STARTED_S and max(abs(a - b) for a, b in zip(q, first)) < 0.01:
                    raise self._stop("the controller didn't start the move")
                if now - sent > budget:
                    raise self._stop(f"the move didn't arrive within {budget:.1f} s")
