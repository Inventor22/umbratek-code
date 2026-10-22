"""The safety shell for what the brain proposes: a path for a planned move, or a burst of a learned skill, is checked
whole before anything moves, and refused with every reason found if any part of it fails, so a refused proposal is
never partly run. What passes becomes an ordinary plan for the supervised executor (executor.py), which watches it run
as it watches every move, and which a STOP or pause interrupts as any other.

The checks, beyond the protocol's own (brain_protocol.py: this version, the reply to this very request and state,
every number finite, the waypoints counted and each six joints):
- where it starts and ends: within GOAL_TOLERANCE_DEG of the joints the spine sent, and for a planned move of the goal
  the spine set; the plan then runs from and to exactly those;
- joint limits: every waypoint within the planning limits (planner.DRAG_LIMITS_DEG), so every point between is too;
- collisions: each segment, the straight joint-space move the controller makes between two waypoints, through the
  continuous check every move gets (utra_model.ArmModel.check_path: the arm itself, its cart, the floor, the tool and
  the cell's keep-out zones, with the 10 mm margin), which steps along it so finely that no gap can close unseen;
- speed caps: each segment at the step's speed (or the brain's, if lower), each joint within its rated speed, and the
  tool point within a cap (planner.Planner.joint; the runner passes 250 mm/s unless the program is marked for full
  speed);
- for a learned skill's burst: short (at most BURST_MAX_JOINT_DEG of turn for any joint, BURST_MAX_TOOL_MM of tool
  point travel) and inside the step's region (the tool point within `within` mm of its `near` target all the way,
  sampled every SAMPLE_DEG of turn);
- just before it runs (check_still): the controller's status fresh, the arm still where the brain was told it was
  (within STILL_DEG), and the cell unchanged since the brain was asked.
Standard library only.
"""
from __future__ import annotations

import math

import kinematics as kin
import planner as plannermod
from executor import FRESH_S

GOAL_TOLERANCE_DEG = 0.01  # a proposal starts and ends where it was asked to, but for rounding
STILL_DEG = 0.1  # the arm holds within 0.005 deg at rest (capture), so more than this means it moved
BURST_MAX_JOINT_DEG = 15.0
BURST_MAX_TOOL_MM = 50.0
SAMPLE_DEG = 0.1
DUPLICATE_DEG = 1e-6
MAX_REASONS = 6


class Refused(Exception):
    """The brain's proposal (`what`: "plan", "burst 3", "reply") failed the shell's checks and nothing of it was sent.
    `reasons` says every way it failed; `hit` is where it would have collided first, for the 3D view."""

    def __init__(self, what, reasons, hit=None):
        self.what, self.reasons, self.hit = what, list(reasons), hit
        shown = self.reasons[:MAX_REASONS]
        if len(self.reasons) > MAX_REASONS:
            shown.append(f"and {len(self.reasons) - MAX_REASONS} more")
        super().__init__(f"refused the brain's {what}: {'; '.join(shown)}")


def _worst(a, b):
    """(the largest difference in degrees between two joint sets, and its joint's index)."""
    gaps = [abs(x - y) for x, y in zip(a, b)]
    j = max(range(len(gaps)), key=gaps.__getitem__)
    return gaps[j], j


def _plain(error):
    text = str(error)
    return text[len("Not moving: "):] if text.startswith("Not moving: ") else text


def check_path(planner, path, start, speed, tool, tool_speed, label, what="plan", goal=None, near=None, within=None,
               burst=False):
    """A plan for the brain's `path` (waypoints in degrees, the first where the arm is) that the executor can run, or
    Refused with every reason it can't be.

    planner: the cell's planner (the arm with the active tool's shape and the keep-out zones); start: the joints the
    spine sent the brain; speed: the step's joint speed (deg/s, already lowered to the brain's, if it asked for less);
    tool: flange <- tool point (4x4); tool_speed: the tool point's cap (mm/s); goal: the joints a planned move must
    end at; near: (the region's centre, mm in the base frame, and its name) and within: its radius (mm), for a learned
    skill's burst, whose shortness is checked when `burst` is set."""
    reasons = []
    start = [float(v) for v in start]
    gap, j = _worst(path[0], start)
    if gap > GOAL_TOLERANCE_DEG:
        reasons.append(f"it starts {gap:.2f}\u00b0 (J{j + 1}) from where the arm is")
    if goal is not None:
        gap, j = _worst(path[-1], goal)
        if gap > GOAL_TOLERANCE_DEG:
            reasons.append(f"it ends {gap:.2f}\u00b0 (J{j + 1}) from {label}")
    for k, q in enumerate(path):
        for i, v in enumerate(q):
            low, high = plannermod.DRAG_LIMITS_DEG[i]
            if not low <= v <= high:
                reasons.append(f"waypoint {k} has J{i + 1} at {v:.1f}\u00b0, outside {low:g}\u00b0 to {high:g}\u00b0")
    if burst:
        for i in range(6):
            turn = sum(abs(b[i] - a[i]) for a, b in zip(path, path[1:]))
            if turn > BURST_MAX_JOINT_DEG:
                reasons.append(f"it turns J{i + 1} {turn:.1f}\u00b0, more than the {BURST_MAX_JOINT_DEG:g}\u00b0 a "
                               "burst may")
    if reasons:
        raise Refused(what, reasons)

    # The plan runs from exactly where the arm was and to exactly the goal; repeated waypoints are dropped.
    points = [start] + [[float(v) for v in q] for q in path[1:-1]] + [list(goal) if goal is not None else
                                                                     [float(v) for v in path[-1]]]
    kept = [points[0]]
    for q in points[1:]:
        if _worst(q, kept[-1])[0] > DUPLICATE_DEG:
            kept.append(q)
    hit = None
    plans = []
    for k, (a, b) in enumerate(zip(kept, kept[1:]), 1):
        try:
            plans.append(planner.joint(a, b, speed, label, tool=tool, tool_speed=tool_speed))
        except plannermod.PlanError as e:
            reasons.append(f"segment {k}: {_plain(e).rstrip('.')}")
            hit = hit or e.hit
    if burst or near is not None:
        travel = 0.0
        for k, (a, b) in enumerate(zip(kept, kept[1:]), 1):
            points_mm = _tool_points(planner, a, b, tool)
            chords = [math.dist(p, q) for p, q in zip(points_mm, points_mm[1:])]
            travel += sum(chords)
            if near is not None:
                centre, name = near
                # Between two samples the tool point is within half their chord of one of them.
                far = max(math.dist(p, centre) + c / 2 for p, c in zip(points_mm, chords + [0.0]))
                if far > within:
                    reasons.append(f"segment {k} takes the tool point {far:.0f} mm from {name}, beyond the {within:g} mm "
                                   "the step allows")
        if burst and travel > BURST_MAX_TOOL_MM:
            reasons.append(f"it moves the tool point {travel:.0f} mm, more than the {BURST_MAX_TOOL_MM:g} mm a burst may")
    if reasons:
        raise Refused(what, reasons, hit)

    segments = [p.segments[0] for p in plans]
    durations = [p.duration for p in plans]
    total = sum(durations)
    if not segments:
        text = f"Already at {label}"
    else:
        top = max(s[1] for s in segments)
        why = next((f", {p.limited}" for p in plans if p.limited), "")
        count = f"{len(segments)} segment{'s' if len(segments) > 1 else ''}"
        text = f"{'Burst' if burst else 'Path'} to {label} from the brain, {count} at up to {top:g}\u00b0/s{why}, " \
               f"about {total:.1f} s"
    return plannermod.Plan("joint", kept[0], kept[-1], kept, segments, total, text, tool,
                           segment_durations=durations)


def _tool_points(planner, a, b, tool):
    """The tool point (mm, base frame) along the joint move a -> b, no joint turning more than SAMPLE_DEG between."""
    lead = max(abs(y - x) for x, y in zip(a, b))
    n = max(1, int(math.ceil(lead / SAMPLE_DEG)))
    return [kin.position(planner.kin.tool_pose([x + (y - x) * k / n for x, y in zip(a, b)], tool)) for k in range(n + 1)]


def check_still(what, observed, latest, latest_age, cell_then, cell_now):
    """Refused unless, just before a proposal runs, the controller's status is fresh (`latest`: the measured joints,
    degrees, `latest_age` s old), the arm is still where the brain was told it was (`observed`), and the cell's
    revision is the one the brain was told."""
    reasons = []
    if latest is None or latest_age > FRESH_S:
        reasons.append("the controller's status is stale" + (f" ({latest_age:.1f} s old)" if latest is not None else ""))
    else:
        gap, j = _worst(latest, observed)
        if gap > STILL_DEG:
            reasons.append(f"the arm has moved {gap:.2f}\u00b0 (J{j + 1}) since the brain was asked")
    if cell_then != cell_now:
        reasons.append(f"the cell changed (revision {cell_then} to {cell_now}) while the brain was working")
    if reasons:
        raise Refused(what, reasons)
