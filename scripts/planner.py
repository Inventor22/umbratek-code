"""Plans a move before anything is sent: the joints' path, whether it's clear, how fast, and how long it takes.

Joint moves go straight in joint space, as the controller's point-to-point moves do. Straight-line moves keep the tool
point on a line, its orientation turning about one axis: the planner solves inverse kinematics every LINE_STEP_MM and
LINE_STEP_DEG from the start's joints, and refuses a line that leaves the reachable space, flips the arm's
configuration on the way (a joint jumping more than FLIP_DEG between samples), passes so near a singularity that a
joint would exceed its rated speed, or brings any part of the arm or tool within the collision margin of anything
(the continuous check between samples). The controller then runs it as its own straight line of the flange (measured
straight within 0.21 mm over 100 mm); the tool point is applied here, so a line that also turns is split into
segments short enough to keep the tool point within SEGMENT_TOLERANCE_MM of its line (docs/decisions.md D14).
Durations come from the jerk-limited profile the capture showed the controller following. Standard library only.
"""
from __future__ import annotations

import math

import kinematics as kin
import motion_profile

# Joints' limits for planned moves: Umbratek's xacro limits J3 to +/-165 deg and lets the others turn about +/-359 deg;
# plans stay within +/-180 deg.
DRAG_LIMITS_DEG = ((-180.0, 180.0), (-180.0, 180.0), (-165.0, 165.0),
                   (-180.0, 180.0), (-180.0, 180.0), (-180.0, 180.0))
# Each joint's top speed, from its own servo (see jog_server.py): 80% of the servos' limits, rounded down to 0.1.
SERVO_SPEED_LIMITS = (220.0, 220.0, 250.0, 260.0, 260.0, 260.0)  # rad/s at the motor
GEAR_RATIOS = (120.0, 120.0, 100.0, 100.0, 100.0, 100.0)
SPEED_HEADROOM = 0.8
JOINT_RATED_DEG_S = tuple(math.floor(math.degrees(SPEED_HEADROOM * v / g) * 10) / 10
                          for v, g in zip(SERVO_SPEED_LIMITS, GEAR_RATIOS))
# The controller's configured limits on this arm (data/captures/2026-10-21): used when it won't say.
JOINT_ACC_DEG_S2, JOINT_JERK_DEG_S3 = 200.0, math.degrees(24.0)
TOOL_ACC_MM_S2, TOOL_JERK_MM_S3 = 200.0, 2000.0
START_DELAY_JOINT_S, START_DELAY_LINE_S = 0.020, 0.010  # tools/capture/fit_sim.py
LINE_STEP_MM, LINE_STEP_DEG = 2.0, 1.0
FLIP_DEG = 5.0
SEGMENT_TOLERANCE_MM = 0.1
MAX_TOOL_SPEED_MM_S = 1000.0
MAX_ORI_SPEED_DEG_S = 180.0
TOOL_SPEED_SAMPLES = 16  # points along a joint move at which the tool point's speed is bounded (and both ends)
# Reduced speed: ISO 10218-1's limit for moving the arm by hand, 250 mm/s at the tool. With the server's manual-motion
# policy asking for it (jog_server.py --reduced-speed, or POST /api/manual), every move is sent slowly enough that
# neither the flange nor the tool point in use can pass it (docs/decisions.md D101 and D102).
REDUCED_SPEED_MM_S = 250.0
REDUCED_WHY = f", keeping the tool point and flange under {REDUCED_SPEED_MM_S:g} mm/s (reduced speed)"
REDUCED_STEP_DEG = 1.0  # a joint move's bound is taken where no joint turns more than this between samples
# How fast the controller may turn the tool on a straight move, for the bound: 1 rad/s for every this many mm/s of the
# speed sent. The simulator assumes 100 (unverified, D11); the arm's turns fitted 50 (D54, four turns of one size), so
# this allows for the controller turning twice as fast as measured.
REDUCED_TURN_MM_PER_RAD = 25.0
# Without the arm model the lever arms aren't known: every joint is taken to swing the flange this far from its axis,
# the links from the shoulder to the flange end to end (1145 mm), rounded up.
REDUCED_REACH_MM = 1150.0


class PlanError(Exception):
    """Why a move can't be planned; `hit` is utra_model's description of the collision, when that's why. For a straight
    line, `clear` is the fraction of it that could be planned before the trouble (0 to 1), when that's known."""

    def __init__(self, message, hit=None, clear=None):
        super().__init__(message)
        self.hit = hit
        self.clear = clear


class ZoneClosed(PlanError):
    """A move refused because a guarded zone is closed, its interlocks not all holding: `zone` names it. `inside` is set
    when the arm is in the zone already, and the move would stay in it or go deeper."""

    def __init__(self, message, zone, hit=None, clear=None, inside=False):
        super().__init__(message, hit, clear)
        self.zone, self.inside = zone, inside


def capped_joint_speed(speed, travel):
    """The speed (deg/s) to send a joint move at, and the joint that limits it (or None).

    A point-to-point move turns every joint in step, the one with the furthest to go at the move's speed and the
    others proportionally slower; the move is slowed until no joint would exceed its rated speed.
    """
    longest = max(travel)
    sent, limiting = speed, None
    for i, t in enumerate(travel):
        if t > 1e-6:
            rated = JOINT_RATED_DEG_S[i] if i < len(JOINT_RATED_DEG_S) else min(JOINT_RATED_DEG_S)
            cap = math.floor(rated * longest / t * 10) / 10
            if cap < sent:
                sent, limiting = cap, i
    return sent, limiting


def lever_bound(model, start, goal, tools=()):
    """An upper bound (mm per rad of the leading joint) on how fast the flange, or the point of any of `tools` (each
    flange <- point, 4x4), moves anywhere on the joint move start -> goal (deg), every joint turning in step as the
    controller's point-to-point moves turn them.

    It's the sum of each joint's share of the turn times the point's distance from that joint's axis (joints turning
    together add up), taken where no joint turns more than REDUCED_STEP_DEG from one sample to the next, plus the most
    that sum can grow between samples: a point's distance from a joint's axis changes no faster than the joints beyond
    that joint turn, times the point's reach from them, which is at most the links from the shoulder to the flange, end
    to end, and the point's offset. Without a model every joint is taken to swing the points REDUCED_REACH_MM and the
    tool's offset from its axis."""
    travel = [g - s for s, g in zip(start, goal)]
    lead = max((abs(d) for d in travel), default=0.0)
    if lead <= 0:
        return 0.0
    shares = [abs(d) / lead for d in travel]
    points = [kin.IDENTITY] + [t for t in tools if t is not None]
    offset = max(math.sqrt(t[0][3] ** 2 + t[1][3] ** 2 + t[2][3] ** 2) for t in points)  # m
    if model is None:
        return sum(shares) * (REDUCED_REACH_MM / 1000.0 + offset) * 1000.0
    n = max(1, int(math.ceil(lead / REDUCED_STEP_DEG)))
    worst = 0.0
    for k in range(n + 1):
        q = [s + d * k / n for s, d in zip(start, travel)]
        frames = model.frames(q)
        axes = [([f[r][0] * a[0] + f[r][1] * a[1] + f[r][2] * a[2] for r in range(3)], [f[r][3] for r in range(3)])
                for (_, _, a, _), f in zip(model.joints, frames)]
        for point in points:
            t = kin.mul(frames[-1], point)
            total = 0.0
            for (z, o), share in zip(axes, shares):
                if share > 0:
                    arm = [t[r][3] - o[r] for r in range(3)]
                    along = arm[0] * z[0] + arm[1] * z[1] + arm[2] * z[2]
                    total += share * math.sqrt(max(arm[0] ** 2 + arm[1] ** 2 + arm[2] ** 2 - along * along, 0.0))
            worst = max(worst, total)
    reach = sum(math.sqrt(f[0][3] ** 2 + f[1][3] ** 2 + f[2][3] ** 2) for _, f, _, _ in model.joints[2:]) + offset
    beyond = [sum(abs(d) for d in travel[j + 1:]) for j in range(len(travel))]  # deg the joints past each one turn
    growth = sum(share * turn for share, turn in zip(shares, beyond)) * math.radians(1.0) * reach
    return (worst + growth / (2 * n)) * 1000.0


def reduced_joint_speed(model, start, goal, tools=(), limit=REDUCED_SPEED_MM_S):
    """The fastest the leading joint may turn (deg/s, rounded down to 0.1, at least 0.1) on the joint move start -> goal
    for neither the flange nor any of `tools`' points to pass `limit` mm/s (lever_bound); None if they don't move (by
    less than a micron per radian of the turn, a point on the turning axis, say)."""
    per = lever_bound(model, start, goal, tools)
    if per <= 1e-3:
        return None
    return max(0.1, math.floor(math.degrees(limit / per) * 10) / 10)


def reduced_line_speed(length_mm, turn_deg, reach_mm, limit=REDUCED_SPEED_MM_S):
    """The fastest (mm/s, rounded down to 0.1, at least 0.1) to send a straight move of the flange, length_mm long and
    turning turn_deg, for neither the flange nor a point up to reach_mm from it to pass `limit` mm/s.

    The controller drives the flange along its line at the speed sent or slower, and its turn keeps pace with the line
    (as the simulator's does), at no more than 1 rad/s for every REDUCED_TURN_MM_PER_RAD mm/s (assumed, D102). So the
    move takes at least max(length, turn x REDUCED_TURN_MM_PER_RAD) / speed, while a point reach_mm out travels at most
    length + turn x reach_mm."""
    turn = math.radians(abs(turn_deg))
    need = abs(length_mm) + turn * reach_mm
    pace = max(abs(length_mm), turn * REDUCED_TURN_MM_PER_RAD)
    if need <= pace * (1.0 + 1e-6):  # the limit itself (within a millionth: a turn worked out from poses has noise)
        return limit
    return max(0.1, min(limit, math.floor(limit * pace / need * 10) / 10))


def blocked(hit, margin_mm=10):
    if hit.get("inside"):
        return f"{hit['part']} would go further into {hit['other']}"
    if hit.get("gap", 0.0) < 0.0:  # from within the margin, past what it may come closer (ArmModel._close_limit)
        return f"{hit['part']} would go into {hit['other']}"
    return f"{hit['part']} would come within {margin_mm:.0f} mm of {hit['other']}"


class Plan:
    """What a move will do. kind is "joint" or "line"; segments are what's sent: for joint moves one or more (goal
    joints, speed deg/s, acc deg/s^2), each from the one before (a path from the brain has several), for lines one or
    more (flange pose for the controller, speed mm/s, acc mm/s^2). path is the joints along the way (deg), for
    supervision; duration is the estimate in seconds, and segment_durations each segment's, when they're known."""

    def __init__(self, kind, start, goal, path, segments, duration, text, tool=kin.IDENTITY, poses=None,
                 segment_durations=None):
        self.kind, self.start, self.goal, self.path = kind, start, goal, path
        self.segments, self.duration, self.text, self.tool = segments, duration, text, tool
        self.poses = poses  # for lines: the tool point's pose at the start and the end (4x4)
        self.segment_durations = segment_durations

    def summary(self):
        return {"kind": self.kind, "start": [round(v, 3) for v in self.start], "goal": [round(v, 3) for v in self.goal],
                "segments": len(self.segments), "durationS": round(self.duration, 2), "text": self.text,
                "path": [[round(v, 2) for v in q] for q in thin(self.path, 60)]}


def thin(path, count):
    if len(path) <= count:
        return path
    step = (len(path) - 1) / (count - 1)
    return [path[round(i * step)] for i in range(count)]


class Planner:
    """Plans moves for one arm model (with the cell's tool and zones in it) and its kinematics. With `reduced` (the tool
    in use, flange <- its point, 4x4), reduced speed is on: every move it plans keeps that point and the flange under
    REDUCED_SPEED_MM_S."""

    def __init__(self, model, joint_acc=None, tool_acc=None, joint_jerk=None, tool_jerk=None, reduced=None):
        self.model = model
        self.kin = kin.Kinematics(model)
        self.joint_acc = joint_acc or JOINT_ACC_DEG_S2
        self.tool_acc = tool_acc or TOOL_ACC_MM_S2
        self.joint_jerk = joint_jerk or JOINT_JERK_DEG_S3
        self.tool_jerk = tool_jerk or TOOL_JERK_MM_S3
        self.reduced = reduced

    # -- joint moves ---------------------------------------------------------------------------------------------
    def joint(self, start, goal, speed, label="the joints", tool=None, tool_speed=None, limits=None):
        """A joint move. With `tool_speed` (mm/s) it's also slowed until the tool point (`tool`: flange <- tool point)
        can't pass that speed anywhere on the way (tool_speed_per_deg_s); with reduced speed on, until neither the tool
        point nor the flange can pass REDUCED_SPEED_MM_S (reduced_joint_speed). `limits`: when it's one segment of a
        longer path, the path's (utra_model.ArmModel.start_limits at its start)."""
        start, goal = [float(v) for v in start], [float(v) for v in goal]
        for i, g in enumerate(goal):
            low, high = DRAG_LIMITS_DEG[i]
            if not low <= g <= high:
                raise PlanError(f"J{i + 1} would have to go to {g:.1f}\u00b0, outside {low:g}\u00b0 to {high:g}\u00b0.")
        hit = self.model.check_path(start, goal, limits=limits)
        if hit is not None:
            raise PlanError(f"Not moving: {blocked(hit)}.", hit)
        travel = [abs(g - s) for s, g in zip(start, goal)]
        sent, limiting = capped_joint_speed(speed, travel)
        why = f", J{limiting + 1}'s top speed" if limiting is not None else ""
        if tool_speed is not None:
            per = self.tool_speed_per_deg_s(start, goal, tool if tool is not None else kin.IDENTITY)
            cap = math.floor(tool_speed / per * 10) / 10 if per > 0 else sent
            if cap < sent:
                sent, why = max(cap, 0.1), f", keeping the tool point under {tool_speed:g} mm/s"
        if self.reduced is not None:
            cap = reduced_joint_speed(self.model, start, goal, [self.reduced, tool])
            if cap is not None and cap < sent:
                sent, why = cap, REDUCED_WHY
        acc = min(max(100.0, sent / 0.3), self.joint_acc)
        lead = max(travel)
        profile = motion_profile.double_s(lead, sent, acc, self.joint_jerk) if lead > 0 else None
        duration = (profile.duration if profile else 0.0) + START_DELAY_JOINT_S
        text = f"Joint move to {label} at {sent:g}\u00b0/s{why}, about {duration:.1f} s"
        plan = Plan("joint", start, goal, [start, goal], [(goal, sent, acc)], duration, text)
        plan.limited = why[2:]  # what slowed it below the speed asked for, if anything
        return plan

    def tool_speed_per_deg_s(self, start, goal, tool=kin.IDENTITY):
        """An upper bound on the tool point's speed (mm/s) for each deg/s of the leading joint, anywhere on a joint
        move from start to goal (every joint turning in step): the sum of each joint's share of the turn times the
        tool point's distance from that joint's axis, which no combination of the turns can exceed, taken at
        TOOL_SPEED_SAMPLES points along the way and both ends."""
        travel = [g - s for s, g in zip(start, goal)]
        lead = max(abs(d) for d in travel)
        if lead <= 0:
            return 0.0
        worst = 0.0
        for k in range(TOOL_SPEED_SAMPLES + 1):
            q = [s + d * k / TOOL_SPEED_SAMPLES for s, d in zip(start, travel)]
            frames = self.model.frames(q)
            t = kin.mul(frames[-1], tool)
            p = [t[0][3], t[1][3], t[2][3]]
            total = 0.0
            for (_, _, axis, _), f, d in zip(self.model.joints, frames, travel):
                z = [f[r][0] * axis[0] + f[r][1] * axis[1] + f[r][2] * axis[2] for r in range(3)]
                arm = [p[r] - f[r][3] for r in range(3)]
                along = sum(a * b for a, b in zip(arm, z))
                total += abs(d) / lead * math.sqrt(max(sum(a * a for a in arm) - along * along, 0.0))
            worst = max(worst, total)
        return worst * 1000.0 * math.pi / 180.0

    def solve(self, seed, target, tool):
        """Joints (deg) putting the tool point at `target` (4x4), from `seed`; PlanError when it can't."""
        result = self.kin.solve(seed, target, DRAG_LIMITS_DEG, tool)
        if not result["ok"]:
            if result["atLimit"] >= 0:
                raise PlanError(f"J{result['atLimit'] + 1} would have to pass its limit to reach that pose.")
            raise PlanError("That pose is out of reach (or at a singularity) from the arm's configuration.")
        return result["joints"]

    def joint_to_pose(self, start, target, tool, speed, seed=None, label="the target"):
        goal = self.solve(seed or start, target, tool)
        return self.joint(start, goal, speed, label)

    # -- straight lines --------------------------------------------------------------------------------------------
    def line(self, start, target, tool, speed, ori_speed=60.0, label="the target"):
        """A straight line of the tool point from where `start` puts it to `target` (4x4, base <- tool point)."""
        if not 0 < speed <= MAX_TOOL_SPEED_MM_S:
            raise PlanError(f"Unsupported tool speed: {speed!r} mm/s.")
        start = [float(v) for v in start]
        t0 = self.kin.tool_pose(start, tool)
        length, turn = kin.pose_distance(t0, target)
        if length < 0.01 and turn < 0.01:
            return Plan("line", start, start, [start], [], 0.0, f"Already at {label}", tool, (t0, target))
        n = max(1, int(math.ceil(length / LINE_STEP_MM)), int(math.ceil(turn / LINE_STEP_DEG)))
        segments, why = None, ""
        if self.reduced is not None:  # each segment the controller drives, the tool point and flange under the limit
            segments = self._segments(t0, target, tool, [start] * (n + 1), speed)
            reach = max(math.dist(kin.position(t), (0.0, 0.0, 0.0)) for t in (tool, self.reduced))
            cap = min(reduced_line_speed(seg_len, seg_turn, reach) for _, seg_len, seg_turn in segments)
            if cap < speed:
                speed, why = cap, REDUCED_WHY
        acc = min(max(200.0, speed / 0.3), self.tool_acc)
        # The line's pace: how long each sample takes at the speed it actually reaches (short lines never reach the
        # speed asked for) or at the turning speed, whichever is slower.
        peak = motion_profile.double_s(length, speed, acc, self.tool_jerk).v_lim if length > 0 else speed
        seconds_per_sample = max(length / max(peak, 1e-9), turn / min(ori_speed, MAX_ORI_SPEED_DEG_S)) / n
        path, q = [start], start
        limits = self.model.start_limits(start)  # one allowance for parts already close, for the whole line
        for k in range(1, n + 1):
            pose = kin.interpolate(t0, target, k / n)
            result = self.kin.solve(q, pose, DRAG_LIMITS_DEG, tool)
            # Where on the line: in mm, to a tenth on a short one (a jog may be a fraction of a millimetre), or for a
            # turn in place, in degrees.
            along = (f"{length * k / n:.{1 if length < 10 else 0}f} mm along the line" if length >= 0.5 or turn < 0.5
                     else f"{turn * k / n:.0f}\u00b0 into the turn")
            clear = (k - 1) / n  # the samples before this one were fine
            if not result["ok"]:
                if result["atLimit"] >= 0:
                    raise PlanError(f"Not moving: J{result['atLimit'] + 1} would pass its limit {along}.", clear=clear)
                raise PlanError(f"Not moving: the line leaves the arm's reach {along}.", clear=clear)
            nxt = result["joints"]
            jump = max(abs(a - b) for a, b in zip(nxt, q))
            if jump > FLIP_DEG:
                raise PlanError(f"Not moving: the arm would flip its configuration {along} (a joint jumps "
                                f"{jump:.0f}\u00b0).", clear=clear)
            for j in range(6):
                rate = abs(nxt[j] - q[j]) / seconds_per_sample
                if rate > JOINT_RATED_DEG_S[j]:
                    raise PlanError(f"Not moving: the line passes too close to a singularity {along}: J{j + 1} would "
                                    f"need {rate:.0f}\u00b0/s at this speed (its top speed is {JOINT_RATED_DEG_S[j]:g}).",
                                    clear=clear)
            hit = self.model.check_path(q, nxt, limits=limits)
            if hit is not None:
                raise PlanError(f"Not moving: {blocked(hit)} {along}.", hit, clear=clear)
            path.append(nxt)
            q = nxt
        if segments is None:  # they depend on the line and its number of samples, not on the joints solved
            segments = self._segments(t0, target, tool, path, speed)
        duration = START_DELAY_LINE_S
        for flange, seg_len, seg_turn in segments:
            # unverified: how the controller paces a line's turn; the simulator counts 1 rad as 100 mm
            distance = max(seg_len, math.radians(seg_turn) * 100.0, 1e-9)
            duration += motion_profile.double_s(distance, speed, acc, self.tool_jerk).duration
        sent = [(flange, speed, acc) for flange, _, _ in segments]
        pieces = f" in {len(sent)} segments" if len(sent) > 1 else ""
        text = f"Straight line to {label}, {length:.1f} mm" + (f" turning {turn:.1f}\u00b0" if turn >= 0.05 else "") + \
            f", at {speed:g} mm/s{pieces}{why}, about {duration:.1f} s"
        return Plan("line", start, path[-1], path, sent, duration, text, tool, (t0, target))

    def _segments(self, t0, target, tool, path, speed):
        """The flange poses to send, each as (controller pose, flange path length mm): one, unless the tool point
        would stray from its line because the flange turns; then as many as keep it within SEGMENT_TOLERANCE_MM."""
        inv_tool = kin.invert(tool)
        n_samples = len(path) - 1
        for count in range(1, n_samples + 1):
            ends = [round(i * n_samples / count) for i in range(count + 1)]
            ok = True
            for a, b in zip(ends, ends[1:]):
                fa, fb = kin.mul(kin.interpolate(t0, target, a / n_samples), inv_tool), \
                    kin.mul(kin.interpolate(t0, target, b / n_samples), inv_tool)
                for u in (0.25, 0.5, 0.75):  # the controller's flange line, against the tool point's line
                    flange = kin.interpolate(fa, fb, u)
                    actual = kin.position(kin.mul(flange, tool))
                    ideal = kin.position(kin.interpolate(t0, target, (a + (b - a) * u) / n_samples))
                    if math.dist(actual, ideal) > SEGMENT_TOLERANCE_MM:
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                out = []
                for a, b in zip(ends, ends[1:]):
                    fa = kin.mul(kin.interpolate(t0, target, a / n_samples), inv_tool)
                    fb = kin.mul(kin.interpolate(t0, target, b / n_samples), inv_tool)
                    out.append((kin.controller_pose(fb), math.dist(kin.position(fa), kin.position(fb)),
                                kin.pose_distance(fa, fb)[1]))
                return out
        raise PlanError("Not moving: the tool point can't be kept on this line.")
