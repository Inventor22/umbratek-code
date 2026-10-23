"""Replays of what the real arm did, through the controller's stand-in (tools/utra_sim.py on a virtual clock) and the
twin's own model, against what the arm measured.

- capture(): data/captures/2026-10-21, the whole session. Every request the jog panel sent (Enable, joint moves, the
  straight-line tool jogs, STOP) becomes the controller commands the panel sent for it, at the time it was sent; the
  twin's arm follows the simulated controller, and every one of the 2292 status frames is compared with it: the
  joints, and the tool pose from the twin's kinematics against the pose the controller reported.
- program_moves(): data/captures/2026-10-21-tests, the afternoon's program runs. Each move the runner sent
  (runs/*.jsonl: when, how fast) is replayed from where the arm stood to where it came to rest (the logs don't hold
  the targets) and compared with the 10 Hz stream. The logs' timestamps are wall-clock milliseconds taken after the
  controller replied, so each move is also fitted with its own time offset (reported, and bounded).
- stop_mt2(): MT-2's STOP from J1 at -10 deg/s (stop-test.json), against the simulator's STOP model.
- kinematics(): the twin's forward kinematics of the measured joints against the controller's reported pose, in every
  frame of both captures.
Run `python -m brain.twin replay` for the report; tests/twin_tests.py holds the tolerances.
"""
from __future__ import annotations

import bisect
import datetime
import glob
import gzip
import json
import math
import os
import re
import struct

import numpy as np

from . import repo
from .coupling import Controller

STATUS = struct.Struct("<HBBBIIBBH")
CONFIG = struct.Struct("<H19fBB")
CAPTURE = os.path.join(repo.CAPTURES, "2026-10-21")
TESTS = os.path.join(repo.CAPTURES, "2026-10-21-tests")
PDT = datetime.timezone(datetime.timedelta(hours=-7))  # the run logs' local time on 2026-10-21 (Pacific daylight time)
OFFSET_RANGE_S, OFFSET_STEP_S = 0.15, 0.005


# ---- reading the captures ---------------------------------------------------------------------------------------

def records(path):
    """(wall-clock s, monotonic s, frame bytes) for each record of a capture stream (.bin, or .bin.gz)."""
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rb") as f:
        data = f.read()
    at, out = 0, []
    while at + 18 <= len(data):
        wall, mono, n = struct.unpack_from("<qqH", data, at)
        out.append((wall / 1e9, mono / 1e9, data[at + 18:at + 18 + n]))
        at += 18 + n
    return out


def status_rows(path):
    """The status frames of a capture. "late" marks frames whose receipt time isn't when they were sent: the first
    after a gap of more than 150 ms, and those that came less than 50 ms after the one before. Over Wi-Fi the stream
    stalled for about 3 s every 33 s and then delivered the held-back frames together, all stamped on arrival."""
    rows, prev = [], None
    for wall, mono, frame in records(path):
        _, axis, st, mode, brake, enable, err, warn, queued = STATUS.unpack_from(frame)
        v = struct.unpack_from(f"<{axis}f6f{axis}f", frame, STATUS.size)
        late = prev is not None and not 0.05 <= mono - prev <= 0.15
        rows.append({"wall": wall, "t": mono, "status": st, "mode": mode, "enable": enable, "err": err, "late": late,
                     "joints": [math.degrees(x) for x in v[:axis]], "pose": list(v[axis:axis + 6])})
        prev = mono
    return rows


def controller_args(path):
    """utra_sim options matching the controller's configuration in a capture's config stream."""
    c = CONFIG.unpack_from(records(path)[0][2])
    return ["--tcp-maxacc", repr(c[1]), "--tcp-jerk", repr(c[2]), "--joint-maxacc", repr(math.degrees(c[5])),
            "--joint-jerk", repr(math.degrees(c[6])), "--collision-sens", str(c[20]), "--teach-sens", str(c[21])]


# ---- comparing ------------------------------------------------------------------------------------------------

def rotation(pose):
    return np.array(repo.spine("utra_sim").rot_rpy(*pose[3:6]))


def angle_between(Ra, Rb):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Ra.T @ Rb) - 1.0) / 2.0))))


def pct(values, q):
    s = sorted(values)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))] if s else float("nan")


def stats(values):
    return {"n": len(values), "median": pct(values, 0.5), "p95": pct(values, 0.95), "max": max(values, default=float("nan"))}


class Errors:
    """Per-frame errors, by kind of frame."""

    def __init__(self):
        self.by = {}

    def add(self, kind, joints_deg=None, tcp_mm=None, turn_deg=None):
        d = self.by.setdefault(kind, {"joints_deg": [], "tcp_mm": [], "turn_deg": []})
        for key, value in (("joints_deg", joints_deg), ("tcp_mm", tcp_mm), ("turn_deg", turn_deg)):
            if value is not None:
                d[key].append(value)

    def summary(self):
        return {kind: {key: stats(v) for key, v in d.items() if v} for kind, d in self.by.items()}


def compare(twin, q_sim_rad, row, errors, kind):
    """One frame: the simulated joints against the measured ones, and the twin's tool pose for them against the pose
    the controller reported."""
    q_meas = row["joints"]
    joint_err = max(abs((math.degrees(a) - b + 180.0) % 360.0 - 180.0) for a, b in zip(q_sim_rad, q_meas))
    T = twin.fk(q_sim_rad)
    tcp = math.dist(T[:3, 3] * 1000.0, row["pose"][:3])
    turn = angle_between(T[:3, :3], rotation(row["pose"]))
    errors.add(kind, joint_err, tcp, turn)
    return joint_err, tcp, turn


class Trace:
    """A simulated controller's joints, recorded every 5 ms, so they can be read at any time, shifted or not, without
    running the controller again."""

    def __init__(self):
        self.t, self.q = [], []

    def record(self, ctrl, t):
        ctrl.advance(t)
        self.t.append(t)
        self.q.append(ctrl.joints(t))

    def freeze(self):
        self.t, self.q = np.array(self.t), np.array(self.q)
        return self

    def at(self, times):
        times = np.atleast_1d(np.asarray(times, dtype=float))
        return np.column_stack([np.interp(times, self.t, self.q[:, j]) for j in range(self.q.shape[1])])

    def moving(self, t):
        """Whether the simulated arm is moving at time t (more than 1e-6 rad in the 10 ms around it)."""
        a, b = self.at([t - 0.005, t + 0.005])
        return bool(np.abs(b - a).max() > 1e-6)


def fit_offset(trace, rows, key="t", twin=None):
    """The time offset (s, within +/-OFFSET_RANGE_S) that best lines the simulated motion up with these frames:
    what the capture can't time precisely (when a command reached the controller, when a frame left it). Joint moves
    are lined up on the joints; lines, given the twin, on the tool's position, which is what a line is about."""
    if not rows:
        return 0.0
    times = np.array([r[key] for r in rows])
    measured = np.radians(np.array([r["joints"] for r in rows]))
    positions = np.array([r["pose"][:3] for r in rows])

    def rms(offset):
        q = trace.at(times - offset)
        if twin is None:
            e = np.abs(q - measured).max(axis=1)
        else:
            e = np.array([np.linalg.norm(twin.fk(qi)[:3, 3] * 1000.0 - p) for qi, p in zip(q, positions)])
        return float(np.sqrt(np.mean(np.square(e))))

    coarse = min((rms(o), o) for o in np.arange(-OFFSET_RANGE_S, OFFSET_RANGE_S + 1e-9, 4 * OFFSET_STEP_S))[1]
    fine = np.arange(coarse - 4 * OFFSET_STEP_S, coarse + 4 * OFFSET_STEP_S + 1e-9, OFFSET_STEP_S)
    return float(min((rms(o), o) for o in fine if abs(o) <= OFFSET_RANGE_S + 1e-9)[1])


# ---- capture 1: the whole session ----------------------------------------------------------------------------------

def capture(twin, folder=CAPTURE):
    """Replay data/captures/2026-10-21 end to end: {"raw": Errors at the frames' own times, "fitted": Errors with each
    move's fitted time offset, "offsets": {kind: [s]}, "notes": [...]}. Frame kinds: joint and line (moving), stop
    (the 0.6 s after STOP) and rest."""
    jog = repo.spine("jog_server")
    rows = status_rows(os.path.join(folder, "status10hz.bin"))
    with open(os.path.join(folder, "commands.json"), encoding="utf-8") as f:
        commands = sorted(json.load(f), key=lambda c: c["sentMonoNs"])
    ctrl = Controller(rows[0]["joints"], controller_args(os.path.join(folder, "config.bin")), t0=rows[0]["t"] - 1.0)
    acc_max = (ctrl.arm.config["joint_maxacc"], ctrl.arm.config["tcp_maxacc"])
    if rows[0]["enable"] == 63 and rows[0]["status"] in (0, 2):  # the stream starts after the first Enable
        ctrl.enable()
        ctrl.advance(rows[0]["t"] - 0.5)
        ctrl.arm.q = [math.radians(v) for v in rows[0]["joints"]]

    events, enables = [], []
    for c in commands:
        sent, reply = c["sentMonoNs"] / 1e9, c["replyMonoNs"] / 1e9
        if sent < rows[0]["t"]:
            continue  # before the stream starts: its first frame has the state
        if c["path"] == "/api/enable":
            events.append(((sent + reply) / 2, "enable", c))
            enables.append((sent, reply + 0.1))  # when inside the request the motors came on isn't recorded
        elif c["path"] == "/api/stop":
            events.append((sent, "stop", c))
        elif c["path"] in ("/api/move", "/api/jog") and c["reply"].get("moved"):
            events.append((sent, c["path"][5:], c))

    # Run the session: each command at the time the panel sent it, the controller recorded every 5 ms. The commands
    # are the ones the panel built: from the arm's measured state as it stood (the controller's target registers
    # mirror the measured joints), or, for a click while the arm was busy, from the click before.
    notes, trace, segments = [], Trace(), []
    goal, goal_kind, last_sent = None, None, -math.inf
    times = [r["t"] for r in rows]
    k, t, end = 0, rows[0]["t"], rows[-1]["t"] + 0.01
    while t <= end:
        while k < len(events) and events[k][0] <= t:
            te, what, c = events[k]
            k += 1
            ctrl.advance(te)
            frame = rows[max(0, bisect.bisect_right(times, te) - 1)]
            if what == "enable":  # the arm is at rest: restart from where it measurably is, then Enable as sent
                ctrl.arm.q = [math.radians(v) for v in rows[max(0, bisect.bisect_right(times, c["sentMonoNs"] / 1e9) - 1)]["joints"]]
                ctrl.enable()
                goal = None
                continue
            if what == "stop":
                ctrl.stop()
                goal = None
                segments.append((te, "stop"))
                continue
            if what == "move":
                start = frame["joints"]
                goal_deg = [float(a) if a is not None and abs(a - s) >= 0.05 else s
                            for a, s in zip(c["body"]["targets"], start)]
                m = re.search(r"at (?:up to )?([\d.]+)\u00b0/s", c["reply"]["message"])
                speed = float(m.group(1)) if m else float(c["body"]["speed"])
                acc = min(max(jog.JOINT_ACC_MIN_DEG_S2, speed / jog.RAMP_S), math.degrees(acc_max[0]))
                ctrl.moveto_joint_p2p([math.radians(v) for v in goal_deg], math.radians(speed), math.radians(acc))
                goal, goal_kind, last_sent = None, "joint", te
                segments.append((te, "joint"))
                continue
            b = c["body"]  # a jog: one step from the last goal while the arm is busy, else from where it stands
            busy = frame["status"] == 1 or te - last_sent < jog.BUSY_AFTER_SEND_S
            if goal is None or not busy or goal_kind != b["kind"]:
                goal = list(frame["pose"]) if b["kind"] == "tool" else [math.radians(v) for v in frame["joints"]]
            goal = list(goal)
            if b["kind"] == "tool":
                if b["index"] < 3:
                    goal[b["index"]] += b["dir"] * float(b["step"])
                else:
                    goal[b["index"]] = jog.wrap(goal[b["index"]] + b["dir"] * math.radians(float(b["rotStep"])))
                speed = float(b["speed"])
                ctrl.moveto_cartesian_line(goal, speed, min(max(jog.TOOL_ACC_MIN_MM_S2, speed / jog.RAMP_S), acc_max[1]))
                shown = re.search(r"\u2192 (-?[\d.]+)", c["reply"]["message"])
                if shown and b["index"] < 3 and abs(float(shown.group(1)) - goal[b["index"]]) > 0.1:
                    notes.append(f"jog at {te:.2f} s: replayed goal {goal[b['index']]:.2f} mm, the panel's "
                                 f"{shown.group(1)} mm")
            else:
                goal[b["index"]] += b["dir"] * math.radians(float(b["step"]))
                speed = float(b["speed"])
                acc = min(max(jog.JOINT_ACC_MIN_DEG_S2, speed / jog.RAMP_S), math.degrees(acc_max[0]))
                ctrl.moveto_joint_p2p(goal, math.radians(speed), math.radians(acc))
            goal_kind, last_sent = b["kind"], te
            segments.append((te, "line" if b["kind"] == "tool" else "joint"))
        trace.record(ctrl, t)
        t += 0.005
    trace.freeze()

    # Score every frame, at its own time and with its move's fitted offset. A STOP's frames, and the rest after it
    # until the next move, keep the timing of the move it cut short (see stops()); they're scored raw, as "stop".
    raw, fitted, offsets = Errors(), Errors(), {}
    usable = [r for r in rows if not r["late"] and not any(a <= r["t"] <= b for a, b in enables)]
    bounds = [s[0] for s in segments] + [math.inf]
    for i in range(-1, len(segments)):
        lo, hi = (rows[0]["t"] - 1.0, bounds[0]) if i < 0 else (bounds[i], bounds[i + 1])
        kind = "rest" if i < 0 else segments[i][1]
        frames = [r for r in usable if lo <= r["t"] < hi]
        if kind == "stop":
            for r in frames:
                compare(twin, trace.at(r["t"])[0], r, raw, "stop")
            continue
        moving = [r for r in frames if r["status"] == 1]
        offset = fit_offset(trace, moving, twin=twin if kind == "line" else None) if moving else 0.0
        if moving:
            offsets.setdefault(kind, []).append(offset)
        moving_ids = {id(r) for r in moving}
        for r in frames:
            category = kind if id(r) in moving_ids or trace.moving(r["t"] - offset) else "rest"
            compare(twin, trace.at(r["t"])[0], r, raw, category)
            compare(twin, trace.at(r["t"] - offset)[0], r, fitted, category)
    return {"raw": raw, "fitted": fitted, "offsets": offsets, "notes": notes, "frames": len(usable),
            "late": sum(r["late"] for r in rows)}


# ---- the tests capture: the program runs' moves ------------------------------------------------------------------

def _wall(stamp):
    return datetime.datetime.strptime(stamp, "%Y-%m-%dT%H:%M:%S.%f").replace(tzinfo=PDT).timestamp()


def program_moves(folder=TESTS):
    """Each move the program runner sent, with its start, rest pose and frames: [{"run", "kind", "sent", "speed",
    "start", "rest", "frames", "until"}]. Moves that didn't arrive (interrupted, stopped, faulted) are left out."""
    rows = status_rows(os.path.join(folder, "status10hz.bin.gz"))
    walls = [r["wall"] for r in rows]
    moves = []
    for path in sorted(glob.glob(os.path.join(folder, "runs", "*.jsonl"))):
        with open(path, encoding="utf-8") as f:
            events = [json.loads(line) for line in f if line.strip()]
        kind = None
        for k, e in enumerate(events):
            if e["event"] in ("move.planned", "move.regain"):
                kind = "joint" if e["text"].startswith("Joint move") else "line"
            if e["event"] != "move.sent":
                continue
            after = events[k + 1] if k + 1 < len(events) else None
            if after is None or after["event"] != "move.arrived":
                continue
            sent, arrived = _wall(e["t"]), _wall(after["t"])
            nxt = next((_wall(x["t"]) for x in events[k + 2:] if x["event"] in ("move.sent", "operator", "run.paused")),
                       arrived + 2.0)
            i = bisect.bisect_right(walls, sent)
            start = rows[i - 1]
            j = bisect.bisect_right(walls, min(arrived + 0.5, nxt - 0.05))
            rest = rows[j - 1]
            frames = rows[i:j]
            if start["status"] not in (0, 2) or rows[i - 1]["wall"] < sent - 0.5 or not frames:
                continue  # not at rest when sent, or the stream had a gap there
            if kind == "line":  # a line whose turn sets its pace, by the simulator's own reckoning
                sim = repo.spine("utra_sim")
                turn = math.radians(angle_between(rotation(start["pose"]), rotation(rest["pose"])))
                if turn * sim.ROT_MM_PER_RAD > math.dist(start["pose"][:3], rest["pose"][:3]):
                    kind = "turn"
            moves.append({"run": os.path.basename(path), "kind": kind, "sent": sent, "speed": float(e["speed"]),
                          "start": start, "rest": rest, "frames": frames, "until": rest["wall"]})
            kind = None if kind == "turn" else kind
    return moves, controller_args(os.path.join(folder, "config.bin.gz"))


def replay_move(move, args):
    """One program move through the simulated controller, sent at its logged time: its Trace, from just before the
    move to just after it settled."""
    jog = repo.spine("jog_server")
    ctrl = Controller(move["start"]["joints"], args, t0=move["sent"] - 1.0)
    ctrl.enable()
    ctrl.advance(move["sent"] - 0.5)
    ctrl.arm.q = [math.radians(v) for v in move["start"]["joints"]]  # as measured, without a fresh enable shift
    trace = Trace()
    t = move["sent"] - OFFSET_RANGE_S - 0.01
    while t < move["sent"]:
        trace.record(ctrl, t)
        t += 0.005
    ctrl.advance(move["sent"])
    speed = move["speed"]
    if move["kind"] == "joint":
        acc = min(max(jog.JOINT_ACC_MIN_DEG_S2, speed / jog.RAMP_S), math.degrees(ctrl.arm.config["joint_maxacc"]))
        ctrl.moveto_joint_p2p([math.radians(v) for v in move["rest"]["joints"]], math.radians(speed),
                              math.radians(acc))
    else:  # a line or a turn
        acc = min(max(jog.TOOL_ACC_MIN_MM_S2, speed / jog.RAMP_S), ctrl.arm.config["tcp_maxacc"])
        ctrl.moveto_cartesian_line(move["rest"]["pose"], speed, acc)
    while t <= move["until"] + OFFSET_RANGE_S + 0.01:
        trace.record(ctrl, t)
        t += 0.005
    return trace.freeze()


def programs(twin, folder=TESTS):
    """Replay every program move of the tests capture: {"raw": Errors at the logged times, "fitted": Errors with each
    move's fitted offset, "offsets": {kind: [s]}, "moves": [...], "late": frames left out}. Kinds: joint, line, turn
    (a line whose turn, not its length, sets its pace) and rest."""
    moves, args = program_moves(folder)
    raw, fitted, offsets, late = Errors(), Errors(), {}, 0
    for move in moves:
        trace = replay_move(move, args)
        frames = [r for r in move["frames"] if not r["late"]]
        late += len(move["frames"]) - len(frames)
        moving = [r for r in frames if r["status"] == 1]
        offset = fit_offset(trace, moving, key="wall", twin=twin if move["kind"] == "line" else None)
        if moving:
            offsets.setdefault(move["kind"], []).append(offset)
        for r in frames:
            kind = move["kind"] if r["status"] == 1 or trace.moving(r["wall"] - offset) else "rest"
            compare(twin, trace.at(r["wall"])[0], r, raw, kind)
            compare(twin, trace.at(r["wall"] - offset)[0], r, fitted, kind)
    return {"raw": raw, "fitted": fitted, "offsets": offsets, "moves": moves, "late": late}


# ---- STOP --------------------------------------------------------------------------------------------------------

def stop_replay(start, goal, speed, acc, frames, stop_at, joint):
    """A joint move cut short by STOP, through the simulator: the move's send time is fitted to the frames before the
    STOP (it's the one thing the captures can't time precisely), STOP goes in at its logged time, and the frames after
    are compared. frames: [(t, measured angle of `joint`, deg)]. Returns ([(t, measured, simulated)] from the STOP on,
    {measured and simulated overrun past where STOP found the arm, the fitted send time, its fit})."""
    before = [f for f in frames if f[0] < stop_at and abs(f[1] - start[joint]) > 0.02]

    def run(sent):
        ctrl = Controller(start, t0=sent - 1.0)
        ctrl.enable()
        ctrl.advance(sent - 0.5)
        ctrl.arm.q = [math.radians(v) for v in start]
        ctrl.advance(sent)
        ctrl.moveto_joint_p2p([math.radians(v) for v in goal], math.radians(speed), math.radians(acc))
        out, at_stop = [], None
        for t, measured in frames:
            if t >= stop_at and at_stop is None:
                ctrl.advance(stop_at)
                at_stop = math.degrees(ctrl.joints(stop_at)[joint])
                ctrl.stop()
            ctrl.advance(t)
            out.append((t, measured, math.degrees(ctrl.joints(t)[joint])))
        return out, at_stop

    first = before[0][0] if before else stop_at
    candidates = np.arange(first - 0.3, first, 0.002)
    fit = min((sum((m - s) ** 2 for t, m, s in run(sent)[0] if t < stop_at and abs(m - start[joint]) > 0.02), sent)
              for sent in candidates)
    trace, sim_at_stop = run(fit[1])
    after = [x for x in trace if x[0] >= stop_at]
    measured_at_stop = np.interp(stop_at, [x[0] for x in trace], [x[1] for x in trace])
    return after, {"measured_overrun_deg": abs(after[-1][1] - measured_at_stop),
                   "simulated_overrun_deg": abs(after[-1][2] - sim_at_stop), "sent_s": float(fit[1]),
                   "fit_rms_deg": math.sqrt(fit[0] / max(1, len(before)))}


def stops():
    """Both measured STOPs, replayed: capture 1's (J1 at +30 deg/s, from which the simulator's STOP model was built)
    and the tests capture's MT-2 (J1 at -10 deg/s). Returns {name: (after, summary)}."""
    out = {}
    rows = status_rows(os.path.join(CAPTURE, "status10hz.bin"))
    with open(os.path.join(CAPTURE, "commands.json"), encoding="utf-8") as f:
        commands = sorted(json.load(f), key=lambda c: c["sentMonoNs"])
    k = next(i for i, c in enumerate(commands) if c["path"] == "/api/stop")
    move, stop = commands[k - 1], commands[k]
    t0 = move["sentMonoNs"] / 1e9
    start = [r for r in rows if r["t"] <= t0][-1]["joints"]
    speed = float(re.search(r"at (?:up to )?([\d.]+)\u00b0/s", move["reply"]["message"]).group(1))
    frames = [(r["t"] - t0, r["joints"][0]) for r in rows if t0 - 0.5 < r["t"] < stop["sentMonoNs"] / 1e9 + 0.6]
    out["capture J1 +30 deg/s"] = stop_replay(start, move["body"]["targets"], speed, min(max(100.0, speed / 0.3), 200.0),
                                              frames, stop["sentMonoNs"] / 1e9 - t0, 0)
    with open(os.path.join(TESTS, "stop-test.json"), encoding="utf-8") as f:
        test = json.load(f)
    speed = abs(test["speedDegS"])
    j1 = test["frames"][0]["j1"]
    frames = [(f["t"], f["j1"]) for f in test["frames"] if f["t"] < 0.6]
    out["MT-2 J1 -10 deg/s"] = stop_replay([j1, 0, 0, 0, 0, 0], [j1 - 30.0, 0, 0, 0, 0, 0], speed,
                                           min(max(100.0, speed / 0.3), 200.0), frames, 0.0, 0)
    return out


# ---- the twin's kinematics against the controller's reported pose ------------------------------------------------

def kinematics(twin, rows):
    """For each frame, the gap between the twin's FK of its measured joints and the pose the controller reported in
    it: (position mm, orientation deg) arrays."""
    mm, deg = [], []
    for r in rows:
        T = twin.fk([math.radians(v) for v in r["joints"]])
        mm.append(math.dist(T[:3, 3] * 1000.0, r["pose"][:3]))
        deg.append(angle_between(T[:3, :3], rotation(r["pose"])))
    return np.array(mm), np.array(deg)


# ---- the verdict: every replay against its stated tolerance ------------------------------------------------------

def verify(twin):
    """Every replay, each figure against its tolerance and the reason for it: [{"check", "value", "limit", "unit",
    "ok", "why"}]. tests/twin_tests.py and `python -m brain.twin replay` both use this."""
    out = []

    def check(name, value, limit, unit, why):
        out.append({"check": name, "value": float(value), "limit": limit, "unit": unit, "ok": bool(value <= limit),
                    "why": why})

    rows1 = status_rows(os.path.join(CAPTURE, "status10hz.bin"))
    rows2 = status_rows(os.path.join(TESTS, "status10hz.bin.gz"))
    exact = ("the stream's own float32 resolution: the twin's model is Umbratek's URDF, whose kinematics the "
             "controller's reported pose matches")
    mm, deg = kinematics(twin, rows1)
    check(f"capture: twin kinematics of the measured joints vs the reported tool position, worst of all {len(rows1)} "
          "frames", mm.max(), 0.001, "mm", exact)
    check("capture: ... and orientation", deg.max(), 0.001, "deg", exact)
    mm, deg = kinematics(twin, rows2)
    off = (mm > 0.001) | (deg > 0.001)
    check(f"tests capture: frames (of {len(rows2)}) whose reported pose isn't the twin's kinematics of their joints "
          "within 0.001 mm and 0.001 deg", off.sum(), 20, "frames", exact + "; see the next line for the exceptions")
    check("tests capture: ... those frames, worst", mm.max(), 1.0, "mm",
          "in 6 frames of the slow Taught run (15:19 to 15:20, at 10 mm/s) the controller's pose and joints disagree "
          "by 0.7 to 0.9 mm, about one frame's travel: the controller reported them from different cycles")

    c = capture(twin)
    raw, fit = c["raw"].summary(), c["fitted"].summary()
    settle = ("the real joints settle to within 0.05 deg of their targets and hold within 0.005 deg "
              "(measured in the capture of 2026-10-21, data/captures/2026-10-21); the simulator models neither the "
              "settle nor the servos' steady offset")
    check(f"capture, whole session ({c['frames']} frames): at rest, joints, median", raw["rest"]["joints_deg"]["median"],
          0.01, "deg", settle)
    check("capture: at rest, joints, 95th percentile", raw["rest"]["joints_deg"]["p95"], 0.06, "deg", settle)
    check("capture: joint moves at the logged times, joints, median", raw["joint"]["joints_deg"]["median"], 0.06, "deg",
          "commands crossed Wi-Fi in 12 to 45 ms (the requests' reply times) and frames arrived with about 7 ms of "
          "jitter: 10 ms at 30 to 60 deg/s is 0.3 to 0.6 deg")
    check("capture: joint moves at the logged times, joints, 95th percentile", raw["joint"]["joints_deg"]["p95"], 0.6,
          "deg", "as above")
    model = ("D6's double-S profile, fitted to these moves (median 0.034 deg at the logged times); what's left is the "
             "servos' lag at the ends of moves")
    check("capture: joint moves, each move's timing fitted, joints, median", fit["joint"]["joints_deg"]["median"], 0.03,
          "deg", model)
    check("capture: joint moves, each move's timing fitted, joints, 95th percentile", fit["joint"]["joints_deg"]["p95"],
          0.1, "deg", model)
    offsets = [abs(o) * 1000.0 for o in c["offsets"].get("joint", []) + c["offsets"].get("line", [])]
    check("capture: fitted timing offsets, largest", max(offsets), 50.0, "ms",
          "within the commands' Wi-Fi latency (12 to 45 ms from request to reply) and the frames' jitter")
    lines = ("the 50 mm clicks peak near 90 mm/s, where 7 ms of frame jitter is 0.6 mm; the real tool also strays up "
             "to 0.21 mm from its line")
    check("capture: straight lines, each move's timing fitted, tool position, median", fit["line"]["tcp_mm"]["median"],
          0.3, "mm", lines)
    check("capture: straight lines, each move's timing fitted, tool position, 95th percentile",
          fit["line"]["tcp_mm"]["p95"], 0.7, "mm", lines)

    stops_ = stops()
    after, s = stops_["capture J1 +30 deg/s"]
    check("capture: STOP from +30 deg/s, J1 after the STOP (the move's start fitted to the frames before it)",
          max(abs(sim - m) for _, m, sim in after), 0.03, "deg", "D7's STOP model was built from this stop")

    p = programs(twin)
    raw, fit = p["raw"].summary(), p["fitted"].summary()
    n = sum(len(m["frames"]) for m in p["moves"]) - p["late"]
    check(f"tests capture, {len(p['moves'])} program moves ({n} frames; {p['late']} that Wi-Fi delivered late left out): "
          "at rest, joints, median", raw["rest"]["joints_deg"]["median"], 0.01, "deg", settle)
    check("tests capture: at rest, joints, 95th percentile", raw["rest"]["joints_deg"]["p95"], 0.06, "deg", settle)
    check("tests capture: joint moves, each move's timing fitted, joints, median", fit["joint"]["joints_deg"]["median"],
          0.03, "deg", model)
    check("tests capture: joint moves, each move's timing fitted, joints, 95th percentile",
          fit["joint"]["joints_deg"]["p95"], 0.06, "deg", model)
    slow = ("lines at 4 to 25 mm/s, timed by the run log's wall clock; the long lines ran about 0.25% faster on the "
            "arm than the speed sent (the 520 mm line took 52.02 s, the simulator 52.15 s), over 1 mm by their ends")
    check("tests capture: straight lines, each move's timing fitted, tool position, median",
          fit["line"]["tcp_mm"]["median"], 0.3, "mm", slow)
    check("tests capture: straight lines, each move's timing fitted, tool position, 95th percentile",
          fit["line"]["tcp_mm"]["p95"], 0.6, "mm", slow)
    check("tests capture: straight lines, each move's timing fitted, tool position, worst", fit["line"]["tcp_mm"]["max"],
          2.0, "mm", slow)
    logged = [o * 1000.0 for o in p["offsets"].get("joint", []) + p["offsets"].get("line", [])]
    check("tests capture: fitted timing offsets, median distance from -20 ms", abs(float(np.median(logged)) + 20.0), 10.0,
          "ms", "the runner logs 'move.sent' once the controller has replied, so commands arrived about 20 ms before "
                "their logged times")
    turn = ("known limit: the simulator's pacing of turns is a guess (utra_sim.ROT_MM_PER_RAD, 1 rad as 100 mm, "
            "unverified); the arm turned 10 deg in about 1.2 s at 10 mm/s and 0.85 s at 25 mm/s, the simulator in 1.9 "
            "and 0.92 s (docs/decisions.md, D54). The tool's position is right; the joints lag mid-turn")
    check("tests capture: turns (lines paced by their turn), tool position, worst", fit["turn"]["tcp_mm"]["max"], 0.3,
          "mm", turn)
    check("tests capture: turns, joints, 95th percentile", fit["turn"]["joints_deg"]["p95"], 4.0, "deg", turn)
    after, s = stops_["MT-2 J1 -10 deg/s"]
    check("tests capture: MT-2's STOP from -10 deg/s, rest after STOP, simulated vs measured",
          abs(after[-1][2] - after[-1][1]), 0.15, "deg",
          "known limit: D7's overrun scales with speed (6.4 ms of travel, from the one stop at 30 deg/s); at 10 deg/s "
          f"the arm ran on {s['measured_overrun_deg']:.3f} deg, the model {s['simulated_overrun_deg']:.3f} "
          "(docs/decisions.md, D54)")
    return out
