"""Programs, and the runner that runs them on the server with its own supervision and an event log.

A program is a tree of steps, saved as JSON in cell/programs/<name>.json:
    move     {"kind": "joint" | "line" | "planned", "target": name, "offset": {...}?, "speed": number?}  (offsets:
             cell.apply_offset). A planned move's path comes from the brain, solved afresh each run around what's in
             the cell, and ends at the target's joints (as taught, so in the taught configuration). Instead of a
             target, a move may go to a pattern's slot: {"pattern": name, "slot": n?}, the slot its counter stands at
             unless it names one; a pattern whose every slot is used fails the step.
    learned  {"skill": name, "near": target, "within": mm, "seconds": s, "params": {...}?, "speed": deg/s?,
              "check": {"input": ..., "value": 0 | 1}?}  The brain's skill acts in short bursts, at most 10 a second,
             the tool point staying within `within` mm of the target `near`, until it says it's done (then `check`, if
             given, must read true), gives up, or runs out of `seconds`.
    wait     {"seconds": s} or {"input": "tool.N" | "controller.N", "value": 0 | 1, "timeout": s}
    set      {"output": "tool.N" | "controller.N", "value": 0 | 1}
    pulse    {"output": ..., "seconds": s}
    gripper  {"gripper": name, "action": "open" | "close" | "grasp", "part": {"mass": kg, "cog": [mm]}?}  Sets the
             gripper's outputs and waits for its input to confirm (or its set time, without one). A grasp then checks
             its "closed on a part" input, if it has one, failing without a part, and switches the payload to the tool
             and the part together (the part's centre of gravity in mm from the tool point, in its frame); opening
             switches it back.
    count    {"pattern": name, "to": n?}  Counts the pattern on to its next slot, or sets its counter to slot n (1 to
             start again). Counters persist (cell/counters.json); a dry run counts on a copy.
    if       {"input": ..., "value": 0 | 1, "then": [steps], "else": [steps]}
    repeat   {"count": n, "steps": [steps]}
    retry    {"tries": n, "steps": [steps]}  Runs the steps; if one fails, runs them again from the first, up to n
             tries in all.
    fallback {"steps": [steps], "else": [steps]}  Runs the steps; if one fails, runs the else steps instead. (Their
             paths are numbered on from the steps', so step 4.3 may be the first of the else steps.)
    home     {"target": name, "retract": mm, "speed": deg/s?}  Safe home: back along the tool by up to `retract` mm
             (as far as it can go clear of everything, if not all the way), then a joint move to the target.
    notify   {"message": ...}  Calls the owner: a log entry, and a call the panel shows until it's seen
             (Panel.call_owner; a phone notification later). The run carries on.
    call     {"program": name}
    comment  {"text": ...}
    halt     {"message": ...}  (pauses the run until the operator resumes)
Every step has a stable "id". Moves name targets, so re-teaching a target or recalibrating its frame changes every
move to it. Speeds: tool mm/s for lines, joint deg/s for joint, planned and learned moves, the program's defaults
where a step has none, times the override; lines, and whatever the brain proposes, are capped at REDUCED_SPEED_MM_S
at the tool (ISO 10218-1's reduced speed) unless the program says "fullSpeed": true. With the server's reduced speed
on (jog_server.py's manual-motion policy) every move is, fullSpeed or not, the flange too (planner.Planner's
`reduced`); with its hold-to-move on, a run pauses when the hold lapses, and a move to a target stops.

A step that can't do its job fails: it can't be planned or reached, the brain has no path, a wait or a gripper's input
times out, a grasp finds no part, a pattern is used up, an interlock keeps a zone closed (or the arm is in a zone one
keeps closed). That happens with the arm at rest. A Retry or Fallback around it catches a failure and carries on (if
the arm is still ready); anywhere else a failure stops the arm and faults the run, as it always has. A fault is never
caught: the controller reporting an error or losing status, the arm leaving its path or not arriving, the brain's
proposal refused. A faulted run calls the owner.

Moves into a zone that interlocks guard, or from inside one, go only while the interlocks hold (read just before the
move is planned), and the run watches them as the arm moves: if one stops holding, the arm stops and the run pauses,
and Resume is refused while it still doesn't hold. With the arm in a guarded zone whose interlocks don't hold, nothing
here moves it there; a jog or Move from the page may back it out (Panel.plan_guarded).

What the brain proposes, the safety shell (safety.py) checks whole before anything moves, and refuses with its reasons
if any part fails; a refusal faults the run. The brain only proposes: it can't start, pause, resume or stop anything,
and a STOP ends the run as stopped whatever the step was doing, waiting on the brain included.

The runner: start (from the top or a chosen step; run, step or dry run), pause (STOP, then resume re-plans the
interrupted step from wherever the arm is), resume, stop, step, reset, speed override. It runs in its own thread, so a
run carries on with the page closed; manual moves started from the page instead stop if the page goes quiet. Every
event goes to cell/logs/<date>/<time>-<program>.jsonl, a file of each run's own: another run of the same program
started in the same second gets <time>-<program>~2.jsonl, then ~3 and so on (new_log). Standard library only.
"""
from __future__ import annotations

import copy
import itertools
import json
import math
import os
import re
import tempfile
import threading
import time

import brain_link
import brain_protocol as bp
import cell as cellmod
import kinematics as kin
import planner as plannermod
import safety
from executor import Interrupted, MoveFailed

STEP_TYPES = ("move", "wait", "set", "pulse", "if", "repeat", "call", "comment", "halt", "learned", "gripper", "count",
              "retry", "fallback", "home", "notify")
MOVE_KINDS = ("joint", "line", "planned")
GRIPPER_ACTIONS = ("open", "close", "grasp")
IO = re.compile(r"^(tool|controller)\.(\d{1,2})$")
MAX_DEPTH = 6
MAX_STEPS = 2000
MAX_TRIES = 100
HOME_RETRACT_MM, MAX_RETRACT_MM = 50.0, 500.0
INTERLOCK_POLL_S = 0.1  # how often a move into a guarded zone reads the interlocks that let it in
REDUCED_SPEED_MM_S = plannermod.REDUCED_SPEED_MM_S
DEFAULT_SPEEDS = {"tcp": 50.0, "ori": 30.0, "joint": 20.0}
LEARNED_WITHIN_MM, MAX_WITHIN_MM = 100.0, 500.0
LEARNED_SECONDS = 60.0
ACT_PERIOD_S = 0.1  # a learned skill is asked for its next burst at most 10 times a second
POLL_S = 0.1
PAGE_SILENT_S = 1.5
EVENTS_KEPT = 300


class ProgramError(Exception):
    pass


class Stopped(Exception):
    pass


class Faulted(Exception):
    def __init__(self, reason, path=None, hit=None):
        super().__init__(reason)
        self.reason, self.path, self.hit = reason, path, hit


class Failed(Exception):
    """A step couldn't do its job, with the arm at rest: a Retry or Fallback around it may carry on; otherwise the run
    faults."""

    def __init__(self, reason, path=None, hit=None):
        super().__init__(reason)
        self.reason, self.path, self.hit = reason, path, hit


def drawable_hit(hit):
    """A planner's collision, as the page draws it: the pose, the closest points (m) and the obstacle's index."""
    if not hit:
        return None
    return {"part": hit["part"], "other": hit["other"], "obstacle": hit.get("obstacle"),
            "pose": [round(v, 3) for v in hit["pose"]], "points": [[round(v, 5) for v in p] for p in hit["points"]]}


def io_name(name, kind):
    m = IO.match(name or "") if isinstance(name, str) else None
    if not m or int(m.group(2)) > 15:
        raise ProgramError(f"Unsupported {kind}: {name!r} (use tool.0 to tool.15 or controller.0 to controller.15).")
    return m.group(1), int(m.group(2))


def number(value, what, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ProgramError(f"Unsupported {what}: {value!r} (expected {low:g} to {high:g}).")
    return float(value)


def whole(value, what, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value) or not low <= value <= high:
        raise ProgramError(f"Unsupported {what}: {value!r} (a whole number from {low} to {high}).")
    return int(value)


def check_part(part):
    """A grasped part's payload: {"mass": kg, "cog": [mm from the tool point, in its frame]}."""
    if not isinstance(part, dict):
        raise ProgramError(f"a part is its mass (kg) and centre of gravity (mm from the tool point), not {part!r}")
    cog = part.get("cog", [0.0, 0.0, 0.0])
    if not isinstance(cog, (list, tuple)) or len(cog) != 3:
        raise ProgramError(f"Unsupported part's centre of gravity: {cog!r} (3 numbers, mm from the tool point).")
    return {"mass": number(part.get("mass", 0.0), "part's mass (kg)", 0, cellmod.MAX_MASS_KG),
            "cog": [number(v, "part's centre of gravity (mm)", -1000, 1000) for v in cog]}


def where(path):
    return "step " + ".".join(str(i + 1) for i in path)


def dotted(path):
    return ".".join(str(i + 1) for i in path)


# ---- checking programs ---------------------------------------------------------------------------------------

def check_program(program, cell, load=None, depth=0, seen=None):
    """A program checked and tidied (ids added where missing); ProgramError says which step is wrong and why."""
    if not isinstance(program, dict):
        raise ProgramError("A program is an object with a name and steps.")
    name = cellmod.check_name(program.get("name"), "program")
    out = {"name": name, "steps": [], "speed": dict(DEFAULT_SPEEDS), "fullSpeed": bool(program.get("fullSpeed", False))}
    for key, (low, high) in (("tcp", (1, 1000)), ("ori", (1, 180)), ("joint", (1, 120))):
        if key in (program.get("speed") or {}):
            out["speed"][key] = number(program["speed"][key], f"default {key} speed", low, high)
    seen = (seen or set()) | {name}
    ids = set()
    count = [0]

    def steps(items, path, offset=0):
        if not isinstance(items, list):
            raise ProgramError(f"{where(path) if path else 'The program'}: steps must be a list.")
        return [step(s, path + [offset + i]) for i, s in enumerate(items)]

    def step(s, path):
        count[0] += 1
        if count[0] > MAX_STEPS:
            raise ProgramError(f"A program may have at most {MAX_STEPS} steps.")
        if not isinstance(s, dict) or s.get("type") not in STEP_TYPES:
            raise ProgramError(f"{where(path)}: unknown step {s!r}.")
        kind = s["type"]
        sid = s.get("id") or f"s{count[0]}"
        while sid in ids:
            sid += "x"
        ids.add(sid)
        o = {"id": str(sid), "type": kind}
        try:
            if kind == "move":
                o["kind"] = s.get("kind")
                if o["kind"] not in MOVE_KINDS:
                    raise ProgramError(f"a move is 'joint', 'line' or 'planned', not {o['kind']!r}")
                if s.get("pattern") is not None:
                    if s.get("target") is not None:
                        raise ProgramError("a move goes to a target or to a pattern's slot, not both")
                    cell.pattern(s["pattern"])
                    o["pattern"] = s["pattern"]
                    if s.get("slot") is not None:
                        o["slot"] = whole(s["slot"], "slot", 1, cellmod.MAX_PATTERN_SLOTS)
                else:
                    target = cell.target(s.get("target"))
                    o["target"] = s["target"]
                    if o["kind"] == "line" and "joints" in target:
                        raise ProgramError(f"{s['target']!r} is a joint target; a straight line needs a tool target")
                if s.get("offset"):
                    cellmod.apply_offset(kin_identity(), kin_identity(), s["offset"])
                    o["offset"] = s["offset"]
                if s.get("speed") is not None:
                    o["speed"] = number(s["speed"], "speed", 1, 1000 if o["kind"] == "line" else 120)
            elif kind == "wait":
                if "input" in s:
                    io_name(s["input"], "input")
                    o.update(input=s["input"], value=int(number(s.get("value"), "input value", 0, 1)),
                             timeout=number(s.get("timeout", 30), "timeout (s)", 0.1, 86400))
                else:
                    o["seconds"] = number(s.get("seconds"), "wait (s)", 0, 86400)
            elif kind in ("set", "pulse"):
                io_name(s.get("output"), "output")
                o["output"] = s["output"]
                if kind == "set":
                    o["value"] = int(number(s.get("value"), "output value", 0, 1))
                else:
                    o["seconds"] = number(s.get("seconds"), "pulse (s)", 0.01, 60)
            elif kind == "if":
                io_name(s.get("input"), "input")
                o.update(input=s["input"], value=int(number(s.get("value"), "input value", 0, 1)),
                         then=steps(s.get("then", []), path), **{"else": steps(s.get("else", []), path)})
            elif kind == "repeat":
                o["count"] = int(number(s.get("count"), "repeat count", 1, 100000))
                o["steps"] = steps(s.get("steps", []), path)
            elif kind == "retry":
                o["tries"] = whole(s.get("tries", 3), "number of tries", 1, MAX_TRIES)
                o["steps"] = steps(s.get("steps", []), path)
            elif kind == "fallback":
                o["steps"] = steps(s.get("steps", []), path)
                o["else"] = steps(s.get("else", []), path, offset=len(o["steps"]))
            elif kind == "gripper":
                cell.gripper(s.get("gripper"))
                o["gripper"] = s["gripper"]
                o["action"] = s.get("action")
                if o["action"] not in GRIPPER_ACTIONS:
                    raise ProgramError(f"a gripper step opens, closes or grasps, not {o['action']!r}")
                if s.get("part") is not None:
                    if o["action"] != "grasp":
                        raise ProgramError("only a grasp takes a part")
                    o["part"] = check_part(s["part"])
            elif kind == "count":
                cell.pattern(s.get("pattern"))
                o["pattern"] = s["pattern"]
                if s.get("to") is not None:
                    o["to"] = whole(s["to"], "slot to count to", 1, cellmod.MAX_PATTERN_SLOTS + 1)
            elif kind == "home":
                cell.target(s.get("target"))
                o["target"] = s["target"]
                o["retract"] = number(s.get("retract", HOME_RETRACT_MM), "retract (mm)", 0, MAX_RETRACT_MM)
                if s.get("speed") is not None:
                    o["speed"] = number(s["speed"], "speed", 1, 120)
            elif kind == "notify":
                o["message"] = str(s.get("message") or "The run needs you.")[:500]
            elif kind == "call":
                callee = cellmod.check_name(s.get("program"), "program")
                if callee in seen:
                    raise ProgramError(f"calling {callee!r} would loop back to it")
                if depth + 1 > MAX_DEPTH:
                    raise ProgramError(f"calls nest more than {MAX_DEPTH} deep")
                if load is not None:
                    check_program(load(callee), cell, load, depth + 1, seen)
                o["program"] = callee
            elif kind == "comment":
                o["text"] = str(s.get("text", ""))[:500]
            elif kind == "halt":
                o["message"] = str(s.get("message", "Halted."))[:500]
            elif kind == "learned":
                skill = s.get("skill")
                if not isinstance(skill, str) or not bp.NAME.match(skill):
                    raise ProgramError(f"Unsupported skill name: {skill!r}. Use up to 40 letters, digits, '_', '-' or "
                                       "'.', starting with a letter.")
                o["skill"] = skill
                if s.get("params", {}) != {}:
                    try:
                        o["params"] = bp.params(s["params"])
                    except bp.ProtocolError as e:
                        raise ProgramError(str(e))
                cell.target(s.get("near"))
                o["near"] = s["near"]
                o["within"] = number(s.get("within", LEARNED_WITHIN_MM), "region (mm)", 5, MAX_WITHIN_MM)
                o["seconds"] = number(s.get("seconds", LEARNED_SECONDS), "time limit (s)", 1, 3600)
                if s.get("speed") is not None:
                    o["speed"] = number(s["speed"], "speed", 1, 120)
                if s.get("check") is not None:
                    check = s["check"]
                    if not isinstance(check, dict):
                        raise ProgramError(f"a check is an input and the value it must read, not {check!r}")
                    io_name(check.get("input"), "input")
                    o["check"] = {"input": check["input"], "value": int(number(check.get("value"), "input value", 0, 1))}
        except (ProgramError, cellmod.CellError) as e:
            raise ProgramError(f"{where(path)} ({kind}): {e}")
        return o

    out["steps"] = steps(program.get("steps", []), [])
    return out


def kin_identity():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def walk(steps):
    """Every step in a list of steps and the lists inside them (repeats', retries' and fallbacks' steps, ifs' and
    fallbacks' branches)."""
    for s in steps if isinstance(steps, list) else []:
        if isinstance(s, dict):
            yield s
            for key in ("steps", "then", "else"):
                yield from walk(s.get(key))


def step_list(steps, path):
    """The list the step at `path` (0-based indexes, as runs number steps: a fallback's else steps after its own) is in,
    and its index there."""
    container, index = steps, path[0]
    for nxt in path[1:]:
        node = container[index]
        own = node.get("steps") or []
        if node.get("type") == "fallback" and nxt >= len(own):
            container, index = node.get("else") or [], nxt - len(own)
        elif node.get("type") == "if":
            container, index = node.get("then") or [], nxt  # an if's branches share their numbers
        else:
            container, index = own, nxt
    return container, index


def programs_using(programs, name, what="target"):
    """The saved programs that use the target `name` (a move to it, a learned step near it, safe home to it), or with
    `what` "pattern" the pattern (a move to one of its slots, or counting it), or "gripper" the gripper, as
    'program <name>'."""
    def uses(s):
        kind = s.get("type")
        if what == "pattern":
            return kind in ("move", "count") and s.get("pattern") == name
        if what == "gripper":
            return kind == "gripper" and s.get("gripper") == name
        return (kind in ("move", "home") and s.get("target") == name) or (kind == "learned" and s.get("near") == name)
    out = []
    for program in programs.names():
        try:
            if any(uses(s) for s in walk(programs.load(program).get("steps"))):
                out.append(f"program {program}")
        except (ProgramError, ValueError, OSError):
            continue
    return out


def brain_needs(steps, load, calls=0):
    """What a checked program's steps (and the programs they call) need of the brain: (whether any move is planned,
    the skills of its learned steps)."""
    plans, skills = False, set()
    for s in steps if isinstance(steps, list) else []:
        if not isinstance(s, dict):
            continue
        plans = plans or (s.get("type") == "move" and s.get("kind") == "planned")
        if s.get("type") == "learned" and isinstance(s.get("skill"), str):
            skills.add(s["skill"])
        for key in ("steps", "then", "else"):
            p, k = brain_needs(s.get(key), load, calls)
            plans, skills = plans or p, skills | k
        if s.get("type") == "call" and calls < MAX_DEPTH:
            try:
                p, k = brain_needs(load(s.get("program")).get("steps"), load, calls + 1)
            except (ProgramError, cellmod.CellError, ValueError, OSError):
                continue
            plans, skills = plans or p, skills | k
    return plans, skills


def brain_state(state):
    """The arm's state as the brain is told it (brain_protocol.state)."""
    out = {"seq": state["seq"], "ageS": round(max(0.0, time.monotonic() - state["at"]), 3),
           "joints": [round(v, 4) for v in state["joints"]], "tool": state["tool"]}
    if state.get("virtual"):
        out["virtual"] = True
    if "inputs" in state:
        out["inputs"] = state["inputs"]
    return out


def brain_limits(burst=False):
    """The limits the shell holds the brain to, so it can plan within them (docs/brain-interface.md)."""
    out = {"joints": [list(v) for v in plannermod.DRAG_LIMITS_DEG], "ratedDegS": list(plannermod.JOINT_RATED_DEG_S)}
    if burst:
        out.update(burstMaxWaypoints=bp.MAX_BURST_WAYPOINTS, burstMaxJointDeg=safety.BURST_MAX_JOINT_DEG,
                   burstMaxToolMm=safety.BURST_MAX_TOOL_MM, periodS=ACT_PERIOD_S, holdMaxS=bp.HOLD_MAX_S)
    else:
        out["maxWaypoints"] = bp.MAX_PLAN_WAYPOINTS
    return out


class Programs:
    """Programs saved as cell/programs/<name>.json, each with a revision."""

    def __init__(self, folder):
        self.folder = os.path.join(folder, "programs")
        self.lock = threading.Lock()

    def path(self, name):
        return os.path.join(self.folder, cellmod.check_name(name, "program") + ".json")

    def names(self):
        if not os.path.isdir(self.folder):
            return []
        return sorted(f[:-5] for f in os.listdir(self.folder) if f.endswith(".json") and not f.startswith("."))

    def load(self, name):
        try:
            with open(self.path(name), encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            raise ProgramError(f"There's no program called {name!r}.")

    def save(self, program, cell, revision=None):
        with self.lock:
            checked = check_program(program, cell, self.load)
            path = self.path(checked["name"])
            current = 0
            if os.path.exists(path):
                current = self.load(checked["name"]).get("revision", 0)
            if revision is not None and revision != current:
                raise ProgramError(f"{checked['name']!r} changed since this page loaded it (revision {revision}, now "
                                   f"{current}). Reload and try again.")
            checked["revision"] = current + 1
            os.makedirs(self.folder, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".prog-", suffix=".json", dir=self.folder)
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(checked, f, indent=1)
                f.write("\n")
            os.replace(tmp, path)
            return checked

    def delete(self, name):
        with self.lock:
            try:
                os.remove(self.path(name))
            except FileNotFoundError:
                raise ProgramError(f"There's no program called {name!r}.")


# ---- the runner ------------------------------------------------------------------------------------------------

def new_log(folder, stamp, name):
    """A new, empty run log in `folder`: <stamp>-<name>.jsonl, or, if another run of the program started in the same
    second, <stamp>-<name>~2.jsonl, ~3 and so on ('~' is never in a program's name, so the name still reads back).
    Created exclusively, so no two runs share a log, even two panels on one cell folder."""
    for n in itertools.count(1):
        path = os.path.join(folder, f"{stamp}-{name}{'' if n == 1 else f'~{n}'}.jsonl")
        try:
            with open(path, "x", encoding="utf-8"):
                return path
        except FileExistsError:
            continue


class Runner:
    """Runs one program (or one manual move) at a time, in its own thread."""

    def __init__(self, panel, programs, log_folder):
        self.panel, self.programs, self.log_folder = panel, programs, log_folder
        self.lock = threading.Condition()
        self.state, self.mode, self.reason = "idle", None, ""
        self.program, self.manual = None, False
        self.path, self.step_text = None, ""
        self.override = 100
        self.events = []
        self.request = None  # "pause" | "stop" while one is pending
        self.go = None  # "resume" | "step" | "stop" for a paused run
        self.thread = None
        self.log_path = None
        self.started_at = None
        self.estimate = None
        self.halt_message = ""
        self.hit = None  # where a faulted run's step would have collided, for the 3D view
        self.dropped = None  # (zone, why) when an interlock stopped a move of this run (_watching)
        self.run_tag, self.episodes = "", 0  # for learned skills' episode ids

    # -- the status and log ----------------------------------------------------------------------------------------
    def status(self):
        with self.lock:
            return {"state": self.state, "mode": self.mode, "manual": self.manual,
                    "program": self.program["name"] if self.program else None, "path": self.path,
                    "step": self.step_text, "reason": self.reason, "override": self.override,
                    "events": self.events[-40:], "log": os.path.relpath(self.log_path, self.log_folder) if self.log_path else None,
                    "estimateS": self.estimate, "halt": self.halt_message, "hit": self.hit}

    @property
    def busy(self):
        """A run is under way (not idle, done, stopped or faulted)."""
        return self.state in ("running", "pausing", "paused", "stopping")

    @property
    def moving(self):
        return self.state in ("running", "pausing", "stopping")

    def log(self, event, **detail):
        entry = {"t": time.strftime("%Y-%m-%dT%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}", "event": event}
        if self.path is not None and "path" not in detail:
            detail["path"] = ".".join(str(i + 1) for i in self.path)
        entry.update(detail)
        with self.lock:
            self.events.append(entry)
            del self.events[:-EVENTS_KEPT]
        if self.log_path:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry) + "\n")
                f.flush()

    # -- the operator's controls -------------------------------------------------------------------------------------
    def start(self, name=None, mode="run", from_path=None, program=None, manual=False):
        if mode not in ("run", "step", "dry"):
            raise ProgramError(f"Unsupported mode: {mode!r}.")
        cell = self.panel.cell
        if program is None:
            program = self.programs.load(name)
        checked = check_program(program, cell, self.programs.load)
        if from_path is not None and (not isinstance(from_path, list) or not all(isinstance(i, int) and i >= 0
                                                                                  for i in from_path)):
            raise ProgramError(f"Unsupported starting step: {from_path!r}.")
        plans, skills = brain_needs(checked["steps"], self.programs.load)
        if (plans or skills) and not self.busy:  # one question, so a run doesn't fault at its first brain step
            self.panel.check_brain(plans, sorted(skills))
        with self.lock:
            if self.busy:
                raise ProgramError(f"A run is under way ({self.state}); stop it first.")
            if self.state == "faulted":
                raise ProgramError(f"The last run faulted ({self.reason}). Reset it first.")
            if mode != "dry" and self.panel.grip_lock.locked():
                raise ProgramError("A gripper is opening or closing from the page. Wait for it, then start again.")
            self.panel.check_can_run(dry=(mode == "dry"))
            self.state, self.mode, self.reason, self.halt_message, self.hit = "running", mode, "", "", None
            self.program, self.manual, self.path, self.step_text = checked, manual, None, ""
            self.request = self.go = None
            self.events, self.estimate = [], None
            self.started_at = time.monotonic()
            now = time.localtime()
            self.run_tag, self.episodes = time.strftime("%H%M%S", now), 0
            folder = os.path.join(self.log_folder, time.strftime("%Y-%m-%d", now))
            os.makedirs(folder, exist_ok=True)
            self.log_path = new_log(folder, self.run_tag, checked["name"])
            self.thread = threading.Thread(target=self._run, args=(from_path,), daemon=True)
        self.log("run.start", program=checked["name"], mode=mode, override=self.override, manual=manual,
                 **({"from": ".".join(str(i + 1) for i in from_path)} if from_path else {}))
        self.thread.start()

    def pause(self):
        with self.lock:
            if self.state != "running":
                raise ProgramError(f"Nothing to pause (the run is {self.state}).")
            self.state, self.request = "pausing", "pause"
            self.lock.notify_all()
        self.log("operator", action="pause")

    def resume(self):
        with self.lock:
            if self.state != "paused":
                raise ProgramError(f"Nothing to resume (the run is {self.state}).")
            self.go, self.mode = "resume", ("run" if self.mode == "step" else self.mode)
            self.lock.notify_all()
        self.log("operator", action="resume")

    def step(self):
        with self.lock:
            if self.state != "paused":
                raise ProgramError(f"Step works on a paused run (the run is {self.state}).")
            self.go, self.mode = "step", "step"
            self.lock.notify_all()
        self.log("operator", action="step")

    def stop(self, cause="the operator pressed Stop"):
        """Ends the run. Returns True if a run was stopped."""
        with self.lock:
            if not self.busy:
                return False
            self.request, self.go = "stop", "stop"
            self.reason = cause
            if self.state != "paused":
                self.state = "stopping"
            self.lock.notify_all()
        self.log("operator", action="stop", cause=cause)
        return True

    def reset(self):
        with self.lock:
            if self.busy:
                raise ProgramError(f"The run is {self.state}; stop it first.")
            self.state, self.reason, self.path, self.step_text, self.halt_message = "idle", "", None, "", ""
            self.hit = None
        self.log("operator", action="reset")

    def set_override(self, percent):
        percent = int(number(percent, "speed override (%)", 10, 100))
        with self.lock:
            self.override = percent
        self.log("operator", action="override", percent=percent)

    # -- the run --------------------------------------------------------------------------------------------------
    def _run(self, from_path):
        self.skip_to = from_path
        self.virtual = None  # a dry run's planned joints
        self.counts = None  # a dry run's copy of the patterns' counters
        self.anchor = None  # (step id, line poses, tool) of a straight line under way
        self.homing = None  # (step id, where its retract goes, whether it's done) of a safe home under way
        self.dropped = None  # (zone, why) when an interlock stopped holding during a move: the pause is the interlock's
        self.total = 0.0
        try:
            if self.mode == "dry":
                self.virtual = self.panel.current_joints()
                self.counts = self.panel.counters.snapshot()
            self._steps(self.program, self.program["steps"], [], depth=0)
            # Each run's last event is written with the lock held, as its state changes: another run may start as
            # soon as this one is done, stopped or faulted, and its events go to its own log.
            with self.lock:
                self.state, self.path, self.step_text = "done", None, ""
                if self.mode == "dry":
                    self.estimate = round(self.total, 1)
                self.log("run.done", seconds=round(time.monotonic() - self.started_at, 2),
                         **({"estimateS": round(self.total, 1)} if self.mode == "dry" else {}))
        except Stopped:
            if self.virtual is None:
                self.panel.stop(self.reason or "the run was stopped")
            with self.lock:
                self.state = "stopped"
                self.log("run.stopped", reason=self.reason)
        except Failed as e:  # a failure no Retry or Fallback caught: the run faults, as any failure always has
            if self.virtual is None:
                self.panel.stop(f"{where(e.path)} failed")
            self._fault(e.reason, e.hit, e.path)
        except Faulted as e:
            self._fault(e.reason, e.hit, e.path)
        except Exception as e:  # a bug: fault the run rather than die silently
            self.panel.stop("the program runner failed")
            self._fault(f"internal error: {e!r}")

    def _fault(self, reason, hit=None, path=None):
        with self.lock:
            self.state, self.reason, self.hit = "faulted", reason, drawable_hit(hit)
            self.log("run.faulted", reason=reason)
        if not self.manual and self.mode != "dry":  # nobody may be watching an unattended run: call the owner
            self.panel.call_owner(f"{self.program['name']} faulted: {reason}", program=self.program["name"],
                                  path=dotted(path) if path else None, why="fault")

    def _steps(self, program, steps, path, depth, offset=0):
        for i, s in enumerate(steps):
            self._step(program, s, path + [offset + i], depth)

    def _skipping(self, path):
        if self.skip_to is None:
            return False
        if path == self.skip_to[:len(path)]:
            if len(path) >= len(self.skip_to):
                self.skip_to = None
                return False
            return False  # on the way to it: enter
        return True

    def _step(self, program, s, path, depth):
        if self._skipping(path):
            return
        kind = s["type"]
        if kind == "repeat":
            for n in range(s["count"]):
                self.log("repeat", iteration=n + 1, of=s["count"], path=dotted(path))
                self._steps(program, s["steps"], path, depth)
            return
        if kind == "retry":
            for attempt in range(1, s["tries"] + 1):
                try:
                    self._steps(program, s["steps"], path, depth)
                    return
                except Failed as e:
                    if attempt == s["tries"]:
                        self.log("retry.spent", tries=s["tries"], reason=e.reason, path=dotted(path))
                        raise
                    self._carry_on(e)
                    self.log("retry", attempt=attempt + 1, of=s["tries"], reason=e.reason, path=dotted(path))
            return
        if kind == "fallback":
            # "Run from" a step of the else branch goes straight there.
            into_else = self.skip_to is not None and len(self.skip_to) > len(path) and \
                self.skip_to[:len(path)] == path and self.skip_to[len(path)] >= len(s["steps"])
            if not into_else:
                try:
                    self._steps(program, s["steps"], path, depth)
                    return
                except Failed as e:
                    self._carry_on(e)
                    self.log("fallback", reason=e.reason, path=dotted(path))
            self._steps(program, s["else"], path, depth, offset=len(s["steps"]))
            return
        if kind == "if":
            value = self._read_input(s["input"], path)
            branch = "then" if value == s["value"] else "else"
            self.log("if", input=s["input"], read=value, branch=branch, path=dotted(path))
            self._steps(program, s[branch], path, depth)
            return
        if kind == "call":
            callee = check_program(self.programs.load(s["program"]), self.panel.cell, self.programs.load)
            self.log("call", program=s["program"], path=dotted(path))
            self._steps(callee, callee["steps"], path, depth + 1)
            return
        self._leaf(program, s, path)

    def _carry_on(self, failure):
        """A Retry or Fallback carries on after a failure only if the arm is ready, as it is when a step fails (before
        anything is sent, or once a move has arrived); otherwise the run faults."""
        with self.lock:
            if self.request == "stop":
                raise Stopped()
        self.anchor = self.homing = None  # what failed isn't resumed
        if self.virtual is not None:
            return
        why = self.panel.why_not_ready()
        if why:
            self.panel.stop(f"{where(failure.path)} failed")
            raise Faulted(f"{failure.reason}; the run couldn't carry on from it, as {why}", failure.path, failure.hit)

    def _leaf(self, program, s, path):
        held = False  # this step was paused by an interlock that stopped holding (_watching)
        while True:
            with self.lock:
                self.path, self.step_text = path, describe(s)
                if self.request == "stop":
                    raise Stopped()
                pausing = self.request == "pause"
            if pausing:  # asked for between steps
                self._pause_here("the operator pressed Pause")
            started = time.monotonic()
            self.log("step.start", id=s["id"], type=s["type"], text=self.step_text)
            try:
                self._do(program, s, path)
                self.log("step.done", id=s["id"], seconds=round(time.monotonic() - started, 2))
                if self.mode == "step":
                    self._pause_here("stepped")
                return
            except Interrupted as e:
                if self.request == "stop":
                    raise Stopped()
                self.log("step.interrupted", id=s["id"], reason=e.reason)
                held, self.dropped = self.dropped is not None, None
                self._pause_here(e.reason, resume_same=True)
            except plannermod.ZoneClosed as e:
                if self.request == "stop":
                    raise Stopped()
                if not held:
                    self.log("step.failed", id=s["id"], reason=str(e))
                    raise Failed(f"{where(path)}: {e}", path, e.hit)
                # Resumed while the interlock that paused it still doesn't hold: it stays paused, nothing sent.
                why = str(e)[len("Not moving: "):] if str(e).startswith("Not moving: ") else str(e)
                self.log("resume.refused", id=s["id"], reason=why)
                self._pause_here(f"Resume refused: {why}", resume_same=True)
            except HaltHere as e:
                self._pause_here(e.message, halt=True)
                self.log("step.done", id=s["id"], seconds=round(time.monotonic() - started, 2))
                return
            except MoveFailed as e:
                if self.request == "stop":  # STOP wins: whatever else went wrong meanwhile, the run ends stopped
                    raise Stopped()
                raise Faulted(f"{where(path)}: {e.reason}", path)
            except (safety.Refused, bp.ProtocolError) as e:
                if self.request == "stop":
                    raise Stopped()
                self.log("brain.refused", id=s["id"], reason=str(e), reasons=getattr(e, "reasons", [str(e)]))
                if self.virtual is None:
                    self.panel.stop(f"{where(path)}: the brain's proposal was refused")
                raise Faulted(f"{where(path)}: {e}", path, getattr(e, "hit", None))
            except (ProgramError, cellmod.CellError, plannermod.PlanError, brain_link.BrainError) as e:
                if self.request == "stop":
                    raise Stopped()
                self.log("step.failed", id=s["id"], reason=str(e))
                raise Failed(f"{where(path)}: {e}", path, getattr(e, "hit", None))

    def _pause_here(self, why, resume_same=False, halt=False):
        """Wait, paused, for resume, step or stop. Raises Stopped on stop."""
        with self.lock:
            self.state, self.request, self.go = "paused", None, None
            self.reason = why
            if halt:
                self.halt_message = why
            self.lock.notify_all()
        self.log("run.paused", reason=why)
        with self.lock:
            while self.go is None:
                self.lock.wait(0.5)
            go = self.go
            self.go = None
            if go == "stop":
                raise Stopped()
            self.state, self.reason, self.halt_message = "running", "", ""
        self.log("run.resumed", how=go)
        if resume_same and self.virtual is None:
            self.panel.rearm()

    def _abort(self):
        """The executor's check, a few times a second: a pause or stop asked for, or (for a manual move) a quiet page;
        and with hold-to-move on, its hold lapsed, which pauses a program's run and stops a move to a target, as a quiet
        page does."""
        if self.request == "stop":
            return self.reason or "the operator pressed Stop"
        if self.request == "pause":
            return "the operator pressed Pause"
        if self.manual and time.monotonic() - self.panel.heard_at > PAGE_SILENT_S:
            self.request, self.reason = "stop", "the jog page went quiet"
            return "the jog page went quiet"
        lapsed = self.panel.hold_lapsed() if self.mode != "dry" else None
        if lapsed and self.manual:
            self.request, self.reason = "stop", lapsed
        return lapsed

    def _watching(self, plan):
        """The executor's check for `plan`: _abort's, and for a move that depends on guarded zones (plan.guards), their
        interlocks, read every INTERLOCK_POLL_S (Panel.watch_guards). If one stops holding, the arm stops and the run
        pauses (a move to a target stops), and the owner is called; Resume plans the rest again, reading them again,
        and stays paused while one doesn't hold (_leaf)."""
        guards = getattr(plan, "guards", None)
        if not guards:
            return self._abort
        last = [time.monotonic()]  # the plan was made with them read just now
        watch = {"at": last[0]}

        def check():
            why = self._abort()
            if why:
                return why
            now = time.monotonic()
            if now - last[0] < INTERLOCK_POLL_S:
                return None
            last[0] = now
            dropped = self.panel.watch_guards(guards, watch)
            if not dropped:
                return None
            why = f"{dropped[1]}, during a move into the {dropped[0]} zone"
            if self.manual:
                self.request, self.reason = "stop", why
            else:
                self.dropped = dropped
                self.panel.call_owner(f"{self.program['name']} paused: {why}", program=self.program["name"],
                                      path=dotted(self.path or []), why="interlock")
            return why
        return check

    def _execute(self, plan):
        self.panel.executor.execute(plan, self._watching(plan), lambda event, **d: self.log(event, **d))

    def _guards(self, plan):
        """In a dry run, say which guarded zones a planned move enters, their interlocks assumed to hold."""
        for zone, names in sorted((getattr(plan, "guards", None) or {}).items()):
            self.log("dry.assumed", zone=zone, interlocks=names, note=f"assumed {', '.join(names)} will let it in")

    def _check_live(self):
        """Between and during non-move steps: a controller error or lost status ends the run."""
        frame, age = self.panel.frame, time.monotonic() - self.panel.frame_at
        if frame is None or age > 4.5:
            self.panel.stop("the controller's status went quiet")
            raise MoveFailed("the controller's status went quiet")
        if frame["err"]:
            self.panel.stop(f"the controller reported error {frame['err']}")
            raise MoveFailed(f"the controller reported error {frame['err']}")

    def _speed(self, program, s):
        speeds = program["speed"]
        base = s.get("speed", speeds["tcp"] if s["kind"] == "line" else speeds["joint"])
        speed = base * self.override / 100.0
        capped = s["kind"] == "line" and (self.panel.reduced_speed or not program.get("fullSpeed")) and \
            speed > REDUCED_SPEED_MM_S
        return (REDUCED_SPEED_MM_S if capped else speed), capped

    def _do(self, program, s, path):
        kind = s["type"]
        dry = self.virtual is not None
        if kind == "comment":
            return
        if kind == "halt":
            if dry:
                self.log("dry.halt", message=s["message"])
                return
            raise HaltHere(s["message"])
        if kind == "learned":
            self._learned(program, s, path)
            return
        if kind == "move":
            s = self._slotted(s)
            speed, capped = self._speed(program, s)
            if capped:
                why = "the program isn't marked for full speed" if not program.get("fullSpeed") else \
                    "the server's reduced speed is on"
                self.log("speed.capped", speed=REDUCED_SPEED_MM_S, reason=f"reduced speed: {why}")
            if s["kind"] == "planned":
                self._planned(program, s, speed)
                return
            start = self.virtual if dry else self.panel.current_joints()
            if not dry and self.anchor is not None and self.anchor[0] == s["id"]:
                start = self._regain(start, speed)
            plan = self.panel.plan_move(s, start, speed, program["speed"]["ori"] * self.override / 100.0, dry=dry)
            self.log("move.planned", text=plan.text, durationS=round(plan.duration, 2))
            if dry:
                self._guards(plan)
                self.virtual = plan.goal
                self.total += plan.duration
                return
            if plan.kind == "line" and plan.poses is not None:
                self.anchor = (s["id"], plan.poses, plan.tool)  # where this line runs, to rejoin it after a pause
            self._execute(plan)
            self.anchor = None
            return
        if kind == "gripper":
            if dry:
                g = self.panel.cell.gripper(s["gripper"])
                self.log("dry.would-grip", gripper=s["gripper"], action=s["action"],
                         note="assumed it confirms" + (" and holds a part" if s["action"] == "grasp" else ""))
                self.total += g["seconds"]
                return
            self.panel.grip(s["gripper"], s["action"], s.get("part"), sleep=self._sleep_checking,
                            log=lambda event, **d: self.log(event, **d))
            return
        if kind == "count":
            self._count(s)
            return
        if kind == "home":
            self._home(program, s)
            return
        if kind == "notify":
            if dry:
                self.log("dry.would-notify", message=s["message"])
                return
            call = self.panel.call_owner(s["message"], program=program["name"], path=dotted(path))
            self.log("owner.called", message=s["message"], call=call["id"])
            return
        if kind == "wait" and "seconds" in s:
            if dry:
                self.total += s["seconds"]
                return
            end = time.monotonic() + s["seconds"]
            while time.monotonic() < end:
                self._sleep_checking(min(POLL_S, end - time.monotonic()))
            return
        if kind == "wait":
            if dry:
                self.log("dry.assumed", input=s["input"], value=s["value"])
                return
            started = time.monotonic()
            self.log("io.wait", input=s["input"], value=s["value"], timeout=s["timeout"])
            while True:
                if self._read_input(s["input"], path) == s["value"]:
                    self.log("io.waited", input=s["input"], seconds=round(time.monotonic() - started, 2))
                    return
                if time.monotonic() - started > s["timeout"]:
                    raise ProgramError(f"timed out after {s['timeout']:g} s waiting for {s['input']} to be {s['value']}")
                self._sleep_checking(POLL_S)
        if kind == "set":
            if dry:
                self.log("dry.would-set", output=s["output"], value=s["value"])
                return
            self.panel.write_output(s["output"], s["value"])
            self.log("io.set", output=s["output"], value=s["value"])
            return
        if kind == "pulse":
            if dry:
                self.log("dry.would-pulse", output=s["output"], seconds=s["seconds"])
                self.total += s["seconds"]
                return
            self.panel.write_output(s["output"], 1)
            self.log("io.set", output=s["output"], value=1)
            try:
                end = time.monotonic() + s["seconds"]
                while time.monotonic() < end:
                    self._sleep_checking(min(POLL_S, end - time.monotonic()))
            finally:
                self.panel.write_output(s["output"], 0)
                self.log("io.set", output=s["output"], value=0)

    # -- patterns and safe home ------------------------------------------------------------------------------------
    def _counter(self, name):
        """Where a pattern's counter stands (a dry run's copy, in a dry run)."""
        if self.counts is not None:
            return self.counts.get(name, 1)
        return self.panel.counters.slot(name)

    def _slotted(self, s):
        """A move to a pattern's next slot names the slot its counter stands at; a pattern with no slot left fails."""
        if s.get("pattern") is None or s.get("slot") is not None:
            return s
        name = s["pattern"]
        total = self.panel.cell.pattern_geometry(name)["count"]
        slot = self._counter(name)
        if slot > total:
            raise ProgramError(f"{name} is used up: all {total} of its slots are done. Reset its counter (Cell, Patterns), "
                               "or count it back to slot 1.")
        return dict(s, slot=slot)

    def _count(self, s):
        name = s["pattern"]
        total = self.panel.cell.pattern_geometry(name)["count"]
        was = self._counter(name)
        if "to" in s:
            if s["to"] > total + 1:
                raise ProgramError(f"{name} has {total} slots; its counter can't go to {s['to']}.")
            slot = s["to"]
        else:
            slot = min(was, total) + 1  # one past the last: used up
        if self.counts is not None:
            self.counts[name] = slot
        else:
            self.panel.counters.set(name, slot)
        self.log("pattern.count", pattern=name, slot=slot, was=was, of=total, **({"usedUp": True} if slot > total else {}))

    def _home(self, program, s):
        """Safe home: back along the tool, as far as it can go clear of everything up to `retract` mm, then a joint
        move to the target, each planned and checked as any move is. A pause during either re-plans the rest of it: the
        rest of the retract, or the joint move."""
        panel, dry = self.panel, self.virtual is not None
        start = self.virtual if dry else panel.current_joints()
        if s["retract"] > 0 and not (self.homing and self.homing[0] == s["id"] and self.homing[2]):
            if not (self.homing and self.homing[0] == s["id"]):
                goal = kin.mul(panel.tool_point(start), kin.offset_matrix([0.0, 0.0, -s["retract"]]))
                self.homing = (s["id"], goal, False)
            speed, capped = self._speed(program, {"kind": "line"})
            if capped:
                self.log("speed.capped", speed=REDUCED_SPEED_MM_S, reason="reduced speed: the retract is a straight line")
            plan, got, why = panel.plan_retract(start, self.homing[1], speed, program["speed"]["ori"] * self.override / 100.0,
                                                dry=dry)
            self.log("home.retract", wantedMm=s["retract"], gotMm=round(got, 1), text=plan.text if plan else "",
                     **({"short": why} if why else {}))
            if plan is not None and plan.segments:
                if dry:
                    self._guards(plan)
                    self.virtual = plan.goal
                    self.total += plan.duration
                else:
                    self._execute(plan)
                start = self.virtual if dry else panel.current_joints()
            self.homing = (s["id"], self.homing[1], True)
        speed = s.get("speed", program["speed"]["joint"]) * self.override / 100.0
        plan = panel.plan_move({"type": "move", "kind": "joint", "target": s["target"], "id": s["id"]}, start, speed,
                               dry=dry)
        self.log("move.planned", text=plan.text, durationS=round(plan.duration, 2))
        if dry:
            self._guards(plan)
            self.virtual = plan.goal
            self.total += plan.duration
        else:
            self._execute(plan)
        self.homing = None

    # -- steps the brain proposes ------------------------------------------------------------------------------------
    def _tool_cap(self, program):
        """The tool point's speed limit for what the brain proposes (mm/s): reduced speed unless marked full speed and
        the server's reduced speed is off."""
        return plannermod.MAX_TOOL_SPEED_MM_S if program.get("fullSpeed") and not self.panel.reduced_speed \
            else REDUCED_SPEED_MM_S

    def _ask_brain(self, kind, fields):
        """Ask the brain, waiting as long as its deadline allows but not a moment past a STOP or pause, which raise
        Interrupted (a reply that comes later is dropped)."""
        link = self.panel.brain
        if link is None:
            raise ProgramError("the brain is off (the server was started with --brain off)")
        with self.lock:
            shown = self.step_text
            self.step_text = f"{shown} \u00b7 asking the brain"
        try:
            return link.ask(kind, fields, should_abort=self._abort)
        except brain_link.Abandoned as e:
            raise Interrupted(e.reason)
        finally:
            with self.lock:
                self.step_text = shown

    def _planned(self, program, s, speed):
        """A planned move: the brain plans a path from where the arm is to the target's joints, around what's in the
        cell; the shell checks all of it, then the executor runs it. A dry run asks the brain too, from where the run
        would be, and checks the path just the same. Guarded zones whose interlocks hold are left out of the world the
        brain is told and the shell checks against; the run watches those the path enters, or starts in. If the arm is
        in one whose interlocks don't hold, the brain isn't asked (Panel.zones_for_brain)."""
        panel = self.panel
        state = panel.observe(virtual=self.virtual)
        start = state["joints"]
        goal, label = panel.goal_joints(s, start)
        if max(abs(a - b) for a, b in zip(start, goal)) <= safety.GOAL_TOLERANCE_DEG:
            self.log("move.planned", text=f"Already at {label}", durationS=0.0)
            return
        passable = panel.zones_for_brain(start, dry=self.virtual is not None)
        world = panel.world(passable)
        aim = {"joints": [round(v, 4) for v in goal], "target": s.get("target") or label,
               "tool": kin.matrix_to_pose(panel.tool_point(goal), 4)}
        if s.get("pattern") is not None:
            aim.update(pattern=s["pattern"], slot=s["slot"])
        fields = {"state": brain_state(state), "goal": aim, "world": world, "limits": brain_limits(),
                  "timeoutS": panel.brain.plan_timeout if panel.brain else 0}
        _, reply = self._ask_brain("plan", fields)
        if not reply["ok"]:
            raise ProgramError(f"the brain found no path to {label}: {reply['reason']}")
        plan = safety.check_path(panel.planner(passable), reply["path"], start, speed, panel.cell.tool_matrix(),
                                 self._tool_cap(program), label, goal=goal)
        plan.guards = panel.zones_entered(plan.path, passable)
        self.log("brain.plan", waypoints=len(reply["path"]), note=reply["note"], text=plan.text,
                 durationS=round(plan.duration, 2))
        if self.virtual is not None:
            self._guards(plan)
            self.virtual = plan.goal
            self.total += plan.duration
            return
        q, age = panel.latest_joints()
        safety.check_still("plan", start, q, age, world["cellRevision"], panel.cell.revision)
        self._still_open(plan)
        self._execute(plan)

    def _still_open(self, plan):
        """Just before a path from the brain runs: the guarded zones it depends on must still be open."""
        dropped = self.panel.guard_dropped(getattr(plan, "guards", None))
        if dropped:
            raise plannermod.ZoneClosed(f"the {dropped[0]} zone closed while the brain worked out a way into it: "
                                        f"{dropped[1]}", dropped[0])

    def _learned(self, program, s, path):
        """A learned step: the brain's skill acts in short bursts, each asked for with the arm's latest state (at most
        ACT_PERIOD_S apart), checked whole by the shell and run by the executor, until the skill says it's done (then
        the step's check, if any, must read true), gives up, or runs out of time. A dry run only makes sure the brain
        has the skill, and takes the arm to end where it began."""
        panel, skill = self.panel, s["skill"]
        if self.virtual is not None:
            panel.check_brain(False, [skill])
            self.log("dry.assumed", skill=skill, note="assumed to end where it began")
            return
        speed = s.get("speed", program["speed"]["joint"]) * self.override / 100.0
        tool = panel.cell.tool_matrix()
        anchor = panel.anchor_point(s["near"])
        self.episodes += 1
        episode = f"{self.run_tag}-{re.sub(r'[^A-Za-z0-9_.:-]', '_', str(s['id']))[:40]}-{self.episodes}"
        deadline = time.monotonic() + s["seconds"]
        after, asked, tick, outcome, failure = None, 0.0, 0, "faulted", ""
        self.log("skill.start", skill=skill, near=s["near"], within=s["within"], seconds=s["seconds"])
        try:
            while True:
                why = self._abort()
                if why:
                    raise Interrupted(why)
                self._check_live()
                if time.monotonic() > deadline:
                    outcome = "timeout"
                    raise ProgramError(f"the skill {skill} didn't finish within {s['seconds']:g} s")
                while True:  # at most 10 a second, by perf_counter (monotonic() ticks in 15.6 ms on Windows)
                    wait = asked + ACT_PERIOD_S - time.perf_counter()
                    if wait <= 0:
                        break
                    self._sleep_checking(wait)
                state = panel.observe(after=after, inputs=True)
                passable = panel.zones_for_brain(state["joints"])  # refused if the arm is in a closed zone
                world = panel.world(passable)
                fields = {"skill": skill, "params": s.get("params", {}), "episode": episode, "tick": tick,
                          "state": brain_state(state), "anchor": {"xyz": [round(v, 3) for v in anchor]},
                          "withinMm": s["within"], "world": world, "limits": brain_limits(burst=True),
                          "timeoutS": brain_link.ACT_TIMEOUT_S,
                          "remainingS": round(max(0.0, deadline - time.monotonic()), 2)}
                asked = time.perf_counter()
                _, reply = self._ask_brain("act", fields)
                action = reply["action"]
                if action["kind"] == "done":
                    check = s.get("check")
                    if check:
                        got = self._read_input(check["input"], path)
                        if got != check["value"]:
                            outcome = "failed"
                            raise ProgramError(f"the skill {skill} said it was done, but {check['input']} reads {got}, "
                                               f"not {check['value']}")
                    outcome = "done"
                    self.log("skill.done", skill=skill, bursts=tick, note=action["note"])
                    return
                if action["kind"] == "fail":
                    outcome = "failed"
                    raise ProgramError(f"the skill {skill} gave up: {action['reason']}")
                if action["kind"] == "hold":
                    self.log("skill.hold", tick=tick, seconds=action["seconds"])
                    self._sleep_checking(action["seconds"])
                else:
                    what = f"burst {tick + 1}"
                    plan = safety.check_path(panel.planner(passable), action["path"], state["joints"],
                                             min(speed, action.get("speed", speed)), tool, self._tool_cap(program),
                                             f"{skill}'s next pose", what=what, near=(anchor, s["near"]),
                                             within=s["within"], burst=True)
                    plan.guards = panel.zones_entered(plan.path, passable)
                    q, age = panel.latest_joints()
                    safety.check_still(what, state["joints"], q, age, world["cellRevision"], panel.cell.revision)
                    self._still_open(plan)
                    self.log("skill.burst", tick=tick, waypoints=len(action["path"]) - 1, text=plan.text)
                    self._execute(plan)
                after = time.monotonic()
                tick += 1
        except BaseException as e:
            if isinstance(e, Interrupted):
                outcome = "stopped" if self.request == "stop" else "paused"
            elif isinstance(e, (safety.Refused, bp.ProtocolError)):
                outcome = "refused"
            elif isinstance(e, plannermod.ZoneClosed):
                outcome = "failed"
            failure = str(e)[:bp.MAX_TEXT]
            raise
        finally:
            if panel.brain is not None:
                panel.brain.end({"episode": episode, "skill": skill, "outcome": outcome, "reason": failure})
            self.log("skill.end", skill=skill, outcome=outcome, bursts=tick)

    def _regain(self, start, speed):
        """Before resuming an interrupted straight line: move straight back to the nearest point on it (the arm
        may have left it by the STOP's swing, the enable shift of about 0.5 mm, or a jog while paused), as RAPID's
        regain does. Returns the joints to carry on from."""
        _, (t0, target), tool = self.anchor
        here = self.panel.tool_point(start)
        a, b, p = kin.position(t0), kin.position(target), kin.position(here)
        d = [y - x for x, y in zip(a, b)]
        dd = sum(v * v for v in d)
        f = 0.0 if dd < 1e-12 else max(0.0, min(1.0, sum((pi - ai) * di for pi, ai, di in zip(p, a, d)) / dd))
        back = kin.interpolate(t0, target, f)
        off_mm, off_deg = kin.pose_distance(here, back)
        if off_mm < 0.02 and off_deg < 0.02:
            return start
        plan = self.panel.plan_guarded(lambda p: p.line(start, back, tool, min(speed, 20.0), 10.0,
                                                        label="the interrupted line"), start)
        self.log("move.regain", offMm=round(off_mm, 3), offDeg=round(off_deg, 3), text=plan.text)
        self._execute(plan)
        return self.panel.current_joints()

    def _sleep_checking(self, seconds):
        time.sleep(max(0.0, seconds))
        why = self._abort()
        if why:
            raise Interrupted(why)
        self._check_live()

    def _read_input(self, name, path):
        line, bit = io_name(name, "input")
        return self.panel.read_input(line, bit)


class HaltHere(Exception):
    def __init__(self, message):
        super().__init__(message)
        self.message = message


def describe(s):
    kind = s["type"]
    if kind == "move":
        extra = " with an offset" if s.get("offset") else ""
        if s.get("pattern") is not None:
            to = f"{s['pattern']} slot {s['slot']}" if s.get("slot") is not None else f"{s['pattern']}'s next slot"
        else:
            to = s["target"]
        if s["kind"] == "planned":
            return f"Planned move to {to}{extra}"
        return f"{'Straight line' if s['kind'] == 'line' else 'Joint move'} to {to}{extra}"
    if kind == "learned":
        return f"Learned skill {s['skill']} near {s['near']}"
    if kind == "wait":
        return f"Wait {s['seconds']:g} s" if "seconds" in s else f"Wait for {s['input']} = {s['value']}"
    if kind == "set":
        return f"Set {s['output']} to {s['value']}"
    if kind == "pulse":
        return f"Pulse {s['output']} for {s['seconds']:g} s"
    if kind == "gripper":
        if s["action"] == "grasp":
            mass = (s.get("part") or {}).get("mass")
            return f"Grasp with {s['gripper']}" + (f", a {mass:g} kg part" if mass else "")
        return f"{'Open' if s['action'] == 'open' else 'Close'} {s['gripper']}"
    if kind == "count":
        if s.get("to") is None:
            return f"Count {s['pattern']} on to its next slot"
        return f"Count {s['pattern']} back to slot 1" if s["to"] == 1 else f"Set {s['pattern']}'s counter to slot {s['to']}"
    if kind == "retry":
        return f"Retry: up to {s['tries']} tries"
    if kind == "fallback":
        return "Fallback: if a step fails, the steps under If it fails run instead"
    if kind == "home":
        if s["retract"]:
            return f"Safe home to {s['target']}: back up to {s['retract']:g} mm along the tool, then a joint move"
        return f"Safe home to {s['target']}: a joint move"
    if kind == "notify":
        return f"Call the owner: {s['message']}"
    if kind == "halt":
        return f"Halt: {s['message']}"
    if kind == "comment":
        return s.get("text", "")
    return kind
