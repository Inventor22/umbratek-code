"""Programs, and the runner that runs them on the server with its own supervision and an event log.

A program is a tree of steps, saved as JSON in cell/programs/<name>.json:
    move     {"kind": "joint" | "line" | "planned", "target": name, "offset": {...}?, "speed": number?}  (offsets:
             cell.apply_offset). A planned move's path comes from the brain, solved afresh each run around what's in
             the cell, and ends at the target's joints (as taught, so in the taught configuration).
    learned  {"skill": name, "near": target, "within": mm, "seconds": s, "params": {...}?, "speed": deg/s?,
              "check": {"input": ..., "value": 0 | 1}?}  The brain's skill acts in short bursts, at most 10 a second,
             the tool point staying within `within` mm of the target `near`, until it says it's done (then `check`, if
             given, must read true), gives up, or runs out of `seconds`.
    wait     {"seconds": s} or {"input": "tool.N" | "controller.N", "value": 0 | 1, "timeout": s}
    set      {"output": "tool.N" | "controller.N", "value": 0 | 1}
    pulse    {"output": ..., "seconds": s}
    if       {"input": ..., "value": 0 | 1, "then": [steps], "else": [steps]}
    repeat   {"count": n, "steps": [steps]}
    call     {"program": name}
    comment  {"text": ...}
    halt     {"message": ...}  (pauses the run until the operator resumes)
Every step has a stable "id". Moves name targets, so re-teaching a target or recalibrating its frame changes every
move to it. Speeds: tool mm/s for lines, joint deg/s for joint, planned and learned moves, the program's defaults
where a step has none, times the override; lines, and whatever the brain proposes, are capped at REDUCED_SPEED_MM_S
at the tool (ISO 10218-1's reduced speed) unless the program says "fullSpeed": true.

What the brain proposes, the safety shell (safety.py) checks whole before anything moves, and refuses with its reasons
if any part fails; a refusal faults the run. The brain only proposes: it can't start, pause, resume or stop anything,
and a STOP ends the run as stopped whatever the step was doing, waiting on the brain included.

The runner (docs/capabilities.md P2-P5): start (from the top or a chosen step; run, step or dry run), pause (STOP,
then resume re-plans the interrupted step from wherever the arm is), resume, stop, step, reset, speed override. It
runs in its own thread, so a run carries on with the page closed; manual moves started from the page instead stop if
the page goes quiet. Every event goes to cell/logs/<date>/<time>-<program>.jsonl, a file of each run's own: another
run of the same program started in the same second gets <time>-<program>~2.jsonl, then ~3 and so on (new_log).
Standard library only.
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

STEP_TYPES = ("move", "wait", "set", "pulse", "if", "repeat", "call", "comment", "halt", "learned")
MOVE_KINDS = ("joint", "line", "planned")
IO = re.compile(r"^(tool|controller)\.(\d{1,2})$")
MAX_DEPTH = 6
MAX_STEPS = 2000
REDUCED_SPEED_MM_S = 250.0
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


def where(path):
    return "step " + ".".join(str(i + 1) for i in path)


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

    def steps(items, path):
        if not isinstance(items, list):
            raise ProgramError(f"{where(path) if path else 'The program'}: steps must be a list.")
        return [step(s, path + [i]) for i, s in enumerate(items)]

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


def programs_using(programs, target):
    """The saved programs with a move to `target`, or a learned step near it, as 'program <name>'."""
    def uses(steps):
        for s in steps if isinstance(steps, list) else []:
            if not isinstance(s, dict):
                continue
            if (s.get("type") == "move" and s.get("target") == target) or \
                    (s.get("type") == "learned" and s.get("near") == target):
                return True
            if any(uses(s.get(key)) for key in ("steps", "then", "else")):
                return True
        return False
    out = []
    for name in programs.names():
        try:
            if uses(programs.load(name).get("steps")):
                out.append(f"program {name}")
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
        self.anchor = None  # (step id, line poses, tool) of a straight line under way
        self.total = 0.0
        try:
            if self.mode == "dry":
                self.virtual = self.panel.current_joints()
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
        except Faulted as e:
            with self.lock:
                self.state, self.reason, self.hit = "faulted", e.reason, drawable_hit(e.hit)
                self.log("run.faulted", reason=e.reason)
        except Exception as e:  # a bug: fault the run rather than die silently
            self.panel.stop("the program runner failed")
            with self.lock:
                self.state, self.reason = "faulted", f"internal error: {e!r}"
                self.log("run.faulted", reason=self.reason)

    def _steps(self, program, steps, path, depth):
        for i, s in enumerate(steps):
            self._step(program, s, path + [i], depth)

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
                self.log("repeat", iteration=n + 1, of=s["count"], path=".".join(str(i + 1) for i in path))
                self._steps(program, s["steps"], path, depth)
            return
        if kind == "if":
            value = self._read_input(s["input"], path)
            branch = "then" if value == s["value"] else "else"
            self.log("if", input=s["input"], read=value, branch=branch, path=".".join(str(i + 1) for i in path))
            self._steps(program, s[branch], path, depth)
            return
        if kind == "call":
            callee = check_program(self.programs.load(s["program"]), self.panel.cell, self.programs.load)
            self.log("call", program=s["program"], path=".".join(str(i + 1) for i in path))
            self._steps(callee, callee["steps"], path, depth + 1)
            return
        self._leaf(program, s, path)

    def _leaf(self, program, s, path):
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
                self._pause_here(e.reason, resume_same=True)
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
                if self.virtual is None:
                    self.panel.stop(f"{where(path)} failed")
                raise Faulted(f"{where(path)}: {e}", path, getattr(e, "hit", None))

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
        """The executor's check, a few times a second: a pause or stop asked for, or (for a manual move) a quiet page."""
        if self.request == "stop":
            return self.reason or "the operator pressed Stop"
        if self.request == "pause":
            return "the operator pressed Pause"
        if self.manual and time.monotonic() - self.panel.heard_at > PAGE_SILENT_S:
            self.request, self.reason = "stop", "the jog page went quiet"
            return "the jog page went quiet"
        return None

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
        capped = s["kind"] == "line" and not program.get("fullSpeed") and speed > REDUCED_SPEED_MM_S
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
            speed, capped = self._speed(program, s)
            if capped:
                self.log("speed.capped", speed=REDUCED_SPEED_MM_S, reason="reduced speed: the program isn't marked "
                                                                         "for full speed")
            if s["kind"] == "planned":
                self._planned(program, s, speed)
                return
            start = self.virtual if dry else self.panel.current_joints()
            if not dry and self.anchor is not None and self.anchor[0] == s["id"]:
                start = self._regain(start, speed)
            plan = self.panel.plan_move(s, start, speed, program["speed"]["ori"] * self.override / 100.0)
            self.log("move.planned", text=plan.text, durationS=round(plan.duration, 2))
            if dry:
                self.virtual = plan.goal
                self.total += plan.duration
                return
            if plan.kind == "line" and plan.poses is not None:
                self.anchor = (s["id"], plan.poses, plan.tool)  # where this line runs, to rejoin it after a pause
            self.panel.executor.execute(plan, self._abort, lambda event, **d: self.log(event, **d))
            self.anchor = None
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

    # -- steps the brain proposes ------------------------------------------------------------------------------------
    def _tool_cap(self, program):
        """The tool point's speed limit for what the brain proposes (mm/s): reduced speed unless marked full speed."""
        return plannermod.MAX_TOOL_SPEED_MM_S if program.get("fullSpeed") else REDUCED_SPEED_MM_S

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
        would be, and checks the path just the same."""
        panel = self.panel
        state = panel.observe(virtual=self.virtual)
        start = state["joints"]
        goal, label = panel.goal_joints(s, start)
        if max(abs(a - b) for a, b in zip(start, goal)) <= safety.GOAL_TOLERANCE_DEG:
            self.log("move.planned", text=f"Already at {label}", durationS=0.0)
            return
        world = panel.world()
        fields = {"state": brain_state(state),
                  "goal": {"joints": [round(v, 4) for v in goal], "target": s["target"],
                           "tool": kin.matrix_to_pose(panel.tool_point(goal), 4)},
                  "world": world, "limits": brain_limits(), "timeoutS": panel.brain.plan_timeout if panel.brain else 0}
        _, reply = self._ask_brain("plan", fields)
        if not reply["ok"]:
            raise ProgramError(f"the brain found no path to {label}: {reply['reason']}")
        plan = safety.check_path(panel.planner(), reply["path"], start, speed, panel.cell.tool_matrix(),
                                 self._tool_cap(program), label, goal=goal)
        self.log("brain.plan", waypoints=len(reply["path"]), note=reply["note"], text=plan.text,
                 durationS=round(plan.duration, 2))
        if self.virtual is not None:
            self.virtual = plan.goal
            self.total += plan.duration
            return
        q, age = panel.latest_joints()
        safety.check_still("plan", start, q, age, world["cellRevision"], panel.cell.revision)
        panel.executor.execute(plan, self._abort, lambda event, **d: self.log(event, **d))

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
                world = panel.world()
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
                    plan = safety.check_path(panel.planner(), action["path"], state["joints"],
                                             min(speed, action.get("speed", speed)), tool, self._tool_cap(program),
                                             f"{skill}'s next pose", what=what, near=(anchor, s["near"]),
                                             within=s["within"], burst=True)
                    q, age = panel.latest_joints()
                    safety.check_still(what, state["joints"], q, age, world["cellRevision"], panel.cell.revision)
                    self.log("skill.burst", tick=tick, waypoints=len(action["path"]) - 1, text=plan.text)
                    panel.executor.execute(plan, self._abort, lambda event, **d: self.log(event, **d))
                after = time.monotonic()
                tick += 1
        except BaseException as e:
            if isinstance(e, Interrupted):
                outcome = "stopped" if self.request == "stop" else "paused"
            elif isinstance(e, (safety.Refused, bp.ProtocolError)):
                outcome = "refused"
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
        plan = self.panel.planner().line(start, back, tool, min(speed, 20.0), 10.0, label="the interrupted line")
        self.log("move.regain", offMm=round(off_mm, 3), offDeg=round(off_deg, 3), text=plan.text)
        self.panel.executor.execute(plan, self._abort, lambda event, **d: self.log(event, **d))
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
        if s["kind"] == "planned":
            return f"Planned move to {s['target']}{extra}"
        return f"{'Straight line' if s['kind'] == 'line' else 'Joint move'} to {s['target']}{extra}"
    if kind == "learned":
        return f"Learned skill {s['skill']} near {s['near']}"
    if kind == "wait":
        return f"Wait {s['seconds']:g} s" if "seconds" in s else f"Wait for {s['input']} = {s['value']}"
    if kind == "set":
        return f"Set {s['output']} to {s['value']}"
    if kind == "pulse":
        return f"Pulse {s['output']} for {s['seconds']:g} s"
    if kind == "halt":
        return f"Halt: {s['message']}"
    if kind == "comment":
        return s.get("text", "")
    return kind
