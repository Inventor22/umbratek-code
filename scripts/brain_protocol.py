"""The interface between the spine and the brain, version 1: its messages, their limits, and the checks both ends make.

The spine (jog_server.py and its runner) owns the arm. The brain is a separate process that proposes motion: a path
for a planned move, or the next short burst of a learned skill. The brain is a small HTTP server on this PC (a
loopback address); the spine is its only client and asks it one question at a time, as POST /v1/<type> with a JSON
object each way. The brain never speaks unless asked, and nothing it proposes reaches the controller until the
spine's safety shell (safety.py) has checked all of it. docs/brain-interface.md specifies the protocol; this module
is that specification in code, so the spine and any brain written in Python share one definition. Standard library
only, so it runs on Windows and in WSL2 alike.

Every message is a JSON object with "v" (the protocol version), "type" ("hello", "plan", "act" or "end") and "id"
(chosen by the spine, echoed in the reply). Replies to "plan" and "act" also echo "stateSeq", the sequence number of
the status frame the spine described, so a reply is tied to the state it was worked out from. Joint angles are in
degrees and lengths in millimetres, as everywhere in the spine. A peer ignores fields it doesn't know; a change to the
meaning of a field it does know needs a new version.
"""
from __future__ import annotations

import json
import math
import re

VERSION = 1
TYPES = ("hello", "plan", "act", "end")
PATHS = {kind: f"/v{VERSION}/{kind}" for kind in TYPES}
MAX_BYTES = 1 << 20  # one message, either way
AXES = 6
JOINT_RANGE_DEG = 360.0  # any joint angle in a message; the shell holds paths to the planning limits
MAX_PLAN_WAYPOINTS = 64  # a planned path, its start and goal included
MAX_BURST_WAYPOINTS = 10  # a learned skill's burst, after the waypoint it starts from
HOLD_MAX_S = 1.0  # the longest a skill may hold still before it's asked again
MAX_TEXT = 300  # notes, reasons and descriptions are cut to this
MAX_PARAMS_BYTES = 2000  # a learned step's parameters for its skill
MAX_SKILLS = 100
MAX_ZONES = 200
MAX_CAPSULES = 8
MAX_DEPTH = 20
ACTIONS = ("move", "hold", "done", "fail")
OUTCOMES = ("done", "failed", "refused", "stopped", "paused", "faulted", "timeout")
ID = re.compile(r"^[A-Za-z0-9_.:-]{1,80}$")
NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_.-]{0,39}$")  # skills, as the cell's names


class ProtocolError(Exception):
    """A message that breaks the protocol: not JSON, another version, malformed, or not the reply to the request."""


# ---- encoding ------------------------------------------------------------------------------------------------------

def _refuse_constant(name):
    raise ValueError(f"{name} isn't a number JSON allows")


def encode(message):
    """A message as UTF-8 JSON, refusing NaN and infinities (which JSON doesn't have) and anything over MAX_BYTES."""
    try:
        data = json.dumps(message, allow_nan=False, separators=(",", ":")).encode("utf-8")
    except (TypeError, ValueError) as e:
        raise ProtocolError(f"can't be sent as JSON ({e})")
    if len(data) > MAX_BYTES:
        raise ProtocolError(f"is {len(data)} bytes, over the {MAX_BYTES}-byte limit")
    return data


def decode(data):
    """A message from UTF-8 JSON: a JSON object of at most MAX_BYTES, nested at most MAX_DEPTH deep, every number
    finite (Python's json module would otherwise take NaN, Infinity and 1e999)."""
    if len(data) > MAX_BYTES:
        raise ProtocolError(f"is over the {MAX_BYTES}-byte limit")
    try:
        message = json.loads(data.decode("utf-8"), parse_constant=_refuse_constant)
    except (UnicodeDecodeError, ValueError, RecursionError) as e:
        raise ProtocolError(f"isn't JSON ({str(e)[:80]})")
    if not isinstance(message, dict):
        raise ProtocolError(f"isn't a JSON object ({_show(message)})")
    _check_finite(message, 0)
    return message


def _check_finite(value, depth):
    if depth > MAX_DEPTH:
        raise ProtocolError(f"nests deeper than {MAX_DEPTH} levels")
    if isinstance(value, float) and not math.isfinite(value):
        raise ProtocolError("has a number that isn't finite")
    if isinstance(value, dict):
        for item in value.values():
            _check_finite(item, depth + 1)
    elif isinstance(value, list):
        for item in value:
            _check_finite(item, depth + 1)


def message(kind, request_id, **fields):
    """A request from the spine."""
    return dict({"v": VERSION, "type": kind, "id": request_id}, **fields)


def reply(request, **fields):
    """The reply to `request`: its version, type and id, and for plan and act the state's sequence number."""
    out = {"v": VERSION, "type": request["type"], "id": request["id"]}
    if request["type"] in ("plan", "act"):
        out["stateSeq"] = request["state"]["seq"]
    out.update(fields)
    return out


# ---- checking fields ---------------------------------------------------------------------------------------------

def _show(value):
    try:
        shown = json.dumps(value)
    except (TypeError, ValueError):
        shown = repr(value)
    return shown if len(shown) <= 40 else shown[:37] + "..."


def number(value, what, low=None, high=None):
    """A finite number (never a boolean), optionally within [low, high]."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProtocolError(f"{what} is {_show(value)}, not a number")
    if (low is not None and value < low) or (high is not None and value > high):
        raise ProtocolError(f"{what} is {value:g}, outside {low:g} to {high:g}")
    return float(value)


def integer(value, what, low=0, high=2 ** 53):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ProtocolError(f"{what} is {_show(value)}, not a whole number from {low} to {high}")
    return value


def text(value, what, required=True):
    """A string, cut to MAX_TEXT characters; "" when it may be left out and is."""
    if value is None and not required:
        return ""
    if not isinstance(value, str) or (required and not value.strip()):
        raise ProtocolError(f"{what} is {_show(value)}, not some text")
    return value[:MAX_TEXT]


def name(value, what):
    if not isinstance(value, str) or not NAME.match(value):
        raise ProtocolError(f"{what} is {_show(value)}, not a name (up to 40 letters, digits, '_', '-' or '.', "
                            "starting with a letter)")
    return value


def identifier(value, what):
    if not isinstance(value, str) or not ID.match(value):
        raise ProtocolError(f"{what} is {_show(value)}, not an id (up to 80 letters, digits, '_', '-', '.' or ':')")
    return value


def triple(value, what, limit=10000.0):
    if not isinstance(value, list) or len(value) != 3:
        raise ProtocolError(f"{what} is {_show(value)}, not 3 numbers")
    return [number(v, f"{what}[{i}]", -limit, limit) for i, v in enumerate(value)]


def joints(value, what="joints"):
    """Six joint angles in degrees."""
    if not isinstance(value, list) or len(value) != AXES:
        raise ProtocolError(f"{what} is {_show(value)}, not {AXES} joint angles")
    return [number(v, f"{what}[{i}]", -JOINT_RANGE_DEG, JOINT_RANGE_DEG) for i, v in enumerate(value)]


def path(value, what, most):
    """A path: 2 to `most` waypoints, each six joint angles in degrees; the first is where it starts."""
    if not isinstance(value, list) or not 2 <= len(value) <= most:
        count = len(value) if isinstance(value, list) else None
        raise ProtocolError(f"{what} has {count if count is not None else _show(value)} waypoints; it needs 2 to {most}")
    return [joints(q, f"{what}[{k}]") for k, q in enumerate(value)]


def pose(value, what):
    if not isinstance(value, dict):
        raise ProtocolError(f"{what} is {_show(value)}, not a pose")
    return {"xyz": triple(value.get("xyz"), f"{what}.xyz"), "rpy": triple(value.get("rpy"), f"{what}.rpy", 360.0)}


def state(value, what="state"):
    """The arm's state as the spine describes it: a status frame's sequence number and age, the measured joints,
    the active tool point's pose in the base frame, the digital inputs (for act) and whether it's a dry run's."""
    if not isinstance(value, dict):
        raise ProtocolError(f"{what} is {_show(value)}, not an object")
    out = {"seq": integer(value.get("seq"), f"{what}.seq"), "ageS": number(value.get("ageS"), f"{what}.ageS", 0),
           "joints": joints(value.get("joints"), f"{what}.joints"), "tool": pose(value.get("tool"), f"{what}.tool"),
           "virtual": value.get("virtual", False) is True}
    if value.get("inputs") is not None:
        inputs = value["inputs"]
        if not isinstance(inputs, dict):
            raise ProtocolError(f"{what}.inputs is {_show(inputs)}, not an object")
        out["inputs"] = {line: integer(inputs.get(line), f"{what}.inputs.{line}", 0, 0xFFFF)
                         for line in ("tool", "controller")}
    return out


def world(value, what="world"):
    """What the spine knows of the cell: the active tool's point and shape, and the keep-out zones in the base frame."""
    if not isinstance(value, dict):
        raise ProtocolError(f"{what} is {_show(value)}, not an object")
    tool = value.get("tool")
    if not isinstance(tool, dict):
        raise ProtocolError(f"{what}.tool is {_show(tool)}, not an object")
    capsules = tool.get("capsules", [])
    if not isinstance(capsules, list) or len(capsules) > MAX_CAPSULES:
        raise ProtocolError(f"{what}.tool.capsules should be a list of at most {MAX_CAPSULES}")
    zones = value.get("zones", [])
    if not isinstance(zones, list) or len(zones) > MAX_ZONES:
        raise ProtocolError(f"{what}.zones should be a list of at most {MAX_ZONES}")
    out_capsules = []
    for i, c in enumerate(capsules):
        where = f"{what}.tool.capsules[{i}]"
        if not isinstance(c, dict):
            raise ProtocolError(f"{where} is {_show(c)}, not a capsule")
        out_capsules.append({"a": triple(c.get("a"), f"{where}.a"), "b": triple(c.get("b"), f"{where}.b"),
                             "r": number(c.get("r"), f"{where}.r", 0.001, 1000.0)})
    out_zones = []
    for i, z in enumerate(zones):
        where = f"{what}.zones[{i}]"
        if not isinstance(z, dict) or not isinstance(z.get("axes"), list) or len(z["axes"]) != 3:
            raise ProtocolError(f"{where} is {_show(z)}, not a zone")
        out_zones.append({"name": text(z.get("name"), f"{where}.name"), "centre": triple(z.get("centre"), f"{where}.centre"),
                          "axes": [triple(a, f"{where}.axes[{k}]", 1.0) for k, a in enumerate(z["axes"])],
                          "size": [number(v, f"{where}.size[{k}]", 0.001, 20000.0)
                                   for k, v in enumerate(triple(z.get("size"), f"{where}.size", 20000.0))]})
    return {"cellRevision": integer(value.get("cellRevision"), f"{what}.cellRevision"),
            "marginMm": number(value.get("marginMm"), f"{what}.marginMm", 0, 1000),
            "tool": {"name": text(tool.get("name"), f"{what}.tool.name"), "tcp": pose(tool.get("tcp"), f"{what}.tool.tcp"),
                     "capsules": out_capsules},
            "zones": out_zones}


def params(value, what="params"):
    """A learned step's parameters for its skill: a JSON object of at most MAX_PARAMS_BYTES."""
    if not isinstance(value, dict):
        raise ProtocolError(f"{what} is {_show(value)}, not an object")
    try:
        size = len(json.dumps(value, allow_nan=False).encode("utf-8"))
    except (TypeError, ValueError):
        raise ProtocolError(f"{what} can't be sent as JSON")
    if size > MAX_PARAMS_BYTES:
        raise ProtocolError(f"{what} is {size} bytes as JSON, over {MAX_PARAMS_BYTES}")
    return value


# ---- whole messages ------------------------------------------------------------------------------------------------

def check_request(request, kind=None):
    """A spine's request, checked: what a brain written in Python can rely on. Raises ProtocolError."""
    if not isinstance(request, dict):
        raise ProtocolError(f"isn't a JSON object ({_show(request)})")
    if request.get("v") != VERSION:
        raise ProtocolError(f"speaks protocol version {_show(request.get('v'))}, not {VERSION}")
    if request.get("type") not in TYPES or (kind is not None and request["type"] != kind):
        raise ProtocolError(f"is of type {_show(request.get('type'))}" + (f", not {kind!r}" if kind else ""))
    identifier(request.get("id"), "id")
    t = request["type"]
    try:
        if t in ("plan", "act"):
            state(request.get("state"))
            world(request.get("world"))
            number(request.get("timeoutS"), "timeoutS", 0, 3600)
        if t == "plan":
            goal = request.get("goal")
            if not isinstance(goal, dict):
                raise ProtocolError(f"goal is {_show(goal)}, not an object")
            joints(goal.get("joints"), "goal.joints")
        elif t == "act":
            name(request.get("skill"), "skill")
            params(request.get("params", {}))
            identifier(request.get("episode"), "episode")
            integer(request.get("tick"), "tick")
            anchor = request.get("anchor")
            triple(anchor.get("xyz") if isinstance(anchor, dict) else None, "anchor.xyz")
            number(request.get("withinMm"), "withinMm", 0)
            number(request.get("remainingS"), "remainingS", 0)
        elif t == "end":
            identifier(request.get("episode"), "episode")
            name(request.get("skill"), "skill")
            if request.get("outcome") not in OUTCOMES:
                raise ProtocolError(f"outcome is {_show(request.get('outcome'))}, not one of {', '.join(OUTCOMES)}")
    except ProtocolError as e:
        raise ProtocolError(f"is malformed: {e}")
    return request


def check_reply(request, answer):
    """The brain's reply to `request`, checked: only the fields this version defines, each checked. Raises
    ProtocolError for another version, a reply to another request or to another state (stale), or a malformed one."""
    if not isinstance(answer, dict):
        raise ProtocolError(f"isn't a JSON object ({_show(answer)})")
    if answer.get("v") != VERSION:
        raise ProtocolError(f"speaks protocol version {_show(answer.get('v'))}, not {VERSION}")
    kind = request["type"]
    if answer.get("type") != kind:
        raise ProtocolError(f"is a {_show(answer.get('type'))} reply to a {kind!r} request")
    if answer.get("id") != request["id"]:
        raise ProtocolError(f"answers request {_show(answer.get('id'))}, not {request['id']!r}: it's stale")
    if kind in ("plan", "act"):
        seq = request["state"]["seq"]
        got = answer.get("stateSeq")
        if isinstance(got, bool) or got != seq:
            raise ProtocolError(f"was worked out from state {_show(got)}, not from the state it was sent ({seq}): "
                                "it's stale")
    try:
        return _BODIES[kind](answer)
    except ProtocolError as e:
        raise ProtocolError(f"is malformed: {e}")


def _hello_body(answer):
    skills = answer.get("skills", {})
    if not isinstance(skills, dict) or len(skills) > MAX_SKILLS:
        raise ProtocolError(f"skills should be an object of at most {MAX_SKILLS} names and descriptions")
    planner = answer.get("planner", False)
    if not isinstance(planner, bool):
        raise ProtocolError(f"planner is {_show(planner)}, not true or false")
    return {"brain": text(answer.get("brain"), "brain")[:80], "planner": planner,
            "skills": {name(k, "a skill's name"): text(v, f"skill {k}'s description", required=False)
                       for k, v in skills.items()}}


def _plan_body(answer):
    ok = answer.get("ok")
    if not isinstance(ok, bool):
        raise ProtocolError(f"ok is {_show(ok)}, not true or false")
    if not ok:
        return {"ok": False, "reason": text(answer.get("reason"), "reason")}
    return {"ok": True, "path": path(answer.get("path"), "path", MAX_PLAN_WAYPOINTS),
            "note": text(answer.get("note"), "note", required=False)}


def _act_body(answer):
    action = answer.get("action")
    if not isinstance(action, dict) or action.get("kind") not in ACTIONS:
        raise ProtocolError(f"action is {_show(action)}; its kind should be one of {', '.join(ACTIONS)}")
    kind = action["kind"]
    if kind == "move":
        out = {"kind": "move", "path": path(action.get("path"), "action.path", MAX_BURST_WAYPOINTS + 1)}
        if action.get("speed") is not None:
            out["speed"] = number(action["speed"], "action.speed (deg/s)", 0.1, 120.0)
    elif kind == "hold":
        out = {"kind": "hold", "seconds": number(action.get("seconds"), "action.seconds", 0.0, HOLD_MAX_S)}
    elif kind == "done":
        out = {"kind": "done", "note": text(action.get("note"), "action.note", required=False)}
    else:
        out = {"kind": "fail", "reason": text(action.get("reason"), "action.reason")}
    return {"action": out}


_BODIES = {"hello": _hello_body, "plan": _plan_body, "act": _act_body, "end": lambda answer: {}}
