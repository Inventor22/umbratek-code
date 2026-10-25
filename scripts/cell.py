"""The cell: tools, frames, targets, keep-out zones, grippers, interlocks and patterns, kept in cell/cell.json.

- A tool is the tool point's pose in the flange frame, its payload (mass, centre of gravity) and its shape (capsules,
  for the collision check). The tool point is applied here, on the PC: the controller's own tool offset stays zero, so
  its reported pose stays the flange's (docs/decisions.md D14).
- A frame is a pose relative to a parent frame: "base" (the arm's base frame) or another frame, as RAPID's user and
  object frames. Recalibrating a frame moves everything defined in it.
- A target is a tool point pose in a frame, with the tool it was taught with and the joint angles it was taught at
  (the seed that keeps the arm in the same configuration); or a joint target, just joint angles. Touching one up
  re-teaches it in place and keeps its old value, for Undo; every change to a target, with the old value and the new,
  goes to cell/logs/targets.jsonl.
- A zone is a box in a frame that no part of the arm or tool may come within the collision margin of. A zone may be
  guarded by interlocks: then the arm may enter it while all of them hold, and it's a keep-out zone otherwise.
- An interlock is a named condition on one digital input (a machine's door open, its spindle stopped).
- A gripper is bound to outputs (what to set to close it, and to open it) and, optionally, inputs that confirm it
  closed, opened, or closed on a part; without them, opening or closing takes a set time.
- A pattern is a grid of slots, rows x columns (x layers), taught by three tool targets in one frame: its first
  slot, the last slot of its first row and the last slot of its first column. Its counter (the next slot to use) is
  kept apart, in cell/counters.json (Counters), so counting doesn't change the cell.

Poses are {"xyz": [mm], "rpy": [deg]} (kinematics.py). Every change is written atomically and bumps the cell's
revision; a change made against an older revision is refused, so two pages can't overwrite each other. Standard
library only.
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import tempfile
import threading
import time

import kinematics as kin
import utra_model

VERSION = 1
NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_.-]{0,39}$")
IO = re.compile(r"^(tool|controller)\.(\d{1,2})$")
MAX_XYZ_MM = 5000.0
MAX_MASS_KG = 10.0
MAX_CAPSULES = 8
BASE = "base"
FLANGE_BODY = 6  # the flange's link in utra_model's numbering, which a tool's capsules ride on
# Keys added to cell.json after its first version: a cell without them reads as having none.
COLLECTIONS = ("tools", "frames", "targets", "zones", "grippers", "interlocks", "patterns")
GRIPPER_SECONDS, GRIPPER_TIMEOUT_S = 0.5, 3.0  # how long a gripper takes, and may take to confirm, by default
MAX_GRIPPER_SECONDS, MAX_GRIPPER_TIMEOUT_S = 30.0, 60.0
MAX_GRIPPER_OUTPUTS = 4
MAX_ZONE_INTERLOCKS = 4
MAX_PATTERN_SIDE, MAX_PATTERN_SLOTS = 100, 10000
MIN_PITCH_MM = 1.0  # neighbouring slots closer than this are a teaching mistake
MIN_GRID_ANGLE_DEG = 20.0  # rows and columns at less than this to each other are too
MAX_LAYER_PITCH_MM = 1000.0


class CellError(Exception):
    pass


# ---- checking what's sent ------------------------------------------------------------------------------------

def check_name(name, what):
    if not isinstance(name, str) or not NAME.match(name) or name == BASE:
        raise CellError(f"Unsupported {what} name: {name!r}. Use up to 40 letters, digits, '_', '-' or '.', "
                        "starting with a letter.")
    return name


def numbers(value, count, what, limit):
    if not isinstance(value, (list, tuple)) or len(value) != count or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > limit
            for v in value):
        raise CellError(f"Unsupported {what}: {value!r} (expected {count} numbers within +/-{limit:g}).")
    return [float(v) for v in value]


def check_pose(pose, what="pose"):
    if not isinstance(pose, dict):
        raise CellError(f"Unsupported {what}: {pose!r}.")
    return {"xyz": numbers(pose.get("xyz"), 3, f"{what} position (mm)", MAX_XYZ_MM),
            "rpy": numbers(pose.get("rpy"), 3, f"{what} orientation (deg)", 360.0)}


def check_joints(joints, what="joints"):
    return numbers(joints, 6, f"{what} (deg)", 360.0)


def check_capsules(capsules):
    if capsules is None:
        return []
    if not isinstance(capsules, list) or len(capsules) > MAX_CAPSULES:
        raise CellError(f"A tool's shape is a list of at most {MAX_CAPSULES} capsules.")
    out = []
    for c in capsules:
        if not isinstance(c, dict):
            raise CellError(f"Unsupported capsule: {c!r}.")
        radius = numbers([c.get("r")], 1, "capsule radius (mm)", 300.0)[0]
        if radius <= 0:
            raise CellError("A capsule's radius must be more than 0 mm.")
        out.append({"a": numbers(c.get("a"), 3, "capsule end (mm)", 1000.0),
                    "b": numbers(c.get("b"), 3, "capsule end (mm)", 1000.0), "r": radius})
    return out


def check_io(name, kind):
    """A digital line's name, "tool.N" or "controller.N" (N 0 to 15): (line, bit)."""
    m = IO.match(name) if isinstance(name, str) else None
    if not m or int(m.group(2)) > 15:
        raise CellError(f"Unsupported {kind}: {name!r} (use tool.0 to tool.15 or controller.0 to controller.15).")
    return m.group(1), int(m.group(2))


def check_signal(value, kind, what):
    """{kind ("input" or "output"): "tool.N" | "controller.N", "value": 0 | 1 (1 if left out)}, checked."""
    if not isinstance(value, dict):
        raise CellError(f"Unsupported {what}: {value!r} (an {kind} and its value, 0 or 1).")
    check_io(value.get(kind), kind)
    v = value.get("value", 1)
    if isinstance(v, bool) or v not in (0, 1):
        raise CellError(f"Unsupported value for the {what}: {v!r} (0 or 1).")
    return {kind: value[kind], "value": int(v)}


def check_outputs(values, what):
    """What a gripper sets to close or open: 1 to MAX_GRIPPER_OUTPUTS outputs and their values, each output once."""
    if isinstance(values, dict):
        values = [values]
    if not isinstance(values, list) or not 1 <= len(values) <= MAX_GRIPPER_OUTPUTS:
        raise CellError(f"A gripper's {what} is a list of 1 to {MAX_GRIPPER_OUTPUTS} outputs and their values.")
    out = [check_signal(v, "output", f"{what} output") for v in values]
    names = [o["output"] for o in out]
    if len(set(names)) != len(names):
        raise CellError(f"A gripper's {what} names {', '.join(sorted(n for n in set(names) if names.count(n) > 1))} "
                        "more than once.")
    return out


def check_count(value, what, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value) or not 1 <= value <= high:
        raise CellError(f"Unsupported {what}: {value!r} (a whole number from 1 to {high}).")
    return int(value)


# ---- calibration -----------------------------------------------------------------------------------------------

def frame_from_points(origin, on_x, in_xy, min_mm=20.0):
    """A frame from three points (mm, in the parent frame): its origin, a point along +x, and a point in the xy plane
    on the +y side (as UR's Plane feature). Returns the pose; refuses points too close or nearly in line."""
    o, px, pxy = (numbers(p, 3, "calibration point (mm)", MAX_XYZ_MM) for p in (origin, on_x, in_xy))
    x = [a - b for a, b in zip(px, o)]
    lx = math.sqrt(sum(v * v for v in x))
    if lx < min_mm:
        raise CellError(f"The point on +x is only {lx:.1f} mm from the origin; touch one at least {min_mm:g} mm away.")
    x = [v / lx for v in x]
    w = [a - b for a, b in zip(pxy, o)]
    z = [x[1] * w[2] - x[2] * w[1], x[2] * w[0] - x[0] * w[2], x[0] * w[1] - x[1] * w[0]]
    lz = math.sqrt(sum(v * v for v in z))  # the third point's distance from the x axis
    if lz < min_mm:
        raise CellError(f"The third point is only {lz:.1f} mm from the line through the first two; touch one at least "
                        f"{min_mm:g} mm to the side.")
    z = [v / lz for v in z]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    r = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    return kin.matrix_to_pose(kin.transform(r, o))


def tool_from_flange_poses(flange_poses, min_spread_deg=20.0):
    """The tool point (mm, flange frame) from four or more flange poses (4x4) that touch one fixed point, by least
    squares: R_i t + p_i = P for unknown t and P. Returns (tool point, fixed point in base mm, RMS residual mm)."""
    if len(flange_poses) < 4:
        raise CellError(f"Touch the point from at least 4 orientations ({len(flange_poses)} so far).")
    rs = [kin.rotation_part(t) for t in flange_poses]
    spread = max(kin.angle_between(a, b) for a in rs for b in rs)
    if spread < min_spread_deg:
        raise CellError(f"The orientations differ by at most {spread:.0f} deg; tilt the tool at least {min_spread_deg:g} "
                        "deg between touches.")
    # Normal equations of [R_i, -I] [t; P] = -p_i, in metres.
    A = [[0.0] * 6 for _ in range(6)]
    b = [0.0] * 6
    for t in flange_poses:
        R = kin.rotation_part(t)
        p = [t[0][3], t[1][3], t[2][3]]
        for row in range(3):
            m = R[row] + [-1.0 if k == row else 0.0 for k in range(3)]
            for i in range(6):
                b[i] += m[i] * -p[row]
                for j in range(6):
                    A[i][j] += m[i] * m[j]
    x = kin.solve_linear(A, b)
    if x is None:
        raise CellError("These touches don't pin down the tool point; touch from more varied orientations.")
    tool, point = x[:3], x[3:]
    residuals = []
    for t in flange_poses:
        R = kin.rotation_part(t)
        q = [sum(R[r][k] * tool[k] for k in range(3)) + t[r][3] - point[r] for r in range(3)]
        residuals.append(sum(v * v for v in q))
    rms = math.sqrt(sum(residuals) / len(residuals)) * 1000.0
    return [v * 1000.0 for v in tool], [v * 1000.0 for v in point], rms


# ---- the cell ----------------------------------------------------------------------------------------------------

def empty_cell():
    return {"version": VERSION, "revision": 0,
            "tools": {"flange": {"tcp": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}, "mass": 0.0,
                                 "cog": [0.0, 0.0, 0.0], "capsules": []}},
            "frames": {}, "targets": {}, "zones": {}, "grippers": {}, "interlocks": {}, "patterns": {},
            "active": {"tool": "flange"}}


class Cell:
    """The cell as saved in `folder`/cell.json, with every change checked, saved and versioned."""

    def __init__(self, folder):
        self.folder = folder
        self.path = os.path.join(folder, "cell.json")
        self.lock = threading.RLock()
        if os.path.exists(self.path):
            with open(self.path, encoding="utf-8") as f:
                self.data = json.load(f)
            if self.data.get("version") != VERSION:
                raise CellError(f"{self.path} is version {self.data.get('version')!r}; this server reads {VERSION}.")
            for key in COLLECTIONS:  # a cell saved before grippers, interlocks and patterns has none
                self.data.setdefault(key, {})
        else:
            self.data = empty_cell()

    # -- saving --------------------------------------------------------------------------------------------------
    def _save(self):
        os.makedirs(self.folder, exist_ok=True)
        self.data["revision"] += 1
        fd, tmp = tempfile.mkstemp(prefix=".cell-", suffix=".json", dir=self.folder)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(self.data, f, indent=1)
                f.write("\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except BaseException:
            try:
                os.remove(tmp)
            except OSError:
                pass
            self.data["revision"] -= 1
            raise

    def _edit(self, revision):
        if revision is not None and revision != self.data["revision"]:
            raise CellError(f"The cell changed since this page loaded it (revision {revision}, now "
                            f"{self.data['revision']}). Reload and try again.")

    @property
    def revision(self):
        return self.data["revision"]

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.data)

    # -- frames --------------------------------------------------------------------------------------------------
    def frame_matrix(self, name):
        """base <- frame (4x4, metres)."""
        seen, t = set(), kin.IDENTITY
        while name != BASE:
            if name in seen:
                raise CellError(f"Frame {name!r} is its own ancestor.")
            seen.add(name)
            frame = self.data["frames"].get(name)
            if frame is None:
                raise CellError(f"There's no frame called {name!r}.")
            t = kin.mul(kin.pose_to_matrix(frame["pose"]), t)
            name = frame["parent"]
        return t

    def ancestors(self, name):
        out = []
        while name != BASE:
            out.append(name)
            name = self.data["frames"][name]["parent"]
        return out

    def set_frame(self, name, parent, pose, revision=None, calibration=None, model=None):
        """Create a frame or move one; returns the re-mapping report for the targets it moved."""
        with self.lock:
            self._edit(revision)
            name = check_name(name, "frame")
            pose = check_pose(pose, "frame pose")
            if parent != BASE:
                if parent not in self.data["frames"]:
                    raise CellError(f"There's no frame called {parent!r} to put {name!r} in.")
                if name in self.ancestors(parent):
                    raise CellError(f"{name!r} can't be inside {parent!r}, which is inside it.")
            old = self.data["frames"].get(name)
            before = self._target_matrices() if old is not None else {}
            entry = dict(old or {})
            entry.update({"parent": parent, "pose": pose})
            if calibration is not None:
                entry["calibration"] = calibration
            self.data["frames"][name] = entry
            report = self._remap(name, before, model) if old is not None else []
            self._save()
            return report

    def delete_frame(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            if name not in self.data["frames"]:
                raise CellError(f"There's no frame called {name!r}.")
            users = ([f"frame {n}" for n, f in self.data["frames"].items() if f["parent"] == name]
                     + [f"target {n}" for n, t in self.data["targets"].items() if t.get("frame") == name]
                     + [f"zone {n}" for n, z in self.data["zones"].items() if z["frame"] == name])
            if users:
                raise CellError(f"Frame {name!r} is still used by {', '.join(users)}.")
            del self.data["frames"][name]
            self._save()

    def calibrate_frame(self, name, parent, points, revision=None, model=None, tool=None):
        """Set a frame from three points (mm, in the parent frame): origin, +x, xy plane. Returns the report."""
        with self.lock:
            pose = frame_from_points(points["origin"], points["x"], points["xy"])
            calibration = {"method": "3-point", "points": points, "tool": tool,
                           "at": time.strftime("%Y-%m-%dT%H:%M:%S")}
            return self.set_frame(name, parent, pose, revision, calibration, model)

    # -- tools ---------------------------------------------------------------------------------------------------
    def tool_matrix(self, name=None):
        """flange <- tool point (4x4, metres)."""
        tool = self.tool(name)
        return kin.pose_to_matrix(tool["tcp"])

    def tool(self, name=None):
        name = name or self.data["active"]["tool"]
        tool = self.data["tools"].get(name)
        if tool is None:
            raise CellError(f"There's no tool called {name!r}.")
        return tool

    def set_tool(self, name, tcp, mass=0.0, cog=(0.0, 0.0, 0.0), capsules=None, revision=None, calibration=None):
        with self.lock:
            self._edit(revision)
            name = check_name(name, "tool")
            mass = numbers([mass], 1, "payload (kg)", MAX_MASS_KG)[0]
            if mass < 0:
                raise CellError("A payload can't be negative.")
            entry = {"tcp": check_pose(tcp, "tool point"), "mass": mass,
                     "cog": numbers(list(cog), 3, "centre of gravity (mm)", 1000.0), "capsules": check_capsules(capsules)}
            if calibration is not None:
                entry["calibration"] = calibration
            self.data["tools"][name] = entry
            self._save()

    def delete_tool(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            if name not in self.data["tools"]:
                raise CellError(f"There's no tool called {name!r}.")
            if name == self.data["active"]["tool"]:
                raise CellError(f"Tool {name!r} is in use; pick another first.")
            users = [n for n, t in self.data["targets"].items() if t.get("tool") == name]
            if users:
                raise CellError(f"Tool {name!r} was used to teach {', '.join(users)}.")
            del self.data["tools"][name]
            self._save()

    def activate_tool(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            self.tool(name)
            self.data["active"]["tool"] = name
            self._save()

    # -- targets -------------------------------------------------------------------------------------------------
    def target(self, name):
        target = self.data["targets"].get(name)
        if target is None:
            raise CellError(f"There's no target called {name!r}.")
        return target

    def target_matrix(self, name, offset=None):
        """base <- tool point at a tool target (4x4, metres), with an optional offset {"frame": "target" or "tool",
        "xyz", "rpy"}: "target" translates in the target's frame (RAPID's Offs), "tool" moves and turns about the
        tool point along the tool's own axes (RelTool)."""
        target = self.target(name)
        if "joints" in target:
            raise CellError(f"{name!r} is a joint target; it has no tool pose.")
        frame = self.frame_matrix(target["frame"])
        pose = kin.mul(frame, kin.pose_to_matrix(target["pose"]))
        return apply_offset(pose, frame, offset)

    def set_target(self, name, frame=None, pose=None, tool=None, seed=None, joints=None, revision=None, taught=None):
        with self.lock:
            self._edit(revision)
            name = check_name(name, "target")
            if joints is not None:
                entry = {"joints": check_joints(joints)}
            else:
                if frame != BASE and frame not in self.data["frames"]:
                    raise CellError(f"There's no frame called {frame!r}.")
                tool = tool or self.data["active"]["tool"]
                self.tool(tool)
                entry = {"frame": frame, "tool": tool, "pose": check_pose(pose, "target pose")}
                if seed is not None:
                    entry["seed"] = check_joints(seed, "seed")
            if taught is not None:
                entry["taught"] = taught
            old = self.data["targets"].get(name)
            self.data["targets"][name] = entry
            self._save()
            self._log_target("retaught" if old is not None else "taught", name, old, entry)
            return entry

    def delete_target(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            old = self.target(name)
            users = self.patterns_using(name)
            if users:
                raise CellError(f"Target {name!r} is a corner of {', '.join(users)}; delete or change "
                                f"{'them' if len(users) > 1 else 'it'} first.")
            del self.data["targets"][name]
            self._save()
            self._log_target("deleted", name, old, None)

    def touch_up(self, name, pose=None, seed=None, joints=None, revision=None, taught=None):
        """Re-teach the target `name` in place: a tool target to `pose` (in its own frame, for its own tool) and `seed`
        (the joints it's taught at), or a joint target to `joints`. The old value is kept in the target ("previous", for
        undo_touch_up) and in the targets' log. Returns (the new entry, what changed)."""
        with self.lock:
            self._edit(revision)
            old = _plain_target(self.target(name))
            if "joints" in old:
                if joints is None:
                    raise CellError(f"{name!r} is a joint target: touch it up with the joints where the arm is.")
                new = {"joints": check_joints(joints)}
                change = {"jointsDeg": [round(b - a, 4) for a, b in zip(old["joints"], new["joints"])]}
            else:
                new = {"frame": old["frame"], "tool": old["tool"], "pose": check_pose(pose, "target pose")}
                if seed is not None:
                    new["seed"] = check_joints(seed, "seed")
                a, b = kin.pose_to_matrix(old["pose"]), kin.pose_to_matrix(new["pose"])
                moved, turned = kin.pose_distance(a, b)
                change = {"movedMm": round(moved, 3), "turnedDeg": round(turned, 3),
                          "deltaMm": [round(q - p, 3) for p, q in zip(kin.position(a), kin.position(b))]}
                if old.get("seed") and new.get("seed"):
                    change["jointsDeg"] = [round(q - p, 4) for p, q in zip(old["seed"], new["seed"])]
            if "jointsDeg" in change:
                change["jointsMaxDeg"] = round(max(abs(v) for v in change["jointsDeg"]), 3)
            if taught is not None:
                new["taught"] = taught
            change["at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            self.data["targets"][name] = dict(new, touchUp=change, previous=old)
            self._save()
            self._log_target("touched-up", name, old, new, change=change)
            return self.data["targets"][name], change

    def undo_touch_up(self, name, revision=None):
        """Put back the value a target had before its last touch-up. Returns the restored entry."""
        with self.lock:
            self._edit(revision)
            target = self.target(name)
            old = target.get("previous")
            if old is None:
                raise CellError(f"Target {name!r} hasn't been touched up since it was taught: there's nothing to undo.")
            current = _plain_target(target)
            self.data["targets"][name] = copy.deepcopy(old)
            self._save()
            self._log_target("undone", name, current, old)
            return self.data["targets"][name]

    def _log_target(self, event, name, old, new, **detail):
        """Every change to a target, with its old value and its new, in cell/logs/targets.jsonl."""
        entry = {"t": time.strftime("%Y-%m-%dT%H:%M:%S"), "event": event, "target": name,
                 "old": _plain_target(old) if old is not None else None,
                 "new": _plain_target(new) if new is not None else None, **detail}
        folder = os.path.join(self.folder, "logs")
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, "targets.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps(entry) + "\n")

    # -- zones ---------------------------------------------------------------------------------------------------
    def set_zone(self, name, frame, pose, size, revision=None, interlocks=None):
        """A keep-out zone; with `interlocks` (names), a guarded zone: the arm may enter it while they all hold."""
        with self.lock:
            self._edit(revision)
            name = check_name(name, "zone")
            if frame != BASE and frame not in self.data["frames"]:
                raise CellError(f"There's no frame called {frame!r}.")
            size = numbers(size, 3, "zone size (mm)", 2 * MAX_XYZ_MM)
            if min(size) <= 0:
                raise CellError("A zone's sides must be longer than 0 mm.")
            entry = {"frame": frame, "pose": check_pose(pose, "zone centre"), "size": size}
            guards = self._check_interlock_names(interlocks)
            if guards:
                entry["interlocks"] = guards
            self.data["zones"][name] = entry
            self._save()

    def _check_interlock_names(self, names):
        if names is None or names == []:
            return []
        if isinstance(names, str):
            names = [names]
        if not isinstance(names, list) or len(names) > MAX_ZONE_INTERLOCKS:
            raise CellError(f"A zone is guarded by a list of at most {MAX_ZONE_INTERLOCKS} interlocks.")
        out = []
        for n in names:
            if n not in self.data["interlocks"]:
                raise CellError(f"There's no interlock called {n!r}.")
            if n not in out:
                out.append(n)
        return out

    def delete_zone(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            if name not in self.data["zones"]:
                raise CellError(f"There's no zone called {name!r}.")
            del self.data["zones"][name]
            self._save()

    def guarded_zones(self):
        """{zone: [the interlocks that guard it]} for the zones the arm may enter while they hold."""
        return {name: list(z["interlocks"]) for name, z in self.data["zones"].items() if z.get("interlocks")}

    def zone_obstacles(self, passable=()):
        """The zones as utra_model obstacles, in the base frame, leaving out those in `passable` (guarded zones whose
        interlocks hold). Each carries its zone's name (`zone`), and a guarded one is marked so (`guarded`): no part
        gets into it from outside, even by the allowance for parts already close (utra_model.ArmModel._close_limit)."""
        out = []
        for name, zone in self.data["zones"].items():
            if name in passable:
                continue
            t = kin.mul(self.frame_matrix(zone["frame"]), kin.pose_to_matrix(zone["pose"]))
            axes = [[t[0][k], t[1][k], t[2][k]] for k in range(3)]
            o = utra_model.Obstacle(f"the {name} zone", "box", [t[0][3], t[1][3], t[2][3]], axes,
                                    [v / 1000.0 for v in zone["size"]])
            o.zone = name
            o.guarded = bool(zone.get("interlocks"))
            out.append(o)
        return out

    def tool_capsules(self, name=None):
        """The tool's capsules, for ArmModel.with_cell: (body, end a, end b (m, flange frame), radius (m), label)."""
        tool = self.tool(name)
        return [(FLANGE_BODY, [v / 1000.0 for v in c["a"]], [v / 1000.0 for v in c["b"]], c["r"] / 1000.0, "the tool")
                for c in tool["capsules"]]

    def collision_model(self, model, tool=None, passable=()):
        """The arm model with the tool's shape and the zones added (but those in `passable`)."""
        return model.with_cell(self.tool_capsules(tool), self.zone_obstacles(passable))

    # -- interlocks ----------------------------------------------------------------------------------------------
    def set_interlock(self, name, input_name, value=1, revision=None):
        """A named condition: the input `input_name` reads `value`."""
        with self.lock:
            self._edit(revision)
            name = check_name(name, "interlock")
            self.data["interlocks"][name] = check_signal({"input": input_name, "value": value}, "input", "interlock")
            self._save()

    def delete_interlock(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            if name not in self.data["interlocks"]:
                raise CellError(f"There's no interlock called {name!r}.")
            users = [f"zone {z}" for z, zone in self.data["zones"].items() if name in zone.get("interlocks", [])]
            if users:
                raise CellError(f"Interlock {name!r} guards {', '.join(users)}; take it off "
                                f"{'them' if len(users) > 1 else 'it'} first.")
            del self.data["interlocks"][name]
            self._save()

    # -- grippers ------------------------------------------------------------------------------------------------
    def gripper(self, name):
        gripper = self.data["grippers"].get(name)
        if gripper is None:
            raise CellError(f"There's no gripper called {name!r}.")
        return gripper

    def set_gripper(self, name, close, open_, seconds=GRIPPER_SECONDS, timeout=GRIPPER_TIMEOUT_S, closed=None,
                    opened=None, part=None, revision=None):
        """A gripper: the outputs to set to close it and to open it, how long it takes (waited when there's no input to
        confirm), how long a confirming input may take, and its optional inputs: closed, opened, and closed on a part."""
        with self.lock:
            self._edit(revision)
            name = check_name(name, "gripper")
            entry = {"close": check_outputs(close, "close"), "open": check_outputs(open_, "open"),
                     "seconds": numbers([seconds], 1, "gripper's time (s)", MAX_GRIPPER_SECONDS)[0],
                     "timeout": numbers([timeout], 1, "gripper's timeout (s)", MAX_GRIPPER_TIMEOUT_S)[0]}
            if entry["seconds"] < 0 or entry["timeout"] < 0.1:
                raise CellError("A gripper's time can't be negative, and its timeout is at least 0.1 s.")
            for key, value in (("closed", closed), ("opened", opened), ("part", part)):
                if value is not None:
                    entry[key] = check_signal(value, "input", f"{key} input")
            self.data["grippers"][name] = entry
            self._save()
            return entry

    def delete_gripper(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            self.gripper(name)
            del self.data["grippers"][name]
            self._save()

    # -- patterns ------------------------------------------------------------------------------------------------
    def pattern(self, name):
        pattern = self.data["patterns"].get(name)
        if pattern is None:
            raise CellError(f"There's no pattern called {name!r}.")
        return pattern

    def set_pattern(self, name, rows, columns, layers=1, layer_pitch=0.0, first=None, row_end=None, column_end=None,
                    revision=None):
        """A grid of rows x columns (x layers) slots, from three tool targets in one frame: the first slot (`first`),
        the last slot of the first row (`row_end`, when there's more than one column) and the last slot of the first
        column (`column_end`, when there's more than one row). Layers stack `layer_pitch` mm apart along the frame's z.
        Returns the pattern's geometry (pattern_geometry)."""
        with self.lock:
            self._edit(revision)
            name = check_name(name, "pattern")
            entry = {"rows": check_count(rows, "number of rows", MAX_PATTERN_SIDE),
                     "columns": check_count(columns, "number of columns", MAX_PATTERN_SIDE),
                     "layers": check_count(layers, "number of layers", MAX_PATTERN_SIDE),
                     "layerPitch": numbers([layer_pitch or 0.0], 1, "layer pitch (mm)", MAX_LAYER_PITCH_MM)[0],
                     "first": first}
            if entry["rows"] * entry["columns"] * entry["layers"] > MAX_PATTERN_SLOTS:
                raise CellError(f"A pattern has at most {MAX_PATTERN_SLOTS} slots.")
            if entry["columns"] > 1:
                entry["rowEnd"] = row_end
            if entry["rows"] > 1:
                entry["columnEnd"] = column_end
            geometry = self._geometry(entry)
            self.data["patterns"][name] = entry
            self._save()
            return geometry

    def delete_pattern(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            self.pattern(name)
            del self.data["patterns"][name]
            self._save()

    def patterns_using(self, target):
        """The patterns with `target` as a corner, as 'pattern <name>'."""
        return [f"pattern {n}" for n, p in self.data["patterns"].items()
                if target in (p.get("first"), p.get("rowEnd"), p.get("columnEnd"))]

    def _corner(self, name, what):
        if not isinstance(name, str) or name not in self.data["targets"]:
            raise CellError(f"Pick the target taught at the pattern's {what}"
                            + (f" (there's no target called {name!r})." if name else "."))
        target = self.data["targets"][name]
        if "joints" in target:
            raise CellError(f"{name!r} is a joint target; a pattern's {what} is a tool pose. Record it as a tool pose.")
        return target

    def _geometry(self, p):
        first = self._corner(p["first"], "first slot")
        origin = list(first["pose"]["xyz"])
        steps = {}
        for key, count, what in (("rowEnd", p["columns"], "last slot of the first row"),
                                 ("columnEnd", p["rows"], "last slot of the first column")):
            if count == 1:
                steps[key] = [0.0, 0.0, 0.0]
                continue
            corner = self._corner(p.get(key), what)
            if corner["frame"] != first["frame"] or corner["tool"] != first["tool"]:
                raise CellError(f"{p[key]!r}, the {what}, was taught in {corner['frame']} with {corner['tool']}; the "
                                f"first slot {p['first']!r} in {first['frame']} with {first['tool']}. Teach all three "
                                "in one frame with one tool.")
            step = [(b - a) / (count - 1) for a, b in zip(origin, corner["pose"]["xyz"])]
            pitch = math.sqrt(sum(v * v for v in step))
            if pitch < MIN_PITCH_MM:
                raise CellError(f"The {what} is only {pitch:.2f} mm a slot from the first: check the targets.")
            steps[key] = step
        along, down = steps["rowEnd"], steps["columnEnd"]
        pitches = [math.sqrt(sum(v * v for v in along)), math.sqrt(sum(v * v for v in down)), abs(p["layerPitch"])]
        angle = None
        if p["columns"] > 1 and p["rows"] > 1:
            cos = sum(a * b for a, b in zip(along, down)) / (pitches[0] * pitches[1])
            angle = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
            if not MIN_GRID_ANGLE_DEG <= angle <= 180.0 - MIN_GRID_ANGLE_DEG:
                raise CellError(f"The first row and the first column run only {min(angle, 180 - angle):.0f}\u00b0 "
                                "apart: check the targets at their ends.")
        if p["layers"] > 1 and pitches[2] < MIN_PITCH_MM:
            raise CellError(f"With {p['layers']} layers, give the layer pitch (mm between layers along the frame's z; "
                            "negative to stack downwards).")
        return {"frame": first["frame"], "tool": first["tool"], "origin": origin, "rpy": list(first["pose"]["rpy"]),
                "along": along, "down": down, "up": [0.0, 0.0, p["layerPitch"]],
                "count": p["rows"] * p["columns"] * p["layers"], "pitchMm": [round(v, 3) for v in pitches],
                "angleDeg": None if angle is None else round(angle, 2)}

    def pattern_geometry(self, name):
        """The pattern's geometry, in its frame: the first slot's position (origin) and orientation (rpy), the step from
        one slot to the next along a row (along), down a column (down) and up a layer (up), in mm; the number of slots,
        the pitches and the angle between rows and columns."""
        return self._geometry(self.pattern(name))

    def slot(self, name, slot):
        """Slot `slot` (from 1, along the first row, then the next row, then the next layer) of a pattern: its pose
        in the pattern's frame, its (layer, row, column) from 0, and the joints to solve it from (the corners' taught
        joints, interpolated)."""
        p = self.pattern(name)
        g = self._geometry(p)
        if isinstance(slot, bool) or not isinstance(slot, int) or not 1 <= slot <= g["count"]:
            raise CellError(f"{name} has slots 1 to {g['count']}, not {slot!r}.")
        k = slot - 1
        layer, rest = divmod(k, p["rows"] * p["columns"])
        row, column = divmod(rest, p["columns"])
        xyz = [o + column * a + row * d + layer * u for o, a, d, u in zip(g["origin"], g["along"], g["down"], g["up"])]
        seeds = [self.data["targets"][p[key]].get("seed") if p.get(key) else None
                 for key in ("first", "rowEnd", "columnEnd")]
        seed = seeds[0]
        if seed is not None:
            seed = list(seed)
            for corner, f in ((seeds[1], column / (p["columns"] - 1) if p["columns"] > 1 else 0.0),
                              (seeds[2], row / (p["rows"] - 1) if p["rows"] > 1 else 0.0)):
                if corner is not None and f:
                    seed = [s + f * (c - s0) for s, c, s0 in zip(seed, corner, seeds[0])]
        return {"pose": {"xyz": xyz, "rpy": list(g["rpy"])}, "frame": g["frame"], "tool": g["tool"], "seed": seed,
                "at": [layer, row, column], "count": g["count"]}

    def slot_matrix(self, name, slot, offset=None):
        """base <- tool point at a pattern's slot (4x4, metres), with an optional offset, as target_matrix."""
        s = self.slot(name, slot)
        frame = self.frame_matrix(s["frame"])
        return apply_offset(kin.mul(frame, kin.pose_to_matrix(s["pose"])), frame, offset)

    # -- re-mapping ----------------------------------------------------------------------------------------------
    def _target_matrices(self):
        out = {}
        for name, t in self.data["targets"].items():
            if "joints" not in t:
                out[name] = kin.mul(self.frame_matrix(t["frame"]), kin.pose_to_matrix(t["pose"]))
        return out

    def _remap(self, frame, before, model):
        """After `frame` moved: every target defined in it (or in a frame inside it) moved with it. Re-solve each
        from its seed, so it keeps its configuration, and report how far it moved and whether it's still reachable."""
        report = []
        for name, target in self.data["targets"].items():
            if "joints" in target or frame not in self.ancestors(target["frame"]):
                continue
            after = kin.mul(self.frame_matrix(target["frame"]), kin.pose_to_matrix(target["pose"]))
            moved_mm, moved_deg = kin.pose_distance(before[name], after)
            entry = {"target": name, "movedMm": round(moved_mm, 3), "turnedDeg": round(moved_deg, 3)}
            if model is not None and "seed" in target:
                k = kin.Kinematics(model)
                tool = kin.pose_to_matrix(self.tool(target["tool"])["tcp"]) if target["tool"] in self.data["tools"] \
                    else kin.IDENTITY
                result = k.solve(target["seed"], after, JOINT_LIMITS, tool)
                if result["ok"]:
                    old = target["seed"]
                    target["seed"] = [round(v, 4) for v in result["joints"]]
                    target.pop("unreachable", None)
                    entry["jointsChangedDeg"] = round(max(abs(a - b) for a, b in zip(old, target["seed"])), 3)
                    entry["configurationChanged"] = configuration(old) != configuration(target["seed"])
                else:
                    target["unreachable"] = True
                    entry["unreachable"] = True
            report.append(entry)
        return report


JOINT_LIMITS = ((-180.0, 180.0), (-180.0, 180.0), (-165.0, 165.0), (-180.0, 180.0), (-180.0, 180.0), (-180.0, 180.0))


def _plain_target(target):
    """A target's value without what a touch-up keeps beside it (its previous value and the change)."""
    if target is None:
        return None
    return {k: copy.deepcopy(v) for k, v in target.items() if k not in ("previous", "touchUp")}


class Counters:
    """Where each pattern's counter stands: the next slot to use, from 1 (one past the last slot once they're all
    used), in `folder`/counters.json, apart from the cell so that counting doesn't change the cell's revision. Written
    atomically on every change, so a counter carries on where it was after a restart."""

    def __init__(self, folder):
        self.folder = folder
        self.path = os.path.join(folder, "counters.json")
        self.lock = threading.Lock()
        self.data = {}
        if os.path.exists(self.path):
            try:
                with open(self.path, encoding="utf-8") as f:
                    data = json.load(f)
                self.data = {name: {"slot": int(c["slot"]), "at": str(c.get("at", ""))} for name, c in data.items()
                             if isinstance(c, dict) and isinstance(c.get("slot"), int) and c["slot"] >= 1}
            except (ValueError, AttributeError, TypeError, OSError) as e:
                raise CellError(f"{self.path} can't be read ({e}); fix it, or delete it to start every pattern's counter "
                                "at slot 1.")

    def slot(self, name):
        with self.lock:
            return self.data.get(name, {}).get("slot", 1)

    def snapshot(self):
        with self.lock:
            return {name: c["slot"] for name, c in self.data.items()}

    def set(self, name, slot):
        with self.lock:
            self.data[name] = {"slot": int(slot), "at": time.strftime("%Y-%m-%dT%H:%M:%S")}
            self._save()

    def drop(self, name):
        with self.lock:
            if self.data.pop(name, None) is not None:
                self._save()

    def _save(self):
        os.makedirs(self.folder, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".counters-", suffix=".json", dir=self.folder)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(self.data, f, indent=1)
                f.write("\n")
            os.replace(tmp, self.path)
        except BaseException:
            try:
                os.remove(tmp)
            except OSError:
                pass
            raise


def configuration(joints):
    """The arm's configuration as its elbow's and wrist's signs (which way J3 and J5 bend)."""
    return (joints[2] >= 0, joints[4] >= 0)


def apply_offset(pose, frame, offset):
    """`pose` (base <- tool point) moved by an offset {"frame": "target" | "tool", "xyz": mm, "rpy": deg}."""
    if not offset:
        return pose
    if not isinstance(offset, dict) or offset.get("frame") not in ("target", "tool"):
        raise CellError(f"Unsupported offset: {offset!r} (frame must be 'target' or 'tool').")
    xyz = numbers(offset.get("xyz", [0, 0, 0]), 3, "offset (mm)", 2000.0)
    rpy = numbers(offset.get("rpy", [0, 0, 0]), 3, "offset turn (deg)", 180.0)
    if offset["frame"] == "tool":  # RelTool: along and about the tool's own axes
        return kin.mul(pose, kin.offset_matrix(xyz, rpy))
    if any(rpy):
        raise CellError("An offset in the target's frame only translates; turn the tool with a tool offset.")
    r = kin.rotation_part(frame)  # Offs: translate along the frame's axes
    d = [sum(r[i][k] * xyz[k] for k in range(3)) for i in range(3)]
    out = [list(row) for row in pose]
    for i in range(3):
        out[i][3] += d[i] / 1000.0
    return out
