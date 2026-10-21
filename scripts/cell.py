"""The cell: tools, frames, targets and keep-out zones, kept in cell/cell.json.

- A tool is the tool point's pose in the flange frame, its payload (mass, centre of gravity) and its shape (capsules,
  for the collision check). The tool point is applied here, on the PC: the controller's own tool offset stays zero, so
  its reported pose stays the flange's (docs/decisions.md D14).
- A frame is a pose relative to a parent frame: "base" (the arm's base frame) or another frame, as RAPID's user and
  object frames. Recalibrating a frame moves everything defined in it.
- A target is a tool point pose in a frame, with the tool it was taught with and the joint angles it was taught at
  (the seed that keeps the arm in the same configuration); or a joint target, just joint angles.
- A zone is a box in a frame that no part of the arm or tool may come within the collision margin of.

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
MAX_XYZ_MM = 5000.0
MAX_MASS_KG = 10.0
MAX_CAPSULES = 8
BASE = "base"
FLANGE_BODY = 6  # the flange's link in utra_model's numbering, which a tool's capsules ride on


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
            "frames": {}, "targets": {}, "zones": {}, "active": {"tool": "flange"}}


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
            self.data["targets"][name] = entry
            self._save()
            return entry

    def delete_target(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            self.target(name)
            del self.data["targets"][name]
            self._save()

    # -- zones ---------------------------------------------------------------------------------------------------
    def set_zone(self, name, frame, pose, size, revision=None):
        with self.lock:
            self._edit(revision)
            name = check_name(name, "zone")
            if frame != BASE and frame not in self.data["frames"]:
                raise CellError(f"There's no frame called {frame!r}.")
            size = numbers(size, 3, "zone size (mm)", 2 * MAX_XYZ_MM)
            if min(size) <= 0:
                raise CellError("A zone's sides must be longer than 0 mm.")
            self.data["zones"][name] = {"frame": frame, "pose": check_pose(pose, "zone centre"), "size": size}
            self._save()

    def delete_zone(self, name, revision=None):
        with self.lock:
            self._edit(revision)
            if name not in self.data["zones"]:
                raise CellError(f"There's no zone called {name!r}.")
            del self.data["zones"][name]
            self._save()

    def zone_obstacles(self):
        """The zones as utra_model obstacles, in the base frame."""
        out = []
        for name, zone in self.data["zones"].items():
            t = kin.mul(self.frame_matrix(zone["frame"]), kin.pose_to_matrix(zone["pose"]))
            axes = [[t[0][k], t[1][k], t[2][k]] for k in range(3)]
            out.append(utra_model.Obstacle(f"the {name} zone", "box", [t[0][3], t[1][3], t[2][3]], axes,
                                           [v / 1000.0 for v in zone["size"]]))
        return out

    def tool_capsules(self, name=None):
        """The tool's capsules, for ArmModel.with_cell: (body, end a, end b (m, flange frame), radius (m), label)."""
        tool = self.tool(name)
        return [(FLANGE_BODY, [v / 1000.0 for v in c["a"]], [v / 1000.0 for v in c["b"]], c["r"] / 1000.0, "the tool")
                for c in tool["capsules"]]

    def collision_model(self, model, tool=None):
        """The arm model with the tool's shape and the zones added."""
        return model.with_cell(self.tool_capsules(tool), self.zone_obstacles())

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
