#!/usr/bin/env python3
"""Jog panel: a web page on this PC for moving the UTRA arm one exact step at a time, or to a planned pose.

    python scripts/jog_server.py 192.168.2.1
    # then open http://127.0.0.1:8765

Every click moves the arm by exactly one step from the last commanded goal, so quick clicks add up
instead of being lost. Holding a key repeats steps, but the goal may run at most MAX_LEAD_STEPS steps
ahead of the measured position, so a lost mouse-up can't run the arm away. Dragging joints in the 3D view
(or their faders) builds a plan; Move sends all the planned joints together as one joint move. Joint
moves, planned or stepped, are refused if on the way the arm would come within utra_model.MARGIN_M of
itself, the cart it stands on or the floor (tool steps aren't checked: the controller works out their
joint angles). The server listens on this PC only, stops the arm if the page goes quiet mid-move, and
refuses to move when the arm's state is unknown. The physical E-stop is still the real safety.
"""
from __future__ import annotations

import argparse
import hashlib
import http.client
import json
import math
import os
import secrets
import struct
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import arm
import cell as cellmod
import kinematics as kin
import planner as plannermod
import runner as runnermod
import utra_model
from executor import Executor, MoveFailed

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "jog_panel.html")
CELL_DIR = os.path.join(os.path.dirname(HERE), "cell")
# Files the page may load: the 3D view's scripts and the arm and cart models (and three.js, below). Anything else is
# a 404.
STATIC_ROOTS = {"/web/": os.path.join(HERE, "web"), "/model/": os.path.join(HERE, "model")}
STATIC_TYPES = {".js": "text/javascript; charset=utf-8", ".urdf": "application/xml; charset=utf-8",
                ".stl": "model/stl"}
MODEL_URL = "/model/utra6_1000/"
CART_URL = "/model/cart/"
# three.js draws the 3D view. It isn't kept in the repo: the first time the server starts, it downloads these files of
# three 0.186.1 into THREE_DIR (which git ignores) from the first mirror that has them, and every start checks them
# against these SHA-256 hashes of the files as published on npm. The page gets them from this server only, at
# THREE_URL, served from the copy checked at startup, never from the disk or a mirror unchecked.
THREE_VERSION = "0.186.1"
THREE_FILES = {
    "build/three.module.js": "9052042d676cb0fdc1ddfefe193053f34b7ac0513a616fdac4535d49987812ea",
    "build/three.core.js": "9edde002b066a9a05676a6127f67735b62baf399bdea529f2f7e31657da769e6",
    "examples/jsm/controls/OrbitControls.js": "3d79d07ecb686b4e5d93232eedab255331c1beef711e13164eaa1f68655a5f2b",
    "examples/jsm/loaders/STLLoader.js": "023ed97f848b633d8bcd53d4db3b996d29d0c644088700691297c552257d480b",
    "LICENSE": "8b378ebe60e2fe500158cb0ac71cb5e8b7d92953c2abcc63a0eb90499653b5bc",  # MIT: kept with the copy, not served
}
THREE_MIRRORS = (f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/", f"https://unpkg.com/three@{THREE_VERSION}/")
THREE_DIR = os.path.join(HERE, "vendor", "three")
THREE_URL = "/vendor/three/"
DOWNLOAD_TIMEOUT_S = 10.0

# The step and speed detents the page's faders offer and the server accepts. Joint speeds run up to 120 deg/s,
# just past the fastest joints' top speed, and each move is capped at the top speeds of the joints it turns
# (JOINT_RATED_DEG_S). Tool speeds run up to 1000 mm/s, the UTRA 6-1000's published top speed and Umbratek
# Studio's limit.
JOINT_STEPS = (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0)  # deg per click
JOINT_SPEEDS = (1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0, 120.0)  # deg/s
TOOL_STEPS = (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)  # mm per click
ROT_STEPS = (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0)  # deg per click
TOOL_SPEEDS = (1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 250.0, 500.0, 1000.0)  # mm/s
DEFAULTS = {"jointStep": 1.0, "jointSpeed": 10.0, "toolStep": 5.0, "rotStep": 2.0, "toolSpeed": 20.0}
# The speed faders' redline, like a speedometer's red zone: speeds that could move the tool faster than ISO
# 10218-1's limit for jogging by hand (reduced speed, 250 mm/s at the tool) show red. They still work. On the
# joint speed fader the redline follows the arm's pose; until the pose is known it assumes full reach.
TOOL_REDLINE_MM_S = 250.0
ARM_REACH_MM = 1000.0
# Each joint's top speed, from its own servo. Read from this arm through the SDK's pass-through to the servo bus
# (RS485 line 92, servo ids 1-6): the servos' speed limits (VEL_LIMIT_MAX, register 0x42, rad/s at the motor) are
# 220 for J1 and J2, 250 for J3 and 260 for the wrists, and their gearboxes (each servo's position over its joint's
# angle) are 120:1 for J1 and J2 and 100:1 for the rest. So the joints top out at 105.0, 143.2 and 149.0 deg/s.
# The controller faulted ("Motor_proc angle speed limit") on a J3 move at 120.3 deg/s, under J3's servo limit, so
# moves keep 20% in hand: 84.0, 114.5 and 119.1 deg/s, rounded down to 0.1. (Umbratek's ROS model of the arm rates
# the joints faster, at 2.0, 2.0, 2.1 and 3.0 rad/s, which these servos can't do.) The numbers live in planner.py,
# which plans programs' moves with the same caps.
SERVO_SPEED_LIMITS = plannermod.SERVO_SPEED_LIMITS
GEAR_RATIOS = plannermod.GEAR_RATIOS
SPEED_HEADROOM = plannermod.SPEED_HEADROOM
JOINT_RATED_DEG_S = plannermod.JOINT_RATED_DEG_S
# A move reaches its speed in about RAMP_S, but never ramps more gently than the minimums below, nor harder than
# the maximum acceleration configured in the controller (read when the panel starts).
RAMP_S = 0.3
JOINT_ACC_MIN_DEG_S2 = 100.0
TOOL_ACC_MIN_MM_S2 = 200.0

MAX_LEAD_STEPS = 3  # how far the commanded goal may run ahead of the arm, in steps
BUSY_AFTER_SEND_S = 0.6  # a just-sent command may not show up in the status report yet
# Over Wi-Fi the controller's 10 Hz status stream stalls for about 3 s roughly every half minute.
FRESH_S = 1.0  # a command needs a status frame at most this old...
STALL_WAIT_S = 4.0  # ...and waits this long for one, to ride out a stall
LOST_S = 5.0  # the page shows NO SIGNAL after this long without status
WATCHDOG_STALE_S = 4.5  # a move in progress is stopped after this long without status
PAGE_SILENT_S = 1.5  # stop a move when the page has made no request for this long
# The controller serves one command connection at a time, so stops go over the panel's own. A stop waits this
# long for a request already in flight (each one times out within 1 s) before taking over the connection.
STOP_LOCK_WAIT_S = 1.2
SYNC_TOLERANCE_MM = 2.0
SYNC_TOLERANCE_DEG = 1.0

# Planned moves send joints to absolute angles. Umbratek's URDF xacro limits J3 to +/-165 deg and lets the
# others turn about +/-359 deg; plans stay within the faders' +/-180 deg.
DRAG_LIMITS_DEG = plannermod.DRAG_LIMITS_DEG
FREEDRIVE_MAX_S = 600.0  # freedrive ends on its own after this long
MAX_BODY_BYTES = 262144  # programs can be long


def wrap(angle):
    """Angle in radians wrapped to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rounded(values, digits):
    return [round(v, digits) if math.isfinite(v) else None for v in values]


def pick(value, allowed, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or float(value) not in allowed:
        raise arm.ArmError(f"Unsupported {name}: {value!r}.")
    return float(value)


def pick_index(value, size, name):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < size:
        raise arm.ArmError(f"Unsupported {name}: {value!r}.")
    return value


def blocked(hit):
    """Why a joint move was refused, from utra_model.ArmModel.check_path's hit."""
    return f"Not moving: {hit['part']} would come within {utra_model.MARGIN_M * 1000:.0f} mm of {hit['other']}."


capped_joint_speed = plannermod.capped_joint_speed


class Panel:
    """Owns the arm connection, the live status and the commanded goal."""

    def __init__(self, ip, model=None, cell_dir=CELL_DIR):
        self.ip = ip
        self.model = model  # utra_model.ArmModel, or None when planned moves are off
        self.lock = threading.Lock()  # one SDK conversation at a time
        self.arm = arm.open_arm(ip)
        self.axis = arm.arm_axis(self.arm)
        self.mask = arm.full_mask(self.axis)
        # The controller's configured maximum accelerations (deg/s^2, mm/s^2), or None if it won't say.
        self.joint_acc_max = self._read_limit(self.arm.get_joint_maxacc, 180 / math.pi)
        self.tool_acc_max = self._read_limit(self.arm.get_tcp_maxacc, 1.0)
        self.frame, self.frame_at = None, 0.0
        self.goal_kind, self.goal = None, None
        self.sent_at = 0.0
        self.heard_at = time.monotonic()
        self.note, self.note_ok = "Connected to the controller.", True
        self.stop_alarm = False  # a stop we couldn't confirm; cleared once the arm is seen stopped
        self.closing = False
        # The machine-tending featureset: the cell (tools, frames, targets, zones), programs and their runner.
        self.cell = cellmod.Cell(cell_dir)
        self.programs = runnermod.Programs(cell_dir)
        self.executor = Executor(self)
        self.runner = runnermod.Runner(self, self.programs, os.path.join(cell_dir, "logs"))
        self._cell_model = (None, None)
        self.pending_points = {}  # frame name -> {"parent", "tool", "origin", "x", "xy"} touched so far
        self.pending_poses = {}  # tool name -> flange poses (4x4) touching one point
        self.freedrive_since = None
        self.teach_level = None  # the teach sensitivity this server last set for freedrive
        threading.Thread(target=self._read_reports, daemon=True).start()
        threading.Thread(target=self._watch, daemon=True).start()

    @staticmethod
    def _read_limit(getter, scale):
        try:
            ret, value = getter()
        except Exception:
            return None
        ok = ret in arm.OK_CODES and isinstance(value, (int, float)) and math.isfinite(value) and value > 0
        return value * scale if ok else None

    def _joint_acc(self, speed):
        """Acceleration (deg/s^2) for a joint move at `speed` deg/s."""
        acc = max(JOINT_ACC_MIN_DEG_S2, speed / RAMP_S)
        return min(acc, self.joint_acc_max) if self.joint_acc_max else acc

    def _tool_acc(self, speed):
        """Acceleration (mm/s^2) for a tool move at `speed` mm/s."""
        acc = max(TOOL_ACC_MIN_MM_S2, speed / RAMP_S)
        return min(acc, self.tool_acc_max) if self.tool_acc_max else acc

    # ---- live status ---------------------------------------------------------

    def _read_reports(self):
        while not self.closing:
            try:
                report = arm.StatusReport(self.ip, timeout=2.0)
            except OSError:
                time.sleep(1.0)
                continue
            try:
                while not self.closing:
                    frame = report.read()
                    self.frame, self.frame_at = frame, time.monotonic()
                    if self.stop_alarm and (frame["status"] == 4 or not frame["enable"] & self.mask or frame["err"]):
                        self.stop_alarm = False
            except (OSError, ValueError, struct.error):
                time.sleep(0.5)
            finally:
                report.close()

    def _fresh_frame(self):
        """The latest status frame, waiting out a short stall in the controller's status stream."""
        deadline = time.monotonic() + STALL_WAIT_S
        while self.frame is None or time.monotonic() - self.frame_at > FRESH_S:
            if time.monotonic() >= deadline or self.closing:
                raise arm.ArmError("No live status from the controller, so not moving. Check the Wi-Fi link.")
            time.sleep(0.05)
        return self.frame

    def _busy(self, frame):
        return frame["status"] == 1 or frame["queued"] > 0 or time.monotonic() - self.sent_at < BUSY_AFTER_SEND_S

    def _say(self, text, ok=True):
        self.note, self.note_ok = text, ok
        return text

    def heard(self):
        self.heard_at = time.monotonic()

    def status(self):
        frame = self.frame
        age = time.monotonic() - self.frame_at if frame is not None else None
        out = {"ip": self.ip, "note": self.note, "noteOk": self.note_ok, "stopAlarm": self.stop_alarm,
               "live": age is not None and age <= LOST_S, "age": None if age is None else round(age, 2)}
        if frame is None:
            return out
        out.update({
            "state": frame["status"],
            "enabled": frame["enable"] & self.mask == self.mask,
            "err": frame["err"],
            "errName": arm.ERROR_NAMES.get(frame["err"], "unknown") if frame["err"] else "",
            "warn": frame["warn"],
            "queued": frame["queued"],
            "joints": rounded([math.degrees(j) for j in frame["joints"][:self.axis]], 3),
            "pose": rounded(frame["pose"][:3], 2) + rounded([math.degrees(v) for v in frame["pose"][3:6]], 3),
        })
        if self.model is not None:  # mm from the flange to each joint's axis, for the joint speed redline
            out["toolRadii"] = rounded([r * 1000.0 for r in self.model.tool_radii(out["joints"])], 1)
        goal, kind = self.goal, self.goal_kind
        if goal is not None and self._busy(frame):
            if kind == "joint":
                out["goal"] = {"kind": kind, "values": rounded([math.degrees(g) for g in goal], 3)}
            else:
                out["goal"] = {"kind": kind, "values": rounded(goal[:3], 2) + rounded([math.degrees(g) for g in goal[3:]], 3)}
        current = self.executor.current
        if current is not None:  # a program's move: the 3D view shows where it's heading
            out["goal"] = {"kind": "joint", "values": rounded(current[0].goal, 3), "program": True}
        run = self.runner.status()
        out["run"] = {k: run[k] for k in ("state", "mode", "manual", "program", "path", "step", "reason", "override",
                                          "halt", "hit")}
        out["run"]["lastEvent"] = run["events"][-1] if run["events"] else None
        out["freedrive"] = frame["mode"] == 2
        out["teachSensitivity"] = self.teach_level  # what this server last set for freedrive (None before)
        out["mode"] = frame["mode"]
        out["cellRevision"] = self.cell.revision
        out["tool"] = self.cell.data["active"]["tool"]
        return out

    # ---- commands --------------------------------------------------------------

    def enable(self):
        with self.lock:
            frame = self._fresh_frame()
            if self.stop_alarm:
                raise arm.ArmError("A stop wasn't confirmed. Press the hardware E-stop, release it, then hold Enable.")
            if frame["status"] in (1, 3) or frame["queued"]:
                raise arm.ArmError("The arm is still moving or has moves queued. Wait for it, or press STOP first.")
            arm.ensure_idle(self.arm, self.ip)
            ret, codes = self.arm.get_error_code()
            if ret in arm.OK_CODES and codes[0]:
                if self.arm.reset_err() not in arm.OK_CODES:
                    raise arm.ArmError("The controller refused the error reset.")
                time.sleep(1.5)  # let the controller re-check its E-stop and joints
            start = arm.preflight(self.arm, self.ip, self.axis)

            def switch_on():
                arm.prepare_for_motion(self.arm, self.axis)
                arm.verified_targets(self.arm, self.ip, self.axis, start)

            arm.guarded(self.ip, self.arm, switch_on, STOP_LOCK_WAIT_S)
            self.goal_kind = self.goal = None
            self.sent_at = 0.0
        return self._say("Motors on. Click a key to move one step; hold it to repeat.")

    def stop(self, cause=None):
        """Stop the arm. For automatic stops, `cause` finishes "Stopped because ..."."""
        # Over the panel's own connection (the controller serves only one), without waiting for self.lock: the
        # SDK's own lock keeps the stop from interleaving with a request another thread has in flight.
        confirmed = arm.stop_arm(self.ip, self.arm, STOP_LOCK_WAIT_S)
        self.goal_kind = self.goal = None
        self.sent_at = 0.0
        self.stop_alarm = not confirmed
        frame = self.frame
        if frame is not None and frame["mode"] == 2:  # STOP also ends freedrive
            if self.lock.acquire(timeout=STOP_LOCK_WAIT_S):
                try:
                    self.arm.set_motion_mode(0)
                except Exception:
                    pass
                finally:
                    self.lock.release()
            self.freedrive_since = None
        if confirmed:
            said = f"Stopped because {cause}." if cause else "Stopped."
            return True, self._say(f"{said} The controller confirmed it. Hold Enable to carry on.")
        why = f" The stop was sent because {cause}." if cause else ""
        return False, self._say(f"STOP NOT CONFIRMED: press the hardware E-stop now.{why}", ok=False)

    # ---- the cell, planning and the runner's needs -------------------------------------------------------------

    def cell_model(self):
        """The arm model with the active tool's shape and the cell's keep-out zones, rebuilt when the cell changes."""
        key = (self.cell.revision, self.cell.data["active"]["tool"])
        if self._cell_model[0] != key:
            self._cell_model = (key, self.cell.collision_model(self.model))
        return self._cell_model[1]

    def planner(self):
        if self.model is None:
            raise arm.ArmError("The arm model didn't load, so planned moves are off.")
        return plannermod.Planner(self.cell_model(), self.joint_acc_max, self.tool_acc_max)

    def send_payload(self):
        """Tell the controller the active tool's payload (mass kg, centre of gravity mm from the flange), as the SDK
        asks whenever the payload changes; the controller's own tool offset stays zero (decision D14)."""
        tool = self.cell.tool()
        with self.lock:
            ret = self.arm.set_tcp_load(float(tool["mass"]), [float(v) for v in tool["cog"]])
        if ret not in arm.OK_CODES:
            raise arm.ArmError(f"The controller refused the payload (code {ret}).")

    def current_joints(self):
        """The measured joints (deg), from fresh status."""
        frame = self._fresh_frame()
        q = [math.degrees(v) for v in frame["joints"][:self.axis]]
        if not arm.all_finite(q):
            raise arm.ArmError("The status report has unusable joint angles.")
        return q

    def tool_point(self, joints=None, tool=None):
        """base <- the active (or named) tool's point (4x4), at `joints` or where the arm is."""
        q = joints if joints is not None else self.current_joints()
        return kin.mul(self.model.frames(q)[-1], self.cell.tool_matrix(tool))

    def plan_move(self, step, start, speed, ori_speed=60.0):
        """A Plan for a program's move step (a joint or straight move to a target, with an optional offset)."""
        target = self.cell.target(step["target"])
        p = self.planner()
        if "joints" in target:
            if step["kind"] != "joint":
                raise plannermod.PlanError(f"{step['target']!r} is a joint target; a straight line needs a tool target.")
            return p.joint(start, target["joints"], speed, step["target"])
        active = self.cell.data["active"]["tool"]
        if target["tool"] != active:
            raise plannermod.PlanError(f"{step['target']!r} was taught with the tool {target['tool']!r}, but the active "
                                       f"tool is {active!r}.")
        if target.get("unreachable"):
            raise plannermod.PlanError(f"{step['target']!r} became unreachable when its frame was recalibrated; "
                                       "teach it again.")
        pose = self.cell.target_matrix(step["target"], step.get("offset"))
        tool = self.cell.tool_matrix()
        label = step["target"] + (" (offset)" if step.get("offset") else "")
        if step["kind"] == "joint":
            return p.joint_to_pose(start, pose, tool, speed, seed=target.get("seed"), label=label)
        return p.line(start, pose, tool, speed, ori_speed, label=label)

    def check_can_run(self, dry=False):
        """Refuses (ArmError) unless a run or a planned move may start now."""
        frame = self._fresh_frame()
        if self.model is None:
            raise arm.ArmError("The arm model didn't load, so programs can't be planned.")
        if dry:
            return
        if self.stop_alarm:
            raise arm.ArmError("A stop wasn't confirmed. Press the hardware E-stop, release it, then hold Enable.")
        if frame["err"]:
            raise arm.ArmError(f"The controller reports error {arm.error_text(frame['err'])}. Fix it, then hold Enable.")
        if frame["mode"] == 2:
            raise arm.ArmError("The arm is in freedrive. Leave freedrive first.")
        if frame["enable"] & self.mask != self.mask:
            raise arm.ArmError("The motors are off. Hold Enable first.")
        if frame["status"] not in (0, 2) or self._busy(frame):
            raise arm.ArmError("The arm isn't ready and still. Wait for it, or hold Enable.")
        self.sent_at = 0.0
        self.goal_kind = self.goal = None

    def rearm(self):
        """After a pause's STOP: make the arm ready again (Enable's checks and sequence), if it's stopped, then wait
        for a status frame from after it, so what the runner plans from includes the enable shift (about 0.5 mm)."""
        frame = self._fresh_frame()
        if frame["status"] == 4 or frame["enable"] & self.mask != self.mask:
            self.enable()
            armed = time.monotonic()
            deadline = armed + STALL_WAIT_S
            while self.frame_at <= armed + 0.02 and time.monotonic() < deadline:
                time.sleep(0.02)

    def read_input(self, line, bit):
        """A digital input: line "tool" or "controller", bit 0-15."""
        with self.lock:
            st, values = (self.arm.get_tgpio_in if line == "tool" else self.arm.get_cgpio_in)()
        if not isinstance(values, list) or len(values) < 2:
            raise MoveFailed(f"couldn't read the {line}'s inputs (code {st})")
        return (int(values[1]) >> bit) & 1

    def write_output(self, name, value):
        line, bit = runnermod.io_name(name, "output")
        word = (1 << (16 + bit)) | ((1 if value else 0) << bit)
        with self.lock:
            ret = (self.arm.set_tgpio_digit_out if line == "tool" else self.arm.set_cgpio_digit_out)(word)
        if ret not in arm.OK_CODES:
            raise MoveFailed(f"the controller refused to set {name} (code {ret})")

    def io_state(self):
        out = {}
        for line in ("tool", "controller"):
            with self.lock:
                st_i, ins = (self.arm.get_tgpio_in if line == "tool" else self.arm.get_cgpio_in)()
                st_o, outs = (self.arm.get_tgpio_out if line == "tool" else self.arm.get_cgpio_out)()
            out[line] = {"inputs": int(ins[1]) if isinstance(ins, list) else None,
                         "outputs": int(outs[1]) if isinstance(outs, list) else None}
        return out

    def freedrive(self, on, sensitivity=1):
        """Enter freedrive (motion mode 2, at a teach sensitivity of 1 to 5) or leave it."""
        if on:
            level = pick(sensitivity, (1, 2, 3, 4, 5), "teach sensitivity")
            if self.runner.busy:
                raise arm.ArmError("A program is running. Stop it before freedrive.")
            with self.lock:
                frame = self._fresh_frame()
                if frame["err"] or self.stop_alarm:
                    raise arm.ArmError("Clear the fault first (hold Enable).")
                if frame["enable"] & self.mask != self.mask:
                    raise arm.ArmError("The motors are off. Hold Enable first.")
                if frame["status"] == 1 or self._busy(frame):
                    raise arm.ArmError("The arm is moving. Wait for it to stop.")
                if self.arm.set_teach_sens(int(level)) not in arm.OK_CODES or \
                        self.arm.set_motion_mode(2) not in arm.OK_CODES:
                    raise arm.ArmError("The controller refused freedrive.")
            self.freedrive_since = time.monotonic()
            self.teach_level = int(level)
            return self._say(f"Freedrive on (sensitivity {level:g}): guide the arm by hand. Record targets as you go; "
                             "leave freedrive or press STOP when done.")
        with self.lock:
            if self.arm.set_motion_mode(0) not in arm.OK_CODES:
                raise arm.ArmError("The controller refused to leave freedrive. Press STOP.")
        self.freedrive_since = None
        frame = self._fresh_frame()
        targets = arm.target_joints(self.arm)
        joint, diff = arm.worst_diff(targets, frame["joints"])
        if diff > arm.MAX_JOINT_MISMATCH_DEG:
            self.stop("the controller's targets didn't match the arm after freedrive")
            raise arm.ArmError(f"After freedrive, J{joint}'s target is {diff:.1f}\u00b0 from where it is; stopped. "
                               "Hold Enable to carry on.")
        return self._say("Freedrive off: the arm holds its pose.")

    def jog(self, body):
        """Move one step. Returns (moved, message)."""
        kind, direction = body.get("kind"), body.get("dir")
        if isinstance(direction, bool) or direction not in (1, -1):
            raise arm.ArmError(f"Unsupported direction: {direction!r}.")
        if kind == "joint":
            index = pick_index(body.get("index"), self.axis, "joint")
            step = pick(body.get("step"), JOINT_STEPS, "joint step")
            speed = pick(body.get("speed"), JOINT_SPEEDS, "joint speed")
        elif kind == "tool":
            index = pick_index(body.get("index"), 6, "tool axis")
            step = pick(body.get("step"), TOOL_STEPS, "tool step")
            rot_step = pick(body.get("rotStep"), ROT_STEPS, "rotation step")
            speed = pick(body.get("speed"), TOOL_SPEEDS, "tool speed")
        else:
            raise arm.ArmError(f"Unsupported jog kind: {kind!r}.")

        with self.lock:
            frame = self._fresh_frame()
            if self.stop_alarm:
                raise arm.ArmError("A stop wasn't confirmed. Press the hardware E-stop, release it, then hold Enable.")
            if frame["err"]:
                raise arm.ArmError(f"The controller reports error {arm.error_text(frame['err'])}. "
                                   "Fix the cause, then hold Enable.")
            if frame["enable"] & self.mask != self.mask:
                raise arm.ArmError("The motors are off. Hold Enable first.")
            if frame["status"] not in (0, 1, 2):
                state = arm.MOTION_STATES.get(frame["status"], "in an unknown state")
                raise arm.ArmError(f"The arm is {state}. Hold Enable to make it ready.")
            if self.runner.moving:
                raise arm.ArmError("A program is running. Pause or stop it first.")
            if frame["mode"] == 2:
                raise arm.ArmError("The arm is in freedrive. Leave freedrive to jog.")
            busy = self._busy(frame)
            if self.goal is None or not busy:
                self.goal, self.goal_kind = self._synced_goal(kind, frame), kind
            elif self.goal_kind != kind:
                raise arm.ArmError("Let the current move finish before switching between joint and tool keys.")

            goal = list(self.goal)
            sign = "+" if direction > 0 else "\u2212"
            if kind == "joint":
                goal[index] += direction * math.radians(step)
                lead = max(abs(math.degrees(g - m)) for g, m in zip(goal, frame["joints"]))
                if lead > MAX_LEAD_STEPS * step + 0.5:
                    return False, self._say("Holding: the arm is still catching up with earlier clicks.")
                if self.model is not None:  # the step this click adds, from the goal before it
                    hit = self.cell_model().check_path([math.degrees(g) for g in self.goal],
                                                       [math.degrees(g) for g in goal])
                    if hit is not None:
                        raise arm.ArmError(blocked(hit))
                sent, limiting = capped_joint_speed(speed, [step if i == index else 0.0 for i in range(self.axis)])
                ret = self.arm.moveto_joint_p2p(goal, math.radians(sent), math.radians(self._joint_acc(sent)), 0)
                text = f"J{index + 1} {sign}{step:g}\u00b0 \u2192 {math.degrees(goal[index]):.2f}\u00b0"
                if limiting is not None:
                    text += f" at {sent:g}\u00b0/s, J{index + 1}'s top speed"
            else:
                if index < 3:
                    goal[index] += direction * step
                else:
                    goal[index] = wrap(goal[index] + direction * math.radians(rot_step))
                pos_lead = math.dist(goal[:3], frame["pose"][:3])
                rot_lead = max(abs(math.degrees(wrap(g - m))) for g, m in zip(goal[3:], frame["pose"][3:6]))
                if pos_lead > MAX_LEAD_STEPS * step + 1.0 or rot_lead > MAX_LEAD_STEPS * rot_step + 0.5:
                    return False, self._say("Holding: the arm is still catching up with earlier clicks.")
                if self.model is not None:  # the line this click adds, planned and checked like a program's
                    self._check_tool_step(frame, self.goal, goal, speed)
                ret = self.arm.moveto_cartesian_line(goal, speed, self._tool_acc(speed), 0)
                axis_name = ("X", "Y", "Z", "Roll", "Pitch", "Yaw")[index]
                value = goal[index] if index < 3 else math.degrees(goal[index])
                unit = " mm" if index < 3 else "\u00b0"
                amount = step if index < 3 else rot_step
                text = f"{axis_name} {sign}{amount:g}{unit} \u2192 {value:.1f}{unit}"
            if ret not in arm.OK_CODES:
                self.goal = None
                raise arm.abort_motion(self.ip, self.arm, f"The controller rejected the move (code {ret}).", STOP_LOCK_WAIT_S)
            self.goal, self.sent_at = goal, time.monotonic()
            self.heard_at = self.sent_at  # the page is waiting on this very request, so it isn't quiet
        return True, self._say(text)

    def _check_tool_step(self, frame, before, after, speed):
        """Plan a tool step as a straight line of the flange (the controller's own tool point is the flange), from
        the goal before it to the new one, with the cell's tool and zones: refuses a step out of reach, through a
        singularity or into anything."""
        p = self.planner()
        q = [math.degrees(v) for v in frame["joints"][:self.axis]]
        start = p.kin.solve(q, kin.from_controller_pose(before), DRAG_LIMITS_DEG)
        if not start["ok"]:
            return  # the arm is between goals it hasn't reached; the controller has the earlier step in hand
        try:
            p.line(start["joints"], kin.from_controller_pose(after), kin.IDENTITY, speed, label="the step's end")
        except plannermod.PlanError as e:
            raise arm.ArmError(str(e))

    def move(self, body):
        """Move the arm to a plan: absolute angles for one or more joints, all sent as one joint move.

        body is {"targets": [deg or None per joint], "speed": deg/s}; None leaves that joint where it is.
        The older {"joint": i, "angle": deg} form is the same with one target. Returns (moved, message).
        """
        if self.model is None:
            raise arm.ArmError("The arm model didn't load, so planned moves are off. Use the + and \u2212 keys.")
        targets = body.get("targets")
        if targets is None and "joint" in body:
            targets = [None] * self.axis
            targets[pick_index(body.get("joint"), self.axis, "joint")] = body.get("angle")
        if not isinstance(targets, list) or len(targets) != self.axis:
            raise arm.ArmError(f"Unsupported targets: {targets!r}.")
        for i, angle in enumerate(targets):
            if angle is None:
                continue
            if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle):
                raise arm.ArmError(f"Unsupported angle for J{i + 1}: {angle!r}.")
            low, high = DRAG_LIMITS_DEG[i]
            if not low <= angle <= high:
                raise arm.ArmError(f"J{i + 1} can only be planned between {low:g}\u00b0 and {high:g}\u00b0.")
        if all(angle is None for angle in targets):
            raise arm.ArmError("The plan is empty: drag a joint first.")
        speed = pick(body.get("speed"), JOINT_SPEEDS, "joint speed")

        with self.lock:
            frame = self._fresh_frame()
            if self.stop_alarm:
                raise arm.ArmError("A stop wasn't confirmed. Press the hardware E-stop, release it, then hold Enable.")
            if frame["err"]:
                raise arm.ArmError(f"The controller reports error {arm.error_text(frame['err'])}. "
                                   "Fix the cause, then hold Enable.")
            if frame["enable"] & self.mask != self.mask:
                raise arm.ArmError("The motors are off. Hold Enable first.")
            if frame["status"] not in (0, 1, 2):
                state = arm.MOTION_STATES.get(frame["status"], "in an unknown state")
                raise arm.ArmError(f"The arm is {state}. Hold Enable to make it ready.")
            if self.runner.moving:
                raise arm.ArmError("A program is running. Pause or stop it first.")
            if self._busy(frame):
                raise arm.ArmError("The arm is still moving. Wait for it to settle, or press STOP.")
            if frame["mode"] == 2:
                raise arm.ArmError("The arm is in freedrive. Leave freedrive first.")
            start = self._synced_goal("joint", frame)
            start_deg = [math.degrees(v) for v in start]
            moving = [i for i, angle in enumerate(targets) if angle is not None and abs(angle - start_deg[i]) >= 0.05]
            if not moving:
                return False, self._say("The arm is already in that position.")
            goal_deg = [float(targets[i]) if i in moving else start_deg[i] for i in range(self.axis)]
            # The whole path, not just the plan: never within the margin of the arm itself, the cart, the floor, the
            # tool or the cell's keep-out zones (or, for parts already that close, no closer than they are now).
            hit = self.cell_model().check_path(start_deg, goal_deg)
            if hit is not None:
                raise arm.ArmError(blocked(hit))
            # Joints not in the plan stay exactly at their current targets.
            goal = [math.radians(goal_deg[i]) if i in moving else start[i] for i in range(self.axis)]
            sent, limiting = capped_joint_speed(speed, [abs(math.degrees(g - s)) for g, s in zip(goal, start)])
            ret = self.arm.moveto_joint_p2p(goal, math.radians(sent), math.radians(self._joint_acc(sent)), 0)
            if ret not in arm.OK_CODES:
                self.goal = None
                raise arm.abort_motion(self.ip, self.arm, f"The controller rejected the move (code {ret}).", STOP_LOCK_WAIT_S)
            self.goal, self.goal_kind, self.sent_at = goal, "joint", time.monotonic()
            self.heard_at = self.sent_at  # the page is waiting on this very request, so it isn't quiet
        why = f", J{limiting + 1}'s top speed" if limiting is not None else ""
        if len(moving) == 1:
            return True, self._say(f"J{moving[0] + 1} \u2192 {goal_deg[moving[0]]:.1f}\u00b0 at {sent:g}\u00b0/s{why}")
        names = ", ".join(f"J{i + 1}" for i in moving)
        return True, self._say(f"Moving {names} together to the plan at up to {sent:g}\u00b0/s{why}")

    def _synced_goal(self, kind, frame):
        """A fresh goal from the controller's targets, checked against the measured state."""
        if kind == "joint":
            targets = arm.target_joints(self.arm)
            joint, diff = arm.worst_diff(targets, frame["joints"])
            if diff > arm.MAX_JOINT_MISMATCH_DEG:
                raise arm.ArmError(f"Joint {joint}'s target is {diff:.1f}\u00b0 from its measured angle, so not moving. "
                                   "Check the arm in Umbratek Studio.")
            return targets
        ret, pose = self.arm.get_tcp_target_pos()
        if ret not in arm.OK_CODES or not isinstance(pose, list) or len(pose) < 6 or not arm.all_finite(pose):
            raise arm.ArmError("Could not read the tool's target pose, so not moving.")
        pos_gap = math.dist(pose[:3], frame["pose"][:3])
        rot_gap = max(abs(math.degrees(wrap(p - m))) for p, m in zip(pose[3:6], frame["pose"][3:6]))
        if pos_gap > SYNC_TOLERANCE_MM or rot_gap > SYNC_TOLERANCE_DEG:
            raise arm.ArmError("The tool's target pose differs from its measured pose, so not moving. "
                               "Check the arm in Umbratek Studio.")
        return list(pose[:6])

    # ---- watchdog --------------------------------------------------------------

    def _watch(self):
        while not self.closing:
            time.sleep(0.1)
            frame, sent_at = self.frame, self.sent_at
            if self.freedrive_since is not None and time.monotonic() - self.freedrive_since > FREEDRIVE_MAX_S:
                try:
                    self.freedrive(False)
                    self._say(f"Freedrive ended on its own after {FREEDRIVE_MAX_S / 60:.0f} minutes.")
                except arm.ArmError:
                    pass
                self.freedrive_since = None
            if frame is None or not sent_at or self.runner.moving:
                continue  # nothing of the page's under way (a program's moves have their own supervision)
            now = time.monotonic()
            if not (frame["status"] == 1 or frame["queued"] > 0 or now - sent_at < BUSY_AFTER_SEND_S):
                continue
            if now - self.heard_at > PAGE_SILENT_S:
                self.stop("the jog page went quiet")
            elif now - self.frame_at > WATCHDOG_STALE_S:
                self.stop("the controller's status went quiet")
            elif frame["err"]:
                self.stop(f"the controller reported error {arm.error_text(frame['err'])}")


def static_file(url_path):
    """(path, content type) for an allowlisted file under STATIC_ROOTS, or None."""
    for prefix, root in STATIC_ROOTS.items():
        if not url_path.startswith(prefix):
            continue
        parts = urllib.parse.unquote(url_path[len(prefix):]).split("/")
        if any(p in ("", ".", "..") or any(c in p for c in "\\:\0") for p in parts):
            return None
        kind = STATIC_TYPES.get(os.path.splitext(parts[-1])[1].lower())
        base = os.path.realpath(root)
        path = os.path.realpath(os.path.join(base, *parts))
        if kind is None or os.path.commonpath([path, base]) != base or not os.path.isfile(path):
            return None
        return path, kind
    return None


def load_three():
    """three.js's files as {path: bytes}, each matching its hash in THREE_FILES: read from THREE_DIR, or downloaded
    there when missing or altered. Returns (files, None), or ({}, why) when one can't be had, say with no internet."""
    files, dead, announced = {}, {}, False
    for path, digest in THREE_FILES.items():
        local = os.path.join(THREE_DIR, *path.split("/"))
        try:
            with open(local, "rb") as f:
                data = f.read()
        except OSError:
            data = b""
        if hashlib.sha256(data).hexdigest() == digest:
            files[path] = data
            continue
        if not announced:
            print(f"Downloading three.js {THREE_VERSION} for the 3D view (the first time only)...")
            announced = True
        problems = []
        for base in THREE_MIRRORS:
            host = urllib.parse.urlsplit(base).hostname
            if base in dead:  # it didn't answer for an earlier file, so don't wait on it again
                problems.append(f"{host}: {dead[base]}")
                continue
            try:
                with urllib.request.urlopen(base + path, timeout=DOWNLOAD_TIMEOUT_S) as reply:
                    data = reply.read()
            except (OSError, http.client.HTTPException) as e:  # URLError, HTTPError and timeouts are OSErrors
                if isinstance(e, urllib.error.HTTPError):
                    problems.append(f"{host}: HTTP {e.code}")
                else:
                    dead[base] = str(getattr(e, "reason", None) or e)
                    problems.append(f"{host}: {dead[base]}")
                continue
            if hashlib.sha256(data).hexdigest() == digest:
                break
            problems.append(f"{host}'s copy of {path} isn't the published file")
        else:
            return {}, "; ".join(problems)
        files[path] = data
        try:  # write it whole or not at all, so a failed save can't leave a half file
            os.makedirs(os.path.dirname(local), exist_ok=True)
            with open(local + ".part", "wb") as f:
                f.write(data)
            os.replace(local + ".part", local)
        except OSError as e:
            print(f"Warning: couldn't save {path} ({e}); it'll be downloaded again next time.", file=sys.stderr)
    if announced:
        print(f"Saved three.js in {THREE_DIR}. From now on the 3D view works offline.")
    return files, None


def make_handler(panel, token, page, port, three):
    hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
    html_headers = {
        "Content-Security-Policy": "default-src 'none'; script-src 'self' 'unsafe-inline'; style-src 'unsafe-inline'; "
                                   "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'",
        "X-Frame-Options": "DENY",
    }

    class Handler(BaseHTTPRequestHandler):
        server_version = "JogPanel/1"
        protocol_version = "HTTP/1.1"  # keep-alive: the page polls ~7 times a second
        timeout = 30  # drop idle keep-alive connections

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, payload, kind="application/json"):
            data = payload if isinstance(payload, bytes) else json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if code >= 400:  # the request body may be unread, so don't reuse the connection
                self.send_header("Connection", "close")
                self.close_connection = True
            if kind.startswith("text/html"):
                for name, value in html_headers.items():
                    self.send_header(name, value)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.headers.get("Host") not in hosts:
                return self._send(403, {"ok": False, "message": f"Open the panel at http://127.0.0.1:{port}"})
            path = urllib.parse.urlsplit(self.path).path
            if path == "/":
                return self._send(200, page, "text/html; charset=utf-8")
            if path == "/api/status":
                panel.heard()
                return self._send(200, panel.status())
            if path.startswith("/api/"):
                query = dict(urllib.parse.parse_qsl(urllib.parse.urlsplit(self.path).query))
                try:
                    reply = api_get(panel, path, query)
                except (arm.ArmError, cellmod.CellError, runnermod.ProgramError, plannermod.PlanError, MoveFailed) as e:
                    reply = {"ok": False, "message": str(e)}
                if reply is not None:
                    return self._send(200, reply)
            if path.startswith(THREE_URL):  # only the checked copy, and only its scripts
                data = three.get(path[len(THREE_URL):]) if path.endswith(".js") else None
                if data is not None:
                    return self._send(200, data, STATIC_TYPES[".js"])
                return self._send(404, {"ok": False, "message": "Not found."})
            found = static_file(path)
            if found is not None:
                with open(found[0], "rb") as f:
                    return self._send(200, f.read(), found[1])
            return self._send(404, {"ok": False, "message": "Not found."})

        def do_POST(self):
            if self.headers.get("Host") not in hosts or \
                    not secrets.compare_digest(self.headers.get("X-Panel-Token", ""), token):
                return self._send(403, {"ok": False, "message": "Forbidden."})
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                length = -1
            if not 0 <= length <= MAX_BODY_BYTES:
                return self._send(400, {"ok": False, "message": "Bad request."})
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                return self._send(400, {"ok": False, "message": "Bad request."})
            if not isinstance(body, dict):
                return self._send(400, {"ok": False, "message": "Bad request."})
            panel.heard()
            try:
                if self.path == "/api/stop":
                    panel.runner.stop("STOP was pressed")
                    confirmed, text = panel.stop()
                    return self._send(200, {"ok": confirmed, "message": text})
                if self.path == "/api/enable":
                    if panel.runner.moving:
                        raise arm.ArmError("A program is running. Pause or stop it first.")
                    return self._send(200, {"ok": True, "message": panel.enable()})
                if self.path == "/api/jog":
                    moved, text = panel.jog(body)
                    return self._send(200, {"ok": True, "moved": moved, "message": text})
                if self.path == "/api/move":
                    moved, text = panel.move(body)
                    return self._send(200, {"ok": True, "moved": moved, "message": text})
                reply = api_post(panel, self.path, body)
                if reply is not None:
                    return self._send(200, reply)
            except (arm.ArmError, cellmod.CellError, runnermod.ProgramError, plannermod.PlanError, MoveFailed) as e:
                return self._send(200, {"ok": False, "message": panel._say(str(e), ok=False)})
            return self._send(404, {"ok": False, "message": "Not found."})

    return Handler


# ---- the featureset's API ----------------------------------------------------------------------------------------

def cell_view(panel):
    """The cell, with every frame and target placed in the base frame and the collision model the page should
    check against (with the active tool's shape and the zones)."""
    cell = panel.cell
    data = cell.snapshot()
    frames = {}
    for name in data["frames"]:
        frames[name] = kin.matrix_to_pose(cell.frame_matrix(name), 3)
    targets = {}
    for name, t in data["targets"].items():
        if "joints" in t:
            targets[name] = None
        else:
            try:
                targets[name] = kin.matrix_to_pose(cell.target_matrix(name), 3)
            except cellmod.CellError:
                targets[name] = None
    out = {"ok": True, "cell": data, "framesInBase": frames, "targetsInBase": targets,
           "pending": {"frames": panel.pending_points, "tools": {k: len(v) for k, v in panel.pending_poses.items()}}}
    scene = {"zones": [], "frames": [], "targets": []}  # what the 3D view draws (metres, base frame)
    for o, (name, zone) in zip(cell.zone_obstacles(), data["zones"].items()):
        scene["zones"].append({"name": name, "centre": o.centre, "axes": o.axes, "half": o.half})
    for name in data["frames"]:
        t = cell.frame_matrix(name)
        scene["frames"].append({"name": name, "origin": [t[i][3] for i in range(3)], "x": [t[i][0] for i in range(3)],
                                "y": [t[i][1] for i in range(3)]})
    for name, pose in targets.items():
        if pose is not None:
            scene["targets"].append({"name": name, "p": [v / 1000.0 for v in pose["xyz"]]})
    out["scene"] = scene
    if panel.model is not None:
        out["collision"] = panel.cell_model().collision_config()
    return out


def api_get(panel, path, query):
    if path == "/api/cell":
        return cell_view(panel)
    if path == "/api/programs":
        return {"ok": True, "programs": panel.programs.names()}
    if path == "/api/program":
        return {"ok": True, "program": panel.programs.load(query.get("name"))}
    if path == "/api/run":
        return {"ok": True, "run": panel.runner.status()}
    if path == "/api/io":
        return {"ok": True, "io": panel.io_state()}
    return None


def _idle_for_setup(panel):
    if panel.runner.moving:
        raise arm.ArmError("A program is running. Pause or stop it first.")


def api_post(panel, path, body):
    cell, runner = panel.cell, panel.runner
    rev = body.get("revision")
    if path == "/api/cell/tool":
        _idle_for_setup(panel)
        cell.set_tool(body.get("name"), body.get("tcp"), body.get("mass", 0.0), body.get("cog", [0, 0, 0]),
                      body.get("capsules"), rev)
        if body.get("name") == cell.data["active"]["tool"]:
            panel.send_payload()
        return dict(cell_view(panel), message=f"Tool {body.get('name')} saved.")
    if path == "/api/cell/tool/delete":
        cell.delete_tool(body.get("name"), rev)
        return dict(cell_view(panel), message=f"Tool {body.get('name')} deleted.")
    if path == "/api/cell/tool/activate":
        _idle_for_setup(panel)
        cell.activate_tool(body.get("name"), rev)
        panel.send_payload()
        return dict(cell_view(panel), message=f"Tool {body.get('name')} in use; its payload sent to the controller.")
    if path == "/api/cell/tool/touch":
        name = cellmod.check_name(body.get("name"), "tool")
        poses = panel.pending_poses.setdefault(name, [])
        poses.append(panel.model.frames(panel.current_joints())[-1])
        return dict(cell_view(panel), message=f"Touch {len(poses)} recorded for {name}'s tool point"
                                              + (" (4 or more are needed)." if len(poses) < 4 else "."))
    if path == "/api/cell/tool/calibrate":
        name = cellmod.check_name(body.get("name"), "tool")
        point, fixed, rms = cellmod.tool_from_flange_poses(panel.pending_poses.get(name, []))
        old = cell.data["tools"].get(name)
        tcp = {"xyz": [round(v, 3) for v in point], "rpy": old["tcp"]["rpy"] if old else [0.0, 0.0, 0.0]}
        calibration = {"method": "4-point", "touches": len(panel.pending_poses[name]), "rmsMm": round(rms, 3),
                       "fixedPointMm": [round(v, 2) for v in fixed], "at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        cell.set_tool(name, tcp, old["mass"] if old else 0.0, old["cog"] if old else [0, 0, 0],
                      old["capsules"] if old else [], rev, calibration)
        panel.pending_poses.pop(name, None)
        return dict(cell_view(panel), message=f"Tool {name}'s point is {tcp['xyz']} mm from the flange "
                                              f"(fit within {rms:.2f} mm RMS).")
    if path == "/api/cell/tool/touch/clear":
        panel.pending_poses.pop(body.get("name"), None)
        return cell_view(panel)
    if path == "/api/cell/frame":
        report = cell.set_frame(body.get("name"), body.get("parent", cellmod.BASE), body.get("pose"), rev,
                                model=panel.model)
        return dict(cell_view(panel), remapped=report, message=f"Frame {body.get('name')} saved"
                    + (f"; {len(report)} target(s) moved with it." if report else "."))
    if path == "/api/cell/frame/delete":
        cell.delete_frame(body.get("name"), rev)
        return dict(cell_view(panel), message=f"Frame {body.get('name')} deleted.")
    if path == "/api/cell/frame/touch":
        name = cellmod.check_name(body.get("name"), "frame")
        which, parent = body.get("which"), body.get("parent", cellmod.BASE)
        if which not in ("origin", "x", "xy"):
            raise cellmod.CellError("Touch the origin, a point along +x, or a point in the xy plane (+y side).")
        point = kin.position(kin.mul(kin.invert(cell.frame_matrix(parent)), panel.tool_point()))
        pending = panel.pending_points.setdefault(name, {"parent": parent})
        if pending.get("parent") != parent:
            pending.clear()
            pending["parent"] = parent
        pending[which] = [round(v, 3) for v in point]
        pending["tool"] = cell.data["active"]["tool"]
        return dict(cell_view(panel), message=f"{which} of {name} recorded at {pending[which]} mm in {parent}.")
    if path == "/api/cell/frame/calibrate":
        name = cellmod.check_name(body.get("name"), "frame")
        pending = panel.pending_points.get(name, {})
        points = body.get("points") or {k: pending.get(k) for k in ("origin", "x", "xy")}
        if any(points.get(k) is None for k in ("origin", "x", "xy")):
            missing = [k for k in ("origin", "x", "xy") if points.get(k) is None]
            raise cellmod.CellError(f"Touch {', '.join(missing)} first.")
        parent = body.get("parent", pending.get("parent", cellmod.BASE))
        report = cell.calibrate_frame(name, parent, points, rev, panel.model, pending.get("tool"))
        panel.pending_points.pop(name, None)
        moved = [r for r in report if r["movedMm"] > 0.0005 or r["turnedDeg"] > 0.0005]
        lost = [r["target"] for r in report if r.get("unreachable")]
        flipped = [r["target"] for r in report if r.get("configurationChanged")]
        text = f"Frame {name} calibrated; {len(moved)} target(s) moved with it."
        if lost:
            text += f" Now out of reach: {', '.join(lost)}."
        if flipped:
            text += f" Now in another configuration: {', '.join(flipped)}."
        return dict(cell_view(panel), remapped=report, message=text)
    if path == "/api/cell/target":
        return teach_target(panel, body)
    if path == "/api/cell/target/delete":
        users = runnermod.programs_using(panel.programs, body.get("name"))
        if users:
            raise cellmod.CellError(f"Target {body.get('name')!r} is used by {', '.join(users)}; take it out of "
                                    f"{'them' if len(users) > 1 else 'it'} first.")
        cell.delete_target(body.get("name"), rev)
        return dict(cell_view(panel), message=f"Target {body.get('name')} deleted.")
    if path == "/api/cell/zone":
        cell.set_zone(body.get("name"), body.get("frame", cellmod.BASE), body.get("pose"), body.get("size"), rev)
        return dict(cell_view(panel), message=f"Zone {body.get('name')} saved.")
    if path == "/api/cell/zone/delete":
        cell.delete_zone(body.get("name"), rev)
        return dict(cell_view(panel), message=f"Zone {body.get('name')} deleted.")
    if path == "/api/program":
        saved = panel.programs.save(body.get("program"), cell, rev)
        return {"ok": True, "program": saved, "message": f"Program {saved['name']} saved."}
    if path == "/api/program/delete":
        panel.programs.delete(body.get("name"))
        return {"ok": True, "message": f"Program {body.get('name')} deleted."}
    if path == "/api/run/start":
        runner.start(body.get("name"), body.get("mode", "run"), body.get("from"))
        return {"ok": True, "run": runner.status(), "message": f"Running {body.get('name')}"
                + {"run": ".", "step": ", one step at a time.", "dry": " dry: planning only, nothing moves."}[
                    body.get("mode", "run")]}
    if path in ("/api/run/pause", "/api/run/resume", "/api/run/step", "/api/run/reset"):
        getattr(runner, path.rsplit("/", 1)[1])()
        return {"ok": True, "run": runner.status()}
    if path == "/api/run/stop":
        stopped = runner.stop("Stop run was pressed")
        return {"ok": True, "run": runner.status(), "message": "Stopping the run." if stopped else "No run to stop."}
    if path == "/api/run/override":
        runner.set_override(body.get("percent"))
        return {"ok": True, "run": runner.status(), "message": f"Speed override {runner.override}%."}
    if path in ("/api/moveto", "/api/plan"):
        step = {"type": "move", "kind": body.get("kind", "joint"), "target": body.get("target")}
        if body.get("offset"):
            step["offset"] = body["offset"]
        if body.get("speed") is not None:
            step["speed"] = body["speed"]
        program = {"name": "manual", "steps": [step], "speed": body.get("defaults") or {}}
        if path == "/api/plan":
            checked = runnermod.check_program(program, cell)
            s = checked["steps"][0]
            speed = s.get("speed", checked["speed"]["tcp" if s["kind"] == "line" else "joint"])
            plan = panel.plan_move(s, panel.current_joints(), speed, checked["speed"]["ori"])
            return {"ok": True, "plan": plan.summary(), "message": plan.text}
        runner.start(program=program, manual=True)
        return {"ok": True, "run": runner.status(), "message": f"Moving to {body.get('target')}."}
    if path == "/api/io/set":
        _idle_for_setup(panel)
        if runner.busy:
            raise arm.ArmError("A program is under way; its outputs are its own.")
        panel.write_output(body.get("output"), 1 if body.get("value") else 0)
        return {"ok": True, "io": panel.io_state(), "message": f"{body.get('output')} set to {1 if body.get('value') else 0}."}
    if path == "/api/freedrive":
        return {"ok": True, "message": panel.freedrive(bool(body.get("on")), body.get("sensitivity", 1))}
    return None


def teach_target(panel, body):
    """Save a target: given ({"pose"} or {"joints"}), or where the arm is ({"here": true}), optionally adding a move
    to it into a program ({"program", "at": step path to insert after, "move": "joint" | "line"})."""
    cell, rev = panel.cell, body.get("revision")
    name, frame = body.get("name"), body.get("frame", cellmod.BASE)
    taught = None
    if body.get("here"):
        if panel.runner.moving:
            raise arm.ArmError("A program is running. Pause or stop it first.")
        q = panel.current_joints()
        taught = {"at": time.strftime("%Y-%m-%dT%H:%M:%S"), "how": "freedrive" if panel.frame["mode"] == 2 else "here",
                  "joints": [round(v, 4) for v in q]}
        if body.get("kind") == "joint":
            entry = cell.set_target(name, joints=[round(v, 4) for v in q], revision=rev, taught=taught)
        else:
            pose = kin.mul(kin.invert(cell.frame_matrix(frame)), panel.tool_point(q))
            entry = cell.set_target(name, frame, kin.matrix_to_pose(pose, 4), seed=[round(v, 4) for v in q],
                                    revision=rev, taught=taught)
    elif body.get("joints") is not None:
        entry = cell.set_target(name, joints=body["joints"], revision=rev)
    else:
        entry = cell.set_target(name, frame, body.get("pose"), body.get("tool"), body.get("seed"), revision=rev)
    message = f"Target {name} saved" + (" where the arm is." if taught else ".")
    if body.get("program"):
        program = panel.programs.load(body["program"])
        move = body.get("move") or ("joint" if "joints" in entry else "line")
        step = {"type": "move", "kind": move, "target": name}
        steps, at = program["steps"], body.get("at")
        if at:  # insert after this step (a path of 0-based indexes)
            container = steps
            for i in at[:-1]:
                node = container[i]
                container = node.get("steps") or node.get("then") or []
            container.insert(at[-1] + 1, step)
        else:
            steps.append(step)
        saved = panel.programs.save(program, cell, program.get("revision"))
        message += f" Added a {'straight' if move == 'line' else 'joint'} move to it in {saved['name']}."
    return dict(cell_view(panel), message=message, target=entry)


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description="Serve a jog panel for the UTRA arm at http://127.0.0.1:PORT.")
    parser.add_argument("ip", help="controller address, e.g. 192.168.2.1")
    parser.add_argument("--port", type=int, default=8765, help="local port (default 8765)")
    parser.add_argument("--cell", default=CELL_DIR, help="folder for the cell, programs and run logs (default cell/)")
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be between 1024 and 65535")

    try:
        model = utra_model.ArmModel()
    except Exception as e:  # the 3D view and planned moves are optional; jogging works without them
        print(f"Warning: couldn't load the arm model ({e}). The 3D view and planned moves are off.", file=sys.stderr)
        model = None
    try:
        panel = Panel(args.ip, model, args.cell)
    except (arm.ArmError, cellmod.CellError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    if model is not None and len(model.joints) != panel.axis:
        print(f"Warning: the arm model has {len(model.joints)} joints but the arm has {panel.axis}. "
              "The 3D view and planned moves are off.", file=sys.stderr)
        panel.model = None
    token = secrets.token_urlsafe(24)
    model = panel.model
    three, missing = load_three() if model else ({}, None)  # with no model there's no 3D view to draw
    if missing:
        print(f"Warning: couldn't download three.js {THREE_VERSION}, which draws the 3D view ({missing}). The panel "
              "works without the 3D view; to get it, check this PC can reach the internet and start the jog panel "
              "again.", file=sys.stderr)
    config = {"ip": args.ip, "axis": panel.axis, "jointSteps": JOINT_STEPS, "jointSpeeds": JOINT_SPEEDS,
              "toolSteps": TOOL_STEPS, "rotSteps": ROT_STEPS, "toolSpeeds": TOOL_SPEEDS,
              "jointRated": JOINT_RATED_DEG_S[:panel.axis], "toolRedline": TOOL_REDLINE_MM_S, "armReach": ARM_REACH_MM,
              "defaults": DEFAULTS, "maxLeadSteps": MAX_LEAD_STEPS, "freshS": FRESH_S,
              "model": {"base": MODEL_URL, "urdf": os.path.basename(utra_model.URDF_PATH)} if model else None,
              "cart": {"base": CART_URL, "visuals": model.cart_visuals} if model else None,
              "collision": model.collision_config() if model else None,
              "floorZ": model.floor_z if model else 0.0, "dragLimits": DRAG_LIMITS_DEG, "three": bool(three)}
    with open(PAGE, encoding="utf-8") as f:
        page = f.read().replace("__TOKEN__", token).replace("__CONFIG__", json.dumps(config))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port),
                                     make_handler(panel, token, page.encode("utf-8"), args.port, three))
    except OSError as e:
        print(f"Error: cannot listen on 127.0.0.1:{args.port} ({e}). Try --port.", file=sys.stderr)
        return 1
    server.daemon_threads = True
    print(f"Jog panel for {args.ip}: open http://127.0.0.1:{args.port}  (Ctrl+C to quit)")
    limits = [f"joints {panel.joint_acc_max:.0f}\u00b0/s\u00b2" if panel.joint_acc_max else "",
              f"tool {panel.tool_acc_max:.0f} mm/s\u00b2" if panel.tool_acc_max else ""]
    if any(limits):
        print("The controller's maximum acceleration: " + ", ".join(v for v in limits if v) + ".")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        panel.closing = True
        frame = panel.frame
        if panel.runner.busy:
            panel.runner.stop("the jog panel was closed")
            panel.stop("the jog panel was closed during a program")
        elif panel.sent_at and frame is not None and (frame["status"] == 1 or frame["queued"] > 0):
            panel.stop("the jog panel was closed")
        server.server_close()
    print("Jog panel closed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
