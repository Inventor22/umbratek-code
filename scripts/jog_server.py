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
import utra_model

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "jog_panel.html")
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
# the joints faster, at 2.0, 2.0, 2.1 and 3.0 rad/s, which these servos can't do.)
SERVO_SPEED_LIMITS = (220.0, 220.0, 250.0, 260.0, 260.0, 260.0)  # rad/s at the motor
GEAR_RATIOS = (120.0, 120.0, 100.0, 100.0, 100.0, 100.0)
SPEED_HEADROOM = 0.8
JOINT_RATED_DEG_S = tuple(math.floor(math.degrees(SPEED_HEADROOM * v / g) * 10) / 10
                          for v, g in zip(SERVO_SPEED_LIMITS, GEAR_RATIOS))
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
DRAG_LIMITS_DEG = ((-180.0, 180.0), (-180.0, 180.0), (-165.0, 165.0),
                   (-180.0, 180.0), (-180.0, 180.0), (-180.0, 180.0))


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


class Panel:
    """Owns the arm connection, the live status and the commanded goal."""

    def __init__(self, ip, model=None):
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
        if confirmed:
            said = f"Stopped because {cause}." if cause else "Stopped."
            return True, self._say(f"{said} The controller confirmed it. Hold Enable to carry on.")
        why = f" The stop was sent because {cause}." if cause else ""
        return False, self._say(f"STOP NOT CONFIRMED: press the hardware E-stop now.{why}", ok=False)

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
                    hit = self.model.check_path([math.degrees(g) for g in self.goal], [math.degrees(g) for g in goal])
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
            if self._busy(frame):
                raise arm.ArmError("The arm is still moving. Wait for it to settle, or press STOP.")
            start = self._synced_goal("joint", frame)
            start_deg = [math.degrees(v) for v in start]
            moving = [i for i, angle in enumerate(targets) if angle is not None and abs(angle - start_deg[i]) >= 0.05]
            if not moving:
                return False, self._say("The arm is already in that position.")
            goal_deg = [float(targets[i]) if i in moving else start_deg[i] for i in range(self.axis)]
            # The whole path, not just the plan: never within the margin of the arm itself, the cart or the
            # floor (or, for parts already that close, no closer than they are now).
            hit = self.model.check_path(start_deg, goal_deg)
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
            if frame is None or not sent_at:
                continue
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
            if not 0 <= length <= 4096:
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
                    confirmed, text = panel.stop()
                    return self._send(200, {"ok": confirmed, "message": text})
                if self.path == "/api/enable":
                    return self._send(200, {"ok": True, "message": panel.enable()})
                if self.path == "/api/jog":
                    moved, text = panel.jog(body)
                    return self._send(200, {"ok": True, "moved": moved, "message": text})
                if self.path == "/api/move":
                    moved, text = panel.move(body)
                    return self._send(200, {"ok": True, "moved": moved, "message": text})
            except arm.ArmError as e:
                return self._send(200, {"ok": False, "message": panel._say(str(e), ok=False)})
            return self._send(404, {"ok": False, "message": "Not found."})

    return Handler


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description="Serve a jog panel for the UTRA arm at http://127.0.0.1:PORT.")
    parser.add_argument("ip", help="controller address, e.g. 192.168.2.1")
    parser.add_argument("--port", type=int, default=8765, help="local port (default 8765)")
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be between 1024 and 65535")

    try:
        model = utra_model.ArmModel()
    except Exception as e:  # the 3D view and planned moves are optional; jogging works without them
        print(f"Warning: couldn't load the arm model ({e}). The 3D view and planned moves are off.", file=sys.stderr)
        model = None
    try:
        panel = Panel(args.ip, model)
    except arm.ArmError as e:
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
        if panel.sent_at and frame is not None and (frame["status"] == 1 or frame["queued"] > 0):
            panel.stop("the jog panel was closed")
        server.server_close()
    print("Jog panel closed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
