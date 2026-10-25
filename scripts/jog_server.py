#!/usr/bin/env python3
"""Jog panel: a web page on this PC for moving the UTRA arm one exact step at a time, or to a planned pose.

    python scripts/jog_server.py 192.168.2.1
    # then open http://127.0.0.1:8765

Every click moves the arm by exactly one step from the last commanded goal, so quick clicks add up
instead of being lost. Holding a key repeats steps, but the goal may run at most MAX_LEAD_STEPS steps
ahead of the measured position, so a lost mouse-up can't run the arm away. Dragging joints in the 3D view
(or their faders) builds a plan; Move sends all the planned joints together as one joint move. Moves, planned or
stepped, are refused if on the way the arm would come within utra_model.MARGIN_M of itself, the cart it stands on, the
floor, the tool or the cell's keep-out zones, or go further into one it's already in (a tool step that can't be checked,
from a joint beyond the planning limits, isn't sent). The server listens on this PC only, stops the arm if the page
goes quiet mid-move, and refuses to move when the arm's state is unknown. The physical E-stop is still the real safety.

The manual-motion policy (--reduced-speed, --hold-to-move, POST /api/manual): with reduced speed on, every move is sent
slowly enough that neither the tool point in use nor the flange can pass 250 mm/s; with hold-to-move on, nothing moves
without a current hold (POST /api/hold, an enabling device), and a move stops, or a run pauses, when the hold lapses.
The program that starts the server may set its token (UMBRATEK_PANEL_TOKEN) to call the POST API itself.

Programs may also have planned and learned steps, whose motion the brain proposes: a separate process on this PC that
the server asks (--brain, docs/brain-interface.md), and whose every proposal the server checks before it runs.
"""
from __future__ import annotations

import argparse
import hashlib
import http.client
import ipaddress
import json
import math
import os
import re
import secrets
import socket
import struct
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler

import arm
import brain_link
import cell as cellmod
import kinematics as kin
import loopback_http
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
# 10218-1's limit for jogging by hand (reduced speed, 250 mm/s at the tool) show red. They still work, unless reduced
# speed is on (below). On the joint speed fader the redline follows the arm's pose; until the pose is known it assumes
# full reach.
TOOL_REDLINE_MM_S = plannermod.REDUCED_SPEED_MM_S
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
FREEDRIVE_CONFIRM_S = 2.0  # how long the controller has to report a change of motion mode once asked
# The teach sensitivity, in % as Umbratek's current SDK documents it (90 to 110, 100 its default). On the arm
# (2026-10-21) 90 held where the hand let go; 100 and 110 made the shoulder and elbow lighter, but a raised elbow sank
# back a few degrees. The controller boots at 0, which felt like 90; the pinned SDK's 1 to 5 was never tried.
TEACH_SENSITIVITIES = (90, 100, 110)
TEACH_SENSITIVITY_DEFAULT = 90
MAX_BODY_BYTES = 262144  # programs can be long
# The token every POST must carry. The program that starts the server may choose it, in this environment variable, to
# call the POST API itself; otherwise it's random (docs/jog-panel.md).
TOKEN_ENV = "UMBRATEK_PANEL_TOKEN"
TOKEN_MIN_CHARS = 16
TOKEN_CHARS = re.compile(r"[A-Za-z0-9_-]+")  # URL-safe base64, unpadded, as secrets.token_urlsafe writes it
# The manual-motion policy (docs/jog-panel.md): reduced speed keeps every move's tool point and flange under
# plannermod.REDUCED_SPEED_MM_S; hold-to-move, an enabling device, lets nothing move without a current hold, which a
# client renews about every 100 ms while its key is down (POST /api/hold).
HOLD_DEFAULT_MS, HOLD_MIN_MS, HOLD_MAX_MS = 400, 100, 1000
# Grippers: how often a gripper's confirming inputs are read while it opens or closes, and how long a grasp's "closed
# on a part" input may take to read true once it has closed.
GRIP_POLL_S = 0.05
PART_SETTLE_S = 0.3
CALLS_KEPT = 50  # calls to the owner kept for the panel
# Watching a move into a guarded zone: the interlocks are read without waiting longer than this for the SDK (a request
# may hold it for seconds while the status stalls), and the arm stops if they can't be read for this long.
WATCH_LOCK_WAIT_S = 0.05
GUARD_CONFIRM_S = 0.5
# An Enable the server runs on its own, after writing the payload stopped the arm, needs the arm seen ready on a status
# frame that came after the grip began and this long after any STOP ended (so it shows the STOP's effect); it waits up to
# FRAME_WAIT_S past that for one.
STOP_SEEN_S = 0.3
FRAME_WAIT_S = 0.3


class StoppedMeanwhile(arm.ArmError):
    """An Enable the server ran on its own gave way to a STOP that came since its cause began (Panel.enable)."""


def wrap(angle):
    """Angle in radians wrapped to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rounded(values, digits):
    return [round(v, digits) if math.isfinite(v) else None for v in values]


def in_freedrive(frame):
    """Whether a status frame shows the arm free to guide: mode 2, not stopped, no error. The real arm reports state 1
    then; stopped in mode 2 it holds rigid, and with an error flagged it holds too (2026-10-21)."""
    return frame["mode"] == 2 and frame["status"] != 4 and not frame["err"]


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
    return f"Not moving: {plannermod.blocked(hit, utra_model.MARGIN_M * 1000)}."


def panel_token(environ=os.environ):
    """The token the page and every POST carry: TOKEN_ENV's value if the program that started the server set it, so it
    can call the POST API too, else a random one. ValueError if it's set but too short or not URL-safe base64."""
    given = environ.get(TOKEN_ENV)
    if given is None:
        return secrets.token_urlsafe(24)
    if len(given) < TOKEN_MIN_CHARS or not TOKEN_CHARS.fullmatch(given):
        raise ValueError(f"{TOKEN_ENV} must be at least {TOKEN_MIN_CHARS} characters of URL-safe base64 (A-Z, a-z, "
                         "0-9, '-' and '_', no padding), or unset for a random token.")
    return given


def local_controller(host):
    """Whether the controller's address is this PC's own: a loopback address (127.0.0.0/8, ::1) or localhost, when it
    resolves to loopback only. Only the simulator or the twin can serve there, so the status says "simulated"."""
    name = host.strip().lower().rstrip(".")
    if name.startswith("[") and name.endswith("]"):
        name = name[1:-1]
    try:
        address = ipaddress.ip_address(name)
    except ValueError:
        if name != "localhost":
            return False
        try:
            found = {info[4][0] for info in socket.getaddrinfo(name, None)}
        except OSError:
            return False
        return bool(found) and all(local_controller(a) for a in found)  # addresses, never names
    mapped = getattr(address, "ipv4_mapped", None)
    return (mapped or address).is_loopback


capped_joint_speed = plannermod.capped_joint_speed


class Panel:
    """Owns the arm connection, the live status and the commanded goal."""

    def __init__(self, ip, model=None, cell_dir=CELL_DIR, brain=None, reduced_speed=False, hold_to_move=False):
        self.ip = ip
        self.simulated = local_controller(ip)
        self.model = model  # utra_model.ArmModel, or None when planned moves are off
        self.lock = threading.Lock()  # one SDK conversation at a time
        self.arm = arm.open_arm(ip)
        self.axis = arm.arm_axis(self.arm)
        self.mask = arm.full_mask(self.axis)
        # The controller's configured maximum accelerations (deg/s^2, mm/s^2), or None if it won't say.
        self.joint_acc_max = self._read_limit(self.arm.get_joint_maxacc, 180 / math.pi)
        self.tool_acc_max = self._read_limit(self.arm.get_tcp_maxacc, 1.0)
        self.frame, self.frame_at = None, 0.0
        self.frame_seq = 0
        self.latest = (0, None, 0.0)  # (sequence number, frame, when it came), set in one go for other threads
        self.goal_kind, self.goal = None, None
        self.sent_at = 0.0
        self.heard_at = time.monotonic()
        self.note, self.note_ok = "Connected to the controller.", True
        self.stop_alarm = False  # a stop we couldn't confirm; cleared once the arm is seen stopped
        self.closing = False
        self.brain = brain  # brain_link.BrainLink, or None with --brain off
        # The machine-tending featureset: the cell (tools, frames, targets, zones), programs and their runner.
        self.cell = cellmod.Cell(cell_dir)
        self.counters = cellmod.Counters(cell_dir)
        self.programs = runnermod.Programs(cell_dir)
        self.executor = Executor(self)
        self.runner = runnermod.Runner(self, self.programs, os.path.join(cell_dir, "logs"))
        self._cell_models = {}
        self.pending_points = {}  # frame name -> {"parent", "tool", "origin", "x", "xy"} touched so far
        self.pending_poses = {}  # tool name -> flange poses (4x4) touching one point
        self.freedrive_since = None
        self.teach_level = None  # the teach sensitivity this server last set for freedrive
        # Grippers: what each holds ({gripper: {"mass", "cog"}}), and the payload last sent to the controller, or due to
        # be sent once freedrive ends.
        self.held = {}
        self.payload_sent, self.payload_due = None, False
        self.grip_lock = threading.Lock()  # held while a gripper opens or closes from the page: no move or run starts
        # Every stop() call, begun and ended, and when the last began or ended (monotonic): an Enable the server runs on
        # its own gives way to any STOP since its cause began (apply_payload, enable).
        self.stop_count, self.stops_ended, self.stop_at = 0, 0, -math.inf
        self.stop_lock = threading.Lock()
        self.page_guards = {}  # guarded zones a jog or planned move from the page enters: {zone: [interlocks]}
        self.page_watch = {"at": 0.0}  # when the page's guarded zones' interlocks were last read, holding
        # Calls to the owner: a program's notify steps and its faults, shown until they're seen.
        self.calls, self.call_seq, self.calls_lock = [], 0, threading.Lock()
        # The manual-motion policy, and the hold: until when (monotonic) it's held, and whether it was let go.
        self.reduced_speed, self.hold_to_move = bool(reduced_speed), bool(hold_to_move)
        self.hold_until, self.hold_released = 0.0, False
        self._wake = threading.Event()  # wakes the watchdog when the hold is let go
        threading.Thread(target=self._read_reports, daemon=True).start()
        threading.Thread(target=self._watch, daemon=True).start()
        if self.brain is not None:
            threading.Thread(target=self._watch_brain, daemon=True).start()

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
                    now = time.monotonic()
                    self.frame_seq += 1
                    self.latest = (self.frame_seq, frame, now)
                    self.frame, self.frame_at = frame, now
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

    def _live_joints(self):
        """Measured joint angles [rad] from a frame of the panel's own status stream that arrived after this call: a
        fresh measurement for Enable's checks without another status connection. The real controller serves only a
        few, and holds a closed one open for a while, so opening more made Enable fail (2026-10-21)."""
        asked = time.monotonic()
        deadline = asked + STALL_WAIT_S
        while self.frame is None or self.frame_at <= asked:
            if time.monotonic() >= deadline or self.closing:
                raise arm.ArmError("No live status from the controller to check the joints against; not moving.")
            time.sleep(0.02)
        joints = list(self.frame["joints"][:self.axis])
        if len(joints) != self.axis or not arm.all_finite(joints):
            raise arm.ArmError("The status report returned unusable joint angles; not moving.")
        return joints

    def _busy(self, frame):
        return frame["status"] == 1 or frame["queued"] > 0 or time.monotonic() - self.sent_at < BUSY_AFTER_SEND_S

    def _await_frame(self, since, test, timeout_s):
        """The first status frame to arrive after `since` (monotonic) that passes `test`, or None after timeout_s."""
        deadline = since + timeout_s
        while True:
            frame, at = self.frame, self.frame_at
            if frame is not None and at > since and test(frame):
                return frame
            if time.monotonic() > deadline or self.closing:
                return None
            time.sleep(0.02)

    def _say(self, text, ok=True):
        self.note, self.note_ok = text, ok
        return text

    def heard(self):
        self.heard_at = time.monotonic()

    def status(self):
        frame = self.frame
        age = time.monotonic() - self.frame_at if frame is not None else None
        out = {"ip": self.ip, "simulated": self.simulated, "note": self.note, "noteOk": self.note_ok,
               "stopAlarm": self.stop_alarm, "live": age is not None and age <= LOST_S,
               "age": None if age is None else round(age, 2), "manual": self.manual_status()}
        out["brain"] = self.brain.status() if self.brain is not None else \
            {"url": None, "ok": False, "error": "the brain is off (the server was started with --brain off)"}
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
        out["freedrive"] = in_freedrive(frame)
        out["teachSensitivity"] = self.teach_level  # what this server last set for freedrive (None before)
        out["mode"] = frame["mode"]
        out["cellRevision"] = self.cell.revision
        out["tool"] = self.cell.data["active"]["tool"]
        out["held"] = {name: dict(part) for name, part in self.held.items()}
        out["counters"] = self.counters.snapshot()
        with self.calls_lock:
            out["calls"] = [dict(c) for c in self.calls if not c["seen"]][-10:]
        return out

    # ---- commands --------------------------------------------------------------

    def enable(self, auto=None):
        """Enable's checked sequence: errors reset, the joints checked (no jump), the motors on, ready in position mode.
        `auto`, for an Enable the server runs on its own (after writing the payload stopped the arm: apply_payload), is
        the stop count when its cause began. Then it gives way to any STOP since (StoppedMeanwhile), even one while it
        switches the motors on, which it then makes again; and a controller error it refuses (MoveFailed) rather than
        resets, as one may be what stopped the arm."""
        def stopped():
            return auto is not None and (self.stop_count != auto or self.stops_ended != auto)

        with self.lock:
            frame = self._fresh_frame()
            if self.stop_alarm:
                raise arm.ArmError("A stop wasn't confirmed. Press the hardware E-stop, release it, then hold Enable.")
            if stopped():
                raise StoppedMeanwhile("STOP was pressed meanwhile; the arm stays stopped.")
            if frame["status"] in (1, 3) or frame["queued"]:
                raise arm.ArmError("The arm is still moving or has moves queued. Wait for it, or press STOP first.")
            arm.ensure_idle(self.arm, self.ip)
            ret, codes = self.arm.get_error_code()
            error = codes[0] if ret in arm.OK_CODES else 0
            if auto is not None:
                error = error or frame["err"]
                if error:
                    raise MoveFailed(f"the controller reports error {arm.error_text(error)}, which isn't reset on its "
                                     "own")
            if error:
                if self.arm.reset_err() not in arm.OK_CODES:
                    raise arm.ArmError("The controller refused the error reset.")
                time.sleep(1.5)  # let the controller re-check its E-stop and joints
            if self.payload_due:  # a grasp or release in freedrive that ended without sending it (STOP, say)
                mass, cog = self.payload()
                if self.arm.set_tcp_load(float(mass), [float(v) for v in cog]) not in arm.OK_CODES:
                    raise arm.ArmError("The controller refused the payload the grippers hold, so the motors stay off.")
                self.payload_sent, self.payload_due = (mass, cog), False
            start = arm.preflight(self.arm, self.ip, self.axis, measured=self._live_joints)
            if stopped():
                raise StoppedMeanwhile("STOP was pressed meanwhile; the arm stays stopped.")

            def switch_on():
                arm.prepare_for_motion(self.arm, self.axis, reset=auto is None)
                arm.verified_targets(self.arm, self.ip, self.axis, start, measured=self._live_joints)
                if self.frame["mode"] != 0:  # the frame _live_joints waited for, from after the switch-on
                    raise arm.ArmError(f"After Enable the controller reports motion mode {self.frame['mode']}, not "
                                       "position mode (0).")

            arm.guarded(self.ip, self.arm, switch_on, STOP_LOCK_WAIT_S)
            self.goal_kind = self.goal = None
            self.sent_at = 0.0
            if stopped():  # a STOP while the motors were switched on, which switching them on may have undone: it
                # stands, made again before the lock lets a move in (stop() takes self.lock only in freedrive)
                self.stop("STOP was pressed while Enable made the arm ready")
                raise StoppedMeanwhile("STOP was pressed while Enable made the arm ready; it stays stopped.")
        return self._say("Motors on. Click a key to move one step; hold it to repeat.")

    def stop(self, cause=None):
        """Stop the arm. For automatic stops, `cause` finishes "Stopped because ..."."""
        with self.stop_lock:
            self.stop_count += 1
            self.stop_at = time.monotonic()
        # Over the panel's own connection (the controller serves only one), without waiting for self.lock: the
        # SDK's own lock keeps the stop from interleaving with a request another thread has in flight.
        try:
            confirmed = arm.stop_arm(self.ip, self.arm, STOP_LOCK_WAIT_S)
        finally:
            with self.stop_lock:
                self.stops_ended += 1
                self.stop_at = time.monotonic()
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

    # ---- the manual-motion policy: reduced speed and hold-to-move --------------------------------------------------

    def manual_status(self):
        left = self.hold_until - time.monotonic()
        return {"reducedSpeed": self.reduced_speed, "holdToMove": self.hold_to_move, "held": left > 0,
                "holdLeftMs": max(0, int(left * 1000))}

    def manual_text(self):
        reduced = (f"Reduced speed on: moves keep the tool point and flange under {TOOL_REDLINE_MM_S:g} mm/s"
                   if self.reduced_speed else "Reduced speed off")
        hold = ("hold-to-move on: nothing moves unless the hold-to-move key is held" if self.hold_to_move
                else "hold-to-move off")
        return f"{reduced}; {hold}."

    def set_manual(self, body):
        """POST /api/manual {"reducedSpeed": bool, "holdToMove": bool}, either optional. A change is refused while the
        arm moves: the server's moves follow one policy from start to end."""
        wanted = {}
        for key in ("reducedSpeed", "holdToMove"):
            if key in body:
                if not isinstance(body[key], bool):
                    raise arm.ArmError(f"Unsupported {key}: {body[key]!r} (true or false).")
                wanted[key] = body[key]
        # Under both locks, so no jog, planned move or run can plan by one policy and be sent under the other.
        with self.lock, self.runner.lock:
            reduced = wanted.get("reducedSpeed", self.reduced_speed)
            hold = wanted.get("holdToMove", self.hold_to_move)
            if (reduced, hold) != (self.reduced_speed, self.hold_to_move):
                if self.runner.moving:
                    raise arm.ArmError("A program is running. Pause or stop it first.")
                if self._page_move_under_way():
                    raise arm.ArmError("The arm is moving. Wait for it to stop, or press STOP.")
                self.reduced_speed, self.hold_to_move = reduced, hold
        return self._say(self.manual_text())

    def hold(self, ms):
        """POST /api/hold {"ms": N}: hold-to-move held for N ms from now (HOLD_MIN_MS to HOLD_MAX_MS), or let go (0)."""
        if isinstance(ms, bool) or not isinstance(ms, (int, float)) or not math.isfinite(ms) \
                or not (ms == 0 or HOLD_MIN_MS <= ms <= HOLD_MAX_MS):
            raise arm.ArmError(f"Unsupported hold: {ms!r} ms (0 lets go; hold for {HOLD_MIN_MS} to {HOLD_MAX_MS}).")
        if ms == 0:
            self.hold_until, self.hold_released = 0.0, True
            if self.hold_to_move:
                self._wake.set()  # the watchdog stops a move at once
        else:
            self.hold_until, self.hold_released = time.monotonic() + ms / 1000.0, False
        return self.manual_status()

    def hold_lapsed(self):
        """Why motion must stop, when hold-to-move is on and nothing holds it (the key let go, or its hold lapsed);
        None otherwise."""
        if not self.hold_to_move or time.monotonic() < self.hold_until:
            return None
        return "the hold-to-move key was let go" if self.hold_released else "the hold-to-move key's hold lapsed"

    def require_hold(self):
        """Refuses (ArmError) to start motion while hold-to-move is on and nothing holds it."""
        if self.hold_lapsed():
            raise arm.ArmError("Not moving: hold-to-move is on, and the hold-to-move key isn't held.")

    def _page_move_under_way(self):
        """Whether a jog or planned move sent from the page is still moving the arm, or about to (not freedrive)."""
        frame = self.frame
        return frame is not None and frame["mode"] != 2 and bool(self.sent_at) and self._busy(frame)

    def _reduced_joint_cap(self, before, after):
        """Reduced speed's cap (deg/s) on a joint move between two sets of joint targets (rad), or None."""
        return plannermod.reduced_joint_speed(self.model, [math.degrees(v) for v in before[:self.axis]],
                                              [math.degrees(v) for v in after[:self.axis]], [self.cell.tool_matrix()])

    def _tool_reach_mm(self):
        """How far the tool point in use is from the flange (mm)."""
        return math.dist(kin.position(self.cell.tool_matrix()), (0.0, 0.0, 0.0))

    # ---- the cell, planning and the runner's needs -------------------------------------------------------------

    def cell_model(self, passable=()):
        """The arm model with the active tool's shape and the cell's zones (but the guarded zones in `passable`, open
        while their interlocks hold), rebuilt when the cell changes."""
        key = (self.cell.revision, self.cell.data["active"]["tool"])
        models = self._cell_models
        if models.get("key") != key:
            models = self._cell_models = {"key": key}
        passable = frozenset(passable)
        if passable not in models:
            models[passable] = self.cell.collision_model(self.model, passable=passable)
        return models[passable]

    def planner(self, passable=()):
        if self.model is None:
            raise arm.ArmError("The arm model didn't load, so planned moves are off.")
        return plannermod.Planner(self.cell_model(passable), self.joint_acc_max, self.tool_acc_max,
                                  reduced=self.cell.tool_matrix() if self.reduced_speed else None)

    def payload(self):
        """What the controller should carry: the tool in use and every part its grippers hold, as (mass kg, centre of
        gravity mm from the flange). A part's centre of gravity is given from the tool point, in its frame."""
        tool = self.cell.tool()
        mass = float(tool["mass"])
        moment = [mass * float(c) for c in tool["cog"]]
        tcp = self.cell.tool_matrix()
        for part in self.held.values():
            m = float(part.get("mass", 0.0))
            if m > 0:
                at = kin.apply(tcp, part.get("cog", [0.0, 0.0, 0.0]))
                mass += m
                moment = [a + m * b for a, b in zip(moment, at)]
        cog = [v / mass for v in moment] if mass > 0 else [0.0, 0.0, 0.0]
        return round(mass, 4), [round(v, 3) for v in cog]

    def send_payload(self):
        """Tell the controller the payload (mass kg, centre of gravity mm from the flange): the active tool's, and any
        part its grippers hold. The SDK asks for it whenever the payload changes; the controller's own tool offset stays
        zero (decision D14)."""
        mass, cog = self.payload()
        with self.lock:
            ret = self.arm.set_tcp_load(float(mass), [float(v) for v in cog])
        if ret not in arm.OK_CODES:
            raise arm.ArmError(f"The controller refused the payload (code {ret}).")
        self.payload_sent, self.payload_due = (mass, cog), False

    def apply_payload(self, begun=None, stops=None):
        """After a grasp or a release: send the payload if it changed. In freedrive it waits until freedrive ends (the
        hand holds the arm meanwhile, and writing a setting may stop the arm), or the next Enable. If writing it stops
        a ready arm, as writing the teach sensitivity does (unverified for the payload on the arm), Enable's checked
        sequence makes the arm ready again, run as the server's own (enable's `auto`): only if no STOP has come since
        the grip began (`begun`, monotonic, when the stop count was `stops`: grip), none is under way, and the arm was
        seen ready since, after any STOP before it had taken effect. A controller error after the write is a fault
        (MoveFailed), never reset here, and with the run's stop pending the arm stays stopped. (A pause pending is no
        reason to leave it stopped: the run pauses between steps, where it leaves the arm ready.) Returns a note for the
        message."""
        if begun is None:
            begun, stops = time.monotonic(), self.stop_count
        mass, cog = self.payload()
        if self.payload_sent == (mass, cog):
            return ""
        frame = self.frame
        if frame is not None and frame["mode"] == 2:
            self.payload_due = True
            return f" The payload ({mass:g} kg) goes to the controller when freedrive ends."
        with self.stop_lock:
            quiet, since = self.stop_count == self.stops_ended == stops, max(begun, self.stop_at + STOP_SEEN_S)
        ready = quiet and self._ready_since(since)
        written = time.monotonic()
        self.send_payload()
        note = f" Payload {mass:g} kg sent."
        if ready:
            after = self._await_frame(written + 0.1, lambda f: True, 1.0)  # a status frame from after the write
            if after is not None and after["status"] == 4:
                if after["err"]:
                    raise MoveFailed(f"the controller reported error {arm.error_text(after['err'])} as the payload "
                                     "was written")
                held = " The arm stopped as it was written, and stays stopped: a stop came meanwhile."
                if self.stop_count != stops or (self.runner.busy and self.runner.request == "stop"):
                    return note + held
                try:
                    self.enable(auto=stops)
                except StoppedMeanwhile:
                    return note + held
                except arm.ArmError as e:
                    raise MoveFailed(f"writing the payload stopped the arm, and Enable couldn't make it ready again: "
                                     f"{str(e).rstrip('.')}")
                note += " Writing it stopped the arm, so Enable's checks made it ready again."
        return note

    def _ready_since(self, since):
        """Whether the arm was ready (motors on, no error, still or moving) on the latest status frame, once one has
        come after `since` (monotonic); False if none comes within FRAME_WAIT_S past it (or past now, if later)."""
        deadline = max(since, time.monotonic()) + FRAME_WAIT_S
        while True:
            _, frame, at = self.latest
            if frame is not None and at > since:
                return frame["status"] in (0, 2) and not frame["err"] and frame["mode"] != 2 and \
                    frame["enable"] & self.mask == self.mask
            if time.monotonic() > deadline or self.closing:
                return False
            time.sleep(0.02)

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

    def _target_for(self, step):
        """The target a move step names, after the checks every move to it makes: (target, label)."""
        target = self.cell.target(step["target"])
        if "joints" in target:
            return target, step["target"]
        active = self.cell.data["active"]["tool"]
        if target["tool"] != active:
            raise plannermod.PlanError(f"{step['target']!r} was taught with the tool {target['tool']!r}, but the active "
                                       f"tool is {active!r}.")
        if target.get("unreachable"):
            raise plannermod.PlanError(f"{step['target']!r} became unreachable when its frame was recalibrated; "
                                       "teach it again.")
        return target, step["target"] + (" (offset)" if step.get("offset") else "")

    def _goal(self, step):
        """Where a move step goes, after the checks every move there makes: {"joints"} for a joint target, else
        {"pose" (base <- tool point, 4x4), "seed"}; with its "label". A step to a pattern names its slot."""
        if step.get("pattern") is not None:
            name, slot = step["pattern"], step.get("slot")
            if slot is None:
                raise plannermod.PlanError(f"Name a slot of {name} to plan a move to it.")
            s = self.cell.slot(name, slot)
            active = self.cell.data["active"]["tool"]
            if s["tool"] != active:
                raise plannermod.PlanError(f"{name} was taught with the tool {s['tool']!r}, but the active tool is "
                                           f"{active!r}.")
            return {"pose": self.cell.slot_matrix(name, slot, step.get("offset")), "seed": s["seed"],
                    "label": f"{name} slot {slot}" + (" (offset)" if step.get("offset") else "")}
        target, label = self._target_for(step)
        if "joints" in target:
            return {"joints": [float(v) for v in target["joints"]], "label": label}
        return {"pose": self.cell.target_matrix(step["target"], step.get("offset")), "seed": target.get("seed"),
                "label": label}

    def plan_move(self, step, start, speed, ori_speed=60.0, dry=False):
        """A Plan for a program's move step (a joint or straight move to a target or a pattern's slot, with an optional
        offset), with guarded zones open while their interlocks hold (plan_guarded)."""
        if step["kind"] == "planned":
            raise plannermod.PlanError("A planned move's path comes from the brain: run it as a program step.")
        goal = self._goal(step)
        if "joints" in goal:
            if step["kind"] != "joint":
                raise plannermod.PlanError(f"{step['target']!r} is a joint target; a straight line needs a tool target.")
            return self.plan_guarded(lambda p: p.joint(start, goal["joints"], speed, goal["label"]), start, dry)
        tool = self.cell.tool_matrix()
        if step["kind"] == "joint":
            return self.plan_guarded(lambda p: p.joint_to_pose(start, goal["pose"], tool, speed, seed=goal["seed"],
                                                               label=goal["label"]), start, dry)
        return self.plan_guarded(lambda p: p.line(start, goal["pose"], tool, speed, ori_speed, label=goal["label"]),
                                 start, dry)

    def plan_retract(self, start, goal, speed, ori_speed=30.0, dry=False):
        """Safe home's retract: the straight move from `start` back along the tool to `goal` (4x4), or, if it can't all
        be planned, as far towards it as can (the line's clear part). Returns (plan or None, mm it goes, why it stops
        short or None). If the arm is in a guarded zone whose interlocks don't hold, it doesn't go at all (ZoneClosed:
        plan_guarded)."""
        tool = self.cell.tool_matrix()
        here = self.tool_point(start)
        wanted = math.dist(kin.position(here), kin.position(goal))
        label = f"{wanted:.0f} mm back along the tool"
        try:
            return self.plan_guarded(lambda p: p.line(start, goal, tool, speed, ori_speed, label=label), start,
                                     dry), wanted, None
        except plannermod.PlanError as e:
            if isinstance(e, plannermod.ZoneClosed) and e.inside:
                raise
            clear = e.clear or 0.0
            why = str(e)[len("Not moving: "):] if str(e).startswith("Not moving: ") else str(e)
            if clear * wanted < 1.0:
                return None, 0.0, why
            part = kin.interpolate(here, goal, clear)
            plan = self.plan_guarded(lambda p: p.line(start, part, tool, speed, ori_speed,
                                                      label=f"{clear * wanted:.0f} mm back along the tool"), start, dry)
            return plan, clear * wanted, why

    # ---- interlocks and the zones they guard ---------------------------------------------------------------------

    def _input_word(self, line, locked=False):
        """A line's digital inputs as a 16-bit word. `locked`: the caller holds self.lock already."""
        def read():
            return (self.arm.get_tgpio_in if line == "tool" else self.arm.get_cgpio_in)()
        if locked:
            st, values = read()
        else:
            with self.lock:
                st, values = read()
        if not isinstance(values, list) or len(values) < 2:
            raise MoveFailed(f"couldn't read the {line}'s inputs (code {st})")
        return int(values[1]) & 0xFFFF

    def interlock_states(self, names=None, locked=False, words=None):
        """{interlock: {"input", "value", "reads", "holds"}}, read now (or from given input `words`)."""
        interlocks = self.cell.data["interlocks"]
        names = list(interlocks) if names is None else list(names)
        words = dict(words or {})
        out = {}
        for name in names:
            il = interlocks[name]
            line, bit = cellmod.check_io(il["input"], "input")
            if line not in words:
                words[line] = self._input_word(line, locked)
            reads = (words[line] >> bit) & 1
            out[name] = {"input": il["input"], "value": il["value"], "reads": reads, "holds": reads == il["value"]}
        return out

    @staticmethod
    def interlock_text(failing, states):
        """'door_open doesn't hold (controller.2 reads 0, not 1)' for the interlocks in `failing`."""
        readings = ", ".join(f"{states[n]['input']} reads {states[n]['reads']}, not {states[n]['value']}" for n in failing)
        names = " and ".join(failing)
        return f"{names} {'doesn' if len(failing) == 1 else 'don'}'t hold ({readings})"

    def zones_in(self, joints, zones=None):
        """The guarded zones (or those named in `zones`) the arm, with the tool in use, is in at `joints` (deg):
        {zone: (gap m, negative: minus how far into it the part deepest in reaches; that part)}."""
        names = set(self.cell.guarded_zones()) if zones is None else set(zones)
        if not names or self.model is None:
            return {}
        model = self.cell_model()
        index = {i: o.zone for i, o in enumerate(model.obstacles) if getattr(o, "zone", None) in names}
        return {index[i]: g for i, g in model.obstacle_gaps(joints, set(index)).items() if g[0] < 0.0}

    def _closed(self, names, locked=False):
        """The interlocks among `names` that don't hold now, and every state read: (failing, states)."""
        states = self.interlock_states(names, locked)
        return [n for n in names if not states[n]["holds"]], states

    def _in_closed(self, zone, at, failing, states):
        """The refusal (ZoneClosed) of a move that starts with the arm in the guarded `zone` while its interlocks
        `failing` don't hold: `at` is (gap, part) from zones_in."""
        gap, part = at
        return plannermod.ZoneClosed(f"Not moving: {part} is {-gap * 1000:.0f} mm into the {zone} zone, which is "
                                     f"closed, as {self.interlock_text(failing, states)}. Nothing moves the arm in it "
                                     f"until {' and '.join(failing)} {'holds' if len(failing) == 1 else 'hold'}; or "
                                     "guide the arm out by hand, in freedrive.", zone, inside=True)

    @staticmethod
    def _zone_of(model, hit):
        """The cell zone a collision check's hit is with, or None."""
        if not hit or hit.get("obstacle") is None:
            return None
        return getattr(model.obstacles[hit["obstacle"]], "zone", None)

    def _open_or_refuse(self, zone, guarded, error, locked=False):
        """A move would enter the guarded `zone` (or go deeper into it): read its interlocks, and refuse (ZoneClosed)
        unless they all hold."""
        failing, states = self._closed(guarded[zone], locked)
        if failing:
            hit = getattr(error, "hit", None)
            raise plannermod.ZoneClosed(f"{str(error).rstrip('.')}: the {zone} zone is closed, as "
                                        f"{self.interlock_text(failing, states)}.", zone, hit,
                                        getattr(error, "clear", None), inside=bool(hit and hit.get("inside")))

    def plan_guarded(self, plan_fn, start=None, dry=False, locked=False):
        """plan_fn(planner) -> Plan from `start` (deg), planned against every zone but the guarded ones it may go into:
        - one the plan would enter: its interlocks are read, and if they all hold it's planned again with the zone open;
          otherwise it's refused (ZoneClosed), saying which doesn't hold;
        - one the arm is in already (zones_in, at `start`): open if its interlocks hold, whatever the move does in it;
          refused (ZoneClosed) if they don't, whatever asked for the move, the page included: a box can't say which way
          is out, and the door may be shut on the arm (docs/decisions.md D121).
        A dry run assumes the interlocks hold. The plan's `guards` are the guarded zones the move depends on, with their
        interlocks, which are watched as the arm moves."""
        guarded = self.cell.guarded_zones()
        passable = set()
        inside = self.zones_in(start, guarded) if guarded and start is not None else {}
        for zone in sorted(inside):
            if not dry:
                failing, states = self._closed(guarded[zone], locked)
                if failing:
                    raise self._in_closed(zone, inside[zone], failing, states)
            passable.add(zone)
        while True:
            planner = self.planner(passable)
            try:
                plan = plan_fn(planner)
            except plannermod.PlanError as e:
                zone = self._zone_of(planner.model, e.hit)
                if zone is None or zone not in guarded or zone in passable:
                    raise
                if not dry:
                    self._open_or_refuse(zone, guarded, e, locked)
                passable.add(zone)
                continue
            plan.guards = {zone: guarded[zone] for zone in sorted(passable)}
            return plan

    def check_guarded(self, start_deg, goal_deg, locked=False):
        """The cell's collision check of a joint move from the page (a jog or Move), with the guarded zones it may go
        into open, as plan_guarded has them: (why it's refused, or None; the guarded zones it depends on, with their
        interlocks)."""
        guarded = self.cell.guarded_zones()
        passable = set()
        inside = self.zones_in(start_deg, guarded) if guarded else {}
        for zone in sorted(inside):
            failing, states = self._closed(guarded[zone], locked)
            if failing:
                return str(self._in_closed(zone, inside[zone], failing, states)), {}
            passable.add(zone)
        while True:
            model = self.cell_model(passable)
            hit = model.check_path(start_deg, goal_deg)
            if hit is None:
                return None, {zone: guarded[zone] for zone in sorted(passable)}
            zone = self._zone_of(model, hit)
            if zone not in guarded or zone in passable:
                return blocked(hit), {}
            failing, states = self._closed(guarded[zone], locked)
            if failing:
                return f"{blocked(hit)[:-1]}: the {zone} zone is closed, as {self.interlock_text(failing, states)}.", {}
            passable.add(zone)

    def zones_for_brain(self, joints, dry=False):
        """The guarded zones a path or burst from the brain may go into, from `joints` (deg): those whose interlocks
        all hold, read now (in a dry run, all of them: it assumes they'll hold). Refuses (ZoneClosed) if the arm is in
        one whose interlocks don't: the server doesn't move the arm in a closed zone on its own."""
        guarded = self.cell.guarded_zones()
        if not guarded or dry:
            return set(guarded)
        states = self.interlock_states(sorted({n for names in guarded.values() for n in names}))
        failing = {zone: [n for n in names if not states[n]["holds"]] for zone, names in guarded.items()}
        inside = self.zones_in(joints, [zone for zone, f in failing.items() if f])
        for zone in sorted(inside):
            raise self._in_closed(zone, inside[zone], failing[zone], states)
        return {zone for zone, f in failing.items() if not f}

    def zones_entered(self, path, passable):
        """The guarded zones among `passable` (open) that a joint-space path from the brain (waypoints, deg) depends on,
        with their interlocks: those the arm is in at its start, and those it comes within the margin of on the way."""
        if not passable or self.model is None:
            return {}
        guarded = self.cell.guarded_zones()
        out = {zone: guarded.get(zone, []) for zone in self.zones_in(path[0], passable)}
        for zone in sorted(set(passable) - set(out)):
            model = self.cell_model(set(passable) - {zone})
            limits = model.start_limits(path[0])  # the whole path's allowance, not each segment's
            if any(model.check_path(a, b, limits=limits) is not None for a, b in zip(path, path[1:])):
                out[zone] = guarded.get(zone, [])
        return dict(sorted(out.items()))

    def guard_dropped(self, guards, locked=False):
        """For a move into guarded zones ({zone: [interlocks]}): (zone, why) if an interlock no longer holds, or they
        can't be read; None while they all hold. Never raises."""
        if not guards:
            return None
        names = sorted({n for v in guards.values() for n in v})
        try:
            states = self.interlock_states(names, locked)
        except Exception as e:  # can't confirm they hold: stop
            return sorted(guards)[0], f"its interlocks couldn't be read ({e})"
        for zone, ils in sorted(guards.items()):
            failing = [n for n in ils if not states[n]["holds"]]
            if failing:
                return zone, self.interlock_text(failing, states)
        return None

    def watch_guards(self, guards, watch):
        """guard_dropped, read a few times a second while the arm moves (the page's watchdog, a run's supervision). It
        waits at most WATCH_LOCK_WAIT_S for the SDK, which a request can hold for seconds while the status stream
        stalls, so the other reasons to stop are never held up behind it; but if the interlocks can't be read within
        GUARD_CONFIRM_S of when they last were (`watch["at"]`, monotonic), that's a stop too."""
        if not guards:
            return None
        if not self.lock.acquire(timeout=WATCH_LOCK_WAIT_S):
            if time.monotonic() - watch["at"] > GUARD_CONFIRM_S:
                return sorted(guards)[0], f"its interlocks couldn't be read for {GUARD_CONFIRM_S:g} s"
            return None
        try:
            dropped = self.guard_dropped(guards, locked=True)
        finally:
            self.lock.release()
        if dropped is None:
            watch["at"] = time.monotonic()
        return dropped

    # ---- grippers ------------------------------------------------------------------------------------------------

    def read_signal(self, signal):
        """An input named in a {"input", "value"} (0 or 1)."""
        line, bit = cellmod.check_io(signal["input"], "input")
        return self.read_input(line, bit)

    def grip(self, name, action, part=None, sleep=time.sleep, log=lambda event, **detail: None):
        """Open, close or grasp with gripper `name`: set its outputs, then wait for the input that confirms it (or,
        without one, its set time). A grasp then reads the gripper's "closed on a part" input, if it has one, failing
        (runner.ProgramError) when there's no part, and switches the payload to the tool and the part together; opening
        switches it back. `sleep` waits (a run's checks its pause, stop and the controller as it waits). Returns what
        happened, for the message."""
        g = self.cell.gripper(name)
        begun, stops = time.monotonic(), self.stop_count  # a STOP from now on keeps the arm stopped (apply_payload)
        outputs = g["open"] if action == "open" else g["close"]
        for o in outputs:
            self.write_output(o["output"], o["value"])
        log("gripper.set", gripper=name, action=action, outputs={o["output"]: o["value"] for o in outputs})
        started = time.monotonic()
        confirm = g.get("opened") if action == "open" else g.get("closed")
        verb = "opened" if action == "open" else "closed"
        if confirm is not None:
            while self.read_signal(confirm) != confirm["value"]:
                if time.monotonic() - started > g["timeout"]:
                    raise runnermod.ProgramError(f"{name} didn't confirm it {verb} within {g['timeout']:g} s "
                                                 f"({confirm['input']} doesn't read {confirm['value']})")
                sleep(GRIP_POLL_S)
        else:
            end = started + g["seconds"]
            while time.monotonic() < end:
                sleep(min(GRIP_POLL_S, max(0.0, end - time.monotonic())))
        how = f"{confirm['input']} confirmed it" if confirm is not None else f"after its {g['seconds']:g} s"
        if action == "grasp" and g.get("part") is not None:
            sensor, settle = g["part"], time.monotonic() + PART_SETTLE_S
            while True:
                got = self.read_signal(sensor)
                if got == sensor["value"]:
                    break
                if time.monotonic() > settle:
                    raise runnermod.ProgramError(f"{name} closed, but {sensor['input']} reads {got}, not "
                                                 f"{sensor['value']}: there's no part in it")
                sleep(GRIP_POLL_S)
            how += f"; {sensor['input']} says it holds a part"
        if action == "grasp":
            self.held[name] = dict(part or {"mass": 0.0, "cog": [0.0, 0.0, 0.0]})
        elif action == "open":
            self.held.pop(name, None)
        note = self.apply_payload(begun, stops) if action in ("grasp", "open") else ""
        seconds = round(time.monotonic() - started, 2)
        detail = {"payload": self.payload()[0]} if action != "close" else {}
        if note:
            detail["note"] = note.strip()
        log("gripper.done", gripper=name, action=action, seconds=seconds, **detail)
        word = {"open": "opened", "close": "closed", "grasp": "grasped"}[action]
        return f"{name} {word} in {seconds:g} s ({how}).{note}"

    def gripper_states(self, io):
        """Each gripper as its outputs and inputs read now (`io`: io_state()): whether its outputs are set to close it
        or open it, each confirming input's reading, and the part it holds, if any."""
        def bit(line_name, kind):
            line, n = cellmod.check_io(line_name, kind)
            word = io[line]["inputs" if kind == "input" else "outputs"]
            return None if word is None else (word >> n) & 1
        out = {}
        for name, g in self.cell.data["grippers"].items():
            def matches(settings):
                return all(bit(o["output"], "output") == o["value"] for o in settings)
            state = "closed" if matches(g["close"]) else "open" if matches(g["open"]) else "neither"
            entry = {"outputs": state, "held": self.held.get(name)}
            for key in ("closed", "opened", "part"):
                if g.get(key) is not None:
                    entry[key] = bit(g[key]["input"], "input") == g[key]["value"]
            out[name] = entry
        return out

    # ---- calling the owner ---------------------------------------------------------------------------------------

    def call_owner(self, message, program=None, path=None, why="step"):
        """A call to the owner: kept, and shown in the panel (status "calls") until it's seen; a run's log has it too.
        A phone notification can follow the same calls later."""
        with self.calls_lock:
            self.call_seq += 1
            call = {"id": self.call_seq, "at": time.strftime("%Y-%m-%dT%H:%M:%S"), "message": str(message)[:500],
                    "program": program, "path": path, "why": why, "seen": False}
            self.calls.append(call)
            del self.calls[:-CALLS_KEPT]
        return dict(call)

    def calls_seen(self, call_id=None):
        """Marks a call (or, without an id, every call) as seen. Returns how many were marked."""
        with self.calls_lock:
            marked = 0
            for call in self.calls:
                if not call["seen"] and (call_id is None or call["id"] == call_id):
                    call["seen"] = True
                    marked += 1
        return marked

    def why_not_ready(self):
        """Why a run couldn't carry on moving the arm now, or None if it can: as check_can_run, without changing
        anything."""
        try:
            frame = self._fresh_frame()
        except arm.ArmError:
            return "there's no fresh status from the controller"
        if self.stop_alarm:
            return "a stop wasn't confirmed"
        if frame["err"]:
            return f"the controller reports error {arm.error_text(frame['err'])}"
        if frame["mode"] == 2:
            return "the arm is in freedrive"
        if frame["enable"] & self.mask != self.mask:
            return "the motors are off"
        if frame["status"] not in (0, 2):
            return f"the arm isn't ready (state {frame['status']})"
        return None

    # ---- what the brain is told, and asked ------------------------------------------------------------------------

    def goal_joints(self, step, start):
        """Where a planned move ends (deg): a joint target's joints, or a tool target's (or a pattern slot's) solved
        from the joints it was taught at, so the arm arrives in the taught configuration. Returns (joints, label)."""
        goal = self._goal(step)
        if "joints" in goal:
            for i, g in enumerate(goal["joints"]):
                low, high = DRAG_LIMITS_DEG[i]
                if not low <= g <= high:
                    raise plannermod.PlanError(f"J{i + 1} would have to go to {g:.1f}\u00b0, outside {low:g}\u00b0 to "
                                               f"{high:g}\u00b0.")
            return goal["joints"], goal["label"]
        return self.planner().solve(goal["seed"] or start, goal["pose"], self.cell.tool_matrix()), goal["label"]

    def observe(self, after=None, inputs=False, virtual=None):
        """The arm's state for the brain: a fresh status frame's sequence number and arrival time (one that came after
        `after`, if given), the measured joints (deg) and the active tool point's pose, with the digital inputs'
        words if asked. A dry run's `virtual` joints stand in as state 0."""
        if virtual is not None:
            q = [float(v) for v in virtual]
            return {"seq": 0, "at": time.monotonic(), "joints": q, "tool": kin.matrix_to_pose(self.tool_point(q), 4),
                    "virtual": True}
        deadline = time.monotonic() + STALL_WAIT_S
        while True:
            seq, frame, at = self.latest
            now = time.monotonic()
            if frame is not None and now - at <= FRESH_S and (after is None or at > after):
                break
            if now > deadline or self.closing:
                raise MoveFailed("there's no fresh status from the controller to tell the brain")
            time.sleep(0.02)
        q = [math.degrees(v) for v in frame["joints"][:self.axis]]
        if not arm.all_finite(q):
            raise MoveFailed("the status report has unusable joint angles")
        out = {"seq": seq, "at": at, "joints": q, "tool": kin.matrix_to_pose(self.tool_point(q), 4)}
        if inputs:
            out["inputs"] = self.input_words()
        return out

    def latest_joints(self):
        """The latest measured joints (deg) and their age (s); (None, inf) before any status."""
        _, frame, at = self.latest
        if frame is None:
            return None, math.inf
        return [math.degrees(v) for v in frame["joints"][:self.axis]], time.monotonic() - at

    def input_words(self):
        """Every digital input, as a 16-bit word for the tool and one for the controller."""
        out = {}
        for line in ("tool", "controller"):
            with self.lock:
                st, values = (self.arm.get_tgpio_in if line == "tool" else self.arm.get_cgpio_in)()
            if not isinstance(values, list) or len(values) < 2:
                raise MoveFailed(f"couldn't read the {line}'s inputs (code {st})")
            out[line] = int(values[1]) & 0xFFFF
        return out

    def world(self, passable=()):
        """What the spine knows of the cell, for the brain (mm, base frame): the active tool's point and shape, and
        the zones as boxes, leaving out the guarded zones in `passable` (open while their interlocks hold)."""
        with self.cell.lock:
            data = self.cell.data
            name = data["active"]["tool"]
            tool = data["tools"][name]
            zones = [{"name": o.zone, "centre": [round(v * 1000.0, 3) for v in o.centre],
                      "axes": [[round(v, 9) for v in axis] for axis in o.axes], "size": [round(h * 2000.0, 3) for h in o.half]}
                     for o in self.cell.zone_obstacles(passable)]
            return {"cellRevision": data["revision"], "marginMm": utra_model.MARGIN_M * 1000.0,
                    "tool": {"name": name, "tcp": {"xyz": list(tool["tcp"]["xyz"]), "rpy": list(tool["tcp"]["rpy"])},
                             "capsules": [dict(c) for c in tool["capsules"]]},
                    "zones": zones}

    def anchor_point(self, name):
        """The centre of a learned step's region (mm, base frame): the tool point at the target `name`."""
        target = self.cell.target(name)
        if "joints" in target:
            return kin.position(self.tool_point(target["joints"]))
        return kin.position(self.cell.target_matrix(name))

    def check_brain(self, plans, skills):
        """Refuses (runner.ProgramError) unless the brain answers now, plans paths (if `plans`) and has `skills`."""
        if self.brain is None:
            raise runnermod.ProgramError("This program has planned or learned steps, but the brain is off (the server "
                                         "was started with --brain off).")
        known = self.brain.hello()
        if not known["ok"]:
            raise runnermod.ProgramError(f"This program has planned or learned steps, but {known['error']}. Start the "
                                         "brain (python brain/stub/stub_brain.py), then try again.")
        if plans and not known["planner"]:
            raise runnermod.ProgramError(f"This program has planned moves, but the brain ({known['brain']}) doesn't "
                                         "plan paths.")
        missing = [k for k in skills if k not in known["skills"]]
        if missing:
            raise runnermod.ProgramError(f"The brain ({known['brain']}) has no skill called {', '.join(missing)}.")
        return known

    def _watch_brain(self):
        """Asks the brain who it is every few seconds, so the panel shows whether it's there."""
        while not self.closing:
            self.brain.hello()
            time.sleep(brain_link.HELLO_EVERY_S)

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
        return (self._input_word(line) >> bit) & 1

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

    def freedrive(self, on, sensitivity=None, reenable=True):
        """Enter freedrive (motion mode 2, joint teaching) at a teach sensitivity in TEACH_SENSITIVITIES, or leave it.

        Umbratek's order (their SDK's readme, and the controller's own dashboard): the settings first, then the motors on
        and the arm ready, then mode 2. What this arm did (2026-10-21, tools/capture/teach_probe.py): writing the teach
        sensitivity stops it; mode 2 is ignored while it's stopped, and taken at once from ready, after which it reports
        state 1; STOP holds it rigid, still in mode 2; leaving mode 2 switches the motors off onto their brakes (J3
        settles about 0.45 deg). So entering may run Enable's checked sequence first, and leaving runs it again after,
        unless `reenable` is False (freedrive timing out, with nobody at the arm)."""
        if not on:
            return self._leave_freedrive(reenable)
        level = pick(TEACH_SENSITIVITY_DEFAULT if sensitivity is None else sensitivity, TEACH_SENSITIVITIES,
                     "teach sensitivity")
        if self.runner.busy:
            raise arm.ArmError("A program is running. Stop it before freedrive.")
        if self.grip_lock.locked():
            raise arm.ArmError("A gripper is opening or closing. Wait for it, then hold freedrive.")
        frame = self._fresh_frame()
        if frame["err"] or self.stop_alarm:
            raise arm.ArmError("Clear the fault first (hold Enable).")
        if frame["mode"] == 2:
            raise arm.ArmError("Already in freedrive: to change the sensitivity, leave freedrive first."
                               if in_freedrive(frame) else "The arm is stopped in freedrive mode. Hold Enable first.")
        if frame["enable"] & self.mask != self.mask:
            raise arm.ArmError("The motors are off. Hold Enable first.")
        if frame["status"] in (1, 3) or self._busy(frame):
            raise arm.ArmError("The arm is moving. Wait for it to stop.")
        with self.lock:
            ret, current = self.arm.get_teach_sens()
            written = None
            if ret not in arm.OK_CODES or current != level:
                written = time.monotonic()
                if self.arm.set_teach_sens(int(level)) not in arm.OK_CODES:
                    raise arm.ArmError("The controller refused the teach sensitivity.")
        self.teach_level = int(level)
        if written is not None:  # the write stops the arm: wait for that, then make it ready again
            self._await_frame(written, lambda f: f["status"] == 4, 1.0)
        if self._fresh_frame()["status"] == 4:
            self.enable()  # Enable's checks (no jump) and sequence: ready in position mode
        with self.lock:
            frame = self._fresh_frame()
            if frame["status"] not in (0, 2) or frame["mode"] != 0 or frame["err"]:
                raise arm.ArmError(f"The arm isn't ready for freedrive (state {frame['status']}, mode {frame['mode']}, "
                                   f"error {frame['err']}). Hold Enable, then try again.")
            asked = time.monotonic()
            ret = self.arm.set_motion_mode(2)
        if ret not in arm.OK_CODES:
            raise arm.ArmError(f"The controller refused freedrive (code {ret}); the arm holds, ready.")
        frame = self._await_frame(asked, in_freedrive, FREEDRIVE_CONFIRM_S)
        if frame is None:
            self.stop("the controller didn't enter freedrive")  # and leaves mode 2 if it took it: motors off
            last = self.frame or {}
            raise arm.ArmError(f"The controller didn't enter freedrive (it reports state {last.get('status')}, mode "
                               f"{last.get('mode')}, error {last.get('err')}); stopped. Hold Enable to carry on.")
        self.freedrive_since = time.monotonic()
        return self._say(f"Freedrive on (sensitivity {level}%): guide the arm by hand and record targets as you go. "
                         "Leaving switches the motors off and on again.")

    def _leave_freedrive(self, reenable):
        frame = self._fresh_frame()
        self.freedrive_since = None
        if frame["mode"] != 2:
            return self._say("Not in freedrive.")
        with self.lock:
            asked = time.monotonic()
            ret = self.arm.set_motion_mode(0)
        if ret not in arm.OK_CODES or self._await_frame(asked, lambda f: f["mode"] != 2, FREEDRIVE_CONFIRM_S) is None:
            confirmed, _ = self.stop("the controller didn't confirm leaving freedrive")
            if not confirmed:
                raise arm.ArmError("STOP NOT CONFIRMED leaving freedrive: press the hardware E-stop now.")
            raise arm.ArmError("The controller didn't confirm leaving freedrive; stopped. Hold Enable to carry on.")
        note = ""
        if self.payload_due:  # a grasp or release in freedrive: the payload goes now, with the motors off
            self.send_payload()
            note = f" The payload it holds now ({self.payload_sent[0]:g} kg) went to the controller."
        if not reenable:
            return self._say("Freedrive off: the motors are off and the arm rests on its brakes. Hold Enable to carry "
                             "on." + note)
        try:
            self.enable()  # Enable's checks (no jump) and sequence: ready in position mode
        except arm.ArmError as e:
            raise arm.ArmError(f"Freedrive off; the motors are off and the arm rests on its brakes. {e}")
        return self._say("Freedrive off: motors back on, ready to move. The arm settled onto its brakes first "
                         "(about 0.5 mm); targets recorded in freedrive are unaffected." + note)

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
            if self.grip_lock.locked():
                raise arm.ArmError("A gripper is opening or closing. Wait for it, then jog.")
            self.require_hold()
            busy = self._busy(frame)
            if self.goal is None or not busy:
                self.goal, self.goal_kind = self._synced_goal(kind, frame), kind
            elif self.goal_kind != kind:
                raise arm.ArmError("Let the current move finish before switching between joint and tool keys.")

            goal = list(self.goal)
            sign = "+" if direction > 0 else "\u2212"
            guards = {}
            if kind == "joint":
                goal[index] += direction * math.radians(step)
                lead = max(abs(math.degrees(g - m)) for g, m in zip(goal, frame["joints"]))
                if lead > MAX_LEAD_STEPS * step + 0.5:
                    return False, self._say("Holding: the arm is still catching up with earlier clicks.")
                if self.model is not None:  # the step this click adds, from the goal before it
                    why, guards = self.check_guarded([math.degrees(g) for g in self.goal],
                                                     [math.degrees(g) for g in goal], locked=True)
                    if why:
                        raise arm.ArmError(why)
                sent, limiting = capped_joint_speed(speed, [step if i == index else 0.0 for i in range(self.axis)])
                why = f", J{index + 1}'s top speed" if limiting is not None else ""
                if self.reduced_speed:  # the step from the goal before it, where the controller takes it from
                    cap = self._reduced_joint_cap(self.goal, goal)
                    if cap is not None and cap < sent:
                        sent, why = cap, plannermod.REDUCED_WHY
                ret = self.arm.moveto_joint_p2p(goal, math.radians(sent), math.radians(self._joint_acc(sent)), 0)
                text = f"J{index + 1} {sign}{step:g}\u00b0 \u2192 {math.degrees(goal[index]):.2f}\u00b0"
                if why:
                    text += f" at {sent:g}\u00b0/s{why}"
            else:
                if index < 3:
                    goal[index] += direction * step
                else:
                    goal[index] = wrap(goal[index] + direction * math.radians(rot_step))
                pos_lead = math.dist(goal[:3], frame["pose"][:3])
                rot_lead = max(abs(math.degrees(wrap(g - m))) for g, m in zip(goal[3:], frame["pose"][3:6]))
                if pos_lead > MAX_LEAD_STEPS * step + 1.0 or rot_lead > MAX_LEAD_STEPS * rot_step + 0.5:
                    return False, self._say("Holding: the arm is still catching up with earlier clicks.")
                sent, why = speed, ""
                if self.reduced_speed:  # a slide moves the flange `step` mm; a turn turns it rot_step about itself
                    cap = plannermod.reduced_line_speed(step if index < 3 else 0.0, rot_step if index >= 3 else 0.0,
                                                        self._tool_reach_mm())
                    if cap < sent:
                        sent, why = cap, plannermod.REDUCED_WHY
                if self.model is not None:  # the line this click adds, planned and checked like a program's
                    guards = self._check_tool_step(frame, self.goal, goal, sent, busy)
                    if guards is None:
                        return False, self._say("Holding: the arm is still catching up with earlier clicks.")
                ret = self.arm.moveto_cartesian_line(goal, sent, self._tool_acc(sent), 0)
                axis_name = ("X", "Y", "Z", "Roll", "Pitch", "Yaw")[index]
                value = goal[index] if index < 3 else math.degrees(goal[index])
                unit = " mm" if index < 3 else "\u00b0"
                amount = step if index < 3 else rot_step
                text = f"{axis_name} {sign}{amount:g}{unit} \u2192 {value:.1f}{unit}"
                if why:
                    text += f" at {sent:g} mm/s{why}"
            if ret not in arm.OK_CODES:
                self.goal = None
                raise arm.abort_motion(self.ip, self.arm, f"The controller rejected the move (code {ret}).", STOP_LOCK_WAIT_S)
            self.page_guards = dict(self.page_guards, **guards) if busy else guards
            if guards:  # read just now, holding
                self.page_watch = {"at": time.monotonic()}
            self.goal, self.sent_at = goal, time.monotonic()
            self.heard_at = self.sent_at  # the page is waiting on this very request, so it isn't quiet
        return True, self._say(text)

    def _check_tool_step(self, frame, before, after, speed, busy):
        """Plan a tool step as a straight line of the flange (the controller's own tool point is the flange), from
        the goal before it to the new one, with the cell's tool and zones: refuses a step out of reach, through a
        singularity or into anything (or into a guarded zone whose interlocks don't hold). Returns the guarded zones
        it depends on, or None to hold the click while the arm is still on its way (`busy`) to a goal its joints can't
        be solved for from where it is. Called with self.lock held."""
        p = self.planner()
        q = [math.degrees(v) for v in frame["joints"][:self.axis]]
        start = p.kin.solve(q, kin.from_controller_pose(before), DRAG_LIMITS_DEG)
        if not start["ok"]:
            if busy:
                return None
            raise arm.ArmError(self._unsolved_text(q))
        try:
            plan = self.plan_guarded(lambda planner: planner.line(start["joints"], kin.from_controller_pose(after),
                                                                  kin.IDENTITY, speed, label="the step's end"),
                                     start["joints"], locked=True)
        except plannermod.PlanError as e:
            raise arm.ArmError(str(e))
        return plan.guards

    @staticmethod
    def _unsolved_text(q):
        """Why a tool step from the joints q (deg) can't be checked, and so isn't sent."""
        out = [f"J{i + 1} is at {v:.1f}\u00b0, outside {low:g}\u00b0 to {high:g}\u00b0"
               for i, (v, (low, high)) in enumerate(zip(q, DRAG_LIMITS_DEG)) if not low <= v <= high]
        if out:
            return (f"Not moving: {' and '.join(out)}, where tool steps are planned and checked. Bring it back with "
                    "the joint keys first.")
        return "Not moving: the arm's pose can't be solved from its joints here, so the tool step can't be checked."

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
            if self.grip_lock.locked():
                raise arm.ArmError("A gripper is opening or closing. Wait for it, then press Move.")
            self.require_hold()
            start = self._synced_goal("joint", frame)
            start_deg = [math.degrees(v) for v in start]
            moving = [i for i, angle in enumerate(targets) if angle is not None and abs(angle - start_deg[i]) >= 0.05]
            if not moving:
                return False, self._say("The arm is already in that position.")
            goal_deg = [float(targets[i]) if i in moving else start_deg[i] for i in range(self.axis)]
            # The whole path, not just the plan: never within the margin of the arm itself, the cart, the floor, the
            # tool or the cell's keep-out zones (or, for parts already that close, no closer than they are now), nor of
            # a guarded zone unless its interlocks hold, nor at all from inside a closed one.
            why, guards = self.check_guarded(start_deg, goal_deg, locked=True)
            if why:
                raise arm.ArmError(why)
            # Joints not in the plan stay exactly at their current targets.
            goal = [math.radians(goal_deg[i]) if i in moving else start[i] for i in range(self.axis)]
            sent, limiting = capped_joint_speed(speed, [abs(math.degrees(g - s)) for g, s in zip(goal, start)])
            why = f", J{limiting + 1}'s top speed" if limiting is not None else ""
            if self.reduced_speed:
                cap = self._reduced_joint_cap(start, goal)
                if cap is not None and cap < sent:
                    sent, why = cap, plannermod.REDUCED_WHY
            ret = self.arm.moveto_joint_p2p(goal, math.radians(sent), math.radians(self._joint_acc(sent)), 0)
            if ret not in arm.OK_CODES:
                self.goal = None
                raise arm.abort_motion(self.ip, self.arm, f"The controller rejected the move (code {ret}).", STOP_LOCK_WAIT_S)
            self.goal, self.goal_kind, self.sent_at = goal, "joint", time.monotonic()
            self.page_guards = guards
            self.page_watch = {"at": time.monotonic()}  # any it depends on were read just now, holding
            self.heard_at = self.sent_at  # the page is waiting on this very request, so it isn't quiet
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
            self._wake.wait(0.1)
            self._wake.clear()
            frame, sent_at = self.frame, self.sent_at
            if self.freedrive_since is not None and time.monotonic() - self.freedrive_since > FREEDRIVE_MAX_S:
                try:
                    self.freedrive(False, reenable=False)  # nobody may be there: the motors stay off
                    self._say(f"Freedrive ended on its own after {FREEDRIVE_MAX_S / 60:.0f} minutes; the motors are "
                              "off. Hold Enable to carry on.")
                except arm.ArmError:
                    pass
                self.freedrive_since = None
            if frame is None or not sent_at or self.runner.moving:
                continue  # nothing of the page's under way (a program's moves have their own supervision)
            now = time.monotonic()
            if not (frame["status"] == 1 or frame["queued"] > 0 or now - sent_at < BUSY_AFTER_SEND_S):
                continue
            # Every reason to stop that needs no SDK request comes first: the interlocks are read last, and without
            # waiting long for the SDK (watch_guards), so nothing here is held up behind a request in flight.
            lapsed = self.hold_lapsed() if frame["mode"] != 2 else None  # freedrive is the hand's
            if lapsed:
                self.stop(lapsed)
            elif now - self.heard_at > PAGE_SILENT_S:
                self.stop("the jog page went quiet")
            elif now - self.frame_at > WATCHDOG_STALE_S:
                self.stop("the controller's status went quiet")
            elif frame["err"]:
                self.stop(f"the controller reported error {arm.error_text(frame['err'])}")
            elif self.page_guards and frame["mode"] != 2:
                dropped = self.watch_guards(self.page_guards, self.page_watch)
                if dropped:
                    self.page_guards = {}
                    self.stop(f"{dropped[1]}, during a move into the {dropped[0]} zone")


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
    check against (with the active tool's shape and the zones, guarded zones marked with their interlocks), the
    patterns' geometry and their counters."""
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
    patterns = {}
    for name in data["patterns"]:
        try:
            g = cell.pattern_geometry(name)
            patterns[name] = {"count": g["count"], "frame": g["frame"], "tool": g["tool"], "pitchMm": g["pitchMm"],
                              "angleDeg": g["angleDeg"]}
        except cellmod.CellError as e:
            patterns[name] = {"error": str(e)}
    out = {"ok": True, "cell": data, "framesInBase": frames, "targetsInBase": targets, "patterns": patterns,
           "counters": panel.counters.snapshot(),
           "pending": {"frames": panel.pending_points, "tools": {k: len(v) for k, v in panel.pending_poses.items()}}}
    scene = {"zones": [], "frames": [], "targets": [], "slots": []}  # what the 3D view draws (metres, base frame)
    for o in cell.zone_obstacles():
        scene["zones"].append({"name": o.zone, "centre": o.centre, "axes": o.axes, "half": o.half,
                               "interlocks": data["zones"][o.zone].get("interlocks", [])})
    for name in data["frames"]:
        t = cell.frame_matrix(name)
        scene["frames"].append({"name": name, "origin": [t[i][3] for i in range(3)], "x": [t[i][0] for i in range(3)],
                                "y": [t[i][1] for i in range(3)]})
    for name, pose in targets.items():
        if pose is not None:
            entry = {"name": name, "p": [v / 1000.0 for v in pose["xyz"]]}
            previous = data["targets"][name].get("previous")
            if previous and "pose" in previous and previous.get("frame") in ([cellmod.BASE] + list(data["frames"])):
                was = kin.mul(cell.frame_matrix(previous["frame"]), kin.pose_to_matrix(previous["pose"]))
                entry["was"] = [v / 1000.0 for v in kin.position(was)]  # where it was before its touch-up
            scene["targets"].append(entry)
    for name, info in patterns.items():
        if "error" in info or info["count"] > 400:  # a big pattern's slots would bury the view
            continue
        for slot in range(1, info["count"] + 1):
            p = kin.position(cell.slot_matrix(name, slot))
            scene["slots"].append({"pattern": name, "slot": slot, "p": [v / 1000.0 for v in p]})
    out["scene"] = scene
    if panel.model is not None:
        model = panel.cell_model()
        config = model.collision_config()
        guarded = cell.guarded_zones()
        for entry, o in zip(config["obstacles"], model.obstacles):
            zone = getattr(o, "zone", None)
            if zone in guarded:
                entry["zone"], entry["guard"] = zone, guarded[zone]
        out["collision"] = config
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
        io = panel.io_state()
        words = {line: io[line]["inputs"] for line in io if io[line]["inputs"] is not None}
        interlocks = panel.interlock_states(words=words) if len(words) == len(io) else {}
        return {"ok": True, "io": io, "interlocks": interlocks, "grippers": panel.gripper_states(io)}
    if path == "/api/calls":
        with panel.calls_lock:
            return {"ok": True, "calls": [dict(c) for c in panel.calls]}
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
        was = cell.data["active"]["tool"]
        cell.activate_tool(body.get("name"), rev)
        if cell.data["active"]["tool"] != was:
            panel.held.clear()  # another tool on the flange: whatever the last one's grippers held isn't carried now
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
    if path == "/api/cell/target/touchup":
        return touch_up_target(panel, body)
    if path == "/api/cell/target/undo":
        entry = cell.undo_touch_up(body.get("name"), rev)
        return dict(cell_view(panel), target=entry, message=f"Target {body.get('name')} is back as it was before its "
                                                            "touch-up (the change is in cell/logs/targets.jsonl).")
    if path == "/api/cell/target/delete":
        users = runnermod.programs_using(panel.programs, body.get("name"))
        if users:
            raise cellmod.CellError(f"Target {body.get('name')!r} is used by {', '.join(users)}; take it out of "
                                    f"{'them' if len(users) > 1 else 'it'} first.")
        cell.delete_target(body.get("name"), rev)
        return dict(cell_view(panel), message=f"Target {body.get('name')} deleted.")
    if path == "/api/cell/zone":
        cell.set_zone(body.get("name"), body.get("frame", cellmod.BASE), body.get("pose"), body.get("size"), rev,
                      body.get("interlocks"))
        guards = cell.data["zones"][body.get("name")].get("interlocks")
        return dict(cell_view(panel), message=f"Zone {body.get('name')} saved"
                    + (f": the arm may enter it while {' and '.join(guards)} {'holds' if len(guards) == 1 else 'hold'}."
                       if guards else "."))
    if path == "/api/cell/zone/delete":
        cell.delete_zone(body.get("name"), rev)
        return dict(cell_view(panel), message=f"Zone {body.get('name')} deleted.")
    if path == "/api/cell/interlock":
        cell.set_interlock(body.get("name"), body.get("input"), body.get("value", 1), rev)
        return dict(cell_view(panel), message=f"Interlock {body.get('name')} saved: it holds while {body.get('input')} "
                                              f"reads {body.get('value', 1)}.")
    if path == "/api/cell/interlock/delete":
        cell.delete_interlock(body.get("name"), rev)
        return dict(cell_view(panel), message=f"Interlock {body.get('name')} deleted.")
    if path == "/api/cell/gripper":
        _idle_for_setup(panel)
        cell.set_gripper(body.get("name"), body.get("close"), body.get("open"),
                         body.get("seconds", cellmod.GRIPPER_SECONDS), body.get("timeout", cellmod.GRIPPER_TIMEOUT_S),
                         body.get("closed"), body.get("opened"), body.get("part"), rev)
        return dict(cell_view(panel), message=f"Gripper {body.get('name')} saved.")
    if path == "/api/cell/gripper/delete":
        users = runnermod.programs_using(panel.programs, body.get("name"), "gripper")
        if users:
            raise cellmod.CellError(f"Gripper {body.get('name')!r} is used by {', '.join(users)}; take it out of "
                                    f"{'them' if len(users) > 1 else 'it'} first.")
        cell.delete_gripper(body.get("name"), rev)
        panel.held.pop(body.get("name"), None)
        return dict(cell_view(panel), message=f"Gripper {body.get('name')} deleted.")
    if path == "/api/cell/pattern":
        g = cell.set_pattern(body.get("name"), body.get("rows"), body.get("columns"), body.get("layers", 1),
                             body.get("layerPitch", 0.0), body.get("first"), body.get("rowEnd"), body.get("columnEnd"), rev)
        return dict(cell_view(panel), message=f"Pattern {body.get('name')} saved: {pattern_text(g)}.")
    if path == "/api/cell/pattern/delete":
        users = runnermod.programs_using(panel.programs, body.get("name"), "pattern")
        if users:
            raise cellmod.CellError(f"Pattern {body.get('name')!r} is used by {', '.join(users)}; take it out of "
                                    f"{'them' if len(users) > 1 else 'it'} first.")
        cell.delete_pattern(body.get("name"), rev)
        panel.counters.drop(body.get("name"))
        return dict(cell_view(panel), message=f"Pattern {body.get('name')} deleted.")
    if path == "/api/cell/pattern/counter":
        _idle_for_setup(panel)
        name = body.get("name")
        count = cell.pattern_geometry(name)["count"]
        slot = body.get("slot", 1)
        if isinstance(slot, bool) or not isinstance(slot, int) or not 1 <= slot <= count + 1:
            raise cellmod.CellError(f"{name}'s counter goes from slot 1 to {count}, or {count + 1} for used up; not "
                                    f"{slot!r}.")
        panel.counters.set(name, slot)
        return dict(cell_view(panel), message=f"{name}'s counter is at slot {slot} of {count}." if slot <= count
                    else f"{name}'s counter is past its last slot: used up.")
    if path == "/api/program":
        saved = panel.programs.save(body.get("program"), cell, rev)
        return {"ok": True, "program": saved, "message": f"Program {saved['name']} saved."}
    if path == "/api/program/delete":
        panel.programs.delete(body.get("name"))
        return {"ok": True, "message": f"Program {body.get('name')} deleted."}
    if path == "/api/run/start":
        if body.get("mode", "run") != "dry":  # a dry run only plans
            panel.require_hold()
        runner.start(body.get("name"), body.get("mode", "run"), body.get("from"))
        return {"ok": True, "run": runner.status(), "message": f"Running {body.get('name')}"
                + {"run": ".", "step": ", one step at a time.", "dry": " dry: planning only, nothing moves."}[
                    body.get("mode", "run")]}
    if path in ("/api/run/pause", "/api/run/resume", "/api/run/step", "/api/run/reset"):
        if path in ("/api/run/resume", "/api/run/step") and runner.state == "paused" and runner.mode != "dry":
            panel.require_hold()
        getattr(runner, path.rsplit("/", 1)[1])()
        return {"ok": True, "run": runner.status()}
    if path == "/api/run/stop":
        stopped = runner.stop("Stop run was pressed")
        return {"ok": True, "run": runner.status(), "message": "Stopping the run." if stopped else "No run to stop."}
    if path == "/api/run/override":
        runner.set_override(body.get("percent"))
        return {"ok": True, "run": runner.status(), "message": f"Speed override {runner.override}%."}
    if path in ("/api/moveto", "/api/plan"):
        step = {"type": "move", "kind": body.get("kind", "joint")}
        if body.get("pattern") is not None:
            step["pattern"] = body["pattern"]
            if body.get("slot") is not None:
                step["slot"] = body["slot"]
        else:
            step["target"] = body.get("target")
        if body.get("offset"):
            step["offset"] = body["offset"]
        if body.get("speed") is not None:
            step["speed"] = body["speed"]
        program = {"name": "manual", "steps": [step], "speed": body.get("defaults") or {}}
        if path == "/api/plan":
            checked = runnermod.check_program(program, cell)
            s = checked["steps"][0]
            if s.get("pattern") is not None and s.get("slot") is None:  # the slot its counter stands at
                count = cell.pattern_geometry(s["pattern"])["count"]
                s["slot"] = panel.counters.slot(s["pattern"])
                if s["slot"] > count:
                    raise cellmod.CellError(f"{s['pattern']} is used up: all {count} of its slots are done.")
            speed = s.get("speed", checked["speed"]["tcp" if s["kind"] == "line" else "joint"])
            plan = panel.plan_move(s, panel.current_joints(), speed, checked["speed"]["ori"])
            return {"ok": True, "plan": plan.summary(), "message": plan.text}
        panel.require_hold()
        runner.start(program=program, manual=True)
        to = body.get("target") if body.get("pattern") is None else \
            f"{body['pattern']} slot {body['slot']}" if body.get("slot") is not None else f"{body['pattern']}'s next slot"
        return {"ok": True, "run": runner.status(), "message": f"Moving to {to}."}
    if path == "/api/home":
        step = {"type": "home", "target": body.get("target"), "retract": body.get("retract", runnermod.HOME_RETRACT_MM)}
        if body.get("speed") is not None:
            step["speed"] = body["speed"]
        panel.require_hold()
        runner.start(program={"name": "manual", "steps": [step], "speed": body.get("defaults") or {}}, manual=True)
        return {"ok": True, "run": runner.status(), "message": f"Safe home to {body.get('target')}: backing off along "
                                                              "the tool, then a joint move."}
    if path == "/api/gripper":
        action = body.get("action")
        if action not in runnermod.GRIPPER_ACTIONS:
            raise runnermod.ProgramError(f"A gripper opens, closes or grasps, not {action!r}.")
        part = runnermod.check_part(body["part"]) if body.get("part") is not None and action == "grasp" else None
        # One gripper action from the page at a time, and none while the arm moves, as the payload may change: while it
        # works, no run starts (Runner.start), nor a jog, Move or freedrive.
        if not panel.grip_lock.acquire(blocking=False):
            raise arm.ArmError("A gripper is already opening or closing. Wait for it.")
        try:
            with runner.lock:
                if runner.busy:
                    raise arm.ArmError("A program is under way; its gripper is its own.")
            with panel.lock:
                if panel._page_move_under_way():
                    raise arm.ArmError("The arm is moving. Wait for it to stop: the gripper may change the payload.")
            message = panel.grip(body.get("name"), action, part)
        finally:
            panel.grip_lock.release()
        return {"ok": True, "held": panel.held, "message": message}
    if path == "/api/calls/ack":
        marked = panel.calls_seen(body.get("id"))
        with panel.calls_lock:
            unseen = [dict(c) for c in panel.calls if not c["seen"]]
        return {"ok": True, "calls": unseen, "message": f"{marked} call{'s' if marked != 1 else ''} marked as seen."}
    if path == "/api/io/set":
        _idle_for_setup(panel)
        if runner.busy:
            raise arm.ArmError("A program is under way; its outputs are its own.")
        panel.write_output(body.get("output"), 1 if body.get("value") else 0)
        return {"ok": True, "io": panel.io_state(), "message": f"{body.get('output')} set to {1 if body.get('value') else 0}."}
    if path == "/api/freedrive":
        return {"ok": True, "message": panel.freedrive(bool(body.get("on")), body.get("sensitivity"))}
    if path == "/api/manual":
        message = panel.set_manual(body)
        return {"ok": True, "manual": panel.manual_status(), "message": message}
    if path == "/api/hold":
        manual = panel.hold(body.get("ms", HOLD_DEFAULT_MS))
        return {"ok": True, "manual": manual, "message": f"Held for {manual['holdLeftMs']} ms." if manual["held"]
                else "Let go."}
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
        if at:  # insert after this step (a path of 0-based indexes, as runs number them)
            container, index = runnermod.step_list(steps, at)
            container.insert(index + 1, step)
        else:
            steps.append(step)
        saved = panel.programs.save(program, cell, program.get("revision"))
        message += f" Added a {'straight' if move == 'line' else 'joint'} move to it in {saved['name']}."
    return dict(cell_view(panel), message=message, target=entry)


def pattern_text(g):
    """A pattern's geometry in words: '12 slots, columns 40.0 mm apart, rows 35.0 mm apart, at 90.0°'."""
    bits = [f"{g['count']} slot{'s' if g['count'] != 1 else ''} in {g['frame']}"]
    along, down, up = g["pitchMm"]
    if along:
        bits.append(f"columns {along:.1f} mm apart")
    if down:
        bits.append(f"rows {down:.1f} mm apart")
    if g["angleDeg"] is not None:
        bits.append(f"at {g['angleDeg']:.1f}\u00b0 to each other")
    if up:
        bits.append(f"layers {up:.1f} mm apart")
    return ", ".join(bits)


def touch_up_target(panel, body):
    """Re-teach an existing target in place, where the arm is now: a tool target keeps its frame and tool (which must
    be the tool in use), a joint target stays one. The reply says how far it moved; the old value stays in the target,
    for Undo, and in cell/logs/targets.jsonl."""
    cell, name, rev = panel.cell, body.get("name"), body.get("revision")
    if panel.runner.moving:
        raise arm.ArmError("A program is running. Pause or stop it first.")
    target = cell.target(name)
    q = panel.current_joints()
    taught = {"at": time.strftime("%Y-%m-%dT%H:%M:%S"), "how": "freedrive" if panel.frame["mode"] == 2 else "here",
              "joints": [round(v, 4) for v in q]}
    if "joints" in target:
        entry, change = cell.touch_up(name, joints=[round(v, 4) for v in q], revision=rev, taught=taught)
        text = f"Target {name} touched up: its joints changed by up to {change['jointsMaxDeg']:.2f}\u00b0"
    else:
        active = cell.data["active"]["tool"]
        if target["tool"] != active:
            raise cellmod.CellError(f"{name!r} was taught with the tool {target['tool']!r}, not {active!r}: make it the "
                                    "tool in use (Use) to touch the target up.")
        pose = kin.mul(kin.invert(cell.frame_matrix(target["frame"])), panel.tool_point(q))
        entry, change = cell.touch_up(name, pose=kin.matrix_to_pose(pose, 4), seed=[round(v, 4) for v in q],
                                      revision=rev, taught=taught)
        d = change["deltaMm"]
        text = (f"Target {name} touched up: moved {change['movedMm']:.2f} mm ({d[0]:+.2f}, {d[1]:+.2f}, {d[2]:+.2f} "
                f"along {target['frame']}'s x, y, z) and turned {change['turnedDeg']:.2f}\u00b0")
        if "jointsMaxDeg" in change:
            text += f"; its joints by up to {change['jointsMaxDeg']:.2f}\u00b0"
    users = runnermod.programs_using(panel.programs, name) + cell.patterns_using(name)
    if users:
        text += f". Every move to it changes with it ({', '.join(users)})"
    text += ". Undo puts the old value back; it's also in cell/logs/targets.jsonl."
    return dict(cell_view(panel), message=text, target=entry, change=change)


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description="Serve a jog panel for the UTRA arm at http://127.0.0.1:PORT.")
    parser.add_argument("ip", help="controller address, e.g. 192.168.2.1")
    parser.add_argument("--port", type=int, default=8765, help="local port (default 8765)")
    parser.add_argument("--cell", default=CELL_DIR, help="folder for the cell, programs and run logs (default cell/)")
    parser.add_argument("--brain", default=brain_link.DEFAULT_URL,
                        help=f"the brain's address, on this PC (default {brain_link.DEFAULT_URL}), or 'off'")
    parser.add_argument("--brain-timeout", type=float, default=brain_link.PLAN_TIMEOUT_S,
                        help=f"how long the brain may take to plan a path, s (default {brain_link.PLAN_TIMEOUT_S:g})")
    parser.add_argument("--reduced-speed", action="store_true",
                        help="start with reduced speed on: every move keeps the tool point and flange under "
                             f"{TOOL_REDLINE_MM_S:g} mm/s (POST /api/manual changes it)")
    parser.add_argument("--hold-to-move", action="store_true",
                        help="start with hold-to-move on: nothing moves without a current hold (POST /api/hold)")
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be between 1024 and 65535")
    if not 0.5 <= args.brain_timeout <= 120:
        parser.error("--brain-timeout must be between 0.5 and 120 s")
    try:
        token = panel_token()
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    link = None
    if args.brain != "off":
        try:
            link = brain_link.BrainLink(args.brain, args.brain_timeout)
        except ValueError as e:
            parser.error(f"--brain: {e}")

    try:
        model = utra_model.ArmModel()
    except Exception as e:  # the 3D view and planned moves are optional; jogging works without them
        print(f"Warning: couldn't load the arm model ({e}). The 3D view and planned moves are off.", file=sys.stderr)
        model = None
    try:
        panel = Panel(args.ip, model, args.cell, link, args.reduced_speed, args.hold_to_move)
    except (arm.ArmError, cellmod.CellError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    if model is not None and len(model.joints) != panel.axis:
        print(f"Warning: the arm model has {len(model.joints)} joints but the arm has {panel.axis}. "
              "The 3D view and planned moves are off.", file=sys.stderr)
        panel.model = None
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
        server = loopback_http.Server(("127.0.0.1", args.port),
                                      make_handler(panel, token, page.encode("utf-8"), args.port, three))
    except OSError as e:
        print(f"Error: cannot listen on 127.0.0.1:{args.port} ({e}). Try --port.", file=sys.stderr)
        return 1
    server.daemon_threads = True
    print(f"Jog panel for {args.ip}: open http://127.0.0.1:{args.port}  (Ctrl+C to quit)")
    print(f"Planned and learned steps ask the brain at {link.url}." if link else "The brain is off (--brain off).")
    if panel.reduced_speed or panel.hold_to_move:
        print(panel.manual_text())
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
