#!/usr/bin/env python3
"""Command-line helper for the Umbratek UTRA arm and its NTRO controller.

Built on the Umbratek Python SDK in ../ut_sdk_python (a git submodule; run
`git submodule update --init` once after cloning).

Read-only:
    python scripts/arm.py find                  # look for the arm on this PC's networks
    python scripts/arm.py status IP             # identity, state, errors, joints, TCP pose
    python scripts/arm.py watch IP              # live 10 Hz status report (Ctrl+C to quit)

Changes the arm's state (motion commands ask for confirmation unless --yes):
    python scripts/arm.py reset IP              # clear a latched error, e.g. after releasing an E-stop
    python scripts/arm.py stop IP               # stop motion and clear queued commands
    python scripts/arm.py jog IP -j 6 -d 5      # rotate joint 6 by +5 degrees
    python scripts/arm.py home IP               # move to the controller's home pose

Keep the hardware E-stop within reach whenever the arm can move.
"""
from __future__ import annotations

import argparse
import asyncio
import ipaddress
import math
import os
import signal
import socket
import struct
import sys
import threading
import time

SDK_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ut_sdk_python", "utapi"))

CMD_PORT = 502  # UTRC command/response port used by UtraApiTcp
REPORT_PORT = 30001  # 10 Hz status report (same stream as UtraReportStatus10Hz)
REG_MOTION_STATUS = 0x25  # register numbers from ut_sdk_python/utapi/base/arm_reg.py
REG_CMD_NUM = 0x26

# SDK return codes: 0 is success. -4 means the reply carried the controller's "state"
# flag (an error or warning is active); the returned data is still valid.
OK_CODES = (0, -4)

MOTION_MODES = {0: "position control", 1: "servo", 2: "joint teach", 3: "cartesian teach"}
MOTION_STATES = {0: "ready", 1: "moving", 2: "ready (idle)", 3: "paused", 4: "stopped"}
IDLE_STATES = (0, 2, 4)
READY_STATES = (0, 2)

MAX_SPEED_DEG_S = 60.0
MAX_ACC_DEG_S2 = 300.0
MAX_STEP_LIMIT_DEG = 90.0
MIN_JOG_DEG = 0.1
MAX_JOINT_MISMATCH_DEG = 1.0  # allowed gap between the controller's target and the measured angle
ARRIVAL_TOLERANCE_DEG = 0.5  # jogs use the smaller of this and half the requested step
CGNAT = ipaddress.ip_network("100.64.0.0/10")  # e.g. Tailscale; never the arm

POWER_HINT = ("The arm's joints aren't answering, which usually means the arm has no power: release the front "
              "E-stop, make sure the rear Expansion E-STOP socket has its switch or shorting plug, and check that "
              "the arm cable is seated in Robot Connector (the Robot Power Indicator beside it should be lit).")
ESTOP_HINT = ("An emergency stop is active (error 1). Release the front E-stop and any external E-stop, and make sure "
              "the rear Expansion E-STOP socket has its switch or shorting plug. The error stays latched until a reset: "
              "python scripts/arm.py reset {ip}")

# Error and warning names from the error table in the controller's own Studio web app.
ERROR_NAMES = {0: "NONE", 1: "ESTOP_SWITCH", 2: "DC12_SWITCH", 10: "Servo error", 21: "Inverse kinematics error",
               22: "Mechanical collision limit", 23: "Mechanical angle limit", 24: "Motor_proc angle speed limit",
               25: "Speed planning error", 26: "Rtlinux set timing error", 27: "Reply command failed",
               28: "gripper error", 29: "OTHER", 30: "tau max limit", 31: "tau collision limit", 32: "CLA_CIRCLE",
               33: "GPIO", 51: "TGPIO_UTRC_ERROR", 52: "CGPIO_UTRC_ERROR"}
WARNING_NAMES = {0: "NONE", 11: "Uxbus que is full", 12: "Parameter error", 13: "The instruction does not exist",
                 14: "command has no solution"}


def error_text(code):
    return f"{code} ({ERROR_NAMES.get(code, 'unknown')})" if code else "0"


def warning_text(code):
    return f"{code} ({WARNING_NAMES.get(code, 'unknown')})" if code else "0"


class ArmError(Exception):
    stopped = False  # set once abort_motion has sent (or tried to send) a stop for this failure


def load_sdk():
    if not os.path.isfile(os.path.join(SDK_DIR, "utra", "utra_api_tcp.py")):
        raise ArmError("The Umbratek SDK submodule is missing. From the repo root run:\n"
                       "    git submodule update --init")
    if SDK_DIR not in sys.path:
        sys.path.insert(0, SDK_DIR)


def clean(text):
    return text.replace("\x00", " ").strip()


def version_text(raw):
    """Versions are two 10-character fields, e.g. '739c89bb82' and '2021-12-18'."""
    return " / ".join(p for p in (clean(raw[:10]), clean(raw[10:20])) if p)


def deg_list(radians):
    return " ".join(f"{math.degrees(r):8.2f}" for r in radians)


def all_finite(values):
    return all(math.isfinite(v) for v in values)


def full_mask(axis):
    return (1 << axis) - 1


def worst_diff(a, b):
    """(joint number, degrees) for the joint where a and b differ most; non-finite values count as infinite."""
    diffs = [abs(math.degrees(x - y)) if math.isfinite(x) and math.isfinite(y) else math.inf for x, y in zip(a, b)]
    i = max(range(len(diffs)), key=diffs.__getitem__)
    return i + 1, diffs[i]


############################################################
#                  Raw UTRC and stopping
############################################################

def utrc_request(sock, rw, cmd, payload=b"", timeout=2.0):
    """Send one UTRC frame (the SDK's wire format) and return the reply's (state flag, data)."""
    from common import crc16

    opcode = ((rw & 1) << 7) | (cmd & 0x7F)
    frame = bytes([0xAA, 0x55, (len(payload) + 1) & 0x7F, opcode]) + bytes(payload)
    sock.sendall(frame + crc16.crc_modbus(frame))
    buf = b""
    deadline = time.monotonic() + timeout
    while len(buf) < 3 or len(buf) < (buf[2] & 0x7F) + 5:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise socket.timeout("no reply")
        sock.settimeout(remaining)
        chunk = sock.recv(512)
        if not chunk:
            raise ConnectionError("connection closed")
        buf += chunk
    reply = buf[:(buf[2] & 0x7F) + 5]
    if (reply[0], reply[1]) != (0x55, 0xAA) or reply[3] != opcode or reply[-2:] != crc16.crc_modbus(reply[:-2]):
        raise ValueError("not a UTRC reply")
    return reply[2] >> 7, reply[4:-2]


def _stop_once(ip):
    try:
        with socket.create_connection((ip, CMD_PORT), timeout=1.0) as sock:
            utrc_request(sock, 1, REG_MOTION_STATUS, b"\x04", timeout=1.0)
            for _ in range(10):
                _, state = utrc_request(sock, 0, REG_MOTION_STATUS, timeout=1.0)
                _, queued = utrc_request(sock, 0, REG_CMD_NUM, timeout=1.0)
                if state[:1] == b"\x04" and queued == bytes(4):
                    return True
                time.sleep(0.1)
    except (OSError, ValueError):
        time.sleep(0.2)
    return False


def _stop_via(arm, deadline, lock_wait_s):
    """Stop over an open SDK connection. Each SDK request holds arm.mutex until its reply arrives or times out
    (1 s), so a lock still held after lock_wait_s was left by an interrupted request, and is released."""
    if arm.mutex.acquire(timeout=lock_wait_s) if lock_wait_s > 0 else arm.mutex.acquire(blocking=False):
        arm.mutex.release()
    elif arm.mutex.locked():
        try:
            arm.mutex.release()
        except RuntimeError:
            pass
    try:
        if arm.set_motion_status(4) not in OK_CODES:
            return False
        while time.monotonic() < deadline:
            ret, state = arm.get_motion_status()
            ret_q, queued = arm.get_cmd_num()
            if ret in OK_CODES and ret_q in OK_CODES and state == 4 and queued == 0:
                return True
            time.sleep(0.05)
    except Exception:
        pass
    return False


def stop_arm(ip, arm=None, lock_wait_s=0.0):
    """Stop motion and clear queued commands; True once the controller confirms both.

    The controller serves one command connection at a time and drops any other at once, so the stop goes over
    `arm`'s connection when there is one. A fresh connection is the fallback, for when that one has failed (or
    there is none). Pass lock_wait_s > 0 when other threads share `arm`: the stop then waits that long for a
    request in flight before taking over the SDK's lock. On the main thread, Ctrl+C is ignored while stopping.
    """
    load_sdk()
    on_main = threading.current_thread() is threading.main_thread()
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN) if on_main else None
    try:
        start = time.monotonic()
        if arm is not None and _stop_via(arm, start + 2.0 + lock_wait_s, lock_wait_s):
            return True
        deadline = max(start + 2.5, time.monotonic() + 1.0)
        while time.monotonic() < deadline:
            if _stop_once(ip):
                return True
        return False
    finally:
        if on_main:
            signal.signal(signal.SIGINT, previous)


def abort_motion(ip, arm, reason, lock_wait_s=0.0):
    """Stop the arm after a failed or interrupted move and return the error to raise."""
    print("Stopping the arm...", file=sys.stderr, flush=True)
    if stop_arm(ip, arm, lock_wait_s):
        err = ArmError(f"{reason} Sent stop; the controller confirmed the arm is stopped.")
    else:
        err = ArmError(f"{reason} COULD NOT CONFIRM THE STOP: press the hardware E-stop now.")
    err.stopped = True
    return err


def guarded(ip, arm, action, lock_wait_s=0.0):
    """Run the state-changing part of a command; any failure or Ctrl+C ends with a confirmed stop."""
    try:
        return action()
    except KeyboardInterrupt:
        raise abort_motion(ip, arm, "Interrupted.", lock_wait_s)
    except ArmError as e:
        if e.stopped:
            raise
        raise abort_motion(ip, arm, str(e), lock_wait_s)
    except Exception as e:
        raise abort_motion(ip, arm, f"Unexpected error ({e!r}).", lock_wait_s)


############################################################
#                       Discovery
############################################################

def identify(ip, timeout):
    with socket.create_connection((ip, CMD_PORT), timeout=timeout) as sock:
        _, uuid = utrc_request(sock, 0, 0x01, timeout=timeout)
        _, version = utrc_request(sock, 0, 0x02, timeout=timeout)
        _, axis = utrc_request(sock, 0, 0x04, timeout=timeout)
    return {
        "uuid": clean(uuid.decode("ascii", "replace")),
        "version": version_text(version.decode("ascii", "replace")),
        "axis": axis[0] if axis else "?",
    }


def local_networks():
    """Map each local /24 network to this PC's addresses in it."""
    ips = set()
    try:
        ips.update(ai[4][0] for ai in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET))
    except OSError:
        pass
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.254.254.254", 1))  # UDP connect sends nothing; reveals the primary IP
            ips.add(s.getsockname()[0])
    except OSError:
        pass
    networks = {}
    for ip in ips:
        addr = ipaddress.ip_address(ip)
        if addr.is_loopback or addr.is_link_local or addr.is_unspecified or addr in CGNAT:
            continue
        networks.setdefault(ipaddress.ip_network(f"{ip}/24", strict=False), []).append(addr)
    return networks


async def _probe(host, port, timeout, limit):
    async with limit:
        try:
            _, writer = await asyncio.wait_for(asyncio.open_connection(host, port), timeout)
        except (OSError, asyncio.TimeoutError):
            return None
        writer.close()
        try:
            await writer.wait_closed()
        except OSError:
            pass
        return host


async def _scan(hosts, port, timeout):
    limit = asyncio.Semaphore(256)
    found = await asyncio.gather(*(_probe(h, port, timeout, limit) for h in hosts))
    return [h for h in found if h]


def cmd_find(args):
    load_sdk()
    if not (math.isfinite(args.timeout) and 0 < args.timeout <= 10):
        raise ArmError("--timeout must be between 0 and 10 seconds.")
    if args.subnet:
        try:
            networks = {ipaddress.ip_network(s, strict=False): [] for s in args.subnet}
        except ValueError as e:
            raise ArmError(f"Bad --subnet: {e}")
    else:
        networks = local_networks()
    if not networks:
        raise ArmError("This PC has no usable IPv4 network. Connect to the controller first (see docs/connecting.md).")

    hosts = []
    for net, mine in sorted(networks.items(), key=lambda kv: kv[0]):
        if net.num_addresses > 4096:
            raise ArmError(f"{net} is too large to scan; use a /20 or smaller.")
        if len(mine) > 1:
            print(f"Warning: this PC has several addresses in {net} ({', '.join(map(str, sorted(mine)))}). "
                  "If one is a direct link to the arm, traffic may leave through the wrong adapter "
                  "(see docs/troubleshooting.md).")
        hosts += [str(h) for h in (net if net.prefixlen >= 31 else net.hosts())]

    print(f"Scanning {', '.join(str(n) for n in sorted(networks))} for TCP port {CMD_PORT} ...")
    open_hosts = asyncio.run(_scan(hosts, CMD_PORT, args.timeout))

    arms = []
    for host in sorted(open_hosts, key=ipaddress.ip_address):
        try:
            info = identify(host, timeout=max(args.timeout, 1.5))
        except (OSError, ValueError) as e:
            print(f"  {host}: port {CMD_PORT} is open but it did not answer like an NTRO controller ({e}).")
            continue
        arms.append(host)
        print(f"\nFound a UTRA controller at {host}")
        print(f"  UUID     : {info['uuid']}")
        print(f"  Software : {info['version']}")
        print(f"  Axes     : {info['axis']}")
        print(f"  Studio   : http://{host}  (Umbratek Studio web UI)")

    if arms:
        print(f"\nNext: python scripts/arm.py status {arms[0]}")
        return 0
    print("\nNo arm found. Check that:\n"
          "  - the controller is on: press its power button and wait ~25 s for the LED to\n"
          "    'breathe' slowly (a fast 0.3 s blink means an error: check the E-stop and cables);\n"
          "  - Wi-Fi: this PC is joined to the controller's NTRO_xxx network (default password 12345678);\n"
          "  - wired: an Ethernet cable runs from the controller's RJ45 port to this PC and that\n"
          "    adapter is set to obtain an IP address automatically (DHCP);\n"
          "  - or try the IP printed on the controller's label: python scripts/arm.py status <ip>")
    return 1


############################################################
#                    Connection helpers
############################################################

def open_arm(ip):
    load_sdk()
    # The SDK's connect() blocks for ~20 s when the host is absent, so check reachability first.
    try:
        socket.create_connection((ip, CMD_PORT), timeout=3).close()
    except OSError as e:
        raise ArmError(f"Cannot reach {ip}:{CMD_PORT} ({e}). Is the controller on and connected? "
                       "Try: python scripts/arm.py find")
    from utra.utra_api_tcp import UtraApiTcp

    arm = UtraApiTcp(ip)
    if arm.socket_fp.is_error() != 0 or arm.is_err():
        raise ArmError(f"Connected to {ip}:{CMD_PORT}, but it did not respond like a UTRA controller. It serves "
                       "one command connection at a time: if the jog panel or another script is connected, "
                       "close it first.")
    return arm


class StatusReport:
    """Reader for the controller's 10 Hz status stream on port 30001.

    Frame (little-endian): u16 frame length, u8 axis, u8 motion status, u8 motion mode,
    u32 brake mask, u32 enable mask, u8 error, u8 warning, u16 queued commands, then
    axis x f32 measured joints [rad], 6 x f32 TCP pose [mm, rad], axis x f32 joint torques.
    """

    HEADER = struct.Struct("<HBBBIIBBH")

    def __init__(self, ip, timeout=3.0):
        self.sock = socket.create_connection((ip, REPORT_PORT), timeout=timeout)
        self.sock.settimeout(timeout)
        self.buf = b""

    def _take(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("the controller closed the status connection")
            self.buf += chunk
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def read(self):
        length, axis, status, mode, brake, enable, err, warn, queued = self.HEADER.unpack(self._take(17))
        if not 1 <= axis <= 7 or length != 41 + 8 * axis:
            raise ValueError(f"unexpected status frame (length={length}, axis={axis})")
        values = struct.unpack(f"<{axis}f6f{axis}f", self._take(length - 17))
        return {"axis": axis, "status": status, "mode": mode, "brake": brake, "enable": enable,
                "err": err, "warn": warn, "queued": queued, "joints": list(values[:axis]),
                "pose": list(values[axis:axis + 6]), "torque": list(values[axis + 6:])}

    def close(self):
        self.sock.close()


def read_report(ip, timeout=2.0):
    report = StatusReport(ip, timeout=timeout)
    try:
        return report.read()
    finally:
        report.close()


def measure(ip, axis):
    """Measured joint angles [rad] from the status report; motion refuses to start without them."""
    try:
        frame = read_report(ip)
    except (OSError, ValueError, struct.error) as e:
        raise ArmError(f"Cannot read the measured joint angles from {ip}:{REPORT_PORT} ({e}); not moving.")
    if frame["axis"] != axis or not all_finite(frame["joints"]):
        raise ArmError("The status report returned unusable joint angles; not moving.")
    return frame["joints"]


def target_joints(arm):
    ret, joints = arm.get_joint_target_pos()
    if ret not in OK_CODES:
        raise ArmError(f"Could not read the controller's joint targets (code {ret}).")
    joints = list(joints) if isinstance(joints, list) else [joints]
    if not all_finite(joints):
        raise ArmError("The controller returned invalid joint targets; not moving.")
    return joints


def arm_axis(arm):
    ret, axis = arm.get_axis()
    if ret not in OK_CODES:
        raise ArmError(f"Could not read the number of axes (code {ret}).")
    return axis


############################################################
#                       Read-only
############################################################

def cmd_status(args):
    arm = open_arm(args.ip)
    axis = arm_axis(arm)

    def get(name):
        ret, value = getattr(arm, name)()
        return value if ret in OK_CODES else None

    def row(label, value):
        print(f"  {label:<14}: {'unavailable' if value is None else value}")

    try:
        frame = read_report(args.ip)
    except (OSError, ValueError, struct.error):
        frame = None

    print(f"\nUTRA arm at {args.ip}")
    uuid, sw, hw = get("get_uuid"), get("get_sw_version"), get("get_hw_version")
    row("UUID", uuid and clean(uuid))
    row("Software", sw and version_text(sw))
    row("Hardware", hw and version_text(hw))
    row("Axes", axis)
    mode, state = get("get_motion_mode"), get("get_motion_status")
    row("Motion mode", None if mode is None else f"{mode} ({MOTION_MODES.get(mode, 'unknown')})")
    row("Motion state", None if state is None else f"{state} ({MOTION_STATES.get(state, 'unknown')})")
    enable, brake = get("get_motion_enable"), get("get_brake_enable")
    row("Joints enabled", None if enable is None else f"0x{enable & 0xFFFFFFFF:x}")
    row("Brakes", None if brake is None else f"0x{brake & 0xFFFFFFFF:x}")
    codes = get("get_error_code")
    row("Error/warning", None if codes is None else f"{error_text(codes[0])} / {warning_text(codes[1])}")
    servo, unpowered = "not read (only read while stopped with the motors off)", False
    if state in IDLE_STATES and enable is not None and enable & full_mask(axis) == 0:
        msg = get("get_servo_msg")
        servo = None
        if msg is not None:
            parts = [int(x) for x in msg.split()]
            comm, errors = parts[:axis], parts[axis:2 * axis]
            unpowered = -3 in comm
            servo = f"comm [{' '.join(map(str, comm))}]  errors [{' '.join(map(str, errors))}]"
    row("Servos", servo)
    row("Queued cmds", get("get_cmd_num"))
    row("Joints (deg)", frame and deg_list(frame["joints"]))
    targets = get("get_joint_target_pos")
    row("Targets (deg)", None if targets is None else deg_list(targets if isinstance(targets, list) else [targets]))
    pose = frame["pose"] if frame else get("get_tcp_target_pos")
    if pose is not None:
        pose = (f"x {pose[0]:.1f}  y {pose[1]:.1f}  z {pose[2]:.1f} mm   rx {math.degrees(pose[3]):.1f}  "
                f"ry {math.degrees(pose[4]):.1f}  rz {math.degrees(pose[5]):.1f} deg" + ("" if frame else "  (target)"))
    row("TCP pose", pose)
    if codes is not None and codes[0] == 1:
        print("\n" + ESTOP_HINT.format(ip=args.ip))
    elif unpowered:
        print(f"\n{POWER_HINT} Then run: python scripts/arm.py reset {args.ip}")
    elif codes is not None and codes[0]:
        print(f"\nThe controller reports error {error_text(codes[0])}. Once the cause is fixed, clear it with: "
              f"python scripts/arm.py reset {args.ip}")
    if unpowered:
        print("Until the joints answer, the joint readings above are placeholders.")
    if frame is None:
        print(f"\nNo status report from port {REPORT_PORT}; `jog` and `home` need it.")
    return 0


def cmd_watch(args):
    try:
        report = StatusReport(args.ip)
    except OSError as e:
        raise ArmError(f"Cannot open the status report at {args.ip}:{REPORT_PORT} ({e}).")
    print(f"Status from {args.ip}:{REPORT_PORT} (Ctrl+C to stop)")
    shown, last = 0, 0.0
    try:
        while not args.count or shown < args.count:
            f = report.read()
            if time.monotonic() - last < args.interval:
                continue
            last = time.monotonic()
            shown += 1
            x, y, z = f["pose"][:3]
            print(f"{time.strftime('%H:%M:%S')} {MOTION_STATES.get(f['status'], f['status']):<12} "
                  f"mode={f['mode']} en=0x{f['enable']:x} err={error_text(f['err'])} warn={warning_text(f['warn'])} "
                  f"q={f['queued']} | deg {deg_list(f['joints'])} | tcp mm {x:.1f} {y:.1f} {z:.1f}")
    except KeyboardInterrupt:
        print()
    except (OSError, ValueError, struct.error) as e:
        raise ArmError(f"Status stream failed: {e}")
    finally:
        report.close()
    return 0


############################################################
#                     State-changing
############################################################

def cmd_stop(args):
    if stop_arm(args.ip):
        print("Stopped: the controller confirms the arm is stopped with no queued commands.")
        return 0
    raise ArmError(f"Could not confirm a stop at {args.ip}. If the arm is moving, press the hardware E-stop. "
                   "(The controller serves one command connection at a time: while the jog panel runs, use its STOP.)")


def cmd_reset(args):
    """Clear the controller's latched error. Doesn't enable the motors or move the arm."""
    arm = open_arm(args.ip)
    axis = arm_axis(arm)
    ensure_idle(arm, args.ip)
    ret = arm.reset_err()
    if ret not in OK_CODES:
        raise ArmError(f"The reset command failed (code {ret}).")
    time.sleep(1.5)  # let the controller re-check its E-stop and joints
    ret, codes = arm.get_error_code()
    if ret not in OK_CODES:
        raise ArmError(f"Could not read the error code after the reset (code {ret}).")
    print(f"Reset sent. Error now: {error_text(codes[0])}; warning: {warning_text(codes[1])}.")
    comm = None
    ret_s, state = arm.get_motion_status()
    ret_m, enabled = arm.get_motion_enable()
    if ret_s in OK_CODES and ret_m in OK_CODES and state in IDLE_STATES and enabled & full_mask(axis) == 0:
        ret, msg = arm.get_servo_msg()
        if ret in OK_CODES:
            comm = [int(x) for x in msg.split()][:axis]
    if codes[0] == 1:
        print(ESTOP_HINT.format(ip=args.ip))
        return 1
    if comm is not None and -3 in comm:
        print(f"The joints still aren't answering (comm codes {comm}). {POWER_HINT} If an E-stop was only just "
              "released, restart the controller with its power button and run `reset` again.")
        return 1
    if codes[0]:
        print("The error came back, so its cause is still present. Run `status` for details.")
        return 1
    print(f"No errors{'' if comm is None else ' and every joint answers'}. Next: python scripts/arm.py status {args.ip}")
    return 0


def check_motion_args(args):
    jog = hasattr(args, "deg")
    for name in ["speed", "acc"] + (["deg", "max_step"] if jog else []):
        if not math.isfinite(getattr(args, name)):
            raise ArmError(f"--{name.replace('_', '-')} must be a finite number.")
    if not 0 < args.speed <= MAX_SPEED_DEG_S:
        raise ArmError(f"--speed must be in (0, {MAX_SPEED_DEG_S:g}] deg/s.")
    if not 0 < args.acc <= MAX_ACC_DEG_S2:
        raise ArmError(f"--acc must be in (0, {MAX_ACC_DEG_S2:g}] deg/s^2.")
    if jog:
        if not 0 < args.max_step <= MAX_STEP_LIMIT_DEG:
            raise ArmError(f"--max-step must be in (0, {MAX_STEP_LIMIT_DEG:g}] deg.")
        if abs(args.deg) < MIN_JOG_DEG:
            raise ArmError(f"--deg must be at least {MIN_JOG_DEG:g} deg (or -{MIN_JOG_DEG:g}).")
        if abs(args.deg) > args.max_step:
            raise ArmError(f"--deg {args.deg:g} exceeds the per-jog limit of {args.max_step:g} deg (see --max-step).")


def ensure_idle(arm, ip):
    """Refuse unless the arm is stopped or ready with an empty queue. Fails closed on read errors."""
    ret, state = arm.get_motion_status()
    if ret not in OK_CODES:
        raise ArmError(f"Could not read the motion state (code {ret}); not moving.")
    if state not in IDLE_STATES:
        name = MOTION_STATES.get(state, f"in unknown state {state}")
        raise ArmError(f"The arm is {name}; not moving. Let it finish, or run: python scripts/arm.py stop {ip}")
    ret, queued = arm.get_cmd_num()
    if ret not in OK_CODES:
        raise ArmError(f"Could not read the command queue (code {ret}); not moving.")
    if queued:
        raise ArmError(f"The controller has {queued} queued command(s); not moving. "
                       f"Clear them with: python scripts/arm.py stop {ip}")


def servo_problem(arm, axis):
    """Why the joints can't be driven, or None. The SDK says to call this only with the motors off."""
    ret, msg = arm.get_servo_msg()
    if ret not in OK_CODES:
        return f"could not read the joints' status (code {ret})."
    comm = [int(x) for x in msg.split()][:axis]
    if -3 in comm:
        return f"the controller can't reach the joints (comm codes {comm}). {POWER_HINT}"
    if any(c not in (0, -4) for c in comm):  # -4 is a joint fault, which the reset before moving may clear
        return f"unexpected joint comm codes {comm}."
    return None


def preflight(arm, ip, axis, measured=None):
    """Read-only checks before asking to move; returns the measured joint angles [rad]. `measured`, if given, returns
    them from a status stream the caller already holds, instead of a new connection: the real controller serves only a
    few status connections and doesn't free a closed one at once (2026-10-21)."""
    ensure_idle(arm, ip)
    ret, enabled = arm.get_motion_enable()
    if ret not in OK_CODES:
        raise ArmError(f"Could not read the motor enable state (code {ret}); not moving.")
    if enabled & full_mask(axis) == 0:
        problem = servo_problem(arm, axis)
        if problem:
            ret_e, codes = arm.get_error_code()
            if ret_e in OK_CODES and codes[0] == 1:
                raise ArmError("Not moving: " + ESTOP_HINT.format(ip=ip))
            raise ArmError(f"Not moving: {problem} Then run: python scripts/arm.py reset {ip}")
    measured = measured() if measured else measure(ip, axis)
    joint, diff = worst_diff(target_joints(arm), measured)
    if diff > MAX_JOINT_MISMATCH_DEG:
        raise ArmError(f"Not moving: the controller's target for joint {joint} is {diff:.1f} deg from its measured "
                       "angle, so enabling the motors could make the arm jump. Check the arm in Umbratek Studio.")
    return measured


def confirm(args):
    if args.yes:
        return
    if not sys.stdin.isatty():
        raise ArmError("Refusing to move without confirmation; re-run with --yes.")
    try:
        answer = input("Keep the E-stop within reach. Type 'yes' to move: ")
    except EOFError:
        answer = ""
    if answer.strip().lower() != "yes":
        raise ArmError("Cancelled; no motion commands were sent.")


def prepare_for_motion(arm, axis, reset=True):
    """Umbratek's documented sequence (reset errors, position mode, enable, ready), then verify it took. With `reset`
    False (an Enable the server runs on its own), errors aren't reset: one present stops it here."""
    steps = [("reset errors", arm.reset_err)] if reset else []
    steps += [("select position-control mode", lambda: arm.set_motion_mode(0)),
              ("enable all joints", lambda: arm.set_motion_enable(8, 1)),
              ("set the arm to ready", lambda: arm.set_motion_status(0))]
    for label, step in steps:
        ret = step()
        if ret not in OK_CODES:
            raise ArmError(f"Failed to {label} (code {ret}); not moving.")
    deadline = time.monotonic() + 1.5
    while True:
        ret_e, codes = arm.get_error_code()
        ret_m, enabled = arm.get_motion_enable()
        ret_s, state = arm.get_motion_status()
        if ret_e in OK_CODES and codes[0]:
            raise ArmError(f"The controller still reports error {error_text(codes[0])} after a reset; not moving. "
                           "Check the E-stop and cables, then run `status`." if reset else
                           f"The controller reports error {error_text(codes[0])}; not moving.")
        if (ret_e in OK_CODES and ret_m in OK_CODES and ret_s in OK_CODES
                and enabled & full_mask(axis) == full_mask(axis) and state in READY_STATES):
            return
        if time.monotonic() > deadline:
            raise ArmError("The arm did not become enabled and ready; not moving. Run `status`.")
        time.sleep(0.1)


def verified_targets(arm, ip, axis, confirmed, measured=None):
    """The controller's joint targets after enabling, checked against a fresh measurement and the confirmed pose.
    `measured`, if given, returns the fresh measurement (as for preflight)."""
    targets = target_joints(arm)
    for label, reference in (("measured", measured() if measured else measure(ip, axis)), ("confirmed", confirmed)):
        joint, diff = worst_diff(targets, reference)
        if diff > MAX_JOINT_MISMATCH_DEG:
            raise ArmError(f"Not moving: joint {joint}'s target is {diff:.1f} deg from its {label} angle. "
                           "Check the arm in Umbratek Studio.")
    return targets


def run_motion(arm, ip, send, timeout_s):
    """Send one motion command and follow it on the status report until it finishes.

    Stops the arm (and confirms the stop) on Ctrl+C, a controller error, an early pause or
    stop, a lost report or a timeout. Returns True if the arm was seen moving.
    """
    try:
        report = StatusReport(ip)
    except OSError as e:
        raise ArmError(f"Cannot open the status report at {ip}:{REPORT_PORT} ({e}); not moving.")
    try:
        ret = send()
        if ret not in OK_CODES:
            raise abort_motion(ip, arm, f"The controller rejected the motion command (code {ret}).")
        started, seen_moving = time.monotonic(), False
        while True:
            frame = report.read()
            elapsed = time.monotonic() - started
            if frame["err"]:
                raise abort_motion(ip, arm, f"The controller reported error {error_text(frame['err'])} during the move.")
            if frame["status"] == 1:
                seen_moving = True
            elif frame["status"] in READY_STATES:
                if frame["queued"] == 0 and (seen_moving or elapsed > 2.0):
                    return seen_moving
            elif seen_moving or elapsed > 0.5:  # a frame from just before the command may still say "stopped"
                state = MOTION_STATES.get(frame["status"], f"in state {frame['status']}")
                raise abort_motion(ip, arm, f"The move ended early: the arm is {state}.")
            if elapsed > timeout_s:
                raise abort_motion(ip, arm, f"The move did not finish within {timeout_s:.0f} s.")
    except ArmError:
        raise
    except KeyboardInterrupt:
        raise abort_motion(ip, arm, "Interrupted.")
    except (OSError, ValueError, struct.error) as e:
        raise abort_motion(ip, arm, f"Lost the status report ({e}).")
    except Exception as e:
        raise abort_motion(ip, arm, f"Unexpected error ({e!r}).")
    finally:
        report.close()


def settle(ip, axis, expected, tolerance, timeout=2.0):
    """Measured joints once within `tolerance` degrees of `expected`, else the last reading at the timeout."""
    try:
        report = StatusReport(ip)
        try:
            deadline = time.monotonic() + timeout
            while True:
                frame = report.read()
                if frame["axis"] != axis or not all_finite(frame["joints"]):
                    raise ValueError("unusable joint angles in the status report")
                if worst_diff(frame["joints"], expected)[1] <= tolerance or time.monotonic() > deadline:
                    return frame["joints"]
        finally:
            report.close()
    except (OSError, ValueError, struct.error) as e:
        raise ArmError(f"Could not read the final joint angles ({e}). Run `status`.")


def recheck(arm, ip, axis, shown):
    """Repeat the read-only checks after the prompt and make sure the arm hasn't moved since the plan was shown."""
    joint, diff = worst_diff(preflight(arm, ip, axis), shown)
    if diff > MAX_JOINT_MISMATCH_DEG:
        raise ArmError(f"Not moving: joint {joint} moved {diff:.1f} deg since the plan was shown. Run the command again.")


def cmd_jog(args):
    check_motion_args(args)
    arm = open_arm(args.ip)
    axis = arm_axis(arm)
    if not 1 <= args.joint <= axis:
        raise ArmError(f"--joint must be between 1 and {axis}.")
    idx = args.joint - 1
    start = preflight(arm, args.ip, axis)
    print(f"Current joints (deg): {deg_list(start)}")
    print(f"Plan: joint {args.joint} {math.degrees(start[idx]):.2f} -> {math.degrees(start[idx]) + args.deg:.2f} deg "
          f"at {args.speed:g} deg/s; other joints hold.")
    confirm(args)
    recheck(arm, args.ip, axis, start)
    tolerance = min(ARRIVAL_TOLERANCE_DEG, abs(args.deg) / 2)  # so a discarded jog can't pass as arrived

    def move():
        prepare_for_motion(arm, axis)
        goal = verified_targets(arm, args.ip, axis, start)
        goal[idx] += math.radians(args.deg)
        duration = abs(args.deg) / args.speed + args.speed / args.acc
        run_motion(arm, args.ip,
                   lambda: arm.moveto_joint_p2p(goal, math.radians(args.speed), math.radians(args.acc), 0),
                   timeout_s=3 * duration + 10)
        final = settle(args.ip, axis, goal, tolerance)
        joint, diff = worst_diff(final, goal)
        if diff > tolerance:
            raise ArmError(f"Joint {joint} ended {diff:.2f} deg from its commanded angle; the controller may have "
                           "rejected the move (joint limit?). Run `status`.")
        return final

    final = guarded(args.ip, arm, move)
    print(f"Done. Joints (deg): {deg_list(final)}  (motors stay enabled, holding position)")
    return 0


def cmd_home(args):
    check_motion_args(args)
    arm = open_arm(args.ip)
    axis = arm_axis(arm)
    start = preflight(arm, args.ip, axis)
    print(f"Current joints (deg): {deg_list(start)}")
    print(f"Plan: move every joint to the controller's home pose at up to {args.speed:g} deg/s.\n"
          "This can sweep the whole arm through a large arc: clear its full reach first.")
    confirm(args)
    recheck(arm, args.ip, axis, start)

    def move():
        prepare_for_motion(arm, axis)
        verified_targets(arm, args.ip, axis, start)
        moved = run_motion(arm, args.ip,
                           lambda: arm.moveto_home_p2p(math.radians(args.speed), math.radians(args.acc), 0),
                           timeout_s=360 / args.speed + 30)
        home = target_joints(arm)
        final = settle(args.ip, axis, home, ARRIVAL_TOLERANCE_DEG)
        joint, diff = worst_diff(final, home)
        if diff > ARRIVAL_TOLERANCE_DEG:
            raise ArmError(f"Joint {joint} ended {diff:.2f} deg from the home target. Run `status`.")
        if not moved and worst_diff(final, start)[1] < ARRIVAL_TOLERANCE_DEG:
            raise ArmError("The arm didn't move: either it was already at home or the controller ignored the "
                           "command. Run `status`.")
        return final

    final = guarded(args.ip, arm, move)
    print(f"Done. Joints (deg): {deg_list(final)}  (motors stay enabled, holding position)")
    return 0


############################################################
#                          CLI
############################################################

def build_parser():
    parser = argparse.ArgumentParser(
        description="Find, inspect and carefully move an Umbratek UTRA arm (NTRO controller).",
        epilog="Units: joint angles in degrees, speeds in deg/s. Keep the E-stop within reach.")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("find", help="look for the arm on this PC's local networks (read-only)")
    p.add_argument("--subnet", action="append", metavar="CIDR",
                   help="network to scan instead of the local /24s, e.g. 192.168.2.0/24 (repeatable)")
    p.add_argument("--timeout", type=float, default=0.5, help="per-host connect timeout in seconds")
    p.set_defaults(func=cmd_find)

    p = sub.add_parser("status", help="print identity, state, errors, joints and TCP pose (read-only)")
    p.add_argument("ip")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("watch", help="stream the 10 Hz status report (read-only)")
    p.add_argument("ip")
    p.add_argument("--interval", type=float, default=0.5, help="seconds between printed lines")
    p.add_argument("--count", type=int, default=0, help="stop after N lines (default: until Ctrl+C)")
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("stop", help="stop the current motion and clear queued commands")
    p.add_argument("ip")
    p.set_defaults(func=cmd_stop)

    p = sub.add_parser("reset", help="clear a latched controller error, e.g. after releasing an E-stop (no motion)")
    p.add_argument("ip")
    p.set_defaults(func=cmd_reset)

    def motion_args(p, speed):
        p.add_argument("--speed", type=float, default=speed, help=f"deg/s (default {speed:g}, max {MAX_SPEED_DEG_S:g})")
        p.add_argument("--acc", type=float, default=50.0, help=f"deg/s^2 (default 50, max {MAX_ACC_DEG_S2:g})")
        p.add_argument("-y", "--yes", action="store_true", help="skip the confirmation prompt")

    p = sub.add_parser("jog", help="move one joint by a small relative angle")
    p.add_argument("ip")
    p.add_argument("-j", "--joint", type=int, required=True, help="joint number, 1 = base")
    p.add_argument("-d", "--deg", type=float, required=True, help="relative angle in degrees, e.g. 5 or -5")
    p.add_argument("--max-step", type=float, default=15.0,
                   help=f"largest allowed |deg| (default 15, max {MAX_STEP_LIMIT_DEG:g})")
    motion_args(p, speed=10.0)
    p.set_defaults(func=cmd_jog)

    p = sub.add_parser("home", help="move to the controller's home pose")
    p.add_argument("ip")
    motion_args(p, speed=10.0)
    p.set_defaults(func=cmd_home)
    return parser


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)  # keep stdout/stderr ordered when piped to a log
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except ArmError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nAborted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
