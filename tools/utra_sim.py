"""Fake NTRO controller for offline testing of scripts/arm.py and scripts/jog_server.py.

Speaks UTRC on HOST:502 (same framing/registers as ut_sdk_python) and streams the
10 Hz status report on HOST:30001 (89-byte frames, first u16 = frame length, as
observed on the real controller). Motion commands queue and run one after another, like
the real controller. Logs every write it receives. HOST is 127.0.0.1 unless --host picks
another loopback address, so several simulators can run side by side.
"""
import argparse
import collections
import math
import os
import socket
import struct
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ut_sdk_python", "utapi"))
from common import crc16  # noqa: E402

AXIS = 6
NAMES = {0x0C: "RESET_ERR", 0x20: "MOTION_MODE", 0x21: "MOTION_ENABLE", 0x25: "MOTION_STATUS",
         0x30: "MOVET_LINE", 0x38: "MOVEJ_P2P", 0x3A: "MOVEJ_HOME"}


def log(msg):
    print(f"[sim {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Arm:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        self.uuid = b"SIMUTRA6100000001"
        self.sw = b"V1.0.SIM  20220302  "
        self.hw = b"NTRO-SIM  HW1.0     "
        self.mode, self.enable, self.brake = 0, 0, 0
        self.err, self.warn = a.error, 0
        self.status = 4  # stopped at power-on
        self.phantom_queued = a.queued  # a queue count with nothing behind it
        self.queue = collections.deque()  # (kind, goal, speed, label)
        self.current = None  # [kind, goal, speed, started]
        self.joints = [math.radians(d) for d in a.joints]
        self.offset = [0.0] * 6  # tool moves shift the pose by this much
        self.report_connections, self.nan_joints = 0, False
        self.started = time.monotonic()

    def queued(self):
        return len(self.queue) + self.phantom_queued

    def base_pose(self):
        return [418.0 + math.degrees(self.joints[0]), 56.0, 186.0, math.pi, 0.0, 1.5]

    def pose(self):
        b = self.base_pose()
        return [b[i] + self.offset[i] for i in range(3)] + [wrap(b[i] + self.offset[i]) for i in range(3, 6)]

    def target(self):
        """Mirrors the measured joints, like the real controller, plus injected faults."""
        t = list(self.joints)
        t[0] += math.radians(self.a.target_offset)
        t[0] += math.radians(self.a.enable_jump if self.enable else self.a.stale_target)
        return t

    def finish(self):
        self.current = None
        self.nan_joints = self.a.nan_after_move
        log("motion complete -> " + ("next" if self.queue else "ready"))
        if not self.queue:
            self.status = 2

    def tick(self, dt):
        with self.lock:
            if self.current is None:
                if self.queue and self.status in (0, 1, 2):
                    kind, goal, speed, label = self.queue.popleft()
                    self.current = [kind, goal, speed, time.monotonic()]
                    self.status = 1
                return
            if self.status != 1:
                return
            kind, goal, speed, started = self.current
            moving_for = time.monotonic() - started
            if self.a.error_after and moving_for > self.a.error_after:
                self.err, self.status, self.current = 9, 4, None
                self.queue.clear()
                log("injected error 9 mid-move -> stopped")
                return
            if self.a.pause_after and moving_for > self.a.pause_after:
                self.status = 3
                log("injected pause mid-move")
                return
            if kind == "joint":
                deltas = [g - j for g, j in zip(goal, self.joints)]
                lead, step = max(abs(d) for d in deltas), speed * dt
                if lead <= step:
                    self.joints = list(goal)
                    self.finish()
                else:
                    self.joints = [j + d * step / lead for j, d in zip(self.joints, deltas)]
            else:
                deltas = [g - o for g, o in zip(goal, self.offset)]
                dist = math.sqrt(sum(d * d for d in deltas[:3]) + sum((d * 100) ** 2 for d in deltas[3:]))
                step = speed * dt
                if dist <= step:
                    self.offset = list(goal)
                    self.finish()
                else:
                    self.offset = [o + d * step / dist for o, d in zip(self.offset, deltas)]

    def start_motion(self, kind, goal, speed, label, acc=None):
        shown = goal if kind == "cart" else [math.degrees(g) for g in goal]
        shown_acc = "" if acc is None else f" acc={acc if kind == 'cart' else math.degrees(acc):.1f}"
        log(f"{label}: goal=[{' '.join(f'{g:.2f}' for g in shown)}] speed={speed if kind == 'cart' else math.degrees(speed):.1f}"
            f"{shown_acc} (status={self.status} enable=0x{self.enable:x} err={self.err} queued={self.queued()})")
        if self.a.ignore_moves or self.status not in (0, 1, 2) or not self.enable or self.err:
            log(f"{label}: discarded")
        elif kind == "cart" and self.a.ik_fail:
            self.err, self.status, self.current = 21, 4, None
            self.queue.clear()
            log(f"{label}: no inverse kinematics solution -> error 21")
        else:
            self.queue.append((kind, goal, speed, label))
        return b"\x00\x00\x00\x00"

    def handle(self, rw, cmd, data):
        with self.lock:
            if rw == 0:
                comm = [self.a.servo_comm & 0xFF] * AXIS
                reads = {0x01: lambda: self.uuid, 0x02: lambda: self.sw, 0x03: lambda: self.hw,
                         0x04: lambda: bytes([AXIS]), 0x20: lambda: bytes([self.mode]),
                         0x21: lambda: struct.pack(">i", self.enable), 0x22: lambda: struct.pack(">i", self.brake),
                         0x23: lambda: bytes([self.err, self.warn]), 0x24: lambda: bytes(comm + [0] * AXIS),
                         0x25: lambda: bytes([self.status]), 0x26: lambda: struct.pack(">i", self.queued()),
                         0x40: lambda: struct.pack(">f", 20000.0), 0x41: lambda: struct.pack(">f", self.a.tcp_maxacc),
                         0x42: lambda: struct.pack(">f", math.radians(5000)),
                         0x43: lambda: struct.pack(">f", math.radians(self.a.joint_maxacc)),
                         0x50: lambda: struct.pack(">6f", *self.pose()),
                         0x51: lambda: struct.pack(f">{AXIS}f", *self.target())}
                return reads[cmd]() if cmd in reads else None
            log(f"WRITE {NAMES.get(cmd, hex(cmd))} data={data.hex()}")
            if cmd == 0x0C:
                if not self.a.sticky_error:
                    self.err = 0
                return b""
            if cmd == 0x20:
                self.mode = data[0]
                return b""
            if cmd == 0x21:
                if not self.a.enable_fails:
                    self.enable = ((1 << AXIS) - 1) if data[1] else 0
                return b""
            if cmd == 0x25:
                v = data[0]
                if v == 0 and not self.err:
                    self.status = 1 if self.current is not None else 2
                elif v == 3 and self.status == 1:
                    self.status = 3
                elif v == 4 and self.a.ignore_stop:
                    log("stop ignored (fault injection)")
                elif v == 4:
                    self.status, self.current, self.phantom_queued = 4, None, 0
                    self.queue.clear()
                return b""
            if cmd == 0x30:
                vals = struct.unpack(">9f", data)
                base = self.base_pose()
                goal = [vals[i] - base[i] for i in range(3)] + [wrap(vals[i] - base[i]) for i in range(3, 6)]
                return self.start_motion("cart", goal, vals[6], "MOVET_LINE", vals[7])
            if cmd == 0x38:
                vals = struct.unpack(f">{AXIS + 3}f", data)
                return self.start_motion("joint", list(vals[:AXIS]), vals[AXIS], "MOVEJ_P2P", vals[AXIS + 1])
            if cmd == 0x3A:
                vals = struct.unpack(">3f", data)
                return self.start_motion("joint", [0.0] * AXIS, vals[0], "MOVEJ_HOME")
            return None


def serve_cmd(conn, arm):
    buf = b""
    with conn:
        while True:
            try:
                chunk = conn.recv(1024)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while True:
                i = buf.find(b"\xAA\x55")
                if i < 0:
                    buf = buf[-1:]
                    break
                buf = buf[i:]
                if len(buf) < 3 or len(buf) < (buf[2] & 0x7F) + 5:
                    break
                n = (buf[2] & 0x7F) + 5
                frame, buf = buf[:n], buf[n:]
                if frame[-2:] != crc16.crc_modbus(frame[:-2]):
                    log("bad CRC, dropped")
                    continue
                rw, cmd = frame[3] >> 7, frame[3] & 0x7F
                reply = arm.handle(rw, cmd, frame[4:-2])
                if reply is None:
                    log(f"unhandled rw={rw} cmd=0x{cmd:02x}")
                    continue
                head = bytes([0x55, 0xAA, ((1 if arm.err else 0) << 7) | (len(reply) + 1), (rw << 7) | cmd])
                try:
                    conn.sendall(head + reply + crc16.crc_modbus(head + reply))
                except OSError:
                    return


def serve_report(conn, arm):
    with arm.lock:
        arm.report_connections += 1
        if arm.report_connections == arm.a.drift_on_connect:
            arm.joints[0] += math.radians(5)
            log(f"injected +5 deg drift on J1 at report connection {arm.report_connections}")
    with conn:
        try:
            while True:
                if arm.a.stall_every:
                    phase = (time.monotonic() - arm.started) % arm.a.stall_every
                    if phase >= arm.a.stall_every - arm.a.stall_for:  # the controller's Wi-Fi stall
                        time.sleep(0.02)
                        continue
                with arm.lock:
                    joints = [math.nan] * AXIS if arm.nan_joints else arm.joints
                    frame = struct.pack("<HBBBIIBBH", 41 + 8 * AXIS, AXIS, arm.status, arm.mode, arm.brake,
                                        arm.enable, arm.err, arm.warn, arm.queued())
                    frame += struct.pack(f"<{AXIS}f6f{AXIS}f", *joints, *arm.pose(), *([0.1] * AXIS))
                conn.sendall(frame)
                time.sleep(0.1)
        except OSError:
            pass


def listen(host, port, handler, arm, single=False):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Exclusive on Windows, where SO_REUSEADDR would let a second simulator share the port and steal connections.
    srv.setsockopt(socket.SOL_SOCKET, getattr(socket, "SO_EXCLUSIVEADDRUSE", socket.SO_REUSEADDR), 1)
    srv.bind((host, port))
    srv.listen(8)
    serving = None
    while True:
        conn, _ = srv.accept()
        if single and serving is not None and serving.is_alive():
            # Like the real controller: one command connection at a time; any other is closed at once.
            log(f"closed a second connection on :{port}")
            conn.close()
            continue
        serving = threading.Thread(target=handler, args=(conn, arm), daemon=True)
        serving.start()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--error", type=int, default=0, help="initial controller error code")
    p.add_argument("--sticky-error", action="store_true", help="RESET_ERR does not clear the error")
    p.add_argument("--target-offset", type=float, default=0.0, help="J1 target-vs-measured offset, always (deg)")
    p.add_argument("--stale-target", type=float, default=0.0, help="J1 target offset only while disabled (deg)")
    p.add_argument("--enable-jump", type=float, default=0.0, help="J1 target offset only while enabled (deg)")
    p.add_argument("--enable-fails", action="store_true", help="enable command has no effect")
    p.add_argument("--servo-comm", type=int, default=0, help="servo comm code reported for every joint")
    p.add_argument("--queued", type=int, default=0, help="initial phantom queued command count")
    p.add_argument("--ignore-moves", action="store_true", help="discard motion commands")
    p.add_argument("--ik-fail", action="store_true", help="reject tool moves with error 21")
    p.add_argument("--pause-after", type=float, default=0.0, help="pause N s into a move")
    p.add_argument("--error-after", type=float, default=0.0, help="raise error 9 N s into a move")
    p.add_argument("--drift-on-connect", type=int, default=0, help="move J1 +5 deg when the Nth report connection opens")
    p.add_argument("--nan-after-move", action="store_true", help="report NaN joints once a move completes")
    p.add_argument("--ignore-stop", action="store_true", help="ignore MOTION_STATUS=4 (stop) writes")
    p.add_argument("--estop-after", type=float, default=0.0, help="press the hardware E-stop N s after start")
    p.add_argument("--no-report", action="store_true", help="do not serve port 30001")
    p.add_argument("--stall-every", type=float, default=0.0, help="pause the status report every N s...")
    p.add_argument("--stall-for", type=float, default=3.1, help="...for this long, like the real controller over Wi-Fi")
    p.add_argument("--joints", type=lambda s: [float(v) for v in s.split(",")], default=[0, -30, 50, -10, 90, 0],
                   help="starting joint angles, deg, comma-separated")
    p.add_argument("--multi-client", action="store_true",
                   help="serve several command connections at once (the real controller serves one)")
    p.add_argument("--joint-maxacc", type=float, default=500.0, help="configured joint max acceleration, deg/s^2")
    p.add_argument("--tcp-maxacc", type=float, default=3000.0, help="configured tool max acceleration, mm/s^2")
    p.add_argument("--host", default="127.0.0.1", help="loopback address to serve on (default 127.0.0.1)")
    a = p.parse_args()
    arm = Arm(a)
    threading.Thread(target=listen, args=(a.host, 502, serve_cmd, arm, not a.multi_client), daemon=True).start()
    if not a.no_report:
        threading.Thread(target=listen, args=(a.host, 30001, serve_report, arm), daemon=True).start()
    log(f"listening on {a.host}:502{'' if a.no_report else ' and :30001'} ({vars(a)})")
    started = time.monotonic()
    while True:
        time.sleep(0.01)
        arm.tick(0.01)
        if a.estop_after and time.monotonic() - started > a.estop_after and arm.err != 1:
            with arm.lock:
                arm.err, arm.status, arm.enable, arm.current = 1, 4, 0, None
                arm.queue.clear()
            log("hardware E-stop pressed (fault injection)")
            a.estop_after = 0.0


if __name__ == "__main__":
    main()
