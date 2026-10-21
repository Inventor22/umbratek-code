"""Step-by-step probe of the controller's joint teaching (freedrive) mode: one register write per run, then the motion
mode and state the controller holds (read over port 502) beside what its 10 Hz report says (read from the recorder's
file, so no extra status connection). Run it with the jog panel server stopped (the controller serves one command
connection), with someone at the arm with a hand near it and the other on the E-stop.

    python tools/capture/teach_probe.py --ip 192.168.1.43 show       # read only
    python tools/capture/teach_probe.py --ip 192.168.1.43 sens 1     # set_teach_sens(1)
    python tools/capture/teach_probe.py --ip 192.168.1.43 mode 2     # set_motion_mode(2)
    python tools/capture/teach_probe.py --ip 192.168.1.43 ready      # set_motion_status(0), after a target check
    python tools/capture/teach_probe.py --ip 192.168.1.43 stop       # set_motion_status(4)
    python tools/capture/teach_probe.py --ip 192.168.1.43 enable     # set_motion_enable(8, 1), after a target check
    python tools/capture/teach_probe.py --ip 192.168.1.43 reset      # reset_err

Each run appends a JSON line to <dir>/teach_probe.jsonl. "ready" refuses when the controller's joint targets are more
than 1 degree from the measured joints, since making the arm ready would then pull it back to its targets.
"""
import argparse
import json
import math
import os
import struct
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import arm as armcli  # noqa: E402

HEADER = struct.Struct("<HBBBIIBBH")
RECORD = struct.Struct("<qqH")


def report_frames(path, since_wall):
    """Frames from the recorder's status10hz.bin at or after since_wall [s]: (wall, status, mode, enable, err, joints
    [deg])."""
    out = []
    try:
        data = open(path, "rb").read()
    except OSError:
        return out
    i = 0
    while i + RECORD.size <= len(data):
        wall, _, n = RECORD.unpack_from(data, i)
        i += RECORD.size
        if i + n > len(data):
            break
        if wall / 1e9 >= since_wall and n >= HEADER.size:
            _, axis, status, mode, _, enable, err, _, _ = HEADER.unpack_from(data, i)
            joints = struct.unpack_from(f"<{axis}f", data, i + HEADER.size)
            out.append((wall / 1e9, status, mode, enable, err, [math.degrees(j) for j in joints]))
        i += n
    return out


def registers(a):
    values = {}
    for name, call in (("mode", a.get_motion_mode), ("state", a.get_motion_status), ("enable", a.get_motion_enable),
                       ("brake", a.get_brake_enable), ("error", a.get_error_code), ("queued", a.get_cmd_num),
                       ("teach_sens", a.get_teach_sens), ("collis_sens", a.get_collis_sens)):
        ret, value = call()
        values[name] = value if ret in armcli.OK_CODES else f"read failed ({ret})"
    return values


def stamp(t):
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t % 1 * 1000):03d}"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("action", choices=("show", "sens", "mode", "ready", "stop", "enable", "reset"))
    parser.add_argument("value", nargs="?", type=int)
    parser.add_argument("--ip", default="192.168.2.1")
    parser.add_argument("--dir", default=os.path.join(REPO, "data", "captures", time.strftime("%Y-%m-%d")))
    parser.add_argument("--watch", type=float, default=3.0, help="seconds to follow the controller after the write")
    args = parser.parse_args()
    if args.action in ("sens", "mode") and args.value is None:
        parser.error(f"{args.action} needs a value")
    status_file = os.path.join(args.dir, "status10hz.bin")

    a = armcli.open_arm(args.ip)
    entry = {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "action": args.action, "value": args.value}
    try:
        before = registers(a)
        targets = [math.degrees(j) for j in armcli.target_joints(a)]
        latest = report_frames(status_file, time.time() - 1.0)
        entry["before"] = dict(before, targets=[round(t, 3) for t in targets])
        print("before:", before)
        if latest:
            measured = latest[-1][5]
            gap = max(abs(t - m) for t, m in zip(targets, measured))
            entry["before"]["target_gap_deg"] = round(gap, 3)
            print(f"report:  state {latest[-1][1]} mode {latest[-1][2]}; controller targets are within {gap:.3f} deg "
                  "of the measured joints")
        else:
            gap = None
            print(f"(no recent frames in {status_file}; is the recorder running?)")

        write = {"sens": lambda: a.set_teach_sens(args.value), "mode": lambda: a.set_motion_mode(args.value),
                 "ready": lambda: a.set_motion_status(0), "stop": lambda: a.set_motion_status(4),
                 "enable": lambda: a.set_motion_enable(8, 1), "reset": a.reset_err}.get(args.action)
        if write is None:
            return
        if args.action in ("ready", "enable") and (gap is None or gap > armcli.MAX_JOINT_MISMATCH_DEG):
            print("Not making the arm ready or enabling it: the targets can't be checked against the measured joints, "
                  f"or differ by more than {armcli.MAX_JOINT_MISMATCH_DEG:g} deg.")
            entry["refused"] = "target check"
            return
        sent = time.time()
        reply = write()
        entry["reply"] = reply
        print(f"{stamp(sent)} {args.action} {'' if args.value is None else args.value} -> reply {reply}")
        trace, last = [], None
        while time.time() - sent < args.watch:
            now = time.time()
            ret_m, mode = a.get_motion_mode()
            ret_s, state = a.get_motion_status()
            row = (mode if ret_m in armcli.OK_CODES else f"?{ret_m}", state if ret_s in armcli.OK_CODES else f"?{ret_s}")
            if row != last:
                print(f"  {stamp(now)} (+{now - sent:.2f} s) registers: mode {row[0]} state {row[1]}")
                trace.append([round(now - sent, 3), row[0], row[1]])
                last = row
            time.sleep(0.1)
        entry["registers_trace"] = trace
        time.sleep(0.3)  # let the recorder write the last frames
        frames, last, seen = report_frames(status_file, sent - 0.3), None, []
        for wall, status, mode, enable, err, joints in frames:
            if (status, mode, enable, err) != last:
                print(f"  {stamp(wall)} (+{wall - sent:.2f} s) report: state {status} mode {mode} enable {enable} "
                      f"err {err}")
                seen.append([round(wall - sent, 3), status, mode, enable, err])
                last = (status, mode, enable, err)
        entry["report_trace"] = seen
        entry["after"] = registers(a)
        print("after: ", entry["after"])
    finally:
        with open(os.path.join(args.dir, "teach_probe.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps(entry) + "\n")
        try:
            a.socket_fp.close()
        except Exception as e:  # the process exits anyway
            print(f"close: {e!r}")


if __name__ == "__main__":
    code = 0
    try:
        main()
    except armcli.ArmError as e:
        print(e, file=sys.stderr)
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # the SDK's receive thread would keep the process alive
