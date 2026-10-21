"""Regression fixtures from the capture of 2026-10-21 (data/captures/2026-10-21): what the real arm measured, checked
against our model and tools. Standard library only. Usage: python tests/capture_tests.py"""
import json
import math
import os
import struct
import subprocess
import sys

from simrig import CAPTURE_DIR, ROOT, SCRIPTS, TOOLS, log_path

sys.path.insert(0, SCRIPTS)
import utra_model  # noqa: E402

results = []
STATUS = struct.Struct("<HBBBIIBBH")
CONFIG = struct.Struct("<H19fBB")
# Joint/pose pairs the controller reported (degrees; mm and degrees), as quoted in the brief and in ik_tests.mjs.
PAIRS = [
    ([14.982, 31.004, 120.245, 92.48, 96.434, 63.004], [287.24, -59.93, 628.6], [83.023, -62.467, 117.619]),
    ([36.111, 28.528, 138.784, 108.432, 45.765, 64.622], [308.38, -13.28, 477.29], [92.912, -63.32, 79.26]),
    ([14.982, 32.512, 120.169, 92.483, 96.434, 63.004], [273.99, -63.48, 635.98], [79.714, -62.079, 120.548]),
]


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)}")


def read_bin(name):
    with open(os.path.join(CAPTURE_DIR, name), "rb") as f:
        data = f.read()
    at, out = 0, []
    while at + 18 <= len(data):
        wall, mono, length = struct.unpack_from("<qqH", data, at)
        out.append((wall, mono, data[at + 18:at + 18 + length]))
        at += 18 + length
    return out


def status_rows():
    rows = []
    for _, mono, frame in read_bin("status10hz.bin"):
        _, axis, st, mode, brake, enable, err, warn, queued = STATUS.unpack_from(frame)
        v = struct.unpack_from(f"<{axis}f6f{axis}f", frame, STATUS.size)
        rows.append({"t": mono / 1e9, "status": st, "joints": [math.degrees(x) for x in v[:axis]], "pose": list(v[axis:axis + 6]),
                     "tau": list(v[axis + 6:])})
    return rows


def rpy_deg(T):
    """Roll, pitch, yaw (deg) the controller's way: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = math.asin(max(-1.0, min(1.0, -T[2][0])))
    return [math.degrees(math.atan2(T[2][1], T[2][2])), math.degrees(pitch), math.degrees(math.atan2(T[1][0], T[0][0]))]


def angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def main():
    model = utra_model.ArmModel()

    print("== the analysis tool reproduces the committed summary")
    out = log_path("capture_summary.json")
    r = subprocess.run([sys.executable, os.path.join(TOOLS, "capture", "capture.py"), "analyze", "--dir", CAPTURE_DIR,
                        "--summary", out], cwd=ROOT, capture_output=True, text=True, timeout=300)
    with open(os.path.join(CAPTURE_DIR, "summary.json")) as f:
        committed = json.load(f)
    fresh = json.load(open(out)) if r.returncode == 0 and os.path.exists(out) else None
    check("capture.py analyze runs on the committed raw data", fresh is not None, r.stderr[-500:])
    check("and its summary matches summary.json exactly", fresh == committed,
          [k for k in committed if fresh is None or fresh.get(k) != committed.get(k)])

    print("== the controller's reported pose is our forward kinematics of its measured joints")
    rows = status_rows()
    worst_mm, worst_deg = 0.0, 0.0
    for row in rows:
        T = model.frames(row["joints"])[-1]
        worst_mm = max(worst_mm, math.dist([T[i][3] * 1000.0 for i in range(3)], row["pose"][:3]))
        worst_deg = max(worst_deg, max(angle_diff(a, math.degrees(b)) for a, b in zip(rpy_deg(T), row["pose"][3:])))
    moving = sum(1 for row in rows if row["status"] == 1)
    check(f"all {len(rows)} status frames ({moving} moving): position within 0.001 mm", worst_mm < 0.001, worst_mm)
    check("and roll, pitch and yaw within 0.001 deg", worst_deg < 0.001, worst_deg)
    for joints, xyz, rpy in PAIRS:
        T = model.frames(joints)[-1]
        p = [T[i][3] * 1000.0 for i in range(3)]
        check(f"pair {joints}: position within 0.02 mm of the controller's", math.dist(p, xyz) < 0.02, (p, xyz))
        check("  and orientation within 0.002 deg", all(angle_diff(a, b) < 0.002 for a, b in zip(rpy_deg(T), rpy)),
              (rpy_deg(T), rpy))

    print("== the status and config streams")
    gaps = [(b["t"] - a["t"]) * 1000.0 for a, b in zip(rows, rows[1:])]
    rate = (len(rows) - 1) / (rows[-1]["t"] - rows[0]["t"])
    check("status at 10 Hz (9.95 to 10.05)", 9.95 < rate < 10.05, rate)
    check("no status gap over 150 ms in the whole capture", max(gaps) < 150.0, max(gaps))
    configs = [CONFIG.unpack_from(frame) for _, _, frame in read_bin("config.bin")]
    check("the config stream never changed", all(c == configs[0] for c in configs), len(set(configs)))
    c = configs[0]
    with open(os.path.join(CAPTURE_DIR, "settings.json")) as f:
        settings = json.load(f)["arm"]
    check("config stream: tool offset zero, as get_tcp_offset reads", list(c[7:13]) == settings["get_tcp_offset"][1] == [0.0] * 6)
    check("config stream: payload zero, as get_tcp_load reads", list(c[13:17]) == settings["get_tcp_load"][1] == [0.0] * 4)
    check("collision and teach sensitivity both 0", (c[20], c[21]) == (settings["get_collis_sens"][1], settings["get_teach_sens"][1]) == (0, 0))
    check("joint acceleration limit 200 deg/s^2 (3.4907 rad/s^2), tool 200 mm/s^2",
          abs(math.degrees(c[5]) - 200.0) < 1e-3 and c[1] == 200.0 and settings["get_tcp_maxacc"][1] == 200.0, (c[5], c[1]))
    check("gravity points down the base's z axis", list(c[17:20]) == [0.0, 0.0, -1.0])

    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
