"""Find the units of the joints' "tau" readings: record them with the arm still, with and without a known load, and
compare the change with the torque that load needs from each joint.

    python tools/capture/tau_check.py record before.json [--ip 192.168.2.1] [--seconds 10]
    (hang the load on the flange, or pull with a spring scale, and let the arm settle)
    python tools/capture/tau_check.py record loaded.json
    python tools/capture/tau_check.py compare before.json loaded.json --mass 1.0 [--at 0,0,40]
    python tools/capture/tau_check.py compare before.json pulled.json --force 0,20,0 [--at 0,0,0]

`record` only listens: it reads the 10 Hz status stream (port 30001) as a second client, which works alongside the jog
panel (capture 2026-10-21), and the config stream (port 30003) for the payload and sensitivities set. It sends nothing
to the arm. It refuses a recording in which the arm moved (any joint by more than 0.02 deg).

`compare` needs the two recordings at the same joint angles (within 0.05 deg). --at is where the load acts, in mm in
the flange frame (for a hanging mass, where its string is tied); --mass is in kg, pulling along the base frame's -z;
--force is in newtons, in the base frame. It prints, per joint, the change in tau, the torque each joint must add to
hold the load (N m: J^T m g for a mass, -J^T F for a force; tests/oracle_tests.py checks it against Pinocchio), and
their ratio. The same ratio on every joint that feels the load is tau's scale to N m at the joint; ratios that differ
between joints with different gearboxes (J1 and J2 120:1, the rest 100:1) point at a motor-side unit instead.
Standard library only.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "scripts"))

STATUS = struct.Struct("<HBBBIIBBH")  # as tools/capture/capture.py
CONFIG = struct.Struct("<H19fBB")
G = 9.80665
STILL_DEG = 0.02
SAME_POSE_DEG = 0.05
FEELS_NM = 0.3  # joints the load asks less of than this are reported but not judged


def decode_status(frame):
    _, axis, status, mode, _, enable, err, _, queued = STATUS.unpack_from(frame)
    v = struct.unpack_from(f"<{axis}f6f{axis}f", frame, STATUS.size)
    return {"status": status, "mode": mode, "enable": enable, "err": err, "queued": queued,
            "joints": [math.degrees(x) for x in v[:axis]], "tau": list(v[axis + 6:])}


def decode_config(frame):
    v = CONFIG.unpack_from(frame)
    return {"tcpOffset": list(v[7:13]), "tcpLoad": list(v[13:17]), "collisionSensitivity": v[20],
            "teachSensitivity": v[21]}


def frames(ip, port, seconds):
    """The frames one report port sends in `seconds`."""
    out, buf = [], b""
    deadline = time.monotonic() + seconds
    with socket.create_connection((ip, port), timeout=3.0) as sock:
        sock.settimeout(0.5)
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError(f"the controller closed port {port}")
            buf += chunk
            while len(buf) >= 2:
                length = struct.unpack_from("<H", buf)[0]
                if not 20 <= length <= 512:
                    buf = buf[1:]
                    continue
                if len(buf) < length:
                    break
                out.append(buf[:length])
                buf = buf[length:]
    return out


def summarise(status, config):
    """A recording: mean and spread of each joint's angle and tau, from decoded status frames."""
    if len(status) < 5:
        raise SystemExit(f"Only {len(status)} status frames arrived; is the arm on and the address right?")
    n = len(status[0]["joints"])
    joints = [[s["joints"][j] for s in status] for j in range(n)]
    taus = [[s["tau"][j] for s in status] for j in range(n)]
    mean = lambda v: sum(v) / len(v)
    sd = lambda v: math.sqrt(sum((x - mean(v)) ** 2 for x in v) / max(1, len(v) - 1))
    moved = max(max(v) - min(v) for v in joints)
    out = {"frames": len(status), "joints": [round(mean(v), 4) for v in joints],
           "jointSpreadDeg": round(moved, 4), "tau": [round(mean(v), 4) for v in taus],
           "tauSd": [round(sd(v), 4) for v in taus], "errors": sorted({s["err"] for s in status if s["err"]}),
           "enabled": all(s["enable"] for s in status), "modes": sorted({s["mode"] for s in status})}
    if config:
        out["config"] = config[-1]
    return out


def record(args):
    status = [decode_status(f) for f in frames(args.ip, 30001, args.seconds)]
    try:
        config = [decode_config(f) for f in frames(args.ip, 30003, 1.0) if len(f) >= CONFIG.size]
    except OSError:
        config = []
    out = summarise(status, config)
    out.update(ip=args.ip, at=time.strftime("%Y-%m-%dT%H:%M:%S"), seconds=args.seconds)
    if out["jointSpreadDeg"] > STILL_DEG:
        raise SystemExit(f"The arm moved during the recording ({out['jointSpreadDeg']:.3f} deg); record again with "
                         "it still.")
    with open(args.file, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    print(f"{out['frames']} frames in {args.seconds:g} s; joints {out['joints']} deg")
    print(f"tau {out['tau']} (sd {out['tauSd']})")
    if config:
        c = config[-1]
        print(f"controller: payload {c['tcpLoad']}, tool offset {c['tcpOffset']}, collision sensitivity "
              f"{c['collisionSensitivity']}, teach sensitivity {c['teachSensitivity']}")
    print(f"Saved {args.file}")


def holding_torques(joints_deg, at_mm, force_n, model=None):
    """The torque (N m) each joint must add to hold a force `force_n` (N, base frame) acting at `at_mm` (mm, flange
    frame), with the arm at `joints_deg`: -J^T F, J the point's positional Jacobian (per radian)."""
    if model is None:
        import utra_model
        model = utra_model.ArmModel()
    at = [v / 1000.0 for v in at_mm]

    def point(q):
        t = model.frames(q)[-1]
        return [sum(t[r][c] * at[c] for c in range(3)) + t[r][3] for r in range(3)]

    h = 1e-6  # rad
    out = []
    for j in range(len(joints_deg)):
        up, down = list(joints_deg), list(joints_deg)
        up[j] += math.degrees(h)
        down[j] -= math.degrees(h)
        column = [(a - b) / (2 * h) for a, b in zip(point(up), point(down))]
        out.append(-sum(c * f for c, f in zip(column, force_n)))
    return out


def triple(text, what):
    try:
        values = [float(v) for v in text.split(",")]
    except ValueError:
        values = []
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise SystemExit(f"{what} is three numbers separated by commas, like 0,0,40.")
    return values


def compare(args):
    with open(args.before, encoding="utf-8") as f:
        before = json.load(f)
    with open(args.after, encoding="utf-8") as f:
        after = json.load(f)
    if (args.mass is None) == (args.force is None):
        raise SystemExit("Give the load as --mass KG or as --force FX,FY,FZ (newtons, base frame).")
    force = [0.0, 0.0, -args.mass * G] if args.mass is not None else triple(args.force, "--force")
    at = triple(args.at, "--at")
    gap = max(abs(a - b) for a, b in zip(before["joints"], after["joints"]))
    if gap > SAME_POSE_DEG:
        raise SystemExit(f"The two recordings are {gap:.3f} deg apart; record both at the same pose.")
    joints = [(a + b) / 2 for a, b in zip(before["joints"], after["joints"])]
    expected = holding_torques(joints, at, force)
    rows, ratios = [], []
    for j, (b, a, e) in enumerate(zip(before["tau"], after["tau"], expected)):
        change = a - b
        noise = math.hypot(before["tauSd"][j], after["tauSd"][j])
        judged = abs(e) >= FEELS_NM
        ratio = change / e if judged else None
        if judged:
            ratios.append(ratio)
        rows.append({"joint": j + 1, "tauChange": round(change, 4), "noise": round(noise, 4),
                     "holdNm": round(e, 4), "ratio": None if ratio is None else round(ratio, 4)})
    print(f"Load: {force} N at {at} mm in the flange frame; pose {[round(v, 2) for v in joints]} deg")
    print("joint  tau change (noise)   needs N m   ratio")
    for r in rows:
        ratio = "" if r["ratio"] is None else f"{r['ratio']:8.3f}"
        print(f"  J{r['joint']}   {r['tauChange']:9.3f} ({r['noise']:.3f})   {r['holdNm']:9.3f}   {ratio}")
    verdict = {"rows": rows}
    if len(ratios) >= 2:
        mean = sum(ratios) / len(ratios)
        spread = max(abs(r - mean) for r in ratios) / abs(mean) if mean else float("inf")
        verdict.update(scale=round(mean, 4), spread=round(spread, 4))
        print(f"Mean ratio {mean:.3f} (the joints judged agree within {spread * 100:.0f}%): tau is about "
              f"{mean:.3f} units per N m at the joint" + ("" if spread <= 0.15 else
                                                           "; they disagree by more than 15%, so it's not a plain "
                                                           "joint torque (see the module's notes)."))
    else:
        print("Fewer than two joints feel this load enough to judge; use a heavier load or a pose with the flange "
              "further out.")
    return verdict


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record", help="record the arm's tau at rest (listens only)")
    r.add_argument("file")
    r.add_argument("--ip", default="192.168.2.1")
    r.add_argument("--seconds", type=float, default=10.0)
    c = sub.add_parser("compare", help="compare two recordings with the torque a known load needs")
    c.add_argument("before")
    c.add_argument("after")
    c.add_argument("--mass", type=float, help="kg, hanging (pulls along -z)")
    c.add_argument("--force", help="FX,FY,FZ in newtons, base frame")
    c.add_argument("--at", default="0,0,0", help="where the load acts, mm in the flange frame (default 0,0,0)")
    args = p.parse_args(argv)
    if args.cmd == "record":
        record(args)
    else:
        compare(args)


if __name__ == "__main__":
    main()
