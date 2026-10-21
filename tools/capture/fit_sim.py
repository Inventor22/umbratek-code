#!/usr/bin/env python3
"""Fits the simulator's motion model to the capture: for every joint move the jog panel sent during the capture,
predicts the joints at each 10 Hz status frame with a candidate model (trapezoid or jerk-limited "double S", after a
start delay) and compares them with what the arm measured. Prints the best start delay for each model and the errors.

    python tools/capture/fit_sim.py [--dir data/captures/2026-10-21]

The speeds and accelerations are the ones the panel sent (its joint acceleration rule, capped by the controller's
200 deg/s^2), and the jerk is the controller's configured joint jerk (24 rad/s^3), all read from the capture.
"""
import argparse
import json
import math
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import motion_profile  # noqa: E402

STATUS = struct.Struct("<HBBBIIBBH")
CONFIG = struct.Struct("<H19fBB")
RAMP_S, JOINT_ACC_MIN = 0.3, 100.0  # scripts/jog_server.py's joint acceleration rule when the capture ran


def read_bin(folder, name):
    with open(os.path.join(folder, name), "rb") as f:
        data = f.read()
    at, out = 0, []
    while at + 18 <= len(data):
        _, mono, length = struct.unpack_from("<qqH", data, at)
        out.append((mono / 1e9, data[at + 18:at + 18 + length]))
        at += 18 + length
    return out


def load(folder):
    rows = []
    for t, frame in read_bin(folder, "status10hz.bin"):
        _, axis, st, *_ = STATUS.unpack_from(frame)
        v = struct.unpack_from(f"<{axis}f6f{axis}f", frame, STATUS.size)
        rows.append({"t": t, "status": st, "joints": [math.degrees(x) for x in v[:axis]], "pose": list(v[axis:axis + 6])})
    config = CONFIG.unpack_from(read_bin(folder, "config.bin")[0][1])
    with open(os.path.join(folder, "commands.json")) as f:
        commands = json.load(f)
    return rows, config, commands


def joint_moves(rows, config, commands):
    """(send time, start joints, goal joints, speed, acc, jerk, frames until the next command) per settled-start move."""
    acc_max = math.degrees(config[5])
    jerk = math.degrees(config[6])
    motion = [c for c in commands if c["path"] in ("/api/move", "/api/jog", "/api/stop", "/api/enable")]
    out = []
    for k, c in enumerate(motion):
        if c["path"] != "/api/move" or not c["reply"].get("moved"):
            continue
        if k + 1 < len(motion) and motion[k + 1]["path"] == "/api/stop":
            continue  # the STOP test's move, cut short
        sent = c["sentMonoNs"] / 1e9
        until = motion[k + 1]["sentMonoNs"] / 1e9 if k + 1 < len(motion) else sent + 5.0
        before = [r for r in rows if r["t"] <= sent]
        if len(before) < 3 or max(abs(a - b) for a, b in zip(before[-1]["joints"], before[-3]["joints"])) > 0.005:
            continue  # the arm wasn't still
        m = re.search(r"at (?:up to )?([\d.]+)\u00b0/s", c["reply"]["message"])
        speed = float(m.group(1)) if m else float(c["body"]["speed"])
        acc = min(max(JOINT_ACC_MIN, speed / RAMP_S), acc_max)
        start = before[-1]["joints"]
        goal = [t if t is not None else s for t, s in zip(c["body"]["targets"], start)]
        frames = [r for r in rows if sent < r["t"] < until]
        out.append({"label": c["reply"]["message"], "sent": sent, "start": start, "goal": goal, "speed": speed,
                    "acc": acc, "jerk": jerk, "frames": frames})
    return out


STEP = 0.002  # s: resolution of the lag's integration


def trajectory(move, profile_kind, lag):
    """The leading joint's fraction of the move at t = 0, STEP, 2 STEP, ... after motion starts (until 1 s after the
    profile ends), through a first-order lag of `lag` seconds (the servos' tracking), and the profile's duration."""
    lead = max(abs(g - s) for g, s in zip(move["goal"], move["start"]))
    if profile_kind == "trapezoid":
        p = motion_profile.trapezoid(lead, move["speed"], move["acc"])
    else:
        p = motion_profile.double_s(lead, move["speed"], move["acc"], move["jerk"])
    if lead <= 0:
        return [1.0], 0.0
    n = int((p.duration + 1.0) / STEP) + 1
    target = [p.position(k * STEP) / lead for k in range(n)]
    if lag <= 0:
        return target, p.duration
    alpha, x, out = 1.0 - math.exp(-STEP / lag), 0.0, []
    for value in target:
        out.append(x)
        x += (value - x) * alpha
    return out, p.duration


def sample(traj, t):
    if t <= 0:
        return 0.0
    k = t / STEP
    i = int(k)
    if i + 1 >= len(traj):
        return traj[-1]
    return traj[i] + (traj[i + 1] - traj[i]) * (k - i)


def errors(moves, profile_kind, delay, lag=0.0, cache={}):
    """Per-frame error (deg) of the leading joint over every move's frames while it runs and up to 0.5 s after."""
    out = []
    for mv in moves:
        key = (id(mv), profile_kind, lag)
        if key not in cache:
            cache[key] = trajectory(mv, profile_kind, lag)
        traj, duration = cache[key]
        lead = max(range(6), key=lambda i: abs(mv["goal"][i] - mv["start"][i]))
        span = mv["goal"][lead] - mv["start"][lead]
        for r in mv["frames"]:
            t = r["t"] - mv["sent"]
            if t > delay + duration + 0.5:
                break
            out.append(abs(mv["start"][lead] + span * sample(traj, t - delay) - r["joints"][lead]))
    return out


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def median(values):
    s = sorted(values)
    return s[len(s) // 2]


def line_groups(rows, config, commands):
    """Straight tool lines: runs of tool jogs sent back to back (the capture sent each line as 50 mm clicks)."""
    acc_max, jerk = config[1], config[2]
    jogs = [c for c in commands if c["path"] == "/api/jog" and c["reply"].get("moved")]
    groups, current = [], []
    for c in jogs:
        if current and (c["sentMonoNs"] - current[-1]["sentMonoNs"] > 5e8 or c["body"]["dir"] != current[0]["body"]["dir"]):
            groups.append(current)
            current = []
        current.append(c)
    if current:
        groups.append(current)
    out = []
    for g in groups:
        sent = g[0]["sentMonoNs"] / 1e9
        before = [r for r in rows if r["t"] <= sent]
        axis, sign = g[0]["body"]["index"], g[0]["body"]["dir"]
        speed = float(g[0]["body"]["speed"])
        acc = min(max(200.0, speed / RAMP_S), acc_max)  # the panel's tool acceleration rule, capped by the controller
        frames = [r for r in rows if sent < r["t"] < sent + 6.0]
        out.append({"sent": sent, "axis": axis, "sign": sign, "clicks": [float(c["body"]["step"]) for c in g],
                    "start": before[-1]["pose"][axis], "speed": speed, "acc": acc, "jerk": jerk, "frames": frames})
    return out


def line_errors(groups, delay, blended):
    """Errors (mm) along the line's axis: clicks run one after another from rest, or blended into one move."""
    out = []
    for g in groups:
        if blended:
            profiles = [motion_profile.double_s(sum(g["clicks"]), g["speed"], g["acc"], g["jerk"])]
        else:
            profiles = [motion_profile.double_s(d, g["speed"], g["acc"], g["jerk"]) for d in g["clicks"]]
        end = delay + sum(p.duration for p in profiles)
        for r in g["frames"]:
            t = r["t"] - g["sent"] - delay
            if t > end - delay + 0.5:
                break
            done = 0.0
            for p in profiles:
                if t >= p.duration:
                    done, t = done + p.distance, t - p.duration
                else:
                    done += p.position(t)
                    break
            out.append(abs(g["start"] + g["sign"] * done - r["pose"][g["axis"]]))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dir", default=os.path.join(REPO, "data", "captures", "2026-10-21"))
    args = parser.parse_args()
    rows, config, commands = load(args.dir)
    groups = line_groups(rows, config, commands)
    print(f"{len(groups)} straight lines; controller tool acc {config[1]:.0f} mm/s^2, jerk {config[2]:.0f} mm/s^3")
    for blended in (False, True):
        fits = sorted((rms(line_errors(groups, d / 1000.0, blended)), d) for d in range(0, 110, 5))
        r, d = fits[0]
        e = line_errors(groups, d / 1000.0, blended)
        print(f"  clicks {'blended into one move' if blended else 'one after another, from rest'}: best start delay "
              f"{d} ms, RMS {r:.3f} mm, median {median(e):.3f}, worst {max(e):.2f} mm over {len(e)} frames")
    moves = joint_moves(rows, config, commands)
    print(f"{len(moves)} joint moves from rest; controller joint acc {math.degrees(config[5]):.1f} deg/s^2, "
          f"jerk {math.degrees(config[6]):.1f} deg/s^3")
    fits = []
    for kind in ("trapezoid", "double_s"):
        for lag_ms in (0, 10, 20, 30, 40, 50, 60):
            for delay_ms in range(0, 110, 5):
                e = errors(moves, kind, delay_ms / 1000.0, lag_ms / 1000.0)
                fits.append((rms(e), kind, delay_ms, lag_ms, median(e), max(e), len(e)))
    fits.sort()
    for kind in ("trapezoid", "double_s"):
        for lag_ms in (0, 20, 40, 60):
            r = min(f for f in fits if f[1] == kind and f[3] == lag_ms)
            print(f"  {kind:<10} lag {lag_ms:2d} ms: best start delay {r[2]:3d} ms, RMS {r[0]:.4f}, median {r[4]:.4f}, "
                  f"worst {r[5]:.3f} deg over {r[6]} frames")
    r, kind, delay_ms, lag_ms, med, worst, n = fits[0]
    print(f"best: {kind}, start delay {delay_ms} ms, lag {lag_ms} ms: RMS {r:.4f} deg, median {med:.4f}, worst {worst:.3f}")
    for mv in moves:
        e = errors([mv], kind, delay_ms / 1000.0, lag_ms / 1000.0)
        print(f"    {mv['label'][:40]:<40} speed {mv['speed']:5.1f} acc {mv['acc']:5.1f}: RMS {rms(e):.3f} worst {max(e):.3f}")
    print(json.dumps({"model": kind, "startDelayS": delay_ms / 1000.0, "lagS": lag_ms / 1000.0, "rmsDeg": round(r, 4),
                      "medianDeg": round(med, 4), "worstDeg": round(worst, 4), "moves": len(moves)}))


if __name__ == "__main__":
    main()
