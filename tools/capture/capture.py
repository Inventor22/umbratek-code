#!/usr/bin/env python3
"""Data capture for the machine-tending featureset work (2026-10-21, with the owner present).

Records the controller's 10 Hz status stream (port 30001; this firmware refuses the SDK's 100 Hz port, 30002)
and its config stream (port 30003) with host timestamps, while moving the arm through small excursions with the
jog panel's own API, so every joint move gets the panel's collision check, speed caps and watchdog, and STOP stays
one key away in the panel. Straight tool lines, which the panel doesn't check, are checked first with
line_check.mjs. Any error stops the arm.

    python tools/capture/capture.py probe     # 3 s of each stream, no motion
    python tools/capture/capture.py plan      # check the planned moves, no motion
    python tools/capture/capture.py go        # rest, joint excursions, tool lines and one STOP, then back to the start pose
    python tools/capture/capture.py analyze   # decode and summarise what was recorded

--dir picks the capture folder (default data/captures/2026-10-21 for analyze, data/captures/<today> otherwise), and
analyze --summary PATH writes the summary somewhere other than the folder's summary.json.
"""
import argparse
import json
import math
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(REPO, "data", "captures", "2026-10-21")
sys.path.insert(0, os.path.join(REPO, "scripts"))
import utra_model  # noqa: E402

IP = "192.168.2.1"
PANEL = "http://127.0.0.1:8765"
# Status frame: u16 length, u8 axis, u8 motion status, u8 motion mode, u32 brake mask, u32 enable mask, u8 error,
# u8 warning, u16 queued commands; then axis x f32 joints [rad], 6 x f32 TCP pose [mm, rad], axis x f32 joint
# current ("tau" in the SDK).
STATUS = struct.Struct("<HBBBIIBBH")
# Config frame: u16 length; f32 linear max acc and jerk, rotational max acc and jerk, joint (p2p) max acc and
# jerk, TCP offset x6, TCP load x4, gravity direction x3; u8 collision sensitivity, u8 teach sensitivity.
CONFIG = struct.Struct("<H19fBB")
MAX_FLANGE_MM = 300.0  # the owner's limit for how far the tool may move from where it starts

# Joint excursions from the start pose: (joint, +/- degrees, speeds in deg/s), each going +, back, -, back at
# each speed. The speeds keep the flange under about 220 mm/s.
JOINT_PLAN = [(0, 15.0, (10.0, 30.0)), (1, 10.0, (10.0, 20.0)), (2, 10.0, (10.0, 20.0)),
              (3, 20.0, (15.0, 45.0)), (4, 20.0, (15.0, 45.0)), (5, 30.0, (15.0, 60.0))]
# Straight tool lines, with the controller's own linear moves, out and back in LINE_STEP clicks: (axis, mm). -X
# is shorter: 95 mm that way the arm nears a singularity (the joints would swing ~10 deg per 5 mm).
LINES, LINE_STEP, LINE_SPEEDS = ((0, 100.0), (0, -50.0), (1, 100.0), (1, -100.0), (2, 100.0), (2, -100.0)), 50.0, (50.0, 100.0)
STOP_TEST = (0, 30.0, 30.0)  # J1 +30 deg at 30 deg/s, stopped about halfway


def decode_status(frame):
    _, axis, status, mode, brake, enable, err, warn, queued = STATUS.unpack_from(frame)
    v = struct.unpack_from(f"<{axis}f6f{axis}f", frame, STATUS.size)
    return {"status": status, "mode": mode, "brake": brake, "enable": enable, "err": err, "warn": warn,
            "queued": queued, "joints": [math.degrees(x) for x in v[:axis]],
            "pose": list(v[axis:axis + 3]) + [math.degrees(x) for x in v[axis + 3:axis + 6]],
            "current": list(v[axis + 6:])}


def decode_config(frame):
    v = CONFIG.unpack_from(frame)
    return {"linearMaxAcc": v[1], "linearJerk": v[2], "rotationalMaxAcc": v[3], "rotationalJerk": v[4],
            "jointMaxAcc": v[5], "jointJerk": v[6], "tcpOffset": list(v[7:13]), "tcpLoad": list(v[13:17]),
            "gravityDir": list(v[17:20]), "collisionSensitivity": v[20], "teachSensitivity": v[21]}


class Recorder(threading.Thread):
    """Every frame from one report port with the host's wall-clock and monotonic arrival times, written raw to
    <name>.bin as repeated (int64 wall ns, int64 monotonic ns, uint16 length, frame bytes)."""

    def __init__(self, port, name):
        super().__init__(daemon=True)
        self.port, self.name = port, name
        self.frames, self.chunks, self.resyncs = [], 0, 0
        self.halt, self.error = False, None

    def run(self):
        try:
            sock = socket.create_connection((IP, self.port), timeout=3.0)
            sock.settimeout(0.5)
            buf = b""
            with open(os.path.join(OUT, self.name + ".bin"), "wb") as f:
                while not self.halt:
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("the controller closed the connection")
                    wall, mono = time.time_ns(), time.perf_counter_ns()
                    self.chunks += 1
                    buf += chunk
                    while len(buf) >= 2:
                        length = struct.unpack_from("<H", buf)[0]
                        if not 20 <= length <= 512:
                            buf = buf[1:]
                            self.resyncs += 1
                            continue
                        if len(buf) < length:
                            break
                        frame, buf = buf[:length], buf[length:]
                        f.write(struct.pack("<qqH", wall, mono, length) + frame)
                        self.frames.append((wall, mono, frame))
            sock.close()
        except Exception as e:  # reported by the caller
            self.error = repr(e)


class Panel:
    """The running jog panel's API, as the page uses it, with every request logged."""

    def __init__(self):
        html = urllib.request.urlopen(PANEL + "/", timeout=5).read().decode("utf-8")
        self.token = re.search(r'const TOKEN = "([^"]+)"', html).group(1)
        self.config = json.loads(re.search(r"const CONFIG = (\{.*?\});\n", html).group(1))
        self.log, self.statuses, self.latest, self.polling = [], [], None, False

    def get_status(self):
        with urllib.request.urlopen(PANEL + "/api/status", timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body):
        req = urllib.request.Request(PANEL + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json", "X-Panel-Token": self.token})
        sent_wall, sent_mono = time.time_ns(), time.perf_counter_ns()
        try:
            with urllib.request.urlopen(req, timeout=15) as r:
                reply = json.loads(r.read())
        except urllib.error.HTTPError as e:
            reply = {"ok": False, "http": e.code}
        self.log.append({"path": path, "body": body, "sentWallNs": sent_wall, "sentMonoNs": sent_mono,
                         "replyMonoNs": time.perf_counter_ns(), "reply": reply})
        return reply

    def poll(self):
        """Keeps the panel's quiet-page watchdog fed, and logs what the panel sees, about 10 times a second."""
        self.polling = True
        while self.polling:
            try:
                s = self.get_status()
                self.latest = s
                self.statuses.append((time.time_ns(), time.perf_counter_ns(), s))
            except Exception:
                self.latest = None
            time.sleep(0.1)


def idle(s):
    return bool(s and s.get("live") and s.get("state") in (0, 2) and s.get("queued") == 0 and not s.get("goal"))


def wait_idle(panel, timeout):
    time.sleep(0.4)  # let the move show in the status first
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = panel.latest
        if s and s.get("err"):
            raise RuntimeError(f"controller error {s['err']} ({s.get('errName')})")
        if s and s.get("state") == 4:
            raise RuntimeError("the arm stopped")
        if idle(s):
            time.sleep(0.3)  # and stays settled
            if idle(panel.latest):
                return panel.latest
        time.sleep(0.05)
    raise RuntimeError("timed out waiting for the arm to settle")


def move(panel, targets, speed, label):
    reply = panel.post("/api/move", {"targets": [round(t, 3) for t in targets], "speed": speed})
    if not reply.get("ok") or not reply.get("moved"):
        raise RuntimeError(f"{label}: refused: {reply}")
    travel = max(abs(t - c) for t, c in zip(targets, panel.latest["joints"]))
    s = wait_idle(panel, travel / speed + 10.0)
    gap = max(abs(t - c) for t, c in zip(targets, s["joints"]))
    if gap > 0.2:
        raise RuntimeError(f"{label}: settled {gap:.2f} deg from its target")
    return s


def line(panel, axis, mm, speed, label):
    for _ in range(int(round(abs(mm) / LINE_STEP))):
        reply = panel.post("/api/jog", {"kind": "tool", "index": axis, "dir": 1 if mm > 0 else -1, "step": LINE_STEP,
                                        "rotStep": 5.0, "speed": speed})
        if not reply.get("ok") or not reply.get("moved"):
            raise RuntimeError(f"{label}: refused: {reply}")
    return wait_idle(panel, abs(mm) / speed + 10.0)


def plan_moves(start):
    moves = []
    for j, delta, speeds in JOINT_PLAN:
        for v in speeds:
            for sign in (1, -1):
                target = list(start)
                target[j] += sign * delta
                moves.append((f"J{j + 1} {sign * delta:+g} deg at {v:g} deg/s", target, v))
                moves.append((f"J{j + 1} back at {v:g} deg/s", list(start), v))
    return moves


def check_plan(panel, start):
    """No motion: every joint target's flange travel and collision check, and every tool line's IK and check."""
    model = utra_model.ArmModel()

    def flange(q):
        return [model.frames(q)[-1][i][3] * 1000.0 for i in range(3)]

    home = flange(start)
    ok = True
    targets = [m for m in plan_moves(start) if "back" not in m[0]]
    j, delta, _ = STOP_TEST
    stop_target = list(start)
    stop_target[j] += delta
    targets.append(("STOP test target", stop_target, STOP_TEST[2]))
    for label, target, _ in targets:
        travel = math.dist(home, flange(target))
        hit = model.check_path(start, target)
        good = hit is None and travel <= MAX_FLANGE_MM
        ok &= good
        print(f"  {'ok ' if good else 'NO '} {label:28s} flange {travel:6.1f} mm  {'clear' if hit is None else hit}")
    request = {"collision": panel.config["collision"], "limits": panel.config["dragLimits"], "start": start,
               "lines": [{"axis": a, "mm": mm} for a, mm in LINES]}
    path = os.path.join(OUT, "line_check_input.json")
    with open(path, "w") as f:
        json.dump(request, f)
    out = subprocess.run(["node", os.path.join(HERE, "line_check.mjs"), path], capture_output=True, text=True)
    if out.returncode != 0:
        print(out.stderr)
        return False
    report = json.loads(out.stdout)
    print(f"  tool at {report['homeMm']} mm (model)")
    for r in report["results"]:
        good = r["ok"] and r["worstJumpDegPer5mm"] < 3.0
        ok &= good
        print(f"  {'ok ' if good else 'NO '} line {r['axis']} {r['mm']:+.0f} mm: largest joint step "
              f"{r['worstJumpDegPer5mm']} deg per 5 mm {r['why']} {r['hit'] or ''}")
    return ok


def probe():
    os.makedirs(OUT, exist_ok=True)
    recs = [Recorder(30001, "probe_status10hz"), Recorder(30003, "probe_config")]
    for r in recs:
        r.start()
    time.sleep(3.0)
    for r in recs:
        r.halt = True
        r.join(2.0)
        print(f"{r.name}: {len(r.frames)} frames in 3 s, {r.chunks} reads, {r.resyncs} resyncs, error {r.error}")
    if recs[0].frames:
        print(json.dumps(decode_status(recs[0].frames[-1][2])))
    if recs[1].frames:
        print(json.dumps(decode_config(recs[1].frames[-1][2])))
    print("panel still live:", Panel().get_status().get("live"))


def go():
    os.makedirs(OUT, exist_ok=True)
    panel = Panel()
    threading.Thread(target=panel.poll, daemon=True).start()
    time.sleep(0.6)
    s = panel.latest
    if s and s.get("live") and not s.get("enabled") and s.get("state") == 4 and not s.get("err") and not s.get("queued"):
        print("Motors are off: switching them on through the panel's Enable.")
        reply = panel.post("/api/enable", {})
        if not reply.get("ok"):
            raise SystemExit(f"Not starting: enable refused ({reply}).")
        time.sleep(1.5)
        s = panel.latest
    if not idle(s) or not s.get("enabled") or s.get("err"):
        raise SystemExit(f"Not starting: the arm isn't ready and idle ({s}).")
    start = s["joints"]
    if not check_plan(panel, start):
        raise SystemExit("Not starting: a planned move failed its check.")
    status_rec, config_rec = Recorder(30001, "status10hz"), Recorder(30003, "config")
    status_rec.start()
    config_rec.start()
    marks = []

    def mark(label):
        marks.append({"label": label, "wallNs": time.time_ns(), "monoNs": time.perf_counter_ns()})
        print(f"{time.strftime('%H:%M:%S')} {label}", flush=True)

    def latest_joint(j):
        return decode_status(status_rec.frames[-1][2])["joints"][j]

    try:
        mark("rest")
        time.sleep(60.0)
        for label, target, v in plan_moves(start):
            mark(label)
            move(panel, target, v, label)
        for axis, mm in LINES:
            for v in LINE_SPEEDS:
                name = f"line {'XYZ'[axis]} {mm:+.0f} mm at {v:g} mm/s"
                mark(name)
                line(panel, axis, mm, v, name)
                mark(f"line {'XYZ'[axis]} back at {v:g} mm/s")
                line(panel, axis, -mm, v, name + " back")
        mark("joint return after lines")
        move(panel, start, 10.0, "return after lines")
        j, delta, v = STOP_TEST
        target = list(start)
        target[j] += delta
        mark(f"STOP test: J{j + 1} {delta:+g} deg at {v:g} deg/s")
        reply = panel.post("/api/move", {"targets": [round(t, 3) for t in target], "speed": v})
        if not reply.get("moved"):
            raise RuntimeError(f"STOP test move refused: {reply}")
        deadline = time.monotonic() + 10.0
        while abs(latest_joint(j) - start[j]) < delta / 2 and time.monotonic() < deadline:
            time.sleep(0.002)
        mark("STOP sent")
        reply = panel.post("/api/stop", {})
        if not reply.get("ok"):
            raise RuntimeError(f"STOP not confirmed: {reply}")
        time.sleep(2.5)
        mark("enable after STOP")
        reply = panel.post("/api/enable", {})
        if not reply.get("ok"):
            raise RuntimeError(f"enable refused: {reply}")
        time.sleep(1.0)
        wait_idle(panel, 10.0)
        mark("return to start")
        move(panel, start, 10.0, "return to start")
        mark("rest at end")
        time.sleep(15.0)
        mark("done")
    except BaseException as e:
        panel.post("/api/stop", {})
        mark(f"aborted: {e!r}")
        raise
    finally:
        status_rec.halt = config_rec.halt = True
        status_rec.join(2.0)
        config_rec.join(2.0)
        panel.polling = False
        with open(os.path.join(OUT, "marks.json"), "w") as f:
            json.dump({"start": start, "marks": marks, "recorderErrors": [status_rec.error, config_rec.error],
                       "resyncs": [status_rec.resyncs, config_rec.resyncs]}, f, indent=1)
        with open(os.path.join(OUT, "commands.json"), "w") as f:
            json.dump(panel.log, f, indent=1)
        with open(os.path.join(OUT, "panel_status.jsonl"), "w") as f:
            for wall, mono, st in panel.statuses:
                f.write(json.dumps({"wallNs": wall, "monoNs": mono, "status": st}) + "\n")
        with open(os.path.join(OUT, "panel_config.json"), "w") as f:
            json.dump(panel.config, f)
        print(f"saved: {len(status_rec.frames)} status frames, {len(config_rec.frames)} config frames, "
              f"{len(panel.log)} commands; recorder errors {status_rec.error} {config_rec.error}")


def read_bin(name):
    frames = []
    with open(os.path.join(OUT, name), "rb") as f:
        data = f.read()
    at = 0
    while at + 18 <= len(data):
        wall, mono, length = struct.unpack_from("<qqH", data, at)
        frames.append((wall, mono, data[at + 18:at + 18 + length]))
        at += 18 + length
    return frames


def pct(values, q):
    s = sorted(values)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))] if s else None


def analyze(summary_path=None):
    """Decodes the recordings and writes summary.json: stream timing, noise at rest, command latency, joint speeds
    and currents, how straight the controller's straight lines are, and the STOP test."""
    meta = json.load(open(os.path.join(OUT, "marks.json")))
    commands = json.load(open(os.path.join(OUT, "commands.json")))
    rows = []
    for wall, mono, frame in read_bin("status10hz.bin"):
        d = decode_status(frame)
        d["mono"] = mono / 1e9
        rows.append(d)
    configs = [decode_config(f) for _, _, f in read_bin("config.bin")]
    marks = meta["marks"]
    t_of = {m["label"]: m["monoNs"] / 1e9 for m in marks}
    summary = {"frames": len(rows), "configFrames": len(configs)}

    gaps = [(b["mono"] - a["mono"]) * 1000 for a, b in zip(rows, rows[1:])]
    summary["status10hz"] = {
        "seconds": round(rows[-1]["mono"] - rows[0]["mono"], 1),
        "rateHz": round((len(rows) - 1) / (rows[-1]["mono"] - rows[0]["mono"]), 2),
        "intervalMs": {"p1": round(pct(gaps, 0.01), 1), "p50": round(pct(gaps, 0.5), 1),
                       "p99": round(pct(gaps, 0.99), 1), "max": round(max(gaps), 1)},
        "gapsOver150ms": sum(g > 150 for g in gaps), "gapsOver1s": sum(g > 1000 for g in gaps)}
    summary["configConstant"] = all(c == configs[0] for c in configs)
    summary["config"] = configs[0]

    rest = [r for r in rows if t_of["rest"] <= r["mono"] < t_of[marks[1]["label"]]]

    def spread(values):
        mean = sum(values) / len(values)
        return mean, math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

    summary["rest"] = {"frames": len(rest), "joints": [], "current": []}
    for j in range(6):
        m, s = spread([r["joints"][j] for r in rest])
        summary["rest"]["joints"].append({"meanDeg": round(m, 4), "stdDeg": round(s, 5),
                                          "rangeDeg": round(max(r["joints"][j] for r in rest) - min(r["joints"][j] for r in rest), 5)})
        m, s = spread([r["current"][j] for r in rest])
        summary["rest"]["current"].append({"mean": round(m, 4), "std": round(s, 4),
                                           "min": round(min(r["current"][j] for r in rest), 4),
                                           "max": round(max(r["current"][j] for r in rest), 4)})
    xyz = [r["pose"][:3] for r in rest]
    centre = [sum(p[i] for p in xyz) / len(xyz) for i in range(3)]
    summary["rest"]["tcpScatterMm"] = round(max(math.dist(p, centre) for p in xyz), 4)

    # Command to first motion: the first frame after the request whose joints differ from the last frame before it,
    # for requests sent with the arm still (so not a second click during a move).
    latencies = []
    for c in commands:
        if c["path"] not in ("/api/move", "/api/jog") or not c["reply"].get("moved"):
            continue
        sent = c["sentMonoNs"] / 1e9
        before = [r for r in rows if r["mono"] <= sent]
        after = [r for r in rows if r["mono"] > sent]
        if len(before) < 3 or not after:
            continue
        if max(abs(a - b) for a, b in zip(before[-1]["joints"], before[-3]["joints"])) > 0.005:
            continue
        base = before[-1]["joints"]
        for r in after[:40]:
            if max(abs(a - b) for a, b in zip(r["joints"], base)) > 0.02:
                latencies.append(round((r["mono"] - sent) * 1000))
                break
    summary["commandToMotionMs"] = {"n": len(latencies), "min": min(latencies), "p50": pct(latencies, 0.5),
                                    "max": max(latencies), "note": "frame-quantised: status arrives every ~100 ms"}

    # Each segment between marks (by position: labels repeat): for joint moves, the peak speed seen at 10 Hz
    # against the speed asked for and the joint's current; for tool lines, how straight the path was.
    segments = []
    for i, mk in enumerate(marks[:-1]):
        label = mk["label"]
        t0, t1 = mk["monoNs"] / 1e9, marks[i + 1]["monoNs"] / 1e9
        seg = [r for r in rows if t0 <= r["mono"] < t1]
        if len(seg) < 3:
            continue
        entry = {"label": label, "seconds": round(t1 - t0, 2)}
        m = re.match(r"J(\d) ([+-]?[\d.]+) deg at ([\d.]+) deg/s", label) or re.match(r"J(\d) back at ([\d.]+) deg/s", label)
        if m:
            j = int(m.group(1)) - 1
            speeds = [abs(b["joints"][j] - a["joints"][j]) / max(1e-3, b["mono"] - a["mono"]) for a, b in zip(seg, seg[1:])]
            entry.update({"joint": j + 1, "peakDegPerS": round(max(speeds), 1),
                          "askedDegPerS": float(m.group(3) if m.lastindex == 3 else m.group(2)),
                          "currentRange": [round(min(r["current"][j] for r in seg), 3), round(max(r["current"][j] for r in seg), 3)],
                          "currentAtEnd": round(seg[-1]["current"][j], 3)})
        m = re.match(r"line ([XYZ]) ([+-][\d.]+) mm at ([\d.]+) mm/s", label)
        if m:
            p = [r["pose"][:3] for r in seg]
            a, b = p[0], p[-1]
            ab = [b[k] - a[k] for k in range(3)]
            n = math.sqrt(sum(v * v for v in ab)) or 1.0
            u = [v / n for v in ab]

            def off_line(q):
                w = [q[k] - a[k] for k in range(3)]
                along = sum(w[k] * u[k] for k in range(3))
                return math.sqrt(max(0.0, sum(v * v for v in w) - along * along))

            rot0 = seg[0]["pose"][3:]
            turn = max(max(abs((r["pose"][3 + k] - rot0[k] + 180) % 360 - 180) for k in range(3)) for r in seg)
            speeds = [math.dist(x["pose"][:3], y["pose"][:3]) / max(1e-3, y["mono"] - x["mono"]) for x, y in zip(seg, seg[1:])]
            entry.update({"axis": m.group(1), "travelMm": round(n, 2), "maxOffLineMm": round(max(off_line(q) for q in p), 3),
                          "maxOrientationChangeDeg": round(turn, 3), "peakMmPerS": round(max(speeds), 1),
                          "askedMmPerS": float(m.group(3))})
        segments.append(entry)
    summary["segments"] = segments

    # The STOP test: how long after the request the joint stops, and how far it goes meanwhile.
    stop_sent = next(c for c in commands if c["path"] == "/api/stop")["sentMonoNs"] / 1e9
    j = STOP_TEST[0]
    before = [r for r in rows if r["mono"] <= stop_sent][-1]
    after = [r for r in rows if r["mono"] > stop_sent]
    still = next((b for a, b in zip(after, after[1:]) if abs(b["joints"][j] - a["joints"][j]) < 0.02), None)
    model = utra_model.ArmModel()

    def flange(q):
        return [model.frames(q)[-1][i][3] * 1000.0 for i in range(3)]

    prior = [r for r in rows if r["mono"] <= stop_sent][-2]
    summary["stopTest"] = {
        "jointSpeedBeforeDegPerS": round(abs(before["joints"][j] - prior["joints"][j]) / max(1e-3, before["mono"] - prior["mono"]), 1),
        "stoppedWithinMs": round((still["mono"] - stop_sent) * 1000) if still else None,
        "travelAfterStopDeg": round(abs(still["joints"][j] - before["joints"][j]), 3) if still else None,
        "flangeTravelAfterStopMm": round(math.dist(flange(before["joints"]), flange(still["joints"])), 1) if still else None,
        "note": "frame-quantised: status arrives every ~100 ms"}
    # Switching the motors back on after the STOP: how far the arm shifts as the servos take hold, up to the next
    # move command (or 2 s). And the same for the first enable, which came before the 10 Hz recording started,
    # from the panel's own status log.
    enable = next(c for c in commands if c["path"] == "/api/enable" and c["sentMonoNs"] / 1e9 > stop_sent)
    sent = enable["sentMonoNs"] / 1e9
    next_move = next((c["sentMonoNs"] / 1e9 for c in commands if c["path"] == "/api/move" and c["sentMonoNs"] / 1e9 > sent),
                     sent + 2.0)
    was = [r for r in rows if r["mono"] <= sent][-1]
    now = [r for r in rows if r["mono"] < min(sent + 2.0, next_move)][-1]
    summary["reenableShift"] = {"windowS": round(min(2.0, next_move - sent), 2),
                                "maxJointDeg": round(max(abs(a - b) for a, b in zip(now["joints"], was["joints"])), 4),
                                "tcpMm": round(math.dist(now["pose"][:3], was["pose"][:3]), 3)}
    first = next(c for c in commands if c["path"] == "/api/enable")
    log = [json.loads(line) for line in open(os.path.join(OUT, "panel_status.jsonl"))]
    pre = [e["status"] for e in log if e["monoNs"] < first["sentMonoNs"] and e["status"].get("pose")]
    post = [e["status"] for e in log if first["sentMonoNs"] + 2e9 < e["monoNs"] < meta["marks"][0]["monoNs"]]
    if pre and post:
        summary["firstEnableShift"] = {
            "maxJointDeg": round(max(abs(a - b) for a, b in zip(post[-1]["joints"], pre[-1]["joints"])), 3),
            "tcpMm": round(math.dist(post[-1]["pose"][:3], pre[-1]["pose"][:3]), 2), "source": "panel status (0.01 mm, 0.001 deg)"}
    with open(summary_path or os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k != "segments"}, indent=1))
    for s in segments:
        if "maxOffLineMm" in s or ("peakDegPerS" in s and "back" not in s["label"]):
            print(s)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture the arm's status streams in small moves, or analyse a capture.")
    parser.add_argument("what", choices=("probe", "plan", "go", "analyze"))
    parser.add_argument("--dir", help="capture folder (default data/captures/2026-10-21 to analyze, else today's)")
    parser.add_argument("--summary", help="where analyze writes its summary (default <dir>/summary.json)")
    args = parser.parse_args()
    OUT = args.dir or (OUT if args.what == "analyze" else os.path.join(REPO, "data", "captures", time.strftime("%Y-%m-%d")))
    what = args.what
    if what == "probe":
        probe()
    elif what == "plan":
        p = Panel()
        print("all clear" if check_plan(p, p.get_status()["joints"]) else "SOMETHING FAILED")
    elif what == "go":
        go()
    elif what == "analyze":
        analyze(args.summary)
