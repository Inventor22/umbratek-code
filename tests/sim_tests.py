"""The simulator against the real arm: tools/utra_sim.py's motion model replayed against the capture of 2026-10-21,
then the running simulator checked over the network for what the capture showed (10 Hz state, the refused 100 Hz
port, the config stream, the start delay, the enable shift, STOP) and for the registers and the control port the
featureset's tests rely on. Standard library only. Usage: python tests/sim_tests.py"""
import json
import math
import os
import socket
import struct
import sys
import time

from simrig import CAPTURE_DIR, HOST, SCRIPTS, TOOLS, start_sim, stop, wait_port

sys.path.insert(0, SCRIPTS)
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.join(TOOLS, "capture"))
import arm as armcli  # noqa: E402
import fit_sim  # noqa: E402
import utra_model  # noqa: E402
import utra_sim  # noqa: E402

results = []
MODEL = utra_model.ArmModel()
IDENTITY = utra_sim.pose_matrix([0.0] * 6)


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)}")


def pct(values, q):
    s = sorted(values)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


def tcp_mm(q_rad):
    T = utra_sim.flange(q_rad)
    return [T[i][3] * 1000.0 for i in range(3)]


# ---- (a) the model against the capture --------------------------------------------------------------------------

def replay_joint_moves(rows, config, commands):
    print("== joint moves: the simulator's profile against the capture's 43 joint moves from rest")
    moves = fit_sim.joint_moves(rows, config, commands)
    jerk = config[6]
    errors = []
    for mv in moves:
        start, goal = [math.radians(v) for v in mv["start"]], [math.radians(v) for v in mv["goal"]]
        m = utra_sim.JointMotion("replay", start, goal, math.radians(mv["speed"]), math.radians(mv["acc"]), jerk)
        lead = max(range(6), key=lambda i: abs(goal[i] - start[i]))
        for r in mv["frames"]:
            t = r["t"] - mv["sent"] - utra_sim.START_DELAY_JOINT_S
            if t > m.duration + 0.5:
                break
            errors.append(abs(math.degrees(m.joints(t)[lead]) - r["joints"][lead]))
    check(f"{len(moves)} moves, {len(errors)} frames: median error under 0.05 deg", pct(errors, 0.5) < 0.05, pct(errors, 0.5))
    check("95th percentile under 0.3 deg", pct(errors, 0.95) < 0.3, pct(errors, 0.95))


def replay_lines(rows, config, commands):
    print("== straight lines: the simulator's line moves (its own inverse kinematics) against the capture's 24 lines")
    groups = fit_sim.line_groups(rows, config, commands)
    errors, off_line = [], []
    for g in groups:
        before = [r for r in rows if r["t"] <= g["sent"]][-1]
        q = [math.radians(v) for v in before["joints"]]
        pose0 = list(before["pose"])
        motions, ok = [], True
        for k in range(len(g["clicks"])):
            goal = list(pose0)
            goal[g["axis"]] += g["sign"] * sum(g["clicks"][:k + 1])
            m = utra_sim.LineMotion("replay", q, IDENTITY, goal, g["speed"], g["acc"], g["jerk"])
            if m.solve(IDENTITY) is not None:
                ok = False
                break
            motions.append(m)
            q = m.joints(m.duration)
        if not ok:
            check(f"line along {'XYZ'[g['axis']]} solved", False)
            continue
        end = utra_sim.START_DELAY_LINE_S + sum(m.duration for m in motions) + 0.3
        for r in g["frames"]:
            if r["t"] - g["sent"] > end:
                break
            t = r["t"] - g["sent"] - utra_sim.START_DELAY_LINE_S
            for m in motions:
                if t <= m.duration or m is motions[-1]:
                    p = tcp_mm(m.joints(min(t, m.duration)))
                    break
                t -= m.duration
            errors.append(abs(p[g["axis"]] - r["pose"][g["axis"]]))
            others = [i for i in range(3) if i != g["axis"]]
            off_line.append(max(abs(p[i] - pose0[i]) for i in others))
    check(f"{len(groups)} lines, {len(errors)} frames: median error along the line under 0.15 mm", pct(errors, 0.5) < 0.15,
          pct(errors, 0.5))
    check("95th percentile under 1 mm", pct(errors, 0.95) < 1.0, pct(errors, 0.95))
    check("and the simulated tool never leaves its line by more than 0.01 mm", max(off_line) < 0.01, max(off_line))


def replay_stop(rows, commands):
    print("== STOP: the simulator's halt against the capture's (J1 at 30 deg/s); it was built from this one")
    stopped = next(c for c in commands if c["path"] == "/api/stop")["sentMonoNs"] / 1e9
    a, b = [r for r in rows if r["t"] <= stopped][-2:]
    speed = (b["joints"][0] - a["joints"][0]) / (b["t"] - a["t"])  # deg/s, from the last two frames before STOP
    check("J1 was turning at 30 deg/s when STOP was sent", abs(speed - 30.0) < 0.2, speed)
    at_stop = b["joints"][0] + speed * (stopped - b["t"])
    overrun = speed * utra_sim.STOP_OVERRUN_S
    final = at_stop + overrun
    after = [r for r in rows if r["t"] > stopped][:6]
    worst = 0.0
    for r in after:
        t = r["t"] - stopped
        k_off = next((v0 + (v1 - v0) * (t - t0) / (t1 - t0) for (t0, v0), (t1, v1)
                      in zip(utra_sim.STOP_TRANSIENT, utra_sim.STOP_TRANSIENT[1:]) if t <= t1), 0.0)
        worst = max(worst, abs(final + k_off * overrun - r["joints"][0]))
    check("the simulator's halt matches J1 in the six frames after STOP within 0.005 deg", worst < 0.005, worst)
    check("coming to rest 0.21 deg past where STOP found it, as measured", abs(after[-1]["joints"][0] - final) < 0.005,
          (after[-1]["joints"][0], final))


def replay_enable(rows, commands):
    print("== enable shift")
    enable = [c for c in commands if c["path"] == "/api/enable"][-1]
    sent = enable["sentMonoNs"] / 1e9
    was = [r for r in rows if r["t"] <= sent][-1]["joints"]
    now = [r for r in rows if sent + 0.5 < r["t"] < sent + 1.5][-1]["joints"]
    measured = [b - a for a, b in zip(was, now)]
    check("ENABLE_SHIFT_DEG is the measured shift, joint by joint, within 0.0005 deg",
          all(abs(a - b) < 5e-4 for a, b in zip(measured, utra_sim.ENABLE_SHIFT_DEG)), measured)
    q = [math.radians(v) for v in was]
    shifted = [a + math.radians(d) for a, d in zip(q, utra_sim.ENABLE_SHIFT_DEG)]
    shift_mm = math.dist(tcp_mm(q), tcp_mm(shifted))
    check("which moves the tool 0.46 mm there, as measured (0.456)", abs(shift_mm - 0.456) < 0.01, shift_mm)


# ---- (b) the running simulator ------------------------------------------------------------------------------

class Control:
    """The simulator's control port."""

    def __init__(self):
        self.sock = socket.create_connection((HOST, utra_sim.CONTROL_PORT), timeout=5)
        self.buf = b""

    def __call__(self, **request):
        self.sock.sendall(json.dumps(request).encode() + b"\n")
        while b"\n" not in self.buf:
            self.buf += self.sock.recv(65536)
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)


def frames(seconds):
    """Status frames for `seconds`, each with its arrival time from the high-resolution clock (time.monotonic()
    follows the ~15.6 ms system tick on Windows before Python 3.13, too coarse to time 100 ms gaps)."""
    report = armcli.StatusReport(HOST)
    out, t0 = [], time.perf_counter()
    try:
        while time.perf_counter() - t0 < seconds:
            f = report.read()
            f["t"] = time.perf_counter()
            out.append(f)
    finally:
        report.close()
    return out


def wait_state(ctl, pred, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = ctl(cmd="state")
        if pred(s):
            return s
        time.sleep(0.02)
    return ctl(cmd="state")


def enabled(arm):
    arm.reset_err()
    arm.set_motion_mode(0)
    arm.set_motion_enable(8, 1)
    arm.set_motion_status(0)


def live_streams():
    print("== the running simulator: streams and ports")
    sim, _ = start_sim((), "sim_streams.log")
    try:
        got = frames(3.05)
        gaps = [(b["t"] - a["t"]) * 1000 for a, b in zip(got, got[1:])]
        check("status at 10 Hz: 30 or 31 frames in 3 s", len(got) in (30, 31, 32), len(got))
        check("frame gaps 85 to 115 ms for 90% of frames (the capture's: 89 to 103 ms for 98%)",
              pct(gaps, 0.05) > 85 and pct(gaps, 0.95) < 115, sorted(gaps))
        try:
            socket.create_connection((HOST, 30002), timeout=1).close()
            refused = False
        except OSError:
            refused = True
        check("port 30002 (the SDK's 100 Hz stream) refuses connections, as on this firmware", refused)
        sock = socket.create_connection((HOST, 30003), timeout=2)
        data = b""
        while len(data) < 160:
            data += sock.recv(4096)
        sock.close()
        v = struct.unpack_from("<H19fBB", data)
        check("port 30003 streams 80-byte config frames", v[0] == 80)
        check("with the capture's settings: 200 mm/s^2, 2000 mm/s^3, 200 deg/s^2, 24 rad/s^3, no tool, sensitivity 0",
              abs(v[1] - 200) < 1e-3 and abs(v[2] - 2000) < 1e-3 and abs(math.degrees(v[5]) - 200) < 1e-3
              and abs(v[6] - 24) < 1e-4 and v[7:17] == (0.0,) * 10 and v[17:20] == (0.0, 0.0, -1.0) and v[20:] == (0, 0), v)
        r2 = armcli.StatusReport(HOST)
        r2.read()
        r2.close()
        check("a second client on 30001 is served alongside the first", True)
    finally:
        stop(sim)


def live_motion():
    print("== the running simulator: start delay, enable shift, STOP, the queue count")
    sim, log = start_sim(("--joints", "36.111,28.528,138.784,108.432,45.765,64.622"), "sim_motion.log")
    arm = None
    try:
        arm = armcli.open_arm(HOST)
        ctl = Control()
        before = ctl(cmd="state")["joints"]
        enabled(arm)
        after = wait_state(ctl, lambda s: s["status"] in (0, 2))["joints"]
        shift = [b - a for a, b in zip(before, after)]
        check("enabling shifts the joints by ENABLE_SHIFT_DEG", all(abs(s - e) < 1e-4 for s, e in
                                                                    zip(shift, utra_sim.ENABLE_SHIFT_DEG)), shift)
        latencies = []
        report = armcli.StatusReport(HOST)
        try:
            for k in range(12):
                time.sleep(0.137 * (k % 5) / 5)  # spread the commands across the 100 ms frame period
                last = report.read()
                base = last["joints"][0]
                target = list(last["joints"])
                target[0] += math.radians(2.0 if k % 2 == 0 else -2.0)
                t0 = time.perf_counter()
                arm.moveto_joint_p2p(target, math.radians(10), math.radians(100), 0)
                while True:
                    f = report.read()
                    if abs(math.degrees(f["joints"][0] - base)) > 0.02:
                        latencies.append((time.perf_counter() - t0) * 1000)
                        break
                while f["status"] == 1 or abs(math.degrees(f["joints"][0] - target[0])) > 0.01:
                    f = report.read()
        finally:
            report.close()
        med = pct(latencies, 0.5)
        check(f"command to first visible motion (> 0.02 deg): median {med:.0f} ms, as the capture's 111 ms within 40",
              abs(med - 111) < 40, latencies)
        check("never under 40 ms or over 200", min(latencies) > 40 and max(latencies) < 200, latencies)

        report = armcli.StatusReport(HOST)
        try:
            start = report.read()["joints"]
            goal = list(start)
            goal[0] += math.radians(30.0)
            arm.moveto_joint_p2p(goal, math.radians(30), math.radians(100), 0)
            while True:
                f = report.read()
                if math.degrees(f["joints"][0] - start[0]) > 15.0:
                    break
            at_stop = f["joints"][0]
            arm.set_motion_status(4)
            after = [report.read() for _ in range(6)]
        finally:
            report.close()
        travel = math.degrees(after[-1]["joints"][0] - at_stop)
        check("STOP from 30 deg/s: the first frame after says stopped", after[0]["status"] == 4, after[0]["status"])
        check(f"and J1 comes to rest {travel:.2f} deg past the frame before STOP (0.18 to 3.5: 0.19 plus up to one frame at "
              "30 deg/s)", 0.18 < travel < 3.5, travel)
        swing = max(abs(math.degrees(a["joints"][0] - after[-1]["joints"][0])) for a in after[3:])
        check("settled within 0.3 s (the last three frames within 0.005 deg)", swing < 0.005, swing)

        enabled(arm)
        wait_state(ctl, lambda s: s["status"] in (0, 2))
        start = ctl(cmd="state")["joints"]
        report = armcli.StatusReport(HOST)
        try:
            report.read()
            for k in range(3):
                target = [math.radians(v) for v in start]
                target[5] += math.radians(2.0 * (k + 1))
                arm.moveto_joint_p2p(target, math.radians(10), math.radians(100), 0)
            time.sleep(0.06)  # the first has started
            ret, count = arm.get_cmd_num()
            first = [report.read() for _ in range(2)]
        finally:
            report.close()
        check("three moves sent at once, the first under way: CMD_NUM and the next frames count 1 queued, not the one "
              "in the planner", count == 1 and all(f["queued"] == 1 for f in first), (count, [f["queued"] for f in first]))
        s = wait_state(ctl, lambda s: not s["moving"] and s["queued"] == 0, timeout=12)
        check("they run one after another to the last goal", abs(s["joints"][5] - (start[5] + 6.0)) < 0.01, s["joints"])
    finally:
        if arm is not None:
            arm.socket_fp.close()
        stop(sim)


def live_lines_and_pause():
    print("== the running simulator: straight lines, pause and resume, faults")
    sim, log = start_sim((), "sim_lines.log")
    arm = None
    try:
        arm = armcli.open_arm(HOST)
        ctl = Control()
        enabled(arm)
        wait_state(ctl, lambda s: s["status"] in (0, 2))
        s = ctl(cmd="state")
        p0 = s["pose"]
        goal = list(p0)
        goal[0] += 80.0
        goal[2] -= 40.0
        report = armcli.StatusReport(HOST)
        worst, track = 0.0, []
        try:
            arm.moveto_cartesian_line(goal, 40.0, 200.0, 0)
            paused = resumed = False
            t0 = time.monotonic()
            while time.monotonic() - t0 < 10:
                f = report.read()
                p = f["pose"]
                u = [goal[i] - p0[i] for i in range(3)]
                n = math.sqrt(sum(x * x for x in u))
                w = [p[i] - p0[i] for i in range(3)]
                along = sum(w[i] * u[i] for i in range(3)) / n
                worst = max(worst, math.sqrt(max(0.0, sum(x * x for x in w) - along * along)))
                track.append((f["status"], along))
                if not paused and along > 30.0:
                    arm.set_motion_status(3)
                    paused, t_pause = True, time.monotonic()
                if paused and not resumed and time.monotonic() - t_pause > 1.0:
                    held = along
                    arm.set_motion_status(0)
                    resumed = True
                if resumed and f["status"] in (0, 2) and abs(along - n) < 0.01:
                    break
        finally:
            report.close()
        check("a straight line keeps the tool within 0.01 mm of it, through a pause", worst < 0.01, worst)
        check("pause holds still (status 3) and resume finishes the line", paused and resumed and abs(track[-1][1] - n) < 0.01
              and any(st == 3 for st, _ in track), track[-5:])
        end = ctl(cmd="state")["pose"]
        check("ending at the goal pose", math.dist(end[:3], goal[:3]) < 0.01 and
              max(abs(utra_sim.wrap(a - b)) for a, b in zip(end[3:], goal[3:])) < 1e-4, (end, goal))

        far = list(end)
        far[0] += 2000.0
        arm.moveto_cartesian_line(far, 50.0, 200.0, 0)
        s = wait_state(ctl, lambda s: s["err"] == 21, timeout=5)
        check("a line out of reach raises error 21 (Inverse kinematics error) before moving", s["err"] == 21
              and math.dist(s["pose"][:3], end[:3]) < 0.01, (s["err"], s["pose"]))

        enabled(arm)
        wait_state(ctl, lambda s: s["status"] in (0, 2))
        q = ctl(cmd="state")["joints"]
        target = [math.radians(v) for v in q]
        target[2] = math.radians(170.0)
        arm.moveto_joint_p2p(target, math.radians(10), math.radians(100), 0)
        s = wait_state(ctl, lambda s: s["err"] == 23, timeout=3)
        check("a joint target past J3's limit (165.07 deg) raises error 23", s["err"] == 23, s["err"])

        enabled(arm)
        wait_state(ctl, lambda s: s["status"] in (0, 2))
        q = ctl(cmd="state")["joints"]
        target = [math.radians(v) for v in q]
        target[2] -= math.radians(120)  # long enough to reach 130 deg/s at 200 deg/s^2
        arm.moveto_joint_p2p(target, math.radians(130), math.radians(200), 0)
        s = wait_state(ctl, lambda s: s["err"] == 24, timeout=5)
        check("J3 sent at 130 deg/s faults with error 24 past 120.3 deg/s, as the real J3 did", s["err"] == 24, s["err"])
    finally:
        if arm is not None:
            arm.socket_fp.close()
        stop(sim)


def live_registers():
    print("== the running simulator: tool, payload, sensitivities, GPIO, freedrive, collision detection")
    sim, log = start_sim((), "sim_registers.log")
    arm = None
    try:
        arm = armcli.open_arm(HOST)
        ctl = Control()
        offset = [10.0, -20.0, 150.0, 0.1, -0.2, 0.3]
        arm.set_tcp_offset(offset)
        arm.set_tcp_load(1.25, [0.0, 5.0, 60.0])
        arm.set_collis_sens(3)
        arm.set_teach_sens(2)
        ret_o, got_o = arm.get_tcp_offset()
        ret_l, got_l = arm.get_tcp_load()
        check("tool offset and payload read back as set", all(abs(a - b) < 1e-5 for a, b in zip(got_o, offset))
              and all(abs(a - b) < 1e-5 for a, b in zip(got_l, [1.25, 0.0, 5.0, 60.0])), (got_o, got_l))
        check("sensitivities read back", arm.get_collis_sens()[1] == 3 and arm.get_teach_sens()[1] == 2)
        sock = socket.create_connection((HOST, 30003), timeout=2)
        time.sleep(0.25)
        data = b""
        while len(data) < 80:
            data += sock.recv(4096)
        sock.close()
        v = struct.unpack_from("<H19fBB", data[-80:] if len(data) >= 80 else data)
        check("and the config stream reports them", all(abs(a - b) < 1e-5 for a, b in zip(v[7:13], offset))
              and abs(v[13] - 1.25) < 1e-6 and v[20:] == (3, 2), v)
        s = ctl(cmd="state")
        T = utra_sim.mat_mul(utra_sim.flange([math.radians(x) for x in s["joints"]]), utra_sim.pose_matrix(offset))
        expect = utra_sim.matrix_pose(T)
        frame = frames(0.15)[-1]
        check("the status pose is the flange's pose with the tool offset applied (flange frame, mm and rpy)",
              math.dist(frame["pose"][:3], expect[:3]) < 1e-3, (frame["pose"], expect))

        arm.set_tgpio_digit_out(0x00030001)  # outputs 1 and 2: 1 high, 2 low
        arm.set_cgpio_digit_out(0x00040004)
        out = ctl(cmd="state")["outputs"]
        check("tool and controller digital outputs set through the SDK", out == {"tool": 1, "controller": 4}, out)
        ctl(cmd="input", line="tool", bit=0, value=1)
        ctl(cmd="input", line="controller", bit=3, value=1)
        st_t, tin = arm.get_tgpio_in()
        st_c, cin = arm.get_cgpio_in()
        check("inputs set on the control port read back through the SDK", tin[1] == 1 and cin[1] == 8, (tin, cin))

        # Freedrive as the real arm did it (2026-10-21, data/captures/2026-10-21-tests/teach_probe.jsonl).
        enabled(arm)
        wait_state(ctl, lambda s: s["status"] in (0, 2))
        start = ctl(cmd="state")["joints"]
        goal = [a + d for a, d in zip(start, (10, -5, 8, 0, 15, -20))]
        arm.set_teach_sens(90)
        s = ctl(cmd="state")
        check("writing the teach sensitivity stops a ready arm, as on the arm", s["status"] == 4
              and s["config"]["teach_sens"] == 90, (s["status"], s["config"]["teach_sens"]))
        arm.set_motion_mode(2)
        s = ctl(cmd="state")
        r = ctl(cmd="hand", joints=goal, seconds=0.3)
        check("mode 2 is ignored while stopped (both of the server's first tries on the arm), so the hand can't move it",
              s["mode"] == 0 and s["status"] == 4 and not r["ok"], (s["mode"], s["status"], r))
        arm.set_motion_status(0)
        wait_state(ctl, lambda s: s["status"] in (0, 2))
        arm.set_motion_mode(2)
        s = ctl(cmd="state")
        check("from ready, mode 2 is taken at once, and the arm reports state 1 (moving), as on the arm",
              s["mode"] == 2 and s["status"] == 1 and s["enable"] == 63, (s["mode"], s["status"]))
        r = ctl(cmd="hand", joints=goal, seconds=0.6)
        s = wait_state(ctl, lambda s: max(abs(a - b) for a, b in zip(s["joints"], goal)) < 1e-6, timeout=3)
        check("and the hand moves the arm", r["ok"] and max(abs(a - b) for a, b in zip(s["joints"], goal)) < 1e-6,
              (r, s["joints"]))
        target = [math.radians(v + 5) for v in goal]
        arm.moveto_joint_p2p(target, math.radians(10), math.radians(100), 0)
        time.sleep(0.3)
        check("motion commands are refused in freedrive", "discarded" in open(log).read().splitlines()[-1]
              and not ctl(cmd="state")["moving"])
        arm.set_motion_status(4)
        s = ctl(cmd="state")
        r = ctl(cmd="hand", joints=start, seconds=0.3)
        check("STOP holds the arm rigid, still in mode 2 with the motors on, as on the arm", s["status"] == 4
              and s["mode"] == 2 and s["enable"] == 63 and not r["ok"], (s["status"], s["mode"], s["enable"], r))
        before = ctl(cmd="state")["joints"]
        arm.set_motion_mode(0)
        s = ctl(cmd="state")
        check("leaving mode 2 switches the motors off, and J3 settles 0.45 deg onto its brake (0.43 on the arm)",
              s["mode"] == 0 and s["enable"] == 0 and s["status"] == 4 and abs(s["joints"][2] - before[2] - 0.45) < 1e-6,
              (s["mode"], s["enable"], s["joints"][2] - before[2]))
        enabled(arm)
        s = wait_state(ctl, lambda s: s["status"] in (0, 2))
        check("Enable's sequence makes it ready in position mode again", s["mode"] == 0 and s["enable"] == 63
              and s["status"] in (0, 2), (s["mode"], s["enable"], s["status"]))
        arm.set_motion_mode(2)
        far = list(ctl(cmd="state")["joints"])
        far[3] = 400.0
        ctl(cmd="hand", joints=far, seconds=0.3)
        s = wait_state(ctl, lambda s: s["err"] == 23, timeout=3)
        check("wrist 1 pushed past its limit by hand: error 23 with the motors off, out of freedrive (as at 358 deg)",
              s["err"] == 23 and s["enable"] == 0 and s["mode"] == 0 and abs(s["joints"][3] - 358.85) < 0.01,
              (s["err"], s["enable"], s["mode"], s["joints"][3]))
        arm.set_motion_mode(2)
        r = ctl(cmd="hand", joints=start, seconds=0.3)
        check("and with the error flagged freedrive holds: mode 2 is taken, the arm isn't free",
              ctl(cmd="state")["mode"] == 2 and not r["ok"], r)
        arm.set_motion_mode(0)
        enabled(arm)
        wait_state(ctl, lambda s: s["status"] in (0, 2))
        arm.set_motion_enable(4, 0)
        check("joints are switched by number, from 1: (4, off) switches off just J4, as on the arm",
              ctl(cmd="state")["enable"] == 0b110111, ctl(cmd="state")["enable"])
        enabled(arm)
        home = list(ctl(cmd="state")["joints"])
        home[3] = 0.0
        arm.moveto_joint_p2p([math.radians(v) for v in home], math.radians(100), math.radians(200), 0)
        wait_state(ctl, lambda s: not s["moving"] and s["status"] == 2, timeout=10)

        q = ctl(cmd="state")["joints"]
        target = [math.radians(v) for v in q]
        target[0] += math.radians(20)
        arm.moveto_joint_p2p(target, math.radians(15), math.radians(100), 0)
        wait_state(ctl, lambda s: s["moving"] and s["status"] == 1, timeout=2)
        time.sleep(0.3)
        r1 = ctl(cmd="contact", torque=15.0)
        r2 = ctl(cmd="contact", torque=25.0)
        s = ctl(cmd="state")
        check("collision sensitivity 3: a 15 bump passes, a 25 bump trips error 31 and stops", not r1["tripped"]
              and r2["tripped"] and s["err"] == 31 and s["status"] == 4 and not s["moving"], (r1, r2, s["err"]))
    finally:
        if arm is not None:
            arm.socket_fp.close()
        stop(sim)


def live_morning_tools():
    print("== the morning test plan's tools against the simulator: tau_check record, sensitivity")
    sim, _ = start_sim(("--joints", "36.111,28.528,138.784,108.432,45.765,64.622"), "sim_morning.log")
    try:
        import subprocess
        import tempfile
        capture = os.path.join(TOOLS, "capture")
        folder = tempfile.mkdtemp(prefix="morning-")

        def tool(*args):
            r = subprocess.run([sys.executable, *args], capture_output=True, text=True, timeout=60)
            return r.returncode, r.stdout + r.stderr

        code, text = tool(os.path.join(capture, "sensitivity.py"), "--ip", HOST)
        check("sensitivity.py reads collision 0 and teach 0, as captured", code == 0
              and "collision sensitivity 0, teach sensitivity 0" in text, text)
        code, text = tool(os.path.join(capture, "sensitivity.py"), "--ip", HOST, "--collision", "4", "--teach", "2")
        check("sets collision 4 and teach 2 and reads them back", code == 0
              and "After:  collision sensitivity 4, teach sensitivity 2" in text, text)
        code, text = tool(os.path.join(capture, "sensitivity.py"), "--ip", HOST, "--collision", "6")
        check("refuses a level outside 0 to 5 without connecting", code != 0 and "0 to 5" in text, text)
        path = os.path.join(folder, "rest.json")
        code, text = tool(os.path.join(capture, "tau_check.py"), "record", path, "--ip", HOST, "--seconds", "1.5")
        with open(path, encoding="utf-8") as f:
            rec = json.load(f)
        check("tau_check record: 13 to 17 frames in 1.5 s (10 Hz), the start pose, the simulator's rest tau",
              code == 0 and 13 <= rec["frames"] <= 17
              and max(abs(a - b) for a, b in zip(rec["joints"], (36.111, 28.528, 138.784, 108.432, 45.765, 64.622))) < 0.01
              and max(abs(a - b) for a, b in zip(rec["tau"], utra_sim.REST_TAU)) < 0.05, (code, text, rec))
        check("and the controller's settings from the config stream", rec.get("config", {}).get("collisionSensitivity") == 4
              and rec["config"]["teachSensitivity"] == 2, rec.get("config"))
    finally:
        stop(sim)


def main():
    rows, config, commands = fit_sim.load(CAPTURE_DIR)
    replay_joint_moves(rows, config, commands)
    replay_lines(rows, config, commands)
    replay_stop(rows, commands)
    replay_enable(rows, commands)
    for scenario in (live_streams, live_motion, live_lines_and_pause, live_registers, live_morning_tools):
        try:
            scenario()
        except Exception as e:  # report and carry on
            import traceback
            traceback.print_exc()
            check(f"{scenario.__name__} crashed", False, repr(e))
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    sys.stdout.flush()
    os._exit(1 if failed else 0)  # the SDK's receive threads would keep the process alive


if __name__ == "__main__":
    main()
