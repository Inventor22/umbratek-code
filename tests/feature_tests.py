"""The featureset on the running server against the simulator: the cell's API, calibration by touching points, moves
to targets (joint, straight, with offsets) checked against keep-out zones, the program runner's controls, faults and
event log, and freedrive teaching. Where the arm ends up is judged independently: the simulator's own joint angles
(its control port) through Pinocchio's kinematics. Runs on the oracle environment's Python; the server under test
runs on UTRA_TEST_PY. Usage: python tests/feature_tests.py"""
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pinocchio as pin

from simrig import HOST, HTTP_PORT, PY, ROOT, SCRIPTS, SERVER, log_path, start_sim, stop, wait_port

sys.path.insert(0, SCRIPTS)
import kinematics as kin  # noqa: E402
import utra_model  # noqa: E402

BASE = f"http://127.0.0.1:{HTTP_PORT}"
URDF = os.path.join(ROOT, "scripts", "model", "utra6_1000", "utra6_1000_robot.urdf")
PMODEL = pin.buildModelFromUrdf(URDF)
PDATA = PMODEL.createData()
FLANGE = PMODEL.getFrameId("link_6")
MODEL = utra_model.ArmModel()
KIN = kin.Kinematics(MODEL)
LIMITS = [(-180, 180)] * 2 + [(-165, 165)] + [(-180, 180)] * 3
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)[:600]}", flush=True)


def pin_flange(q_deg):
    """Pinocchio's flange pose (4x4 numpy, metres) for joints in degrees: the independent judge."""
    pin.framesForwardKinematics(PMODEL, PDATA, np.radians(q_deg))
    return PDATA.oMf[FLANGE].homogeneous.copy()


class Rig:
    def __init__(self, name, sim_args=(), cell_dir=None, keep=False):
        self.name = name
        self.cell_dir = cell_dir or log_path(f"featcell_{name}")
        if not keep:
            shutil.rmtree(self.cell_dir, ignore_errors=True)
        self.sim, self.sim_log = start_sim(sim_args, f"feature_{name}.log")
        self.start_server()
        self.ctl_sock = socket.create_connection((HOST, 30099), timeout=5)
        self.ctl_buf = b""
        self.polling = False

    def start_server(self):
        self.server = subprocess.Popen([PY, SERVER, HOST, "--port", str(HTTP_PORT), "--cell", self.cell_dir],
                                       cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        assert wait_port(HTTP_PORT, 15, host="127.0.0.1"), "server did not start"
        html = urllib.request.urlopen(BASE + "/", timeout=10).read().decode()
        self.token = re.search(r'const TOKEN = "([^"]+)"', html).group(1)

    def stop_server(self):
        self.server.kill()
        self.server.wait()

    def ctl(self, **request):
        self.ctl_sock.sendall(json.dumps(request).encode() + b"\n")
        while b"\n" not in self.ctl_buf:
            self.ctl_buf += self.ctl_sock.recv(65536)
        line, self.ctl_buf = self.ctl_buf.split(b"\n", 1)
        return json.loads(line)

    def get(self, path):
        with urllib.request.urlopen(BASE + path, timeout=30) as r:
            return json.loads(r.read())

    def post(self, path, body=None):
        req = urllib.request.Request(BASE + path, data=json.dumps(body or {}).encode(), method="POST",
                                     headers={"Content-Type": "application/json", "X-Panel-Token": self.token})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return {"ok": False, "http": e.code}

    def status(self):
        return self.get("/api/status")

    def wait(self, pred, timeout=20.0, poll=0.1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.status()
            if pred(s):
                return s
            time.sleep(poll)
        return self.status()

    def log(self):
        return open(self.sim_log).read()

    def enable(self):
        self.wait(lambda s: s.get("live"))
        r = self.post("/api/enable")
        self.wait(lambda s: s.get("enabled") and s.get("state") in (0, 2))
        return r

    def run_state(self, *states, timeout=30.0):
        return self.wait(lambda s: s.get("run", {}).get("state") in states, timeout)

    def sim_joints(self):
        return self.ctl(cmd="state")["joints"]

    def close(self):
        try:
            self.ctl_sock.close()
        finally:
            self.stop_server()
            stop(self.sim)


def tool_target_seed(rig, pose_in_frame, frame, tool, seed):
    """Joints reaching a tool pose in a frame (from our IK; the test judges with Pinocchio later)."""
    cell = rig.get("/api/cell")
    frame_t = kin.pose_to_matrix(cell["framesInBase"][frame]) if frame != "base" else kin.IDENTITY
    target = kin.mul(frame_t, kin.pose_to_matrix(pose_in_frame))
    r = KIN.solve(seed, target, LIMITS, kin.pose_to_matrix(tool))
    assert r["ok"], r
    return r["joints"]


def goto_joints(rig, joints, name="tmp_joints"):
    rev = rig.get("/api/cell")["cell"]["revision"]
    rig.post("/api/cell/target", {"name": name, "joints": [round(v, 5) for v in joints], "revision": rev})
    r = rig.post("/api/moveto", {"target": name, "kind": "joint", "speed": 60})
    assert r["ok"], r
    s = rig.run_state("done", "faulted", "stopped")
    assert s["run"]["state"] == "done", s["run"]
    return s


# ---- scenarios -----------------------------------------------------------------------------------------------------

def scenario_cell_api():
    print("== the cell: tools, frames, targets and zones through the API, saved and reloaded")
    rig = Rig("cell")
    try:
        c = rig.get("/api/cell")
        check("a new cell has the bare flange as its tool", c["ok"] and c["cell"]["active"]["tool"] == "flange"
              and c["cell"]["revision"] == 0, c["cell"])
        rev = c["cell"]["revision"]
        r = rig.post("/api/cell/tool", {"name": "gripper", "tcp": {"xyz": [0, 5, 135], "rpy": [0, 0, 90]}, "mass": 0.8,
                                        "cog": [0, 0, 45], "capsules": [{"a": [0, 0, 0], "b": [0, 0, 110], "r": 35}],
                                        "revision": rev})
        check("a tool with its payload and shape is saved", r["ok"] and "gripper" in r["cell"]["tools"], r)
        r = rig.post("/api/cell/tool/activate", {"name": "gripper", "revision": r["cell"]["revision"]})
        cfg = rig.ctl(cmd="state")["config"]
        check("activating it sends its payload to the controller (read back from the simulator)",
              r["ok"] and abs(cfg["payload"][0] - 0.8) < 1e-6 and abs(cfg["payload"][3] - 45) < 1e-4
              and cfg["tcp_offset"] == [0.0] * 6, (r.get("message"), cfg["payload"], cfg["tcp_offset"]))
        check("the collision model the page gets includes the tool's capsule, labelled",
              any(cap.get("label") == "the tool" for cap in r["collision"]["capsules"]), len(r["collision"]["capsules"]))
        rev = r["cell"]["revision"]
        r = rig.post("/api/cell/frame", {"name": "fixture", "parent": "base",
                                         "pose": {"xyz": [400, 100, 50], "rpy": [0, 0, 20]}, "revision": rev})
        r = rig.post("/api/cell/target", {"name": "above", "frame": "fixture", "pose": {"xyz": [0, 0, 150],
                                                                                         "rpy": [180, 0, 0]},
                                          "revision": r["cell"]["revision"]})
        check("a frame and a target in it are saved, and the target is placed in the base frame",
              r["ok"] and r["targetsInBase"]["above"]["xyz"][2] > 150, r.get("targetsInBase"))
        stale = rig.post("/api/cell/zone", {"name": "box", "frame": "fixture", "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
                                            "size": [100, 100, 100], "revision": rev})
        check("an edit made against an old revision is refused", not stale["ok"] and "changed since" in stale["message"],
              stale)
        r = rig.post("/api/cell/zone", {"name": "box", "frame": "fixture", "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
                                        "size": [100, 100, 100], "revision": r["cell"]["revision"]})
        check("a zone is saved, and appears as an obstacle in the page's collision model",
              r["ok"] and any(o["label"] == "the box zone" for o in r["collision"]["obstacles"]), r.get("message"))
        before = rig.get("/api/cell")["cell"]
        bad = rig.post("/api/cell/frame/delete", {"name": "fixture", "revision": before["revision"]})
        check("a frame in use can't be deleted", not bad["ok"] and "still used" in bad["message"], bad)
        rig.stop_server()
        rig.start_server()
        after = rig.get("/api/cell")["cell"]
        check("after a server restart the cell is exactly as saved", after == before)
    finally:
        rig.close()


def scenario_calibration():
    print("== calibrating a frame and a tool by touching points, through the API")
    rig = Rig("calib")
    try:
        rig.enable()
        truth = {"xyz": [420.0, -120.0, 80.0], "rpy": [2.0, -1.5, 25.0]}
        T = kin.pose_to_matrix(truth)
        seed = [-10, 10, 100, 0, 70, 0]
        down = {"xyz": [0, 0, 0], "rpy": [180, 0, 0]}
        for which, local in (("origin", [0, 0, 0]), ("x", [150, 0, 0]), ("xy", [40, 120, 0])):
            pose = dict(down, xyz=local)
            q = KIN.solve(seed, kin.mul(T, kin.pose_to_matrix(pose)), LIMITS)["joints"]
            goto_joints(rig, q)
            r = rig.post("/api/cell/frame/touch", {"name": "table", "parent": "base", "which": which})
            check(f"touched the table's {which}", r["ok"], r)
        r = rig.post("/api/cell/frame/calibrate", {"name": "table"})
        got = kin.pose_to_matrix(r["cell"]["frames"]["table"]["pose"]) if r["ok"] else None
        err = kin.pose_distance(got, T) if got else None
        check("the calibrated frame is the true one within 0.05 mm and 0.01 deg", r["ok"] and err[0] < 0.05 and err[1] < 0.01,
              (r.get("message"), err))

        tool_true = [15.0, -10.0, 140.0]
        fixed = [450.0, 50.0, 150.0]
        for i, rpy in enumerate(([180, 0, 0], [150, 20, 0], [200, -15, 30], [170, 25, -40], [190, 10, 60])):
            r_flange = kin.rpy_matrix(*rpy)
            p = [fixed[k] - sum(r_flange[k][m] * tool_true[m] for m in range(3)) for k in range(3)]
            q = KIN.solve(seed, kin.transform(r_flange, p), LIMITS)
            assert q["ok"], q
            goto_joints(rig, q["joints"])
            rig.post("/api/cell/tool/touch", {"name": "pointer"})
        r = rig.post("/api/cell/tool/calibrate", {"name": "pointer"})
        got = r["cell"]["tools"]["pointer"]["tcp"]["xyz"] if r["ok"] else None
        check("the pointer's tool point from five touches is the true one within 0.05 mm",
              r["ok"] and math.dist(got, tool_true) < 0.05, (r.get("message"), got))
    finally:
        rig.close()


def setup_cell(rig, zone=False):
    """A pointer tool (100 mm), a table frame, targets A, B (and C beyond a zone)."""
    rev = rig.get("/api/cell")["cell"]["revision"]
    tool = {"xyz": [0, 0, 100], "rpy": [0, 0, 0]}
    rig.post("/api/cell/tool", {"name": "pointer", "tcp": tool, "mass": 0.2, "cog": [0, 0, 50],
                                "capsules": [{"a": [0, 0, 0], "b": [0, 0, 90], "r": 12}], "revision": rev})
    rev = rig.post("/api/cell/tool/activate", {"name": "pointer"})["cell"]["revision"]
    rev = rig.post("/api/cell/frame", {"name": "table", "parent": "base",
                                       "pose": {"xyz": [380, -100, 100], "rpy": [0, 0, 15]}, "revision": rev})["cell"]["revision"]
    seed = [-10, 10, 100, 0, 70, 0]
    points = {"A": [0, 0, 120], "B": [120, 80, 60], "C": [0, 260, 120]}
    for name, xyz in points.items():
        pose = {"xyz": xyz, "rpy": [180, 0, 0]}
        q = tool_target_seed(rig, pose, "table", tool, seed)
        r = rig.post("/api/cell/target", {"name": name, "frame": "table", "pose": pose, "seed": q, "revision": rev})
        rev = r["cell"]["revision"]
    if zone:
        r = rig.post("/api/cell/zone", {"name": "post", "frame": "table", "pose": {"xyz": [0, 130, 100], "rpy": [0, 0, 0]},
                                        "size": [60, 60, 200], "revision": rev})
        rev = r["cell"]["revision"]
    rig.post("/api/cell/target", {"name": "home", "joints": [0, 0, 90, 0, 90, 0], "revision": rev})
    return tool


def tool_point_in(rig, frame_pose, tool, joints):
    """Where Pinocchio puts the tool point for these joints, in the frame (mm), and its rotation."""
    T = np.linalg.inv(np.array(kin.pose_to_matrix(frame_pose))) @ pin_flange(joints) @ np.array(kin.pose_to_matrix(tool))
    return T[:3, 3] * 1000.0, T[:3, :3]


def sample_while(rig, until, period=0.02):
    """The simulator's joints sampled until until() is true."""
    out = []
    while not until():
        out.append(rig.sim_joints())
        time.sleep(period)
    return out


def scenario_moves():
    print("== moves to targets: joint, straight and offset moves; refusals for reach and keep-out zones")
    rig = Rig("moves")
    try:
        rig.enable()
        tool = setup_cell(rig)
        frame_pose = rig.get("/api/cell")["cell"]["frames"]["table"]["pose"]
        r = rig.post("/api/moveto", {"target": "A", "kind": "joint", "speed": 40})
        s = rig.run_state("done", "faulted", "stopped")
        p, R = tool_point_in(rig, frame_pose, tool, rig.sim_joints())
        check("a joint move to target A ends with the tool point on it within 0.1 mm (Pinocchio)",
              r["ok"] and s["run"]["state"] == "done" and np.linalg.norm(p - np.array([0, 0, 120])) < 0.1, (s["run"], p))

        r = rig.post("/api/plan", {"target": "B", "kind": "line", "speed": 60})
        check("a straight line to B is planned before moving", r["ok"] and r["plan"]["kind"] == "line", r)
        done = threading.Event()
        rig.post("/api/moveto", {"target": "B", "kind": "line", "speed": 60})
        poll = threading.Thread(target=lambda: (rig.run_state("done", "faulted", "stopped"), done.set()))
        poll.start()
        samples = sample_while(rig, done.is_set)
        poll.join()
        a, b = np.array([0, 0, 120.0]), np.array([120, 80, 60.0])
        u = (b - a) / np.linalg.norm(b - a)
        worst = max(np.linalg.norm((tool_point_in(rig, frame_pose, tool, q)[0] - a) -
                                   np.dot(tool_point_in(rig, frame_pose, tool, q)[0] - a, u) * u) for q in samples)
        end, _ = tool_point_in(rig, frame_pose, tool, rig.sim_joints())
        check(f"the line to B, sampled {len(samples)} times as it ran, stays within 0.1 mm of the line (Pinocchio)",
              worst < 0.1 and len(samples) > 20, worst)
        check("and ends on B within 0.1 mm", np.linalg.norm(end - b) < 0.1, end)

        rig.post("/api/moveto", {"target": "A", "kind": "line", "speed": 80, "offset": {"frame": "tool", "xyz": [0, 0, -50]}})
        rig.run_state("done", "faulted", "stopped")
        p, R = tool_point_in(rig, frame_pose, tool, rig.sim_joints())
        # The tool points down (roll 180), so 50 mm back along its own z is 50 mm up in the table's frame.
        check("an approach 50 mm back along the tool from A ends 50.00 mm above it", abs(np.linalg.norm(p - [0, 0, 120]) - 50) < 0.05
              and abs(p[2] - 170) < 0.05, p)

        rev = rig.get("/api/cell")["cell"]["revision"]
        rig.post("/api/cell/zone", {"name": "post", "frame": "table", "pose": {"xyz": [0, 130, 100], "rpy": [0, 0, 0]},
                                    "size": [60, 60, 200], "revision": rev})
        writes = rig.log().count("MOVET_LINE:")
        r = rig.post("/api/moveto", {"target": "C", "kind": "line", "speed": 60})
        s = rig.run_state("done", "faulted", "stopped")
        check("a line through the keep-out zone is refused before anything moves, naming the zone",
              s["run"]["state"] == "faulted" and "post zone" in s["run"]["reason"] and rig.log().count("MOVET_LINE:") == writes,
              s["run"])
        hit = s["run"].get("hit") or {}
        obstacles = rig.get("/api/cell")["collision"]["obstacles"]
        box = obstacles[hit["obstacle"]] if isinstance(hit.get("obstacle"), int) else {}

        def on_box(p):
            d = np.array(p) - np.array(box["centre"])
            return all(abs(float(d @ np.array(axis))) <= half + 1e-6 for axis, half in zip(box["axes"], box["half"]))

        check("the faulted run reports where, for the 3D view: the zone's index in the page's collision model, a "
              "contact point on the zone's box, and the pose", box.get("label") == "the post zone"
              and any(on_box(p) for p in hit.get("points", [])) and len(hit.get("pose", [])) == 6, hit)
        rig.post("/api/run/reset")
        check("and Reset clears it", rig.status()["run"].get("hit") is None, rig.status()["run"])
        rig.enable()
        rev = rig.get("/api/cell")["cell"]["revision"]
        rev = rig.post("/api/cell/zone/delete", {"name": "post", "revision": rev})["cell"]["revision"]
        rig.post("/api/cell/target", {"name": "far", "frame": "table", "pose": {"xyz": [1500, 0, 0], "rpy": [180, 0, 0]},
                                      "revision": rev})
        r = rig.post("/api/plan", {"target": "far", "kind": "line", "speed": 60})
        check("a target out of reach is refused at planning, saying so", not r["ok"] and "reach" in r["message"], r)
        r = rig.post("/api/plan", {"target": "far", "kind": "joint", "speed": 60})
        check("as a joint move too", not r["ok"] and "reach" in r["message"], r)
    finally:
        rig.close()


def save_program(rig, program):
    r = rig.post("/api/program", {"program": program})
    assert r["ok"], r
    return r["program"]


def scenario_runner():
    print("== the program runner: a full program with I/O, halts, calls and repeats; its log")
    rig = Rig("runner")
    try:
        rig.enable()
        setup_cell(rig)
        save_program(rig, {"name": "sub", "steps": [{"type": "move", "kind": "joint", "target": "home"}]})
        save_program(rig, {"name": "cycle", "speed": {"tcp": 100, "joint": 40}, "steps": [
            {"type": "move", "kind": "joint", "target": "A"},
            {"type": "set", "output": "controller.1", "value": 1},
            {"type": "wait", "input": "controller.2", "value": 1, "timeout": 10},
            {"type": "pulse", "output": "tool.0", "seconds": 0.3},
            {"type": "if", "input": "tool.3", "value": 1, "then": [{"type": "move", "kind": "line", "target": "B"}],
             "else": [{"type": "comment", "text": "no part"}]},
            {"type": "repeat", "count": 2, "steps": [
                {"type": "move", "kind": "line", "target": "A", "offset": {"frame": "tool", "xyz": [0, 0, -30]}},
                {"type": "move", "kind": "line", "target": "A"}]},
            {"type": "call", "program": "sub"},
            {"type": "halt", "message": "check the part"},
            {"type": "set", "output": "controller.1", "value": 0}]})
        rig.ctl(cmd="input", line="tool", bit=3, value=1)
        r = rig.post("/api/run/start", {"name": "cycle"})
        s = rig.wait(lambda s: (s["run"].get("lastEvent") or {}).get("event") == "io.wait", 20)
        out = rig.ctl(cmd="state")["outputs"]
        check("the run sets controller.1 and waits for controller.2", r["ok"] and out["controller"] == 2, (r, out))
        time.sleep(1.0)
        check("still waiting a second later", rig.status()["run"]["state"] == "running")
        rig.ctl(cmd="input", line="controller", bit=2, value=1)
        s = rig.run_state("paused", "faulted", "done", timeout=60)
        check("the input arrives; the run carries on to the halt, which pauses it with its message",
              s["run"]["state"] == "paused" and s["run"]["halt"] == "check the part", s["run"])
        r = rig.post("/api/run/resume")
        s = rig.run_state("done", "faulted", "stopped", timeout=30)
        out = rig.ctl(cmd="state")["outputs"]
        check("resumed, it runs to the end and clears controller.1", s["run"]["state"] == "done" and out["controller"] == 0,
              (s["run"], out))
        run = rig.get("/api/run")["run"]
        events = [e["event"] for e in run["events"]]
        log_file = os.path.join(rig.cell_dir, "logs", run["log"])
        lines = [json.loads(line) for line in open(log_file)]
        check("the event log on disk has the whole run, ending in run.done", lines[0]["event"] == "run.start"
              and lines[-1]["event"] == "run.done" and len(lines) >= len(events), [e["event"] for e in lines][-5:])
        kinds = [(e.get("path"), e["event"]) for e in lines if e["event"] in ("step.done", "if", "repeat", "call")]
        check("with the if taking 'then' (tool.3 was 1), two repeats, the call, and every step done in order",
              any(e["event"] == "if" and e["branch"] == "then" for e in lines)
              and sum(1 for e in lines if e["event"] == "repeat") == 2 and any(e["event"] == "call" for e in lines)
              and sum(1 for e in lines if e["event"] == "step.done") == 12, kinds)
        check("the pulse set tool.0 then cleared it", [(e["output"], e["value"]) for e in lines if e["event"] == "io.set"
                                                         and e["output"] == "tool.0"] == [("tool.0", 1), ("tool.0", 0)])
    finally:
        rig.close()


def scenario_controls():
    print("== the runner's controls: pause and resume mid-line, step, stop, override, the reduced-speed cap, dry run")
    rig = Rig("controls")
    try:
        rig.enable()
        tool = setup_cell(rig)
        frame_pose = rig.get("/api/cell")["cell"]["frames"]["table"]["pose"]
        save_program(rig, {"name": "line", "steps": [{"type": "move", "kind": "joint", "target": "A"},
                                                      {"type": "move", "kind": "line", "target": "B", "speed": 25}]})
        rig.post("/api/run/start", {"name": "line"})
        rig.wait(lambda s: (s["run"].get("path") or [None])[0] == 1 and s.get("state") == 1, 30, 0.05)
        time.sleep(1.5)
        r = rig.post("/api/run/pause")
        s = rig.run_state("paused", timeout=10)
        time.sleep(0.6)  # past the STOP's settling transient (0.5 s on the arm, as captured)
        held = rig.sim_joints()
        time.sleep(0.8)
        check("pause mid-line: the run pauses and the arm stops (controller state 4) and stays put",
              r["ok"] and s["run"]["state"] == "paused" and rig.ctl(cmd="state")["status"] == 4
              and max(abs(a - b) for a, b in zip(held, rig.sim_joints())) < 1e-6, s["run"])
        a, b = np.array([0, 0, 120.0]), np.array([120, 80, 60.0])
        p, _ = tool_point_in(rig, frame_pose, tool, held)
        u = (b - a) / np.linalg.norm(b - a)
        off = np.linalg.norm((p - a) - np.dot(p - a, u) * u)
        check("where it stopped is still on the line (within 0.1 mm)", off < 0.1, off)
        done = threading.Event()
        rig.post("/api/run/resume")
        poll = threading.Thread(target=lambda: (rig.run_state("done", "faulted", "stopped"), done.set()))
        poll.start()
        samples = sample_while(rig, done.is_set)
        poll.join()
        dev = [np.linalg.norm((tool_point_in(rig, frame_pose, tool, q)[0] - a)
                              - np.dot(tool_point_in(rig, frame_pose, tool, q)[0] - a, u) * u) for q in samples]
        end, _ = tool_point_in(rig, frame_pose, tool, rig.sim_joints())
        regain = [e for e in rig.get("/api/run")["run"]["events"] if e["event"] == "move.regain"]
        check("resume re-enables the motors, which shifts the arm (0.46 mm on the real arm), then first moves back "
              "onto the line (regain)", rig.status()["run"]["state"] == "done" and len(regain) == 1
              and 0.2 < regain[0]["offMm"] < 0.8 and max(dev) < 0.8, (regain, max(dev)))
        tail = dev[len(dev) // 3:]
        check("and finishes the line on the line (within 0.1 mm), at B", max(tail) < 0.1 and np.linalg.norm(end - b) < 0.1,
              (max(tail), end))

        rig.post("/api/run/start", {"name": "line", "mode": "step"})
        s = rig.run_state("paused", "faulted", timeout=30)
        first = s["run"]["path"]
        r = rig.post("/api/run/step")
        s = rig.wait(lambda s: s["run"]["state"] in ("paused", "done", "faulted") and s["run"]["path"] != first, 30)
        check("step mode runs exactly one step, then waits", first == [0] and s["run"]["state"] == "paused"
              and s["run"]["path"] == [1], (first, s["run"]))
        rig.post("/api/run/stop")
        s = rig.run_state("stopped", timeout=10)
        check("stop ends a paused run", s["run"]["state"] == "stopped", s["run"])
        rig.enable()

        save_program(rig, {"name": "fast", "steps": [{"type": "move", "kind": "line", "target": "A", "speed": 500},
                                                      {"type": "move", "kind": "line", "target": "B", "speed": 100}]})
        rig.post("/api/run/override", {"percent": 50})
        rig.post("/api/run/start", {"name": "fast"})
        rig.run_state("done", "faulted", "stopped", timeout=60)
        speeds = [float(v) for v in re.findall(r"MOVET_LINE: goal=\[[^\]]*\] speed=([\d.]+)", rig.log())][-2:]
        check("override 50%: 500 mm/s becomes 250 (also the reduced-speed cap) and 100 becomes 50", speeds == [250.0, 50.0],
              speeds)
        rig.post("/api/run/override", {"percent": 100})
        rig.post("/api/run/start", {"name": "fast"})
        rig.run_state("done", "faulted", "stopped", timeout=60)
        speeds = [float(v) for v in re.findall(r"MOVET_LINE: goal=\[[^\]]*\] speed=([\d.]+)", rig.log())][-2:]
        check("at 100%, a program not marked for full speed is still capped at 250 mm/s", speeds == [250.0, 100.0], speeds)

        rig.post("/api/moveto", {"target": "home", "kind": "joint", "speed": 60})
        rig.run_state("done", timeout=30)
        writes_before = rig.log().count("WRITE")
        t0 = time.monotonic()
        rig.post("/api/run/start", {"name": "line", "mode": "dry"})
        s = rig.run_state("done", "faulted", timeout=30)
        estimate = rig.get("/api/run")["run"]["estimateS"]
        check("a dry run sends nothing to the controller, and estimates the run", s["run"]["state"] == "done"
              and rig.log().count("WRITE") == writes_before and estimate and estimate > 1, (s["run"], estimate))
        t0 = time.monotonic()
        rig.post("/api/run/start", {"name": "line"})
        rig.run_state("done", "faulted", timeout=60)
        actual = time.monotonic() - t0
        check(f"its estimate ({estimate:.1f} s) is within 10% of the real run's moving time ({actual:.1f} s, less 0.5 s "
              "of supervision)", abs(estimate - (actual - 0.5)) <= 0.1 * actual, (estimate, actual))
    finally:
        rig.close()


def scenario_faults():
    print("== faults: a wait that times out, a controller error mid-move, the arm knocked off its path")
    rig = Rig("faults")
    try:
        rig.enable()
        setup_cell(rig)
        save_program(rig, {"name": "waiter", "steps": [{"type": "wait", "input": "tool.5", "value": 1, "timeout": 2}]})
        t0 = time.monotonic()
        rig.post("/api/run/start", {"name": "waiter"})
        s = rig.run_state("faulted", "done", timeout=10)
        took = time.monotonic() - t0
        check("a wait with a 2 s timeout faults the run at 2.0 to 2.6 s, naming the input",
              s["run"]["state"] == "faulted" and "tool.5" in s["run"]["reason"] and 2.0 <= took <= 2.6, (s["run"], took))
        r = rig.post("/api/run/start", {"name": "waiter"})
        check("a faulted run must be reset before another can start", not r["ok"] and "Reset" in r["message"], r)
        rig.post("/api/run/reset")
        rig.enable()

        save_program(rig, {"name": "long", "steps": [{"type": "move", "kind": "joint", "target": "home", "speed": 10},
                                                      {"type": "move", "kind": "joint", "target": "A", "speed": 10}]})
        rig.post("/api/run/start", {"name": "long"})
        rig.wait(lambda s: s.get("state") == 1, 10, 0.05)
        time.sleep(0.5)
        rig.ctl(cmd="fault", err=9)
        s = rig.run_state("faulted", timeout=10)
        check("a controller error mid-move faults the run with the error", s["run"]["state"] == "faulted"
              and "error 9" in s["run"]["reason"], s["run"])
        rig.post("/api/run/reset")
        rig.enable()
        rig.post("/api/run/start", {"name": "long"})
        rig.wait(lambda s: s.get("state") == 1, 10, 0.05)
        time.sleep(0.5)
        rig.ctl(cmd="nudge", joints=[0, 3, 0, 0, 0, 0])
        s = rig.run_state("faulted", timeout=10)
        check("the arm knocked 3 deg off its path: the run stops it and faults within a frame or two",
              s["run"]["state"] == "faulted" and "left its path" in s["run"]["reason"]
              and rig.ctl(cmd="state")["status"] == 4, s["run"])
    finally:
        rig.close()


def scenario_page():
    print("== runs carry on without the page; manual moves don't; jogs wait for pauses")
    rig = Rig("page")
    try:
        rig.enable()
        setup_cell(rig)
        save_program(rig, {"name": "slow", "steps": [{"type": "move", "kind": "joint", "target": "home", "speed": 15},
                                                      {"type": "move", "kind": "joint", "target": "A", "speed": 15}]})
        rig.post("/api/run/start", {"name": "slow"})
        time.sleep(0.5)
        r = rig.post("/api/jog", {"kind": "joint", "index": 5, "dir": 1, "step": 1.0, "speed": 10.0})
        check("jogs are refused while a program runs", not r["ok"] and "program is running" in r["message"], r)
        r = rig.post("/api/move", {"targets": [None, None, None, None, None, 10.0], "speed": 10})
        check("and so are planned joint moves", not r["ok"] and "program is running" in r["message"], r)
        time.sleep(4.0)  # no page requests at all
        s = rig.run_state("done", "faulted", "stopped", timeout=60)
        check("with the page silent for 4 s, the run carries on to the end", s["run"]["state"] == "done", s["run"])

        rig.post("/api/moveto", {"target": "home", "kind": "joint", "speed": 10})
        time.sleep(3.0)  # the page goes quiet mid-move
        s = rig.run_state("done", "faulted", "stopped", timeout=10)
        check("a manual move stops when the page goes quiet", s["run"]["state"] == "stopped"
              and "went quiet" in s["run"]["reason"], s["run"])
        rig.enable()
        rig.post("/api/run/start", {"name": "slow"})
        time.sleep(0.6)
        rig.post("/api/run/pause")
        rig.run_state("paused", timeout=10)
        rig.wait(lambda s: s.get("state") == 4, 5)  # the pause's STOP shows in the status up to a frame later
        rig.enable()
        r = rig.post("/api/jog", {"kind": "joint", "index": 5, "dir": 1, "step": 2.0, "speed": 10.0})
        check("while paused, jogging is allowed", r["ok"] and r.get("moved"), r)
        rig.wait(lambda s: s.get("state") in (0, 2) and not s.get("goal"), 10)
        rig.post("/api/run/resume")
        s = rig.run_state("done", "faulted", timeout=60)
        check("and resume re-plans from wherever the jog left the arm", s["run"]["state"] == "done", s["run"])
    finally:
        rig.close()


def scenario_teach():
    print("== freedrive and teaching: guide the arm by hand, record targets into a frame and a program")
    rig = Rig("teach")
    try:
        rig.enable()
        tool = setup_cell(rig)
        save_program(rig, {"name": "taught", "steps": [{"type": "move", "kind": "joint", "target": "home"}]})
        r = rig.post("/api/freedrive", {"on": True, "sensitivity": 2})
        s = rig.wait(lambda s: s.get("freedrive"), 5)
        check("freedrive on at sensitivity 2: the controller is in motion mode 2", r["ok"] and s["freedrive"]
              and rig.ctl(cmd="state")["config"]["teach_sens"] == 2, r)
        r = rig.post("/api/jog", {"kind": "joint", "index": 5, "dir": 1, "step": 1.0, "speed": 10.0})
        check("jogs are refused in freedrive", not r["ok"] and "freedrive" in r["message"], r)
        r = rig.post("/api/run/start", {"name": "taught"})
        check("so are runs", not r["ok"] and "freedrive" in r["message"], r)
        hand = [12.0, 15.0, 95.0, -20.0, 75.0, 30.0]
        rig.ctl(cmd="hand", joints=hand, seconds=1.0)
        s = rig.wait(lambda s: max(abs(a - b) for a, b in zip(s["joints"], hand)) < 0.01, 5)
        check("the hand moves the arm, and the panel sees it", max(abs(a - b) for a, b in zip(s["joints"], hand)) < 0.01,
              s["joints"])
        rev = rig.get("/api/cell")["cell"]["revision"]
        r = rig.post("/api/cell/target", {"name": "picked", "frame": "table", "here": True, "program": "taught",
                                          "at": [0], "move": "line", "revision": rev})
        frame_pose = r["cell"]["frames"]["table"]["pose"]
        p, R = tool_point_in(rig, frame_pose, tool, rig.sim_joints())
        saved = r["cell"]["targets"]["picked"]
        got = kin.pose_to_matrix(saved["pose"])
        check("the recorded target is Pinocchio's tool point for the hand's joints, in the frame, within 0.001 mm",
              np.linalg.norm(np.array(got)[:3, 3] * 1000.0 - p) < 0.001
              and np.abs(np.array(got)[:3, :3] - R).max() < 1e-6 and saved["taught"]["how"] == "freedrive", (saved, p))
        prog = rig.get("/api/program?name=taught")["program"]
        check("and a straight move to it was added after step 1 of the program",
              [(s["type"], s.get("target"), s.get("kind")) for s in prog["steps"]] == [("move", "home", "joint"),
                                                                                       ("move", "picked", "line")],
              prog["steps"])
        r = rig.post("/api/freedrive", {"on": False})
        s = rig.wait(lambda s: not s.get("freedrive"), 5)
        check("leaving freedrive: position mode, the arm holds where the hand left it", r["ok"] and not s["freedrive"]
              and max(abs(a - b) for a, b in zip(rig.sim_joints(), hand)) < 0.01, r)
        r = rig.post("/api/run/start", {"name": "taught"})
        s = rig.run_state("done", "faulted", timeout=60)
        p, _ = tool_point_in(rig, frame_pose, tool, rig.sim_joints())
        check("the taught program runs, ending on the taught target within 0.1 mm", s["run"]["state"] == "done"
              and np.linalg.norm(p - np.array(got)[:3, 3] * 1000.0) < 0.1, (s["run"], p))
        rev = rig.get("/api/cell")["cell"]["revision"]
        r = rig.post("/api/cell/target/delete", {"name": "picked", "revision": rev})
        check("a target a program moves to can't be deleted, naming the program", not r["ok"]
              and "program taught" in r["message"] and "picked" in rig.get("/api/cell")["cell"]["targets"], r)
        rig.post("/api/freedrive", {"on": True, "sensitivity": 1})
        rig.wait(lambda s: s.get("freedrive"), 5)
        rig.post("/api/stop")
        s = rig.wait(lambda s: not s.get("freedrive"), 5)
        check("STOP ends freedrive", not s["freedrive"] and rig.ctl(cmd="state")["mode"] == 0, s.get("mode"))
    finally:
        rig.close()


def scenario_model():
    print("== the program model and the runner's states: validation, a round trip, refusals by state, the log on a kill")
    rig = Rig("model")
    try:
        rig.enable()
        setup_cell(rig)
        bad = [({"name": "bad1", "steps": [{"type": "jump"}]}, "step 1: unknown step"),
               ({"name": "bad2", "steps": [{"type": "move", "kind": "line", "target": "nowhere"}]}, "step 1 (move)"),
               ({"name": "bad3", "steps": [{"type": "repeat", "count": 2, "steps": [{"type": "wait", "seconds": -1}]}]},
                "step 1.1 (wait)"),
               ({"name": "bad4", "steps": [{"type": "set", "output": "tool.99", "value": 1}]}, "step 1 (set)"),
               ({"name": "bad5", "steps": [{"type": "move", "kind": "line", "target": "home"}]}, "joint target"),
               ({"name": "loopy", "steps": [{"type": "call", "program": "loopy"}]}, "loop back")]
        for program, words in bad:
            r = rig.post("/api/program", {"program": program})
            check(f"refuses {json.dumps(program['steps'][0])[:70]}, saying where and why", not r["ok"]
                  and words in r["message"] and program["name"] not in rig.get("/api/programs")["programs"], r)
        save_program(rig, {"name": "sub", "steps": [{"type": "comment", "text": "called"}]})
        every = {"name": "every", "speed": {"tcp": 40, "joint": 15}, "steps": [
            {"type": "move", "kind": "joint", "target": "home"},
            {"type": "move", "kind": "line", "target": "A", "offset": {"frame": "tool", "xyz": [0, 0, -30]}, "speed": 30},
            {"type": "wait", "seconds": 0.5}, {"type": "wait", "input": "tool.1", "value": 1, "timeout": 5},
            {"type": "set", "output": "controller.2", "value": 1}, {"type": "pulse", "output": "tool.0", "seconds": 0.2},
            {"type": "if", "input": "tool.3", "value": 1, "then": [{"type": "comment", "text": "yes"}],
             "else": [{"type": "halt", "message": "no"}]},
            {"type": "repeat", "count": 2, "steps": [{"type": "comment", "text": "again"}]},
            {"type": "call", "program": "sub"}, {"type": "halt", "message": "done?"}]}
        r = rig.post("/api/program", {"program": every})
        loaded = rig.get("/api/program?name=every")["program"]
        check("a program with every kind of step saves, and loads back exactly as saved (with ids added)",
              r["ok"] and loaded == r["program"] and len({s["id"] for s in loaded["steps"]}) == len(every["steps"]), loaded)
        r2 = rig.post("/api/program", {"program": loaded, "revision": loaded["revision"]})
        same = {k: v for k, v in r2.get("program", {}).items() if k != "revision"} == \
            {k: v for k, v in loaded.items() if k != "revision"}
        check("saving what was loaded changes nothing but the revision", r2["ok"] and same
              and r2["program"]["revision"] == loaded["revision"] + 1, r2)
        r3 = rig.post("/api/program", {"program": loaded, "revision": loaded["revision"]})
        check("saving against an old revision is refused", not r3["ok"] and "changed since" in r3["message"], r3)

        refusals = {"/api/run/pause": "Nothing to pause", "/api/run/resume": "Nothing to resume",
                    "/api/run/step": "Step works on a paused run", "/api/run/reset": None}
        for path, words in refusals.items():
            r = rig.post(path)
            if words:
                check(f"with no run, {path} is refused, saying why", not r["ok"] and words in r["message"], r)
        r = rig.post("/api/run/stop")
        check("and stop says there's no run", r["ok"] and "No run" in r["message"], r)
        save_program(rig, {"name": "slow", "steps": [{"type": "move", "kind": "joint", "target": "A", "speed": 6},
                                                     {"type": "move", "kind": "joint", "target": "home", "speed": 6}]})
        rig.post("/api/run/start", {"name": "slow"})
        rig.wait(lambda s: s["run"]["state"] == "running" and s.get("state") == 1, 20, 0.05)
        checks = [("/api/run/start", {"name": "slow"}, "under way"), ("/api/run/resume", {}, "Nothing to resume"),
                  ("/api/run/step", {}, "Step works on a paused run"), ("/api/run/reset", {}, "stop it first"),
                  ("/api/freedrive", {"on": True, "sensitivity": 1}, "A program is running"),
                  ("/api/enable", {}, "A program is running"),
                  ("/api/jog", {"kind": "joint", "index": 5, "dir": 1, "step": 1.0, "speed": 10.0}, "program is running")]
        for path, body, words in checks:
            r = rig.post(path, body)
            check(f"while running, {path} is refused, saying why", not r["ok"] and words in r["message"], r)
        rig.post("/api/run/pause")
        rig.run_state("paused", timeout=10)
        for path, body, words in [("/api/run/pause", {}, "Nothing to pause"), ("/api/run/start", {"name": "slow"}, "under way"),
                                  ("/api/run/reset", {}, "stop it first")]:
            r = rig.post(path, body)
            check(f"while paused, {path} is refused, saying why", not r["ok"] and words in r["message"], r)
        rig.post("/api/run/stop")
        s = rig.run_state("stopped", timeout=10)
        r = rig.post("/api/run/reset")
        check("stop ends a paused run; reset then clears it", s["run"]["state"] == "stopped" and r["ok"]
              and rig.status()["run"]["state"] == "idle", (s["run"], r))

        rig.enable()
        rig.post("/api/run/start", {"name": "slow"})
        rig.wait(lambda s: s.get("state") == 1, 20, 0.05)
        time.sleep(0.5)
        log = rig.get("/api/run")["run"]["log"]
        rig.stop_server()  # killed mid-move, as a crash would
        with open(os.path.join(rig.cell_dir, "logs", log), encoding="utf-8") as f:
            lines = f.read().splitlines()
        try:
            events = [json.loads(line)["event"] for line in lines]
        except ValueError:
            events = None
        check("the server killed mid-run leaves a log of whole lines, from run.start to the move under way",
              events is not None and events[0] == "run.start" and "move.sent" in events, lines[-3:])
        rig.ctl(cmd="state")
        rig.start_server()  # so close() can stop it
    finally:
        rig.close()


def scenario_status_slots():
    print("== the controller serves few status clients: Enable works with every other slot taken")
    rig = Rig("slots", sim_args=("--report-clients", "2"))
    held = None
    try:
        rig.wait(lambda s: s.get("live"))
        held = socket.create_connection((HOST, 30001), timeout=3)  # the second slot (the server's reader has the first)
        held.settimeout(3)
        got = held.recv(4096)
        extra = socket.create_connection((HOST, 30001), timeout=3)
        extra.settimeout(3)
        try:
            refused = extra.recv(4096) == b""
        except OSError:
            refused = True
        extra.close()
        check("with two status clients connected, the simulator closes a third at once, as the arm did", got and refused,
              (len(got), refused))
        r = rig.post("/api/enable")
        s = rig.wait(lambda s: s.get("enabled") and s.get("state") in (0, 2), 10)
        check("Enable still works: its checks use the server's own status stream, not a new connection",
              r["ok"] and s.get("enabled") and s.get("state") in (0, 2), r)
    finally:
        if held is not None:
            held.close()
        rig.close()


def main():
    print(f"server runs on {PY}; the test on {sys.executable}")
    for fn in (scenario_cell_api, scenario_calibration, scenario_moves, scenario_runner, scenario_controls,
               scenario_faults, scenario_page, scenario_teach, scenario_model, scenario_status_slots):
        try:
            fn()
        except Exception as e:  # report and carry on
            import traceback
            traceback.print_exc()
            check(f"{fn.__name__} crashed", False, repr(e))
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
