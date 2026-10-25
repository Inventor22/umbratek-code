"""The tending basics, against the simulator: grippers (open, close and grasp, confirmed by their inputs or a set time,
the grasp checked for a part and the payload switched to the tool and the part), patterns of slots and their
counters, interlocks guarding zones (before and during every move into them, jogs and the brain's paths included),
Retry and Fallback, safe home and calling the owner, and touch-up. First in-process: the cell's and the program
model's checks, slots against a grid worked out here, counters, touch-up and its log, and an old cell read unchanged.
Then the jog server against tools/utra_sim.py, whose control port plays a gripper (its inputs following its output, a
part between its jaws or not, jammed), a machine's door and the arm's position, judged from the simulator's joints. The
stub brain (brain/stub/stub_brain.py) plans the planned moves, as a separate process. Standard library only.
Usage: python tests/tending_tests.py [scenario ...]"""
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

from simrig import HOST, HTTP_PORT, PY, ROOT, SCRIPTS, SERVER, log_path, start_sim, stop, wait_port

sys.path.insert(0, SCRIPTS)
import cell as cellmod  # noqa: E402
import kinematics as kin  # noqa: E402
import planner as plannermod  # noqa: E402
import runner as runnermod  # noqa: E402
import utra_model  # noqa: E402

STUB = os.path.join(ROOT, "brain", "stub", "stub_brain.py")
BRAIN_PORT = int(os.environ.get("UTRA_TEST_BRAIN_PORT") or HTTP_PORT + 1000)  # clear of every suite's panel port
BRAIN_URL = f"http://127.0.0.1:{BRAIN_PORT}"
BASE = f"http://127.0.0.1:{HTTP_PORT}"
MODEL = utra_model.ArmModel()
KIN = kin.Kinematics(MODEL)
LIMITS = plannermod.DRAG_LIMITS_DEG
TOOL_POSE = {"xyz": [0.0, 0.0, 100.0], "rpy": [0.0, 0.0, 0.0]}
TOOL = kin.pose_to_matrix(TOOL_POSE)
CAPSULE = {"a": [0, 0, 0], "b": [0, 0, 90], "r": 12}
TOOL_MASS, TOOL_COG = 0.2, [0.0, 0.0, 50.0]
FRAME_POSE = {"xyz": [380, -100, 100], "rpy": [0, 0, 15]}
FRAME = kin.pose_to_matrix(FRAME_POSE)
SEED = [-10, 10, 100, 0, 70, 0]
DOWN = [180, 0, 0]
HOME = [0, 0, 90, 0, 90, 0]
# Tool points in the table's frame, pointing down: a working point A and one beside it, B; a tray of 3 rows x 4 columns
# (columns 40 mm apart, rows 35 mm apart) taught at three corners; a machine's opening and the point inside it.
POINTS = {"A": [0, 0, 120], "B": [100, 0, 120], "T1": [0, 0, 90], "T4": [120, 0, 90], "T9": [0, 70, 90],
          "Mout": [0, 120, 120], "M": [0, 200, 120]}
MACHINE = ({"xyz": [0, 200, 100], "rpy": [0, 0, 0]}, [100, 100, 100])  # the machine's work area, a guarded zone
PNEU = {"name": "pneu", "close": [{"output": "tool.0", "value": 1}], "open": [{"output": "tool.0", "value": 0}],
        "seconds": 0.3, "timeout": 1.0, "closed": {"input": "tool.1"}, "opened": {"input": "tool.2"},
        "part": {"input": "tool.3"}}
CHESS = {"name": "chess", "close": [{"output": "tool.5", "value": 0}], "open": [{"output": "tool.5", "value": 1}],
         "seconds": 0.8}
PART = {"mass": 0.5, "cog": [0, 0, 20]}
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)[:900]}", flush=True)


def table_pose(xyz, back=0.0):
    """base <- tool point at a point of the table (pointing down), `back` mm back along the tool."""
    t = kin.mul(FRAME, kin.pose_to_matrix({"xyz": xyz, "rpy": DOWN}))
    return kin.mul(t, kin.offset_matrix([0, 0, -back])) if back else t


def joints_for(xyz, back=0.0, seed=SEED):
    r = KIN.solve(seed, table_pose(xyz, back), LIMITS, TOOL)
    assert r["ok"], r
    return r["joints"]


def tool_mm(q):
    return kin.position(KIN.tool_pose(q, TOOL))


def refused(fn, *errors):
    """The message fn() is refused with, or None if it isn't."""
    try:
        fn()
    except errors or (cellmod.CellError, runnermod.ProgramError) as e:
        return str(e)
    return None


class Rig:
    """The simulator (with `sim_args`), optionally the stub brain, and the jog server on a fresh cell folder (or `cell`,
    kept as it is)."""

    def __init__(self, name, sim_args=(), brain=False, cell=None):
        self.name, self.brain, self.server, self.sim, self.ctl_sock = name, None, None, None, None
        self.cell_dir = cell or log_path(f"tendcell_{name}")
        if cell is None:
            shutil.rmtree(self.cell_dir, ignore_errors=True)
        self.outs = []
        try:
            self.sim, self.sim_log = start_sim(sim_args, f"tending_{name}.log")
            if brain:
                out = open(log_path(f"tending_{name}_brain.log"), "w")
                self.outs.append(out)
                self.brain = subprocess.Popen([PY, STUB, "--port", str(BRAIN_PORT)], cwd=ROOT, stdout=out,
                                              stderr=subprocess.STDOUT)
                assert wait_port(BRAIN_PORT, 15, host="127.0.0.1"), "the stub brain did not start"
            self.start_server()
            self.ctl_sock = socket.create_connection((HOST, 30099), timeout=5)
            self.ctl_buf = b""
        except BaseException:
            self.close()
            raise

    def start_server(self):
        out = open(log_path(f"tending_{self.name}_server.log"), "a")
        self.outs.append(out)
        self.server = subprocess.Popen([PY, SERVER, HOST, "--port", str(HTTP_PORT), "--cell", self.cell_dir, "--brain",
                                        BRAIN_URL if self.brain else "off"], cwd=ROOT, stdout=out,
                                       stderr=subprocess.STDOUT)
        assert wait_port(HTTP_PORT, 15, host="127.0.0.1"), "the server did not start"
        html = urllib.request.urlopen(BASE + "/", timeout=10).read().decode()
        self.token = re.search(r'const TOKEN = "([^"]+)"', html).group(1)

    def restart_server(self):
        self.server.kill()
        self.server.wait()
        self.start_server()

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

    def wait(self, pred, timeout=20.0, poll=0.05):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.status()
            if pred(s):
                return s
            time.sleep(poll)
        return self.status()

    def run_state(self, *states, timeout=30.0):
        return self.wait(lambda s: s.get("run", {}).get("state") in states, timeout)

    def enable(self):
        self.wait(lambda s: s.get("live"))
        r = self.post("/api/enable")
        self.wait(lambda s: s.get("enabled") and s.get("state") in (0, 2))
        return r

    def rev(self):
        return self.get("/api/cell")["cell"]["revision"]

    def sim_joints(self):
        return self.ctl(cmd="state")["joints"]

    def payload(self):
        """The payload the simulated controller carries: (kg, centre of gravity mm)."""
        p = self.ctl(cmd="state")["config"]["payload"]
        return p[0], p[1:4]

    def log(self):
        with open(self.sim_log) as f:
            return f.read()

    def events(self):
        return self.get("/api/run")["run"]["events"]

    def run_log(self):
        """The current run's whole event log, from its file."""
        with open(os.path.join(self.cell_dir, "logs", self.get("/api/run")["run"]["log"]), encoding="utf-8") as f:
            return [json.loads(line) for line in f]

    def run(self, name, mode="run", timeout=60.0, **extra):
        """Start a program and wait for it to end (or pause). Returns (the start's reply, the status)."""
        r = self.post("/api/run/start", dict({"name": name, "mode": mode}, **extra))
        s = self.run_state("done", "faulted", "stopped", "paused", timeout=timeout)
        return r, s

    def recover(self):
        """After a fault: Reset, and Enable."""
        self.post("/api/run/reset")
        self.enable()

    def close(self):
        if self.ctl_sock is not None:
            self.ctl_sock.close()
        for proc in (self.server, self.brain):
            if proc is not None:
                proc.kill()
                proc.wait()
        stop(self.sim)
        for out in self.outs:
            out.close()


def setup(rig, points=("A", "B"), joints=None):
    """The pointer tool (100 mm, 0.2 kg), the table's frame and targets at `points` (with the joints that reach them
    from SEED), and joint targets `joints` ({name: joints})."""
    rev = rig.rev()
    rev = rig.post("/api/cell/tool", {"name": "pointer", "tcp": TOOL_POSE, "mass": TOOL_MASS, "cog": TOOL_COG,
                                      "capsules": [CAPSULE], "revision": rev})["cell"]["revision"]
    rev = rig.post("/api/cell/tool/activate", {"name": "pointer", "revision": rev})["cell"]["revision"]
    rev = rig.post("/api/cell/frame", {"name": "table", "parent": "base", "pose": FRAME_POSE,
                                       "revision": rev})["cell"]["revision"]
    for name in points:
        r = rig.post("/api/cell/target", {"name": name, "frame": "table", "pose": {"xyz": POINTS[name], "rpy": DOWN},
                                          "seed": joints_for(POINTS[name]), "revision": rev})
        assert r["ok"], r
        rev = r["cell"]["revision"]
    for name, q in (joints or {}).items():
        rev = rig.post("/api/cell/target", {"name": name, "joints": q, "revision": rev})["cell"]["revision"]
    return rev


def save(rig, program):
    r = rig.post("/api/program", {"program": program})
    assert r["ok"], r
    return r["program"]


def goto(rig, name, kind="joint"):
    r = rig.post("/api/moveto", {"target": name, "kind": kind, "speed": 60})
    assert r["ok"], r
    s = rig.run_state("done", "faulted", "stopped")
    assert s["run"]["state"] == "done", s["run"]


def sampler(rig, period=0.02):
    """Samples the simulator's joints, with the run's step, until finished: returns finish() -> [(time, path, joints)]."""
    samples, done = [], threading.Event()

    def run():
        while not done.is_set():
            try:
                path = rig.status()["run"]["path"]
                samples.append((time.monotonic(), path, rig.sim_joints()))
            except OSError:
                pass
            time.sleep(period)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    def finish():
        done.set()
        thread.join()
        return samples
    return finish


def named(events, kind):
    return [e for e in events if e["event"] == kind]


# ---- in-process -------------------------------------------------------------------------------------------------------

def unit_cell():
    print("== the cell and the program model in-process: patterns, counters, grippers, interlocks, touch-up, old cells")
    folder = tempfile.mkdtemp(prefix="umbratek-tending-")
    try:
        cell = cellmod.Cell(folder)
        cell.set_frame("table", "base", FRAME_POSE)
        for name in ("T1", "T4", "T9"):
            cell.set_target(name, "table", {"xyz": POINTS[name], "rpy": DOWN}, tool="flange",
                            seed=joints_for(POINTS[name]))
        g = cell.set_pattern("tray", 3, 4, first="T1", row_end="T4", column_end="T9")
        check("a 3 x 4 pattern from its first slot and the ends of its first row and column: 12 slots, columns 40 mm "
              "apart, rows 35 mm apart, at 90 degrees", g["count"] == 12 and g["pitchMm"] == [40.0, 35.0, 0.0]
              and g["angleDeg"] == 90.0 and g["frame"] == "table", g)
        worst = 0.0
        order = []
        for slot in range(1, 13):
            s = cell.slot("tray", slot)
            row, column = divmod(slot - 1, 4)
            truth = [40.0 * column, 35.0 * row, 90.0]
            worst = max(worst, math.dist(s["pose"]["xyz"], truth))
            order.append(tuple(s["at"]))
            assert s["pose"]["rpy"] == DOWN
        check(f"each slot is where the grid puts it (within {worst:.1e} mm), numbered along the first row, then the "
              "next, with the first slot's orientation", worst < 0.01 and order[:5] == [(0, 0, 0), (0, 0, 1), (0, 0, 2),
                                                                                       (0, 0, 3), (0, 1, 0)], order)
        s, first, row_end = cell.slot("tray", 2), cell.target("T1")["seed"], cell.target("T4")["seed"]
        between = [a + (b - a) / 3 for a, b in zip(first, row_end)]
        check("a slot's joints to solve from are the corners' taught joints, interpolated",
              max(abs(a - b) for a, b in zip(s["seed"], between)) < 1e-9, (s["seed"], between))
        layered = cell.set_pattern("stack", 1, 1, 4, -25.0, first="T1")
        check("a stack of 4 layers, 25 mm apart downwards along the frame's z, from one taught slot",
              layered["count"] == 4 and abs(cell.slot("stack", 4)["pose"]["xyz"][2] - (90 - 75)) < 1e-9,
              cell.slot("stack", 4))
        cell.set_target("J", joints=HOME)
        cell.set_frame("other", "base", {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})
        cell.set_target("X", "other", {"xyz": [40, 0, 90], "rpy": DOWN}, tool="flange")
        cell.set_target("Near", "table", {"xyz": [0.5, 0, 90], "rpy": DOWN}, tool="flange")
        cell.set_target("Same", "table", {"xyz": [60, 0.5, 90], "rpy": DOWN}, tool="flange")
        bad = {
            "a joint target as a corner": lambda: cell.set_pattern("p", 1, 2, first="T1", row_end="J"),
            "corners in two frames": lambda: cell.set_pattern("p", 1, 2, first="T1", row_end="X"),
            "a row without its last slot": lambda: cell.set_pattern("p", 1, 4, first="T1"),
            "slots closer than 1 mm": lambda: cell.set_pattern("p", 1, 2, first="T1", row_end="Near"),
            "a row and a column running the same way": lambda: cell.set_pattern("p", 2, 2, first="T1", row_end="T4",
                                                                                column_end="Same"),
            "layers without a pitch": lambda: cell.set_pattern("p", 1, 1, 3, first="T1"),
            "no slots": lambda: cell.set_pattern("p", 0, 4, first="T1", row_end="T4"),
            "slot 13 of 12": lambda: cell.slot("tray", 13),
            "deleting a corner": lambda: cell.delete_target("T4"),
        }
        reasons = {what: refused(fn) for what, fn in bad.items()}
        check("patterns refuse a joint target as a corner, corners in two frames, a missing corner, slots under 1 mm "
              "apart, a row and column the same way, layers without a pitch, no slots, a slot past the last, and "
              "deleting a corner", all(reasons.values()), reasons)

        counters = cellmod.Counters(folder)
        counters.set("tray", 5)
        again = cellmod.Counters(folder)
        check("a pattern's counter is saved apart from the cell (cell/counters.json) and read back",
              again.slot("tray") == 5 and again.slot("other") == 1 and os.path.exists(os.path.join(folder, "counters.json"))
              and cell.revision == cellmod.Cell(folder).revision, again.snapshot())

        r = cell.revision
        gripper = cell.set_gripper("pneu", PNEU["close"], PNEU["open"], 0.3, 1.0, PNEU["closed"], PNEU["opened"],
                                   PNEU["part"])
        check("a gripper is its outputs to close and to open it, its time and timeout, and its optional inputs",
              gripper["closed"] == {"input": "tool.1", "value": 1} and gripper["part"] == {"input": "tool.3", "value": 1}
              and cell.revision == r + 1, gripper)
        bad = {
            "no outputs": lambda: cell.set_gripper("g", [], PNEU["open"]),
            "an output twice": lambda: cell.set_gripper("g", [{"output": "tool.0", "value": 1},
                                                             {"output": "tool.0", "value": 0}], PNEU["open"]),
            "tool.16": lambda: cell.set_gripper("g", [{"output": "tool.16", "value": 1}], PNEU["open"]),
            "a value of 2": lambda: cell.set_gripper("g", [{"output": "tool.1", "value": 2}], PNEU["open"]),
            "an input that isn't one": lambda: cell.set_gripper("g", PNEU["close"], PNEU["open"], closed={"input": "x"}),
        }
        reasons = {what: refused(fn) for what, fn in bad.items()}
        check("grippers refuse no outputs, an output named twice, lines past 15, values but 0 and 1, bad inputs",
              all(reasons.values()), reasons)

        cell.set_interlock("door_open", "controller.2", 1)
        cell.set_zone("machine", "table", MACHINE[0], MACHINE[1], interlocks=["door_open"])
        check("an interlock is a named input condition; a zone it guards lists it",
              cell.data["interlocks"]["door_open"] == {"input": "controller.2", "value": 1}
              and cell.guarded_zones() == {"machine": ["door_open"]}
              and [o.zone for o in cell.zone_obstacles()] == ["machine"] and cell.zone_obstacles({"machine"}) == [],
              cell.data["zones"])
        reasons = {"an unknown interlock": refused(lambda: cell.set_zone("z", "table", MACHINE[0], MACHINE[1],
                                                                          interlocks=["nope"])),
                   "deleting one a zone uses": refused(lambda: cell.delete_interlock("door_open"))}
        check("a zone can't name an interlock that isn't there, and an interlock guarding a zone can't be deleted",
              all(reasons.values()), reasons)

        old = cell.target("T1")
        entry, change = cell.touch_up("T1", pose={"xyz": [1.0, -2.0, 92.0], "rpy": [180, 0, 1.0]},
                                      seed=[v + 0.5 for v in old["seed"]])
        check("touching up a target keeps its frame and tool, and says how far it moved and turned and how its "
              "joints changed", entry["frame"] == "table" and entry["tool"] == "flange"
              and abs(change["movedMm"] - 3.0) < 1e-9 and abs(change["turnedDeg"] - 1.0) < 1e-6
              and change["deltaMm"] == [1.0, -2.0, 2.0] and change["jointsMaxDeg"] == 0.5, change)
        check("the old value is kept in the target, for Undo", entry["previous"] == old, entry.get("previous"))
        cell.undo_touch_up("T1")
        with open(os.path.join(folder, "logs", "targets.jsonl"), encoding="utf-8") as f:
            logged = [json.loads(line) for line in f]
        check("Undo puts the old value back, and the targets' log has every change with its old and new values",
              cell.target("T1") == old and [e["event"] for e in logged[-2:]] == ["touched-up", "undone"]
              and logged[-2]["old"] == old and logged[-2]["new"]["pose"]["xyz"] == [1.0, -2.0, 92.0]
              and logged[-1]["new"] == old and logged[0]["event"] == "taught", [e["event"] for e in logged])
        check("a target never touched up has nothing to undo", refused(lambda: cell.undo_touch_up("T1")))

        programs = [
            ({"steps": [{"type": "gripper", "gripper": "nope", "action": "open"}]}, "no such gripper"),
            ({"steps": [{"type": "gripper", "gripper": "pneu", "action": "squeeze"}]}, "an unknown action"),
            ({"steps": [{"type": "gripper", "gripper": "pneu", "action": "open", "part": PART}]}, "a part on opening"),
            ({"steps": [{"type": "gripper", "gripper": "pneu", "action": "grasp", "part": {"mass": 11}}]}, "11 kg"),
            ({"steps": [{"type": "count", "pattern": "nope"}]}, "no such pattern"),
            ({"steps": [{"type": "count", "pattern": "tray", "to": 0}]}, "counting to 0"),
            ({"steps": [{"type": "retry", "tries": 0, "steps": []}]}, "no tries"),
            ({"steps": [{"type": "home", "target": "nope"}]}, "safe home to no target"),
            ({"steps": [{"type": "home", "target": "J", "retract": 600}]}, "a 600 mm retract"),
            ({"steps": [{"type": "move", "kind": "line", "target": "T1", "pattern": "tray"}]}, "a target and a pattern"),
            ({"steps": [{"type": "move", "kind": "line", "pattern": "tray", "slot": 2.5}]}, "slot 2.5"),
        ]
        reasons = {what: refused(lambda p=p: runnermod.check_program(dict(p, name="p"), cell)) for p, what in programs}
        check("programs refuse an unknown gripper or action, a part on anything but a grasp, an 11 kg part, an unknown "
              "pattern, counting to 0, no tries, safe home to no target or 600 mm back, a move to a target and a "
              "pattern, and a slot that isn't whole", all(reasons.values()), reasons)
        p = runnermod.check_program({"name": "p", "steps": [
            {"type": "fallback", "steps": [{"type": "gripper", "gripper": "pneu", "action": "grasp", "part": PART}],
             "else": [{"type": "notify", "message": "No part"}, {"type": "home", "target": "J", "retract": 40}]},
            {"type": "retry", "tries": 3, "steps": [{"type": "move", "kind": "line", "pattern": "tray"},
                                                    {"type": "count", "pattern": "tray"}]},
            {"type": "count", "pattern": "tray", "to": 1}]}, cell)
        texts = [runnermod.describe(s) for s in runnermod.walk(p["steps"])]
        check("the new steps read as sentences", texts == [
            "Fallback: if a step fails, the steps under If it fails run instead", "Grasp with pneu, a 0.5 kg part",
            "Call the owner: No part", "Safe home to J: back up to 40 mm along the tool, then a joint move",
            "Retry: up to 3 tries", "Straight line to tray's next slot", "Count tray on to its next slot",
            "Count tray back to slot 1"], texts)

        plain = os.path.join(folder, "old")
        os.makedirs(plain)
        old_cell = {"version": 1, "revision": 7, "tools": cellmod.empty_cell()["tools"], "frames": {},
                    "targets": {"home": {"joints": HOME}}, "zones": {}, "active": {"tool": "flange"}}
        with open(os.path.join(plain, "cell.json"), "w", encoding="utf-8") as f:
            json.dump(old_cell, f)
        before = open(os.path.join(plain, "cell.json"), "rb").read()
        loaded = cellmod.Cell(plain)
        check("a cell saved before grippers, interlocks and patterns reads as having none, its file untouched",
              loaded.revision == 7 and loaded.data["grippers"] == {} and loaded.data["patterns"] == {}
              and loaded.data["targets"] == old_cell["targets"]
              and open(os.path.join(plain, "cell.json"), "rb").read() == before, loaded.data)
    finally:
        shutil.rmtree(folder, ignore_errors=True)


# ---- against the simulator ----------------------------------------------------------------------------------------------

def scenario_grippers():
    print("== grippers: open, close and grasp, confirmed by inputs or a set time; the grasp checked for a part; the "
          "payload switched to the tool and the part, and back")
    rig = Rig("grip")
    try:
        rig.enable()
        rev = setup(rig)
        rig.ctl(cmd="gripper", output="tool.0", close=1, seconds=0.3, closed="tool.1", opened="tool.2", part="tool.3")
        rig.ctl(cmd="part", present=True)
        for g in (PNEU, CHESS):
            r = rig.post("/api/cell/gripper", dict(g, revision=rev))
            assert r["ok"], r
            rev = r["cell"]["revision"]
        mass0, cog0 = rig.payload()
        check("the tool's own payload is on the controller to begin with (0.2 kg, 50 mm out)",
              abs(mass0 - TOOL_MASS) < 1e-6 and abs(cog0[2] - 50) < 1e-4, (mass0, cog0))

        r = rig.post("/api/gripper", {"name": "pneu", "action": "grasp", "part": PART})
        mass, cog = rig.payload()
        s = rig.status()
        check(f"the panel's Grasp closes it, its inputs confirm it closed on a part ({r.get('message')})",
              r["ok"] and "holds a part" in r["message"] and s["held"] == {"pneu": PART}, r)
        check(f"and the payload is the tool and the part together: {mass:.2f} kg, {cog[2]:.1f} mm out (0.2 kg at 50 mm "
              "and 0.5 kg 20 mm past the 100 mm tool point: 0.7 kg at 100 mm)",
              abs(mass - 0.7) < 1e-5 and abs(cog[2] - 100.0) < 1e-3 and abs(cog[0]) < 1e-6, (mass, cog))
        io = rig.get("/api/io")
        check("GET /api/io says the gripper's outputs are set to close it, it reads closed and on a part, and what it "
              "holds", io["grippers"]["pneu"] == {"outputs": "closed", "held": PART, "closed": True, "opened": False,
                                                  "part": True}, io.get("grippers"))
        r = rig.post("/api/gripper", {"name": "pneu", "action": "open"})
        mass, cog = rig.payload()
        check("Open opens it (its open input confirms it) and puts the tool's own payload back",
              r["ok"] and abs(mass - TOOL_MASS) < 1e-6 and abs(cog[2] - 50) < 1e-4 and rig.status()["held"] == {},
              (r, mass, cog))

        save(rig, {"name": "pick", "steps": [
            {"type": "gripper", "gripper": "pneu", "action": "open"},
            {"type": "gripper", "gripper": "pneu", "action": "grasp", "part": PART},
            {"type": "halt", "message": "holding it"},
            {"type": "gripper", "gripper": "pneu", "action": "open"}]})
        writes = rig.log().count("WRITE")
        r, s = rig.run("pick", "dry")
        check("a dry run of it sends the controller nothing and counts the gripper's time",
              r["ok"] and s["run"]["state"] == "done" and rig.log().count("WRITE") == writes
              and len(named(rig.events(), "dry.would-grip")) == 3, s["run"])
        rig.post("/api/run/reset")
        r, s = rig.run("pick")
        held_payload = rig.payload()
        check("in a program, the grasp step holds the part and switches the payload (read while the run is halted)",
              s["run"]["state"] == "paused" and s["run"]["halt"] == "holding it" and abs(held_payload[0] - 0.7) < 1e-5,
              (s["run"], held_payload))
        rig.post("/api/run/resume")
        s = rig.run_state("done", "faulted", "stopped")
        events = rig.run_log()
        done = [e for e in named(events, "gripper.done")]
        check("and the release puts it back; each step's confirmation is in the log (the first open at once, as the "
              "gripper was open already)", s["run"]["state"] == "done" and abs(rig.payload()[0] - TOOL_MASS) < 1e-6
              and len(done) == 3 and done[0]["seconds"] < 0.25 and all(0.25 <= e["seconds"] < 1.0 for e in done[1:]),
              (s["run"], done))

        rig.ctl(cmd="part", present=False)
        save(rig, {"name": "grab", "steps": [{"type": "gripper", "gripper": "pneu", "action": "open"},
                                              {"type": "gripper", "gripper": "pneu", "action": "grasp", "part": PART}]})
        r, s = rig.run("grab")
        calls = s.get("calls", [])
        check(f"with no part between its jaws the grasp fails, and the run faults ({s['run']['reason']})",
              s["run"]["state"] == "faulted" and "tool.3 reads 0, not 1: there's no part in it" in s["run"]["reason"]
              and s["run"]["path"] == [1], s["run"])
        check("the payload stays the tool's, and the fault calls the owner",
              abs(rig.payload()[0] - TOOL_MASS) < 1e-6 and calls and calls[-1]["why"] == "fault"
              and calls[-1]["program"] == "grab" and "no part" in calls[-1]["message"], (rig.payload(), calls))
        rig.recover()

        rig.ctl(cmd="gripper", output="tool.0", close=1, seconds=0.3, closed="tool.1", opened="tool.2", part="tool.3",
                jammed=True)
        save(rig, {"name": "squeeze", "steps": [{"type": "gripper", "gripper": "pneu", "action": "close"}]})
        started = time.monotonic()
        r, s = rig.run("squeeze")
        took = time.monotonic() - started
        check(f"a jammed gripper never confirms: the step fails after its 1 s timeout ({took:.1f} s)",
              s["run"]["state"] == "faulted" and "pneu didn't confirm it closed within 1 s (tool.1 doesn't read 1)"
              in s["run"]["reason"] and 1.0 <= took < 3.0, (s["run"], took))
        rig.recover()

        save(rig, {"name": "chess", "steps": [{"type": "gripper", "gripper": "chess", "action": "open"},
                                               {"type": "gripper", "gripper": "chess", "action": "grasp",
                                                "part": {"mass": 0.03}}]})
        r, s = rig.run("chess")
        done = named(rig.run_log(), "gripper.done")
        outputs = rig.ctl(cmd="state")["outputs"]["tool"]
        check("a gripper without inputs (the chess gripper) waits its set time, 0.8 s, and its grasp holds the part",
              s["run"]["state"] == "done" and len(done) == 2 and all(0.8 <= e["seconds"] < 1.2 for e in done)
              and (outputs >> 5) & 1 == 0 and s["held"].get("chess") == {"mass": 0.03, "cog": [0.0, 0.0, 0.0]},
              (s["run"], done, outputs, s["held"]))
        r = rig.post("/api/cell/gripper/delete", {"name": "chess", "revision": rig.rev()})
        check("a gripper a program uses can't be deleted", not r["ok"] and "program chess" in r["message"], r)
    finally:
        rig.close()


def scenario_payload_stops():
    print("== if writing the payload stops the arm (unverified on the arm; the simulator's --payload-stops), Enable's "
          "checks make it ready again and the run carries on")
    rig = Rig("payload", sim_args=["--payload-stops"])
    try:
        rig.enable()
        rev = setup(rig)
        rig.enable()  # making the pointer the tool in use wrote its payload, which stopped the arm
        rig.ctl(cmd="gripper", output="tool.0", close=1, seconds=0.3, closed="tool.1", opened="tool.2", part="tool.3")
        rig.ctl(cmd="part", present=True)
        rig.post("/api/cell/gripper", dict(PNEU, revision=rev))
        goto(rig, "A")
        save(rig, {"name": "carry", "speed": {"joint": 40}, "steps": [
            {"type": "gripper", "gripper": "pneu", "action": "grasp", "part": PART},
            {"type": "move", "kind": "joint", "target": "B"},
            {"type": "gripper", "gripper": "pneu", "action": "open"},
            {"type": "move", "kind": "joint", "target": "A"}]})
        mark = rig.log().count("payload set: stopped")
        r, s = rig.run("carry")
        stops = rig.log().count("payload set: stopped") - mark
        notes = [e.get("note", "") for e in named(rig.run_log(), "gripper.done")]
        check(f"the run grasps, carries the part to B, lets go and comes back: each payload write stopped the arm "
              f"({stops} times) and the server made it ready again", s["run"]["state"] == "done" and stops == 2
              and all("ready again" in n for n in notes), (s["run"], stops, notes))
    finally:
        rig.close()


def scenario_patterns():
    print("== a 3 x 4 tray taught at three corners: a program visits every slot by its counter, which persists")
    rig = Rig("tray")
    try:
        rig.enable()
        rev = setup(rig, points=("A", "T1", "T4", "T9"))
        r = rig.post("/api/cell/pattern", {"name": "tray", "rows": 3, "columns": 4, "first": "T1", "rowEnd": "T4",
                                           "columnEnd": "T9", "revision": rev})
        check(f"the pattern is saved ({r.get('message')})", r["ok"] and r["patterns"]["tray"]["count"] == 12
              and r["counters"] == {} and len([x for x in r["scene"]["slots"] if x["pattern"] == "tray"]) == 12, r)
        save(rig, {"name": "visit", "speed": {"joint": 60, "tcp": 150}, "steps": [
            {"type": "move", "kind": "joint", "pattern": "tray", "offset": {"frame": "tool", "xyz": [0, 0, -30]}},
            {"type": "repeat", "count": 12, "steps": [
                {"type": "move", "kind": "line", "pattern": "tray"},
                {"type": "wait", "seconds": 0.3},
                {"type": "count", "pattern": "tray"}]}]})
        finish = sampler(rig)
        r, s = rig.run("visit", timeout=120)
        samples = finish()
        truth = [kin.position(table_pose([40 * c, 35 * r_, 90])) for r_ in range(3) for c in range(4)]
        visits = []
        for _, path, q in samples:
            if path == [1, 1]:  # waiting at a slot
                p = tool_mm(q)
                k, d = min(((k, math.dist(p, t)) for k, t in enumerate(truth)), key=lambda x: x[1])
                if not visits or visits[-1][0] != k + 1:
                    visits.append((k + 1, d))
                else:
                    visits[-1] = (k + 1, max(visits[-1][1], d))
        check(f"the run visits the 12 slots in order, along each row then the next, the tool point within 1 mm of each "
              f"slot's centre (worst {max([d for _, d in visits] or [0]):.2f} mm)",
              s["run"]["state"] == "done" and [k for k, _ in visits] == list(range(1, 13))
              and all(d < 1.0 for _, d in visits), (s["run"], visits))
        check("its counter ends past the last slot: used up", s["counters"] == {"tray": 13}, s["counters"])
        rig.restart_server()
        rig.enable()
        s = rig.status()
        check("the counter persists through a restart of the server", s["counters"] == {"tray": 13}, s["counters"])
        r, s = rig.run("visit")
        check(f"a move to a used-up pattern fails ({s['run']['reason']})", s["run"]["state"] == "faulted"
              and "tray is used up: all 12 of its slots are done" in s["run"]["reason"], s["run"])
        rig.recover()
        r = rig.post("/api/cell/pattern/counter", {"name": "tray", "slot": 1})
        check("the panel's reset puts the counter back at slot 1", r["ok"] and rig.status()["counters"] == {"tray": 1}, r)
        r, s = rig.run("visit", "dry", timeout=60)
        counts = named(rig.run_log(), "pattern.count")
        check("a dry run counts on a copy: 12 counts in its log, and the counter is still at slot 1",
              s["run"]["state"] == "done" and len(counts) == 12 and counts[-1]["slot"] == 13
              and rig.status()["counters"] == {"tray": 1}, (s["run"], len(counts)))
        rig.post("/api/run/reset")
        save(rig, {"name": "skip", "speed": {"joint": 60}, "steps": [
            {"type": "count", "pattern": "tray", "to": 7},
            {"type": "move", "kind": "joint", "pattern": "tray"},
            {"type": "move", "kind": "joint", "pattern": "tray", "slot": 2}]})
        r, s = rig.run("skip", timeout=60)
        texts = [e["text"] for e in named(rig.run_log(), "move.planned")]
        check("a count step can set the counter (to slot 7), and a move can name a fixed slot",
              s["run"]["state"] == "done" and s["counters"] == {"tray": 7} and texts[0].startswith("Joint move to tray "
                                                                                                   "slot 7")
              and texts[1].startswith("Joint move to tray slot 2")
              and math.dist(tool_mm(rig.sim_joints()), truth[1]) < 1.0, (s["run"], texts))
        r = rig.post("/api/plan", {"pattern": "tray", "kind": "joint"})
        check("Plan on a pattern stages a joint move to its next slot", r["ok"] and "tray slot 7" in r["message"], r)
        bad = {"deleting a corner": rig.post("/api/cell/target/delete", {"name": "T9", "revision": rig.rev()}),
               "deleting a pattern a program uses": rig.post("/api/cell/pattern/delete",
                                                             {"name": "tray", "revision": rig.rev()}),
               "a counter past one beyond the last": rig.post("/api/cell/pattern/counter", {"name": "tray", "slot": 14})}
        check("a corner target and a pattern in use can't be deleted, nor a counter set past the end",
              not any(r["ok"] for r in bad.values()) and "pattern tray" in bad["deleting a corner"]["message"]
              and "program" in bad["deleting a pattern a program uses"]["message"], bad)
    finally:
        rig.close()


def scenario_interlocks():
    print("== interlocks: a machine's work area is a zone the arm may enter only while its door is open, checked "
          "before every move into it and watched during it")
    rig = Rig("door", brain=True)
    try:
        rig.enable()
        rev = setup(rig, points=("A", "Mout", "M"))
        rev = rig.post("/api/cell/interlock", {"name": "door_open", "input": "controller.2", "value": 1,
                                               "revision": rev})["cell"]["revision"]
        r = rig.post("/api/cell/zone", {"name": "machine", "frame": "table", "pose": MACHINE[0], "size": MACHINE[1],
                                        "interlocks": ["door_open"], "revision": rev})
        guard = [o for o in r["collision"]["obstacles"] if o.get("zone") == "machine"]
        check(f"a zone guarded by an interlock ({r.get('message')}), marked for the page's check",
              r["ok"] and guard and guard[0]["guard"] == ["door_open"]
              and r["scene"]["zones"][0]["interlocks"] == ["door_open"], r.get("collision", {}).get("obstacles"))
        goto(rig, "A")
        lines = rig.log().count("MOVET_LINE")
        r = rig.post("/api/moveto", {"target": "M", "kind": "line", "speed": 60})
        s = rig.run_state("done", "faulted", "stopped")
        check(f"with the door shut, a move into the zone is refused before anything is sent ({s['run']['reason']})",
              s["run"]["state"] == "faulted" and "the machine zone is closed, as door_open doesn't hold (controller.2 "
              "reads 0, not 1)" in s["run"]["reason"] and rig.log().count("MOVET_LINE") == lines, s["run"])
        rig.recover()
        save(rig, {"name": "tend", "speed": {"tcp": 100}, "steps": [
            {"type": "move", "kind": "line", "target": "Mout"}, {"type": "move", "kind": "line", "target": "M"},
            {"type": "move", "kind": "line", "target": "Mout"}]})
        r, s = rig.run("tend", "dry")
        assumed = [e for e in named(rig.events(), "dry.assumed") if e.get("zone") == "machine"]
        check("a dry run assumes the interlock will hold for the move into the zone, as it assumes inputs arrive, and "
              "says so (the move out of it needs none)", s["run"]["state"] == "done" and len(assumed) == 1
              and assumed[0]["interlocks"] == ["door_open"] and assumed[0]["path"] == "2", (s["run"], assumed))
        rig.post("/api/run/reset")
        r, s = rig.run("tend")
        check("a run with the door shut fails at the move into the zone, and calls the owner",
              s["run"]["state"] == "faulted" and s["run"]["path"] == [1] and "door_open doesn't hold" in s["run"]["reason"]
              and s["calls"] and s["calls"][-1]["program"] == "tend", (s["run"], s.get("calls")))
        rig.recover()
        goto(rig, "A")
        rig.ctl(cmd="input", line="controller", bit=2, value=1)
        finish = sampler(rig)
        r, s = rig.run("tend")
        samples = finish()
        inside = min(math.dist(tool_mm(q), kin.position(table_pose(POINTS["M"]))) for _, _, q in samples)
        check(f"with the door open, the run goes in and out (the tool point came within {inside:.2f} mm of M)",
              s["run"]["state"] == "done" and inside < 1.0, (s["run"], inside))

        save(rig, {"name": "slow", "speed": {"tcp": 100}, "steps": [
            {"type": "move", "kind": "line", "target": "M", "speed": 15},
            {"type": "move", "kind": "line", "target": "Mout"}]})
        r = rig.post("/api/run/start", {"name": "slow"})
        rig.wait(lambda s: s["run"].get("lastEvent", {}).get("event") == "move.sent", 10)
        time.sleep(1.5)
        before = rig.ctl(cmd="state")
        dropped_at = time.monotonic()
        rig.ctl(cmd="input", line="controller", bit=2, value=0)
        s = rig.run_state("paused", "faulted", "done", "stopped", timeout=10)
        paused_after = time.monotonic() - dropped_at
        time.sleep(0.6)
        still = rig.ctl(cmd="state")
        check(f"shutting the door during the move into the zone stops the arm and pauses the run within "
              f"{paused_after:.2f} s ({s['run']['reason']})", before["moving"] and s["run"]["state"] == "paused"
              and "door_open doesn't hold" in s["run"]["reason"] and paused_after < 1.0 and not still["moving"],
              (before["moving"], s["run"], paused_after, still["moving"]))
        check("and calls the owner", any(c["why"] == "interlock" for c in s["calls"]), s["calls"])
        rig.ctl(cmd="input", line="controller", bit=2, value=1)
        rig.post("/api/run/resume")
        s = rig.run_state("done", "faulted", "stopped", timeout=60)
        check("with the door open again, Resume carries on into the zone and out", s["run"]["state"] == "done", s["run"])

        goto(rig, "Mout")
        rig.ctl(cmd="input", line="controller", bit=2, value=0)
        inside_joints = joints_for(POINTS["M"])
        r = rig.post("/api/move", {"targets": [round(v, 3) for v in inside_joints], "speed": 30})
        check(f"the page's Move into the zone is refused while the door is shut ({r.get('message')})",
              not r["ok"] and "door_open doesn't hold" in r["message"], r)
        rig.ctl(cmd="input", line="controller", bit=2, value=1)
        r = rig.post("/api/move", {"targets": [round(v, 3) for v in inside_joints], "speed": 30})
        s = rig.wait(lambda s: s.get("state") in (0, 2) and not s.get("goal"), 20)
        check("and goes once it's open", r["ok"] and r["moved"]
              and math.dist(tool_mm(rig.sim_joints()), kin.position(table_pose(POINTS["M"]))) < 1.0, r)
        goto(rig, "Mout")

        rig.wait(lambda s: s["brain"].get("ok"), 10)
        save(rig, {"name": "planned", "speed": {"joint": 30}, "steps": [
            {"type": "move", "kind": "planned", "target": "M"}, {"type": "move", "kind": "line", "target": "Mout"}]})
        r, s = rig.run("planned", timeout=60)
        plans = named(rig.run_log(), "brain.plan")
        check("the brain is told the zone is open while its interlock holds, and plans straight in",
              s["run"]["state"] == "done" and plans and plans[0]["waypoints"] == 2, (s["run"], plans))
        rig.ctl(cmd="input", line="controller", bit=2, value=0)
        r, s = rig.run("planned", timeout=60)
        check(f"with the door shut the zone is in the brain's world, and it finds no way in ({s['run']['reason']})",
              s["run"]["state"] == "faulted" and "the brain found no path to M" in s["run"]["reason"], s["run"])
    finally:
        rig.close()


def shelf_over(q, rise):
    """A thin zone over the wrist at q, placed so that backing the tool straight up brings the arm within the margin
    of it after about `rise` mm: (pose in base, size)."""
    flange = kin.position(MODEL.frames(q)[-1])
    probe_z = flange[2] + 600.0
    probe = utra_model.Obstacle("the probe", "box", [flange[0] / 1000, flange[1] / 1000, probe_z / 1000],
                                [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.2, 0.2, 0.02])
    model = MODEL.with_cell([(6, [0, 0, 0], [0, 0, 0.09], 0.012, "the tool")], [probe])
    gap = min(g for g, _, other in model.clearances(q) if other == "the probe") * 1000.0
    top = probe_z - 10.0 - gap  # the arm's highest point under it
    bottom = top + utra_model.MARGIN_M * 1000.0 + rise
    return {"xyz": [flange[0], flange[1], bottom + 10.0], "rpy": [0, 0, 0]}, [200, 200, 20]


def scenario_recovery():
    print("== recovery: Retry and Fallback steps, safe home (back along the tool, then a checked joint move home) and "
          "calling the owner")
    rig = Rig("recover")
    try:
        rig.enable()
        rev = setup(rig, joints={"home": HOME})
        rig.ctl(cmd="gripper", output="tool.0", close=1, seconds=0.3, closed="tool.1", opened="tool.2", part="tool.3")
        rig.post("/api/cell/gripper", dict(PNEU, revision=rig.rev()))
        save(rig, {"name": "patient", "steps": [
            {"type": "retry", "tries": 3, "steps": [{"type": "wait", "input": "tool.7", "value": 1, "timeout": 0.6}]}]})

        def on_third_try():  # the input arrives during the third try
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if any(e["event"] == "retry" and e.get("attempt") == 3 for e in rig.events()):
                    rig.ctl(cmd="input", line="tool", bit=7, value=1)
                    return
                time.sleep(0.05)
        timer = threading.Thread(target=on_third_try, daemon=True)
        timer.start()
        r, s = rig.run("patient")
        timer.join()
        events = rig.run_log()
        check("Retry runs its steps again when one fails: two waits time out, the third sees the input",
              s["run"]["state"] == "done" and len(named(events, "step.failed")) == 2 and len(named(events, "retry")) == 2
              and len(named(events, "io.waited")) == 1, (s["run"], [e["event"] for e in events]))
        save(rig, {"name": "spent", "steps": [
            {"type": "retry", "tries": 2, "steps": [{"type": "wait", "input": "tool.6", "value": 1, "timeout": 0.3}]}]})
        r, s = rig.run("spent")
        events = rig.run_log()
        check("when its tries are spent, the failure faults the run as any failure does",
              s["run"]["state"] == "faulted" and len(named(events, "retry")) == 1 and len(named(events, "retry.spent")) == 1
              and "waiting for tool.6" in s["run"]["reason"], (s["run"], [e["event"] for e in events]))
        rig.recover()
        save(rig, {"name": "fault", "steps": [
            {"type": "retry", "tries": 3, "steps": [{"type": "wait", "input": "tool.6", "value": 1, "timeout": 10}]}]})
        timer = threading.Timer(0.8, lambda: rig.ctl(cmd="fault", err=9))
        timer.start()
        r, s = rig.run("fault")
        timer.join()
        check(f"a fault isn't a failure: Retry doesn't catch a controller error ({s['run']['reason']})",
              s["run"]["state"] == "faulted" and "error 9" in s["run"]["reason"] and not named(rig.run_log(), "retry"),
              s["run"])
        rig.recover()

        goto(rig, "A")
        a_flange = kin.position(MODEL.frames(rig.sim_joints())[-1])
        save(rig, {"name": "rescue", "speed": {"joint": 40, "tcp": 80}, "steps": [
            {"type": "fallback", "steps": [{"type": "gripper", "gripper": "pneu", "action": "grasp", "part": PART}],
             "else": [{"type": "notify", "message": "No part at A"},
                      {"type": "home", "target": "home", "retract": 40}]}]})
        mark = len(rig.log())
        r, s = rig.run("rescue", timeout=60)
        events = rig.run_log()
        retract = named(events, "home.retract")
        check("Fallback: the grasp finds no part, so the steps under If it fails run: the owner is called, then safe "
              "home", s["run"]["state"] == "done" and [e["event"] for e in events if e["event"] in (
                  "step.failed", "fallback", "owner.called", "home.retract")] == ["step.failed", "fallback",
                                                                                 "owner.called", "home.retract"],
              (s["run"], [e["event"] for e in events]))
        check("the call reaches the panel's status, with the program and the step",
              any(c["message"] == "No part at A" and c["program"] == "rescue" and c["path"] == "1.2" and c["why"] == "step"
                  for c in s["calls"]), s["calls"])
        sent = rig.log()[mark:]
        lines = [[float(v) for v in m.group(1).split()] for m in re.finditer(r"MOVET_LINE: goal=\[([^\]]*)\]", sent)]
        up = [a_flange[0], a_flange[1], a_flange[2] + 40.0]
        check(f"safe home backs the tool 40 mm straight up its axis (the one line sent ends "
              f"{math.dist(lines[0][:3], up) if lines else math.nan:.2f} mm from 40 mm above A), then a joint move "
              "takes it home", retract and retract[0]["gotMm"] == 40.0 and len(lines) == 1
              and math.dist(lines[0][:3], up) < 0.05 and "MOVEJ_P2P" in sent.split("MOVET_LINE")[-1]
              and max(abs(a - b) for a, b in zip(rig.sim_joints(), HOME)) < 0.2, (retract, lines, up))
        r = rig.post("/api/calls/ack", {})
        check("the panel's Seen clears the calls", r["ok"] and rig.status()["calls"] == [], r)
        r, s = rig.run("rescue", **{"from": [0, 1]}, timeout=60)
        events = rig.run_log()
        check("Run from a step under If it fails runs from there, without the steps above it",
              s["run"]["state"] == "done" and not named(events, "step.failed")
              and [e["text"] for e in named(events, "step.start")][0] == "Call the owner: No part at A",
              [e["event"] for e in events])

        goto(rig, "A")
        q = rig.sim_joints()
        shelf, size = shelf_over(q, 15.0)
        rig.post("/api/cell/zone", {"name": "shelf", "frame": "base", "pose": shelf, "size": size, "revision": rig.rev()})
        save(rig, {"name": "duck", "speed": {"joint": 30, "tcp": 50}, "steps": [
            {"type": "home", "target": "A", "retract": 60}]})
        r, s = rig.run("duck", timeout=60)
        retract = named(rig.run_log(), "home.retract")
        check(f"with a shelf over the wrist, safe home backs off as far as it can ({retract[0]['gotMm'] if retract else '?'} "
              "of 60 mm, saying why) before its joint move",
              s["run"]["state"] == "done" and retract and 5.0 < retract[0]["gotMm"] < 30.0
              and "the shelf zone" in retract[0].get("short", ""), (s["run"], retract))
        rig.post("/api/cell/zone/delete", {"name": "shelf", "revision": rig.rev()})
        r = rig.post("/api/home", {"target": "home", "retract": 20, "defaults": {"joint": 40}})
        s = rig.run_state("done", "faulted", "stopped", timeout=60)
        check("the panel's Safe home runs the same procedure", r["ok"] and s["run"]["state"] == "done"
              and s["run"]["manual"] and max(abs(a - b) for a, b in zip(rig.sim_joints(), HOME)) < 0.2, s["run"])
    finally:
        rig.close()


def scenario_touchup():
    print("== touch-up: re-teach a target in place; the change is shown, the old value kept for Undo and in the log")
    rig = Rig("touch")
    try:
        rig.enable()
        setup(rig, points=("A", "B"), joints={"J": HOME})
        save(rig, {"name": "go", "speed": {"tcp": 80, "joint": 40}, "steps": [
            {"type": "move", "kind": "joint", "target": "B"}, {"type": "move", "kind": "line", "target": "A"}]})
        goto(rig, "A")
        old = rig.get("/api/cell")["cell"]["targets"]["A"]
        r = rig.post("/api/jog", {"kind": "tool", "index": 0, "dir": 1, "step": 5, "rotStep": 2, "speed": 20})
        rig.wait(lambda s: s.get("state") in (0, 2) and not s.get("goal"), 10)
        time.sleep(0.4)
        q = rig.sim_joints()
        now = kin.mul(kin.invert(FRAME), KIN.tool_pose(q, TOOL))
        expected = math.dist(kin.position(now), old["pose"]["xyz"])
        r = rig.post("/api/cell/target/touchup", {"name": "A", "revision": rig.rev()})
        new = r.get("target", {})
        check(f"Touch up re-teaches A where the arm is ({r.get('message')})", r["ok"] and new.get("frame") == "table"
              and new.get("tool") == "pointer" and abs(r["change"]["movedMm"] - expected) < 0.05
              and 4.0 < r["change"]["movedMm"] < 6.0 and "program go" in r["message"], (r, expected))
        check("the target keeps its old value and the change, for the page to show and Undo",
              new.get("previous") == old and new.get("touchUp", {}).get("movedMm") == r["change"]["movedMm"], new)
        with open(os.path.join(rig.cell_dir, "logs", "targets.jsonl"), encoding="utf-8") as f:
            last = [json.loads(line) for line in f][-1]
        check("cell/logs/targets.jsonl has the old value and the new", last["event"] == "touched-up"
              and last["old"] == old and last["new"]["pose"] == new["pose"], last)
        r, s = rig.run("go", timeout=60)
        end = tool_mm(rig.sim_joints())
        check("a program's move to A now goes to where it was touched up",
              s["run"]["state"] == "done" and math.dist(end, kin.position(KIN.tool_pose(q, TOOL))) < 1.0, (s["run"], end))
        r = rig.post("/api/cell/target/undo", {"name": "A", "revision": rig.rev()})
        with open(os.path.join(rig.cell_dir, "logs", "targets.jsonl"), encoding="utf-8") as f:
            last = [json.loads(line) for line in f][-1]
        check("Undo puts A back as it was, and logs that too", r["ok"] and rig.get("/api/cell")["cell"]["targets"]["A"]
              == old and last["event"] == "undone", (r, last))
        r = rig.post("/api/cell/target/touchup", {"name": "J", "revision": rig.rev()})
        check("a joint target is touched up to the joints where the arm is", r["ok"] and "jointsMaxDeg" in r["change"]
              and max(abs(a - b) for a, b in zip(r["target"]["joints"], rig.sim_joints())) < 0.05, r)
        rig.post("/api/cell/tool/activate", {"name": "flange", "revision": rig.rev()})
        r = rig.post("/api/cell/target/touchup", {"name": "B", "revision": rig.rev()})
        check("a tool target is touched up only with the tool it was taught with in use",
              not r["ok"] and "make it the tool in use" in r["message"], r)
        rig.post("/api/cell/tool/activate", {"name": "pointer", "revision": rig.rev()})
        r = rig.post("/api/cell/target", {"here": True, "name": "B", "frame": "table", "revision": rig.rev()})
        with open(os.path.join(rig.cell_dir, "logs", "targets.jsonl"), encoding="utf-8") as f:
            last = [json.loads(line) for line in f][-1]
        check("recording over an existing target keeps its old value in the log too", r["ok"] and last["event"] ==
              "retaught" and last["target"] == "B" and last["old"]["pose"]["xyz"] == POINTS["B"], last)
    finally:
        rig.close()


def scenario_compat():
    print("== an old cell and program, as saved before this, load and run unchanged; the API keeps its fields")
    folder = log_path("tendcell_compat")
    shutil.rmtree(folder, ignore_errors=True)
    os.makedirs(os.path.join(folder, "programs"))
    old_cell = {"version": 1, "revision": 4, "tools": {"flange": {"tcp": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                                                                "mass": 0.0, "cog": [0.0, 0.0, 0.0], "capsules": []}},
                "frames": {}, "targets": {"home": {"joints": HOME}, "near": {"joints": [5, 0, 90, 0, 90, 0]}},
                "zones": {}, "active": {"tool": "flange"}}
    old_program = {"name": "old", "steps": [{"id": "s1", "type": "move", "kind": "joint", "target": "near"},
                                            {"id": "s2", "type": "repeat", "count": 2, "steps": [
                                                {"id": "s3", "type": "wait", "seconds": 0.1}]},
                                            {"id": "s4", "type": "move", "kind": "joint", "target": "home"}],
                   "speed": {"tcp": 50.0, "ori": 30.0, "joint": 20.0}, "fullSpeed": False, "revision": 2}
    with open(os.path.join(folder, "cell.json"), "w", encoding="utf-8") as f:
        json.dump(old_cell, f, indent=1)
    with open(os.path.join(folder, "programs", "old.json"), "w", encoding="utf-8") as f:
        json.dump(old_program, f, indent=1)
    before = open(os.path.join(folder, "cell.json"), "rb").read()
    rig = Rig("compat", cell=folder)
    try:
        rig.enable()
        c = rig.get("/api/cell")
        p = rig.get("/api/program?name=old")
        check("the old cell reads as saved, with no grippers, interlocks or patterns, and its file is untouched",
              c["ok"] and all(c["cell"][k] == v for k, v in old_cell.items()) and c["cell"]["grippers"] == {}
              and c["cell"]["interlocks"] == {} and c["cell"]["patterns"] == {}
              and open(os.path.join(folder, "cell.json"), "rb").read() == before, c["cell"])
        check("the old program loads as saved", p["ok"] and p["program"] == old_program, p)
        r, s = rig.run("old", timeout=60)
        check("and runs as before", r["ok"] and s["run"]["state"] == "done"
              and [e["event"] for e in rig.events()].count("step.done") == 4, s["run"])
        fields = ("ip", "simulated", "note", "noteOk", "stopAlarm", "live", "age", "manual", "brain", "state", "enabled",
                  "err", "errName", "warn", "queued", "joints", "pose", "run", "freedrive", "teachSensitivity", "mode",
                  "cellRevision", "tool")
        missing = [k for k in fields if k not in s]
        io = rig.get("/api/io")
        check("GET /api/status keeps every field it had (and adds held, counters and calls); GET /api/io its io",
              not missing and {"held", "counters", "calls"} <= set(s) and set(io["io"]) == {"tool", "controller"}
              and set(io["io"]["tool"]) == {"inputs", "outputs"}, (missing, io))
    finally:
        rig.close()


SCENARIOS = (unit_cell, scenario_grippers, scenario_payload_stops, scenario_patterns, scenario_interlocks,
             scenario_recovery, scenario_touchup, scenario_compat)


def main(names):
    print(f"the server and the stub brain run on {PY}; the test on {sys.executable}; the brain's port is {BRAIN_PORT}")
    chosen = [fn for fn in SCENARIOS if not names or fn.__name__ in names]
    for fn in chosen:
        try:
            fn()
        except Exception as e:  # report and carry on
            import traceback
            traceback.print_exc()
            check(f"{fn.__name__} crashed", False, repr(e))
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed or not chosen else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
