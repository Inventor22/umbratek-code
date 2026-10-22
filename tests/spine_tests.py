"""Planned and learned steps, the brain interface (scripts/brain_protocol.py, brain_link.py) and the safety shell
(scripts/safety.py). First in-process: the protocol's checks, the link's deadlines and loopback rule, and the shell's
checks of paths and bursts. Then the jog server against the simulator, with the stub brain (brain/stub/stub_brain.py)
as a separate process, or a brain played by this test on its own port (the stub's answers, which it can delay or
spoil): a taught, planned, taught program end to end; a brain whose paths go through a keep-out zone; stale, malformed
and dangerous replies, each refused with nothing sent; learned skills inside and beyond their region; STOP while the
brain thinks and while its path runs; programs that need a brain that isn't there; and runs of one program started
within the same second, each with a log of its own. Standard library only.
Usage: python tests/spine_tests.py"""
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
from http.server import BaseHTTPRequestHandler

from simrig import HOST, HTTP_PORT, PY, ROOT, SCRIPTS, SERVER, log_path, start_sim, stop, wait_port

sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(ROOT, "brain", "stub"))
import brain_link  # noqa: E402
import brain_protocol as bp  # noqa: E402
import kinematics as kin  # noqa: E402
import planner as plannermod  # noqa: E402
import safety  # noqa: E402
import stub_brain  # noqa: E402
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
FRAME_POSE = {"xyz": [380, -100, 100], "rpy": [0, 0, 15]}
FRAME = kin.pose_to_matrix(FRAME_POSE)
SEED = [-10, 10, 100, 0, 70, 0]
POINTS = {"A": [0, 0, 120], "C": [0, 260, 120]}  # tool point positions in the table's frame, pointing down
POST = ({"xyz": [0, 130, 100], "rpy": [0, 0, 0]}, [60, 60, 200])  # a keep-out zone between them, 200 mm tall
HOME = [0, 0, 90, 0, 90, 0]
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)[:700]}", flush=True)


def target_pose(xyz, back=0.0):
    """base <- tool point at a point of the table (pointing down), `back` mm back along the tool."""
    t = kin.mul(FRAME, kin.pose_to_matrix({"xyz": xyz, "rpy": [180, 0, 0]}))
    return kin.mul(t, kin.offset_matrix([0, 0, -back])) if back else t


def target_joints(xyz, back=0.0, seed=SEED):
    r = KIN.solve(seed, target_pose(xyz, back), LIMITS, TOOL)
    assert r["ok"], r
    return r["joints"]


A = target_joints(POINTS["A"])
C = target_joints(POINTS["C"])
C_UP = target_joints(POINTS["C"], back=50.0, seed=C)  # where the planned move goes: 50 mm above C
A_MM = kin.position(target_pose(POINTS["A"]))


def post_obstacle():
    t = kin.mul(FRAME, kin.pose_to_matrix(POST[0]))
    return utra_model.Obstacle("the post zone", "box", [t[i][3] for i in range(3)], [[t[0][k], t[1][k], t[2][k]]
                                                                                     for k in range(3)],
                               [v / 1000.0 for v in POST[1]])


def cell_model(zone=True):
    return MODEL.with_cell([(6, [0, 0, 0], [0, 0, 0.09], 0.012, "the tool")], [post_obstacle()] if zone else [])


def world(zone=True):
    """The world as the spine describes it."""
    o = post_obstacle()
    zones = [{"name": "post", "centre": [v * 1000.0 for v in o.centre], "axes": o.axes, "size": list(POST[1])}]
    return {"cellRevision": 3, "marginMm": 10.0, "tool": {"name": "pointer", "tcp": TOOL_POSE, "capsules": [CAPSULE]},
            "zones": zones if zone else []}


def tool_mm(q):
    return kin.position(KIN.tool_pose(q, TOOL))


def moved(q, dx=0.0, dy=0.0, dz=0.0):
    """Joints that move the tool point from where q puts it by (dx, dy, dz) mm in the base frame, turned the same."""
    t = [list(row) for row in KIN.tool_pose(q, TOOL)]
    t[0][3] += dx / 1000.0
    t[1][3] += dy / 1000.0
    t[2][3] += dz / 1000.0
    r = KIN.solve(q, t, LIMITS, TOOL)
    assert r["ok"], r
    return r["joints"]


def cart_via(start):
    """A waypoint that swings wrist 2 into one of the cart's base bars, from near A."""
    return [start[0], -120.0, 60.0, start[3], start[4], start[5]]


def refusal(fn):
    """The shell's or the protocol's reason for refusing what fn() checks, or None if it passed."""
    try:
        fn()
    except (safety.Refused, bp.ProtocolError) as e:
        return e
    return None


class FakeBrain:
    """A brain played by the test on its own port: the stub's answers, worked out in-process, unless `misbehave`
    (request, reply) returns something else to send (a dict, raw bytes, or an HTTP status code), and after `delay[type]`
    seconds. It records every request, and checks each against the protocol as a brain would."""

    def __init__(self, port=BRAIN_PORT):
        self.stub = stub_brain.StubBrain()
        self.misbehave = None
        self.delay = {}
        self.requests = []
        self.bad_requests = []
        fake = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt, *args):
                pass

            def _send(self, code, data):
                try:
                    self.send_response(code)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except ConnectionError:
                    pass  # the spine stopped waiting (a deadline, STOP or pause), as it may

            def do_POST(self):
                body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
                try:
                    request = bp.check_request(bp.decode(body))
                except bp.ProtocolError as e:
                    fake.bad_requests.append(str(e))
                    return self._send(400, b"{}")
                fake.requests.append((time.perf_counter(), request))
                reply = fake.stub.handle(request)
                out = fake.misbehave(request, reply) if fake.misbehave else None
                time.sleep(fake.delay.get(request["type"], 0.0))
                if isinstance(out, int):
                    return self._send(out, b'{"error": "played"}')
                self._send(200, out if isinstance(out, bytes) else json.dumps(reply if out is None else out).encode())

        self.server = stub_brain.Server(("127.0.0.1", port), Handler)
        threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()

    def of(self, kind):
        return [r for _, r in self.requests if r["type"] == kind]

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def only(kind, fn):
    """A misbehaviour for one type of request; the others are answered properly."""
    return lambda request, reply: fn(request, reply) if request["type"] == kind else None


class Rig:
    """The simulator, a brain ("stub": the stub brain's process, "fake": a FakeBrain, None: nothing listening) and the
    jog server, asking that brain."""

    def __init__(self, name, brain="stub", brain_args=(), server_args=()):
        self.name, self.fake, self.brain, self.server, self.sim, self.ctl_sock = name, None, None, None, None, None
        self.cell_dir = log_path(f"spinecell_{name}")
        shutil.rmtree(self.cell_dir, ignore_errors=True)
        self.outs = []
        try:
            self.sim, self.sim_log = start_sim((), f"spine_{name}.log")
            if brain == "stub":
                out = open(log_path(f"spine_{name}_brain.log"), "w")
                self.outs.append(out)
                self.brain = subprocess.Popen([PY, STUB, "--port", str(BRAIN_PORT), *brain_args], cwd=ROOT, stdout=out,
                                              stderr=subprocess.STDOUT)
                assert wait_port(BRAIN_PORT, 15, host="127.0.0.1"), "the stub brain did not start"
            elif brain == "fake":
                self.fake = FakeBrain()
            out = open(log_path(f"spine_{name}_server.log"), "w")
            self.outs.append(out)
            self.server = subprocess.Popen([PY, SERVER, HOST, "--port", str(HTTP_PORT), "--cell", self.cell_dir, "--brain",
                                            BRAIN_URL, *server_args], cwd=ROOT, stdout=out, stderr=subprocess.STDOUT)
            assert wait_port(HTTP_PORT, 15, host="127.0.0.1"), "the server did not start"
            html = urllib.request.urlopen(BASE + "/", timeout=10).read().decode()
            self.token = re.search(r'const TOKEN = "([^"]+)"', html).group(1)
            self.ctl_sock = socket.create_connection((HOST, 30099), timeout=5)
            self.ctl_buf = b""
        except BaseException:
            self.close()
            raise

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

    def sim_joints(self):
        return self.ctl(cmd="state")["joints"]

    def log(self):
        with open(self.sim_log) as f:
            return f.read()

    def moves(self):
        """How many joint moves the controller has taken (a move arriving after a STOP is discarded, as it's stopped)."""
        log = self.log()
        return log.count("MOVEJ_P2P: goal=") - log.count("MOVEJ_P2P: discarded")

    def events(self):
        return self.get("/api/run")["run"]["events"]

    def run_log(self):
        """The current run's whole event log, from its file."""
        with open(os.path.join(self.cell_dir, "logs", self.get("/api/run")["run"]["log"]), encoding="utf-8") as f:
            return [json.loads(line) for line in f]

    def close(self):
        if self.ctl_sock is not None:
            self.ctl_sock.close()
        for proc in (self.server, self.brain):
            if proc is not None:
                proc.kill()
                proc.wait()
        if self.fake is not None:
            self.fake.close()
        stop(self.sim)
        for out in self.outs:
            out.close()


def setup(rig, zone=True):
    """The pointer tool (100 mm), the table's frame, targets A and C on it, a joint target home, and the post zone."""
    rev = rig.get("/api/cell")["cell"]["revision"]
    rev = rig.post("/api/cell/tool", {"name": "pointer", "tcp": TOOL_POSE, "mass": 0.2, "cog": [0, 0, 50],
                                      "capsules": [CAPSULE], "revision": rev})["cell"]["revision"]
    rev = rig.post("/api/cell/tool/activate", {"name": "pointer", "revision": rev})["cell"]["revision"]
    rev = rig.post("/api/cell/frame", {"name": "table", "parent": "base", "pose": FRAME_POSE,
                                       "revision": rev})["cell"]["revision"]
    for name, seed in (("A", A), ("C", C)):
        rev = rig.post("/api/cell/target", {"name": name, "frame": "table", "pose": {"xyz": POINTS[name], "rpy": [180, 0, 0]},
                                            "seed": seed, "revision": rev})["cell"]["revision"]
    rev = rig.post("/api/cell/target", {"name": "home", "joints": HOME, "revision": rev})["cell"]["revision"]
    if zone:
        rig.post("/api/cell/zone", {"name": "post", "frame": "table", "pose": POST[0], "size": POST[1], "revision": rev})


def save(rig, program):
    r = rig.post("/api/program", {"program": program})
    assert r["ok"], r
    return r["program"]


def goto(rig, name):
    r = rig.post("/api/moveto", {"target": name, "kind": "joint", "speed": 60})
    assert r["ok"], r
    s = rig.run_state("done", "faulted", "stopped")
    assert s["run"]["state"] == "done", s["run"]


def sampler(rig):
    """Samples the simulator's joints, with the run's step, until stopped: returns (samples, stop)."""
    samples, done = [], threading.Event()

    def run():
        while not done.is_set():
            try:
                path = rig.status()["run"]["path"]
                samples.append((path, rig.sim_joints()))
            except OSError:
                pass
            time.sleep(0.02)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    def finish():
        done.set()
        thread.join()
        return samples
    return finish


TEND = {"name": "tend", "speed": {"joint": 30, "tcp": 60}, "steps": [
    {"type": "move", "kind": "joint", "target": "A"},
    {"type": "move", "kind": "planned", "target": "C", "offset": {"frame": "tool", "xyz": [0, 0, -50]}},
    {"type": "move", "kind": "line", "target": "C"}]}
HOP = {"name": "hop", "speed": {"joint": 30}, "steps": [
    {"type": "move", "kind": "planned", "target": "C", "offset": {"frame": "tool", "xyz": [0, 0, -50]}}]}

# ---- in-process checks --------------------------------------------------------------------------------------------

def plan_request(seq=7, joints=None, goal=None):
    joints = A if joints is None else joints
    state = {"seq": seq, "ageS": 0.02, "joints": joints, "tool": kin.matrix_to_pose(KIN.tool_pose(joints, TOOL), 4)}
    return bp.message("plan", "plan-test-1", state=state, goal={"joints": C_UP if goal is None else goal}, world=world(),
                      limits={"joints": [list(v) for v in LIMITS]}, timeoutS=10)


def unit_protocol():
    print("== the protocol: one version, replies tied to their request and state, malformed messages refused")
    req = plan_request()
    check("the spine's plan request passes the brain's own checks", bp.check_request(req) is req)
    good = bp.reply(req, ok=True, path=[A, C_UP])
    got = bp.check_reply(req, dict(good, extra={"ignored": True}))
    check("a good reply passes, keeping only the fields this version defines", got == {"ok": True, "path": [A, C_UP],
                                                                                    "note": ""}, got)
    nan_path = [A, C_UP[:5] + [float("nan")]]
    cases = [("a reply in another version", dict(good, v=2), "protocol version 2"),
             ("a reply with no version", {k: v for k, v in good.items() if k != "v"}, "protocol version null"),
             ("a reply to another request", dict(good, id="plan-test-0"), "stale"),
             ("a reply worked out from another state", dict(good, stateSeq=6), "stale"),
             ("a state number given as true", dict(good, stateSeq=True), "stale"),
             ("a reply of another type", dict(good, type="act"), "reply to a 'plan' request"),
             ("a waypoint of five joints", dict(good, path=[A, C_UP[:5]]), "joint angles"),
             ("an angle given as text", dict(good, path=[A, C_UP[:5] + ["90"]]), "not a number"),
             ("an angle given as true", dict(good, path=[A, C_UP[:5] + [True]]), "not a number"),
             ("an angle of 400 degrees", dict(good, path=[A, C_UP[:5] + [400.0]]), "outside"),
             ("an angle that isn't finite", dict(good, path=nan_path), "not a number"),
             ("a path of one waypoint", dict(good, path=[A]), "waypoints"),
             ("a path of 65 waypoints", dict(good, path=[A] * 65), "waypoints"),
             ("a reply without ok", {k: v for k, v in good.items() if k != "ok"}, "ok is"),
             ("a refusal without a reason", dict(good, ok=False), "reason"),
             ("a list instead of an object", [good], "isn't a JSON object")]
    for name, answer, words in cases:
        e = refusal(lambda: bp.check_reply(req, answer))
        check(f"refuses {name}, saying so", e is not None and words in str(e), e)
    act = bp.message("act", "act-test-1", state=req["state"], skill="seek", params={}, episode="e-1", tick=0,
                     anchor={"xyz": A_MM}, withinMm=60, world=world(), limits={}, timeoutS=2, remainingS=10)
    check("the spine's act request passes the brain's own checks", bp.check_request(act) is act)
    for name, action, words in [("an unknown action", {"kind": "jump"}, "kind should be"),
                                ("a hold longer than 1 s", {"kind": "hold", "seconds": 2}, "outside"),
                                ("a burst of 11 waypoints after its start", {"kind": "move", "path": [A] * 12}, "waypoints"),
                                ("a move whose speed is 0", {"kind": "move", "path": [A, A], "speed": 0}, "outside"),
                                ("giving up without a reason", {"kind": "fail"}, "reason")]:
        e = refusal(lambda: bp.check_reply(act, bp.reply(act, action=action)))
        check(f"refuses {name}, saying so", e is not None and words in str(e), e)
    ok = bp.check_reply(act, bp.reply(act, action={"kind": "move", "path": [A, A], "speed": 12}))
    check("a burst with a speed passes", ok == {"action": {"kind": "move", "path": [A, A], "speed": 12.0}}, ok)
    for name, data, words in [("bytes that aren't JSON", b"<html>busy</html>", "isn't JSON"),
                              ("NaN, which JSON doesn't have", b'{"v": NaN}', "isn't JSON"),
                              ("Infinity", b'{"v": -Infinity}', "isn't JSON"),
                              ("a number too big to be finite", b'{"v": 1e999}', "isn't finite"),
                              ("bytes that aren't UTF-8", b'{"v": "\xff"}', "isn't JSON"),
                              ("a list", b"[1, 2]", "isn't a JSON object"),
                              ("nesting 30 deep", b'{"a": ' + b"[" * 30 + b"]" * 30 + b"}", "nests deeper"),
                              ("more than 1 MiB", b" " * (bp.MAX_BYTES + 1), "over the")]:
        e = refusal(lambda: bp.decode(data))
        check(f"decoding refuses {name}", e is not None and words in str(e), e)
    check("encoding refuses NaN rather than send what JSON can't carry",
          refusal(lambda: bp.encode({"x": float("nan")})) is not None)
    for name, bad, words in [("another version", dict(req, v=0), "protocol version 0"),
                             ("an id with a space", dict(req, id="plan 1"), "not an id"),
                             ("a state without joints", dict(req, state=dict(req["state"], joints=None)), "joints"),
                             ("a zone with two axes", dict(req, world=dict(world(), zones=[dict(world()["zones"][0],
                                                                                              axes=[[1, 0, 0]] * 2)])),
                              "not a zone")]:
        e = refusal(lambda: bp.check_request(bad))
        check(f"a brain's check refuses a request with {name}", e is not None and words in str(e), e)


def unit_link():
    print("== the link: loopback addresses only, a deadline on every question, a wait STOP can abandon")
    for url in ("http://127.0.0.1:8770", "http://localhost:8770", "http://[::1]:8770", "http://127.0.0.5:9000/"):
        check(f"accepts the brain at {url}", brain_link.loopback(url)[1] in (8770, 9000))
    for url in ("http://10.0.0.5:8770", "https://127.0.0.1:8770", "http://example.com:8770", "http://127.0.0.1",
                "http://user:pw@127.0.0.1:8770", "http://127.0.0.1:8770/v1", "ftp://127.0.0.1:21", "127.0.0.1:8770"):
        try:
            brain_link.loopback(url)
            why = None
        except ValueError as e:
            why = str(e)
        check(f"refuses the brain at {url}", why is not None, why)
    fake = FakeBrain()
    try:
        link = brain_link.BrainLink(BRAIN_URL, plan_timeout=1.0)
        known = link.hello()
        check("hello: the brain's name, that it plans, and its skills", known["ok"] and known["brain"] == stub_brain.NAME
              and known["planner"] and "seek" in known["skills"] and link.status()["ok"], known)
        fields = {k: v for k, v in plan_request(goal=A[:5] + [A[5] + 10.0]).items() if k not in ("v", "type", "id")}
        _, reply = link.ask("plan", fields)
        check("a plan comes back, checked", reply["ok"] and reply["path"][0] == fields["state"]["joints"], reply)
        fake.delay = {"plan": 2.5}
        t0 = time.monotonic()
        try:
            link.ask("plan", fields)
            why, took = None, None
        except brain_link.BrainError as e:
            why, took = str(e), time.monotonic() - t0
        check("a brain slower than the deadline: 'didn't answer within 1 s', at 1 s", why and "within 1 s" in why
              and 0.95 <= took <= 1.5, (why, took))
        t0 = time.monotonic()
        try:
            link.ask("plan", fields, should_abort=lambda: "STOP was pressed" if time.monotonic() - t0 > 0.3 else None)
            why = None
        except brain_link.Abandoned as e:
            why = e.reason
        took = time.monotonic() - t0
        check("waiting is abandoned at once for STOP (within 0.15 s of it)", why == "STOP was pressed" and took < 0.45,
              (why, took))
        fake.delay = {}
        fake.misbehave = only("plan", lambda q, r: 500)
        try:
            link.ask("plan", fields)
            why = None
        except brain_link.BrainError as e:
            why = str(e)
        check("an HTTP error is the brain not answering", why and "HTTP 500" in why, why)
        fake.misbehave = only("plan", lambda q, r: b" " * (bp.MAX_BYTES + 10))
        e = refusal(lambda: link.ask("plan", fields))
        check("a reply over 1 MiB is refused", e is not None and "over the" in str(e), e)
        fake.misbehave = only("plan", lambda q, r: dict(r, id="plan-0-0"))
        e = refusal(lambda: link.ask("plan", fields))
        check("a stale reply is refused as 'the brain's reply answers request ...'",
              e is not None and str(e).startswith("the brain's reply answers request") and "stale" in str(e), e)
        check("every request the link sent passed the brain's checks", not fake.bad_requests, fake.bad_requests)
    finally:
        fake.close()
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    free = probe.getsockname()[1]
    probe.close()
    known = brain_link.BrainLink(f"http://127.0.0.1:{free}").hello()
    check("with nothing listening, hello says the brain isn't answering", not known["ok"] and "isn't answering"
          in known["error"], known)

# ---- the shell, in-process ------------------------------------------------------------------------------------------

def unit_shell():
    print("== the shell: every path and burst checked whole, refused with every reason it fails")
    planner = plannermod.Planner(cell_model())
    stub = stub_brain.StubBrain()
    reply = stub.handle(plan_request())
    check("the stub brain plans up and over the post", reply["ok"] and len(reply["path"]) >= 3
          and "up and over" in reply["note"], reply)
    path = reply["path"]
    plan = safety.check_path(planner, path, A, 20.0, TOOL, 250.0, "C (offset)", goal=C_UP)
    check("its path becomes a plan of one joint move per segment, from exactly where the arm is to exactly the goal, "
          "each at most the step's speed", len(plan.segments) == len(path) - 1 and plan.start == A and plan.goal == C_UP
          and all(s[1] <= 20.0 for s in plan.segments) and abs(plan.duration - sum(plan.segment_durations)) < 1e-9,
          plan.text)
    worst = 0.0
    for (goal, speed, _), a in zip(plan.segments, plan.path):
        lead = max(abs(g - s) for s, g in zip(a, goal))
        points = [tool_mm([s + (g - s) * k / 400 for s, g in zip(a, goal)]) for k in range(401)]
        worst = max(worst, max(math.dist(p, q) for p, q in zip(points, points[1:])) / (lead / 400) * speed)
    check(f"sampled independently, the tool point never passes 250 mm/s at the speeds sent ({worst:.0f} mm/s at most)",
          worst <= 250.0, worst)
    fast = planner.joint(A, path[1], 120.0, "up", tool=TOOL, tool_speed=250.0)
    check("and the cap is what holds it: a lift asked for at 120\u00b0/s is sent far slower, saying why",
          fast.segments[0][1] < 60.0 and "under 250 mm/s" in fast.text, fast.text)
    blind = stub_brain.StubBrain(ignore_zones=True).handle(plan_request())
    check("a zone-blind brain's path goes straight through the post", blind["ok"] and len(blind["path"]) == 2, blind)

    j3 = A[:2] + [170.0] + A[3:]
    cases = [("straight through the post zone", blind["path"], ["segment 1", "the tool would come within 10 mm of the "
                                                                         "post zone"]),
             ("into the cart", [A, cart_via(A), C_UP], ["segment 1", "a cart base bar"]),
             ("past J3's limit", [A, j3, C_UP], ["waypoint 1 has J3 at 170.0\u00b0, outside -165\u00b0 to 165\u00b0"]),
             ("from somewhere else", [[A[0] + 0.5] + A[1:]] + path[1:], ["starts 0.50\u00b0 (J1) from where the arm is"]),
             ("to somewhere else", path[:-1] + [C_UP[:5] + [C_UP[5] + 1.0]], ["ends 1.00\u00b0 (J6) from C (offset)"])]
    for name, bad, words in cases:
        e = refusal(lambda: safety.check_path(planner, bad, A, 20.0, TOOL, 250.0, "C (offset)", goal=C_UP))
        check(f"refuses a path {name}, saying so: {words[-1]}", e is not None and all(w in str(e) for w in words), e)
    e = refusal(lambda: safety.check_path(planner, [[A[0] + 0.5] + A[1:], j3, C_UP], A, 20.0, TOOL, 250.0, "C",
                                          goal=C_UP))
    check("gives every reason a path fails, not just the first", e is not None and len(e.reasons) == 2, e)
    e = refusal(lambda: safety.check_path(planner, blind["path"], A, 20.0, TOOL, 250.0, "C", goal=C_UP))
    check("and where it would first have collided, for the 3D view", e is not None and e.hit
          and e.hit["other"] == "the post zone" and len(e.hit["pose"]) == 6, e and e.hit)

    near = (A_MM, "A")

    def burst(points, within=60.0):
        return safety.check_path(planner, points, A, 20.0, TOOL, 250.0, "seek's next pose", what="burst 2", near=near,
                                 within=within, burst=True)
    plan = burst([A, moved(A, dx=20)])
    check("a short burst near its target passes", len(plan.segments) == 1, plan.text)
    for name, points, within, words in [
            ("taking the tool point 45 mm from A, in a 30 mm region", [A, moved(A, dx=45)], 30.0, "beyond the 30 mm"),
            ("turning J6 20\u00b0", [A, A[:5] + [A[5] + 20.0]], 60.0, "turns J6 20.0\u00b0, more than the 15\u00b0"),
            ("of 60 mm", [A, moved(A, dx=60)], 100.0, "more than the 50 mm a burst may"),
            ("of 66 mm there and back, though it ends 6 mm away", [A, moved(A, dx=30), moved(A, dx=-6)], 100.0,
             "more than the 50 mm a burst may")]:
        e = refusal(lambda: burst(points, within))
        check(f"refuses a burst {name}, saying so", e is not None and words in str(e) and "burst 2" in str(e), e)

    check("just before it runs: nothing changed, so it may", refusal(lambda: safety.check_still("plan", A, A, 0.05, 3, 3))
          is None)
    for name, args, words in [("the arm has moved 0.2\u00b0", (A, A[:1] + [A[1] + 0.2] + A[2:], 0.05, 3, 3),
                               "has moved 0.20\u00b0 (J2)"),
                              ("the status is 1.5 s old", (A, A, 1.5, 3, 3), "stale (1.5 s old)"),
                              ("there's no status", (A, None, math.inf, 3, 3), "stale"),
                              ("the cell has changed", (A, A, 0.05, 3, 4), "cell changed (revision 3 to 4)")]:
        e = refusal(lambda: safety.check_still("plan", *args))
        check(f"just before it runs: refused when {name}", e is not None and words in str(e), e)

# ---- against the simulator ------------------------------------------------------------------------------------------

def scenario_end_to_end():
    print("== taught, planned, taught: the stub brain, a separate process, plans the middle around a keep-out zone")
    rig = Rig("e2e", brain="stub")
    try:
        rig.enable()
        setup(rig)
        save(rig, TEND)
        s = rig.wait(lambda s: s["brain"].get("ok"), 10)
        check("the panel's status says which brain answers, and its skills", s["brain"]["ok"]
              and s["brain"]["brain"] == stub_brain.NAME and "seek" in s["brain"]["skills"]
              and s["brain"]["url"] == BRAIN_URL, s["brain"])
        writes = rig.log().count("WRITE")
        r = rig.post("/api/run/start", {"name": "tend", "mode": "dry"})
        s = rig.run_state("done", "faulted", "stopped")
        run = rig.get("/api/run")["run"]
        planned = [e for e in run["events"] if e["event"] == "brain.plan"]
        check("a dry run asks the brain for the path, checks it, estimates the run, and sends the arm nothing",
              r["ok"] and s["run"]["state"] == "done" and planned and planned[0]["waypoints"] >= 3
              and run["estimateS"] > 4 and rig.log().count("WRITE") == writes, (s["run"], run["estimateS"], planned))
        finish = sampler(rig)
        r = rig.post("/api/run/start", {"name": "tend"})
        s = rig.run_state("done", "faulted", "stopped", timeout=90)
        samples = finish()
        events = rig.run_log()
        plans = [e for e in events if e["event"] == "brain.plan"]
        sent = [e for e in events if e["event"] == "move.sent" and e.get("path") == "2"]
        check("the run goes taught, planned, taught to the end", r["ok"] and s["run"]["state"] == "done"
              and [e["text"] for e in events if e["event"] == "step.start"] == ["Joint move to A",
                                                                                "Planned move to C with an offset",
                                                                                "Straight line to C"], s["run"])
        check("the planned step ran the brain's path, one supervised joint move per segment", len(plans) == 1
              and len(sent) == plans[0]["waypoints"] - 1 >= 2, (plans, len(sent)))
        model = cell_model()
        gaps = [min(g for g, _, other in model.clearances(q) if other == "the post zone") for _, q in samples]
        lifted = max(tool_mm(q)[2] for path, q in samples if path == [1])
        top = kin.position(kin.mul(FRAME, kin.pose_to_matrix({"xyz": [0, 130, 200], "rpy": [0, 0, 0]})))[2]
        check(f"sampled {len(samples)} times as it ran, the arm and tool never came within 10 mm of the post "
              f"(closest {min(gaps) * 1000:.1f} mm); the planned move went over it ({lifted:.0f} mm, its top {top:.0f})",
              len(samples) > 100 and min(gaps) >= 0.0099 and lifted > top + 10, (min(gaps), lifted, top))
        end = tool_mm(rig.sim_joints())
        goal = kin.position(target_pose(POINTS["C"]))
        check("and the arm ends with the tool point on C, within 0.1 mm", math.dist(end, goal) < 0.1, (end, goal))
    finally:
        rig.close()


def scenario_blind_brain():
    print("== a buggy brain whose world model lost the keep-out zones: its path is refused, and nothing moves")
    rig = Rig("blind", brain="stub", brain_args=["--ignore-zones"])
    try:
        rig.enable()
        setup(rig)
        save(rig, TEND)
        r = rig.post("/api/run/start", {"name": "tend"})
        s = rig.run_state("faulted", "done", "stopped", timeout=60)
        run = s["run"]
        refused = [e for e in rig.run_log() if e["event"] == "brain.refused"]
        check("the run faults at the planned step: the brain's plan refused, saying where it would hit the zone",
              r["ok"] and run["state"] == "faulted" and run["path"] == [1] and run["reason"].startswith(
                  "step 2: refused the brain's plan: segment 1: the tool would come within 10 mm of the post zone"),
              run)
        check("the log has the refusal with its reasons", refused and refused[0]["reasons"][0].startswith("segment 1"),
              refused)
        check("nothing of the path was sent: one joint move (step 1), no line, and the arm stopped",
              rig.moves() == 1 and rig.log().count("MOVET_LINE: goal=") == 0 and rig.ctl(cmd="state")["status"] == 4,
              (rig.moves(), rig.ctl(cmd="state")["status"]))
        hit = run.get("hit") or {}
        obstacles = rig.get("/api/cell")["collision"]["obstacles"]
        check("the 3D view is told where: the zone's index in its collision model and the pose",
              isinstance(hit.get("obstacle"), int) and obstacles[hit["obstacle"]]["label"] == "the post zone"
              and len(hit.get("pose", [])) == 6, hit)
    finally:
        rig.close()

# ---- bad replies --------------------------------------------------------------------------------------------------

def scenario_bad_replies():
    print("== stale, malformed and dangerous replies: each refused, saying why, with nothing sent")
    rig = Rig("bad", brain="fake", server_args=["--brain-timeout", "3"])
    try:
        rig.enable()
        setup(rig)
        save(rig, HOP)
        goto(rig, "A")

        def nan_reply(q, r):
            return json.dumps(dict(r, path=[r["path"][0], r["path"][-1][:5] + [float("nan")]])).encode()

        def huge_reply(q, r):
            return json.dumps(r).replace(f'"path": [[{r["path"][0][0]}', f'"path": [[1e999').encode()

        def nudge():
            rig.ctl(cmd="nudge", joints=[0.5, 0, 0, 0, 0, 0])

        def edit_cell():
            rev = rig.get("/api/cell")["cell"]["revision"]
            rig.post("/api/cell/zone", {"name": "far", "frame": "base", "pose": {"xyz": [-900, 0, -500], "rpy": [0, 0, 0]},
                                        "size": [10, 10, 10], "revision": rev})

        plan = lambda fn: only("plan", fn)  # noqa: E731
        cases = [
            ("a reply that isn't JSON", plan(lambda q, r: b"<html>busy</html>"), 0, None,
             "the brain's reply isn't JSON"),
            ("a reply in protocol version 2", plan(lambda q, r: dict(r, v=2)), 0, None, "speaks protocol version 2"),
            ("a stale reply: another request's", plan(lambda q, r: dict(r, id="plan-000000-1")), 0, None,
             "answers request \"plan-000000-1\""),
            ("a reply worked out from an older state", plan(lambda q, r: dict(r, stateSeq=r["stateSeq"] - 3)), 0, None,
             "not from the state it was sent"),
            ("NaN in the path", plan(nan_reply), 0, None, "isn't JSON (NaN"),
            ("an angle too big to be finite", plan(huge_reply), 0, None, "isn't finite"),
            ("a waypoint of five joints", plan(lambda q, r: dict(r, path=[r["path"][0], r["path"][-1][:5]])), 0, None,
             "not 6 joint angles"),
            ("65 waypoints", plan(lambda q, r: dict(r, path=[r["path"][0]] * 64 + [r["path"][-1]])), 0, None,
             "65 waypoints; it needs 2 to 64"),
            ("a path straight through the keep-out zone", plan(lambda q, r: dict(r, path=[r["path"][0], r["path"][-1]])),
             0, None, "the post zone"),
            ("a path into the cart", plan(lambda q, r: dict(r, path=[r["path"][0], cart_via(r["path"][0]),
                                                                     r["path"][-1]])), 0, None, "a cart base bar"),
            ("a waypoint past J3's limit", plan(lambda q, r: dict(r, path=[r["path"][0], r["path"][0][:2] + [170.0]
                                                                           + r["path"][0][3:], r["path"][-1]])), 0, None,
             "J3 at 170.0"),
            ("a path from elsewhere", plan(lambda q, r: dict(r, path=[[r["path"][0][0] + 2.0] + r["path"][0][1:]]
                                                             + r["path"][1:])), 0, None, "starts 2.00\u00b0 (J1)"),
            ("a path to elsewhere", plan(lambda q, r: dict(r, path=r["path"][:-1] + [r["path"][-1][:5]
                                                                                     + [r["path"][-1][5] + 1.0]])), 0, None,
             "ends 1.00\u00b0 (J6)"),
            ("no path, with the brain's reason", plan(lambda q, r: bp.reply(q, ok=False, reason="no room to pass")), 0,
             None, "the brain found no path to C (offset): no room to pass"),
            ("HTTP 500", plan(lambda q, r: 500), 0, None, "answered HTTP 500"),
            ("a reply over 1 MiB", plan(lambda q, r: b" " * (bp.MAX_BYTES + 1)), 0, None, "over the"),
            ("no reply within the 3 s deadline", None, 4.0, None, "didn't answer within 3 s"),
            ("a good path, but the arm was knocked while the brain thought", None, 0.5, nudge,
             "the arm has moved 0.50\u00b0 (J1) since the brain was asked"),
            ("a good path, but the cell changed while the brain thought", None, 0.5, edit_cell, "the cell changed")]
        for name, misbehave, delay, during, words in cases:
            rig.post("/api/run/reset")
            rig.enable()
            before, asked = rig.moves(), len(rig.fake.of("plan"))
            rig.fake.misbehave, rig.fake.delay = misbehave, ({"plan": delay} if delay else {})
            r = rig.post("/api/run/start", {"name": "hop"})
            if during is not None:  # once the brain has the question, while it thinks
                deadline = time.monotonic() + 5
                while len(rig.fake.of("plan")) <= asked and time.monotonic() < deadline:
                    time.sleep(0.01)
                during()
            s = rig.run_state("faulted", "done", "stopped", timeout=20)
            rig.fake.misbehave, rig.fake.delay = None, {}
            check(f"{name}: refused ('{words}'), nothing sent", r["ok"] and s["run"]["state"] == "faulted"
                  and words in s["run"]["reason"] and rig.moves() == before, (r.get("message"), s["run"]))
        check("every request the spine sent passed the brain's own checks", not rig.fake.bad_requests,
              rig.fake.bad_requests)
        rig.post("/api/run/reset")
        rig.enable()
        r = rig.post("/api/run/start", {"name": "hop"})
        s = rig.run_state("done", "faulted", "stopped", timeout=60)
        check("with the brain behaving again, the same step runs", r["ok"] and s["run"]["state"] == "done", s["run"])
    finally:
        rig.close()

# ---- STOP ---------------------------------------------------------------------------------------------------------

def scenario_stop():
    print("== STOP is absolute: while the brain thinks and while its path runs; pause and resume plan afresh")
    rig = Rig("stop", brain="fake", server_args=["--brain-timeout", "5"])
    try:
        rig.enable()
        setup(rig)
        save(rig, dict(HOP, name="slowhop", speed={"joint": 10}))
        goto(rig, "A")

        rig.fake.delay = {"plan": 2.0}
        before, asked = rig.moves(), len(rig.fake.of("plan"))
        rig.post("/api/run/start", {"name": "slowhop"})
        deadline = time.monotonic() + 5
        while len(rig.fake.of("plan")) <= asked and time.monotonic() < deadline:
            time.sleep(0.01)
        shown = rig.status()["run"]["step"]
        t0 = time.monotonic()
        r = rig.post("/api/stop")
        s = rig.run_state("stopped", timeout=3)
        took = time.monotonic() - t0
        check(f"while the brain plans, the run says so ('{shown}'); STOP ends the run at once ({took:.2f} s), without "
              "waiting for the brain", "asking the brain" in (shown or "") and r["ok"] and s["run"]["state"] == "stopped"
              and took < 0.6, (shown, took, s["run"]))
        time.sleep(2.5)
        check("the brain's reply, when it comes, is dropped: nothing is sent and the arm stays stopped",
              len(rig.fake.of("plan")) == asked + 1 and rig.moves() == before and rig.ctl(cmd="state")["status"] == 4,
              (rig.moves(), before, rig.ctl(cmd="state")["status"]))
        rig.fake.delay = {}

        rig.enable()
        before = rig.moves()
        rig.post("/api/run/start", {"name": "slowhop"})
        rig.wait(lambda s: s.get("state") == 1 and s["run"]["state"] == "running", 20, 0.02)
        time.sleep(0.5)
        r = rig.post("/api/stop")
        state = rig.ctl(cmd="state")["status"]
        s = rig.run_state("stopped", timeout=3)
        sent = rig.moves()
        time.sleep(0.7)  # past the STOP's settling (0.5 s on the arm)
        held = rig.sim_joints()
        time.sleep(1.5)
        check("STOP while the brain's path runs: the controller is stopped by the time STOP answers, and the run ends "
              "stopped", r["ok"] and state == 4 and s["run"]["state"] == "stopped" and sent > before,
              (state, s["run"], sent, before))
        check("no later segment of the path is sent, and the arm stays put", rig.moves() == sent
              and max(abs(a - b) for a, b in zip(held, rig.sim_joints())) < 1e-6, (rig.moves(), sent))

        rig.enable()
        plans = len(rig.fake.of("plan"))
        rig.post("/api/run/start", {"name": "slowhop"})
        rig.wait(lambda s: s.get("state") == 1 and s["run"]["state"] == "running", 20, 0.02)
        time.sleep(0.8)
        rig.post("/api/run/pause")
        rig.run_state("paused", timeout=10)
        time.sleep(0.6)
        rig.post("/api/run/resume")
        s = rig.run_state("done", "faulted", "stopped", timeout=90)
        asks = rig.fake.of("plan")[plans:]
        end = tool_mm(rig.sim_joints())
        goal = kin.position(target_pose(POINTS["C"], back=50.0))
        check("pause mid-path, then resume: the brain is asked again from where the arm stopped, and the arm arrives "
              "at the goal (within 0.1 mm)", s["run"]["state"] == "done" and len(asks) == 2
              and max(abs(a - b) for a, b in zip(asks[0]["state"]["joints"], asks[1]["state"]["joints"])) > 0.5
              and math.dist(end, goal) < 0.1, (s["run"], len(asks), end, goal))
    finally:
        rig.close()

# ---- learned steps --------------------------------------------------------------------------------------------------

def learned(name, within=60.0, seconds=30.0, params=None, check_input=None):
    """A program: a taught move to A, the stub's seek skill near A, then a taught move home."""
    step = {"type": "learned", "skill": "seek", "near": "A", "within": within, "seconds": seconds}
    if params:
        step["params"] = params
    if check_input:
        step["check"] = {"input": check_input, "value": 1}
    return {"name": name, "speed": {"joint": 30}, "steps": [{"type": "move", "kind": "joint", "target": "A"}, step,
                                                             {"type": "move", "kind": "joint", "target": "home"}]}


def ended(rig, episodes_before, timeout=3.0):
    """The end messages the brain got since `episodes_before`, waiting a little for the last (it's sent unawaited)."""
    deadline = time.monotonic() + timeout
    while len(rig.fake.of("end")) <= episodes_before and time.monotonic() < deadline:
        time.sleep(0.05)
    return rig.fake.of("end")[episodes_before:]


def run_learned(rig, name, until=None, timeout=60.0):
    """Runs a program from a fresh start; `until` (called with the act requests so far) may act mid-run."""
    rig.post("/api/run/reset")
    rig.enable()
    marks = {"moves": rig.moves(), "acts": len(rig.fake.of("act")), "ends": len(rig.fake.of("end"))}
    r = rig.post("/api/run/start", {"name": name})
    if until is not None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not until(rig.fake.of("act")[marks["acts"]:]):
            time.sleep(0.02)
    s = rig.run_state("done", "faulted", "stopped", timeout=timeout)
    return r, s["run"], marks


def scenario_learned():
    print("== learned steps: a skill acts in short bursts near its target, each checked; how it ends is checked too")
    rig = Rig("learned", brain="fake")
    try:
        rig.enable()
        setup(rig)
        save(rig, learned("seek", check_input="tool.1"))
        finish = sampler(rig)

        def found(asks):
            if len(asks) >= 4:  # three bursts done: the sensor sees the part
                rig.ctl(cmd="input", line="tool", bit=1, value=1)
                return True
            return False
        r, run, marks = run_learned(rig, "seek", until=found)
        samples = finish()
        rig.ctl(cmd="input", line="tool", bit=1, value=0)
        events = rig.run_log()
        bursts = [e for e in events if e["event"] == "skill.burst"]
        asks = [(t, q) for t, q in rig.fake.requests if q["type"] == "act"][marks["acts"]:]
        ends = ended(rig, marks["ends"])
        far = max(math.dist(tool_mm(q), A_MM) for path, q in samples if path == [1])
        check("the skill moves in bursts until tool.1 goes high; the step's check reads it, and the run goes on home",
              r["ok"] and run["state"] == "done" and len(bursts) >= 3 and any(e["event"] == "skill.done" for e in events)
              and rig.moves() == marks["moves"] + 1 + len(bursts) + 1, (run, len(bursts)))
        check(f"sampled as it ran, the tool point stayed inside the step's 60 mm region around A ({far:.1f} mm at most)",
              0 < far <= 60.0, far)
        gaps = [b[0] - a[0] for a, b in zip(asks, asks[1:])]
        seqs = [q["state"]["seq"] for _, q in asks]
        check("the skill was asked at most 10 times a second, each time with a newer state (ticks 0, 1, 2, ...), its "
              "inputs, its region and its time left", min(gaps) >= 0.095 and [q["tick"] for _, q in asks]
              == list(range(len(asks))) and seqs == sorted(set(seqs)) and "inputs" in asks[0][1]["state"]
              and asks[0][1]["withinMm"] == 60 and asks[0][1]["remainingS"] > 25, (gaps, seqs))
        check("and the brain was told the episode ended: done", ends and ends[-1]["outcome"] == "done"
              and ends[-1]["episode"] == asks[0][1]["episode"], ends)

        save(rig, learned("roam", within=30.0, params={"stepMm": 12, "pitchMm": 60, "radiusMm": 150}))
        r, run, marks = run_learned(rig, "roam")
        bursts = [e for e in rig.run_log() if e["event"] == "skill.burst"]
        check("a skill straying from its region: the burst that would leave it is refused, saying so, and not sent",
              run["state"] == "faulted" and "beyond the 30 mm the step allows" in run["reason"]
              and rig.moves() == marks["moves"] + 1 + len(bursts) and len(bursts) >= 2, (run, len(bursts)))
        check("and the brain is told: refused", [e["outcome"] for e in ended(rig, marks["ends"])] == ["refused"])

        save(rig, learned("claims", params={"doneAfter": 2}, check_input="tool.2"))
        r, run, marks = run_learned(rig, "claims")
        check("a skill that says it's done when the step's check reads otherwise faults the run",
              run["state"] == "faulted" and "the skill seek said it was done, but tool.2 reads 0, not 1" in run["reason"]
              and [e["outcome"] for e in ended(rig, marks["ends"])] == ["failed"], run)

        save(rig, learned("slow", seconds=2.0))
        t0 = time.monotonic()
        r, run, marks = run_learned(rig, "slow")
        check("a skill that runs out of time faults the run, and the brain is told: timeout", run["state"] == "faulted"
              and "didn't finish within 2 s" in run["reason"] and time.monotonic() - t0 < 12
              and [e["outcome"] for e in ended(rig, marks["ends"])] == ["timeout"], run)

        save(rig, learned("forever", seconds=60.0))
        stop = {}

        def two_bursts(asks):
            if len(asks) >= 3 and "at" not in stop:
                time.sleep(0.2)  # mid-burst, or nearly
                stop["at"], stop["reply"] = time.monotonic(), rig.post("/api/stop")
                stop["state"], stop["moves"] = rig.ctl(cmd="state")["status"], rig.moves()
                return True
            return False
        r, run, marks = run_learned(rig, "forever", until=two_bursts)
        time.sleep(1.5)
        check("STOP during a learned step: the controller is stopped at once, the run ends stopped, no burst follows, "
              "and the brain is told: stopped", run["state"] == "stopped" and stop.get("state") == 4
              and rig.moves() == stop.get("moves") and [e["outcome"] for e in ended(rig, marks["ends"])] == ["stopped"],
              (run, stop))

        rig.fake.misbehave = only("act", lambda q, r: dict(r, action={"kind": "move", "path": [
            q["state"]["joints"], moved(q["state"]["joints"], dx=60)]}))
        save(rig, learned("leap", within=200.0))
        r, run, marks = run_learned(rig, "leap")
        rig.fake.misbehave = None
        check("a brain proposing one 60 mm leap: refused as more than a burst may move, nothing of it sent",
              run["state"] == "faulted" and "more than the 50 mm a burst may" in run["reason"]
              and rig.moves() == marks["moves"] + 1, run)

        held = {"n": 0}

        def hold_then_done(q, r):
            held["n"] += 1
            return dict(r, action={"kind": "hold", "seconds": 0} if held["n"] <= 3 else {"kind": "done", "note": "ok"})
        rig.fake.misbehave = only("act", hold_then_done)
        save(rig, learned("hold"))
        r, run, marks = run_learned(rig, "hold")
        rig.fake.misbehave = None
        times = [t for t, q in rig.fake.requests if q["type"] == "act"][marks["acts"]:]
        gaps = [b - a for a, b in zip(times, times[1:])]
        # The spine sends them at least 0.1 s apart; the brain stamps them on arrival, and the loopback's latency
        # varies by some milliseconds when the PC is busy, so each gap is judged within 15 ms, and their average too.
        check("a skill that holds still is asked again no sooner than 0.1 s later (10 Hz at most), then finishes",
              run["state"] == "done" and len(times) == 4 and min(gaps) >= 0.085 and sum(gaps) / len(gaps) >= 0.095,
              (run, gaps))
        check("every request the spine sent passed the brain's own checks", not rig.fake.bad_requests,
              rig.fake.bad_requests)
    finally:
        rig.close()

# ---- without a brain, and the program model -----------------------------------------------------------------------

def scenario_no_brain():
    print("== no brain: programs that need one won't start, saying why; others run; the program model's checks")
    rig = Rig("nobrain", brain=None)
    try:
        rig.enable()
        setup(rig)
        save(rig, TEND)
        save(rig, {"name": "plain", "steps": [{"type": "move", "kind": "joint", "target": "A"},
                                              {"type": "move", "kind": "joint", "target": "home"}]})
        s = rig.wait(lambda s: s["brain"].get("ok") is False, 10)
        check("the status says the brain isn't answering", s["brain"]["ok"] is False
              and "isn't answering" in s["brain"]["error"], s["brain"])
        for mode in ("run", "dry"):
            r = rig.post("/api/run/start", {"name": "tend", "mode": mode})
            check(f"a program with a planned move won't start ({mode}), saying the brain isn't answering and how to start "
                  "it", not r["ok"] and "isn't answering" in r["message"] and "stub_brain.py" in r["message"]
                  and rig.status()["run"]["state"] == "idle", r)
        r = rig.post("/api/run/start", {"name": "plain"})
        s = rig.run_state("done", "faulted", "stopped", timeout=60)
        check("a program without brain steps runs as before", r["ok"] and s["run"]["state"] == "done", s["run"])
        r = rig.post("/api/plan", {"target": "C", "kind": "planned"})
        check("a planned move can't be staged from the page: its path comes from the brain, in a run", not r["ok"]
              and "comes from the brain" in r["message"], r)

        every = {"name": "every", "steps": [
            {"type": "move", "kind": "planned", "target": "C", "offset": {"frame": "tool", "xyz": [0, 0, -50]}, "speed": 15},
            {"type": "learned", "skill": "seek", "near": "A", "within": 40, "seconds": 20, "speed": 10,
             "params": {"stepMm": 5}, "check": {"input": "tool.1", "value": 1}},
            {"type": "learned", "skill": "seek", "near": "home"},
            {"type": "repeat", "count": 2, "steps": [{"type": "move", "kind": "planned", "target": "home"}]}]}
        r = rig.post("/api/program", {"program": every})
        loaded = rig.get("/api/program?name=every")["program"]
        check("planned and learned steps save (the learned step's defaults filled in: 100 mm, 60 s) and load back "
              "exactly", r["ok"] and loaded == r["program"] and loaded["steps"][2]["within"] == 100
              and loaded["steps"][2]["seconds"] == 60 and loaded["steps"][1]["params"] == {"stepMm": 5}, (r, loaded))
        r2 = rig.post("/api/program", {"program": loaded, "revision": loaded["revision"]})
        check("saving what was loaded changes nothing but the revision", r2["ok"] and {
            k: v for k, v in r2["program"].items() if k != "revision"} == {k: v for k, v in loaded.items()
                                                                         if k != "revision"}, r2)
        for step, words in [({"type": "move", "kind": "flying", "target": "A"}, "a move is 'joint', 'line' or 'planned'"),
                            ({"type": "move", "kind": "planned", "target": "nowhere"}, "There's no target called 'nowhere'"),
                            ({"type": "learned", "skill": "9lives", "near": "A"}, "Unsupported skill name"),
                            ({"type": "learned", "skill": "seek", "near": "nowhere"}, "There's no target called 'nowhere'"),
                            ({"type": "learned", "skill": "seek", "near": "A", "within": 1000}, "region (mm)"),
                            ({"type": "learned", "skill": "seek", "near": "A", "seconds": 0}, "time limit (s)"),
                            ({"type": "learned", "skill": "seek", "near": "A", "params": [1]}, "params is [1]"),
                            ({"type": "learned", "skill": "seek", "near": "A", "check": {"input": "tool.99", "value": 1}},
                             "Unsupported input"),
                            ({"type": "learned", "skill": "seek", "near": "A", "speed": 500}, "Unsupported speed")]:
            r = rig.post("/api/program", {"program": {"name": "bad", "steps": [step]}})
            check(f"refuses {json.dumps(step)[:90]}, saying where and why", not r["ok"] and "step 1" in r["message"]
                  and words in r["message"] and "bad" not in rig.get("/api/programs")["programs"], r)
        save(rig, {"name": "near", "steps": [{"type": "learned", "skill": "seek", "near": "C"}]})
        r = rig.post("/api/cell/target/delete", {"name": "C", "revision": rig.get("/api/cell")["cell"]["revision"]})
        check("a target that planned moves and learned steps use can't be deleted, naming the programs", not r["ok"]
              and "program near" in r["message"] and "program tend" in r["message"], r)
    finally:
        rig.close()
    r = subprocess.run([PY, SERVER, "127.0.0.1", "--port", str(HTTP_PORT), "--brain", "http://10.0.0.5:8770"], cwd=ROOT,
                       capture_output=True, text=True, timeout=60)
    check("the server won't start with a brain off this PC (the link has no authentication)", r.returncode != 0
          and "loopback" in r.stderr, (r.returncode, r.stderr[-300:]))


# ---- run logs ------------------------------------------------------------------------------------------------------

def same_second_runs(rig, name, count, tries=5):
    """Dry runs of program `name`, one straight after another, started just after a second begins, until `count` of
    them started within one second (their logs' names say when, to the second): their logs, relative to cell/logs/."""
    for _ in range(tries):
        time.sleep(1.0 - time.time() % 1.0 + 0.02)
        logs = []
        for _ in range(count):
            r = rig.post("/api/run/start", {"name": name, "mode": "dry"})
            s = rig.run_state("done", "faulted", "stopped")
            assert r["ok"] and s["run"]["state"] == "done", (r, s["run"])
            logs.append(rig.get("/api/run")["run"]["log"])
        if len({os.path.basename(log).split("-", 1)[0] for log in logs}) == 1:
            break
    return logs


def scenario_run_logs():
    print("== run logs: runs of one program started within the same second each get a log of their own")
    import runner as runnermod
    rig = Rig("logs", brain=None)
    try:
        rig.enable()
        setup(rig)
        save(rig, {"name": "plain", "steps": [{"type": "move", "kind": "joint", "target": "A"},
                                              {"type": "move", "kind": "joint", "target": "home"}]})
        logs = same_second_runs(rig, "plain", 3)
        events = []
        for log in logs:
            with open(os.path.join(rig.cell_dir, "logs", log), encoding="utf-8") as f:
                events.append([json.loads(line) for line in f])
        names = [os.path.basename(log) for log in logs]
        stamp = names[0].split("-", 1)[0]
        starts = [next((e["t"][11:23] for e in ev if e["event"] == "run.start"), "?") for ev in events]
        check(f"three dry runs of one program started within one second (at {', '.join(starts)}) get three logs: "
              f"{', '.join(names)}", names == [f"{stamp}-plain.jsonl", f"{stamp}-plain~2.jsonl",
                                               f"{stamp}-plain~3.jsonl"] and len(set(logs)) == 3, logs)
        kinds = [[e["event"] for e in ev] for ev in events]
        check("each log holds its own run only: one run.start, one run.done, and its own run's steps",
              all(k.count("run.start") == 1 and k.count("run.done") == 1 and k[0] == "run.start" and k[-1] == "run.done"
                  for k in kinds), kinds)
        folder = os.path.join(rig.cell_dir, "race")
        os.makedirs(folder)
        barrier, made = threading.Barrier(8), []

        def take():
            barrier.wait()
            made.append(runnermod.new_log(folder, stamp, "plain"))
        threads = [threading.Thread(target=take) for _ in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        check("eight runs of one program starting at once in one folder (two panels on one cell folder, say) get eight "
              "logs: each is created exclusively", len(set(made)) == 8 and sorted(os.listdir(folder)) == sorted(
                  os.path.basename(p) for p in made), made)
    finally:
        rig.close()


SCENARIOS = (unit_protocol, unit_link, unit_shell, scenario_end_to_end, scenario_blind_brain, scenario_bad_replies,
             scenario_stop, scenario_learned, scenario_no_brain, scenario_run_logs)


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
