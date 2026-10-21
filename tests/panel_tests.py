"""Tests for scripts/jog_server.py against tools/utra_sim.py. Usage: python tests/panel_tests.py [python-exe-for-server]"""
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

from simrig import HOST, HTTP_PORT, ROOT, SERVER, log_path, start_sim, wait_port

PY = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("UTRA_TEST_PY") or sys.executable
PORT = HTTP_PORT
BASE = f"http://127.0.0.1:{PORT}"
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)}")


def request(method, path, body=None, token=None, host=None, raw=None):
    data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
    req = urllib.request.Request(BASE + path, data=data, method=method)
    if body is not None or raw is not None:
        req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("X-Panel-Token", token)
    if host:
        req.add_header("Host", host)
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def status():
    code, body = request("GET", "/api/status")
    return json.loads(body) if code == 200 else None


def api(path, token, body=None):
    code, data = request("POST", path, body or {}, token)
    return json.loads(data) if code == 200 else {"ok": False, "code": code}


def wait_for(pred, timeout=6.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = status()
        if s and pred(s):
            return s
        time.sleep(0.1)
    return status()


def idle(s):
    return s.get("live") and s.get("state") in (0, 2) and s.get("queued") == 0 and not s.get("goal")


# Runs the server with three.js's folder, mirrors and hashes swapped for a test's (see scenario_three).
THREE_WRAPPER = ("import json, sys; sys.path.insert(0, sys.argv[1]); import jog_server as j; o = json.loads(sys.argv[2]); "
                 "j.THREE_DIR, j.THREE_MIRRORS, j.THREE_FILES = o['dir'], tuple(o['mirrors']), o['files']; "
                 "sys.exit(j.main(sys.argv[3:]))")


class Rig:
    def __init__(self, name, sim_args=(), controller=None, three=None):
        controller = controller or HOST
        self.name, self.log_path = name, log_path(f"panel_{name}.log")
        self.cell_dir = log_path(f"cell_{name}")
        shutil.rmtree(self.cell_dir, ignore_errors=True)
        self.sim = None
        self.output = ""  # the server's console, once closed
        if controller == HOST:
            self.sim, _ = start_sim(sim_args, f"panel_{name}.log")
        command = [PY, SERVER, controller, "--port", str(PORT), "--cell", self.cell_dir]
        if three is not None:
            command = [PY, "-c", THREE_WRAPPER, os.path.dirname(SERVER), json.dumps(three), controller, "--port", str(PORT),
                       "--cell", self.cell_dir]
        self.server = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.token = None
        if wait_port(PORT, 10, host="127.0.0.1"):
            code, html = request("GET", "/")
            m = re.search(r'const TOKEN = "([^"]+)"', html.decode())
            self.token = m.group(1) if m else None

    def log(self):
        return open(self.log_path).read() if self.sim else ""

    def close(self):
        self.server.kill()
        self.server.wait()
        self.output = self.server.stdout.read()
        if self.sim:
            self.sim.kill()
            self.sim.wait()


def jog(rig, kind, index, direction, **kw):
    body = {"kind": kind, "index": index, "dir": direction}
    if kind == "joint":
        body.update(step=kw.get("step", 1.0), speed=kw.get("speed", 10.0))
    else:
        body.update(step=kw.get("step", 5.0), rotStep=kw.get("rot", 2.0), speed=kw.get("speed", 20.0))
    return api("/api/jog", rig.token, body)


def move(rig, joint, angle, speed=20.0):
    return api("/api/move", rig.token, {"joint": joint, "angle": angle, "speed": speed})


def fetch(path, host=None):
    """GET returning (status, headers, body)."""
    req = urllib.request.Request(BASE + path, method="GET")
    if host:
        req.add_header("Host", host)
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, r.headers, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers, e.read()


def scenario_assets():
    print("== 3D view assets and page config")
    rig = Rig("assets")
    try:
        code, headers, body = fetch("/")
        csp = headers.get("Content-Security-Policy", "")
        check("CSP lets the page load its own scripts only", "script-src 'self'" in csp and "default-src 'none'" in csp, csp)
        m = re.search(r"const CONFIG = (\{.*?\});\n", body.decode())
        config = json.loads(m.group(1)) if m else {}
        check("config names the arm model", (config.get("model") or {}).get("urdf") == "utra6_1000_robot.urdf", config.get("model"))
        check("config carries the drag limits", (config.get("dragLimits") or [0, 0, 0])[2] == [-165.0, 165.0], config.get("dragLimits"))
        check("config carries the detents and rated joint speeds",
              config.get("jointSpeeds", [])[-1] == 120.0 and config.get("toolSpeeds", [])[-1] == 1000.0
              and config.get("jointRated") == [84.0, 84.0, 114.5, 119.1, 119.1, 119.1], (config.get("jointSpeeds"), config.get("jointRated")))
        page = body.decode()
        remembered = re.search(r"const REMEMBERED = (\[.*?\]);", page)
        check("the page remembers steps only, so speeds start slow every time",
              remembered is not None and set(json.loads(remembered.group(1))) == {"jointStep", "toolStep", "rotStep"}
              and (config.get("defaults") or {}).get("jointSpeed") == 10 and (config.get("defaults") or {}).get("toolSpeed") == 20,
              (remembered and remembered.group(1), config.get("defaults")))
        check("config carries the 250 mm/s redline, a tool speed detent of its own, and the arm's reach",
              config.get("toolRedline") == 250.0 and 250.0 in config.get("toolSpeeds", []) and config.get("armReach") == 1000.0,
              (config.get("toolRedline"), config.get("toolSpeeds"), config.get("armReach")))
        s = wait_for(lambda s: s.get("live") and s.get("toolRadii"))
        radii = (s or {}).get("toolRadii") or []
        sys.path.insert(0, os.path.join(ROOT, "scripts"))
        import math
        import utra_model
        model = utra_model.ArmModel()
        joints = s["joints"] if s else [0.0] * 6

        def flange(q):
            return [model.frames(q)[-1][i][3] for i in range(3)]

        # Each radius is how far the flange moves per radian of that joint alone, measured numerically.
        measured = []
        for j in range(6):
            nudged = list(joints)
            nudged[j] += 1e-4
            measured.append(math.dist(flange(joints), flange(nudged)) / math.radians(1e-4) * 1000.0)
        check("status gives the flange's distance from each joint's axis",
              len(radii) == 6 and all(abs(r - m) < 0.2 for r, m in zip(radii, measured)), (radii, measured))
        f = flange(joints)
        check("and J1's is the flange's distance from the base's axis",
              radii and abs(radii[0] - math.hypot(f[0], f[1]) * 1000.0) < 0.2, (radii, f))
        collision = config.get("collision") or {}
        check("config carries the collision model", len(collision.get("capsules", [])) == 17 and len(collision.get("joints", [])) == 6
              and collision.get("margin") == 0.01 and len(collision.get("pairs", [])) > 100, list(collision))
        labels = {o["label"] for o in collision.get("obstacles", [])}
        check("the obstacles include the arm's base, the cart and the floor",
              {"the arm's base", "the cart's column", "the NTRO controller", "the mounting plate", "the floor"} <= labels, labels)
        check("the floor sits under the casters", abs(config.get("floorZ", 0) + 0.730) < 0.002, config.get("floorZ"))
        visuals = (config.get("cart") or {}).get("visuals") or []
        check("config lists the cart's meshes", [v["file"] for v in visuals] == ["meshes/frame.STL", "meshes/casters.STL",
                                                                               "meshes/controller.STL"], visuals)
        for path, kind in [("/web/arm_view.js", "text/javascript"), ("/web/collide.js", "text/javascript"),
                           ("/web/ik.js", "text/javascript"), ("/web/featureset.js", "text/javascript"),
                           ("/vendor/three/build/three.module.js", "text/javascript"),
                           ("/vendor/three/build/three.core.js", "text/javascript"),
                           ("/vendor/three/examples/jsm/controls/OrbitControls.js", "text/javascript"),
                           ("/vendor/three/examples/jsm/loaders/STLLoader.js", "text/javascript"),
                           ("/model/utra6_1000/utra6_1000_robot.urdf", "application/xml"),
                           ("/model/utra6_1000/meshes/link6.STL", "model/stl"),
                           ("/model/cart/utra_cart.urdf", "application/xml"),
                           ("/model/cart/meshes/frame.STL", "model/stl"),
                           ("/model/cart/meshes/casters.STL", "model/stl"),
                           ("/model/cart/meshes/controller.STL", "model/stl"),
                           ("/web/arm_view.js?v=2", "text/javascript")]:
            code, headers, body = fetch(path)
            check(f"serves {path}", code == 200 and headers.get("Content-Type", "").startswith(kind) and len(body) > 1000,
                  (code, headers.get("Content-Type"), len(body)))
        size = os.path.getsize(os.path.join(ROOT, "scripts", "model", "utra6_1000", "meshes", "link6.STL"))
        code, _, body = fetch("/model/utra6_1000/meshes/link6.STL")
        check("mesh served byte for byte", len(body) == size, (len(body), size))
        check("the page loads the program, teach and cell sections' module after its own script",
              page.rfind('<script type="module" src="/web/featureset.js"></script>') > page.rfind("poll();"))
        with open(os.path.join(ROOT, "scripts", "web", "featureset.js"), encoding="utf-8") as f:
            wanted = sorted(set(re.findall(r'\$\("([A-Za-z]+)"\)', f.read())))
        ids = re.findall(r'\sid="([A-Za-z]+)"', page)
        missing = [i for i in wanted if ids.count(i) != 1]
        check(f"every element the module looks up ({len(wanted)}) is in the page exactly once", wanted and not missing,
              missing)
        import hashlib
        import jog_server
        check("config says three.js is here", config.get("three") is True, config.get("three"))
        importmap = re.search(r'<script type="importmap">(.*?)</script>', page)
        imports = json.loads(importmap.group(1)).get("imports") if importmap else None
        check("the page's import map points three.js at this server's copy",
              imports == {"three": "/vendor/three/build/three.module.js", "three/addons/": "/vendor/three/examples/jsm/"}, imports)
        for name, digest in jog_server.THREE_FILES.items():
            if name.endswith(".js"):
                code, _, body = fetch(jog_server.THREE_URL + name)
                check(f"{name} is three.js {jog_server.THREE_VERSION} as published",
                      code == 200 and hashlib.sha256(body).hexdigest() == digest, (code, len(body)))
        for path in ["/web/../jog_server.py", "/web/%2e%2e/jog_server.py", "/web/..%2fjog_server.py",
                     "/model/utra6_1000/LICENSE", "/model/utra6_1000/collision.json", "/model/cart/SOURCE.txt",
                     "/vendor/three/LICENSE", "/web/vendor/three/three.module.js", "/vendor/three/build%2fthree.module.js",
                     "/vendor/three/build/../build/three.module.js", "/vendor/three/", "/vendor/three/build/three.cjs",
                     "/web//arm_view.js",
                     "/web/arm_view.js%00.js", "/web/arm_view.js::$DATA", "/model/../../scripts/arm.py",
                     "/web/C:%5cWindows%5cwin.ini", "/vendor%5cthree%5cbuild%5cthree.module.js", "/scripts/jog_server.py",
                     "/web/", "/web/missing.js"]:
            code, _, _ = fetch(path)
            check(f"refuses {path}", code == 404, code)
        code, _, _ = fetch("/web/arm_view.js", host=f"evil.example:{PORT}")
        check("assets refused for a foreign Host header", code == 403, code)
    finally:
        rig.close()


def scenario_three():
    print("== three.js: downloaded on the first run, checked at every start, served from this PC")
    import hashlib
    import shutil
    import tempfile
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    # Stand-ins for three.js's files, and a mirror on this PC: /good/ sends them as published, /altered/ the same but
    # with three.core.js changed. A port nothing listens on stands in for having no internet.
    files = {"build/three.module.js": b"export * from './three.core.js';\n" + b"// module\n" * 150,
             "build/three.core.js": b"export const REVISION = 'test';\n" + b"// core\n" * 150,
             "examples/jsm/controls/OrbitControls.js": b"import { Vector3 } from 'three';\n" + b"// controls\n" * 150,
             "examples/jsm/loaders/STLLoader.js": b"import { Loader } from 'three';\n" + b"// loader\n" * 150,
             "LICENSE": b"The MIT License\n"}
    hashes = {name: hashlib.sha256(data).hexdigest() for name, data in files.items()}
    scripts = [name for name in files if name.endswith(".js")]
    core = "build/three.core.js"
    hits = []

    class Mirror(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            kind, _, name = self.path.lstrip("/").partition("/")
            hits.append((kind, name))
            data = files.get(name) if kind in ("good", "altered") else None
            if data is not None and kind == "altered" and name == core:
                data += b"// altered\n"
            self.send_response(200 if data is not None else 404)
            self.send_header("Content-Length", str(len(data or b"")))
            self.end_headers()
            self.wfile.write(data or b"")

    mirror = ThreadingHTTPServer(("127.0.0.1", 0), Mirror)
    threading.Thread(target=mirror.serve_forever, daemon=True).start()
    good = f"http://127.0.0.1:{mirror.server_address[1]}/good/"
    altered = f"http://127.0.0.1:{mirror.server_address[1]}/altered/"
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    offline = f"http://127.0.0.1:{probe.getsockname()[1]}/"
    probe.close()
    root = tempfile.mkdtemp(prefix="three_test_")

    def on_disk(folder, name):
        try:
            with open(os.path.join(folder, *name.split("/")), "rb") as f:
                return f.read()
        except OSError:
            return None

    def served(name):
        code, _, body = fetch("/vendor/three/" + name)
        return body if code == 200 else None

    def start(name, folder, mirrors, during):
        """Start the server on `folder` and `mirrors`, run `during(rig, config)`, and return its console."""
        hits.clear()
        rig = Rig(f"three_{name}", three={"dir": folder, "mirrors": mirrors, "files": hashes})
        try:
            _, _, page = fetch("/")
            m = re.search(r"const CONFIG = (\{.*?\});\n", page.decode())
            during(rig, json.loads(m.group(1)) if m else {})
        finally:
            rig.close()
        return rig.output

    try:
        first = os.path.join(root, "first")

        def first_run(rig, config):
            check("first run: three.js downloaded and served", config.get("three") is True
                  and all(served(name) == files[name] for name in scripts), config.get("three"))
            check("every file fetched once and saved", sorted(name for _, name in hits) == sorted(files)
                  and all(on_disk(first, name) == files[name] for name in files), hits)
            check("the licence is kept, not served", served("LICENSE") is None)
        out = start("first", first, [good], first_run)
        check("first run says it's downloading, once", out.count("Downloading three.js") == 1 and "works offline" in out, out)

        def offline_run(rig, config):
            check("later runs work offline from the saved copy", config.get("three") is True
                  and all(served(name) == files[name] for name in scripts) and not hits, hits)
        out = start("cached", first, [offline], offline_run)
        check("and download nothing", "Downloading" not in out and "Warning" not in out, out)

        with open(os.path.join(first, "build", "three.core.js"), "ab") as f:
            f.write(b"// edited\n")

        def repair_run(rig, config):
            check("a file altered on disk is fetched again, alone", hits == [("good", core)]
                  and on_disk(first, core) == files[core] and served(core) == files[core], hits)
            with open(os.path.join(first, "build", "three.module.js"), "ab") as f:
                f.write(b"// edited while running\n")
            check("a file altered while the server runs isn't served", served("build/three.module.js") == files["build/three.module.js"])
        start("repair", first, [good], repair_run)

        second = os.path.join(root, "second")

        def fallback_run(rig, config):
            check("a mirror's altered copy is refused and the next mirror's used",
                  ("altered", core) in hits and ("good", core) in hits and on_disk(second, core) == files[core]
                  and served(core) == files[core] and config.get("three") is True, hits)
        start("fallback", second, [altered, good], fallback_run)

        bare = os.path.join(root, "bare")

        def no_internet(rig, config):
            check("no internet on the first run: the page is told there's no three.js", config.get("three") is False
                  and served("build/three.module.js") is None, config.get("three"))
            api("/api/enable", rig.token)
            wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
            r = jog(rig, "joint", 5, 1)
            check("but the panel still jogs the arm", r["ok"] and r["moved"], r)
        out = start("offline", bare, [offline], no_internet)
        check("and the console says what to do", "couldn't download three.js" in out and "reach the internet" in out, out)
        check("and nothing was saved", not os.path.exists(bare), os.listdir(bare) if os.path.exists(bare) else None)

        tampered = os.path.join(root, "tampered")

        def only_altered(rig, config):
            check("a mirror with only an altered copy: no 3D view", config.get("three") is False
                  and served(core) is None, config.get("three"))
        out = start("tampered", tampered, [altered], only_altered)
        check("the altered file isn't saved, and the console says so",
              on_disk(tampered, core) is None and "isn't the published file" in out, out)
    finally:
        mirror.shutdown()
        shutil.rmtree(root, ignore_errors=True)


def scenario_move():
    print("== planned moves: one joint or several to absolute angles, checked for collisions")
    rig = Rig("move")
    try:
        t = rig.token
        wait_for(lambda s: s.get("live"))
        r = move(rig, 5, 10.0)
        check("plan refused while motors are off", not r["ok"] and "motors are off" in r["message"], r)
        api("/api/enable", t)
        s = wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        j = s["joints"]
        check("simulator starts where the collision cases expect", [round(v) for v in j] == [0, -30, 50, -10, 90, 0], j)

        # From (0, -30, 50, -10, 90, 0): cases worked out with utra_model and checked against dense sampling.
        sent = rig.log().count("MOVEJ_P2P:")
        for targets, why in [([None, -150, None, None, None, None], "the forearm would come within 10 mm of a cart base bar"),
                             ([None, -120, 160, None, None, None], "the forearm would come within 10 mm of the cart's column"),
                             ([None, None, 165, -150, None, None], "wrist 2 would come within 10 mm of the upper arm")]:
            r = api("/api/move", t, {"targets": targets, "speed": 20})
            check(f"plan {targets} refused: {why}", not r["ok"] and r["message"] == f"Not moving: {why}.", r)
        check("nothing sent for the refused plans", rig.log().count("MOVEJ_P2P:") == sent, rig.log()[-300:])
        r = move(rig, 1, -100.0)  # hangs the arm down beside its cart, well clear of it
        check("shoulder swung below the base plane but clear of the cart: accepted", r["ok"] and r["moved"], r)
        s = wait_for(idle, timeout=12)
        check("shoulder reached -100", abs(s["joints"][1] + 100) < 0.05, s["joints"])
        r = move(rig, 1, -30.0)
        s = wait_for(idle, timeout=12)
        check("and came back", r["ok"] and abs(s["joints"][1] + 30) < 0.05, s["joints"])

        j = s["joints"]
        target = round(j[5] + 30, 1)
        r = move(rig, 5, target, speed=10.0)
        check("one-joint move accepted", r["ok"] and r["moved"], r)
        s = wait_for(lambda s: s.get("goal"))
        goal = (s.get("goal") or {}).get("values") or []
        check("status reports the goal while moving", len(goal) == 6 and abs(goal[5] - target) < 0.01, s.get("goal"))
        check("the other joints' goals stay put", len(goal) == 6 and all(abs(a - b) < 0.05 for a, b in zip(goal[:5], j[:5])), goal)
        r2 = move(rig, 5, target - 10)
        check("second move refused while moving", not r2["ok"] and "still moving" in r2["message"], r2)
        r3 = jog(rig, "joint", 5, 1)
        check("jog held back during a planned move", not r3.get("moved"), r3)
        s = wait_for(idle, timeout=8)
        check("move reached its target", abs(s["joints"][5] - target) < 0.05 and not s.get("goal"), s["joints"])
        r = move(rig, 5, target)
        check("move to the current angle does nothing", r["ok"] and not r["moved"], r)

        j = s["joints"]
        r = api("/api/move", t, {"targets": [None, -20.0, 70.0, None, None, None], "speed": 20})
        check("two-joint plan accepted", r["ok"] and r["moved"] and r["message"].startswith("Moving J2, J3 together"), r)
        s = wait_for(lambda s: s.get("goal"))
        goal = (s.get("goal") or {}).get("values") or []
        check("status reports both planned joints in the goal",
              len(goal) == 6 and abs(goal[1] + 20) < 0.01 and abs(goal[2] - 70) < 0.01, goal)
        check("joints outside the plan keep their angles", len(goal) == 6 and all(abs(goal[i] - j[i]) < 0.05 for i in (0, 3, 4, 5)), goal)
        s = wait_for(idle, timeout=12)
        check("the arm reached the plan", abs(s["joints"][1] + 20) < 0.05 and abs(s["joints"][2] - 70) < 0.05, s["joints"])
        r = api("/api/move", t, {"targets": [None, -20.0, 70.0, None, None, None], "speed": 20})
        check("a plan the arm is already at does nothing", r["ok"] and not r["moved"] and "already" in r["message"], r)
        r = api("/api/move", t, {"targets": [None, -20.0, 70.0, None, None, j[5] + 5], "speed": 10})
        check("only the joints that differ move", r["ok"] and r["moved"] and r["message"].startswith("J6 \u2192"), r)
        wait_for(idle, timeout=8)

        for body, why in [({"joint": 5, "angle": 181, "speed": 10}, "can only be planned"),
                          ({"joint": 2, "angle": 170, "speed": 10}, "can only be planned"),
                          ({"joint": 6, "angle": 0, "speed": 10}, "Unsupported joint"),
                          ({"joint": 5, "angle": "10", "speed": 10}, "Unsupported angle"),
                          ({"joint": 5, "angle": True, "speed": 10}, "Unsupported angle"),
                          ({"joint": 5, "angle": 10, "speed": 500}, "Unsupported joint speed"),
                          ({"targets": [None] * 5, "speed": 10}, "Unsupported targets"),
                          ({"targets": [None] * 7, "speed": 10}, "Unsupported targets"),
                          ({"targets": "0,0,0,0,0,0", "speed": 10}, "Unsupported targets"),
                          ({"targets": {"1": 10}, "speed": 10}, "Unsupported targets"),
                          ({"targets": [None] * 6, "speed": 10}, "The plan is empty"),
                          ({"targets": [None, "10", None, None, None, None], "speed": 10}, "Unsupported angle for J2"),
                          ({"targets": [None, None, False, None, None, None], "speed": 10}, "Unsupported angle for J3"),
                          ({"targets": [None, None, [5], None, None, None], "speed": 10}, "Unsupported angle for J3"),
                          ({"targets": [None, None, 170, None, None, None], "speed": 10}, "J3 can only be planned"),
                          ({"targets": [0, 0, 0, 0, 0, -181], "speed": 10}, "J6 can only be planned"),
                          ({"targets": [None, -10, None, None, None, None], "speed": 7}, "Unsupported joint speed"),
                          ({"targets": [None, -10, None, None, None, None]}, "Unsupported joint speed")]:
            r = api("/api/move", t, body)
            check(f"move rejects {body}", not r["ok"] and why in r["message"], r)
        for raw in (b'{"joint": 5, "angle": NaN, "speed": 10}', b'{"targets": [null, Infinity, null, null, null, null], "speed": 10}',
                    b'{"targets": [null, null, -Infinity, null, null, null], "speed": 10}'):
            code, data = request("POST", "/api/move", raw=raw, token=t)
            r = json.loads(data)
            check(f"move rejects {raw.decode()}", not r["ok"] and "Unsupported angle" in r["message"], r)

        move(rig, 0, 10.0, speed=5.0)
        time.sleep(0.4)
        r = api("/api/stop", t)
        s = wait_for(lambda s: s["state"] == 4)
        check("STOP halts a planned move", r["ok"] and s["joints"][0] < 9.5 and not s.get("goal"), s["joints"])
        r = move(rig, 0, 0.0)
        check("plans refused after STOP until Enable", not r["ok"] and "stopped" in r["message"], r)
    finally:
        rig.close()


def scenario_tight():
    print("== starting within the margin of the cart: backing out is fine, closing in isn't")
    rig = Rig("tight", ["--joints", "0,-124,50,-10,90,0"])  # the forearm 5 mm from a cart base bar
    try:
        t = rig.token
        wait_for(lambda s: s.get("live"))
        api("/api/enable", t)
        wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        sent = rig.log().count("MOVEJ_P2P:")
        r = jog(rig, "joint", 1, -1, step=1.0)
        check("a joint step closer refused", not r["ok"] and "the forearm would come within 10 mm of a cart base bar" in r["message"], r)
        check("and not sent", rig.log().count("MOVEJ_P2P:") == sent, rig.log()[-300:])
        r = move(rig, 1, -128.0)
        check("moving closer refused", not r["ok"] and "the forearm would come within 10 mm of a cart base bar" in r["message"], r)
        r = jog(rig, "joint", 1, 1, step=1.0)
        check("a joint step away accepted", r["ok"] and r["moved"], r)
        s = wait_for(idle, timeout=8)
        check("and taken", abs(s["joints"][1] + 123) < 0.05, s["joints"])
        r = jog(rig, "joint", 5, 1, step=5.0)
        check("a step of a joint that doesn't change the gap accepted", r["ok"] and r["moved"], r)
        wait_for(idle, timeout=8)
        r = move(rig, 1, -60.0)
        check("backing out accepted", r["ok"] and r["moved"], r)
        s = wait_for(idle, timeout=12)
        check("and the arm got there", abs(s["joints"][1] + 60) < 0.05, s["joints"])
    finally:
        rig.close()


def last_move(rig, label="MOVEJ_P2P"):
    """(speed, acc) of the last move of this kind in the simulator's log."""
    found = re.findall(label + r": goal=\[[^\]]*\] speed=([\d.]+) acc=([\d.]+)", rig.log())
    return tuple(float(v) for v in found[-1]) if found else (None, None)


def scenario_speeds():
    print("== step and speed detents: joint top speeds and the controller's acceleration limits")
    rig = Rig("speeds", ("--joint-maxacc", "300", "--tcp-maxacc", "3000"))  # the controller's maximum accelerations: 300 deg/s^2, 3000 mm/s^2
    try:
        t = rig.token
        wait_for(lambda s: s.get("live"))
        api("/api/enable", t)
        wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        r = jog(rig, "joint", 0, 1, step=15.0, speed=120.0)
        check("a J1 step at 120 deg/s is capped at J1's top speed, 84", r["ok"] and r["moved"] and "J1's top speed" in r["message"]
              and last_move(rig) == (84.0, 280.0), (r, last_move(rig)))
        wait_for(idle, timeout=8)
        r = jog(rig, "joint", 5, 1, step=10.0, speed=120.0)
        check("a J6 step at 120 is capped at 119.1, ramping no harder than the controller allows",
              r["ok"] and "J6's top speed" in r["message"] and last_move(rig) == (119.1, 300.0), (r, last_move(rig)))
        wait_for(idle, timeout=8)
        r = jog(rig, "joint", 5, -1, step=0.1, speed=10.0)
        check("a slow step keeps the gentle minimum ramp", r["ok"] and r["moved"] and last_move(rig) == (10.0, 100.0), (r, last_move(rig)))
        wait_for(idle, timeout=8)
        j = status()["joints"]
        r = api("/api/move", t, {"targets": [None, j[1] - 40, None, None, None, j[5] + 10], "speed": 120})
        check("a plan led by J2 is capped at J2's top speed", r["ok"] and r["moved"] and r["message"].endswith("84\u00b0/s, J2's top speed")
              and last_move(rig)[0] == 84.0, (r, last_move(rig)))
        wait_for(idle, timeout=8)
        j = status()["joints"]
        r = api("/api/move", t, {"targets": [None, j[1] + 10, None, None, None, j[5] - 40], "speed": 90})
        check("a plan led by J6 at 90 runs uncapped", r["ok"] and "top speed" not in r["message"] and last_move(rig)[0] == 90.0,
              (r, last_move(rig)))
        wait_for(idle, timeout=8)
        r = jog(rig, "tool", 2, 1, step=50.0, speed=1000.0)
        check("a 50 mm tool step at 1000 mm/s, ramping at the controller's 3000 mm/s^2",
              r["ok"] and r["moved"] and last_move(rig, "MOVET_LINE") == (1000.0, 3000.0), (r, last_move(rig, "MOVET_LINE")))
        wait_for(idle, timeout=8)
        for body, why in [({"kind": "joint", "index": 0, "dir": 1, "step": 30.0, "speed": 10.0}, "Unsupported joint step"),
                          ({"kind": "joint", "index": 0, "dir": 1, "step": 1.0, "speed": 170.0}, "Unsupported joint speed"),
                          ({"kind": "tool", "index": 0, "dir": 1, "step": 100.0, "rotStep": 2.0, "speed": 20.0}, "Unsupported tool step"),
                          ({"kind": "tool", "index": 3, "dir": 1, "step": 5.0, "rotStep": 30.0, "speed": 20.0}, "Unsupported rotation step"),
                          ({"kind": "tool", "index": 0, "dir": 1, "step": 5.0, "rotStep": 2.0, "speed": 2000.0}, "Unsupported tool speed")]:
            r = api("/api/jog", t, body)
            check(f"jog rejects {body}", not r["ok"] and why in r["message"], r)
    finally:
        rig.close()


def scenario_stalls():
    print("== status stream stalls for 3 s now and then, as over Wi-Fi")
    rig = Rig("stall", ["--stall-every", "6", "--stall-for", "3.1"])
    try:
        t = rig.token
        wait_for(lambda s: s.get("live"))
        api("/api/enable", t)
        wait_for(lambda s: s["enabled"] and s["state"] in (0, 2) and s["age"] < 0.5, timeout=10)
        oldest, dropped = 0.0, False
        deadline = time.monotonic() + 7
        while time.monotonic() < deadline:
            s = status()
            oldest = max(oldest, s["age"] or 0)
            dropped = dropped or not s["live"]
            time.sleep(0.1)
        check("a 3 s stall doesn't drop the live flag", not dropped and oldest > 2.5, oldest)
        wait_for(lambda s: s["age"] > 1.2, timeout=8)
        t0 = time.monotonic()
        r = jog(rig, "joint", 5, 1)
        waited = time.monotonic() - t0
        check("a click during a stall waits it out, then moves", r["ok"] and r["moved"] and 0.3 < waited < 4.5, (r, waited))
        s = wait_for(idle, timeout=10)
        j5 = s["joints"][5]
        wait_for(lambda s: s["age"] > 1.2, timeout=10)
        t0 = time.monotonic()
        r = move(rig, 5, round(j5 - 5, 1), speed=20.0)
        waited = time.monotonic() - t0
        check("a drag released during a stall is held for fresh status, then sent", r["ok"] and r["moved"] and 0.3 < waited < 4.5,
              (r, waited))
        s = wait_for(idle, timeout=10)
        j = s["joints"][5]
        stops = rig.log().count("MOTION_STATUS data=04")
        r = move(rig, 5, round(j + 40, 1), speed=10.0)  # about 4 s, so it spans a stall
        s = wait_for(lambda s: idle(s) and s["age"] < 0.5, timeout=14)
        check("a drag move spanning a stall finishes", r["moved"] and abs(s["joints"][5] - round(j + 40, 1)) < 0.05,
              (r, s["joints"]))
        check("the watchdog let it run through the stall", rig.log().count("MOTION_STATUS data=04") == stops,
              rig.log()[-400:])
    finally:
        rig.close()


def scenario_main():
    print("== main flow")
    rig = Rig("main")
    try:
        t = rig.token
        check("page served with a token", t)
        code, _ = request("GET", "/", host=f"evil.example:{PORT}")
        check("page refused for a foreign Host header", code == 403, code)
        code, _ = request("POST", "/api/jog", {"kind": "joint", "index": 5, "dir": 1, "step": 1, "speed": 10})
        check("jog refused without the token", code == 403, code)
        code, _ = request("POST", "/api/jog", {"kind": "joint"}, token="wrong")
        check("jog refused with a wrong token", code == 403, code)
        code, _ = request("POST", "/api/stop", {}, token=t, host=f"evil.example:{PORT}")
        check("POST refused for a foreign Host header", code == 403, code)
        code, _ = request("POST", "/api/jog", raw=b"{not json", token=t)
        check("malformed JSON rejected", code == 400, code)

        s = wait_for(lambda s: s.get("live"))
        check("status is live", s and s["live"], s)
        check("motors start off", s and not s["enabled"] and s["state"] == 4, s)
        r = jog(rig, "joint", 5, 1)
        check("jog refused while motors are off", not r["ok"] and "motors are off" in r["message"], r)

        r = api("/api/enable", t)
        check("enable succeeds", r["ok"], r)
        s = wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        start = s["joints"][5]

        r = jog(rig, "joint", 5, 1)
        check("single click accepted", r["ok"] and r["moved"], r)
        s = wait_for(idle)
        check("single click moved exactly one step", abs(s["joints"][5] - (start + 1)) < 0.05, s["joints"])

        before = s["joints"][5]
        replies = [jog(rig, "joint", 5, 1) for _ in range(6)]
        moved = sum(1 for r in replies if r.get("moved"))
        held = [r for r in replies if r["ok"] and not r.get("moved")]
        s = wait_for(idle)
        check("rapid clicks: every accepted click moved one step", abs(s["joints"][5] - (before + moved)) < 0.05,
              (before, moved, s["joints"][5]))
        check("rapid clicks: lead cap held extra clicks back", 3 <= moved <= 5 and held, (moved, len(held)))

        for body, why in [({"kind": "joint", "index": 5, "dir": 1, "step": 3, "speed": 10}, "joint step"),
                          ({"kind": "joint", "index": 6, "dir": 1, "step": 1, "speed": 10}, "joint"),
                          ({"kind": "joint", "index": 5, "dir": 0, "step": 1, "speed": 10}, "direction"),
                          ({"kind": "joint", "index": 5, "dir": 1, "step": 1, "speed": 100}, "joint speed"),
                          ({"kind": "joint", "index": True, "dir": 1, "step": 1, "speed": 10}, "joint"),
                          ({"kind": "tool", "index": 0, "dir": 1, "step": 5, "rotStep": 9, "speed": 20}, "rotation step"),
                          ({"kind": "spin", "index": 0, "dir": 1}, "jog kind")]:
            r = api("/api/jog", t, body)
            check(f"rejects bad {why}", not r["ok"] and "Unsupported" in r["message"], r)

        s = wait_for(idle)
        z0 = s["pose"][2]
        r = jog(rig, "tool", 2, 1, step=5.0)
        s = wait_for(idle)
        check("tool Z+ moved 5 mm", r["moved"] and abs(s["pose"][2] - (z0 + 5)) < 0.1, (z0, s["pose"]))
        roll0 = s["pose"][3]
        r = jog(rig, "tool", 3, 1, rot=2.0)
        s = wait_for(idle)
        roll1 = s["pose"][3]
        delta = (roll1 - roll0 + 540) % 360 - 180
        check("tool roll +2 deg wraps across +-180", r["moved"] and abs(delta - 2) < 0.1, (roll0, roll1))

        r1 = jog(rig, "joint", 0, 1, step=5.0, speed=5.0)
        r2 = jog(rig, "tool", 0, 1)
        check("switching joint->tool mid-move refused", r1["moved"] and not r2["ok"] and "finish" in r2["message"], r2)
        wait_for(idle)

        s = status()
        j0 = s["joints"][0]
        jog(rig, "joint", 0, 1, step=5.0, speed=5.0)
        time.sleep(0.3)
        t0 = time.monotonic()
        r = api("/api/stop", t)
        took = time.monotonic() - t0
        s = wait_for(lambda s: s["state"] == 4)
        check("STOP confirmed mid-move", r["ok"] and s["state"] == 4, r)
        check("STOP goes over the panel's own connection, at once", took < 0.5 and "closed a second connection" not in rig.log(),
              (round(took, 2), rig.log()[-300:]))
        check("STOP halted the move early", j0 < s["joints"][0] < j0 + 4.5, (j0, s["joints"][0]))
        r = jog(rig, "joint", 0, 1)
        check("jog refused after STOP until Enable", not r["ok"] and "stopped" in r["message"], r)
        r = api("/api/enable", t)
        wait_for(lambda s: s["state"] in (0, 2))
        r2 = jog(rig, "joint", 0, -1)
        check("Enable re-arms after STOP", r["ok"] and r2["moved"], (r, r2))
        wait_for(idle)
    finally:
        rig.close()


def scenario_watchdog():
    print("== watchdog: page goes quiet mid-move")
    rig = Rig("watchdog")
    try:
        wait_for(lambda s: s.get("live"))
        api("/api/enable", rig.token)
        s = wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        j = s["joints"][5]
        jog(rig, "joint", 5, 1, step=5.0, speed=5.0)
        jog(rig, "joint", 5, 1, step=5.0, speed=5.0)
        time.sleep(3.0)  # no requests at all
        log = rig.log()
        s = status()
        check("watchdog sent stop", "WRITE MOTION_STATUS data=04" in log, log[-300:])
        check("watchdog explains itself", s["note"].startswith("Stopped because the jog page went quiet."), s["note"])
        check("arm stopped well short of the queued goal", s["joints"][5] < j + 9, (j, s["joints"][5]))
    finally:
        rig.close()


def scenario_error_mid_move():
    print("== controller error mid-move")
    rig = Rig("error", ["--error-after", "0.3"])
    try:
        wait_for(lambda s: s.get("live"))
        api("/api/enable", rig.token)
        wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        jog(rig, "joint", 5, 1, step=5.0, speed=5.0)
        s = wait_for(lambda s: s["err"] == 9)
        time.sleep(0.5)
        r = jog(rig, "joint", 5, 1)
        check("status shows the fault", s["err"] == 9, s)
        check("watchdog cleared the queue", "WRITE MOTION_STATUS data=04" in rig.log())
        check("jog refused while faulted", not r["ok"] and "error 9" in r["message"], r)
    finally:
        rig.close()


def scenario_ik_fail():
    print("== unreachable tool pose")
    rig = Rig("ik", ["--ik-fail"])
    try:
        wait_for(lambda s: s.get("live"))
        api("/api/enable", rig.token)
        wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        jog(rig, "tool", 0, 1)
        s = wait_for(lambda s: s["err"] == 21)
        r = jog(rig, "tool", 0, 1)
        check("status names the kinematics error", s["err"] == 21 and s["errName"] == "Inverse kinematics error", s)
        check("further tool jogs refused", not r["ok"] and "Inverse kinematics error" in r["message"], r)
    finally:
        rig.close()


def scenario_no_report():
    print("== no status report")
    rig = Rig("noreport", ["--no-report"])
    try:
        time.sleep(1.5)
        s = status()
        r1 = api("/api/enable", rig.token)
        r2 = jog(rig, "joint", 5, 1)
        check("status not live", s and not s["live"], s)
        check("enable refused without live status", not r1["ok"] and "No live status" in r1["message"], r1)
        check("jog refused without live status", not r2["ok"] and "No live status" in r2["message"], r2)
        check("nothing written", "WRITE" not in rig.log())
    finally:
        rig.close()


def scenario_estop():
    print("== E-stop pressed, then released")
    rig = Rig("estop", ["--error", "1", "--sticky-error", "--servo-comm", "-3"])
    try:
        s = wait_for(lambda s: s.get("live"))
        r = api("/api/enable", rig.token)
        check("status reports ESTOP_SWITCH", s["err"] == 1 and s["errName"] == "ESTOP_SWITCH", s)
        check("enable explains the E-stop", not r["ok"] and "emergency stop is active" in r["message"], r)
        check("only a reset was written", all("RESET_ERR" in l for l in rig.log().splitlines() if "WRITE" in l))
    finally:
        rig.close()
    rig = Rig("estop2", ["--error", "1"])
    try:
        wait_for(lambda s: s.get("live"))
        r = api("/api/enable", rig.token)
        s = wait_for(lambda s: s["enabled"])
        check("enable resets a released E-stop and arms", r["ok"] and s["enabled"] and s["err"] == 0, (r, s))
    finally:
        rig.close()


def scenario_stop_unconfirmed():
    print("== stop not confirmed, then the hardware E-stop")
    rig = Rig("stopfail", ["--ignore-stop", "--estop-after", "9"])
    try:
        wait_for(lambda s: s.get("live"))
        api("/api/enable", rig.token)
        wait_for(lambda s: s["enabled"] and s["state"] in (0, 2))
        jog(rig, "joint", 5, 1, step=5.0, speed=5.0)
        time.sleep(0.2)
        r = api("/api/stop", rig.token)
        s = status()
        check("unconfirmed stop reported as a failure",
              not r["ok"] and r["message"].startswith("STOP NOT CONFIRMED") and "E-stop" in r["message"], r)
        check("status raises the stop alarm", s and s["stopAlarm"], s)
        r = jog(rig, "joint", 5, 1)
        check("jog refused while the stop is unconfirmed", not r["ok"] and "wasn't confirmed" in r["message"], r)
        r = api("/api/enable", rig.token)
        check("enable refused while the stop is unconfirmed", not r["ok"] and "wasn't confirmed" in r["message"], r)
        s = wait_for(lambda s: s["err"] == 1, timeout=12)
        check("hardware E-stop clears the alarm", s["err"] == 1 and not s["stopAlarm"], s)
    finally:
        rig.close()


def scenario_unreachable():
    print("== controller unreachable")
    t0 = time.monotonic()
    proc = subprocess.run([PY, SERVER, "192.0.2.1", "--port", str(PORT)], cwd=ROOT, capture_output=True, text=True, timeout=30)
    check("server exits with a clear error", proc.returncode == 1 and "Cannot reach" in proc.stderr, proc.stderr)
    check("and quickly", time.monotonic() - t0 < 8, time.monotonic() - t0)


if __name__ == "__main__":
    print(f"server python: {PY}")
    for fn in (scenario_main, scenario_assets, scenario_three, scenario_move, scenario_tight, scenario_speeds, scenario_stalls,
               scenario_watchdog,
               scenario_error_mid_move, scenario_ik_fail, scenario_no_report, scenario_estop,
               scenario_stop_unconfirmed, scenario_unreachable):
        try:
            fn()
        except Exception as e:  # report and continue
            check(f"{fn.__name__} crashed", False, repr(e))
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    sys.exit(1 if failed else 0)
