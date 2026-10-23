"""The twin's HTTP endpoint, so other processes (the brain, perception, the viewer, tests) can take camera frames, read
its state and place obstacles while it runs. It listens on the twin's loopback address only (port 30100 by default)
and, like the jog panel, refuses requests from web pages (any with an Origin header, or for another Host).

    GET  /state                       time, joints, tool pose, the controller's state, contacts, zones, obstacles (where
                                      each is, and its shape and size), cameras (intrinsics and true poses)
    GET  /cameras/<name>.npz          a frame: rgb, depth (m), segmentation, K, T_base_camera, meta (JSON)
    GET  /cameras/<name>/rgb.png      the colour image
    GET  /cameras/<name>/depth.png    depth, uint16 millimetres
    GET  /pose                        the arm's joints (rad) and where every obstacle is ([x, y, z (m), qw, qx, qy, qz],
                                      base frame), with the model's revision and fingerprint, and the loop's step (seq)
    GET  /pose/stream[?hz=N]          the same, one JSON line per step of the loop (100 a second), or at most N a second
    GET  /model                       the generated model: a zip of twin.xml, its meshes (assets/) and layout.json
                                      (revision, fingerprint, the scene it was built from)
    GET  /loop[?seconds=S][&raw=1]    the loop's cadence over the last S seconds (default 10): ticks, the gaps between
                                      them and the work in each, in ms (raw: every gap); and the camera process's pid
    POST /obstacles                   {"add": {obstacle}} | {"move": {"name", "pose", "frame"}} | {"remove": "name"}
    POST /cameras                     {"add": {camera}} | {"update": {"name", ...}} | {"remove": "name"}

The twin's loop publishes a snapshot of its state every step (twin.Snapshot: copies, and the build they belong to).
Camera frames are rendered from the first snapshot after the request arrived, in a process of their own
(cameras.CameraProcess); /pose, /pose/stream and /model are answered from the newest in the request's thread. None of
them waits for the loop or makes it wait. What changes the twin, and /state, still run on the loop, which calls
process() between steps; the request threads only queue that work and wait.
"""
from __future__ import annotations

import collections
import concurrent.futures
import io
import json
import math
import queue
import threading
import time
import urllib.parse
import zipfile
from http.server import BaseHTTPRequestHandler

import numpy as np

from . import repo
from .cameras import FRAME_KINDS, CameraProcess
from .model import CameraSpec, Obstacle, SceneError

PORT = 30100
WAIT_S = 10.0
LATE = 1.5  # a gap between ticks longer than this many periods counts as late
loopback_http = repo.spine("loopback_http")


class Cadence:
    """When each of the loop's ticks started and how long its work took, for the last minute or so."""

    def __init__(self, period, keep=6000, clock=time.perf_counter):
        self.period, self.clock = period, clock
        self.lock = threading.Lock()
        self.ticks = collections.deque(maxlen=keep)  # (start, work), s
        self.started = None

    def start(self):
        self.started = self.clock()

    def end(self):
        now = self.clock()
        with self.lock:
            self.ticks.append((self.started, now - self.started))

    def stats(self, seconds=10.0, raw=False):
        """The ticks of the last `seconds`: how many, how far apart (ms), how many late, and the work in each (ms);
        with raw, every gap too."""
        now = self.clock()
        with self.lock:
            ticks = [t for t in self.ticks if t[0] >= now - seconds]
        gaps = [(b[0] - a[0]) * 1000.0 for a, b in zip(ticks, ticks[1:])]
        work = sorted(w * 1000.0 for _, w in ticks)
        span = ticks[-1][0] - ticks[0][0] if len(ticks) > 1 else 0.0
        out = {"periodMs": self.period * 1000.0, "seconds": round(span, 3), "ticks": len(ticks),
               "perSecond": round((len(ticks) - 1) / span, 2) if span > 0 else None,
               "gapMs": percentiles(sorted(gaps)), "late": sum(g > LATE * self.period * 1000.0 for g in gaps),
               "lateMs": LATE * self.period * 1000.0, "workMs": percentiles(work)}
        if raw:
            out["gapsMs"] = [round(g, 3) for g in gaps]
        return out


def percentiles(values):
    """{p50, p95, p99, max} of sorted values (nearest rank), rounded to µs; None for none."""
    if not values:
        return None

    def at(q):
        return round(values[min(len(values) - 1, int(math.ceil(q * len(values))) - 1)], 3)

    return {"p50": at(0.5), "p95": at(0.95), "p99": at(0.99), "max": round(values[-1], 3)}


class TwinServer:
    def __init__(self, twin, host, port=PORT, period=0.01):
        self.twin, self.host, self.port = twin, host, port
        self.jobs = queue.Queue()
        self.t, self.controller = None, {}
        self.cadence = Cadence(period)
        self.closed = False
        self.published = threading.Condition()
        self.seq, self.snap = 0, None
        self.zipped = (None, None)  # (layout, GET /model's bytes for it)
        if twin is not None:
            self.publish(twin.snapshot())
        self.cameras = CameraProcess(WAIT_S)
        handler = type("Handler", (_Handler,), {"server_ref": self})
        self.httpd = loopback_http.Server((host, port), handler)
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, name="twin-http", daemon=True)
        self.thread.start()

    def close(self):
        self.closed = True
        with self.published:
            self.published.notify_all()
        self.httpd.shutdown()
        self.httpd.server_close()
        self.cameras.close()

    # ---- the loop's side ------------------------------------------------------------------------------------------
    def publish(self, snap):
        """The twin's newest state (a twin.Snapshot), for camera frames, /pose and its stream. Called by the loop."""
        with self.published:
            self.seq += 1
            snap.seq = self.seq
            self.snap = snap
            self.published.notify_all()

    def process(self, t, controller):
        """Called by the twin's loop: note the time and the controller's state, and run what the request threads
        queued. After each job, what it changed is published before its request is answered."""
        self.t, self.controller = t, controller
        while True:
            try:
                fn, future = self.jobs.get_nowait()
            except queue.Empty:
                return
            if future.set_running_or_notify_cancel():
                try:
                    result = fn(self.twin)
                except Exception as e:  # handed back to the request
                    self.publish(self.twin.snapshot())
                    future.set_exception(e)
                else:
                    self.publish(self.twin.snapshot())
                    future.set_result(result)

    # ---- the requests' side ---------------------------------------------------------------------------------------
    def call(self, fn):
        """Run fn(twin) on the twin's loop and return its result (or raise its exception)."""
        future = concurrent.futures.Future()
        self.jobs.put((fn, future))
        return future.result(timeout=WAIT_S)

    def latest(self):
        """The newest snapshot the loop published."""
        with self.published:
            return self.snap

    def fresh(self):
        """The first snapshot published from now on (the newest, if none comes within a second)."""
        snap = self.latest()
        return self.newer(snap.seq, 1.0) or self.latest()

    def newer(self, seq, timeout):
        """The newest snapshot once there's one after `seq`, or None after `timeout` s (or once closed)."""
        deadline = time.monotonic() + timeout
        with self.published:
            while not self.closed and (self.snap is None or self.snap.seq <= seq):
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self.published.wait(left)
            return None if self.closed else self.snap

    def model_zip(self):
        """GET /model's body for the newest build: twin.xml, assets/<name> and layout.json, stored (not compressed:
        the meshes are most of it and it only crosses this PC)."""
        layout = self.latest().layout
        with self.published:
            if self.zipped[0] is layout:
                return layout, self.zipped[1]
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as z:
            z.writestr("layout.json", json.dumps({"revision": layout.revision, "fingerprint": layout.fingerprint,
                                                  "scene": layout.scene}))
            z.writestr("twin.xml", layout.built.xml)
            for name in sorted(layout.built.assets):
                z.writestr("assets/" + name, layout.built.assets[name])
        data = buf.getvalue()
        with self.published:
            self.zipped = (layout, data)
        return layout, data


class _Handler(BaseHTTPRequestHandler):
    server_ref = None
    protocol_version = "HTTP/1.1"
    disable_nagle_algorithm = True  # /pose/stream's lines go out as they're written, not two or three at a time

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, kind="application/json", headers=()):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def _allowed(self):
        s = self.server_ref
        if self.headers.get("Origin") is not None:
            self._send(403, {"ok": False, "why": "requests from web pages are refused"})
            return False
        if self.headers.get("Host") not in (f"{s.host}:{s.port}", f"localhost:{s.port}"):
            self._send(403, {"ok": False, "why": "unexpected Host"})
            return False
        return True

    def do_GET(self):
        if not self._allowed():
            return
        s = self.server_ref
        path, _, query = self.path.partition("?")
        query = urllib.parse.parse_qs(query)
        try:
            if path == "/state":
                return self._send(200, dict(s.call(lambda twin: state_of(twin, s.t, s.controller)), ok=True))
            if path == "/pose":
                return self._send(200, dict(s.latest().sample(), ok=True))
            if path == "/pose/stream":
                return self._stream(float(query.get("hz", ["0"])[0]))
            if path == "/model":
                layout, data = s.model_zip()
                return self._send(200, data, "application/zip", [("X-Twin-Revision", str(layout.revision)),
                                                                 ("X-Twin-Fingerprint", layout.fingerprint)])
            if path == "/loop":
                stats = s.cadence.stats(float(query.get("seconds", ["10"])[0]), query.get("raw", ["0"])[0] == "1")
                return self._send(200, dict(stats, cameraProcess=s.cameras.pid(), ok=True))
            if path.startswith("/cameras/"):
                rest = path[len("/cameras/"):]
                if rest.endswith(".npz"):
                    name, what = rest[:-4], "npz"
                elif rest.endswith("/rgb.png") or rest.endswith("/depth.png"):
                    name, what = rest.rsplit("/", 1)
                else:
                    return self._send(404, {"ok": False, "why": "unknown path"})
                return self._send(200, s.cameras.frame(s.fresh(), name, what), FRAME_KINDS[what])
            return self._send(404, {"ok": False, "why": "unknown path"})
        except SceneError as e:
            return self._send(404, {"ok": False, "why": str(e)})
        except ValueError as e:
            return self._send(400, {"ok": False, "why": str(e)})
        except Exception as e:  # report, don't crash the server
            return self._send(500, {"ok": False, "why": repr(e)})

    def _stream(self, hz):
        """GET /pose/stream: a JSON line per new snapshot (at most `hz` a second, if given), until the client goes."""
        if not (hz == 0 or 0.1 <= hz <= 1000):
            raise ValueError("hz must be 0.1 to 1000 (or 0 for every step)")
        s = self.server_ref
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        seq, gap = 0, 1.0 / hz if hz else 0.0
        try:
            while not s.closed:
                snap = s.newer(seq, 1.0)
                if snap is None:
                    continue
                seq = snap.seq
                self.wfile.write((json.dumps(snap.sample()) + "\n").encode())
                if gap:
                    time.sleep(gap)
        except OSError:  # the client went
            pass

    def do_POST(self):
        if not self._allowed():
            return
        s = self.server_ref
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            return self._send(415, {"ok": False, "why": "send JSON (Content-Type: application/json)"})
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))) or b"{}")
            if self.path == "/obstacles":
                if "add" in body:
                    s.call(lambda twin: twin.add_obstacle(Obstacle(**body["add"])))
                elif "move" in body:
                    m = body["move"]
                    s.call(lambda twin: twin.move_obstacle(m["name"], m["pose"], m.get("frame")))
                elif "remove" in body:
                    s.call(lambda twin: twin.remove_obstacle(body["remove"]))
                else:
                    return self._send(400, {"ok": False, "why": "expected add, move or remove"})
                return self._send(200, {"ok": True})
            if self.path == "/cameras":
                if "add" in body:
                    s.call(lambda twin: twin.add_camera(CameraSpec(**body["add"])))
                elif "update" in body:
                    u = dict(body["update"])
                    name = u.pop("name")
                    s.call(lambda twin: twin.update_camera(name, **u))
                elif "remove" in body:
                    s.call(lambda twin: twin.remove_camera(body["remove"]))
                else:
                    return self._send(400, {"ok": False, "why": "expected add, update or remove"})
                return self._send(200, {"ok": True})
            return self._send(404, {"ok": False, "why": "unknown path"})
        except (SceneError, TypeError, KeyError, ValueError) as e:
            return self._send(400, {"ok": False, "why": str(e)})
        except Exception as e:
            return self._send(500, {"ok": False, "why": repr(e)})


def state_of(twin, t, controller, clock=time.time):
    """What GET /state reports: poses as {"xyz": mm, "rpy": deg} in the base frame, T_base_camera in metres."""
    import mujoco
    kin = repo.spine("kinematics")

    def pose(T):
        return kin.matrix_to_pose([list(r) for r in np.asarray(T)], 4)

    cams = {}
    for name, c in twin.built.cameras.items():
        i = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        T = np.eye(4)
        T[:3, :3] = twin.data.cam_xmat[i].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T[:3, 3] = twin.data.cam_xpos[i]
        cams[name] = {"width": c.width, "height": c.height, "K": c.K.tolist(), "mount": c.mount,
                      "T_base_camera": T.round(9).tolist()}
    return {"time": t, "wallTime": clock(), "joints": [round(math.degrees(v), 5) for v in twin.data.qpos[twin.qadr]],
            "flange": pose(twin.flange()), "tcp": pose(twin.tcp()), "controller": controller,
            "contacts": twin.contacts(), "zones": {k: list(v) for k, v in twin.zone_clearances().items()},
            "obstacles": {name: pose(twin.obstacle_pose(name)) for name in twin.built.obstacles},
            "obstacleShapes": {o.name: {"shape": o.shape, "size": list(o.size), "fixed": o.fixed, "mesh": o.mesh}
                               for o in twin.scene.obstacles},
            "cameras": cams}
