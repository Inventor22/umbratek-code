"""The twin's HTTP endpoint, so other processes (the brain, perception, tests) can take camera frames, read its state
and place obstacles while it runs. It listens on the twin's loopback address only (port 30100 by default) and, like
the jog panel, refuses requests from web pages (any with an Origin header, or for another Host).

    GET  /state                       time, joints, tool pose, the controller's state, contacts, zones, obstacles (where
                                      each is, and its shape and size), cameras (intrinsics and true poses)
    GET  /cameras/<name>.npz          a frame: rgb, depth (m), segmentation, K, T_base_camera, meta (JSON)
    GET  /cameras/<name>/rgb.png      the colour image
    GET  /cameras/<name>/depth.png    depth, uint16 millimetres
    POST /obstacles                   {"add": {obstacle}} | {"move": {"name", "pose", "frame"}} | {"remove": "name"}
    POST /cameras                     {"add": {camera}} | {"update": {"name", ...}} | {"remove": "name"}

Work that touches the model (rendering, rebuilding) runs on the twin's own loop, which calls process() between
steps; the request threads only queue it and wait.
"""
from __future__ import annotations

import concurrent.futures
import json
import math
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler

import numpy as np

from . import repo
from .cameras import png_bytes
from .model import CameraSpec, Obstacle, SceneError

PORT = 30100
WAIT_S = 10.0
loopback_http = repo.spine("loopback_http")


class TwinServer:
    def __init__(self, twin, host, port=PORT):
        self.twin, self.host, self.port = twin, host, port
        self.jobs = queue.Queue()
        self.t, self.controller = None, {}
        handler = type("Handler", (_Handler,), {"server_ref": self})
        self.httpd = loopback_http.Server((host, port), handler)
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, name="twin-http", daemon=True)
        self.thread.start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()

    def call(self, fn):
        """Run fn(twin) on the twin's loop and return its result (or raise its exception)."""
        future = concurrent.futures.Future()
        self.jobs.put((fn, future))
        return future.result(timeout=WAIT_S)

    def process(self, t, controller):
        """Called by the twin's loop: note the time and the controller's state, and run what the request threads
        queued."""
        self.t, self.controller = t, controller
        while True:
            try:
                fn, future = self.jobs.get_nowait()
            except queue.Empty:
                return
            if future.set_running_or_notify_cancel():
                try:
                    future.set_result(fn(self.twin))
                except Exception as e:  # handed back to the request
                    future.set_exception(e)


class _Handler(BaseHTTPRequestHandler):
    server_ref = None
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, kind="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
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
        path = self.path.split("?")[0]
        try:
            if path == "/state":
                return self._send(200, dict(s.call(lambda twin: state_of(twin, s.t, s.controller)), ok=True))
            if path.startswith("/cameras/"):
                rest = path[len("/cameras/"):]
                if rest.endswith(".npz"):
                    name, what = rest[:-4], "npz"
                elif rest.endswith("/rgb.png") or rest.endswith("/depth.png"):
                    name, what = rest.rsplit("/", 1)
                else:
                    return self._send(404, {"ok": False, "why": "unknown path"})
                frame = s.call(lambda twin: twin.render(name, segmentation=(what == "npz")))
                if what == "npz":
                    return self._send(200, frame.npz(), "application/octet-stream")
                if what == "rgb.png":
                    return self._send(200, png_bytes(frame.rgb), "image/png")
                return self._send(200, png_bytes(np.round(frame.depth * 1000.0).clip(0, 65535).astype(np.uint16)),
                                  "image/png")
            return self._send(404, {"ok": False, "why": "unknown path"})
        except SceneError as e:
            return self._send(404, {"ok": False, "why": str(e)})
        except Exception as e:  # report, don't crash the server
            return self._send(500, {"ok": False, "why": repr(e)})

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
