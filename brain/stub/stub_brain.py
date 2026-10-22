#!/usr/bin/env python3
"""A stub brain: a stand-in for the real one (perception, a world model, cuRobo's planner and learned skills) that
speaks the spine's interface, version 1 (docs/brain-interface.md), so planned and learned steps run end to end now.

    python brain/stub/stub_brain.py            # serves http://127.0.0.1:8770, where the jog server looks by default
    python scripts/jog_server.py 127.0.0.1     # with tools/utra_sim.py running, or the arm's address

It plans planned moves' paths by a simple search over the spine's own model of the arm, its cart and the cell's
keep-out zones (scripts/utra_model.py and planner.py): straight there if that's clear; else up and over, through
poses with the tool point raised above the start and the goal; else through random via points; giving up when its
time is nearly out. It has one learned skill, "seek", a scripted stand-in for a policy: it spirals the tool point out
from where the step began, a few millimetres a burst, until an input goes high. Standard-library Python, like the
spine, whose modules it imports for the protocol and the arm's geometry; the real brain has its own environment
(brain/README.md).

--ignore-zones plays a buggy brain whose world model has lost the cell's keep-out zones: its paths go straight through
them, and the spine must refuse them.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
import os
import random
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(HERE)), "scripts")
sys.path.insert(0, SCRIPTS)
import brain_protocol as bp  # noqa: E402
import kinematics as kin  # noqa: E402
import planner as plannermod  # noqa: E402
import utra_model  # noqa: E402

NAME = "stub-brain 1"
DEFAULT_PORT = 8770
LIFTS_MM = (60.0, 120.0, 200.0, 300.0)  # how far above the start and goal "up and over" tries
RANDOM_TRIES = 200
SPREAD_DEG = 40.0  # how far a random via point strays from the straight way, per joint
TIME_SHARE = 0.6  # of the spine's deadline, spent searching before giving up
EPISODES_KEPT = 50
SKILLS = {"seek": "Spirals the tool point out from where the step began, a few mm a burst, keeping its orientation, "
                  "until an input goes high. Params: input (default tool.1), stepMm (8), pitchMm (10 per turn), "
                  "radiusMm (40), doneAfter (bursts; none)."}


class NoPath(Exception):
    pass


class StubBrain:
    """The stub's answers, apart from HTTP, so tests can use them in-process too."""

    def __init__(self, ignore_zones=False, seed=1):
        self.model = utra_model.ArmModel()
        self.ignore_zones = ignore_zones
        self.seed = seed
        self.lock = threading.Lock()
        self.episodes = {}  # a seek episode's id -> where it began and how far it has got
        self.ended = []  # (episode, outcome) as the spine reported them, most recent last

    def handle(self, request):
        """The reply to a request already checked with brain_protocol.check_request."""
        kind = request["type"]
        if kind == "hello":
            return bp.reply(request, brain=NAME + (" (ignoring keep-out zones)" if self.ignore_zones else ""),
                            planner=True, skills=SKILLS)
        if kind == "plan":
            try:
                path, note = self.plan(request)
            except NoPath as e:
                return bp.reply(request, ok=False, reason=str(e))
            return bp.reply(request, ok=True, path=path, note=note)
        if kind == "act":
            return bp.reply(request, action=self.act(request))
        with self.lock:
            self.episodes.pop(request["episode"], None)
            self.ended = (self.ended + [(request["episode"], request["outcome"])])[-EPISODES_KEPT:]
        return bp.reply(request)

    # ---- planned moves ---------------------------------------------------------------------------------------------
    def cell_model(self, world, zones=True):
        """The arm on its cart, with the tool's shape and (unless it's ignoring them) the zones the spine sent."""
        capsules = [(6, [v / 1000.0 for v in c["a"]], [v / 1000.0 for v in c["b"]], c["r"] / 1000.0, "the tool")
                    for c in world["tool"]["capsules"]]
        obstacles = [utra_model.Obstacle(f"the {z['name']} zone", "box", [v / 1000.0 for v in z["centre"]], z["axes"],
                                         [v / 1000.0 for v in z["size"]]) for z in world["zones"]] if zones else []
        return self.model.with_cell(capsules, obstacles)

    def plan(self, request):
        """A path from the arm's joints to the goal's, clear of everything the stub knows about (2 to 4 waypoints)."""
        deadline = time.monotonic() + TIME_SHARE * request["timeoutS"]
        world = request["world"]
        model = self.cell_model(world, zones=not self.ignore_zones)
        planner = plannermod.Planner(model)
        tool = kin.pose_to_matrix(world["tool"]["tcp"])
        start, goal = request["state"]["joints"], request["goal"]["joints"]
        limits = request.get("limits", {}).get("joints") or plannermod.DRAG_LIMITS_DEG
        checked = {}

        def clear(a, b):
            key = (tuple(a), tuple(b))
            if key not in checked:
                checked[key] = model.check_path(a, b) is None
            return checked[key]

        def free(path):
            if time.monotonic() > deadline:
                raise NoPath("ran out of time looking for a clear path")
            return all(clear(a, b) for a, b in zip(path, path[1:]))

        if free([start, goal]):
            return [start, goal], "straight there: the way is clear"
        for lift in LIFTS_MM:
            up_start, up_goal = raised(planner, start, tool, lift), raised(planner, goal, tool, lift)
            for path in ([start, up_start, goal], [start, up_goal, goal], [start, up_start, up_goal, goal]):
                if None not in path and free(path):
                    return path, f"up and over, {lift:.0f} mm above the start and goal"
        rng = random.Random(self.seed)
        for _ in range(RANDOM_TRIES):
            via = [round(min(max(s + (g - s) * rng.random() + rng.uniform(-SPREAD_DEG, SPREAD_DEG), lo), hi), 6)
                   for s, g, (lo, hi) in zip(start, goal, limits)]
            if free([start, via, goal]):
                return [start, via, goal], "through a random via point"
        raise NoPath(f"no clear path: not straight, not up and over (up to {LIFTS_MM[-1]:.0f} mm), nor through "
                     f"{RANDOM_TRIES} random via points")

    # ---- the learned skill -------------------------------------------------------------------------------------------
    def act(self, request):
        """The next burst of the skill the request names."""
        if request["skill"] != "seek":
            return {"kind": "fail", "reason": f"the stub brain has no skill called {request['skill']}"}
        params = request.get("params") or {}
        state = request["state"]
        q = state["joints"]
        planner = plannermod.Planner(self.model)
        tool = kin.pose_to_matrix(request["world"]["tool"]["tcp"])
        with self.lock:
            episode = self.episodes.get(request["episode"])
            if episode is None:
                episode = {"centre": planner.kin.tool_pose(q, tool), "turn": 0.0, "bursts": 0}
                self.episodes[request["episode"]] = episode
                while len(self.episodes) > EPISODES_KEPT:
                    self.episodes.pop(next(iter(self.episodes)))
        watched = str(params.get("input", "tool.1"))
        line, _, bit = watched.partition(".")
        inputs = state.get("inputs") or {}
        if bit.isdigit() and (inputs.get(line, 0) >> int(bit)) & 1:
            return {"kind": "done", "note": f"{watched} went high after {episode['bursts']} bursts"}
        if params.get("doneAfter") is not None and episode["bursts"] >= params["doneAfter"]:
            return {"kind": "done", "note": f"stopped looking after {episode['bursts']} bursts"}
        step, pitch = float(params.get("stepMm", 8.0)), float(params.get("pitchMm", 10.0))
        radius = float(params.get("radiusMm", 40.0))
        # An Archimedean spiral, r = pitch x turns, out to `radius`: each burst moves about `step` mm along it.
        r = min(radius, pitch * episode["turn"] / (2 * math.pi))
        episode["turn"] += step / max(r, step)
        r = min(radius, pitch * episode["turn"] / (2 * math.pi))
        target = [list(row) for row in episode["centre"]]
        target[0][3] += r * math.cos(episode["turn"]) / 1000.0
        target[1][3] += r * math.sin(episode["turn"]) / 1000.0
        try:
            joints = planner.solve(q, target, tool)
        except plannermod.PlanError as e:
            return {"kind": "fail", "reason": f"can't reach the spiral's next point ({e})"}
        episode["bursts"] += 1
        return {"kind": "move", "path": [q, [round(v, 6) for v in joints]]}


def raised(planner, joints, tool, lift_mm):
    """Joints with the tool point `lift_mm` straight above where `joints` put it, turned the same way; None if out
    of reach."""
    pose = [list(row) for row in planner.kin.tool_pose(joints, tool)]
    pose[2][3] += lift_mm / 1000.0
    try:
        return [round(v, 6) for v in planner.solve(joints, pose, tool)]
    except plannermod.PlanError:
        return None


def summary(request, reply):
    if request["type"] == "plan":
        return f"{len(reply['path'])} waypoints, {reply['note']}" if reply["ok"] else f"no path: {reply['reason']}"
    if request["type"] == "act":
        action = reply["action"]
        return f"{request['skill']} tick {request['tick']}: {action['kind']}" + \
            (f" ({action.get('note') or action.get('reason')})" if action["kind"] in ("done", "fail") else "")
    if request["type"] == "end":
        return f"episode {request['episode']} ended: {request['outcome']}"
    return reply.get("brain", "")


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = os.name != "nt"  # on Windows it would let a second server take the port over

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], ConnectionError):
            return  # the spine stopped waiting for this answer (its deadline, a STOP or a pause), as it may
        super().handle_error(request, client_address)


def make_handler(brain, quiet=False):
    kinds = {path: kind for kind, path in bp.PATHS.items()}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "StubBrain/1"

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, payload):
            data = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self._send(405, {"error": f"POST a version {bp.VERSION} request to one of {', '.join(bp.PATHS.values())}"})

        def do_POST(self):
            kind = kinds.get(self.path)
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                length = -1
            if kind is None:
                return self._send(404, {"error": f"no such path: {self.path}"})
            if not 0 < length <= bp.MAX_BYTES:
                return self._send(400, {"error": "a request needs a body of at most 1 MiB"})
            started = time.monotonic()
            try:
                request = bp.check_request(bp.decode(self.rfile.read(length)), kind)
            except bp.ProtocolError as e:
                return self._send(400, {"error": f"the request {e}"})
            try:
                reply = brain.handle(request)
            except Exception as e:  # a bug in the stub: say so rather than hang up
                return self._send(500, {"error": f"the stub brain failed: {e!r}"})
            if not quiet:
                print(f"{time.strftime('%H:%M:%S')} {kind} {request['id']}: {summary(request, reply)} "
                      f"({time.monotonic() - started:.2f} s)", flush=True)
            self._send(200, reply)

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description="A stub brain for the spine, speaking docs/brain-interface.md.")
    parser.add_argument("--host", default="127.0.0.1", help="address to serve on (default 127.0.0.1, this PC only)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"port (default {DEFAULT_PORT})")
    parser.add_argument("--ignore-zones", action="store_true",
                        help="play a buggy brain whose world model has lost the cell's keep-out zones")
    parser.add_argument("--seed", type=int, default=1, help="seed for the random via points (default 1)")
    parser.add_argument("--quiet", action="store_true", help="don't print a line per request")
    args = parser.parse_args(argv)
    try:
        local = args.host == "localhost" or ipaddress.ip_address(args.host).is_loopback
    except ValueError:
        local = False
    if not local:
        print(f"Warning: serving on {args.host}, beyond this PC's loopback; the interface has no authentication.",
              file=sys.stderr)
    brain = StubBrain(args.ignore_zones, args.seed)
    try:
        server = Server((args.host, args.port), make_handler(brain, args.quiet))
    except OSError as e:
        print(f"Error: can't serve on {args.host}:{args.port} ({e}).", file=sys.stderr)
        return 1
    print(f"Stub brain on http://{args.host}:{args.port}, protocol version {bp.VERSION}"
          + (", ignoring keep-out zones (a buggy brain)" if args.ignore_zones else "") + ". Ctrl+C to quit.", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
