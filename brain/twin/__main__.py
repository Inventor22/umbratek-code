"""The twin's command line. Run from the repo's root with the brain's environment (brain/README.md):

    python -m brain.twin serve [--host 127.0.0.1] [--scene FILE] [--cell DIR] [--viewer] [utra_sim options]
        The simulated controller (tools/utra_sim.py) served from this process, on HOST:502, :30001, :30003 and
        :30099, exactly as the simulator serves them, with the twin following it: point the jog panel, arm.py or the
        brain at HOST as at the real controller. Camera frames, state and obstacles at http://HOST:30100 (server.py).
        --viewer also starts the viewer (below) in a process of its own. Ctrl+C stops the twin; closing the viewer
        doesn't.
    python -m brain.twin view [--follow http://127.0.0.1:30100] [--check SECONDS]
                                       MuJoCo's viewer on a running twin, read-only, in a process of its own (on
                                       Windows natively while the twin runs in WSL2): viewer.py. --check follows it
                                       without a window for that long and says what arrived.
    python -m brain.twin replay        Replays both captures through the simulator and the twin; prints every
                                       figure against its tolerance and why (exit 1 if any fails).
    python -m brain.twin snapshot [--scene FILE] [--joints J1,...,J6] [--out DIR]
                                       Renders every camera of the scene once and saves rgb, depth and meta files.
    python -m brain.twin export [--scene FILE] OUT_DIR
                                       Writes the generated MJCF and its meshes (to open in MuJoCo's viewer, or Newton).
    python -m brain.twin check         Shows the environment (MuJoCo, rendering backend) and renders a test frame.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

from . import model as twin_model
from . import repo
from .model import CameraSpec, Scene

HOME_DEG = [0.0, -30.0, 50.0, -10.0, 90.0, 0.0]  # tools/utra_sim.py's starting pose
OVERVIEW = dict(name="overview", pose={"eye": [1600.0, -900.0, 900.0], "target": [250.0, 0.0, -150.0]})
PERIOD_S = 0.01  # the twin's loop: sample the controller, follow it, check contacts


def load_scene(args):
    scene = Scene.load(args.scene) if args.scene else Scene()
    if getattr(args, "cell", None):
        scene.cell = os.path.abspath(args.cell)
    return scene


def cmd_serve(args, sim_args):
    from . import coupling, server
    from .twin import Twin
    utra_sim = repo.spine("utra_sim")
    scene = load_scene(args)
    twin = Twin(scene)
    host = coupling.Host(args.host, sim_args)
    api = server.TwinServer(twin, args.host, args.port, PERIOD_S)
    url = f"http://{args.host}:{args.port}"
    print(f"twin: the simulated controller is on {args.host} (UTRC :502, status :30001, config :30003, control "
          f":{utra_sim.CONTROL_PORT}); camera frames, state and obstacles on {url}/state", flush=True)
    print(f"twin: {len(twin.built.obstacles)} obstacle(s), cameras: {', '.join(twin.built.cameras) or 'none'}"
          + (f"; cell {scene.cell}" if scene.cell else ""), flush=True)
    viewer = None
    if args.viewer:
        viewer = subprocess.Popen([sys.executable, "-m", "brain.twin", "view", "--follow", url], cwd=repo.ROOT)
        print(f"twin: the viewer is process {viewer.pid}; closing it leaves the twin running", flush=True)
    print("twin: Ctrl+C stops the twin", flush=True)
    touching = set()
    tick = utra_sim.clock()
    try:
        while True:
            api.cadence.start()
            t, q, state = host.sample()
            twin.follow(t, q)
            now = {(c["a"], c["b"]) for c in twin.contacts()}
            for a, b in sorted(now - touching):
                print(f"twin: {a} touches {b}", flush=True)
            for a, b in sorted(touching - now):
                print(f"twin: {a} clear of {b}", flush=True)
            touching = now
            api.publish(twin.snapshot())
            api.process(t, state)
            api.cadence.end()
            tick = utra_sim.next_tick(tick, PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None and viewer.poll() is None:
            viewer.terminate()
        api.close()
        twin.close()


def cmd_view(args):
    from . import viewer
    return viewer.main(args.follow, args.check)


def cmd_replay(args):
    from . import replay
    from .twin import Twin
    t0 = time.perf_counter()
    results = replay.verify(Twin())
    if args.json:
        print(json.dumps(results, indent=1))
    else:
        for r in results:
            print(f"{'PASS' if r['ok'] else 'FAIL'}  {r['check']}: {r['value']:.4g} {r['unit']} (limit {r['limit']:g})")
            print(f"      why: {r['why']}")
        failed = sum(not r["ok"] for r in results)
        print(f"\n{len(results) - failed}/{len(results)} within tolerance ({time.perf_counter() - t0:.0f} s)")
    return 1 if any(not r["ok"] for r in results) else 0


def cmd_snapshot(args):
    from .twin import Twin
    scene = load_scene(args)
    if not scene.cameras:
        scene.cameras.append(CameraSpec(**OVERVIEW))
    twin = Twin(scene)
    joints = [float(v) for v in args.joints.split(",")] if args.joints else HOME_DEG
    twin.set_joints([math.radians(v) for v in joints])
    for name in twin.built.cameras:
        frame = twin.render(name)
        paths = frame.save(args.out, name)
        print(f"{name}: {', '.join(paths.values())}")
    twin.close()
    return 0


def cmd_export(args):
    built = twin_model.build(load_scene(args))
    print(twin_model.export(built, args.out))
    return 0


def cmd_check(args):
    import mujoco
    import numpy as np

    from .twin import Twin
    print(f"Python {sys.version.split()[0]}, MuJoCo {mujoco.__version__}, NumPy {np.__version__}, "
          f"rendering: {os.environ.get('MUJOCO_GL', 'glfw (default)')}")
    t0 = time.perf_counter()
    twin = Twin(Scene(cameras=[CameraSpec(**OVERVIEW)]))
    print(f"model generated and compiled in {time.perf_counter() - t0:.2f} s: {twin.model.nbody} bodies, "
          f"{twin.model.ngeom} geoms")
    twin.set_joints([math.radians(v) for v in HOME_DEG])
    t0 = time.perf_counter()
    frame = twin.render("overview")
    print(f"rendered {frame.width} x {frame.height} RGB-D in {time.perf_counter() - t0:.2f} s; depth "
          f"{frame.depth[frame.depth > 0].min():.2f} to {frame.depth.max():.2f} m; sees "
          f"{', '.join(sorted(set(frame.labels.values())))}")
    twin.close()
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m brain.twin", description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve", help="serve the simulated controller with the twin following it")
    s.add_argument("--host", default="127.0.0.1", help="loopback address to serve on (default 127.0.0.1)")
    s.add_argument("--port", type=int, default=30100, help="the twin's HTTP port (default 30100)")
    s.add_argument("--scene", help="scene file (JSON): cell, obstacles, cameras")
    s.add_argument("--cell", help="cell folder (cell.json), overriding the scene's")
    s.add_argument("--viewer", action="store_true",
                   help="also start the viewer (python -m brain.twin view), in a process of its own")
    v = sub.add_parser("view", help="MuJoCo's viewer on a running twin, read-only")
    v.add_argument("--follow", default="http://127.0.0.1:30100", metavar="URL",
                   help="the twin's HTTP address (default http://127.0.0.1:30100)")
    v.add_argument("--check", type=float, metavar="SECONDS",
                   help="follow it for this long without a window, and say what arrived")
    r = sub.add_parser("replay", help="replay both captures and check every figure against its tolerance")
    r.add_argument("--json", action="store_true", help="print the results as JSON")
    n = sub.add_parser("snapshot", help="render the scene's cameras once")
    n.add_argument("--scene")
    n.add_argument("--cell")
    n.add_argument("--joints", help="joint angles, deg, comma-separated (default the simulator's start pose)")
    n.add_argument("--out", default="twin-frames", help="folder for the files (default ./twin-frames)")
    e = sub.add_parser("export", help="write the generated MJCF and meshes")
    e.add_argument("--scene")
    e.add_argument("--cell")
    e.add_argument("out")
    sub.add_parser("check", help="check the environment and render a test frame")
    args, rest = p.parse_known_args(argv)
    if rest and args.cmd != "serve":
        p.error(f"unrecognised arguments: {' '.join(rest)}")
    if args.cmd == "serve":
        return cmd_serve(args, rest)
    return {"view": cmd_view, "replay": cmd_replay, "snapshot": cmd_snapshot, "export": cmd_export,
            "check": cmd_check}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
