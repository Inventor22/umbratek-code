"""The twin (brain/twin), run in the brain's environment (brain/.venv; see brain/README.md):

- its model is the repo's own: the arm's kinematics, collision capsules, cart, floor and joint limits are the ones
  scripts/utra_model.py and tools/utra_sim.py use, and a cell's tool, frames, zones and targets come through
  scripts/cell.py;
- obstacles can be placed, moved and removed, fixed or free, and the physics and contacts behave;
- its cameras: intrinsics (any fx, fy, cx, cy), depth in metres, and true poses, checked by projecting known points
  and back-projecting depth onto known surfaces;
- its coupling to tools/utra_sim.py: on a virtual clock, fed the SDK's own bytes, and live, served from the twin's
  process and driven through the SDK, with frames and state over HTTP;
- every HTTP server here (the jog panel's, the stub brain's and the twin's) binding without a reverse DNS lookup, each
  made in its own environment with socket's lookups recorded, and no module but scripts/loopback_http.py using
  http.server's own server classes;
- camera frames off the loop: snapshots of the twin rendered in a process of their own, pixel for pixel the twin's
  own frames, and the loop's 10 ms cadence and the controller's 10 Hz status held while frames are pulled;
- the viewer (brain/twin/viewer.py): following the twin from a process of its own, read-only, through moves,
  obstacles added, moved and removed, and the twin restarting; and the twin carrying on when a viewer ends;
- the replays of both captures, every figure against its stated tolerance (brain/twin/replay.py's verify()).

Usage: brain/.venv/Scripts/python tests/twin_tests.py [section ...]   (from tests/run_all.py: python tests/run_all.py
twin). TWIN_TEST_WINDOW=1, on Windows, also opens the viewer's real window (serve --viewer) and closes it as its close
button does; without it no window opens.
"""
import io
import json
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from simrig import HOST, PY, log_path, wait_port  # noqa: E402

import numpy as np  # noqa: E402

from brain.twin import CameraSpec, Controller, Obstacle, Scene, SceneError, Twin, build, repo, replay  # noqa: E402
from brain.twin import model as twin_model  # noqa: E402
from brain.twin.cameras import read_png  # noqa: E402

results = []
HTTP_PORT = 30100
EXAMPLE = os.path.join(ROOT, "brain", "twin", "scenes", "example.json")
utra_model = repo.spine("utra_model")
utra_sim = repo.spine("utra_sim")
kin = repo.spine("kinematics")
cellmod = repo.spine("cell")


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)}", flush=True)


def rad(q):
    return [math.radians(v) for v in q]


def geom_ends(twin, g):
    """A capsule geom's two end points in the base frame (a sphere's centre twice)."""
    import mujoco
    c, R = twin.data.geom_xpos[g], twin.data.geom_xmat[g].reshape(3, 3)
    if twin.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
        return c, c
    h = twin.model.geom_size[g][1]
    return c - h * R[:, 2], c + h * R[:, 2]


# ---- the model, from the repo's sources ---------------------------------------------------------------------------

def model_from_sources():
    print("== the model is the repo's own: kinematics, capsules, cart, floor, limits")
    import mujoco
    a, b = build(), build()
    check("generated the same way every time (MJCF and meshes)", a.xml == b.xml and a.assets == b.assets)
    check("from the URDFs' meshes: the arm's 7 and the cart's 3", sorted(a.assets) == sorted(
        [f"arm_{n}.stl" for n in ("base", "link1", "link2", "link3", "link4", "link5", "link6")]
        + ["cart_casters.stl", "cart_controller.stl", "cart_frame.stl"]), sorted(a.assets))
    twin = Twin()
    arm = utra_model.ArmModel()
    rng = random.Random(7)
    worst_T = worst_caps = 0.0
    capsule_geoms = [g for g, info in enumerate(twin.geoms) if info and info[0] == "arm"]
    for _ in range(300):
        q = [rng.uniform(-179, 179) for _ in range(5)] + [rng.uniform(-179, 179)]
        q[2] = rng.uniform(-164, 164)
        twin.set_joints(rad(q))
        frames = arm.frames(q)
        for T_twin, T_spine in zip(twin.link_poses(), frames):
            worst_T = max(worst_T, np.abs(T_twin - np.array(T_spine)).max())
        for g, (p, r) in zip(capsule_geoms, arm.placed(q)):
            e1, e2 = geom_ends(twin, g)
            worst_caps = max(worst_caps, min(max(np.linalg.norm(e1 - p), np.linalg.norm(e2 - r)),
                                             max(np.linalg.norm(e1 - r), np.linalg.norm(e2 - p))))
    check(f"every link's pose is utra_model's, at 300 random poses (worst {worst_T:.1e})", worst_T < 1e-9, worst_T)
    check(f"and so is every collision capsule ({len(capsule_geoms)} of {len(arm.capsules)}; worst {worst_caps:.1e} m)",
          len(capsule_geoms) == len(arm.capsules) and worst_caps < 1e-9, (len(capsule_geoms), worst_caps))
    cart = [o for o in arm.obstacles if o.body is None and o.label != "the floor"]
    cart_geoms = [g for g, info in enumerate(twin.geoms) if info and info[0] == "cart"]
    worst = 0.0
    for g, o in zip(cart_geoms, cart):
        size = twin.model.geom_size[g]
        if o.kind == "box":
            worst = max(worst, np.abs(twin.data.geom_xpos[g] - o.centre).max(), np.abs(size - o.half).max(),
                        np.abs(twin.data.geom_xmat[g].reshape(3, 3) - np.array(o.axes).T).max())
        else:
            worst = max(worst, np.abs(twin.data.geom_xpos[g] - o.centre).max(), abs(size[0] - o.radius),
                        abs(size[1] - o.half_length))
    check(f"the cart's {len(cart)} collision shapes are the cart URDF's, where utra_model puts them (worst {worst:.1e})",
          len(cart_geoms) == len(cart) and worst < 1e-9, (len(cart_geoms), worst))
    floor = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    check(f"the floor is at utra_model's height ({arm.floor_z * 1000:.2f} mm)",
          abs(twin.model.geom_pos[floor][2] - arm.floor_z) < 1e-12)
    ranges = twin.model.jnt_range[[twin.model.joint(n).id for n in twin_model.JOINTS]]
    check("joint limits are the controller's (utra_sim.JOINT_LIMITS_RAD)",
          np.abs(ranges - np.array(utra_sim.JOINT_LIMITS_RAD)).max() < 1e-12, ranges)
    twin.set_joints(rad([0, -30, 50, -10, 90, 0]))
    check("at the simulator's start pose nothing touches", twin.contacts(include_world=True) == [],
          twin.contacts(include_world=True))
    size = twin.model.stat.meansize
    check(f"the viewer's markers (centres of mass, joints, frames) are sized for the arm: stat.meansize "
          f"{size * 1000:.0f} mm, so a centre of mass is a ball {2 * twin.model.vis.scale.com * size * 1000:.0f} mm "
          f"across (the compiler's guess, from the cart and the floor, made it 320)", abs(size - 0.05) < 1e-9, size)
    folder = tempfile.mkdtemp(prefix="twin-export-")
    try:
        path = twin_model.export(a, folder)
        m = mujoco.MjModel.from_xml_path(path)
        check("export writes an MJCF that loads from disk on its own", m.ngeom == twin.model.ngeom and
              m.nbody == twin.model.nbody, (m.ngeom, m.nbody))
    finally:
        shutil.rmtree(folder, ignore_errors=True)
    twin.close()


# ---- the cell -------------------------------------------------------------------------------------------------------

def make_cell(folder):
    cell = cellmod.Cell(folder)
    cell.set_tool("probe", {"xyz": [0.0, 0.0, 120.0], "rpy": [0.0, 0.0, 30.0]}, mass=0.3, cog=(0.0, 0.0, 50.0),
                  capsules=[{"a": [0.0, 0.0, 0.0], "b": [0.0, 0.0, 100.0], "r": 20.0}])
    cell.activate_tool("probe")
    cell.set_frame("fixture", "base", {"xyz": [600.0, -150.0, -100.0], "rpy": [0.0, 0.0, 25.0]})
    cell.set_zone("clamp", "fixture", {"xyz": [50.0, 40.0, 30.0], "rpy": [0.0, 0.0, 10.0]}, [80.0, 60.0, 60.0])
    cell.set_target("above", "fixture", {"xyz": [0.0, 0.0, 200.0], "rpy": [180.0, 0.0, 0.0]})
    return cell


def cell_parts():
    print("== a cell's tool, frames, zones and targets, through scripts/cell.py")
    import mujoco
    folder = tempfile.mkdtemp(prefix="twin-cell-")
    try:
        cell = make_cell(folder)
        twin = Twin(Scene(cell=folder))
        twin.set_joints(rad([0, -30, 50, -10, 90, 0]))
        check("the tool point is the flange times the cell's tool", np.abs(
            twin.tcp() - twin.flange() @ np.array(cell.tool_matrix())).max() < 1e-12)
        tool = [g for g, info in enumerate(twin.geoms) if info and info[0] == "tool"]
        caps = cell.tool_capsules()
        e1, e2 = geom_ends(twin, tool[0]) if tool else (None, None)
        F = twin.flange()
        want = [F[:3, :3] @ np.array(caps[0][1]) + F[:3, 3], F[:3, :3] @ np.array(caps[0][2]) + F[:3, 3]]
        check("the tool's capsule rides on the flange, where cell.tool_capsules() says", len(tool) == 1 and min(
            np.linalg.norm(e1 - want[0]) + np.linalg.norm(e2 - want[1]),
            np.linalg.norm(e1 - want[1]) + np.linalg.norm(e2 - want[0])) < 1e-9, tool)
        z = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_GEOM, "zone_clamp")
        o = cell.zone_obstacles()[0]
        check("the keep-out zone is the box cell.zone_obstacles() makes, in the base frame", z >= 0 and np.abs(
            twin.data.geom_xpos[z] - o.centre).max() < 1e-9 and np.abs(twin.model.geom_size[z] - o.half).max() < 1e-12
            and np.abs(twin.data.geom_xmat[z].reshape(3, 3) - np.array(o.axes).T).max() < 1e-9)
        check("zones are for the eye only: no contacts, not seen by cameras (group 4)",
              twin.model.geom_contype[z] == 0 and twin.model.geom_conaffinity[z] == 0
              and twin.model.geom_group[z] == twin_model.GROUP_ZONES)
        site = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_SITE, "target_above")
        T = np.array(cell.target_matrix("above"))
        check("targets and frames are marked where the cell has them", site >= 0 and np.abs(
            twin.data.site_xpos[site] - T[:3, 3]).max() < 1e-9 and mujoco.mj_name2id(
            twin.model, mujoco.mjtObj.mjOBJ_SITE, "frame_fixture") >= 0)
        gap, part = twin.zone_clearances()["clamp"]
        check(f"zone clearance: the arm is clear of the zone at the start pose ({gap:.0f} mm)", gap > 10.0, gap)
        # Put the zone around the tool and look again.
        tcp = twin.tcp()[:3, 3] * 1000.0
        cell.set_zone("clamp", "base", {"xyz": list(tcp), "rpy": [0.0, 0.0, 0.0]}, [60.0, 60.0, 60.0])
        twin2 = Twin(Scene(cell=folder))
        twin2.set_joints(rad([0, -30, 50, -10, 90, 0]))
        gap, part = twin2.zone_clearances()["clamp"]
        check(f"and inside it, once the zone is moved onto the tool ({gap:.0f} mm, {part})", gap < 0 and
              part in ("the tool", "the flange"), (gap, part))
        twin.close()
        twin2.close()
    finally:
        shutil.rmtree(folder, ignore_errors=True)


# ---- obstacles --------------------------------------------------------------------------------------------------------

def obstacles():
    print("== obstacles: placed, moved, removed; fixed and free; contacts")
    folder = tempfile.mkdtemp(prefix="twin-cell-")
    try:
        make_cell(folder)
        table = Obstacle("table", "box", [700, 900, 30], {"xyz": [650, 0, -115], "rpy": [0, 0, 0]})
        twin = Twin(Scene(cell=folder, obstacles=[table]))
        T = twin.obstacle_pose("table")
        check("a fixed obstacle stands where it's placed (base frame)", np.abs(
            T - np.array(kin.pose_to_matrix(table.pose))).max() < 1e-9)
        twin.add_obstacle(Obstacle("jig", "cylinder", [30, 80], {"xyz": [10, 20, 40], "rpy": [90, 0, 0]},
                                   frame="fixture"))
        want = np.array(cellmod.Cell(folder).frame_matrix("fixture")) @ np.array(
            kin.pose_to_matrix({"xyz": [10, 20, 40], "rpy": [90, 0, 0]}))
        check("one placed in a cell frame stands in that frame", np.abs(twin.obstacle_pose("jig") - want).max() < 1e-9)
        twin.move_obstacle("jig", {"xyz": [100, 0, 40], "rpy": [0, 0, 0]})
        want = np.array(cellmod.Cell(folder).frame_matrix("fixture")) @ np.array(
            kin.pose_to_matrix({"xyz": [100, 0, 40], "rpy": [0, 0, 0]}))
        check("and moves without a rebuild", np.abs(twin.obstacle_pose("jig") - want).max() < 1e-9)

        # Something in the arm's way: a box around the flange.
        q0 = rad([0, -30, 50, -10, 90, 0])
        twin.set_joints(q0)
        flange = twin.flange()[:3, 3] * 1000.0
        twin.add_obstacle(Obstacle("post", "box", [60, 60, 60], {"xyz": list(flange), "rpy": [0, 0, 0]}))
        twin.set_joints(q0)
        touching = twin.contacts()
        check("the arm in an obstacle: contacts() names the arm's part and the obstacle", any(
            c["b"] == "post" and c["kinds"][0] in ("arm", "tool") and c["depth_mm"] > 0 for c in touching), touching)
        twin.remove_obstacle("post")
        twin.set_joints(q0)
        check("removed, it's gone", "post" not in twin.built.obstacles and not any(c["b"] == "post" for c in
                                                                                  twin.contacts()))

        # A free part dropped onto the table comes to rest on it.
        twin.add_obstacle(Obstacle("part", "box", [40, 40, 40], {"xyz": [600, 200, -20], "rpy": [0, 0, 30]},
                                   fixed=False, mass=0.2))
        t = 0.0
        for _ in range(150):
            t += 0.01
            twin.follow(t, q0)
        z = twin.obstacle_pose("part")[2, 3] * 1000.0
        check(f"a free part dropped on the table comes to rest on it (centre at {z:.1f} mm; table top -100 + 20)",
              abs(z - (-80.0)) < 2.0, z)
        kept = twin.obstacle_pose("part").copy()
        twin.add_obstacle(Obstacle("marker", "sphere", [10], {"xyz": [0, 600, 0], "rpy": [0, 0, 0]}))
        check("adding an obstacle keeps the others' state (the part stays where it fell)",
              np.abs(twin.obstacle_pose("part") - kept).max() < 1e-6)

        # The arm sweeps through the part and pushes it: the flange 40 mm above the table, pointing down, turning
        # about J1 across where the part stands.
        k = kin.Kinematics(utra_model.ArmModel())
        reach = 650.0
        low = k.solve([0, 10, 100, -20, 90, 0], kin.pose_to_matrix({"xyz": [reach, 0, -60], "rpy": [180, 0, 0]}),
                      cellmod.JOINT_LIMITS, kin.IDENTITY)
        check("(the spine's IK puts the flange 40 mm above the table)", low["ok"], low)
        sweep_from, sweep_to = list(low["joints"]), list(low["joints"])
        sweep_from[0], sweep_to[0] = -15.0, 35.0
        angle = math.radians(10.0)
        twin.move_obstacle("part", {"xyz": [reach * math.cos(angle), reach * math.sin(angle), -80], "rpy": [0, 0, 0]})
        twin.set_joints(rad(sweep_from))
        twin.t = None
        before = twin.obstacle_pose("part")[:3, 3].copy()
        pushed = False
        for step in range(1, 201):
            t += 0.01
            twin.follow(t, rad([a + (b - a) * step / 200 for a, b in zip(sweep_from, sweep_to)]))
            pushed = pushed or any(c["b"] == "part" for c in twin.contacts())
        moved = np.linalg.norm(twin.obstacle_pose("part")[:3, 3] - before) * 1000.0
        check(f"the arm sweeping through a free part pushes it ({moved:.0f} mm)", pushed and moved > 20.0,
              (pushed, moved))
        for bad, why in ((dict(name="x y", shape="box", size=[1, 1, 1]), "a bad name"),
                         (dict(name="b", shape="cone", size=[1]), "an unknown shape"),
                         (dict(name="b", shape="box", size=[1, 1]), "the wrong number of sizes"),
                         (dict(name="table", shape="box", size=[1, 1, 1]), "a name in use")):
            try:
                twin.add_obstacle(Obstacle(**bad))
                ok = False
            except SceneError:
                ok = True
            check(f"refused: {why}", ok)
        twin.close()
    finally:
        shutil.rmtree(folder, ignore_errors=True)


# ---- cameras ----------------------------------------------------------------------------------------------------------

def cameras():
    print("== cameras: intrinsics, depth and true poses")
    import mujoco
    points = [(700, -150, -60), (900, 120, -40), (650, 60, -90), (800, -40, 20), (1000, 250, -70)]
    spheres = [Obstacle(f"dot{k}", "sphere", [8], {"xyz": list(p), "rpy": [0, 0, 0]}, rgba=[0.9, 0.2, 0.2, 1])
               for k, p in enumerate(points)]
    table = Obstacle("table", "box", [700, 900, 30], {"xyz": [650, 0, -115], "rpy": [0, 0, 0]})
    oblique = CameraSpec("oblique", width=800, height=600, fx=700, fy=640, cx=380.0, cy=310.0,
                         pose={"eye": [200, -900, 500], "target": [800, 50, -60]})
    down = CameraSpec("down", width=320, height=240, fx=300, fy=300,
                      pose={"eye": [2200, 900, 700], "target": [2200, 900, -500], "up": [1, 0, 0]})
    wrist = CameraSpec("wrist", mount="flange", width=640, height=480, fx=385, fy=385,
                       pose={"xyz": [0, 75, 30], "rpy": [0, 0, 0]})
    twin = Twin(Scene(obstacles=[table] + spheres, cameras=[oblique, down, wrist]))
    twin.set_joints(rad([0, -30, 50, -10, 90, 0]))
    f = twin.render("oblique")
    check("a frame: RGB, depth and segmentation at the camera's size", f.rgb.shape == (600, 800, 3)
          and f.depth.shape == (600, 800) and f.segmentation.shape == (600, 800) and f.rgb.dtype == np.uint8)
    T_spec = twin_model.look_at(oblique.pose["eye"], oblique.pose["target"])
    check("its true pose is the one asked for (eye, target)", np.abs(f.T_base_camera - T_spec).max() < 1e-9,
          f.T_base_camera)
    uv, z = f.project(np.array(points) / 1000.0)
    worst = 0.0
    for k in range(len(points)):
        g = mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_GEOM, f"obstacle_dot{k}")
        vs, us = np.nonzero(f.segmentation == g)
        worst = max(worst, math.hypot(us.mean() - uv[k][0], vs.mean() - uv[k][1]))
    check(f"5 marked points land where K (fx 700, fy 640, cx 380, cy 310) and the pose put them: worst {worst:.2f} px",
          worst < 0.3, worst)
    on_table = f.segmentation == mujoco.mj_name2id(twin.model, mujoco.mjtObj.mjOBJ_GEOM, "obstacle_table")
    table_px = f.points()[on_table[f.depth > 0]]
    d = np.abs(table_px - [0.65, 0.0, -0.115]) - [0.35, 0.45, 0.015]  # the table, axis-aligned: centre, half sizes
    surface = np.where((d > 0).any(axis=1), np.linalg.norm(np.maximum(d, 0), axis=1), -d.max(axis=1)) * 1000.0
    check(f"depth back-projected through K and the pose lands on the table's faces within 0.1 mm ({len(table_px)} "
          f"points, worst {surface.max():.4f} mm)", len(table_px) > 1000 and surface.max() < 0.1, surface.max())
    floor_z = utra_model.ArmModel().floor_z
    d = twin.render("down")
    floor = d.points()
    check(f"a camera looking straight down 1.4 m onto the floor reads depth 1.430 m everywhere ("
          f"{d.depth.min():.4f} to {d.depth.max():.4f})", np.abs(d.depth - (0.7 - floor_z)).max() < 0.0005,
          (d.depth.min(), d.depth.max()))
    check("and its points lie on the floor plane within 0.5 mm", np.abs(floor[:, 2] - floor_z).max() < 0.0005)
    far = CameraSpec("far", width=160, height=120, fx=150, fy=150, far=1.0,
                     pose={"eye": [2200, 900, 700], "target": [2200, 900, -500], "up": [1, 0, 0]})
    twin.add_camera(far)
    check("beyond 'far' depth reads 0, as a depth camera reports no data", (twin.render("far").depth == 0).all())
    T1 = twin.render("wrist").T_base_camera
    M = np.array(kin.pose_to_matrix(wrist.pose))
    ok1 = np.abs(T1 - twin.flange() @ M).max() < 1e-9
    twin.set_joints(rad([30, -10, 80, 20, 60, 45]))
    T2 = twin.render("wrist").T_base_camera
    check("a camera on the flange moves with it: its true pose is flange times mount, at two poses",
          ok1 and np.abs(T2 - twin.flange() @ M).max() < 1e-9 and np.abs(T2 - T1).max() > 0.01)
    folder = tempfile.mkdtemp(prefix="twin-frame-")
    try:
        paths = f.save(folder, "oblique")
        meta = json.load(open(paths["meta"]))
        check("saved: rgb.png, 16-bit depth.png in mm, segmentation.png and meta.json with K and T",
              (read_png(paths["rgb"]) == f.rgb).all()
              and (read_png(paths["depth"]) == np.round(f.depth * 1000).astype(np.uint16)).all()
              and (read_png(paths["segmentation"]).astype(int) - 1 == f.segmentation).all()
              and np.allclose(meta["K"], f.K) and np.allclose(meta["T_base_camera"], f.T_base_camera))
        with np.load(__import__("io").BytesIO(f.npz())) as z:
            check("and as .npz: rgb, depth, segmentation, K, T_base_camera, meta",
                  {"rgb", "depth", "segmentation", "K", "T_base_camera", "meta"} <= set(z.files)
                  and (z["depth"] == f.depth).all())
    finally:
        shutil.rmtree(folder, ignore_errors=True)
    twin.update_camera("oblique", pose={"eye": [300, -900, 600], "target": [800, 50, -60]})
    check("a camera can be moved", np.abs(twin.render("oblique").T_base_camera - twin_model.look_at(
        [300, -900, 600], [800, 50, -60])).max() < 1e-9)
    twin.close()


# ---- coupling, on a virtual clock -----------------------------------------------------------------------------------

def coupling_virtual():
    print("== coupling on a virtual clock: the SDK's bytes in, utra_sim's motion out, the twin following")
    start = [10.0, -20.0, 60.0, 0.0, 80.0, 0.0]
    goal = [40.0, 0.0, 40.0, 30.0, 60.0, 90.0]
    ctrl = Controller(start)
    ctrl.enable()
    ctrl.advance(1.0)
    q0 = ctrl.joints()
    ctrl.moveto_joint_p2p(rad(goal), math.radians(30), math.radians(150))
    sent = ctrl.now
    f32 = (lambda v: float(np.float32(v)))  # what crosses the wire: the SDK sends float32
    m = utra_sim.JointMotion("ref", q0, [f32(v) for v in rad(goal)], f32(math.radians(30)), f32(math.radians(150)),
                             ctrl.arm.config["joint_jerk"])
    twin = Twin()
    worst = worst_twin = 0.0
    t = sent
    while t < sent + m.duration + 0.5:
        t += 0.013
        ctrl.advance(t)
        q = ctrl.joints(t)
        ref = m.joints(max(0.0, t - sent - utra_sim.START_DELAY_JOINT_S))
        worst = max(worst, max(abs(a - b) for a, b in zip(q, ref)))
        twin.follow(t, q)
        worst_twin = max(worst_twin, np.abs(twin.data.qpos[twin.qadr] - q).max())
    check(f"a joint move sent as the SDK's MOVEJ_P2P bytes runs as utra_sim's own profile (worst {worst:.1e} rad)",
          worst < 1e-12, worst)
    check(f"and the twin's joints are the controller's at every step (worst {worst_twin:.1e} rad)", worst_twin == 0.0)
    check("it ends at the goal, ready", np.abs(np.array(ctrl.joints()) - [f32(v) for v in rad(goal)]).max() < 1e-12
          and ctrl.arm.status == 2, ctrl.arm.status)
    pose = ctrl.pose()
    target = list(pose)
    target[0] += 60.0
    target[2] -= 40.0
    ctrl.moveto_cartesian_line(target, 50.0, 200.0)
    a, b = np.array(pose[:3]), np.array(target[:3])
    u = (b - a) / np.linalg.norm(b - a)
    worst_line, t, done = 0.0, ctrl.now, None
    t_start = t
    while t < t_start + 30 and (done is None or t < done + 0.2):
        t += 0.02
        ctrl.advance(t)
        twin.follow(t, ctrl.joints(t))
        p = twin.tcp()[:3, 3] * 1000.0
        worst_line = max(worst_line, np.linalg.norm((p - a) - np.dot(p - a, u) * u))
        if done is None and not ctrl.busy:
            done = t
    end = twin.tcp()[:3, 3] * 1000.0
    check(f"a straight line through the twin: its tool stays within 0.01 mm of the line ({worst_line:.4f}) and ends "
          f"at the goal ({np.linalg.norm(end - b):.4f} mm)", worst_line < 0.01 and np.linalg.norm(end - b) < 0.01,
          (worst_line, end, b))
    ctrl.moveto_joint_p2p(rad(start), math.radians(30), math.radians(150))
    ctrl.advance(ctrl.now + 0.6)
    ctrl.stop()
    ctrl.advance(ctrl.now + 0.6)
    q1 = ctrl.joints()
    ctrl.advance(ctrl.now + 0.5)
    check("STOP: the controller stops (state 4) and the arm stays put", ctrl.arm.status == 4
          and np.abs(np.array(ctrl.joints()) - q1).max() < 1e-12)
    twin.close()


# ---- live: served from the twin's process, driven through the SDK ----------------------------------------------

def http(path, body=None, headers=None):
    req = urllib.request.Request(f"http://{HOST}:{HTTP_PORT}{path}", data=None if body is None else json.dumps(body).encode(),
                                 headers=dict({"Content-Type": "application/json"} if body is not None else {},
                                              **(headers or {})))
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def live():
    print("== live: the twin serving the simulated controller, driven through the SDK")
    sys.path.insert(0, repo.SCRIPTS)
    import arm as armcli
    log = log_path("twin_serve.log")
    with open(log, "w") as out:
        proc = subprocess.Popen([sys.executable, "-m", "brain.twin", "serve", "--host", HOST, "--port", str(HTTP_PORT),
                                 "--scene", EXAMPLE], cwd=ROOT, stdout=out, stderr=subprocess.STDOUT)
    arm = None
    try:
        up = wait_port(502, timeout=30) and wait_port(HTTP_PORT, timeout=10)
        check("python -m brain.twin serve listens on :502 (and the status, config and control ports) and :30100", up,
              open(log).read()[-2000:])
        if not up:
            return
        report = armcli.StatusReport(HOST)
        times, frames = [], []
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 3.0:
            frames.append(report.read())
            times.append(time.perf_counter())
        report.close()
        gaps = sorted((b - a) * 1000 for a, b in zip(times, times[1:]))
        p = lambda q: gaps[min(len(gaps) - 1, int(round(q * (len(gaps) - 1))))]
        check(f"status at 10 Hz from the twin's process, gaps {p(0.05):.1f} to {p(0.95):.1f} ms (5th to 95th "
              "percentile; within 85 to 115)", 29 <= len(frames) <= 32 and p(0.05) > 85 and p(0.95) < 115, gaps)
        code, body = http("/state")
        state = json.loads(body)
        check("GET /state: joints, tool pose, the controller's state, obstacles and cameras", code == 200 and
              state["ok"] and len(state["joints"]) == 6 and {"table", "fixture", "part"} == set(state["obstacles"])
              and {"overhead", "wrist"} == set(state["cameras"]), state)
        arm = armcli.open_arm(HOST)
        arm.reset_err()
        arm.set_motion_mode(0)
        arm.set_motion_enable(8, 1)
        arm.set_motion_status(0)
        time.sleep(0.4)
        start = rad(json.loads(http("/state")[1])["joints"])
        goal = list(start)
        goal[0] += math.radians(20)
        goal[4] -= math.radians(10)
        arm.moveto_joint_p2p(goal, math.radians(20), math.radians(100), 0)
        off_path, moving = 0.0, 0
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            s = json.loads(http("/state")[1])
            q = rad(s["joints"])
            f = (q[0] - start[0]) / (goal[0] - start[0])
            off_path = max(off_path, max(abs(q[j] - (start[j] + f * (goal[j] - start[j]))) for j in range(6)))
            moving += s["controller"]["status"] == 1
            if s["controller"]["status"] == 2 and abs(q[0] - goal[0]) < 1e-6:
                break
            time.sleep(0.05)
        check(f"during a joint move the twin's joints stay on the controller's path ({moving} samples moving, off by "
              f"{math.degrees(off_path):.1e} deg)", moving >= 5 and off_path < 1e-6, (moving, off_path))
        report = armcli.StatusReport(HOST)
        frame = report.read()
        report.close()
        s = json.loads(http("/state")[1])
        check("settled, the twin's joints are the status frame's", max(abs(math.degrees(a) - b) for a, b in zip(
            frame["joints"], s["joints"])) < 1e-4, (frame["joints"], s["joints"]))
        code, body = http("/cameras/overhead.npz")
        with np.load(__import__("io").BytesIO(body)) as z:
            T = twin_model.look_at([600, 0, 900], [620, 0, -100], [1, 0, 0])
            check("GET /cameras/overhead.npz: a frame with its K and true pose", code == 200 and z["rgb"].shape ==
                  (480, 640, 3) and np.abs(z["T_base_camera"] - T).max() < 1e-9 and (z["depth"] > 0).mean() > 0.9)
        code, body = http("/cameras/wrist/rgb.png")
        check("GET /cameras/wrist/rgb.png: a PNG", code == 200 and body[:8] == b"\x89PNG\r\n\x1a\n")
        code, _ = http("/obstacles", {"add": {"name": "crate", "shape": "box", "size": [200, 200, 200],
                                              "pose": {"xyz": [400, -400, -630], "rpy": [0, 0, 0]}}})
        s = json.loads(http("/state")[1])
        check("POST /obstacles adds one; /state lists it where it was put, with its shape and size", code == 200 and
              abs(s["obstacles"]["crate"]["xyz"][1] + 400) < 1e-6 and s["obstacleShapes"]["crate"] == {
                  "shape": "box", "size": [200, 200, 200], "fixed": True, "mesh": None},
              (s["obstacles"].get("crate"), s.get("obstacleShapes", {}).get("crate")))
        code, _ = http("/obstacles", {"move": {"name": "crate", "pose": {"xyz": [400, 400, -630], "rpy": [0, 0, 0]}}})
        s = json.loads(http("/state")[1])
        check("moves it", code == 200 and abs(s["obstacles"]["crate"]["xyz"][1] - 400) < 1e-6)
        code, _ = http("/obstacles", {"remove": "crate"})
        check("and removes it", code == 200 and "crate" not in json.loads(http("/state")[1])["obstacles"])
        code, body = http("/obstacles", {"add": {"name": "bad", "shape": "cone", "size": [1]}})
        check("a bad obstacle is refused with the reason", code == 400 and b"shape" in body, body)
        code, _ = http("/state", headers={"Origin": "http://example.com"})
        check("requests from web pages (with an Origin) are refused", code == 403)
        req = urllib.request.Request(f"http://{HOST}:{HTTP_PORT}/obstacles", data=b'{"remove": "table"}',
                                     headers={"Content-Type": "text/plain"})
        try:
            urllib.request.urlopen(req, timeout=5)
            code = 200
        except urllib.error.HTTPError as e:
            code = e.code
        check("and so are POSTs that aren't JSON (a page's simple request)", code == 415)
    finally:
        if arm is not None:
            arm.socket_fp.close()
        proc.kill()
        proc.wait()


# ---- every HTTP server here: none asks DNS for its address's name when it binds -----------------------------------

BIND_PROBE = """
import json, socket, sys, time
from http.server import BaseHTTPRequestHandler
sys.path[:0] = {paths!r}
asked, real = [], (socket.getfqdn, socket.gethostbyaddr)


def getfqdn(name=""):
    asked.append(name)
    return name


def gethostbyaddr(address):
    asked.append(address)
    return address, [], [address]


ADDRESS = ({host!r}, 0)
{make}
socket.getfqdn, socket.gethostbyaddr = getfqdn, gethostbyaddr  # a lookup is recorded, not made
server, close = make()
bound = list(server.server_address[:2])
close()
socket.getfqdn, socket.gethostbyaddr = real
took = None
if {timed!r}:  # again, as it is: with no lookup, binding takes milliseconds even where DNS doesn't answer
    t0 = time.perf_counter()
    server, close = make()
    took = time.perf_counter() - t0
    close()
print(json.dumps({{"asked": asked, "bound": bound, "took": took}}))
"""
# (whose, which environment ("spine": the Python under test; "brain": this suite's), how it's made: make() returns the
# HTTPServer and what stops it)
HTTP_SERVERS = (
    ("the jog panel's (scripts/loopback_http.py's Server, which its main() makes)", "spine",
     "import loopback_http\ndef make():\n    s = loopback_http.Server(ADDRESS, BaseHTTPRequestHandler)\n"
     "    return s, s.server_close"),
    ("the stub brain's", "spine", "import stub_brain\ndef make():\n"
     "    s = stub_brain.Server(ADDRESS, BaseHTTPRequestHandler)\n    return s, s.server_close"),
    ("the twin's", "brain", "from brain.twin import server as m\ndef make():\n"
     "    s = m.TwinServer(None, *ADDRESS)\n    return s.httpd, s.close"),
)
STDLIB_SERVER = ("http.server's own ThreadingHTTPServer", "spine", "from http.server import ThreadingHTTPServer\n"
                 "def make():\n    s = ThreadingHTTPServer(ADDRESS, BaseHTTPRequestHandler)\n    return s, s.server_close")


def bind_probe(env, make, timed):
    """Makes a server in its own environment, on HOST, with socket's reverse lookups recorded instead of made: (what
    it asked for, where it bound, how long it took to make again with them as they are), or the failure."""
    exe = {"spine": PY, "brain": sys.executable}[env]
    paths = [ROOT, os.path.join(ROOT, "scripts"), os.path.join(ROOT, "brain", "stub")]
    r = subprocess.run([exe, "-c", BIND_PROBE.format(paths=paths, host=HOST, make=make, timed=timed)], cwd=ROOT,
                       capture_output=True, text=True, timeout=180)
    try:
        return json.loads(r.stdout.strip().splitlines()[-1]), None
    except (IndexError, ValueError):
        return None, (r.stdout + r.stderr)[-1500:]


def stdlib_http_users():
    """The modules of scripts/, tools/ and brain/ (not the environments kept in them) that name http.server's
    HTTPServer or ThreadingHTTPServer, other than scripts/loopback_http.py; and how many modules were read."""
    import ast
    skip = {"__pycache__", "node_modules", "vendor", "site-packages", "env", "venv", "ENV"}
    own = os.path.join(ROOT, "scripts", "loopback_http.py")
    named, read = [], 0
    for top in ("scripts", "tools", "brain"):
        for folder, dirs, files in os.walk(os.path.join(ROOT, top)):
            dirs[:] = [d for d in dirs if not d.startswith(".") and d not in skip]
            for f in sorted(files):
                path = os.path.join(folder, f)
                if not f.endswith(".py") or path == own:
                    continue
                with open(path, "rb") as fh:
                    tree = ast.parse(fh.read(), path)
                read += 1
                names = set()
                for node in ast.walk(tree):
                    if isinstance(node, ast.Name):
                        names.add(node.id)
                    elif isinstance(node, ast.Attribute):
                        names.add(node.attr)
                    elif isinstance(node, ast.ImportFrom):
                        names.update(a.name for a in node.names)
                if names & {"HTTPServer", "ThreadingHTTPServer"}:
                    named.append(os.path.relpath(path, ROOT))
    return named, read


def servers():
    print(f"== every HTTP server here binds without asking DNS for its address's name, as http.server's own does "
          f"(where DNS doesn't answer, in WSL2 with a VPN on, that takes 10 s): each made on {HOST} in its own "
          f"environment")
    for whose, env, make in HTTP_SERVERS:
        out, why = bind_probe(env, make, True)
        check(f"{whose} binds on {HOST} without asking for its name"
              + (f" (made again with DNS as it is: {out['took'] * 1000:.0f} ms)" if out else ""),
              out is not None and out["asked"] == [] and out["bound"][0] == HOST, why or out)
    whose, env, make = STDLIB_SERVER
    out, why = bind_probe(env, make, False)
    check(f"{whose}, for comparison, asks for {HOST}'s name: the probe sees a lookup where there is one",
          out is not None and HOST in out["asked"], why or out)
    named, read = stdlib_http_users()
    check(f"no other module serves HTTP with http.server's own server classes: of the {read} modules of scripts/, "
          f"tools/ and brain/, only scripts/loopback_http.py names them", read >= 30 and not named, (read, named))


# ---- off the loop: snapshots, and camera frames in a process of their own ----------------------------------------

def heartbeat_during(fn):
    """fn()'s result, and the longest another thread of this process waited to run while fn ran (ms). That thread
    sleeps 1 ms at a time, so anything holding Python's GIL shows."""
    gaps, done = [], []

    def beat():
        last = time.perf_counter()
        while not done:
            time.sleep(0.001)
            now = time.perf_counter()
            gaps.append(now - last)
            last = now

    th = threading.Thread(target=beat, daemon=True)
    th.start()
    time.sleep(0.05)
    try:
        out = fn()
    finally:
        done.append(True)
        th.join()
    return out, max(gaps) * 1000.0


def npz(body):
    with np.load(io.BytesIO(body)) as z:
        return {k: z[k] for k in z.files}


def off_the_loop():
    print("== camera frames off the loop: snapshots of the twin, rendered in a process of their own")
    import mujoco

    from brain.twin.cameras import CameraProcess, png_bytes
    twin = Twin(Scene.load(EXAMPLE))
    twin.set_joints(rad([20, -10, 70, 15, 60, 30]))
    twin.move_obstacle("fixture", {"xyz": [700, -250, -60], "rpy": [0, 0, 40]})  # a fixed one, moved
    snap = twin.snapshot()
    kept = snap.qpos.copy()
    obstacles, worst, R = snap.obstacles(), 0.0, np.zeros(9)
    for name, p in obstacles.items():
        T = twin.obstacle_pose(name)
        mujoco.mju_quat2Mat(R, np.array(p[3:]))
        worst = max(worst, np.abs(np.array(p[:3]) - T[:3, 3]).max(), np.abs(R.reshape(3, 3) - T[:3, :3]).max())
    check(f"a snapshot holds the arm's joints and where every obstacle is, fixed or free "
          f"({', '.join(sorted(obstacles))}), as the twin has them (worst {worst:.1e})",
          sorted(obstacles) == sorted(twin.built.obstacles) and (snap.joints() == twin.data.qpos[twin.qadr]).all()
          and worst < 1e-12, (obstacles, worst))
    want = {name: twin.render(name) for name in twin.built.cameras}  # in this process, at the snapshot's state
    twin.set_joints(rad([0, -30, 50, -10, 90, 0]))
    check("it's a copy: the twin moving on leaves it as it was", (snap.qpos == kept).all())
    cams = CameraProcess(60.0)
    try:
        body, waited = heartbeat_during(lambda: cams.frame(snap, "overhead", "npz"))
        got, w = npz(body), want["overhead"]
        meta = json.loads(str(got["meta"]))
        same = all((got[k] == v).all() for k, v in (
            ("rgb", w.rgb), ("depth", w.depth), ("segmentation", w.segmentation), ("K", w.K),
            ("T_base_camera", w.T_base_camera)))
        want_labels = {str(k): v for k, v in w.labels.items()}
        check("a frame from the camera process is the twin's own, pixel for pixel: colour, depth, segmentation, K, "
              "true pose, and the labels and joints it was taken at", same and meta["labels"] == want_labels
              and meta["joints_deg"] == w.joints, (same, meta["joints_deg"]))
        check(f"its first frame (a process started, the model compiled, OpenGL contexts made) left this process's "
              f"other threads alone: they waited {waited:.1f} ms at most", waited < 15.0, waited)
        other = Twin(Scene.load(EXAMPLE))
        _, held = heartbeat_during(lambda: other.render("overhead"))
        other.close()
        print(f"  (for comparison, the same first frame made in this process held its other threads up for "
              f"{held:.0f} ms: MuJoCo keeps Python's GIL while it loads a model onto the GPU)")
        if hasattr(os, "sched_getaffinity"):
            mine, theirs = sorted(os.sched_getaffinity(0)), sorted(os.sched_getaffinity(cams.pid()))
            check(f"on Linux it keeps to the upper half of the CPUs (CPUs {theirs[0]} to {theirs[-1]} of "
                  f"{len(mine)}), so the GPU driver's work doesn't delay the loop's wake-ups",
                  len(mine) < 4 and theirs == mine or theirs == mine[len(mine) // 2:], (mine, theirs))
        body = cams.frame(snap, "wrist", "rgb.png")
        check("and as the HTTP endpoint sends them: a PNG, byte for byte the twin's own frame's", body == png_bytes(
            want["wrist"].rgb))
        twin.add_obstacle(Obstacle("crate", "box", [100, 100, 180], {"xyz": [480, 0, -10], "rpy": [0, 0, 0]}))
        snap2 = twin.snapshot()
        labels = set(json.loads(str(npz(cams.frame(snap2, "overhead", "npz"))["meta"]))["labels"].values())
        check("after the twin rebuilds its model (an obstacle added), the process is given the new one: its frame "
              "shows the crate", snap2.layout.revision == snap.layout.revision + 1 and "obstacle: crate" in labels,
              labels)
        try:
            cams.frame(snap2, "nope", "npz")
            ok = False
        except SceneError:
            ok = True
        check("a camera that isn't there: SceneError (404 over HTTP)", ok)
        first = cams.pid()
        cams.proc.kill()
        cams.proc.join()
        got = npz(cams.frame(snap2, "wrist", "npz"))
        check("if the process dies, the next frame starts another", cams.started == 2 and cams.pid() not in (
            None, first) and got["rgb"].shape == (480, 640, 3), (cams.started, first, cams.pid()))
    finally:
        cams.close()
    check("closed, it's gone", cams.proc is None and cams.pid() is None)
    warm = CameraProcess(60.0)
    try:
        t0 = time.perf_counter()
        warm.warm(twin.layout)
        deadline = time.monotonic() + 60
        while warm.warmed != twin.layout.revision and time.monotonic() < deadline:
            time.sleep(0.02)
        ready = time.perf_counter() - t0
        t0 = time.perf_counter()
        warm.frame(twin.snapshot(), "overhead", "npz")
        took = time.perf_counter() - t0
        check(f"warmed up in the background as the twin starts (ready in {ready:.1f} s: the process started, the model "
              f"compiled, each camera rendered once), its first frame takes {took * 1000:.0f} ms",
              warm.warmed == twin.layout.revision and took < 1.0, (warm.warmed, took))
    finally:
        warm.close()
    twin.close()


# ---- live: the loop's cadence while frames are pulled -----------------------------------------------------------

def twin_process(log_name, *extra):
    """python -m brain.twin serve on HOST with the example scene, up and listening: (process, up)."""
    log = log_path(log_name)
    with open(log, "w") as out:
        proc = subprocess.Popen([sys.executable, "-m", "brain.twin", "serve", "--host", HOST, "--port", str(HTTP_PORT),
                                 "--scene", EXAMPLE, *extra], cwd=ROOT, stdout=out, stderr=subprocess.STDOUT)
    up = wait_port(502, timeout=30) and wait_port(HTTP_PORT, timeout=10)
    if not up:
        check(f"python -m brain.twin serve {' '.join(extra)} starts", False, open(log).read()[-2000:])
    return proc, up


def end(proc):
    if proc.poll() is None:
        proc.kill()
    proc.wait()


def loop_gaps(seconds):
    """The gaps (ms) between the twin's last `seconds` of loop ticks, from GET /loop."""
    return json.loads(http(f"/loop?seconds={seconds:.3f}&raw=1")[1])["gapsMs"]


def spread(gaps):
    """(99th percentile, ticks late (more than 15 ms after the one before), worst, how many) of loop gaps (ms)."""
    g = sorted(gaps)
    return g[max(0, int(math.ceil(0.99 * len(g))) - 1)], sum(x > 15.0 for x in g), g[-1], len(g)


class Puller:
    """Pulls camera frames over HTTP as a perception client would, `rate` frames a second (None: one after another),
    while `on` is set."""

    def __init__(self, path, rate=None):
        self.on, self.lock = threading.Event(), threading.Lock()
        self.frames, self.failed, self.busy, self.stop = 0, [], 0, False
        self.thread = threading.Thread(target=self.run, args=(path, rate), daemon=True)
        self.thread.start()

    def run(self, path, rate):
        while not self.stop:
            if not self.on.wait(0.05):
                continue
            t0 = time.perf_counter()
            with self.lock:
                self.busy += 1
            code, body = http(path)
            with self.lock:
                self.busy -= 1
                if code == 200:
                    self.frames += 1
                else:
                    self.failed.append((code, body[:200]))
            if rate:
                time.sleep(max(0.0, 1.0 / rate - (time.perf_counter() - t0)))

    def pause(self):
        self.on.clear()
        while self.busy:
            time.sleep(0.005)

    def close(self):
        self.pause()
        self.stop = True
        self.thread.join()


class StatusTimes:
    """The arrival times of the simulated controller's 10 Hz status frames (port 30001), read in the background."""

    def __init__(self):
        sys.path.insert(0, repo.SCRIPTS)
        import arm as armcli
        self.report, self.times, self.stop = armcli.StatusReport(HOST), [], False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop:
            self.report.read()
            self.times.append(time.perf_counter())

    def close(self):
        self.stop = True
        self.thread.join()
        self.report.close()
        return sorted((b - a) * 1000.0 for a, b in zip(self.times, self.times[1:]))


def cadence():
    print("== the loop keeps its 10 ms cadence, and the controller its 10 Hz status, while camera frames are pulled")
    proc, up = twin_process("twin_cadence.log")
    if not up:
        return
    try:
        time.sleep(0.5)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and json.loads(http("/loop?seconds=1")[1]).get("camerasReady") != 1:
            time.sleep(0.1)
        stats = json.loads(http("/loop?seconds=60&raw=1")[1])
        p99, late, worst, n = spread(stats["gapsMs"][10:])  # from the loop's tenth tick
        ready = stats.get("camerasReady")
        check(f"the camera process starts with the twin and warms up meanwhile (spawned, the model compiled, its "
              f"OpenGL contexts made, each camera rendered once): of the loop's {n} ticks till then, {late} late, the "
              f"worst {worst:.1f} ms after the one before (made in the twin's own process, MuJoCo would hold the loop "
              f"up 45 ms or more)", ready == 1 and worst < 30.0, (ready, worst, late))
        t0 = time.perf_counter()
        code, _ = http("/cameras/overhead.npz")
        took = time.perf_counter() - t0
        check(f"so the first frame asked for takes {took * 1000:.0f} ms", code == 200 and took < 1.0, (code, took))
        status = StatusTimes()
        for label, least, make in (
                ("a perception client's load, one camera's .npz (with segmentation) 5 times a second", 25,
                 lambda: [Puller("/cameras/overhead.npz", 5)]),
                ("frames one after another from two clients, .npz and PNG", 30,
                 lambda: [Puller("/cameras/overhead.npz"), Puller("/cameras/wrist/rgb.png")])):
            pullers, idle, busy = make(), [], []
            for _ in range(6):
                t0 = time.perf_counter()
                time.sleep(1.0)
                idle += loop_gaps(time.perf_counter() - t0)
                t0 = time.perf_counter()
                for pl in pullers:
                    pl.on.set()
                time.sleep(1.0)
                for pl in pullers:
                    pl.pause()
                busy += loop_gaps(time.perf_counter() - t0)
            frames, failed = sum(pl.frames for pl in pullers), [f for pl in pullers for f in pl.failed]
            for pl in pullers:
                pl.close()
            (ip99, ilate, iworst, ni), (bp99, blate, bworst, nb) = spread(idle), spread(busy)
            # Rendered on the loop, as frames were, each one would make a tick late. On a busy computer the camera
            # process's own work makes a few more late than the idle seconds do, but not one a frame.
            excess = blate - ilate * nb / max(ni, 1)
            check(f"{label}: {frames} frames in 6 s; the loop's ticks 99% within {bp99:.2f} ms of the one before, "
                  f"{blate} of {nb} late, the worst {bworst:.1f} ms (between them, idle: {ip99:.2f} ms, {ilate} of "
                  f"{ni}, {iworst:.1f} ms): {max(0.0, excess):.0f} late beyond the idle seconds' share",
                  not failed and frames >= least and bp99 <= max(15.0, ip99 + 2.0)
                  and excess <= max(0.01 * nb, 0.5 * frames), (failed, frames, bp99, blate, nb, ip99, ilate))
        http("/obstacles", {"add": {"name": "crate", "shape": "box", "size": [100, 100, 180],
                                    "pose": {"xyz": [480, 0, -10], "rpy": [0, 0, 0]}}})
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and json.loads(http("/loop?seconds=1")[1]).get("camerasReady") != 2:
            time.sleep(0.05)
        t0 = time.perf_counter()
        code, body = http("/cameras/overhead.npz")
        took = time.perf_counter() - t0
        labels = set(json.loads(str(npz(body)["meta"]))["labels"].values()) if code == 200 else set()
        check(f"an obstacle added (the model rebuilt): the camera process warms up on the new model meanwhile, so the "
              f"next frame takes {took * 1000:.0f} ms, and shows it", code == 200 and took < 1.0
              and "obstacle: crate" in labels, (code, took, labels))
        gaps = status.close()
        q = lambda f: gaps[min(len(gaps) - 1, int(round(f * (len(gaps) - 1))))]  # noqa: E731
        check(f"meanwhile the simulated controller's status came at 10 Hz: {len(gaps) + 1} frames, 5th to 95th "
              f"percentile {q(0.05):.1f} to {q(0.95):.1f} ms apart (within 85 to 115), all {gaps[0]:.1f} to "
              f"{gaps[-1]:.1f} (within 50 to 150: none stalled, none bunched)", len(gaps) > 200
              and q(0.05) > 85 and q(0.95) < 115 and gaps[0] >= 50 and gaps[-1] <= 150, gaps)
    finally:
        end(proc)


# ---- live: frames taken as the arm sweeps ------------------------------------------------------------------------

def frames_in_time():
    print("== frames pulled as the arm sweeps: each taken between its request and its reply, so the joints read around "
          "a request say where the arm was in its frame")
    proc, up = twin_process("twin_frames_in_time.log")
    if not up:
        return
    sys.path.insert(0, repo.SCRIPTS)
    import arm as armcli
    arm, done, frames = None, [], []
    sweep = [(35, -20, 70, -10, 65, 40), (-35, -20, 70, -10, 115, -40)]  # J1 +/-35 deg over the table, 45 deg/s

    def sweeping():
        for k in range(4):
            arm.moveto_joint_p2p(rad(sweep[k % 2]), math.radians(45), math.radians(150), 0)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and json.loads(http("/state")[1])["controller"]["status"] != 1:
                time.sleep(0.05)
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and json.loads(http("/state")[1])["controller"]["status"] != 2:
                time.sleep(0.1)
        done.append(True)

    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and json.loads(http("/loop?seconds=1")[1]).get("camerasReady") != 1:
            time.sleep(0.1)
        arm = armcli.open_arm(HOST)
        arm.reset_err()
        arm.set_motion_mode(0)
        arm.set_motion_enable(8, 1)
        arm.set_motion_status(0)
        time.sleep(0.4)
        th = threading.Thread(target=sweeping, daemon=True)
        th.start()
        while not done and th.is_alive():
            t0 = time.perf_counter()  # the twin's clock is the controller's, perf_counter: one clock for every process
            code, body = http("/cameras/overhead.npz")
            t1 = time.perf_counter()
            if code == 200:
                meta = json.loads(str(npz(body)["meta"]))
                frames.append((t0, t1, meta["time"], meta["joints_deg"]))
            time.sleep(max(0.0, 0.2 - (t1 - t0)))
        th.join()
        # The snapshot a frame shows is the loop's first after the request arrived; the loop takes its sample at the
        # start of a step, so up to one step (10 ms) before.
        early = max((t0 - t for t0, t1, t, q in frames), default=1.0)
        late = max((t - t1 for t0, t1, t, q in frames), default=1.0)
        took = sorted((t1 - t0) * 1000.0 for t0, t1, t, q in frames)
        j1 = [q[0] for *_, q in frames]
        check(f"{len(frames)} frames pulled 5 times a second as the arm swept J1 from {min(j1, default=0):.0f} to "
              f"{max(j1, default=0):.0f} deg: each taken between its request and its reply (at most "
              f"{max(early, 0) * 1000:.1f} ms before the request, by the loop's step; {max(-late, 0) * 1000:.0f} ms "
              f"before the reply at the least), the requests taking {took[len(took) // 2]:.0f} ms (median), "
              f"{took[int(0.95 * (len(took) - 1))]:.0f} ms (95th percentile)", len(frames) >= 25
              and max(j1) - min(j1) > 50 and early <= 0.01 and late <= 0, (len(frames), early, late, j1))
    finally:
        if arm is not None:
            arm.socket_fp.close()
        end(proc)


# ---- the viewer ------------------------------------------------------------------------------------------------

def viewer_follows():
    print("== the viewer follows the twin from its own process, over HTTP, read-only (here without a window)")
    import mujoco

    from brain.twin import viewer
    proc, up = twin_process("twin_viewer.log")
    if not up:
        return
    url, windows, stop, arm = f"http://{HOST}:{HTTP_PORT}", [], [], None

    class Window(viewer.Headless):
        def __init__(self, mirror):
            super().__init__(mirror)
            self.seen = []
            windows.append(self)

        def sync(self):
            super().sync()
            self.seen.append(self.mirror.data.qpos[self.mirror.qadr].tolist())

    follower = viewer.Follower(url)
    th = threading.Thread(target=viewer.follow, args=(follower, Window), kwargs=dict(log=lambda s: None,
                          until=lambda: bool(stop)), daemon=True)
    th.start()

    def body_xyz(m, name):
        return m.data.xpos[mujoco.mj_name2id(m.model, mujoco.mjtObj.mjOBJ_BODY, f"obstacle_{name}")] * 1000.0

    def wait_for(cond, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not cond() and time.monotonic() < deadline:
            time.sleep(0.02)
        return cond()

    try:
        wait_for(lambda: windows and windows[0].syncs, 15.0)
        w = windows[0] if windows else None
        pose = json.loads(http("/pose")[1])
        here = twin_model.fingerprint(build(Scene.load(EXAMPLE)))
        check(f"it shows the twin's own model: its generated MJCF and meshes (GET /model), compiled here, fingerprint "
              f"{pose['fingerprint'][:12]}, the same as a build of the same scene on this side",
              w is not None and w.mirror.fingerprint == pose["fingerprint"] == here, (pose["fingerprint"], here))
        sys.path.insert(0, repo.SCRIPTS)
        import arm as armcli
        arm = armcli.open_arm(HOST)
        arm.reset_err()
        arm.set_motion_mode(0)
        arm.set_motion_enable(8, 1)
        arm.set_motion_status(0)
        time.sleep(0.4)
        start = json.loads(http("/pose")[1])["joints"]
        goal = list(start)
        goal[0] += math.radians(25)
        goal[2] -= math.radians(15)
        n0 = len(w.seen)
        arm.moveto_joint_p2p(goal, math.radians(25), math.radians(100), 0)
        wait_for(lambda: json.loads(http("/state")[1])["controller"]["status"] == 1, 2.0)
        wait_for(lambda: json.loads(http("/state")[1])["controller"]["status"] == 2, 8.0)
        time.sleep(0.3)
        moving = [q for q in w.seen[n0:] if q != start and abs(q[0] - goal[0]) > 1e-6]
        f = (lambda q: (q[0] - start[0]) / (goal[0] - start[0]))  # noqa: E731
        off = max((abs(q[j] - (start[j] + f(q) * (goal[j] - start[j]))) for q in moving for j in range(6)), default=1.0)
        s = json.loads(http("/pose")[1])
        distinct = len({tuple(q) for q in moving})
        check(f"during a joint move it showed {distinct} states of the arm, about 100 a second, each on the "
              f"controller's path (off by {math.degrees(off):.1e} deg); settled, its arm is the twin's exactly",
              distinct > 80 and off < 1e-6 and w.mirror.data.qpos[w.mirror.qadr].tolist() == s["joints"],
              (distinct, off, s["joints"]))
        part = w.mirror.data.qpos[w.mirror.free["part"]:w.mirror.free["part"] + 7]
        check("and the free part where the twin's physics has it", np.abs(part - s["obstacles"]["part"]).max() < 1e-6,
              (part, s["obstacles"]["part"]))
        http("/obstacles", {"add": {"name": "crate", "shape": "box", "size": [100, 100, 180],
                                    "pose": {"xyz": [480, 0, -10], "rpy": [0, 0, 0]}}})
        wait_for(lambda: len(w.mirrors) == 2 and w.syncs)
        s = json.loads(http("/state")[1])
        check("an obstacle added over HTTP: it takes the twin's new model, the crate where the twin has it",
              len(w.mirrors) == 2 and w.mirror.revision == 2 and "crate" in w.mirror.mocap and
              np.abs(body_xyz(w.mirror, "crate") - s["obstacles"]["crate"]["xyz"]).max() < 1e-6,
              (len(w.mirrors), s["obstacles"].get("crate")))
        http("/obstacles", {"move": {"name": "crate", "pose": {"xyz": [480, 60, -10], "rpy": [0, 0, 30]}}})
        moved = wait_for(lambda: abs(body_xyz(w.mirror, "crate")[1] - 60.0) < 1e-6)
        s = json.loads(http("/state")[1])
        R = w.mirror.data.xmat[mujoco.mj_name2id(w.mirror.model, mujoco.mjtObj.mjOBJ_BODY, "obstacle_crate")]
        check("moved, it moves there too, turned as the twin has it, without a new model", moved and len(w.mirrors) == 2
              and abs(math.degrees(math.atan2(R[3], R[0])) - s["obstacles"]["crate"]["rpy"][2]) < 1e-6)
        http("/obstacles", {"remove": "crate"})
        check("removed, it's gone (the twin's next model)", wait_for(lambda: len(w.mirrors) == 3) and
              "crate" not in w.mirror.mocap and w.mirror.revision == 3)
        before = json.loads(http("/state")[1])
        time.sleep(2.0)
        after = json.loads(http("/state")[1])
        unchanged = before["joints"] == after["joints"] and before["controller"] == after["controller"] and all(
            np.abs(np.array(before["obstacles"][k]["xyz"]) - after["obstacles"][k]["xyz"]).max() < 0.05
            for k in before["obstacles"]) and sorted(before["obstacles"]) == sorted(after["obstacles"])
        check(f"read-only: it only ever asked the twin for its model and its state ("
              f"{', '.join(sorted({f'{v} {p}' for v, p in follower.requests}))}), and 2 s of following changed "
              f"nothing in the twin (the free part rests within 0.05 mm)", {p for _, p in follower.requests} == {
                  "/model", "/pose/stream"} and {v for v, _ in follower.requests} == {"GET"} and unchanged,
              (follower.requests, before, after))
        arm.socket_fp.close()
        arm = None
        end(proc)
        gone = wait_for(lambda: not follower.connected and w.text and "isn't answering" in w.text, 5.0)
        syncs = w.syncs
        proc, up = twin_process("twin_viewer_again.log")
        back = up and wait_for(lambda: follower.connected and w.syncs > syncs + 50 and w.text is None, 15.0)
        check("the twin stopped: the window stays, saying the twin isn't answering; the twin back, it follows it again",
              gone and back and w.running(), (gone, back, w.text))
    finally:
        stop.append(True)
        th.join(timeout=5)
        follower.close()
        if arm is not None:
            arm.socket_fp.close()
        end(proc)


def family(pid):
    """pid and its descendants (on Windows a venv's python.exe is a launcher that starts the real one as a child)."""
    import ctypes
    import ctypes.wintypes as wt

    class Entry(ctypes.Structure):
        _fields_ = [("dwSize", wt.DWORD), ("cntUsage", wt.DWORD), ("th32ProcessID", wt.DWORD),
                    ("th32DefaultHeapID", ctypes.c_void_p), ("th32ModuleID", wt.DWORD), ("cntThreads", wt.DWORD),
                    ("th32ParentProcessID", wt.DWORD), ("pcPriClassBase", wt.LONG), ("dwFlags", wt.DWORD),
                    ("szExeFile", ctypes.c_wchar * 260)]

    k32 = ctypes.windll.kernel32
    k32.CreateToolhelp32Snapshot.restype = wt.HANDLE
    snap, e, parents = k32.CreateToolhelp32Snapshot(2, 0), Entry(), {}
    e.dwSize = ctypes.sizeof(Entry)
    ok = k32.Process32FirstW(snap, ctypes.byref(e))
    while ok:
        parents[e.th32ProcessID] = e.th32ParentProcessID
        ok = k32.Process32NextW(snap, ctypes.byref(e))
    k32.CloseHandle(snap)
    out = {pid}
    while True:
        more = {c for c, p in parents.items() if p in out} - out
        if not more:
            return out
        out |= more


def viewer_windows(pid):
    """[(window, its process)]: the visible MuJoCo windows of pid's processes."""
    import ctypes
    import ctypes.wintypes as wt
    user32, pids, found = ctypes.windll.user32, family(pid), []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wt.HWND, wt.LPARAM)
    def each(hwnd, _):
        owner = wt.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(owner))
        if owner.value in pids and user32.IsWindowVisible(hwnd):
            title = ctypes.create_unicode_buffer(256)
            user32.GetWindowTextW(hwnd, title, 256)
            if title.value.startswith("MuJoCo"):
                found.append((hwnd, owner.value))
        return True

    user32.EnumWindows(each, 0)
    return found


def exit_code(pid, timeout):
    """pid's exit code once it has exited (None if it's still running after timeout s)."""
    import ctypes
    import ctypes.wintypes as wt
    k32 = ctypes.windll.kernel32
    k32.OpenProcess.restype = wt.HANDLE
    handle = k32.OpenProcess(0x00100000 | 0x1000, False, pid)  # SYNCHRONIZE, PROCESS_QUERY_LIMITED_INFORMATION
    if not handle:
        return None
    try:
        if k32.WaitForSingleObject(handle, int(timeout * 1000)) != 0:
            return None
        code = wt.DWORD()
        k32.GetExitCodeProcess(handle, ctypes.byref(code))
        return code.value
    finally:
        k32.CloseHandle(handle)


def carries_on(proc, what):
    """Checks that the twin served by proc carries on: its process runs, the controller's status at 10 Hz, its loop
    and HTTP, and it takes a move that the twin then follows."""
    sys.path.insert(0, repo.SCRIPTS)
    import arm as armcli
    seq = json.loads(http("/pose")[1])["seq"]
    report = armcli.StatusReport(HOST)
    times = []
    for _ in range(12):
        report.read()
        times.append(time.perf_counter())
    report.close()
    gaps = sorted((b - a) * 1000.0 for a, b in zip(times[1:], times[2:]))  # the first comes when it connects
    stats = json.loads(http("/loop?seconds=1")[1])
    arm = armcli.open_arm(HOST)
    try:
        arm.reset_err()
        arm.set_motion_mode(0)
        arm.set_motion_enable(8, 1)
        arm.set_motion_status(0)
        time.sleep(0.4)
        q0 = json.loads(http("/pose")[1])["joints"]
        goal = list(q0)
        goal[0] += math.radians(5)
        arm.moveto_joint_p2p(goal, math.radians(20), math.radians(100), 0)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and abs(json.loads(http("/pose")[1])["joints"][0] - goal[0]) > 1e-5:
            time.sleep(0.05)
        q1 = json.loads(http("/pose")[1])
    finally:
        arm.socket_fp.close()
    check(f"{what}, the twin carries on: its process runs, the controller's status comes every {gaps[0]:.0f} to "
          f"{gaps[-1]:.0f} ms, its loop {stats['perSecond']:.0f} times a second, and it takes a joint move that the "
          f"twin follows", proc.poll() is None and q1["seq"] > seq and 85 < gaps[0] and gaps[-1] < 115 and
          abs(stats["perSecond"] - 100) < 2 and abs(q1["joints"][0] - goal[0]) < 1e-5, (gaps, stats, q1["joints"]))


def viewer_process():
    print("== the viewer's process ends, the twin doesn't")
    proc, up = twin_process("twin_viewer_process.log")
    if not up:
        return
    url = f"http://{HOST}:{HTTP_PORT}"
    try:
        fingerprint = json.loads(http("/pose")[1])["fingerprint"]
        out = subprocess.run([sys.executable, "-m", "brain.twin", "view", "--follow", url, "--check", "2"], cwd=ROOT,
                             capture_output=True, text=True, timeout=120)
        said = (out.stdout.strip().splitlines() or [""])[-1]
        check(f"python -m brain.twin view --check follows it from a process of its own: \"{said}\"", out.returncode == 0
              and "states shown" in said and fingerprint[:12] in said and "GET /model, GET /pose/stream" in said,
              out.stdout + out.stderr)
        view = subprocess.Popen([sys.executable, "-m", "brain.twin", "view", "--follow", url, "--check", "60"],
                                cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4.0)
        view.kill()
        view.wait()
        carries_on(proc, "a viewer's process killed while it followed")
        if os.environ.get("TWIN_TEST_WINDOW") != "1" or os.name != "nt":
            print("  (TWIN_TEST_WINDOW=1 on Windows also opens the real window, with serve --viewer, and closes it)")
            return
        end(proc)
        proc, up = twin_process("twin_viewer_window.log", "--viewer")
        if not up:
            return
        found, deadline = [], time.monotonic() + 30
        while not found and time.monotonic() < deadline:
            found = viewer_windows(proc.pid)
            time.sleep(0.2)
        check("serve --viewer opens the viewer's window", len(found) == 1, found)
        if not found:
            return
        time.sleep(3.0)
        import ctypes
        ctypes.windll.user32.PostMessageW(found[0][0], 0x0010, 0, 0)  # WM_CLOSE, as the window's close button sends
        code = exit_code(found[0][1], 15.0)
        check(f"closing its window ends the viewer's process (exit code {code})", code == 0, code)
        carries_on(proc, "the viewer's window closed")
    finally:
        end(proc)


# ---- the replays ----------------------------------------------------------------------------------------------------

def replays():
    print("== the replays of both captures, every figure against its stated tolerance (brain/twin/replay.py)")
    twin = Twin()
    for r in replay.verify(twin):
        check(f"{r['check']}: {r['value']:.4g} {r['unit']} (limit {r['limit']:g}; {r['why']})", r["ok"])
    twin.close()


SECTIONS = (model_from_sources, cell_parts, obstacles, cameras, coupling_virtual, live, servers, off_the_loop, cadence,
            frames_in_time, viewer_follows, viewer_process, replays)


def main():
    named = sys.argv[1:]
    unknown = set(named) - {s.__name__ for s in SECTIONS}
    if unknown:
        sys.exit(f"unknown section(s): {', '.join(sorted(unknown))}; the sections: "
                 f"{', '.join(s.__name__ for s in SECTIONS)}")
    for section in (s for s in SECTIONS if not named or s.__name__ in named):
        try:
            section()
        except Exception as e:  # report and carry on
            import traceback
            traceback.print_exc()
            check(f"{section.__name__} crashed", False, repr(e))
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    sys.stdout.flush()
    os._exit(1 if failed else 0)  # the SDK's receive threads would keep the process alive


if __name__ == "__main__":
    main()
