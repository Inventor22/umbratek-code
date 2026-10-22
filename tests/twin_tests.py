"""The twin (brain/twin), run in the brain's environment (brain/.venv; see brain/README.md):

- its model is the repo's own: the arm's kinematics, collision capsules, cart, floor and joint limits are the ones
  scripts/utra_model.py and tools/utra_sim.py use, and a cell's tool, frames, zones and targets come through
  scripts/cell.py;
- obstacles can be placed, moved and removed, fixed or free, and the physics and contacts behave;
- its cameras: intrinsics (any fx, fy, cx, cy), depth in metres, and true poses, checked by projecting known points
  and back-projecting depth onto known surfaces;
- its coupling to tools/utra_sim.py: on a virtual clock, fed the SDK's own bytes, and live, served from the twin's
  process and driven through the SDK, with frames and state over HTTP;
- the replays of both captures, every figure against its stated tolerance (brain/twin/replay.py's verify()).

Usage: brain/.venv/Scripts/python tests/twin_tests.py   (from tests/run_all.py: python tests/run_all.py twin)
"""
import json
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from simrig import HOST, log_path, wait_port  # noqa: E402

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


# ---- the replays ----------------------------------------------------------------------------------------------------

def replays():
    print("== the replays of both captures, every figure against its stated tolerance (brain/twin/replay.py)")
    twin = Twin()
    for r in replay.verify(twin):
        check(f"{r['check']}: {r['value']:.4g} {r['unit']} (limit {r['limit']:g}; {r['why']})", r["ok"])
    twin.close()


def main():
    for section in (model_from_sources, cell_parts, obstacles, cameras, coupling_virtual, live, replays):
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
