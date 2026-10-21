"""Oracle checks for the featureset's geometry: frame and tool calibration, re-mapping targets when a frame moves,
offsets, keep-out zones in the collision check, and straight-line planning. Independent references: NumPy, SciPy
(Rotation, align_vectors, least_squares) and Pinocchio. Needs the oracle environment. Usage:
python tests/feature_oracle_tests.py"""
import math
import os
import random
import shutil
import sys
import tempfile

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import cell as cellmod  # noqa: E402
import kinematics as kin  # noqa: E402
import planner as plannermod  # noqa: E402
import utra_model  # noqa: E402

rng = random.Random(1021)
nrng = np.random.default_rng(1021)
results = []
MODEL = utra_model.ArmModel()
URDF = os.path.join(ROOT, "scripts", "model", "utra6_1000", "utra6_1000_robot.urdf")


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)}")


def T(pose):
    return np.array(kin.pose_to_matrix(pose))


def rand_pose(span=400.0):
    yaw, pitch, roll = Rotation.random(random_state=rng.randrange(10 ** 6)).as_euler("ZYX", degrees=True)
    return {"xyz": [rng.uniform(-span, span) for _ in range(3)], "rpy": [roll, pitch, yaw]}


def rot_deg(a, b):
    return math.degrees(np.linalg.norm(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).as_rotvec()))


def frames_from_points():
    print("== three-point frames: exact without noise; with noise, errors as the noise predicts; SciPy's Kabsch agrees")
    sigma = 0.1
    exact = agree = 0.0
    origin_err, rot_err = [], []
    for _ in range(200):
        truth = T(rand_pose())
        local = np.array([[0, 0, 0], [rng.uniform(80, 300), 0, 0], [rng.uniform(-100, 200), rng.uniform(60, 250), 0]])
        clean = (truth[:3, :3] @ (local.T / 1000.0)).T * 1000.0 + truth[:3, 3] * 1000.0
        got = T(cellmod.frame_from_points(*clean.tolist()))
        exact = max(exact, np.abs(got - truth).max() * 1000.0)
        res = Rotation.align_vectors(clean - clean.mean(0), local - local.mean(0))
        kabsch_origin = clean.mean(0) - res[0].apply(local.mean(0))
        agree = max(agree, np.abs(res[0].as_matrix() - got[:3, :3]).max(), np.abs(kabsch_origin - got[:3, 3] * 1000.0).max())
        noisy = T(cellmod.frame_from_points(*(clean + nrng.normal(0, sigma, clean.shape)).tolist()))
        origin_err.append(np.linalg.norm(noisy[:3, 3] - truth[:3, 3]) * 1000.0)
        rot_err.append(rot_deg(noisy, truth))
    check("200 frames from exact points: equal to the truth within 1e-9 (mm and rotation entries)", exact < 1e-9, exact)
    check("and to SciPy's Kabsch fit of the same points within 1e-9", agree < 1e-9, agree)
    rms_origin = math.sqrt(np.mean(np.square(origin_err)))
    check(f"with {sigma} mm noise per coordinate: origin RMS error {rms_origin:.3f} mm, within 15% of the noise's "
          f"{sigma * math.sqrt(3):.3f}", abs(rms_origin / (sigma * math.sqrt(3)) - 1) < 0.15, rms_origin)
    check(f"and orientation RMS {math.sqrt(np.mean(np.square(rot_err))):.3f} deg, worst {max(rot_err):.2f} "
          "(under 0.15 and 0.5 deg with points 60 mm or more apart)",
          math.sqrt(np.mean(np.square(rot_err))) < 0.15 and max(rot_err) < 0.5, rot_err)
    for points in (([0, 0, 0], [5, 0, 0], [0, 100, 0]), ([0, 0, 0], [100, 0, 0], [200, 3, 0]),
                   ([0, 0, 0], [100, 0, 0], [50, 0, 0])):
        try:
            cellmod.frame_from_points(points[0], points[1], points[2])
            refused = False
        except cellmod.CellError:
            refused = True
        check(f"refuses degenerate points {points}", refused)


def tool_points():
    print("== four-point tool calibration: exact without noise; with noise, SciPy's least_squares agrees")
    exact = worst_agree = 0.0
    errors = []
    for trial in range(100):
        tool = np.array([rng.uniform(-60, 60), rng.uniform(-60, 60), rng.uniform(30, 250)])
        fixed = np.array([rng.uniform(200, 500), rng.uniform(-300, 300), rng.uniform(0, 400)])
        n = 4 + trial % 5
        rots = [Rotation.from_rotvec(Rotation.random(random_state=rng.randrange(10 ** 6)).as_rotvec() * 0.6) for _ in range(n)]
        clean = [kin.transform(r.as_matrix().tolist(), (fixed - r.apply(tool)).tolist()) for r in rots]
        got, point, rms = cellmod.tool_from_flange_poses(clean)
        exact = max(exact, np.abs(np.array(got) - tool).max(), np.abs(np.array(point) - fixed).max())
        poses = [kin.transform(r.as_matrix().tolist(), (fixed - r.apply(tool) + nrng.normal(0, 0.1, 3)).tolist())
                 for r in rots]
        got, point, rms = cellmod.tool_from_flange_poses(poses)
        errors.append(np.linalg.norm(np.array(got) - tool))

        def residual(x):
            return np.concatenate([np.array(t)[:3, :3] @ x[:3] + np.array(t)[:3, 3] * 1000.0 - x[3:] for t in poses])
        ref = least_squares(residual, np.zeros(6)).x
        worst_agree = max(worst_agree, np.linalg.norm(ref[:3] - np.array(got)))
    check("100 tools from 4 to 8 exact touches: the tool point and the fixed point within 1e-6 mm", exact < 1e-6, exact)
    rms = math.sqrt(np.mean(np.square(errors)))
    check(f"with 0.1 mm noise: RMS error {rms:.3f} mm, worst {max(errors):.2f} (under 0.2 and 0.5 mm)",
          rms < 0.2 and max(errors) < 0.5, errors)
    check("equal to SciPy's least-squares fit of the noisy touches within 1e-6 mm", worst_agree < 1e-6, worst_agree)
    same = [kin.transform(Rotation.from_euler("z", a, degrees=True).as_matrix().tolist(), [300 + a, 0, 0])
            for a in (0, 5, 10, 15)]
    try:
        cellmod.tool_from_flange_poses(same)
        refused = False
    except cellmod.CellError:
        refused = True
    check("refuses touches that barely change orientation", refused)


def remapping():
    print("== recalibrating a frame moves everything in it, by exactly the frame's change")
    folder = tempfile.mkdtemp(prefix="cell_")
    try:
        cell = cellmod.Cell(folder)
        cell.set_tool("gripper", {"xyz": [0, 0, 120], "rpy": [0, 0, 0]}, 0.5, [0, 0, 40])
        cell.activate_tool("gripper")
        cell.set_frame("printer", "base", {"xyz": [350, -150, 50], "rpy": [0, 0, 30]})
        cell.set_frame("plate", "printer", {"xyz": [20, 40, 10], "rpy": [0, 0, -5]})
        k = kin.Kinematics(MODEL)
        names = []
        for i, frame in enumerate(("printer", "plate", "base", "printer")):
            q = [rng.uniform(-40, 40), rng.uniform(-10, 40), rng.uniform(60, 120), rng.uniform(-60, 60),
                 rng.uniform(30, 90), rng.uniform(-90, 90)]
            pose = kin.mul(kin.invert(cell.frame_matrix(frame)), k.tool_pose(q, cell.tool_matrix()))
            name = f"t{i}_{frame}"
            cell.set_target(name, frame, kin.matrix_to_pose(pose), seed=q)
            names.append(name)
        cell.set_zone("printer_body", "printer", {"xyz": [0, 0, -100], "rpy": [0, 0, 0]}, [300, 300, 200])
        before = {n: np.array(cell.target_matrix(n)) for n in names}
        zone_before = cell.zone_obstacles()[0].centre
        old_frame = np.array(cell.frame_matrix("printer"))
        new_pose = {"xyz": [355, -148, 52], "rpy": [0.5, -0.3, 31.0]}
        report = cell.set_frame("printer", "base", new_pose, model=MODEL)
        change = np.array(cell.frame_matrix("printer")) @ np.linalg.inv(old_frame)
        worst = 0.0
        for n in names:
            after = np.array(cell.target_matrix(n))
            expect = before[n] if n.endswith("_base") else change @ before[n]
            worst = max(worst, np.abs(after - expect).max())
        check("targets in the frame, and in a frame inside it, move by the frame's change; others don't (1e-9)",
              worst < 1e-9, worst)
        zone_after = np.array(cell.zone_obstacles()[0].centre)
        check("the frame's zone moves with it", np.abs(zone_after - (change[:3, :3] @ zone_before + change[:3, 3])).max() < 1e-9)
        moved = {r["target"] for r in report}
        check("the report covers exactly the targets that moved", moved == {n for n in names if not n.endswith("_base")},
              report)
        ok = all(not r.get("unreachable") and not r.get("configurationChanged") for r in report)
        k_new = [cell.target(n)["seed"] for n in names if not n.endswith("_base")]
        poses_ok = all(np.abs(np.array(k.tool_pose(s, cell.tool_matrix())) - np.array(cell.target_matrix(n))).max() < 1e-4
                       for s, n in zip(k_new, [n for n in names if not n.endswith("_base")]))
        check("each moved target is re-solved from its seed, same configuration, and the new seed reaches it", ok and poses_ok,
              report)
        cell.set_frame("printer", "base", {"xyz": [2500, 0, 0], "rpy": [0, 0, 0]}, model=MODEL)
        lost = [n for n in names if cell.target(n).get("unreachable")]
        check("a frame moved out of reach flags its targets, rather than silently moving them",
              set(lost) == {n for n in names if not n.endswith("_base")}, lost)
        saved = open(cell.path, "rb").read()
        again = cellmod.Cell(folder)
        check("the cell reloads from disk exactly", again.data == cell.data and open(again.path, "rb").read() == saved)
        try:
            again.set_frame("printer", "base", new_pose, revision=again.revision - 1)
            refused = False
        except cellmod.CellError:
            refused = True
        check("an edit against an old revision is refused", refused)
        try:
            cell.delete_frame("printer")
            refused = False
        except cellmod.CellError:
            refused = True
        check("a frame still in use can't be deleted", refused)
    finally:
        shutil.rmtree(folder, ignore_errors=True)


def offsets():
    print("== offsets: RelTool along the tool's axes, Offs along the target frame's, against SciPy compositions")
    worst_tool = worst_frame = 0.0
    for _ in range(300):
        pose = T(rand_pose())
        frame = T(rand_pose())
        xyz = [rng.uniform(-100, 100) for _ in range(3)]
        rpy = [rng.uniform(-45, 45) for _ in range(3)]
        got = np.array(cellmod.apply_offset(pose.tolist(), frame.tolist(), {"frame": "tool", "xyz": xyz, "rpy": rpy}))
        r = Rotation.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]], degrees=True)
        expect = np.eye(4)
        expect[:3, :3] = pose[:3, :3] @ r.as_matrix()
        expect[:3, 3] = pose[:3, 3] + pose[:3, :3] @ (np.array(xyz) / 1000.0)
        worst_tool = max(worst_tool, np.abs(got - expect).max())
        got = np.array(cellmod.apply_offset(pose.tolist(), frame.tolist(), {"frame": "target", "xyz": xyz}))
        expect = pose.copy()
        expect[:3, 3] += frame[:3, :3] @ (np.array(xyz) / 1000.0)
        worst_frame = max(worst_frame, np.abs(got - expect).max())
    check("300 tool-frame offsets equal SciPy's composition within 1e-12", worst_tool < 1e-12, worst_tool)
    check("300 frame offsets translate along the frame's axes within 1e-12", worst_frame < 1e-12, worst_frame)


def box_distance(points, centre, axes, half):
    """Distance (m) from points to a box (NumPy, from scratch)."""
    local = (points - centre) @ np.array(axes).T
    outside = np.maximum(np.abs(local) - np.array(half), 0.0)
    return np.linalg.norm(outside, axis=1)


def zones_against_sampling():
    print("== keep-out zones in the collision check, against dense sampling with NumPy")
    misses = false_alarms = checked = 0
    for trial in range(40):
        size = [rng.uniform(50, 300) for _ in range(3)]
        centre_pose = {"xyz": [rng.uniform(250, 600), rng.uniform(-400, 400), rng.uniform(-200, 500)],
                       "rpy": [0, 0, rng.uniform(-90, 90)]}
        t = kin.pose_to_matrix(centre_pose)
        zone = utra_model.Obstacle("the test zone", "box", [t[0][3], t[1][3], t[2][3]], [[t[0][k], t[1][k], t[2][k]]
                                                                                        for k in range(3)],
                                   [v / 1000.0 for v in size])
        model = MODEL.with_cell([(6, [0, 0, 0], [0, 0, 0.15], 0.03, "the tool")], [zone])
        start = [rng.uniform(-60, 60), rng.uniform(-30, 30), rng.uniform(40, 120), rng.uniform(-90, 90),
                 rng.uniform(20, 120), 0]
        goal = [s + rng.uniform(-50, 50) for s in start]
        zone_pairs = [p for p in model.pairs if p[2] and p[1] == len(model.obstacles) - 1]
        if min(model._gap(model.placed(start), p)[0] for p in zone_pairs) < utra_model.MARGIN_M:
            continue  # the check lets parts already too close back away; keep to starts clear of the zone
        checked += 1
        hit = model.check_path(start, goal)
        zone_hit = hit is not None and hit["other"] == "the test zone"
        # Dense: every capsule's core sampled every 2 mm, every 0.5 mm of the fastest point's travel.
        worst = math.inf
        steps = 400
        for s in range(steps + 1):
            q = [a + (b - a) * s / steps for a, b in zip(start, goal)]
            for (c, o, _, _) in zone_pairs:
                a, b = (np.array(v) for v in model.placed(q)[c])
                n = max(2, int(np.linalg.norm(b - a) / 0.002) + 1)
                pts = a + np.outer(np.linspace(0, 1, n), b - a)
                d = box_distance(pts, np.array(zone.centre), zone.axes, zone.half).min() - model.capsules[c][3]
                worst = min(worst, d)
        sampled_hit = worst < utra_model.MARGIN_M - 0.002  # the continuous check's 2 mm step allowance
        if sampled_hit and not zone_hit and (hit is None):
            misses += 1
        if zone_hit and worst > utra_model.MARGIN_M + 0.0005:
            false_alarms += 1
    check(f"{checked} random moves near random zones: no intrusion that sampling finds is missed", misses == 0, misses)
    check("and none reported that sampling doesn't find", false_alarms == 0, false_alarms)


def lines_with_pinocchio():
    print("== straight lines: the planned joints keep the tool point on the line (Pinocchio's kinematics)")
    pmodel = pin.buildModelFromUrdf(URDF)
    pdata = pmodel.createData()
    fid = pmodel.getFrameId("link_6")
    plan = plannermod.Planner(MODEL)
    worst, planned, refused = 0.0, 0, 0
    for trial in range(30):
        tool_pose = {"xyz": [rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(0, 200)],
                     "rpy": [0, 0, rng.uniform(-90, 90)]}
        tool = kin.pose_to_matrix(tool_pose)
        start = [rng.uniform(-60, 60), rng.uniform(-10, 40), rng.uniform(60, 130), rng.uniform(-60, 60),
                 rng.uniform(30, 120), rng.uniform(-90, 90)]
        t0 = kin.mul(MODEL.frames(start)[-1], tool)
        d = [rng.uniform(-120, 120) for _ in range(3)]
        turn = [rng.uniform(-20, 20) for _ in range(3)] if trial % 2 else [0, 0, 0]
        target = kin.mul(kin.transform(kin.rotation_part(t0), [a + b for a, b in zip(kin.position(t0), d)]),
                         kin.offset_matrix((0, 0, 0), turn))
        try:
            p = plan.line(start, target, tool, 50.0, 30.0)
        except plannermod.PlanError:
            refused += 1
            continue
        planned += 1
        a, b = np.array(kin.position(t0)), np.array(kin.position(target))
        for q in p.path:
            pin.framesForwardKinematics(pmodel, pdata, np.radians(q))
            x = (pdata.oMf[fid].homogeneous @ np.array(tool))[:3, 3] * 1000.0
            u = (b - a) / max(np.linalg.norm(b - a), 1e-9)
            off = np.linalg.norm((x - a) - np.dot(x - a, u) * u)
            worst = max(worst, off)
    check(f"{planned} lines planned ({refused} refused): every sample within 0.05 mm of the line", worst < 0.05 and planned > 10,
          (worst, planned))


def main():
    for part in (frames_from_points, tool_points, remapping, offsets, zones_against_sampling, lines_with_pinocchio):
        try:
            part()
        except Exception as e:  # report and carry on
            import traceback
            traceback.print_exc()
            check(f"{part.__name__} crashed", False, repr(e))
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
