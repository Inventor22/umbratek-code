"""Oracle checks: our kinematics, poses and motion profiles against independent implementations (Pinocchio, SciPy,
OpenCV, NumPy). Needs the oracle environment (tests/README.md). Usage: python tests/oracle_tests.py"""
import math
import os
import random
import sys

import numpy as np
import pinocchio as pin

from scipy.spatial.transform import Rotation, Slerp

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import kinematics as kin  # noqa: E402
import motion_profile  # noqa: E402
import utra_model  # noqa: E402

URDF = os.path.join(ROOT, "scripts", "model", "utra6_1000", "utra6_1000_robot.urdf")
LIMITS = [(-180.0, 180.0)] * 2 + [(-165.0, 165.0)] + [(-180.0, 180.0)] * 3
rng = random.Random(20261021)
results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)}")


def to_np(t):
    return np.array(t, dtype=float)


def random_joints():
    return [rng.uniform(lo, hi) for lo, hi in LIMITS]


def random_tool():
    xyz = [rng.uniform(-80, 80), rng.uniform(-80, 80), rng.uniform(0, 250)]
    rpy = [rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180)]
    return kin.offset_matrix(xyz, rpy), xyz, rpy


def pin_setup():
    model = pin.buildModelFromUrdf(URDF)
    return model, model.createData(), model.getFrameId("link_6")


def main():
    model = utra_model.ArmModel()
    k = kin.Kinematics(model)
    pmodel, pdata, flange_id = pin_setup()

    print("== forward kinematics: ours against Pinocchio's, from the same URDF")
    worst_p = worst_r = 0.0
    for _ in range(1000):
        q = random_joints()
        pin.framesForwardKinematics(pmodel, pdata, np.radians(q))
        M = pdata.oMf[flange_id]
        T = to_np(k.flange(q))
        worst_p = max(worst_p, np.linalg.norm(T[:3, 3] - M.translation))
        worst_r = max(worst_r, np.linalg.norm(Rotation.from_matrix(T[:3, :3].T @ M.rotation).as_rotvec()))
    check("1000 random joint sets: flange position within 1e-9 m", worst_p < 1e-9, worst_p)
    check("and orientation within 1e-9 rad", worst_r < 1e-9, worst_r)

    worst = 0.0
    for _ in range(20):
        tool, xyz, rpy = random_tool()
        placement = pin.SE3(Rotation.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]], degrees=True).as_matrix(),
                            np.array(xyz) / 1000.0)
        fid = pmodel.addFrame(pin.Frame(f"tool{len(results)}_{rng.random()}", pmodel.frames[flange_id].parentJoint,
                                        flange_id, pmodel.frames[flange_id].placement * placement, pin.FrameType.OP_FRAME))
        data = pmodel.createData()
        for _ in range(50):
            q = random_joints()
            pin.framesForwardKinematics(pmodel, data, np.radians(q))
            T = to_np(k.tool_pose(q, tool))
            M = data.oMf[fid]
            worst = max(worst, np.linalg.norm(T[:3, 3] - M.translation),
                        np.linalg.norm(Rotation.from_matrix(T[:3, :3].T @ M.rotation).as_rotvec()))
    check("20 random tools x 50 joint sets: the tool point's pose equals Pinocchio's frame at the same placement "
          "within 1e-9", worst < 1e-9, worst)

    print("== roll, pitch and yaw: the controller's convention against SciPy's ('ZYX' intrinsic = R = Rz Ry Rx)")
    worst_m = worst_a = 0.0
    for R in Rotation.random(1000, random_state=7):
        m = R.as_matrix()
        rpy = kin.matrix_rpy(m.tolist())
        worst_m = max(worst_m, np.abs(np.array(kin.rpy_matrix(*rpy)) - m).max())
        yaw, pitch, roll = R.as_euler("ZYX", degrees=True)
        worst_a = max(worst_a, max(abs((a - b + 180) % 360 - 180) for a, b in zip(rpy, (roll, pitch, yaw))))
    check("1000 random rotations: rpy_matrix(matrix_rpy(R)) == R within 1e-12", worst_m < 1e-12, worst_m)
    check("and the angles equal SciPy's within 1e-9 deg", worst_a < 1e-9, worst_a)
    m = Rotation.from_euler("ZYX", [30, 90, 0], degrees=True).as_matrix()
    check("at pitch 90 deg (gimbal lock) the rotation still round-trips",
          np.abs(np.array(kin.rpy_matrix(*kin.matrix_rpy(m.tolist()))) - m).max() < 1e-9)

    print("== rotation vectors and interpolation against SciPy")
    worst_log = worst_exp = 0.0
    for R in Rotation.random(500, random_state=8):
        worst_log = max(worst_log, np.linalg.norm(np.array(kin.rot_log(R.as_matrix().tolist())) - R.as_rotvec()))
        worst_exp = max(worst_exp, np.abs(np.array(kin.rot_exp(R.as_rotvec().tolist())) - R.as_matrix()).max())
    check("rot_log equals SciPy's as_rotvec within 1e-9 (500 rotations)", worst_log < 1e-9, worst_log)
    check("rot_exp equals SciPy's from_rotvec within 1e-12", worst_exp < 1e-12, worst_exp)
    worst = 0.0
    for _ in range(200):
        r0, r1 = Rotation.random(2, random_state=rng.randrange(10 ** 6))
        if np.linalg.norm((r0.inv() * r1).as_rotvec()) > math.pi - 1e-3:
            continue
        t0 = kin.transform(r0.as_matrix().tolist(), [rng.uniform(-500, 500) for _ in range(3)])
        t1 = kin.transform(r1.as_matrix().tolist(), [rng.uniform(-500, 500) for _ in range(3)])
        f = rng.random()
        mid = to_np(kin.interpolate(t0, t1, f))
        slerp = Slerp([0, 1], Rotation.from_matrix([r0.as_matrix(), r1.as_matrix()]))
        expect_p = (1 - f) * to_np(t0)[:3, 3] + f * to_np(t1)[:3, 3]
        worst = max(worst, np.abs(mid[:3, :3] - slerp([f]).as_matrix()[0]).max(), np.abs(mid[:3, 3] - expect_p).max())
    check("interpolate: straight position, orientation as SciPy's Slerp, within 1e-9", worst < 1e-9, worst)

    print("== inverse kinematics: solutions checked with Pinocchio's forward kinematics")
    pdata = pmodel.createData()  # the tool frames added above changed the model's size
    solved = tries = 0
    worst_p = worst_r = 0.0
    for _ in range(400):
        truth = [rng.uniform(max(lo, -170), min(hi, 170)) for lo, hi in LIMITS]
        if abs(truth[4]) < 8 or abs(abs(truth[4]) - 180) < 8 or abs(truth[2]) < 8:
            continue  # clear of the wrist singularity and the straight elbow, as tests/ik_tests.mjs
        tries += 1
        tool = random_tool()[0] if tries % 2 else kin.IDENTITY
        target = k.tool_pose(truth, tool)
        seed = [min(max(v + rng.uniform(-6, 6), lo), hi) for v, (lo, hi) in zip(truth, LIMITS)]
        r = k.solve(seed, target, LIMITS, tool)
        if not r["ok"]:
            continue
        solved += 1
        pin.framesForwardKinematics(pmodel, pdata, np.radians(r["joints"]))
        M = pdata.oMf[flange_id].homogeneous @ to_np(tool)
        worst_p = max(worst_p, np.linalg.norm(M[:3, 3] - to_np(target)[:3, 3]))
        worst_r = max(worst_r, np.linalg.norm(Rotation.from_matrix(M[:3, :3].T @ to_np(target)[:3, :3]).as_rotvec()))
    check(f"solves at least 99% of {tries} reachable poses (half with a tool) from a seed within 6 deg",
          solved >= 0.99 * tries, f"{solved}/{tries}")
    check("landing within 0.01 mm (Pinocchio)", worst_p < 1e-5, worst_p)
    check("and within 0.006 deg (its stopping tolerance, as web/ik.js)", math.degrees(worst_r) < 0.006, math.degrees(worst_r))

    print("== joint rates for a tool twist, against Pinocchio's Jacobian")
    worst = 0.0
    for _ in range(50):
        q = random_joints()
        if abs(q[4]) < 10 or abs(q[2]) < 10:
            continue
        twist = [rng.uniform(-100, 100) for _ in range(3)] + [rng.uniform(-30, 30) for _ in range(3)]
        dq = k.joint_rates(q, twist)
        pin.computeJointJacobians(pmodel, pdata, np.radians(q))
        pin.framesForwardKinematics(pmodel, pdata, np.radians(q))
        J = pin.getFrameJacobian(pmodel, pdata, flange_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        v = J @ np.radians(dq)
        expect = np.concatenate([np.array(twist[:3]) / 1000.0, np.radians(twist[3:])])
        worst = max(worst, np.abs(v - expect).max())
    check("Pinocchio's Jacobian times our joint rates gives back the twist, within 1e-9", worst < 1e-9, worst)

    print("== the double S profile against an exact integration of its jerk, phase by phase")
    worst_pos = worst_lim = 0.0
    for _ in range(300):
        d, v, a, j = rng.uniform(0.1, 300), rng.uniform(1, 200), rng.uniform(10, 500), rng.uniform(100, 5000)
        p = motion_profile.double_s(d, v, a, j)
        tj, ta, tv = p.tj, p.ta, p.tv
        # The jerk the phases imply: +j, 0, -j (accelerating), 0 (cruise), -j, 0, +j (decelerating).
        phases = [(tj, p.j), (ta - 2 * tj, 0.0), (tj, -p.j), (tv, 0.0), (tj, -p.j), (ta - 2 * tj, 0.0), (tj, p.j)]
        s = vel = acc = t0 = 0.0
        for h, jerk in phases:
            for u in np.linspace(0.0, h, 7):  # exact polynomial state within the phase, compared at 7 points
                expect = s + vel * u + acc * u * u / 2 + jerk * u ** 3 / 6
                worst_pos = max(worst_pos, abs(p.position(t0 + u) - expect) / d)
                v_now = vel + acc * u + jerk * u * u / 2
                a_now = acc + jerk * u
                worst_lim = max(worst_lim, v_now / v - 1, abs(a_now) / a - 1)
            s, vel, acc, t0 = s + vel * h + acc * h * h / 2 + jerk * h ** 3 / 6, vel + acc * h + jerk * h * h / 2, \
                acc + jerk * h, t0 + h
        worst_pos = max(worst_pos, abs(s - d) / d, abs(vel) / v, abs(acc) / a)  # ends at the distance, at rest
    check("300 random moves: positions equal the integrated jerk within 1e-9 of the distance, ending at rest",
          worst_pos < 1e-9, worst_pos)
    check("and speed and acceleration never exceed their limits", worst_lim < 1e-9, worst_lim)

    print("== tools/capture/tau_check.py: the torque a known load needs, against Pinocchio's generalized gravity")
    sys.path.insert(0, os.path.join(ROOT, "tools", "capture"))
    import argparse
    import json
    import tempfile
    import tau_check
    gmodel = pin.buildModelFromUrdf(URDF)
    for i in range(len(gmodel.inertias)):
        gmodel.inertias[i] = pin.Inertia.Zero()  # only the load's mass counts
    gmodel.gravity.linear = np.array([0.0, 0.0, -tau_check.G])
    flange_joint = gmodel.frames[flange_id].parentJoint
    to_joint = gmodel.frames[flange_id].placement
    worst = 0.0
    cases = []
    for _ in range(40):
        q = random_joints()
        mass = rng.uniform(0.2, 3.0)
        at = [rng.uniform(-60, 60), rng.uniform(-60, 60), rng.uniform(0, 200)]
        gmodel.inertias[flange_joint] = pin.Inertia(mass, to_joint.act(np.array(at) / 1000.0), np.zeros((3, 3)))
        gdata = gmodel.createData()
        expected = pin.computeGeneralizedGravity(gmodel, gdata, np.radians(q))
        ours = tau_check.holding_torques(q, at, [0.0, 0.0, -mass * tau_check.G], model)
        worst = max(worst, float(np.max(np.abs(np.array(ours) - expected))))
        cases.append((q, at, mass, expected))
    check("40 random poses, masses (0.2-3 kg) and points: the holding torques equal Pinocchio's within 1e-6 N m",
          worst < 1e-6, worst)

    # Synthetic recordings with a known scale (tau = 2.5 units per N m, the joint's sign) and noise: compare()
    # must recover it, from Pinocchio's torques rather than its own.
    q, at, mass, expected = max(cases, key=lambda c: sorted(np.abs(c[3]))[-2])  # two joints feel it well
    base = [rng.uniform(-30, 30) for _ in range(6)]
    folder = tempfile.mkdtemp(prefix="tau-")
    for name, add in (("before", [0.0] * 6), ("after", list(2.5 * expected))):
        with open(os.path.join(folder, name + ".json"), "w", encoding="utf-8") as f:
            json.dump({"joints": q, "tau": [b + a + rng.gauss(0, 0.01) for b, a in zip(base, add)], "tauSd": [0.05] * 6}, f)
    verdict = tau_check.compare(argparse.Namespace(
        before=os.path.join(folder, "before.json"), after=os.path.join(folder, "after.json"), mass=mass, force=None,
        at=",".join(str(v) for v in at)))
    check("a recording made with tau = 2.5 x the torque: compare finds 2.5 within 2%, agreeing across joints",
          abs(verdict.get("scale", 0) - 2.5) < 0.05 and verdict.get("spread", 1) < 0.05, verdict)

    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
