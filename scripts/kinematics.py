"""Kinematics for the server: poses, frames, forward kinematics with a tool, and inverse kinematics.

Standard library only. Poses are 4x4 transforms as lists of rows, in metres, or {"xyz": [mm], "rpy": [deg]} dicts in
the controller's terms: roll, pitch and yaw with R = Rz(yaw) Ry(pitch) Rx(roll), as it reports the TCP and as the
panel shows it. Joint angles are degrees.

The inverse kinematics is web/ik.js's (damped least squares from a nearby seed, so the arm keeps its elbow and wrist
configuration), ported so the server can plan straight lines and solve targets. Forward kinematics is utra_model's,
which matches the controller's reported pose in every frame of the capture of 2026-10-21; tests/oracle_tests.py
checks both against Pinocchio.
"""
from __future__ import annotations

import math

RAD = math.pi / 180.0
RHO = 0.3  # metres per radian: how much a rotation error weighs against a position error (as web/ik.js)
TOL_POS, TOL_ROT = 1e-5, 1e-4  # stop iterating within 0.01 mm and 0.006 deg
OK_POS, OK_ROT = 5e-5, 2e-4  # a solution counts within 0.05 mm and 0.011 deg
MAX_STEP = 0.2  # radians any joint may turn in one iteration
IDENTITY = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


# ---- small linear algebra ----------------------------------------------------------------------------------

def mul(a, b):
    """Product of two 4x4 transforms."""
    return [[a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j] for j in range(4)]
            for i in range(4)]


def chain(*ts):
    out = IDENTITY
    for t in ts:
        out = mul(out, t)
    return out


def invert(t):
    """Inverse of a rigid transform."""
    r = [[t[j][i] for j in range(3)] for i in range(3)]
    p = [-(r[i][0] * t[0][3] + r[i][1] * t[1][3] + r[i][2] * t[2][3]) for i in range(3)]
    return [r[0] + [p[0]], r[1] + [p[1]], r[2] + [p[2]], [0.0, 0.0, 0.0, 1.0]]


def rotation_part(t):
    return [row[:3] for row in t[:3]]


def position(t):
    """Translation of a transform, in mm."""
    return [t[0][3] * 1000.0, t[1][3] * 1000.0, t[2][3] * 1000.0]


def transform(r, p_mm):
    return [list(r[0]) + [p_mm[0] / 1000.0], list(r[1]) + [p_mm[1] / 1000.0], list(r[2]) + [p_mm[2] / 1000.0],
            [0.0, 0.0, 0.0, 1.0]]


def mat3_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat3_t(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def apply(t, p_mm):
    """A point (mm) through a transform, in mm."""
    x, y, z = (v / 1000.0 for v in p_mm)
    return [(t[i][0] * x + t[i][1] * y + t[i][2] * z + t[i][3]) * 1000.0 for i in range(3)]


def axis_rotation(axis, angle):
    """Rotation by `angle` radians about the unit vector `axis`, as a 3x3."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return [[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v]]


def rot_exp(w):
    """Rotation for a rotation vector w (radians)."""
    angle = math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2])
    if angle < 1e-15:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return axis_rotation([v / angle for v in w], angle)


def rot_log(r):
    """Rotation vector (radians) of a rotation matrix."""
    c = max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0))
    angle = math.acos(c)
    v = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]]
    if angle < 1e-7:
        return [x / 2.0 for x in v]
    if math.pi - angle < 1e-4:  # half a turn: the axis is the column of (R + I) / 2 with the largest diagonal
        k = max(range(3), key=lambda i: r[i][i])
        axis = [(r[i][k] + (1.0 if i == k else 0.0)) / 2.0 for i in range(3)]
        n = math.sqrt(sum(a * a for a in axis)) or 1.0
        sign = 1.0 if sum(a * b for a, b in zip(axis, v)) >= 0 else -1.0
        return [sign * a / n * angle for a in axis]
    s = angle / (2.0 * math.sin(angle))
    return [x * s for x in v]


def angle_between(ra, rb):
    """Angle (deg) of the rotation that takes ra to rb."""
    w = rot_log(mat3_mul(rb, mat3_t(ra)))
    return math.sqrt(sum(x * x for x in w)) / RAD


# ---- the controller's roll, pitch and yaw --------------------------------------------------------------------

def rpy_matrix(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), angles in degrees."""
    cr, sr = math.cos(roll * RAD), math.sin(roll * RAD)
    cp, sp = math.cos(pitch * RAD), math.sin(pitch * RAD)
    cy, sy = math.cos(yaw * RAD), math.sin(yaw * RAD)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_rpy(r):
    """Roll, pitch, yaw (deg) of a rotation, as the controller reports them; at pitch +/-90 deg roll and yaw share
    an axis and the turn goes into yaw (as web/ik.js)."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    if abs(r[2][0]) > 1 - 1e-9:
        return [0.0, pitch / RAD, math.atan2(-r[0][1], r[1][1]) / RAD]
    return [math.atan2(r[2][1], r[2][2]) / RAD, pitch / RAD, math.atan2(r[1][0], r[0][0]) / RAD]


def pose_to_matrix(pose):
    """{"xyz": [mm], "rpy": [deg]} to a 4x4 transform (metres)."""
    return transform(rpy_matrix(*pose["rpy"]), pose["xyz"])


def matrix_to_pose(t, digits=None):
    xyz, rpy = position(t), matrix_rpy(rotation_part(t))
    if digits is not None:
        xyz, rpy = [round(v, digits) for v in xyz], [round(v, digits) for v in rpy]
    return {"xyz": xyz, "rpy": rpy}


def controller_pose(t):
    """[x, y, z (mm), roll, pitch, yaw (rad)], as the SDK's Cartesian moves take them."""
    return position(t) + [v * RAD for v in matrix_rpy(rotation_part(t))]


def from_controller_pose(p):
    return transform(rpy_matrix(*(v / RAD for v in p[3:6])), p[:3])


def offset_matrix(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    return transform(rpy_matrix(*rpy), xyz)


def interpolate(t0, t1, f):
    """The pose a fraction f of the way from t0 to t1: position along the straight line, orientation turning about
    one axis (the shortest way)."""
    r0 = rotation_part(t0)
    w = rot_log(mat3_mul(mat3_t(r0), rotation_part(t1)))
    r = mat3_mul(r0, rot_exp([x * f for x in w]))
    p0, p1 = position(t0), position(t1)
    return transform(r, [a + (b - a) * f for a, b in zip(p0, p1)])


def pose_distance(t0, t1):
    """(mm between the positions, deg between the orientations)."""
    return math.dist(position(t0), position(t1)), angle_between(rotation_part(t0), rotation_part(t1))


# ---- the arm -----------------------------------------------------------------------------------------------

class Kinematics:
    """Forward and inverse kinematics of utra_model.ArmModel's chain, with an optional tool transform."""

    def __init__(self, model):
        self.model = model
        self.axes = [axis for _, _, axis, _ in model.joints]
        self.n = len(self.axes)

    def flange(self, joints_deg):
        return self.model.frames(joints_deg)[-1]

    def tool_pose(self, joints_deg, tool=IDENTITY):
        return mul(self.flange(joints_deg), tool)

    def jacobian(self, frames, p):
        """Geometric Jacobian at point p (m): rows are linear velocity then angular velocity (rad/s) per joint's
        rad/s, as columns."""
        J = [[0.0] * self.n for _ in range(6)]
        for i in range(self.n):
            t, a = frames[i], self.axes[i]
            z = [t[r][0] * a[0] + t[r][1] * a[1] + t[r][2] * a[2] for r in range(3)]
            d = [p[0] - t[0][3], p[1] - t[1][3], p[2] - t[2][3]]
            v = [z[1] * d[2] - z[2] * d[1], z[2] * d[0] - z[0] * d[2], z[0] * d[1] - z[1] * d[0]]
            for r in range(3):
                J[r][i] = v[r]
                J[r + 3][i] = z[r]
        return J

    def _evaluate(self, q_rad, target, tool):
        frames = self.model.frames([v / RAD for v in q_rad])
        t = mul(frames[-1], tool)
        p = [t[0][3], t[1][3], t[2][3]]
        ep = [target[0][3] - p[0], target[1][3] - p[1], target[2][3] - p[2]]
        er = rot_log(mat3_mul(rotation_part(target), mat3_t(rotation_part(t))))
        e = ep + [x * RHO for x in er]
        return {"frames": frames, "p": p, "e": e, "norm": math.sqrt(sum(x * x for x in e)),
                "pos": math.sqrt(sum(x * x for x in ep)), "rot": math.sqrt(sum(x * x for x in er))}

    def solve(self, seed_deg, target, limits_deg, tool=IDENTITY, max_iterations=100):
        """Joint angles (deg) putting the tool at `target` (4x4), from `seed_deg`, within `limits_deg` ([low, high]
        per joint). Returns {"joints", "ok", "atLimit" (first joint pressed against a limit, or -1), "posErr" (m),
        "rotErr" (rad)}: web/ik.js's algorithm, step for step."""
        low = [lo * RAD for lo, _ in limits_deg]
        high = [hi * RAD for _, hi in limits_deg]
        q = [min(max(d * RAD, lo), hi) for d, lo, hi in zip(seed_deg, low, high)]
        state = self._evaluate(q, target, tool)
        lam = 1e-3
        k = 0
        while k < max_iterations and not (state["pos"] < TOL_POS and state["rot"] < TOL_ROT):
            k += 1
            J = self.jacobian(state["frames"], state["p"])
            for r in range(3, 6):
                J[r] = [x * RHO for x in J[r]]
            A = [[sum(J[r][i] * J[c][i] for i in range(self.n)) + (lam * lam if r == c else 0.0) for c in range(6)]
                 for r in range(6)]
            y = solve_linear(A, state["e"])
            if y is None:
                lam *= 10
                continue
            dq = [sum(J[r][i] * y[r] for r in range(6)) for i in range(self.n)]
            biggest = max(abs(v) for v in dq)
            if biggest > MAX_STEP:
                dq = [v * MAX_STEP / biggest for v in dq]
            nxt = [min(max(v + d, lo), hi) for v, d, lo, hi in zip(q, dq, low, high)]
            trial = self._evaluate(nxt, target, tool)
            if trial["norm"] < state["norm"]:
                q, state = nxt, trial
                lam = max(lam / 3, 1e-7)
            else:
                lam *= 8
                if lam > 100:
                    break
        ok = state["pos"] < OK_POS and state["rot"] < OK_ROT
        at_limit = -1
        if not ok:
            at_limit = next((i for i, v in enumerate(q) if v <= low[i] + 1e-6 or v >= high[i] - 1e-6), -1)
        return {"joints": [v / RAD for v in q], "ok": ok, "atLimit": at_limit, "posErr": state["pos"],
                "rotErr": state["rot"]}

    def joint_rates(self, joints_deg, twist, tool=IDENTITY):
        """Joint speeds (deg/s) that move the tool point at `twist` ([vx, vy, vz] mm/s, [wx, wy, wz] deg/s, base
        frame), or None at a singularity."""
        frames = self.model.frames(joints_deg)
        t = mul(frames[-1], tool)
        J = self.jacobian(frames, [t[0][3], t[1][3], t[2][3]])
        rhs = [v / 1000.0 for v in twist[:3]] + [w * RAD for w in twist[3:]]
        dq = solve_linear(J, rhs)
        return None if dq is None else [v / RAD for v in dq]


def solve_linear(A, b):
    """x with A x = b, by Gaussian elimination with partial pivoting; None when A is singular."""
    n = len(b)
    m = [list(row) + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-14:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            if f:
                for k in range(c, n + 1):
                    m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x
