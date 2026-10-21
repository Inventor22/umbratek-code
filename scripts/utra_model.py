"""The UTRA arm's shape: forward kinematics from Umbratek's URDF, and collision checks along a move.

The jog panel uses this to refuse moves that would bring the arm too close to itself, the cart it stands on,
or the floor. The URDF and meshes are Umbratek's (see model/utra6_1000/SOURCE.txt). Each moving link is
covered by a few capsules fitted around its mesh, and the base by a cylinder (model/utra6_1000/collision.json),
so the checks err on the cautious side. The cart and the floor are boxes and cylinders from
model/cart/utra_cart.urdf, built from the cart's SolidWorks model (see model/cart/SOURCE.txt). Moves are
checked continuously along their path, not at a few samples. jog_panel's 3D view (web/collide.js) runs the
same checks from collision_config(), so its preview matches the server's verdict.
"""
from __future__ import annotations

import json
import math
import os
import xml.etree.ElementTree as ET

MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model")
URDF_PATH = os.path.join(MODEL_DIR, "utra6_1000", "utra6_1000_robot.urdf")
SHAPES_PATH = os.path.join(MODEL_DIR, "utra6_1000", "collision.json")
CART_PATH = os.path.join(MODEL_DIR, "cart", "utra_cart.urdf")
# What messages call the base and the link each joint turns.
BODY_LABELS = ("the arm's base", "the shoulder", "the upper arm", "the forearm", "wrist 1", "wrist 2", "the flange")
MARGIN_M = 0.010  # how close the arm may come to itself, the cart or the floor
SLACK_M = 0.001  # parts already closer than the margin may move apart, but no more than this closer
ADVANCE_M = 0.002  # a step along a path is never shorter than this much movement of the fastest point
EPS = 1e-12


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _translation(x, y, z):
    return [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]]


def _rotation(axis, angle):
    """Rotation by `angle` radians about the unit vector `axis`."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return [[c + x * x * v, x * y * v - z * s, x * z * v + y * s, 0.0],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s, 0.0],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def _origin(element):
    """The transform of a URDF <origin> element (identity when it's missing)."""
    if element is None:
        return _translation(0.0, 0.0, 0.0)
    x, y, z = _floats(element.get("xyz"))
    roll, pitch, yaw = _floats(element.get("rpy"))
    rpy = _matmul(_rotation((0, 0, 1), yaw), _matmul(_rotation((0, 1, 0), pitch), _rotation((1, 0, 0), roll)))
    return _matmul(_translation(x, y, z), rpy)


def _floats(text, default="0 0 0", count=3):
    values = [float(v) for v in (text or default).split()]
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError(f"Expected {count} numbers, got {text!r}.")
    return values


def _apply(t, point):
    x, y, z = point
    return [t[i][0] * x + t[i][1] * y + t[i][2] * z + t[i][3] for i in range(3)]


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _lerp(a, b, t):
    return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]


def _clamp(x, low, high):
    return low if x < low else high if x > high else x


def _distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def segment_segment(p1, q1, p2, q2):
    """Closest points between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection, 5.1.9)."""
    d1, d2, r = _sub(q1, p1), _sub(q2, p2), _sub(p1, p2)
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    if a <= EPS and e <= EPS:
        s = t = 0.0
    elif a <= EPS:
        s, t = 0.0, _clamp(f / e, 0.0, 1.0)
    else:
        c = _dot(d1, r)
        if e <= EPS:
            s, t = _clamp(-c / a, 0.0, 1.0), 0.0
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = _clamp((b * f - c * e) / denom, 0.0, 1.0) if denom > EPS else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = _clamp(-c / a, 0.0, 1.0), 0.0
            elif t > 1.0:
                s, t = _clamp((b - c) / a, 0.0, 1.0), 1.0
    return _lerp(p1, q1, s), _lerp(p2, q2, t)


class Obstacle:
    """Something fixed around the arm, in the base frame: a box or a cylinder, with the label messages use.

    `body` is 0 for the arm's own base (which its neighbouring links never check against) and None otherwise.
    """

    def __init__(self, label, kind, centre, axes, size, body=None):
        self.label, self.kind, self.centre, self.body = label, kind, list(centre), body
        if kind == "box":
            self.axes = [list(a) for a in axes]  # the box's own x, y, z axes
            self.half = [s / 2 for s in size]
        elif kind == "cylinder":
            self.axis = list(axes[2])  # URDF cylinders run along their z axis
            self.radius, self.half_length = size[0], size[1] / 2
        else:
            raise ValueError(f"Unsupported obstacle shape: {kind!r}.")

    def nearest(self, x):
        """The point of the obstacle nearest to x (x itself when inside)."""
        v = _sub(x, self.centre)
        if self.kind == "box":
            out = list(self.centre)
            for axis, half in zip(self.axes, self.half):
                d = _clamp(_dot(v, axis), -half, half)
                out = [out[i] + axis[i] * d for i in range(3)]
            return out
        h = _dot(v, self.axis)
        radial = [v[i] - self.axis[i] * h for i in range(3)]
        rho = math.sqrt(_dot(radial, radial))
        scale = min(1.0, self.radius / rho) if rho > 0 else 0.0
        h = _clamp(h, -self.half_length, self.half_length)
        return [self.centre[i] + self.axis[i] * h + radial[i] * scale for i in range(3)]

    def nearest_to_segment(self, p, q):
        """Closest points (on the segment p-q, on the obstacle)."""
        if self.kind == "box":
            return self._box_segment(p, q)
        # The distance from a convex shape is convex along a segment: a golden-section search finds its minimum.
        def gap(t):
            x = _lerp(p, q, t)
            return _distance(x, self.nearest(x))
        low, high = 0.0, 1.0
        ratio = (math.sqrt(5.0) - 1.0) / 2.0
        t1, t2 = high - ratio * (high - low), low + ratio * (high - low)
        g1, g2 = gap(t1), gap(t2)
        for _ in range(40):
            if g1 <= g2:
                high, t2, g2 = t2, t1, g1
                t1 = high - ratio * (high - low)
                g1 = gap(t1)
            else:
                low, t1, g1 = t1, t2, g2
                t2 = low + ratio * (high - low)
                g2 = gap(t2)
        best = min((0.0, 1.0, 0.5 * (low + high)), key=gap)
        x = _lerp(p, q, best)
        return x, self.nearest(x)

    def _box_segment(self, p, q):
        a = [_dot(_sub(p, self.centre), axis) for axis in self.axes]  # the segment in the box's frame
        d = [_dot(_sub(q, p), axis) for axis in self.axes]
        cuts = [0.0, 1.0]
        for k in range(3):
            if d[k] != 0.0:
                for side in (-self.half[k], self.half[k]):
                    t = (side - a[k]) / d[k]
                    if 0.0 < t < 1.0:
                        cuts.append(t)
        cuts.sort()
        best = None
        for t0, t1 in zip(cuts, cuts[1:]):
            # Between cuts each coordinate stays below, inside or above the box, so the squared distance is
            # a quadratic in t: take its minimum within the piece.
            middle = 0.5 * (t0 + t1)
            num = den = 0.0
            for k in range(3):
                x = a[k] + d[k] * middle
                if x > self.half[k]:
                    side = self.half[k]
                elif x < -self.half[k]:
                    side = -self.half[k]
                else:
                    continue
                num -= d[k] * (a[k] - side)
                den += d[k] * d[k]
            t = t0 if den <= 0.0 else _clamp(num / den, t0, t1)
            local = [a[k] + d[k] * t for k in range(3)]
            near = [_clamp(local[k], -self.half[k], self.half[k]) for k in range(3)]
            gap = sum((local[k] - near[k]) ** 2 for k in range(3))
            if best is None or gap < best[0]:
                best = (gap, t, near)
        _, t, near = best
        point = [self.centre[i] + sum(near[k] * self.axes[k][i] for k in range(3)) for i in range(3)]
        return _lerp(p, q, t), point

    def config(self):
        out = {"label": self.label, "kind": self.kind, "centre": self.centre, "body": self.body}
        if self.kind == "box":
            out.update(axes=self.axes, half=self.half)
        else:
            out.update(axis=self.axis, radius=self.radius, halfLength=self.half_length)
        return out


def load_obstacles(urdf_path):
    """The boxes and cylinders of a one-link URDF's collision elements, labelled by their names."""
    links = ET.parse(urdf_path).getroot().findall("link")
    if len(links) != 1:
        raise ValueError(f"{os.path.basename(urdf_path)} should describe one link, fixed to the arm's base.")
    out = []
    for element in links[0].findall("collision"):
        t = _origin(element.find("origin"))
        centre = [t[0][3], t[1][3], t[2][3]]
        axes = [[t[0][k], t[1][k], t[2][k]] for k in range(3)]
        label = element.get("name") or "the cart"
        box, cylinder = element.find("geometry/box"), element.find("geometry/cylinder")
        if box is not None:
            size = _floats(box.get("size"))
            out.append(Obstacle(label, "box", centre, axes, size))
        elif cylinder is not None:
            size = [float(cylinder.get("radius")), float(cylinder.get("length"))]
            out.append(Obstacle(label, "cylinder", centre, axes, size))
        else:
            raise ValueError(f"{label} in {os.path.basename(urdf_path)}: only boxes and cylinders are supported.")
        if min(size) <= 0 or not all(math.isfinite(v) for v in size):
            raise ValueError(f"{label} in {os.path.basename(urdf_path)} has a bad size.")
    return out


def load_visuals(urdf_path):
    """The mesh files (relative to the URDF) and colours of a URDF's visual elements, all at its origin."""
    out = []
    for element in ET.parse(urdf_path).getroot().iter("visual"):
        mesh, color = element.find("geometry/mesh"), element.find("material/color")
        if mesh is None or element.find("origin") is not None and _origin(element.find("origin")) != _origin(None):
            raise ValueError(f"{os.path.basename(urdf_path)}: visuals must be meshes at the link's origin.")
        name = mesh.get("filename", "").replace("\\", "/")
        if not name.startswith("meshes/") or ".." in name.split("/"):
            raise ValueError(f"{os.path.basename(urdf_path)}: unexpected mesh path {name!r}.")
        rgba = _floats(color.get("rgba"), count=4) if color is not None else [0.6, 0.6, 0.6, 1.0]
        out.append({"file": name, "color": rgba})
    return out


class ArmModel:
    """A serial chain of revolute joints, with capsules around each link and obstacles around the arm."""

    def __init__(self, urdf_path=URDF_PATH, shapes_path=SHAPES_PATH, cart_path=CART_PATH):
        root = ET.parse(urdf_path).getroot()
        self.joints = []  # (name, fixed transform from the parent link, unit axis, child link)
        base = parent = None
        for joint in root.iter("joint"):
            if joint.get("type") not in ("revolute", "continuous"):
                continue
            if parent is not None and joint.find("parent").get("link") != parent:
                raise ValueError("The URDF's moving joints don't form a single chain.")
            base = base or joint.find("parent").get("link")
            axis = _floats(joint.find("axis").get("xyz") if joint.find("axis") is not None else None, "1 0 0")
            norm = math.sqrt(sum(a * a for a in axis))
            child = joint.find("child").get("link")
            self.joints.append((joint.get("name"), _origin(joint.find("origin")), [a / norm for a in axis], child))
            parent = child
        if len(self.joints) + 1 != len(BODY_LABELS):
            raise ValueError(f"Expected a {len(BODY_LABELS) - 1}-joint arm, found {len(self.joints)} joints.")
        bodies = {base: 0, **{child: k + 1 for k, (_, _, _, child) in enumerate(self.joints)}}

        with open(shapes_path, encoding="utf-8") as f:
            shapes = json.load(f)
        self.capsules = []  # (body, end a, end b, radius), the ends in the body's link frame
        for link, capsules in shapes["capsules"].items():
            if bodies.get(link, 0) == 0:
                raise ValueError(f"collision.json has capsules for {link!r}, which isn't a moving link.")
            for c in capsules:
                if len(c) != 7 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in c) or c[6] <= 0:
                    raise ValueError(f"Bad capsule for {link}: {c!r}.")
                self.capsules.append((bodies[link], [float(v) for v in c[0:3]], [float(v) for v in c[3:6]], float(c[6])))
        radius, z0, z1 = (float(v) for v in shapes["base"])
        self.obstacles = [Obstacle(BODY_LABELS[0], "cylinder", [0.0, 0.0, (z0 + z1) / 2], [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                   [radius, z1 - z0], body=0)]
        self.cart_visuals = []
        if cart_path:
            self.obstacles += load_obstacles(cart_path)
            self.cart_visuals = load_visuals(cart_path)
        # The floor is the collision box named "the floor"; without one, the base's mounting face stands in.
        floors = [o.centre[2] + sum(abs(axis[2]) * half for axis, half in zip(o.axes, o.half))
                  for o in self.obstacles if o.label == "the floor" and o.kind == "box"]
        self.floor_z = max(floors) if floors else 0.0
        skip = {frozenset((bodies[a], bodies[b])) for a, b in shapes.get("skip", [])}

        # Pairs to check: (capsule, obstacle or capsule, against an obstacle?, first joint that moves them
        # apart). Neighbouring links touch by design and aren't checked against each other, nor are the pairs
        # in collision.json's skip list.
        self.pairs = []
        for c, (body, _, _, _) in enumerate(self.capsules):
            for o, obstacle in enumerate(self.obstacles):
                if obstacle.body is None or (body - obstacle.body > 1 and frozenset((body, obstacle.body)) not in skip):
                    self.pairs.append((c, o, True, 0))
            for e, (other, _, _, _) in enumerate(self.capsules):
                if body - other > 1 and frozenset((body, other)) not in skip:
                    self.pairs.append((c, e, False, other))
        # How far each capsule's core segment can be from each joint's axis that moves it (m), which bounds how
        # fast its gap to anything changes as the joints turn: its own joint exactly, earlier ones via the
        # joint origins in between. (The radius doesn't count: turning never changes it.)
        offsets = [math.sqrt(sum(fixed[i][3] ** 2 for i in range(3))) for _, fixed, _, _ in self.joints]
        self.reach = []
        for body, a, b, r in self.capsules:
            axis = self.joints[body - 1][2]
            own = max(math.sqrt(max(_dot(p, p) - _dot(p, axis) ** 2, 0.0)) for p in (a, b))
            far = max(math.sqrt(_dot(a, a)), math.sqrt(_dot(b, b)))
            self.reach.append([sum(offsets[j + 1:body]) + far for j in range(body - 1)] + [own])
        self.half_lengths = [_distance(a, b) / 2 for _, a, b, _ in self.capsules]

    def frames(self, joints_deg):
        """World transforms of each moving link for the given joint angles (degrees)."""
        t, out = _translation(0.0, 0.0, 0.0), []
        for (_, fixed, axis, _), q in zip(self.joints, joints_deg):
            t = _matmul(t, _matmul(fixed, _rotation(axis, math.radians(q))))
            out.append(t)
        return out

    def tool_radii(self, joints_deg):
        """How far the flange is from each joint's axis (m): the radius it sweeps when that joint turns alone."""
        frames = self.frames(joints_deg)
        flange = [frames[-1][i][3] for i in range(3)]
        radii = []
        for (_, _, axis, _), t in zip(self.joints, frames):
            arm = _sub(flange, [t[i][3] for i in range(3)])
            along = _dot(arm, [t[i][0] * axis[0] + t[i][1] * axis[1] + t[i][2] * axis[2] for i in range(3)])
            radii.append(math.sqrt(max(_dot(arm, arm) - along * along, 0.0)))
        return radii

    def placed(self, joints_deg):
        """Each capsule's ends in the base frame, for the given joint angles."""
        frames = self.frames(joints_deg)
        return [(_apply(frames[body - 1], a), _apply(frames[body - 1], b)) for body, a, b, _ in self.capsules]

    def _lower_bound(self, placed, pair):
        """A cheap lower bound on a pair's gap, from the capsule's bounding sphere."""
        c, o, fixed, _ = pair
        a, b = placed[c]
        middle = _lerp(a, b, 0.5)
        if fixed:
            near = self.obstacles[o].nearest(middle)
            return _distance(middle, near) - self.half_lengths[c] - self.capsules[c][3]
        other = _lerp(placed[o][0], placed[o][1], 0.5)
        return (_distance(middle, other) - self.half_lengths[c] - self.capsules[c][3]
                - self.half_lengths[o] - self.capsules[o][3])

    def _gap(self, placed, pair):
        """A pair's gap (negative when they overlap) and its closest points."""
        c, o, fixed, _ = pair
        a, b = placed[c]
        if fixed:
            p, q = self.obstacles[o].nearest_to_segment(a, b)
            return _distance(p, q) - self.capsules[c][3], (p, q)
        p, q = segment_segment(a, b, *placed[o])
        return _distance(p, q) - self.capsules[c][3] - self.capsules[o][3], (p, q)

    def clearances(self, joints_deg):
        """Every checked pair's gap (m) at one pose, as (gap, arm part, other thing) sorted nearest first."""
        placed = self.placed(joints_deg)
        out = []
        for pair in self.pairs:
            gap, _ = self._gap(placed, pair)
            out.append((gap, *self._names(pair)))
        return sorted(out)

    def _names(self, pair):
        c, o, fixed, _ = pair
        other = self.obstacles[o].label if fixed else BODY_LABELS[self.capsules[o][0]]
        return BODY_LABELS[self.capsules[c][0]], other

    def check_path(self, start_deg, goal_deg, margin=MARGIN_M):
        """Where a straight joint-space move first brings two things closer than `margin`, or None.

        The check is continuous: each step is short enough that no gap can close before the next check (no
        point can move faster than its reach times its joints' turn). Pairs already closer than the margin
        at the start may move apart, or SLACK_M closer at most, so the arm can always be backed out of a tight
        spot. A hit is {"part", "other", "obstacle" (index, or None for the arm itself), "gap", "at" (0..1
        along the move), "pose", "points" (closest points)}.
        """
        start, goal = [float(q) for q in start_deg], [float(q) for q in goal_deg]
        turn = [abs(math.radians(g - s)) for s, g in zip(start, goal)]
        speeds = [sum(turn[j] * self.reach[c][j] for j in range(low, len(self.reach[c])))
                  for c, _, _, low in self.pairs]
        fastest = max(speeds, default=0.0)
        shortest = ADVANCE_M / fastest if fastest > 0 else 1.0
        limits = [margin] * len(self.pairs)
        t = 0.0
        while True:
            pose = [s + (g - s) * t for s, g in zip(start, goal)]
            placed = self.placed(pose)
            step = math.inf
            for p, pair in enumerate(self.pairs):
                speed = speeds[p]
                if speed == 0.0 and t > 0.0:
                    continue  # nothing on this path moves these two relative to each other
                lower = self._lower_bound(placed, pair)
                if lower >= limits[p] and (speed == 0.0 or lower - limits[p] >= step * speed):
                    continue  # can't come too close before the next check
                gap, points = self._gap(placed, pair)
                if gap < limits[p]:
                    if t > 0.0:
                        part, other = self._names(pair)
                        return {"part": part, "other": other, "obstacle": pair[1] if pair[2] else None, "gap": gap,
                                "at": t, "pose": pose, "points": points}
                    limits[p] = gap - SLACK_M  # already this close: may move apart, not closer
                if speed > 0.0:
                    step = min(step, (gap - limits[p]) / speed)
            if t >= 1.0:
                return None
            t = min(1.0, t + max(step, shortest))

    def collision_config(self):
        """Everything web/collide.js needs to run the same checks in the browser."""
        return {
            "margin": MARGIN_M, "slack": SLACK_M, "advance": ADVANCE_M, "labels": BODY_LABELS,
            "joints": [{"fixed": fixed, "axis": axis} for _, fixed, axis, _ in self.joints],
            "capsules": [{"body": body, "a": a, "b": b, "radius": r} for body, a, b, r in self.capsules],
            "obstacles": [o.config() for o in self.obstacles],
            "pairs": [[c, o, fixed, low] for c, o, fixed, low in self.pairs],
            "reach": self.reach,
        }
