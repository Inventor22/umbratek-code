"""Checks for utra_model's collision geometry against dense sampling (needs NumPy: run with the oracle environment)."""
import math
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import utra_model as um  # noqa: E402

rng = random.Random(4)
fails = 0


def check(name, cond, detail=""):
    global fails
    if not cond:
        fails += 1
    print(f"  {'PASS' if cond else 'FAIL'} {name}{'' if cond else '  <-- ' + str(detail)}")


def rand3(s=1.0):
    return [rng.uniform(-s, s) for _ in range(3)]


def rot():
    q = np.linalg.qr(np.random.default_rng(rng.randrange(10**6)).normal(size=(3, 3)))[0]
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1
    return q


ts = np.linspace(0, 1, 4001)
print("== segment-segment vs dense sampling")
worst = 0
for _ in range(300):
    p1, q1, p2, q2 = (np.array(rand3()) for _ in range(4))
    if rng.random() < 0.2:
        q2 = p2 + (q1 - p1) * rng.uniform(-1, 1)  # parallel
    a, b = um.segment_segment(list(p1), list(q1), list(p2), list(q2))
    exact = math.dist(a, b)
    A = p1 + np.outer(ts[::8], q1 - p1)
    B = p2 + np.outer(ts[::8], q2 - p2)
    brute = np.min(np.linalg.norm(A[:, None] - B[None], axis=2))
    worst = max(worst, exact - brute)
    if exact > brute + 1e-3 or exact < brute - 0.01:
        print("   mismatch", exact, brute)
check("segment-segment never above the sampled minimum, within 1e-3", worst < 1e-3, worst)

print("== box and cylinder vs dense sampling of the shape's surface and the segment")
for kind in ("box", "cylinder"):
    worst_over, worst_under, inside_ok = 0.0, 0.0, True
    for n in range(300):
        R = rot()
        size = [rng.uniform(0.05, 1.0) for _ in range(3)]
        centre = rand3(0.5)
        ob = um.Obstacle("x", kind, centre, R.T.tolist(), size if kind == "box" else [size[0] / 2, size[1]])
        p, q = rand3(1.5), rand3(1.5)
        if n % 10 == 0:
            q = list(p)  # a point
        a, b = ob.nearest_to_segment(p, q)
        exact = math.dist(a, b)
        # brute force: distance from many segment points to the shape via nearest()
        seg = np.array(p) + np.outer(ts, np.array(q) - np.array(p))
        brute = min(math.dist(x, ob.nearest(list(x))) for x in seg[::4])
        worst_over = max(worst_over, exact - brute)
        worst_under = max(worst_under, brute - exact)
        # nearest() must be on/in the shape: a point well inside reports itself
        local = R.T @ (np.array(b) - np.array(centre))
        if kind == "box":
            inside_ok &= all(abs(local[k]) <= size[k] / 2 + 1e-9 for k in range(3))
        else:
            inside_ok &= math.hypot(local[0], local[1]) <= size[0] / 2 + 1e-9 and abs(local[2]) <= size[1] / 2 + 1e-9
    check(f"{kind}: exact never above the sampled minimum", worst_over < 1e-6, worst_over)
    check(f"{kind}: exact within sampling resolution below it", worst_under < 3e-3, worst_under)
    check(f"{kind}: closest point lies on the shape", inside_ok)


def oracle_sd(kind, R, centre, size, points):
    """Signed distances of `points` (n x 3) from a box (sides `size`) or a cylinder (diameter size[0], length size[1],
    along R's third column), worked out here with NumPy: outside, the distance; inside, minus the distance to the
    surface."""
    local = (points - np.array(centre)) @ R
    if kind == "box":
        d = np.abs(local) - np.array(size) / 2
    else:
        d = np.stack([np.hypot(local[:, 0], local[:, 1]) - size[0] / 2, np.abs(local[:, 2]) - size[1] / 2], axis=1)
    return np.linalg.norm(np.maximum(d, 0.0), axis=1) + np.minimum(d.max(axis=1), 0.0)


print("== the signed gap (Obstacle.segment_gap): outside, the distance; reaching inside, minus the deepest point's depth")
for kind in ("box", "cylinder"):
    worst_over, worst_under, worst_points, inside = 0.0, 0.0, 0.0, 0
    for n in range(400):
        R = rot()
        size = [rng.uniform(0.05, 1.0) for _ in range(3)]
        centre = rand3(0.5)
        ob = um.Obstacle("x", kind, centre, R.T.tolist(), size if kind == "box" else [size[0] / 2, size[1]])
        p, q = rand3(1.5), rand3(1.5)
        if n % 3 == 0:  # from somewhere inside the shape
            p = list(np.array(centre) + R @ (np.array([rng.uniform(-0.45, 0.45) for _ in range(3)]) * np.array(size)))
        if n % 10 == 0:
            q = list(p)  # a point
        gap, (x, s) = ob.segment_gap(p, q)
        seg = np.array(p) + np.outer(ts, np.array(q) - np.array(p))
        brute = float(oracle_sd(kind, R, centre, size, seg).min())
        inside += gap < 0
        worst_over = max(worst_over, gap - brute)
        worst_under = max(worst_under, brute - gap)
        # its points: on the segment, as far from the shape as the gap says; and on the shape's surface
        at_x, at_s = oracle_sd(kind, R, centre, size, np.array([x, s]))
        worst_points = max(worst_points, abs(at_x - gap), abs(at_s), abs(math.dist(x, s) - abs(gap)))
    check(f"{kind}: the signed gap never above the sampled minimum ({inside} of 400 segments reach inside)",
          worst_over < 1e-9 and inside > 100, (worst_over, inside))
    check(f"{kind}: and within sampling resolution below it", worst_under < 3e-3, worst_under)
    check(f"{kind}: its points are the deepest (or nearest) point of the segment and the surface's nearest it",
          worst_points < 1e-9, worst_points)

# Deeper is closer: a segment pushed into a box along one axis, from 10 mm out to 90 mm in, has a gap falling
# all the way (the old unsigned gap stopped at 0 once inside).
box = um.Obstacle("b", "box", [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.4, 0.4, 0.4])
gaps = [box.segment_gap([0.21 - d, 0.0, 0.05], [0.21 - d, 0.0, -0.05])[0] for d in (0.0, 0.02, 0.06, 0.1)]
check(f"pushing a segment into a box from 10 mm out to 90 mm in, the gap goes {', '.join(f'{g * 1000:.0f}' for g in gaps)}"
      " mm", all(abs(g - want) < 1e-12 for g, want in zip(gaps, (0.01, -0.01, -0.05, -0.09))), gaps)

# nearest() for a cylinder against an analytic answer
cyl = um.Obstacle("c", "cylinder", [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.1, 0.2])
check("cylinder side", abs(math.dist([0.3, 0, 0], cyl.nearest([0.3, 0, 0])) - 0.2) < 1e-12)
check("cylinder cap", abs(math.dist([0, 0, 0.5], cyl.nearest([0, 0, 0.5])) - 0.4) < 1e-12)
check("cylinder rim", abs(math.dist([0.4, 0, 0.5], cyl.nearest([0.4, 0, 0.5])) - math.hypot(0.3, 0.4)) < 1e-12)

print("== the model")
t0 = time.perf_counter()
model = um.ArmModel()
print(f"  loaded in {(time.perf_counter() - t0) * 1000:.0f} ms: {len(model.capsules)} capsules, "
      f"{len(model.obstacles)} obstacles, {len(model.pairs)} pairs")
zero = [0.0] * 6
print("  nearest pairs at the zero pose:")
for gap, a, b in model.clearances(zero)[:12]:
    print(f"    {gap * 1000:7.1f} mm  {a} / {b}")
print(f"\n{fails} failure(s)")
sys.exit(1 if fails else 0)

