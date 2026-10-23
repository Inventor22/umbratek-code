"""Fit collision capsules to the UTRA 6-1000 link meshes (offline; uses numpy).

Each link gets one capsule per housing or tube, along a line picked from its joint axes. Every mesh vertex
and densely sampled surface point is assigned to its nearest line, and each capsule is the smallest-volume
one around that line covering its points. Output: JSON {link: [[ax, ay, az, bx, by, bz, r], ...]} in the
link's frame, metres.
"""
import json
import struct
import sys

import numpy as np

MESHES = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else None
SAMPLES_PER_M2 = 2.0e6  # surface sampling density (about one point per 0.7 mm square)


def stl(path):
    data = open(path, "rb").read()
    n = struct.unpack_from("<I", data, 80)[0]
    a = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3f4", (3,)), ("a", "<u2")]), count=n, offset=84)
    return a["v"].astype(float)


def surface_points(tris, rng):
    """Vertices plus area-weighted random samples, so big flat faces are covered too."""
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    count = rng.poisson(area * SAMPLES_PER_M2)
    idx = np.repeat(np.arange(len(tris)), count)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    samples = tris[idx, 0] + u[:, None] * e1[idx] + v[:, None] * e2[idx]
    return np.concatenate([tris.reshape(-1, 3), samples])


def fit(points, p0, u):
    """Smallest-volume capsule with its segment on the line p0 + s u that covers `points`."""
    d = points - p0
    s = d @ u
    rho = np.linalg.norm(d - np.outer(s, u), axis=1)
    best = None
    for r in np.arange(rho.max(), rho.max() + 0.08, 0.0005):
        # A point is covered when the segment meets [s - reach, s + reach]; the shortest segment meeting
        # every such interval runs from the lowest upper end to the highest lower end.
        reach = np.sqrt(np.maximum(r * r - rho * rho, 0))
        s0, s1 = (s + reach).min(), (s - reach).max()
        if s1 < s0:  # all intervals share a point: a sphere is enough
            s0 = s1 = 0.5 * (s0 + s1)
        volume = np.pi * r * r * (s1 - s0) + 4 / 3 * np.pi * r ** 3
        if best is None or volume < best[0]:
            best = (volume, s0, s1, r)
    _, s0, s1, r = best
    return p0 + s0 * u, p0 + s1 * u, r


def tube_centre(points, z0, z1):
    mid = points[(points[:, 2] > z0) & (points[:, 2] < z1)]
    return np.array([(mid[:, 0].min() + mid[:, 0].max()) / 2, (mid[:, 1].min() + mid[:, 1].max()) / 2, 0.0])


X, Y, Z = np.eye(3)
INF = np.inf
rng = np.random.default_rng(1)
points = {name: surface_points(stl(f"{MESHES}/{name}.STL"), rng) for name in
          ("link1", "link2", "link3", "link4", "link5", "link6")}
# (point on the line, direction, axial range of the points it may take). The tubes of the upper arm and
# forearm are round and slimmer than the collars and housings at their ends, so each tube line is split
# where its radius changes (measured from the meshes) to keep the long middle capsule tight.
T2, T3 = tube_centre(points["link2"], 0.15, 0.33), tube_centre(points["link3"], 0.13, 0.31)
LINES = {
    "link1": [((0, 0, 0), Z, None), ((0, 0, 0.1175), Y, None)],
    "link2": [((0, 0, 0), Y, None), ((0, 0, 0.463), Y, None),
              (T2, Z, (-INF, 0.11)), (T2, Z, (0.11, 0.40)), (T2, Z, (0.40, INF))],
    "link3": [((0, 0, 0), Y, None), ((0, 0, 0.4325), Y, None),
              (T3, Z, (-INF, 0.09)), (T3, Z, (0.09, 0.382)), (T3, Z, (0.382, INF))],
    "link4": [((0, 0, 0), Y, None), ((0, -0.1405, 0), Z, None)],
    "link5": [((0, 0, 0), Z, None), ((0, 0, 0), Y, None)],
    "link6": [((0, 0, 0), Z, None)],
}
result = {}
for name, lines in LINES.items():
    pts = points[name]
    lines = [(np.array(p0, float), np.asarray(u, float), span) for p0, u, span in lines]
    radial, allowed = [], []
    for p0, u, span in lines:
        d = pts - p0
        s = d @ u
        radial.append(np.linalg.norm(d - np.outer(s, u), axis=1))
        allowed.append(np.ones(len(pts), bool) if span is None else (s >= span[0]) & (s < span[1]))
    allowed = np.array(allowed)
    owner = np.argmin(np.where(allowed, np.array(radial), np.inf), axis=0)
    for rounds in range(12):
        caps = [fit(pts[owner == k], p0, u) for k, (p0, u, _) in enumerate(lines)]
        # Give each point to the capsule it sits deepest inside (among those allowed to take it), so a
        # tube stops covering the housings at its ends, then refit.
        depth = []
        for a, b, r in caps:
            ab = b - a
            t = np.clip(((pts - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
            depth.append(np.linalg.norm(pts - (a + np.outer(t, ab)), axis=1) - r)
        new = np.argmin(np.where(allowed, np.array(depth), np.inf), axis=0)
        if (new == owner).all():
            break
        owner = new
    out = []
    for k, (a, b, r) in enumerate(caps):
        out.append([*a.round(4), *b.round(4), round(r + 0.0005, 4)])
        print(f"{name}[{k}] a={np.round(a * 1000, 1)} b={np.round(b * 1000, 1)} r={r * 1000:.1f} mm "
              f"({(owner == k).sum()} pts, {rounds + 1} rounds)")
    result[f"link_{name[-1]}"] = out
base = stl(f"{MESHES}/base.STL").reshape(-1, 3)
base_r = np.hypot(base[:, 0], base[:, 1]).max()
print(f"base: cylinder r={base_r * 1000:.1f} mm, z {base[:, 2].min() * 1000:.1f}..{base[:, 2].max() * 1000:.1f} mm")
if OUT:
    json.dump({"capsules": result, "base": [round(base_r + 0.0005, 4), round(base[:, 2].min(), 4),
                                            round(base[:, 2].max(), 4)]}, open(OUT, "w"), indent=1)
