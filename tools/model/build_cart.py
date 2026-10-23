"""Build the cart model for the jog panel from the SolidWorks export (offline; uses numpy).

Inputs (from sw_export2/3.py): components.json, components/cart - <name>.STL (assembly coordinates, metres).
Outputs, all in the arm's URDF base_link frame (z up, origin at the centre of the base's mounting face):
  <repo>/scripts/model/cart/meshes/cart.STL  - one lightly decimated display mesh for the whole cart
  <repo>/scripts/model/cart/utra_cart.urdf    - that mesh as the visual, one oriented box per part as collision,
                                                 and the floor as a thin box under the casters
"""
import json
import os
import struct
import sys

import numpy as np

SRC = sys.argv[1]
REPO = sys.argv[2]
OUT = os.path.join(REPO, "scripts", "model", "cart")
CLUSTER_M = 0.002  # vertex clustering cell for the display mesh

comps = json.load(open(os.path.join(SRC, "components.json")))["components"]


def stl(path):
    data = open(path, "rb").read()
    n = struct.unpack_from("<I", data, 80)[0]
    a = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3f4", (3,)), ("a", "<u2")]), count=n, offset=84)
    return a["v"].astype(float)  # (n, 3, 3) metres


# URDF base_link -> SolidWorks 1000-BASE part frame: x_sw = y' + 0.0828, y_sw = z' - 0.1175, z_sw = x'
# (cylinder centre and mounting face from the part's own STL; +90 deg about the axis from the base's bolt holes).
A = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
o = np.array([0.0828, -0.1175, 0.0])
base = next(c for c in comps if c["name"].startswith("1000-BASE"))
t = np.array(base["transform"])
R, T = t[:9].reshape(3, 3), t[9:12]  # SolidWorks row-vector convention: p_asm = p_part @ R + T
M = R.T @ A  # p_asm = M @ p' + c
c = R.T @ o + T


def to_base(v):
    return (v - c) @ M  # p' = M^T (p_asm - c), as row vectors


def rpy(rot):
    """URDF roll, pitch, yaw for a rotation matrix (columns = rotated axes): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = np.arcsin(-np.clip(rot[2, 0], -1, 1))
    if abs(rot[2, 0]) < 0.999999:
        roll, yaw = np.arctan2(rot[2, 1], rot[2, 2]), np.arctan2(rot[1, 0], rot[0, 0])
    else:
        roll, yaw = 0.0, np.arctan2(-rot[0, 1], rot[1, 1])
    return roll, pitch, yaw


def cluster(tris, cell):
    """Vertex-clustering decimation: snap vertices to a grid, drop triangles that collapse."""
    keys = np.round(tris.reshape(-1, 3) / cell).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1, 3)
    snapped = (np.round(tris / cell) * cell)
    keep = (inverse[:, 0] != inverse[:, 1]) & (inverse[:, 1] != inverse[:, 2]) & (inverse[:, 0] != inverse[:, 2])
    tri_keys = np.sort(inverse[keep], axis=1)
    _, first = np.unique(tri_keys, axis=0, return_index=True)  # drop duplicates
    return snapped[keep][np.sort(first)]


parts, display = [], []
for comp in comps:
    name = comp["name"]
    if name.startswith("1000-") or comp["hidden"] or comp["suppressed"]:
        continue  # the arm comes from Umbratek's URDF; hidden parts (spare weights) aren't on the cart
    path = os.path.join(SRC, "components", f"cart - {name}.STL")
    tris = to_base(stl(path).reshape(-1, 3)).reshape(-1, 3, 3)
    verts = tris.reshape(-1, 3)
    # Oriented box along the part's own axes (exact for extrusions, plates and boxes; snug for the rest).
    rc = np.array(comp["transform"][:9]).reshape(3, 3)
    axes = rc @ M  # rows: the part's x, y, z axes in base_link
    proj = verts @ axes.T
    lo, hi = proj.min(0), proj.max(0)
    centre = ((lo + hi) / 2) @ axes
    parts.append({"name": name, "centre": centre, "axes": axes.T, "size": hi - lo, "tris": len(tris)})
    display.append(cluster(tris, CLUSTER_M))

floor = min(p["centre"][2] - 0.5 * abs(p["axes"] @ p["size"])[2] for p in parts)
mesh = np.concatenate(display)
os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
with open(os.path.join(OUT, "meshes", "cart.STL"), "wb") as f:
    f.write(b"UTRA cart, base_link frame, metres".ljust(80, b" "))
    f.write(struct.pack("<I", len(mesh)))
    normals = np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(mesh), dtype=np.dtype([("n", "<3f4"), ("v", "<3f4", (3,)), ("a", "<u2")]))
    rec["n"], rec["v"] = normals, mesh
    f.write(rec.tobytes())

lines = ['<?xml version="1.0" encoding="utf-8"?>',
         "<!-- The 8020 cart the UTRA 6-1000 stands on, built by build_cart.py from the SolidWorks assembly",
         "     Inventor22/umbratek-cad: Utra 1000/UTRA 1000 Aluminum Extrusion Assembly.SLDASM.",
         "     Everything is in the arm's base_link frame: z up, origin at the centre of the base's mounting face.",
         "     Collision: one oriented box per part, and the floor as a thin box under the casters. -->",
         '<robot name="utra_cart">', '  <link name="cart">',
         "    <visual>", '      <origin xyz="0 0 0" rpy="0 0 0"/>',
         '      <geometry><mesh filename="meshes/cart.STL"/></geometry>',
         '      <material name="Anodised"><color rgba="0.55 0.56 0.58 1"/></material>', "    </visual>"]
for p in parts:
    r, pi, y = rpy(p["axes"])
    lines += [f'    <collision name="{p["name"]}">',
              f'      <origin xyz="{" ".join(f"{v:.5f}" for v in p["centre"])}" rpy="{r:.6f} {pi:.6f} {y:.6f}"/>',
              f'      <geometry><box size="{" ".join(f"{v:.5f}" for v in p["size"])}"/></geometry>',
              "    </collision>"]
lines += ['    <collision name="floor">', f'      <origin xyz="0 0 {floor - 0.005:.5f}" rpy="0 0 0"/>',
          '      <geometry><box size="6 6 0.01"/></geometry>', "    </collision>",
          "  </link>", "</robot>", ""]
with open(os.path.join(OUT, "utra_cart.urdf"), "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

print(f"{len(parts)} cart parts; display mesh {len(mesh)} triangles, "
      f"{os.path.getsize(os.path.join(OUT, 'meshes', 'cart.STL')) / 1e6:.2f} MB; floor at z = {floor * 1000:.1f} mm")
for p in sorted(parts, key=lambda p: -p["size"].max()):
    print(f"  {p['name'][:52]:52s} centre {np.round(p['centre'] * 1000, 1)} mm  size {np.round(p['size'] * 1000, 1)} mm"
          f"  ({p['tris']} tris)")
