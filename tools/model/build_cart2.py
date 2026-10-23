"""Build the cart model for the jog panel from the SolidWorks export (offline; uses numpy).

    python build_cart.py <export dir> <repo dir>

Inputs (from the SolidWorks export): components.json and components/cart - <name>.STL, in assembly
coordinates (metres). Outputs, all in the arm's URDF base_link frame (z up, origin at the centre of the base's
mounting face):
  <repo>/scripts/model/cart/meshes/{frame,casters,controller}.STL  lightly decimated display meshes
  <repo>/scripts/model/cart/utra_cart.urdf  those meshes as visuals, one box (or cylinder) per part as
                                            collision, and the floor as a thin box under the casters
"""
import json
import os
import struct
import sys

import numpy as np

SRC = sys.argv[1]
REPO = sys.argv[2]
OUT = os.path.join(REPO, "scripts", "model", "cart")
STL_DTYPE = np.dtype([("n", "<3f4"), ("v", "<3f4", (3,)), ("a", "<u2")])

# (CAD name prefix, label used in messages, display mesh)
PARTS = [
    ("30x30 Aluminum Extrusion Vertical Support", "the cart's column", "frame"),
    ("15x30 Aluminum Extrusion Base Half", "a cart base bar", "frame"),
    ("15x30 Aluminum Extrusion Base", "a cart base bar", "frame"),
    ("15x30 Aluminum Extrusion Support", "a cart brace", "frame"),
    ("Mounting Plate D170mm", "the mounting plate", "frame"),
    ("15 Series 4 Hole - Wide Inside Corner Bracket", "a corner bracket", "frame"),
    ("15 Series 4 Hole Inside Corner Gusset", "a corner gusset", "frame"),
    ("5-16x18 x 0p687in Flanged Button Head Socket Cap Scr", "a cap screw", "frame"),
    ("NTRO Controller Mounting Plank", "the controller's plank", "frame"),
    ("NTRO_350-750", "the NTRO controller", "controller"),
    ("50mm Swivel Caster", "a caster", "casters"),
]
MESHES = {  # vertex-clustering cell (m) and colour for each display mesh
    "frame": (0.0015, "0.62 0.64 0.67 1", "Anodised"),
    "casters": (0.004, "0.16 0.17 0.18 1", "Rubber"),
    "controller": (0.004, "0.24 0.25 0.27 1", "Enclosure"),
}
CYLINDERS = ("Mounting Plate D170mm",)  # round parts: collide as cylinders, not boxes

comps = json.load(open(os.path.join(SRC, "components.json")))["components"]


def stl(path):
    data = open(path, "rb").read()
    n = struct.unpack_from("<I", data, 80)[0]
    return np.frombuffer(data, dtype=STL_DTYPE, count=n, offset=84)["v"].astype(float)  # (n, 3, 3)


# URDF base_link -> SolidWorks 1000-BASE part frame: x_sw = y' + 0.0828, y_sw = z' - 0.1175, z_sw = x'.
# The base's cylinder axis and mounting face come from the part's own geometry; the turn about that axis is
# the best fit of Umbratek's two base meshes (1.2 mm mean gap; the next best, 180 deg off, is 1.7 mm).
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
    """Vertex-clustering decimation: snap vertices to a grid, drop triangles that collapse or repeat."""
    keys = np.round(tris.reshape(-1, 3) / cell).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1, 3)
    snapped = np.round(tris / cell) * cell
    keep = (inverse[:, 0] != inverse[:, 1]) & (inverse[:, 1] != inverse[:, 2]) & (inverse[:, 0] != inverse[:, 2])
    _, first = np.unique(np.sort(inverse[keep], axis=1), axis=0, return_index=True)
    return snapped[keep][np.sort(first)]


def write_stl(path, tris, title):
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=STL_DTYPE)
    rec["n"], rec["v"] = normals, tris
    with open(path, "wb") as f:
        f.write(title.encode("ascii").ljust(80, b" ")[:80])
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


parts, display = [], {name: [] for name in MESHES}
for comp in comps:
    name = comp["name"]
    if name.startswith("1000-") or comp["hidden"] or comp["suppressed"]:
        continue  # the arm comes from Umbratek's URDF; hidden parts (spare weights) aren't on the cart
    prefix, label, mesh = next(p for p in PARTS if name.startswith(p[0]))
    tris = to_base(stl(os.path.join(SRC, "components", f"cart - {name}.STL")).reshape(-1, 3)).reshape(-1, 3, 3)
    verts = tris.reshape(-1, 3)
    # Oriented box along the part's own axes (exact for extrusions, plates and boxes; snug for the rest).
    axes = np.array(comp["transform"][:9]).reshape(3, 3) @ M  # rows: the part's axes in base_link
    proj = verts @ axes.T
    lo, hi = proj.min(0), proj.max(0)
    part = {"name": name, "label": label, "centre": ((lo + hi) / 2) @ axes, "axes": axes.T, "size": hi - lo}
    if prefix in CYLINDERS:
        k = int(np.argmin(part["size"]))  # the thin direction is the axis
        axis = part["axes"][:, k]
        rel = verts - part["centre"]
        radial = np.linalg.norm(rel - np.outer(rel @ axis, axis), axis=1)
        # Put the cylinder's axis on local z: keep a right-handed frame with `axis` as its third column.
        others = [part["axes"][:, j] for j in range(3) if j != k]
        frame = np.column_stack([others[0], np.cross(axis, others[0]), axis])
        part.update(cylinder=(radial.max(), part["size"][k]), axes=frame)
    parts.append(part)
    display[mesh].append(cluster(tris, MESHES[mesh][0]))

floor = min(p["centre"][2] - 0.5 * np.abs(p["axes"] @ p["size"])[2] for p in parts if "cylinder" not in p)
os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
for old in os.listdir(os.path.join(OUT, "meshes")):
    os.remove(os.path.join(OUT, "meshes", old))
sizes = []
for mesh, chunks in display.items():
    tris = np.concatenate(chunks)
    write_stl(os.path.join(OUT, "meshes", f"{mesh}.STL"), tris, f"UTRA cart {mesh}, base_link frame, metres")
    sizes.append(f"{mesh} {len(tris)} triangles")

lines = ['<?xml version="1.0" encoding="utf-8"?>',
         "<!-- The aluminium-extrusion cart the UTRA 6-1000 stands on, from the SolidWorks assembly",
         "     \"UTRA 1000 Aluminum Extrusion Assembly\" (see SOURCE.txt). Everything is in the arm's base_link",
         "     frame: z up, origin at the centre of the base's mounting face; attach this link to base_link with",
         "     a fixed joint at the identity. Collision: one box per part (the round mounting plate is a",
         "     cylinder), and the floor as a thin box under the casters. Collision names label the parts. -->",
         '<robot name="utra_cart">', '  <link name="cart">']
for mesh, (_, rgba, material) in MESHES.items():
    lines += ["    <visual>", '      <origin xyz="0 0 0" rpy="0 0 0"/>',
              f'      <geometry><mesh filename="meshes/{mesh}.STL"/></geometry>',
              f'      <material name="{material}"><color rgba="{rgba}"/></material>', "    </visual>"]
for p in sorted(parts, key=lambda p: p["name"]):
    r, pi, y = rpy(p["axes"])
    xyz = " ".join(f"{v:.5f}" for v in p["centre"])
    if "cylinder" in p:
        geometry = f'<cylinder radius="{p["cylinder"][0]:.5f}" length="{p["cylinder"][1]:.5f}"/>'
    else:
        geometry = f'<box size="{" ".join(f"{v:.5f}" for v in p["size"])}"/>'
    lines += [f'    <collision name="{p["label"]}">  <!-- {p["name"]} -->',
              f'      <origin xyz="{xyz}" rpy="{r:.6f} {pi:.6f} {y:.6f}"/>',
              f"      <geometry>{geometry}</geometry>", "    </collision>"]
lines += ['    <collision name="the floor">', f'      <origin xyz="0 0 {floor - 0.005:.5f}" rpy="0 0 0"/>',
          '      <geometry><box size="6 6 0.01"/></geometry>', "    </collision>", "  </link>", "</robot>", ""]
with open(os.path.join(OUT, "utra_cart.urdf"), "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

total = sum(os.path.getsize(os.path.join(OUT, "meshes", f)) for f in os.listdir(os.path.join(OUT, "meshes")))
print(f"{len(parts)} cart parts; display meshes {', '.join(sizes)}; {total / 1e6:.2f} MB; floor at z = {floor * 1000:.1f} mm")
for p in parts:
    if "cylinder" in p:
        print(f"  {p['name']}: cylinder r={p['cylinder'][0] * 1000:.1f} mm, length {p['cylinder'][1] * 1000:.1f} mm,"
              f" axis {np.round(p['axes'][:, 2], 3)}, centre {np.round(p['centre'] * 1000, 1)} mm")
