"""The twin's MuJoCo model (MJCF), generated every time from the repo's own sources, so there's one source of truth:

- the arm: Umbratek's URDF as scripts/utra_model.py reads it (joint origins and axes), its meshes, and the collision
  capsules fitted to them (scripts/model/utra6_1000/collision.json); joint limits as the controller enforces them
  (tools/utra_sim.py);
- the cart and the floor: scripts/model/cart/utra_cart.urdf (visual meshes, collision boxes, the floor's height);
- the cell, if a cell folder is given: its active tool (tool point and capsules), keep-out zones, frames and targets,
  through scripts/cell.py;
- the obstacles and cameras of a Scene.

The world frame is the arm's base frame (z up, origin at the base's mounting face), in metres, as in the URDFs. Scene
files use the cell's conventions instead: millimetres, and roll, pitch, yaw in degrees (R = Rz(yaw) Ry(pitch) Rx(roll)).

Geom groups: 0 the world (floor, cart, obstacles), 1 the arm, 2 the tool, which cameras see; 3 collision shapes,
4 keep-out zones, 5 frames and targets, for the viewer only. Contacts: the arm's capsules and the tool touch the cart,
the floor, the arm's base and obstacles; free obstacles touch everything. The arm's own capsules never touch each
other (the spine checks that before every move). Contacts only ever push free obstacles: the arm's joints are set
from the controller (twin.py), so contacts() reports what the arm touches, and nothing stops it.
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from . import repo

# Contact classes (contype / conaffinity bits).
ARM, WORLD, FREE = 1, 2, 4
GROUP_WORLD, GROUP_ARM, GROUP_TOOL, GROUP_COLLISION, GROUP_ZONES, GROUP_MARKERS = range(6)
ARM_LINKS = ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6")
JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
SHAPES = ("box", "sphere", "cylinder", "capsule", "mesh")
FLANGE = "link_6"


class SceneError(ValueError):
    pass


# ---- small geometry ---------------------------------------------------------------------------------------------

def matrix(t):
    return np.array(t, dtype=float)


def quat(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix."""
    R = np.asarray(R, dtype=float)[:3, :3]
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """base <- camera (4x4, metres) for a camera at `eye` looking at `target` (mm), in OpenCV's convention: x right,
    y down, z along the view."""
    eye, target, up = (np.asarray(v, dtype=float) for v in (eye, target, up))
    z = target - eye
    if np.linalg.norm(z) < 1e-9:
        raise SceneError("A camera's eye and target must differ.")
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-9:
        raise SceneError("A camera's up direction can't be along its view; give another 'up'.")
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = eye / 1000.0
    return T


# OpenCV's camera frame (x right, y down, z forward) to MuJoCo's (x right, y up, looking down -z), and back.
CV_TO_MJ = np.diag([1.0, -1.0, -1.0])


def fmt(values):
    """Numbers for the MJCF, each the shortest text that reads back as exactly the same double."""
    return " ".join(repr(float(v)) for v in np.ravel(values))


# ---- what's placed in a scene -------------------------------------------------------------------------------------

@dataclasses.dataclass
class Obstacle:
    """Something placed in the cell. Sizes in mm: box [x, y, z] (full lengths), sphere [radius], cylinder and capsule
    [radius, length] (along their z axis), mesh [scale] (mesh units to mm; 1 for a mesh in mm). The pose is the
    shape's centre, {"xyz": mm, "rpy": deg}, in `frame` ("base" or a cell frame). A fixed obstacle stays where it's put
    (Twin.move_obstacle moves it); a free one falls and can be pushed."""
    name: str
    shape: str
    size: list
    pose: dict = dataclasses.field(default_factory=lambda: {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})
    frame: str = "base"
    fixed: bool = True
    mass: float = 0.5
    rgba: list = dataclasses.field(default_factory=lambda: [0.62, 0.55, 0.45, 1.0])
    mesh: str | None = None

    def check(self):
        check_name(self.name, "obstacle")
        if self.shape not in SHAPES:
            raise SceneError(f"Obstacle {self.name!r}: shape must be one of {', '.join(SHAPES)}.")
        need = {"box": 3, "sphere": 1, "cylinder": 2, "capsule": 2, "mesh": 1}[self.shape]
        if len(self.size) != need or not all(isinstance(v, (int, float)) and v > 0 for v in self.size):
            raise SceneError(f"Obstacle {self.name!r}: a {self.shape} needs {need} positive size(s) in mm.")
        if self.shape == "mesh" and not (self.mesh and os.path.isfile(self.mesh)):
            raise SceneError(f"Obstacle {self.name!r}: mesh file {self.mesh!r} not found.")
        if not self.fixed and not self.mass > 0:
            raise SceneError(f"Obstacle {self.name!r}: a free obstacle needs a mass (kg).")
        check_pose(self.pose, f"obstacle {self.name!r}")
        return self


@dataclasses.dataclass
class CameraSpec:
    """A simulated RGB-D camera: colour and depth from one viewpoint (depth aligned to colour), with pinhole
    intrinsics in pixels (OpenCV's convention: pixel centres at whole numbers; cx, cy default to the image centre)
    and no distortion. Default: 640 x 480 at fx = fy = 615, about a RealSense D435's colour stream.

    mount: "base" (fixed in the cell), a cell frame, "flange" (on the arm) or a link ("link_1" to "link_6").
    pose: the camera's optical frame (OpenCV: x right, y down, z along the view) in the mount, {"xyz": mm, "rpy":
    deg}; or, for a fixed camera, {"eye": mm, "target": mm, "up": [0, 0, 1]} in the mount's frame.
    near, far (m): depth outside this range reads 0, as a depth camera reports no data."""
    name: str
    width: int = 640
    height: int = 480
    fx: float = 615.0
    fy: float = 615.0
    cx: float | None = None
    cy: float | None = None
    mount: str = "base"
    pose: dict = dataclasses.field(default_factory=lambda: {"eye": [1500.0, 0.0, 700.0], "target": [300.0, 0.0, 0.0]})
    near: float = 0.1
    far: float = 6.0

    def __post_init__(self):
        if self.cx is None:
            self.cx = (self.width - 1) / 2.0
        if self.cy is None:
            self.cy = (self.height - 1) / 2.0

    @property
    def K(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def check(self):
        check_name(self.name, "camera")
        if not (16 <= self.width <= 4096 and 16 <= self.height <= 4096):
            raise SceneError(f"Camera {self.name!r}: width and height must be 16 to 4096 pixels.")
        if not (self.fx > 0 and self.fy > 0):
            raise SceneError(f"Camera {self.name!r}: focal lengths must be positive (pixels).")
        if not (0 < self.near < self.far):
            raise SceneError(f"Camera {self.name!r}: need 0 < near < far (m).")
        if "eye" in self.pose:
            if self.mount in ARM_LINKS or self.mount == "flange":
                raise SceneError(f"Camera {self.name!r}: a camera on the arm needs an xyz/rpy pose, not eye/target.")
        else:
            check_pose(self.pose, f"camera {self.name!r}")
        return self

    def mount_matrix(self):
        """mount <- camera (4x4, metres, OpenCV axes)."""
        if "eye" in self.pose:
            return look_at(self.pose["eye"], self.pose["target"], self.pose.get("up", (0.0, 0.0, 1.0)))
        return matrix(repo.spine("kinematics").pose_to_matrix(self.pose))


@dataclasses.dataclass
class Scene:
    """What the twin builds: the cell folder (or None), obstacles and cameras."""
    cell: str | None = None
    obstacles: list = dataclasses.field(default_factory=list)
    cameras: list = dataclasses.field(default_factory=list)

    @classmethod
    def load(cls, path):
        """A scene from JSON: {"cell": folder, "obstacles": [...], "cameras": [...]}; relative paths are taken from
        the file's folder."""
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data, os.path.dirname(os.path.abspath(path)))

    @classmethod
    def from_dict(cls, data, folder="."):
        def local(p):
            return p if p is None or os.path.isabs(p) else os.path.normpath(os.path.join(folder, p))

        unknown = set(data) - {"cell", "obstacles", "cameras", "about"}
        if unknown:
            raise SceneError(f"Unknown scene keys: {', '.join(sorted(unknown))}.")
        obstacles = [Obstacle(**dict(o, mesh=local(o.get("mesh")))) for o in data.get("obstacles", [])]
        cameras = [CameraSpec(**c) for c in data.get("cameras", [])]
        return cls(local(data.get("cell")), obstacles, cameras)

    def to_dict(self):
        return {"cell": self.cell, "obstacles": [dataclasses.asdict(o) for o in self.obstacles],
                "cameras": [dataclasses.asdict(c) for c in self.cameras]}


def check_name(name, what):
    cell = repo.spine("cell")
    if not isinstance(name, str) or not cell.NAME.match(name):
        raise SceneError(f"Unsupported {what} name {name!r}: up to 40 letters, digits, '_', '-' or '.', from a letter.")
    return name


def check_pose(pose, what):
    if not isinstance(pose, dict) or set(pose) != {"xyz", "rpy"}:
        raise SceneError(f"{what}: a pose is {{'xyz': [mm], 'rpy': [deg]}}, got {pose!r}.")
    for key in ("xyz", "rpy"):
        v = pose[key]
        if not isinstance(v, (list, tuple)) or len(v) != 3 or not all(
                isinstance(x, (int, float)) and math.isfinite(x) for x in v):
            raise SceneError(f"{what}: {key} must be three numbers, got {v!r}.")


# ---- the model --------------------------------------------------------------------------------------------------

@dataclasses.dataclass
class Built:
    """A generated model: the MJCF, the files it refers to, and what each part is."""
    xml: str
    assets: dict
    labels: dict  # collision geom name -> (kind, label): kind is arm, tool, base, cart, floor, obstacle, zone
    seen: dict  # every geom cameras see -> "kind: label", for segmentation
    cameras: dict  # name -> CameraSpec
    obstacles: dict  # name -> Obstacle
    zones: dict  # name -> (centre, axes, half sizes), metres, base frame
    tool: np.ndarray  # flange <- tool point (4x4, metres)
    cell: object  # scripts/cell.py's Cell, or None


def _urdf_visuals(path):
    """link name -> (mesh basename, rgba) from a URDF's visual elements (all at the link's origin)."""
    root = ET.parse(path).getroot()
    colors = {m.get("name"): [float(v) for v in m.find("color").get("rgba").split()]
              for m in root.findall("material") if m.find("color") is not None}
    out = {}
    for link in root.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh = visual.find("geometry/mesh")
        material = visual.find("material")
        rgba = colors.get(material.get("name"), [0.7, 0.7, 0.7, 1.0]) if material is not None else [0.7, 0.7, 0.7, 1.0]
        out[link.get("name")] = (os.path.basename(mesh.get("filename")), rgba)
    return out


def _box_inertia(mass, size):
    x, y, z = size
    return [mass / 12.0 * (y * y + z * z), mass / 12.0 * (x * x + z * z), mass / 12.0 * (x * x + y * y)]


def _stl_bounds(path):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        v = [float(x) for line in data.decode(errors="ignore").splitlines() if line.strip().startswith("vertex")
             for x in line.split()[1:4]]
        v = np.array(v).reshape(-1, 3)
    else:
        n = int.from_bytes(data[80:84], "little")
        tri = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n,
                            offset=84)
        v = tri["v"].reshape(-1, 3).astype(float)
    return v.min(axis=0), v.max(axis=0)


def build(scene=None):
    """The MJCF for `scene` (a Scene, or None for the arm on its cart alone)."""
    scene = scene or Scene()
    utra_model = repo.spine("utra_model")
    utra_sim = repo.spine("utra_sim")
    kin = repo.spine("kinematics")
    cell_mod = repo.spine("cell")
    arm = utra_model.ArmModel()

    cell = None
    if scene.cell:
        if not os.path.isfile(os.path.join(scene.cell, "cell.json")):
            raise SceneError(f"No cell.json in {scene.cell!r}.")
        cell = cell_mod.Cell(scene.cell)
    for o in scene.obstacles:
        o.check()
    for c in scene.cameras:
        c.check()
    names = [o.name for o in scene.obstacles] + [c.name for c in scene.cameras]
    if len(set(names)) != len(names):
        raise SceneError("Obstacle and camera names must all differ.")

    def frame_matrix(name, what):
        if name == "base":
            return np.eye(4)
        if cell is None:
            raise SceneError(f"{what} is in frame {name!r}, but the scene has no cell.")
        try:
            return matrix(cell.frame_matrix(name))
        except cell_mod.CellError as e:
            raise SceneError(f"{what}: {e}")

    labels, seen, assets = {}, {}, {}
    root = ET.Element("mujoco", model="utra6_1000_twin")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", inertiagrouprange=f"{GROUP_COLLISION} {GROUP_COLLISION}")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast")
    width = max([c.width for c in scene.cameras] + [1280])
    height = max([c.height for c in scene.cameras] + [960])
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    # No multisampling: every pixel's depth and segmentation are then exactly what's at its centre (colour edges are
    # aliased, where a real camera's are soft).
    ET.SubElement(visual, "quality", shadowsize="4096", offsamples="0")
    ET.SubElement(visual, "map", znear="0.005", zfar="25")  # times the extent: 1 cm to 50 m
    ET.SubElement(visual, "headlight", ambient="0.25 0.25 0.25", diffuse="0.45 0.45 0.45", specular="0.05 0.05 0.05")
    ET.SubElement(root, "statistic", extent="2", center="0.35 0 -0.2")

    default = ET.SubElement(root, "default")
    for cls, attrs in (
            ("visual", dict(contype="0", conaffinity="0", group=str(GROUP_ARM))),
            ("arm", dict(type="capsule", group=str(GROUP_COLLISION), contype=str(ARM), conaffinity=str(WORLD | FREE),
                         rgba="0.85 0.35 0.2 0.4")),
            ("world", dict(group=str(GROUP_COLLISION), contype=str(WORLD), conaffinity=str(ARM | FREE),
                           rgba="0.3 0.5 0.85 0.4")),
            ("zone", dict(type="box", group=str(GROUP_ZONES), contype="0", conaffinity="0", rgba="0.85 0.3 0.2 0.18"))):
        d = ET.SubElement(default, "default", {"class": cls})
        ET.SubElement(d, "geom", attrs)
    marker = ET.SubElement(default, "default", {"class": "marker"})
    ET.SubElement(marker, "site", group=str(GROUP_MARKERS), rgba="0.92 0.92 0.88 1")

    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="floor", type="2d", builtin="checker", rgb1="0.34 0.35 0.37",
                  rgb2="0.29 0.30 0.32", width="512", height="512")
    ET.SubElement(asset, "material", name="floor", texture="floor", texrepeat="6 6", reflectance="0.04")

    def add_mesh(name, path, scale=1.0):
        key = f"{name}.stl" if path.lower().endswith(".stl") else f"{name}{os.path.splitext(path)[1].lower()}"
        with open(path, "rb") as f:
            assets[key] = f.read()
        ET.SubElement(asset, "mesh", name=name, file=key, scale=fmt([scale] * 3))
        return name

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="sky", directional="true", pos="0 0 4", dir="0 0 -1",
                  diffuse="0.55 0.55 0.55", specular="0.1 0.1 0.1", castshadow="true")
    ET.SubElement(world, "light", name="fill", directional="true", pos="0 0 4", dir="-0.4 -0.3 -1",
                  diffuse="0.2 0.2 0.2", specular="0 0 0", castshadow="false")

    # The floor: the top of the cart model's floor box, as the spine's collision check has it.
    ET.SubElement(world, "geom", name="floor", type="plane", pos=fmt([0, 0, arm.floor_z]), size="0 0 0.05",
                  material="floor", group=str(GROUP_WORLD), contype=str(WORLD), conaffinity=str(ARM | FREE))
    labels["floor"] = ("floor", "the floor")
    seen["floor"] = "floor: the floor"

    # The cart: its meshes for the eye, its boxes for contacts (as the spine's check), all fixed.
    cart = ET.SubElement(world, "body", name="cart")
    for k, v in enumerate(arm.cart_visuals):
        mesh = add_mesh(f"cart_{os.path.splitext(os.path.basename(v['file']))[0].lower()}",
                        os.path.join(repo.MODEL_DIR, "cart", v["file"]))
        ET.SubElement(cart, "geom", name=f"cart_visual_{k}", type="mesh", mesh=mesh, rgba=fmt(v["color"]),
                      contype="0", conaffinity="0", group=str(GROUP_WORLD))
        seen[f"cart_visual_{k}"] = "cart: the cart"
    k = 0
    for o in arm.obstacles:
        if o.body == 0 or o.label == "the floor":
            continue  # the arm's base (below) and the floor (the plane above)
        name = f"cart_{k}"
        k += 1
        R = np.array(o.axes).T if o.kind == "box" else None
        if o.kind == "box":
            ET.SubElement(cart, "geom", {"class": "world"}, name=name, type="box", pos=fmt(o.centre), quat=fmt(quat(R)),
                          size=fmt(o.half))
        else:
            z = np.array(o.axis)
            x = np.cross([0.0, 1.0, 0.0] if abs(z[1]) < 0.9 else [1.0, 0.0, 0.0], z)
            x /= np.linalg.norm(x)
            R = np.column_stack([x, np.cross(z, x), z])
            ET.SubElement(cart, "geom", {"class": "world"}, name=name, type="cylinder", pos=fmt(o.centre),
                          quat=fmt(quat(R)), size=fmt([o.radius, o.half_length]))
        labels[name] = ("cart", o.label)

    # The arm: Umbratek's chain as utra_model parses it, the meshes for the eye and the fitted capsules for contacts.
    visuals = _urdf_visuals(utra_model.URDF_PATH)
    mesh_dir = os.path.join(repo.MODEL_DIR, "utra6_1000", "meshes")
    base = ET.SubElement(world, "body", name="base_link")
    file, rgba = visuals["base_link"]
    ET.SubElement(base, "geom", {"class": "visual"}, name="base_visual", type="mesh",
                  mesh=add_mesh("arm_base", os.path.join(mesh_dir, file)), rgba=fmt(rgba))
    seen["base_visual"] = f"arm: {utra_model.BODY_LABELS[0]}"
    b = arm.obstacles[0]  # the base's cylinder, which the shoulder and upper arm never check against
    ET.SubElement(base, "geom", {"class": "world"}, name="base_collision", type="cylinder", pos=fmt(b.centre),
                  size=fmt([b.radius, b.half_length]))
    labels["base_collision"] = ("base", b.label)
    parent = base
    bodies = {}
    for k, ((joint, fixed, axis, child), limits) in enumerate(zip(arm.joints, utra_sim.JOINT_LIMITS_RAD)):
        F = matrix(fixed)
        body = ET.SubElement(parent, "body", name=child, pos=fmt(F[:3, 3]), quat=fmt(quat(F)), gravcomp="1")
        ET.SubElement(body, "joint", name=joint, type="hinge", axis=fmt(axis), range=fmt(limits), armature="10",
                      damping="0")
        file, rgba = visuals[child]
        ET.SubElement(body, "geom", {"class": "visual"}, name=f"{child}_visual", type="mesh",
                      mesh=add_mesh(f"arm_{os.path.splitext(file)[0].lower()}", os.path.join(mesh_dir, file)),
                      rgba=fmt(rgba))
        seen[f"{child}_visual"] = f"arm: {utra_model.BODY_LABELS[k + 1]}"
        bodies[k + 1] = body
        parent = body
    for k, (body, a, b_, r) in enumerate(arm.capsules):
        name = f"arm_capsule_{k}"
        if math.dist(a, b_) < 1e-9:  # a capsule with both ends together is a sphere (the flange's)
            ET.SubElement(bodies[body], "geom", {"class": "arm"}, name=name, type="sphere", pos=fmt(a), size=fmt([r]))
        else:
            ET.SubElement(bodies[body], "geom", {"class": "arm"}, name=name, fromto=fmt(list(a) + list(b_)),
                          size=fmt([r]))
        labels[name] = ("arm", utra_model.BODY_LABELS[body])
    flange = bodies[6]
    ET.SubElement(flange, "site", {"class": "marker"}, name="flange", type="sphere", size="0.004")

    # The cell: the tool in use on the flange, zones, frames and targets.
    tool = np.eye(4)
    if cell is not None:
        tool = matrix(cell.tool_matrix())
        for k, c in enumerate(cell.tool()["capsules"]):
            name = f"tool_capsule_{k}"
            a, b_ = [v / 1000.0 for v in c["a"]], [v / 1000.0 for v in c["b"]]
            shape = dict(type="sphere", pos=fmt(a)) if math.dist(a, b_) < 1e-9 else \
                dict(type="capsule", fromto=fmt(a + b_))
            ET.SubElement(flange, "geom", name=name, group=str(GROUP_TOOL), contype=str(ARM),
                          conaffinity=str(WORLD | FREE), rgba="0.55 0.57 0.6 1", size=fmt([c["r"] / 1000.0]), **shape)
            labels[name] = ("tool", "the tool")
            seen[name] = "tool: the tool"
    ET.SubElement(flange, "site", {"class": "marker"}, name="tcp", type="sphere", size="0.005", pos=fmt(tool[:3, 3]),
                  quat=fmt(quat(tool)))
    zones = {}
    if cell is not None:
        snapshot = cell.snapshot()
        for (name, _), o in zip(snapshot["zones"].items(), cell.zone_obstacles()):
            R = np.array(o.axes).T
            zones[name] = (np.array(o.centre), R, np.array(o.half))
            ET.SubElement(world, "geom", {"class": "zone"}, name=f"zone_{name}", pos=fmt(o.centre), quat=fmt(quat(R)),
                          size=fmt(o.half))
            labels[f"zone_{name}"] = ("zone", o.label)
        for name in snapshot["frames"]:
            T = matrix(cell.frame_matrix(name))
            ET.SubElement(world, "site", {"class": "marker"}, name=f"frame_{name}", type="sphere", size="0.006",
                          pos=fmt(T[:3, 3]))
            ET.SubElement(world, "site", {"class": "marker"}, name=f"frame_{name}_x", type="capsule", size="0.002",
                          fromto=fmt(list(T[:3, 3]) + list(T[:3, 3] + 0.06 * T[:3, 0])))
        for name, target in snapshot["targets"].items():
            if "joints" not in target:
                T = matrix(cell.target_matrix(name))
                ET.SubElement(world, "site", {"class": "marker"}, name=f"target_{name}", type="sphere", size="0.004",
                              pos=fmt(T[:3, 3]), rgba="0.95 0.75 0.3 1")

    # Obstacles.
    obstacles = {}
    for o in scene.obstacles:
        T = frame_matrix(o.frame, f"Obstacle {o.name!r}") @ matrix(kin.pose_to_matrix(o.pose))
        body = ET.SubElement(world, "body", name=f"obstacle_{o.name}", pos=fmt(T[:3, 3]), quat=fmt(quat(T)))
        size_m = [v / 1000.0 for v in o.size]
        geom = {"name": f"obstacle_{o.name}", "rgba": fmt(o.rgba), "group": str(GROUP_WORLD),
                "contype": str(WORLD if o.fixed else FREE), "conaffinity": str(ARM | FREE | (0 if o.fixed else WORLD))}
        if o.shape == "box":
            geom.update(type="box", size=fmt([v / 2 for v in size_m]))
            extent = size_m
        elif o.shape == "sphere":
            geom.update(type="sphere", size=fmt(size_m))
            extent = [2 * size_m[0]] * 3
        elif o.shape in ("cylinder", "capsule"):
            geom.update(type=o.shape, size=fmt([size_m[0], size_m[1] / 2]))
            extent = [2 * size_m[0], 2 * size_m[0], size_m[1] + (2 * size_m[0] if o.shape == "capsule" else 0)]
        else:
            scale = size_m[0]
            geom.update(type="mesh", mesh=add_mesh(f"obstacle_{o.name}", o.mesh, scale))
            low, high = _stl_bounds(o.mesh) if o.mesh.lower().endswith(".stl") else (np.zeros(3), np.ones(3) * 0.1)
            extent = list((high - low) * scale)
        if o.fixed:
            body.set("mocap", "true")
        else:
            ET.SubElement(body, "freejoint", name=f"obstacle_{o.name}")
            ET.SubElement(body, "inertial", pos="0 0 0", mass=fmt([o.mass]),
                          diaginertia=fmt(_box_inertia(o.mass, [max(v, 1e-3) for v in extent])))
        ET.SubElement(body, "geom", geom)
        labels[f"obstacle_{o.name}"] = ("obstacle", o.name)
        seen[f"obstacle_{o.name}"] = f"obstacle: {o.name}"
        obstacles[o.name] = o

    # Cameras: MuJoCo's camera looks down its -z with y up; ours are given in OpenCV's axes.
    cameras = {}
    for c in scene.cameras:
        if c.mount in ARM_LINKS or c.mount == "flange":
            parent_el, T = bodies[6 if c.mount == "flange" else ARM_LINKS.index(c.mount) + 1], c.mount_matrix()
        else:
            parent_el, T = world, frame_matrix(c.mount, f"Camera {c.name!r}") @ c.mount_matrix()
        R = T[:3, :3] @ CV_TO_MJ
        # MuJoCo's principal point is an offset from the image centre, whose sign is the opposite of OpenCV's
        # (checked by tests/twin_tests.py's projection test).
        ET.SubElement(parent_el, "camera", name=c.name, pos=fmt(T[:3, 3]), quat=fmt(quat(R)),
                      resolution=f"{c.width} {c.height}", sensorsize=f"{c.width} {c.height}",
                      focalpixel=fmt([c.fx, c.fy]),
                      principalpixel=fmt([(c.width - 1) / 2.0 - c.cx, (c.height - 1) / 2.0 - c.cy]))
        cameras[c.name] = c

    ET.SubElement(ET.SubElement(root, "contact"), "exclude", body1="base_link", body2="link_2")
    # The base is welded to the world, and MuJoCo never filters contacts with the world as a parent: say so here.
    root.find("contact").append(ET.Element("exclude", body1="base_link", body2="link_1"))
    ET.indent(root, "  ")
    xml = "<!-- Generated by brain/twin/model.py from the repo's sources; regenerated every run, don't edit. -->\n" + \
        ET.tostring(root, encoding="unicode")
    return Built(xml, assets, labels, seen, cameras, obstacles, zones, tool, cell)


def compile_model(built):
    import mujoco
    return mujoco.MjModel.from_xml_string(built.xml, built.assets)


def export(built, folder):
    """Writes the MJCF and its meshes to `folder` (to open in MuJoCo's viewer, or load elsewhere)."""
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "twin.xml"), "w", encoding="utf-8") as f:
        f.write(built.xml)
    for name, data in built.assets.items():
        with open(os.path.join(folder, name), "wb") as f:
            f.write(data)
    return os.path.join(folder, "twin.xml")
