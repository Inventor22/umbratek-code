"""The twin: the arm on its cart in the cell, as a MuJoCo model generated from the repo's sources (model.py), with its
joints following the controller's stand-in (coupling.py), obstacles that can be placed and moved, contacts, keep-out
zone clearances and simulated RGB-D cameras (cameras.py).

    twin = Twin(Scene.load("brain/twin/scenes/example.json"))
    twin.follow(t, joints_rad)           # the arm where the controller has it at time t; free obstacles step along
    frame = twin.render("overhead")      # rgb, depth, K, T_base_camera
    twin.contacts()                      # what the arm or tool touches
    snap = twin.snapshot()               # its state now, for another thread to render or send (Snapshot)
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import model as twin_model
from . import repo
from .cameras import Cameras, frame_of

MAX_CATCH_UP_S = 0.25  # after a longer gap between follow() calls, free obstacles jump rather than replay it all


@dataclasses.dataclass(frozen=True, eq=False)
class Layout:
    """One build of the twin's model, as other threads see it: they may read the model, never change it."""
    revision: int  # counts the twin's builds, from 1
    model: object  # the compiled MjModel
    built: object  # model.Built: the MJCF, cameras, obstacles
    scene: dict  # the Scene it was built from, as Scene.to_dict() gave it then (later moves aren't in it)
    fingerprint: str  # model.fingerprint(built)
    seen: list  # geom id -> "kind: label" for what cameras see, else None
    qadr: np.ndarray  # the arm's joints' addresses in qpos
    free: dict  # free obstacle -> (qpos address, dof address)
    mocap: dict  # fixed obstacle -> mocap index


@dataclasses.dataclass(eq=False)
class Snapshot:
    """The twin's state at one step of its loop: copies, so another thread can use them while the loop carries on."""
    layout: Layout
    t: float  # s, on the controller's clock
    qpos: np.ndarray
    mocap_pos: np.ndarray
    mocap_quat: np.ndarray
    seq: int = 0  # set by whoever publishes it (server.TwinServer.publish)

    def joints(self):
        """The arm's joints (rad)."""
        return self.qpos[self.layout.qadr]

    def obstacles(self):
        """Where each obstacle is: {name: [x, y, z (m), qw, qx, qy, qz]}, base frame, its body's pose exactly."""
        out = {}
        for name in self.layout.built.obstacles:
            if name in self.layout.free:
                a = self.layout.free[name][0]
                out[name] = self.qpos[a:a + 7].tolist()
            else:
                k = self.layout.mocap[name]
                out[name] = self.mocap_pos[k].tolist() + self.mocap_quat[k].tolist()
        return out

    def sample(self):
        """What GET /pose and /pose/stream send: the model's own units (metres, radians), unrounded, and which build
        they belong to (its revision, and its fingerprint: a follower needs a new model exactly when that changes)."""
        return {"seq": self.seq, "time": self.t, "revision": self.layout.revision,
                "fingerprint": self.layout.fingerprint, "joints": self.joints().tolist(), "obstacles": self.obstacles()}

    def apply(self, data):
        """Puts this state into `data`, an MjData of the same build (call mj_forward after)."""
        data.qpos[:] = self.qpos
        data.mocap_pos[:] = self.mocap_pos
        data.mocap_quat[:] = self.mocap_quat


class Twin:
    def __init__(self, scene=None):
        self.scene = scene or twin_model.Scene()
        self.cameras = None
        self.t, self.q = None, None
        self.revision = 0
        self._build()

    # ---- building -------------------------------------------------------------------------------------------------
    def _build(self):
        import mujoco
        keep = None
        if self.cameras is not None:
            keep = (self.data.qpos[self.qadr].copy(), {n: (self.data.qpos[a:a + 7].copy(), self.data.qvel[v:v + 6].copy())
                                                       for n, (a, v) in self.free.items()})
            self.cameras.close()
        self.built = twin_model.build(self.scene)
        self.model = twin_model.compile_model(self.built)
        self.data = mujoco.MjData(self.model)
        m = self.model
        joints = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in twin_model.JOINTS]
        self.qadr = np.array([m.jnt_qposadr[j] for j in joints])
        self.vadr = np.array([m.jnt_dofadr[j] for j in joints])
        self.flange_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, twin_model.FLANGE)
        self.geoms = [self.built.labels.get(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)) for g in range(m.ngeom)]
        self.seen = [self.built.seen.get(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)) for g in range(m.ngeom)]
        self.free, self.mocap = {}, {}
        for name, o in self.built.obstacles.items():
            body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"obstacle_{name}")
            if o.fixed:
                self.mocap[name] = m.body_mocapid[body]
            else:
                j = m.body_jntadr[body]
                self.free[name] = (m.jnt_qposadr[j], m.jnt_dofadr[j])
        if keep is not None:
            self.data.qpos[self.qadr] = keep[0]
            for name, (qpos, qvel) in keep[1].items():
                if name in self.free:
                    a, v = self.free[name]
                    self.data.qpos[a:a + 7], self.data.qvel[v:v + 6] = qpos, qvel
        mujoco.mj_forward(m, self.data)
        self.cameras = Cameras(m)
        self.revision += 1
        self.layout = Layout(self.revision, m, self.built, self.scene.to_dict(), twin_model.fingerprint(self.built),
                             self.seen, self.qadr, dict(self.free), dict(self.mocap))

    def close(self):
        if self.cameras is not None:
            self.cameras.close()

    def snapshot(self):
        """The twin's state now (a Snapshot: copies, with this build's Layout), for another thread to render or send
        while this one carries on."""
        return Snapshot(self.layout, self.t if self.t is not None else 0.0, self.data.qpos.copy(),
                        self.data.mocap_pos.copy(), self.data.mocap_quat.copy())

    # ---- the arm --------------------------------------------------------------------------------------------------
    def set_joints(self, q, qd=None):
        """Put the arm at joints q (rad), moving at qd (rad/s, default still), and update everything that follows."""
        import mujoco
        self.data.qpos[self.qadr] = q
        self.data.qvel[self.vadr] = 0.0 if qd is None else qd
        mujoco.mj_forward(self.model, self.data)

    def follow(self, t, q):
        """The arm where the controller has it at time t (s, its clock): joints q (rad). Free obstacles are stepped
        from the last call with the arm moving steadily between the two, so it pushes what it meets."""
        import mujoco
        q = np.asarray(q, dtype=float)
        if self.t is None or t <= self.t:
            self.set_joints(q)
        elif not self.free:
            self.set_joints(q, (q - self.q) / (t - self.t))
        else:
            dt = self.model.opt.timestep
            span = min(t - self.t, MAX_CATCH_UP_S)
            steps = max(1, int(math.ceil(span / dt - 1e-9)))
            qd = (q - self.q) / (t - self.t)
            for k in range(1, steps + 1):
                self.data.qpos[self.qadr] = q - qd * (span - k * span / steps)
                self.data.qvel[self.vadr] = qd
                mujoco.mj_step(self.model, self.data)
            self.set_joints(q, qd)
        self.t, self.q = t, q

    def flange(self):
        """base <- flange (4x4, metres)."""
        T = np.eye(4)
        T[:3, :3] = self.data.xmat[self.flange_body].reshape(3, 3)
        T[:3, 3] = self.data.xpos[self.flange_body]
        return T

    def fk(self, q):
        """base <- flange for joints q (rad), by the twin's model; kinematics only (contacts and cameras aren't
        updated: call set_joints or follow for those)."""
        import mujoco
        self.data.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, self.data)
        return self.flange()

    def tcp(self):
        """base <- tool point of the cell's tool in use (the flange without a cell)."""
        return self.flange() @ self.built.tool

    def link_poses(self):
        """base <- each moving link (4x4, metres), J1's link first."""
        import mujoco
        out = []
        for name in twin_model.ARM_LINKS:
            b = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = self.data.xmat[b].reshape(3, 3), self.data.xpos[b]
            out.append(T)
        return out

    # ---- obstacles ------------------------------------------------------------------------------------------------
    def add_obstacle(self, obstacle):
        """Place an Obstacle (rebuilds the model; the arm and free obstacles keep their state)."""
        if any(o.name == obstacle.name for o in self.scene.obstacles):
            raise twin_model.SceneError(f"There's already an obstacle called {obstacle.name!r}.")
        self.scene.obstacles.append(obstacle.check())
        try:
            self._build()
        except Exception:
            self.scene.obstacles.pop()
            self._build()
            raise

    def remove_obstacle(self, name):
        self._obstacle(name)
        self.scene.obstacles = [o for o in self.scene.obstacles if o.name != name]
        self._build()

    def move_obstacle(self, name, pose, frame=None):
        """Move an obstacle to `pose` ({"xyz": mm, "rpy": deg}) in `frame` (default: its own). A free one stops."""
        import mujoco
        o = self._obstacle(name)
        twin_model.check_pose(pose, f"obstacle {name!r}")
        frame = frame or o.frame
        base = np.eye(4) if frame == "base" else twin_model.matrix(self.built.cell.frame_matrix(frame)) \
            if self.built.cell is not None else None
        if base is None:
            raise twin_model.SceneError(f"Frame {frame!r}: the scene has no cell.")
        T = base @ twin_model.matrix(repo.spine("kinematics").pose_to_matrix(pose))
        o.pose, o.frame = pose, frame
        if o.fixed:
            k = self.mocap[name]
            self.data.mocap_pos[k], self.data.mocap_quat[k] = T[:3, 3], twin_model.quat(T)
        else:
            a, v = self.free[name]
            self.data.qpos[a:a + 3], self.data.qpos[a + 3:a + 7] = T[:3, 3], twin_model.quat(T)
            self.data.qvel[v:v + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def obstacle_pose(self, name):
        """base <- obstacle (4x4, metres), where it is now."""
        import mujoco
        self._obstacle(name)
        b = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"obstacle_{name}")
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.data.xmat[b].reshape(3, 3), self.data.xpos[b]
        return T

    def _obstacle(self, name):
        for o in self.scene.obstacles:
            if o.name == name:
                return o
        raise twin_model.SceneError(f"There's no obstacle called {name!r}.")

    # ---- contacts and clearances ----------------------------------------------------------------------------------
    def contacts(self, include_world=False):
        """What touches what now: [{"a", "b" (labels), "kinds", "depth_mm" (penetration), "at_mm" (base frame)}],
        the arm or tool first. Free obstacles resting on the world are left out unless include_world."""
        out = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            a, b = self.geoms[c.geom1] or ("?", "?"), self.geoms[c.geom2] or ("?", "?")
            if b[0] in ("arm", "tool") and a[0] not in ("arm", "tool"):
                a, b = b, a
            if not include_world and a[0] not in ("arm", "tool"):
                continue
            out.append({"a": a[1], "b": b[1], "kinds": [a[0], b[0]], "depth_mm": round(-c.dist * 1000.0, 3),
                        "at_mm": [round(v * 1000.0, 2) for v in c.pos]})
        return out

    def zone_clearances(self):
        """For each keep-out zone, the arm or tool part nearest to it: {zone: (gap mm, part)}; a negative gap is
        inside. The spine keeps 10 mm (utra_model.MARGIN_M) from every zone; the twin shows what happened."""
        import mujoco
        out = {}
        zones = [g for g, info in enumerate(self.geoms) if info and info[0] == "zone"]
        parts = [g for g, info in enumerate(self.geoms) if info and info[0] in ("arm", "tool")]
        fromto = np.zeros(6)
        for z in zones:
            best = (math.inf, None)
            for p in parts:
                d = mujoco.mj_geomDistance(self.model, self.data, p, z, 1.0, fromto)
                if d < best[0]:
                    best = (d, self.geoms[p][1])
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, z)[len("zone_"):]
            out[name] = (round(best[0] * 1000.0, 3), best[1])
        return out

    # ---- cameras --------------------------------------------------------------------------------------------------
    def camera(self, name):
        try:
            return self.built.cameras[name]
        except KeyError:
            raise twin_model.SceneError(f"There's no camera called {name!r}.") from None

    def render(self, name, segmentation=True):
        """A Frame from camera `name`, as the scene is now."""
        return frame_of(self.cameras, self.data, self.camera(name), self.seen, self.t if self.t is not None else 0.0,
                        self.data.qpos[self.qadr], segmentation)

    def add_camera(self, spec):
        if any(c.name == spec.name for c in self.scene.cameras):
            raise twin_model.SceneError(f"There's already a camera called {spec.name!r}.")
        self.scene.cameras.append(spec.check())
        try:
            self._build()
        except Exception:
            self.scene.cameras.pop()
            self._build()
            raise

    def update_camera(self, name, **changes):
        """Change a camera's pose, mount or intrinsics (rebuilds the model)."""
        old = self.camera(name)
        new = dataclasses.replace(old, **changes)
        if "width" in changes or "height" in changes:
            new.cx = changes.get("cx", (new.width - 1) / 2.0)
            new.cy = changes.get("cy", (new.height - 1) / 2.0)
        new.check()
        self.scene.cameras = [new if c.name == name else c for c in self.scene.cameras]
        try:
            self._build()
        except Exception:
            self.scene.cameras = [old if c.name == name else c for c in self.scene.cameras]
            self._build()
            raise

    def remove_camera(self, name):
        self.camera(name)
        self.scene.cameras = [c for c in self.scene.cameras if c.name != name]
        self._build()
