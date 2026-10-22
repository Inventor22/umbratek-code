"""Simulated RGB-D cameras: colour, depth and segmentation from the twin, with the camera's intrinsics and its true
pose, for perception to be built against before a real camera arrives.

Conventions, as a depth camera's driver and most perception code have them:
- the camera frame is OpenCV's: x right, y down, z along the view; T_base_camera is base <- camera (4x4, metres);
- K is the pinhole matrix in pixels, pixel centres at whole numbers; there is no distortion;
- depth is the distance along the camera's z axis (not along the ray), in metres, aligned pixel for pixel with the
  colour image; 0 means no data (nothing there, or outside the camera's near..far range). Saved, it's a 16-bit PNG in
  millimetres (depth_scale 0.001), as a RealSense writes it;
- segmentation holds each pixel's MuJoCo geom id (-1 for none); Frame.labels names what each id is.
What's modelled is an ideal camera: no noise, no holes at edges, no exposure. Noise belongs in perception's tests.
"""
from __future__ import annotations

import dataclasses
import io
import json
import os
import struct
import zlib

import numpy as np

from .model import CV_TO_MJ


@dataclasses.dataclass
class Frame:
    camera: str
    time: float  # s, on the controller's clock
    rgb: np.ndarray  # (H, W, 3) uint8
    depth: np.ndarray  # (H, W) float32, metres along the view; 0 = no data
    segmentation: np.ndarray | None  # (H, W) int32 geom ids, -1 = none
    K: np.ndarray  # (3, 3)
    T_base_camera: np.ndarray  # (4, 4), metres, OpenCV axes
    joints: list  # the arm's joints (deg) when the frame was taken
    near: float
    far: float
    labels: dict = dataclasses.field(default_factory=dict)  # geom id -> "kind: label"

    @property
    def width(self):
        return self.rgb.shape[1]

    @property
    def height(self):
        return self.rgb.shape[0]

    def points(self, stride=1):
        """The depth image back-projected into the base frame: (N, 3) metres, one point per valid pixel."""
        v, u = np.mgrid[0:self.height:stride, 0:self.width:stride]
        z = self.depth[::stride, ::stride]
        ok = z > 0
        u, v, z = u[ok].astype(float), v[ok].astype(float), z[ok].astype(float)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        cam = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])
        return cam @ self.T_base_camera[:3, :3].T + self.T_base_camera[:3, 3]

    def project(self, points):
        """Pixels (u, v) and depths z of base-frame points (N, 3), through K and the true pose."""
        T = np.linalg.inv(self.T_base_camera)
        cam = np.asarray(points, dtype=float) @ T[:3, :3].T + T[:3, 3]
        z = cam[:, 2]
        uv = cam[:, :2] / z[:, None] * [self.K[0, 0], self.K[1, 1]] + [self.K[0, 2], self.K[1, 2]]
        return uv, z

    def meta(self):
        return {"camera": self.camera, "time": self.time, "width": self.width, "height": self.height,
                "K": self.K.tolist(), "distortion": [0.0] * 5, "T_base_camera": self.T_base_camera.tolist(),
                "depth_scale": 0.001, "near": self.near, "far": self.far, "joints_deg": list(self.joints),
                "labels": {str(k): v for k, v in self.labels.items()},
                "conventions": "camera frame x right, y down, z forward (OpenCV); T_base_camera is base <- camera "
                               "in metres; depth is z in metres (in depth.png, uint16 millimetres); 0 = no data"}

    def save(self, folder, stem=None):
        """rgb.png, depth.png (uint16 mm), segmentation.png (uint16, geom id + 1, 0 = none) and meta.json in `folder`,
        each named <stem>-... when a stem is given. Returns the paths."""
        os.makedirs(folder, exist_ok=True)
        name = (lambda s: os.path.join(folder, f"{stem}-{s}" if stem else s))
        paths = {"rgb": name("rgb.png"), "depth": name("depth.png"), "meta": name("meta.json")}
        write_png(paths["rgb"], self.rgb)
        write_png(paths["depth"], np.round(self.depth * 1000.0).clip(0, 65535).astype(np.uint16))
        if self.segmentation is not None:
            paths["segmentation"] = name("segmentation.png")
            write_png(paths["segmentation"], (self.segmentation + 1).clip(0, 65535).astype(np.uint16))
        with open(paths["meta"], "w", encoding="utf-8") as f:
            json.dump(self.meta(), f, indent=1)
        return paths

    def npz(self):
        """The frame as an .npz file's bytes: rgb, depth, segmentation, K, T_base_camera, and meta (JSON text)."""
        buf = io.BytesIO()
        arrays = {"rgb": self.rgb, "depth": self.depth, "K": self.K, "T_base_camera": self.T_base_camera,
                  "meta": np.array(json.dumps(self.meta()))}
        if self.segmentation is not None:
            arrays["segmentation"] = self.segmentation
        np.savez_compressed(buf, **arrays)
        return buf.getvalue()


class Cameras:
    """Renders the twin's cameras; one MuJoCo renderer (an OpenGL context) per image size, made on first use, in
    the thread that renders."""

    def __init__(self, model):
        self.model = model
        self.renderers = {}

    def renderer(self, width, height):
        import mujoco
        key = (width, height)
        if key not in self.renderers:
            self.renderers[key] = mujoco.Renderer(self.model, height=height, width=width)
        return self.renderers[key]

    def render(self, data, spec, segmentation=True):
        """(rgb, depth, segmentation, T_base_camera) for camera `spec` (a CameraSpec) in the state `data`."""
        import mujoco
        r = self.renderer(spec.width, spec.height)
        r.update_scene(data, camera=spec.name, scene_option=mujoco.MjvOption())  # groups 0 to 2: what's physical
        rgb = r.render().copy()
        r.enable_depth_rendering()
        try:
            depth = r.render().astype(np.float32)
        finally:
            r.disable_depth_rendering()
        depth[(depth < spec.near) | (depth > spec.far)] = 0.0
        seg = None
        if segmentation:
            r.enable_segmentation_rendering()
            try:
                ids = r.render()
            finally:
                r.disable_segmentation_rendering()
            seg = np.where(ids[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM), ids[:, :, 0], -1).astype(np.int32)
        cam = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, spec.name)
        T = np.eye(4)
        T[:3, :3] = data.cam_xmat[cam].reshape(3, 3) @ CV_TO_MJ
        T[:3, 3] = data.cam_xpos[cam]
        return rgb, depth, seg, T

    def close(self):
        for r in self.renderers.values():
            r.close()
        self.renderers = {}


# ---- PNG, with the standard library and NumPy -----------------------------------------------------------------

def png_bytes(image):
    """8-bit RGB (H, W, 3) uint8, or 16-bit grey (H, W) uint16, or 8-bit grey (H, W) uint8, as PNG file bytes."""
    image = np.ascontiguousarray(image)
    if image.ndim == 3 and image.shape[2] == 3 and image.dtype == np.uint8:
        depth_bits, color, raw = 8, 2, image
    elif image.ndim == 2 and image.dtype == np.uint16:
        depth_bits, color, raw = 16, 0, image.astype(">u2")
    elif image.ndim == 2 and image.dtype == np.uint8:
        depth_bits, color, raw = 8, 0, image
    else:
        raise ValueError(f"Unsupported image for PNG: {image.shape} {image.dtype}.")
    h, w = image.shape[:2]
    rows = np.concatenate([np.zeros((h, 1), np.uint8), raw.reshape(h, -1).view(np.uint8)], axis=1)

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, depth_bits, color, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows.tobytes(), 6)) + chunk(b"IEND", b""))


def write_png(path, image):
    with open(path, "wb") as f:
        f.write(png_bytes(image))


def read_png(path):
    """Reads what write_png writes (filter type 0 rows): uint8 RGB or uint8/uint16 grey."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path} isn't a PNG.")
    at, idat, header = 8, b"", None
    while at < len(data):
        n, tag = struct.unpack_from(">I4s", data, at)
        body = data[at + 8:at + 8 + n]
        if tag == b"IHDR":
            header = struct.unpack(">IIBBBBB", body)
        elif tag == b"IDAT":
            idat += body
        at += 12 + n
    w, h, bits, color = header[:4]
    channels = 3 if color == 2 else 1
    row = w * channels * bits // 8
    raw = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(h, row + 1)
    if raw[:, 0].any():
        raise ValueError(f"{path} uses PNG row filters; read it with an image library.")
    pixels = raw[:, 1:]
    if bits == 16:
        return pixels.copy().view(">u2").astype(np.uint16).reshape(h, w)
    return pixels.reshape(h, w, 3) if channels == 3 else pixels.reshape(h, w)
