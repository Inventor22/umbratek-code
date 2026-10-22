"""The twin: a simulated UTRA 6-1000 on its cart in the cell, in MuJoCo, that accepts the same commands as the real arm.

Its model is generated from the repo's own sources (model.py), its joints follow tools/utra_sim.py, the controller's
stand-in fitted to the real arm (coupling.py), and it renders RGB-D cameras with known intrinsics and true poses
(cameras.py). brain/README.md says how to run it.
"""
import os
import sys

# Headless Linux (WSL2 without a display, CI): render through EGL unless told otherwise. Must be set before MuJoCo loads.
if sys.platform.startswith("linux") and "MUJOCO_GL" not in os.environ and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    os.environ["MUJOCO_GL"] = "egl"

from .cameras import Frame  # noqa: E402
from .coupling import Controller, Host  # noqa: E402
from .model import CameraSpec, Obstacle, Scene, SceneError, build  # noqa: E402
from .twin import Twin  # noqa: E402

__all__ = ["CameraSpec", "Controller", "Frame", "Host", "Obstacle", "Scene", "SceneError", "Twin", "build"]
