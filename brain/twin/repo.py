"""Where the repo's own sources are, and the spine's modules the twin is built from.

The twin never copies a dimension, a convention or a motion model: it imports them from the spine (scripts/) and the
controller's stand-in (tools/utra_sim.py), which are standard-library Python and run unchanged in this environment.
"""
import importlib
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(ROOT, "scripts")
TOOLS = os.path.join(ROOT, "tools")
SDK = os.path.join(ROOT, "ut_sdk_python", "utapi")
MODEL_DIR = os.path.join(SCRIPTS, "model")
CAPTURES = os.path.join(ROOT, "data", "captures")


def spine(name):
    """A module from scripts/ or tools/ (utra_model, kinematics, cell, motion_profile, utra_sim, ...)."""
    for path in (SDK, TOOLS, SCRIPTS):  # SCRIPTS ends up first, as the spine's own scripts expect
        if path not in sys.path:
            sys.path.insert(0, path)
    return importlib.import_module(name)
