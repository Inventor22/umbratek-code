"""Shared helpers for the test suites: the repo's paths, the simulator, and where logs go.

Every suite reads its settings from the environment, which tests/run_all.py sets so suites can run side by side:
    UTRA_TEST_HOST       loopback address the simulator serves on (default 127.0.0.1)
    UTRA_TEST_HTTP_PORT  port for the jog panel under test (default 8799)
    UTRA_TEST_PY         Python that runs the code under test (default: this one)
    UTRA_TEST_LOGS       folder for simulator and server logs (default tests/logs, which git ignores)
    UTRA_TEST_TWIN       1: serve the controller's ports from the twin (python -m brain.twin serve, in the brain's
                         environment: UMBRATEK_BRAIN_PYTHON or brain/.venv) instead of tools/utra_sim.py alone
"""
import os
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
TOOLS = os.path.join(ROOT, "tools")
SIM = os.path.join(TOOLS, "utra_sim.py")
SERVER = os.path.join(SCRIPTS, "jog_server.py")
ARM_CLI = os.path.join(SCRIPTS, "arm.py")
CAPTURE_DIR = os.path.join(ROOT, "data", "captures", "2026-10-21")
FIXTURES = os.path.join(ROOT, "tests", "fixtures")
HOST = os.environ.get("UTRA_TEST_HOST", "127.0.0.1")
HTTP_PORT = int(os.environ.get("UTRA_TEST_HTTP_PORT", "8799"))
PY = os.environ.get("UTRA_TEST_PY") or sys.executable
LOGS = os.environ.get("UTRA_TEST_LOGS") or os.path.join(ROOT, "tests", "logs")
USE_TWIN = os.environ.get("UTRA_TEST_TWIN") == "1"


def log_path(name):
    os.makedirs(LOGS, exist_ok=True)
    return os.path.join(LOGS, name)


def wait_port(port, timeout=8.0, host=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            socket.create_connection((host or HOST, port), timeout=0.2).close()
            return True
        except OSError:
            time.sleep(0.05)
    return False


def brain_python():
    """The brain's environment, where the twin runs: UMBRATEK_BRAIN_PYTHON, brain/.venv, or in WSL2 the venv
    brain/README.md describes (~/.venvs/umbratek-brain). Only this platform's: in WSL2 a Windows venv under /mnt/c
    would run on Windows, beyond WSL2's loopback."""
    if os.name == "nt":
        own = [os.path.join(ROOT, "brain", ".venv", "Scripts", "python.exe")]
    else:
        own = [os.path.join(ROOT, "brain", ".venv", "bin", "python"),
               os.path.join(os.path.expanduser("~"), ".venvs", "umbratek-brain", "bin", "python")]
    candidates = [os.environ.get("UMBRATEK_BRAIN_PYTHON") or ""] + own
    return next((p for p in candidates if p and os.path.isfile(p)), None)


def start_sim(args=(), log_name="sim.log", host=None, twin=None, scene=None):
    """Start tools/utra_sim.py on `host` and wait until it answers; returns (process, log path). With twin=True (or
    UTRA_TEST_TWIN=1), the twin serves the same ports instead (python -m brain.twin serve, which runs the simulator
    in its own process and prints the simulator's log), with `scene` (a scene file) if given."""
    path = log_path(log_name)
    if USE_TWIN if twin is None else twin:
        exe = brain_python()
        if exe is None:
            raise RuntimeError("the twin needs the brain's environment (brain/README.md), and there's none")
        command = [exe, "-m", "brain.twin", "serve", "--host", host or HOST] + (["--scene", scene] if scene else [])
        wait, cwd = 60.0, ROOT  # the twin builds its model and opens its renderer first
    else:
        command, wait, cwd = [sys.executable, SIM, "--host", host or HOST], 8.0, None
    with open(path, "w") as logf:
        proc = subprocess.Popen(command + list(args), stdout=logf, stderr=subprocess.STDOUT, cwd=cwd)
    if not wait_port(502, timeout=wait, host=host):
        proc.kill()
        raise RuntimeError(f"the simulator did not start (see {path})")
    return proc, path


def stop(proc):
    if proc is not None and proc.poll() is None:
        proc.kill()
    if proc is not None:
        proc.wait()
