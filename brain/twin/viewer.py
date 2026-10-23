"""The twin's viewer: MuJoCo's own viewer on a running twin, from a process of its own, read-only. Run it on Windows,
natively, while the twin runs in WSL2 (brain/README.md, "Try it"):

    python -m brain.twin view --follow http://127.0.0.1:30100
    python -m brain.twin view --follow http://127.0.0.1:30100 --check 5     # the same for 5 s, without a window

It shows the model the twin built from the repo's sources (GET /model: the generated MJCF and its meshes, checked
against the twin's fingerprint of them), and the twin's state as the twin streams it (GET /pose/stream: the arm's
joints and where every obstacle is, every step of the twin's loop). When the twin rebuilds its model (an obstacle or a
camera added or removed), the viewer takes the new one into the same window, keeping the view. It only ever reads:
nothing in it can move the arm or change the twin. Closing its window ends the viewer, never the twin; if the twin
stops, the window stays, says so, and follows the twin again when it's back. --check follows it without a window
for a few seconds and says what arrived: whether the twin can be followed from here.
"""
from __future__ import annotations

import io
import json
import threading
import time
import types
import urllib.parse
import urllib.request
import zipfile

import numpy as np

from . import model as twin_model

URL = "http://127.0.0.1:30100"
RETRY_S = 0.5


def say(text):
    print(text, flush=True)


class Mirror:
    """A model the twin built (GET /model's zip), compiled here; its state is only ever set from the twin's samples."""

    def __init__(self, zipped):
        import mujoco
        with zipfile.ZipFile(io.BytesIO(zipped)) as z:
            layout = json.loads(z.read("layout.json"))
            xml = z.read("twin.xml").decode()
            assets = {n[len("assets/"):]: z.read(n) for n in z.namelist() if n.startswith("assets/")}
        self.revision, self.scene = layout["revision"], layout["scene"]
        self.fingerprint = twin_model.fingerprint(types.SimpleNamespace(xml=xml, assets=assets))
        if self.fingerprint != layout["fingerprint"]:
            raise ValueError("the model received isn't the one the twin built: their fingerprints differ")
        self.model = m = mujoco.MjModel.from_xml_string(xml, assets)
        self.data = mujoco.MjData(m)
        self.qadr = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in twin_model.JOINTS])
        self.free, self.mocap = {}, {}
        for b in range(m.nbody):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if name.startswith("obstacle_"):
                if m.body_mocapid[b] >= 0:
                    self.mocap[name[len("obstacle_"):]] = int(m.body_mocapid[b])
                else:
                    self.free[name[len("obstacle_"):]] = int(m.jnt_qposadr[m.body_jntadr[b]])
        mujoco.mj_forward(m, self.data)

    def show(self, sample):
        """The twin's state in one of its samples (GET /pose): the arm's joints and the obstacles. Nothing is stepped:
        what the twin's physics did is in the sample."""
        import mujoco
        self.data.qpos[self.qadr] = sample["joints"]
        for name, p in sample["obstacles"].items():
            if name in self.mocap:
                k = self.mocap[name]
                self.data.mocap_pos[k], self.data.mocap_quat[k] = p[:3], p[3:]
            elif name in self.free:
                a = self.free[name]
                self.data.qpos[a:a + 7] = p
        self.data.time = sample["time"]
        mujoco.mj_forward(self.model, self.data)


class Follower:
    """A running twin, followed over HTTP: its model on request (model()) and its state as it streams it, read in the
    background (newer()). Every request it makes is a GET (`requests` lists them)."""

    def __init__(self, url=URL, timeout=5.0):
        parts = urllib.parse.urlsplit(url)
        if parts.scheme != "http" or not parts.hostname:
            raise ValueError(f"Follow the twin at an http:// address, like {URL}; got {url!r}.")
        self.url, self.timeout = url.rstrip("/"), timeout
        self.cond = threading.Condition()
        self.sample, self.count = None, 0  # the newest sample, and how many have arrived
        self.connected, self.error, self.closed = False, None, False
        self.requests = []
        self.thread = threading.Thread(target=self._stream, name="twin-follower", daemon=True)
        self.thread.start()

    def _get(self, path):
        self.requests.append(("GET", path))
        return urllib.request.urlopen(urllib.request.Request(self.url + path, method="GET"), timeout=self.timeout)

    def model(self):
        """The twin's model now, as a Mirror (OSError if the twin doesn't answer)."""
        with self._get("/model") as reply:
            return Mirror(reply.read())

    def newer(self, count, timeout):
        """(sample, count): the newest sample once one has arrived after the count-th, or (None, count) after
        `timeout` s."""
        deadline = time.monotonic() + timeout
        with self.cond:
            while self.count <= count and not self.closed:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None, count
                self.cond.wait(left)
            return (self.sample, self.count) if self.count > count else (None, count)

    def close(self):
        self.closed = True
        with self.cond:
            self.cond.notify_all()

    def _stream(self):
        while not self.closed:
            try:
                with self._get("/pose/stream") as reply:
                    for line in reply:
                        if self.closed:
                            return
                        sample = json.loads(line)
                        with self.cond:
                            self.sample, self.connected, self.error = sample, True, None
                            self.count += 1
                            self.cond.notify_all()
                raise OSError("the twin ended the stream")
            except (OSError, ValueError) as e:  # urllib's errors are OSErrors
                with self.cond:
                    self.connected, self.error = False, str(getattr(e, "reason", e))
                    self.cond.notify_all()
                time.sleep(RETRY_S)


class Window:
    """MuJoCo's viewer (its passive mode, in a window of this process), showing a Mirror."""

    def __init__(self, mirror):
        import mujoco.viewer
        self.handle = mujoco.viewer.launch_passive(mirror.model, mirror.data)
        self.text = None

    def running(self):
        return self.handle.is_running()

    def load(self, mirror):
        """Shows another model in the same window, keeping the view (as MuJoCo's viewer reloads a model itself)."""
        sim = self.handle._sim()
        if sim is not None:
            sim.load(mirror.model, mirror.data, "")

    def sync(self):
        self.handle.sync()

    def say(self, text):
        if text != self.text:
            self.text = text
            if text:
                import mujoco
                self.handle.set_texts((None, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT, text, ""))
            else:
                self.handle.clear_texts()

    def close(self):
        self.handle.close()


class Headless:
    """A window that isn't one, for --check and the tests: what follow() shows it is counted, and kept."""

    def __init__(self, mirror):
        self.mirrors, self.mirror, self.syncs, self.text, self.open = [mirror], mirror, 0, None, True

    def running(self):
        return self.open

    def load(self, mirror):
        self.mirrors.append(mirror)
        self.mirror = mirror

    def sync(self):
        self.syncs += 1

    def say(self, text):
        self.text = text

    def close(self):
        self.open = False


def follow(follower, window_type=Window, log=say, until=None, seconds=None):
    """Shows the twin `follower` follows in a window_type window until the window is closed (or `until()` is true, or
    `seconds` have passed since it opened): waits for the twin, then shows each new sample, taking the twin's new
    model when it rebuilds. Returns the window (None if it never opened)."""
    mirror, said = None, None
    while mirror is None:
        try:
            mirror = follower.model()
        except (OSError, ValueError) as e:
            why = str(getattr(e, "reason", e))
            if why != said:
                log(f"view: waiting for the twin at {follower.url} ({why})")
                said = why
            if until is not None and until():
                return None
            time.sleep(RETRY_S)
    log(f"view: following the twin at {follower.url}: revision {mirror.revision}, model {mirror.fingerprint[:12]}; "
        f"read-only. Closing this window leaves the twin running.")
    window = window_type(mirror)
    stop = None if seconds is None else time.monotonic() + seconds
    shown = 0
    try:
        while window.running() and not (until is not None and until()) and not (stop and time.monotonic() > stop):
            sample, count = follower.newer(shown, 0.1)
            window.say(None if follower.connected else f"The twin at {follower.url} isn't answering "
                       f"({follower.error}): its last state is shown. Retrying.")
            if sample is None:
                continue
            shown = count
            if sample["fingerprint"] != mirror.fingerprint:  # the twin rebuilt its model
                try:
                    new = follower.model()
                except (OSError, ValueError) as e:
                    log(f"view: the twin's new model didn't arrive ({e}); trying again")
                    continue
                if new.fingerprint != mirror.fingerprint:
                    window.load(new)
                    mirror = new
                    log(f"view: the twin rebuilt its model: revision {mirror.revision}, model "
                        f"{mirror.fingerprint[:12]}")
                if sample["fingerprint"] != mirror.fingerprint:
                    continue  # rebuilt again meanwhile: the next sample brings it
            mirror.show(sample)
            window.sync()
    finally:
        window.close()
    return window


def check(follower, seconds, log=say, wait_s=30.0):
    """--check: follows the twin for `seconds` without a window and says what arrived. 0 if it was followed."""
    t0 = time.monotonic()
    window = follow(follower, Headless, log, until=lambda: time.monotonic() - t0 > wait_s + seconds, seconds=seconds)
    if window is None or not window.syncs:
        log(f"view: nothing arrived from the twin at {follower.url} in {time.monotonic() - t0:.0f} s"
            + (f" ({follower.error})" if follower.error else ""))
        return 1
    m = window.mirror
    log(f"view: followed the twin at {follower.url} for {seconds:g} s: {window.syncs} states shown "
        f"({window.syncs / seconds:.0f} a second); its model, revision {m.revision} ({m.fingerprint[:12]}): "
        f"{m.model.nbody} bodies, {m.model.ngeom} geoms, obstacles {', '.join(sorted({**m.free, **m.mocap})) or 'none'}"
        f"; {len(window.mirrors) - 1} rebuild(s) taken; requests made: "
        f"{', '.join(sorted({f'{v} {p}' for v, p in follower.requests}))}")
    return 0


def main(url=URL, seconds=None):
    """`python -m brain.twin view`: the window, or with `seconds`, --check."""
    follower = Follower(url)
    try:
        if seconds is not None:
            return check(follower, seconds)
        from .cameras import confine
        confine()  # on Linux (WSLg) only: its rendering then doesn't delay a twin's loop beside it (D91)
        follow(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.close()
    return 0
