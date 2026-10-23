"""Films the jog panel for the README: GIFs of jogging the joints, the tool gizmo and teaching in freedrive, and the
hero still. It's the real page (scripts/jog_panel.html) served by the real jog server, driven by Playwright with real
mouse moves, drags and holds, against the simulator (tools/utra_sim.py): never the real arm.

    python tools/media/capture_panel.py [jog tool teach hero ...] [--out docs/media]     (the venv's Python)

(tools/media/README.md says how to make its environment.) It starts its own simulator and jog server, on a loopback
address and a panel port of their own (127.0.0.200 and 8960 by default: --host, --port), with a cell folder of its
own, so it leaves any other panel, simulator and the repo's cell/ alone. Where the 3D view's rings and gizmo handles
are comes from the page (ringPoint and gizmoPoint in scripts/web/arm_view.js); each drag follows the page's own
readout until it shows the angle or distance asked for. Chromium films the page as it's drawn (the DevTools
screencast), a pointer drawn into the page shows where the mouse is, and media.py makes the GIFs.
"""
import argparse
import asyncio
import base64
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import media  # noqa: E402

SIM = os.path.join(ROOT, "tools", "utra_sim.py")
SERVER = os.path.join(ROOT, "scripts", "jog_server.py")
VIEWPORT = {"width": 1440, "height": 820}
SCALE = 2 / 3  # device pixels per CSS pixel: frames 960 px wide, the text drawn at that size rather than shrunk
START = [0, 0, 90, 0, 90, 0]  # the arm's pose when the simulator starts, deg: upper arm up, forearm level
NOTE = "jog panel \u00b7 simulated arm"
SCENES = ("jog", "tool", "teach", "hero")

# A pointer drawn into the page (headless Chromium films no cursor): an arrow that follows the mouse, and an amber
# ring while a button is held. It ignores the mouse itself, so the page sees every event as it would without it.
POINTER = r"""
(() => {
  const install = () => {
    if (document.getElementById("media-pointer")) return;
    const style = document.createElement("style");
    style.textContent = `
      #media-pointer { position: fixed; left: 0; top: 0; z-index: 2147483647; pointer-events: none;
        transform: translate(-200px, -200px); }
      #media-pointer .press { position: absolute; left: -15px; top: -15px; width: 30px; height: 30px;
        border-radius: 50%; border: 2px solid #f2aa3c; background: rgb(242 170 60 / 0.28); opacity: 0;
        transition: opacity 0.08s; }
      #media-pointer.down .press { opacity: 1; }
      #media-pointer svg { position: absolute; left: -2px; top: -1px; filter: drop-shadow(0 1px 2px rgb(0 0 0 / 0.6)); }
      html { scrollbar-width: none; }`;
    document.head.append(style);
    const pointer = document.createElement("div");
    pointer.id = "media-pointer";
    pointer.innerHTML = '<div class="press"></div><svg width="21" height="27" viewBox="0 0 21 27">'
      + '<path d="M1.5 1.5 L1.5 21.5 L6.6 16.8 L10.2 25.2 L13.6 23.7 L10.1 15.5 L17.2 15.5 Z" fill="#ffffff" '
      + 'stroke="#15161a" stroke-width="1.6" stroke-linejoin="round"/></svg>';
    document.documentElement.append(pointer);
    const at = (e) => { pointer.style.transform = `translate(${e.clientX}px, ${e.clientY}px)`; };
    window.addEventListener("pointermove", at, true);
    window.addEventListener("pointerdown", (e) => { at(e); pointer.classList.add("down"); }, true);
    window.addEventListener("pointerup", (e) => { at(e); pointer.classList.remove("down"); }, true);
  };
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", install);
  else install();
})();
"""


def wait_port(host, port, timeout=20.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            socket.create_connection((host, port), timeout=0.3).close()
            return True
        except OSError:
            time.sleep(0.1)
    return False


class Rig:
    """The simulator and a jog server on it, each in a process of its own, with a cell folder of their own."""

    def __init__(self, host, port, logs):
        if not host.startswith("127."):
            raise SystemExit(f"{host} isn't a loopback address: the capture runs against the simulator only")
        for h, p in ((host, 502), ("127.0.0.1", port)):
            deadline = time.monotonic() + 5.0  # one scene's processes may take a moment to let go of them
            while wait_port(h, p, timeout=0.3):
                if time.monotonic() > deadline:
                    raise SystemExit(f"{h}:{p} is already in use: pick another --host or --port")
                time.sleep(0.3)
        self.host, self.port, self.procs = host, port, []
        self.cell = tempfile.mkdtemp(prefix="media-cell-")
        self.base = f"http://127.0.0.1:{port}"
        try:
            self.start([sys.executable, SIM, "--host", host, "--joints", ",".join(str(v) for v in START)],
                       os.path.join(logs, "sim.log"))
            assert wait_port(host, 502), "the simulator didn't start"
            self.start([sys.executable, SERVER, host, "--port", str(port), "--cell", self.cell, "--brain", "off"],
                       os.path.join(logs, "server.log"))
            assert wait_port("127.0.0.1", port, timeout=60), "the jog server didn't start"
            self.control = socket.create_connection((host, 30099), timeout=5)
            self.buffer = b""
        except BaseException:
            self.close()
            raise

    def start(self, command, log):
        with open(log, "w") as out:
            self.procs.append(subprocess.Popen(command, cwd=ROOT, stdout=out, stderr=subprocess.STDOUT))

    def ctl(self, **request):
        """The simulator's control port (tools/utra_sim.py, control()): "hand" moves the arm as a hand would."""
        self.control.sendall(json.dumps(request).encode() + b"\n")
        while b"\n" not in self.buffer:
            self.buffer += self.control.recv(65536)
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def status(self):
        with urllib.request.urlopen(self.base + "/api/status", timeout=10) as r:
            return json.loads(r.read())

    def close(self):
        for proc in self.procs:
            proc.kill()
            proc.wait()
        shutil.rmtree(self.cell, ignore_errors=True)


def settled(s):
    return bool(s.get("live")) and s.get("state") in (0, 2) and not s.get("queued")


def moving(s):
    return s.get("state") in (1, 3) or bool(s.get("queued"))


class Film:
    """The page and the screencast of it: frames as Chromium draws them, with their times, and the captions."""

    def __init__(self, page, cdp):
        self.page, self.cdp = page, cdp
        self.clip = None
        self.cdp.on("Page.screencastFrame", lambda p: asyncio.ensure_future(self.frame(p)))

    async def frame(self, params):
        if self.clip is not None:
            self.clip.add(params["metadata"]["timestamp"], base64.b64decode(params["data"]))
        try:
            await self.cdp.send("Page.screencastFrameAck", {"sessionId": params["sessionId"]})
        except Exception:
            pass  # the screencast stopped meanwhile

    async def repaint(self):
        """A still page sends no frames; this makes it draw one (the pointer, nudged by nothing)."""
        await self.page.evaluate("() => { const p = document.getElementById('media-pointer'); p.style.opacity = '0.999';"
                                 " requestAnimationFrame(() => requestAnimationFrame(() => { p.style.opacity = ''; })); }")

    async def roll(self):
        self.clip = media.Clip()
        # Frames at the page's device pixels (the screencast would otherwise scale them up to CSS pixels).
        size = await self.page.evaluate("() => [innerWidth * devicePixelRatio, innerHeight * devicePixelRatio]")
        await self.cdp.send("Page.startScreencast", {"format": "png", "everyNthFrame": 1, "maxWidth": round(size[0]),
                                                     "maxHeight": round(size[1])})
        await self.repaint()
        await self.page.wait_for_timeout(300)

    async def cut(self, tail=1.0):
        await self.page.wait_for_timeout(tail * 1000)
        await self.repaint()
        await self.page.wait_for_timeout(200)
        clip, self.clip = self.clip, None
        clip.stop = time.time()
        await self.cdp.send("Page.stopScreencast")
        return clip

    def say(self, text):
        self.clip.caption(time.time(), text)


class Hand:
    """The mouse, moved the way a person moves it: gliding to a point, clicking, holding, dragging."""

    def __init__(self, page, viewport):
        self.page, self.x, self.y = page, viewport["width"] * 0.75, viewport["height"] * 0.7

    async def glide(self, x, y, seconds=0.45):
        x0, y0 = self.x, self.y
        steps = max(4, int(seconds * 22))
        t0 = time.monotonic()
        for k in range(1, steps + 1):
            u = k / steps
            e = u * u * (3 - 2 * u)
            await self.page.mouse.move(x0 + (x - x0) * e, y0 + (y - y0) * e)
            lag = t0 + seconds * u - time.monotonic()
            if lag > 0:
                await asyncio.sleep(lag)
        self.x, self.y = x, y

    async def centre(self, selector):
        box = await self.page.locator(selector).bounding_box()
        return box["x"] + box["width"] / 2, box["y"] + box["height"] / 2

    async def click(self, selector, pause=0.12):
        await self.glide(*await self.centre(selector))
        await self.page.mouse.down()
        await asyncio.sleep(pause)
        await self.page.mouse.up()

    async def hold(self, selector, seconds):
        await self.glide(*await self.centre(selector))
        await self.page.mouse.down()
        await asyncio.sleep(seconds)
        await self.page.mouse.up()

    async def reading(self):
        """The 3D view's readout during a drag: the joint's angle, the tool's mm along an axis or its turn in deg."""
        text = await self.page.evaluate("() => { const t = document.querySelector('#screenTag .to'); "
                                        "return t && !document.getElementById('screenTag').hidden ? t.textContent : null; }")
        if not text:
            return None
        m = re.search(r"[-+\u2212]?\d+(\.\d+)?", text.replace("\u2212", "-"))
        return float(m.group(0)) if m else None

    async def follow(self, point, goal, seconds, relative=False):
        """Holds the button (and Shift, so the page snaps to its coarse steps: 5 deg, 10 mm) and moves along
        point(u), u from 0 (where the mouse is) either way, until the readout shows `goal` (or `goal` more than at
        the start, if relative), at a pace that takes about `seconds`; returns the readout at release. Which way u
        takes the readout shows in the first moves; the last part goes in fine steps, backing up if it passes."""
        await self.page.keyboard.down("Shift")
        await self.page.mouse.down()
        await asyncio.sleep(0.12)
        first = await self.reading()
        if relative:
            goal += first or 0.0
        u, du, sign, start, value, known = 0.0, 0.01, 1, first, first, False
        t0 = time.monotonic()
        for _ in range(800):
            done = None
            if start is not None and value is not None:
                if abs(goal - value) < 1e-6:
                    break
                done = (value - start) / (goal - start) if goal != start else 1.0
            if done is not None and done > 1.0:
                u -= sign * 0.002
            else:
                u += sign * du
            x, y = point(u)
            await self.page.mouse.move(x, y)
            self.x, self.y = x, y
            value = await self.reading()
            if value is None:
                continue
            if start is None:
                start = value
                continue
            if goal == start:
                break
            done = (value - start) / (goal - start)
            if os.environ.get("MEDIA_DEBUG") == "2":
                print(f"     u {u:+.3f} du {du:.3f} sign {sign:+d} at ({x:.0f}, {y:.0f}) reads {value} done {done:.2f}")
            if not known and value != start:
                known = True
                if done < 0:
                    sign = -1  # the readout went the wrong way: go the other way round
            wanted = min(1.0, (time.monotonic() - t0) / seconds)
            if done > 0.8:
                du = 0.002
            elif done < wanted:
                du = min(du * 1.35, 0.1)
            else:
                du = max(du * 0.7, 0.002)
            lag = t0 + seconds * max(done, 0.0) - time.monotonic()
            if lag > 0:
                await asyncio.sleep(min(lag, 0.05))
        await asyncio.sleep(0.2)
        await self.page.mouse.up()
        await self.page.keyboard.up("Shift")
        if os.environ.get("MEDIA_DEBUG"):
            print(f"   drag to {goal}: released at {value} (from {start}, {time.monotonic() - t0:.1f} s)")
        return value

    async def turn(self, p, goal, seconds):
        """Drags a ring from its point p (x, y and its centre cx, cy) along the ring's tangent there until the
        readout shows `goal`. A ring that faces the view turns with the pointer's angle around its centre; one seen
        side-on, by the distance along that tangent: a straight line serves both."""
        await self.glide(p["x"], p["y"])
        rx, ry = p["x"] - p["cx"], p["y"] - p["cy"]
        n = math.hypot(rx, ry) or 1.0
        tx, ty = -ry / n, rx / n
        return await self.follow(lambda u: (p["x"] + tx * u * 220, p["y"] + ty * u * 220), goal, seconds)

    async def turn_ring(self, joint, goal, seconds=1.4):
        """Drags joint `joint`'s ring (0 = J1) until the readout shows the joint at `goal` degrees."""
        return await self.turn(await self.page.evaluate("j => jogPanel.view().ringPoint(j)", joint), goal, seconds)

    async def slide_tool(self, axis, delta, seconds=1.4):
        """Drags the gizmo's arrow along `axis` (0, 1, 2: X, Y, Z of the base) until it shows the tool `delta` mm
        from where it was."""
        p = await self.page.evaluate("a => jogPanel.view().gizmoPoint('move', a)", axis)
        await self.glide(p["x"], p["y"])
        dx, dy = p["tip"]["x"] - p["x"], p["tip"]["y"] - p["y"]
        n = math.hypot(dx, dy) or 1.0
        return await self.follow(lambda u: (p["x"] + dx / n * u * 150, p["y"] + dy / n * u * 150), delta, seconds,
                                 relative=True)

    async def turn_tool(self, axis, goal, seconds=1.4):
        """Drags the gizmo's ring about `axis` until it shows a turn of `goal` degrees."""
        return await self.turn(await self.page.evaluate("a => jogPanel.view().gizmoPoint('turn', a)", axis), goal,
                               seconds)


async def set_fader(page, key, value):
    ok = await page.evaluate("""([key, want]) => {
        const input = document.getElementById(key + "Fader");
        for (let i = 0; i <= Number(input.max); i += 1) {
          input.value = String(i);
          input.dispatchEvent(new Event("input"));
          if (Math.abs(jogPanel.settings()[key] - want) < 1e-9) return true;
        }
        return false;
      }""", [key, value])
    if not ok:
        raise RuntimeError(f"the {key} fader has no {value}")


async def enable(page, rig, hand):
    await hand.hold("#enableKey", 0.95)
    await wait_arm(page, rig, lambda s: s.get("enabled") and settled(s), 20)
    await page.wait_for_timeout(400)


async def wait_arm(page, rig, pred=settled, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred(rig.status()):
            return
        await page.wait_for_timeout(60)
    raise RuntimeError(f"timed out waiting on the arm: {rig.status()}")


# ---- the scenes -----------------------------------------------------------------------------------------------
# Each films the page at a size of its own: (viewport in CSS pixels, device pixels per CSS pixel).
DESK_VIEW = (VIEWPORT, SCALE)  # the panel as it is on a desktop: the 3D view beside the joints, 960 px wide
CLOSE_VIEW = ({"width": 960, "height": 1250}, 1.0)  # narrower, one column: the 3D view across the top, life size
LAYOUTS = {"jog": DESK_VIEW, "tool": CLOSE_VIEW, "teach": DESK_VIEW, "hero": (VIEWPORT, 1.0)}


async def zoom(page, wheel, pan=0):
    """Turns the mouse wheel over the 3D view (negative: closer) and pans it (right-drag, px down), as anyone can,
    to frame the arm for the moves to come."""
    box = await page.locator("#screen").bounding_box()
    x, y = box["x"] + box["width"] * 0.85, box["y"] + box["height"] * 0.35
    await page.mouse.move(x, y)
    await page.mouse.wheel(0, wheel)
    await page.wait_for_timeout(300)
    if pan:
        await page.mouse.down(button="right")
        await page.mouse.move(x, y + pan, steps=8)
        await page.mouse.up(button="right")
    await page.wait_for_timeout(500)


async def scene_jog(page, rig, film, hand, out):
    await set_fader(page, "jointStep", 10)
    await zoom(page, -250, 150)
    await film.roll()
    film.say("Hold Enable: the motors come on")
    await page.wait_for_timeout(500)
    await enable(page, rig, hand)
    film.say("Click + or \u2212: the joint turns one exact step, every click")
    for _ in range(3):
        await hand.click('button[aria-label="Joint 1 plus one step"]')
        await page.wait_for_timeout(250)
    await wait_arm(page, rig)
    film.say("Drag a joint's ring to plan a pose: an amber shadow shows it")
    await hand.turn_ring(1, -35.0)
    await page.wait_for_timeout(300)
    await hand.turn_ring(2, 125.0)
    await page.wait_for_timeout(800)
    film.say("Move sends the plan, all joints together, checked for collisions first")
    await hand.click("#planMove")
    await wait_arm(page, rig, moving, 5)
    await wait_arm(page, rig)
    clip = await film.cut(0.8)
    return [media.encode(clip, os.path.join(out, "jog.gif"), fps=12.5, note=NOTE)]


async def scene_tool(page, rig, film, hand, out):
    await enable(page, rig, hand)
    await page.click("#modeTool")  # framed with the gizmo in view, then back to Joints for the film to start
    await page.wait_for_timeout(600)
    await zoom(page, -200, 60)
    await page.click("#modeJoints")
    box = await page.locator(".monitor").bounding_box()
    crop = (0, 0, 960, round(box["y"] + box["height"] + 10))
    await film.roll()
    film.say("Press Tool: a gizmo at the flange, X red, Y green, Z blue")
    await page.wait_for_timeout(400)
    await hand.click("#modeTool")
    await page.wait_for_timeout(1000)
    film.say("Drag an arrow to slide the tool: the joints follow")
    await hand.slide_tool(2, 120)
    await page.wait_for_timeout(400)
    await hand.slide_tool(0, -100)
    await page.wait_for_timeout(400)
    film.say("Drag a ring to turn the tool about that axis")
    await hand.turn_tool(2, 30.0)
    await page.wait_for_timeout(800)
    film.say("Move takes the arm to the planned pose")
    await hand.click("#planMove")
    await wait_arm(page, rig, moving, 5)
    await wait_arm(page, rig)
    clip = await film.cut(0.8)
    return [media.encode(clip, os.path.join(out, "tool.gif"), fps=12.5, crop=crop, note=NOTE)]


async def scene_teach(page, rig, film, hand, out):
    await enable(page, rig, hand)
    await zoom(page, -250, 150)
    await page.fill("#programName", "teach")
    await page.click("#programCreate")
    await page.wait_for_timeout(500)
    await page.click('#recordAdd button[data-value="joint"]')
    await page.fill("#recordName", "p1")
    # The program and Teach beside the 3D view (which stays put as the page scrolls), where the joints were.
    await page.evaluate("() => window.scrollTo(0, document.querySelector('.program').getBoundingClientRect().top"
                        " - document.querySelector('.joints').getBoundingClientRect().top)")
    await page.wait_for_timeout(600)
    await film.roll()
    film.say("Hold for freedrive: the arm follows your hand")
    await page.wait_for_timeout(400)
    await hand.hold("#freedriveKey", 0.95)
    await wait_arm(page, rig, lambda s: s.get("freedrive"), 10)
    for pose in ([60, -25, 110, 0, 90, 0], [-10, -40, 130, 0, 90, 0]):
        film.say("Guide it by hand (here, the simulator's)")
        await hand.glide(*await hand.centre("#recordKey"), seconds=0.6)
        assert rig.ctl(cmd="hand", joints=pose, seconds=1.8)["ok"]
        await page.wait_for_timeout(2100)
        film.say("Record here: the pose joins the program as a move")
        await hand.click("#recordKey")
        await page.wait_for_timeout(1000)
    film.say("Leave freedrive and press Run: it plays back what you taught")
    await hand.click("#freedriveKey")
    await wait_arm(page, rig, lambda s: not s.get("freedrive") and s.get("enabled") and settled(s), 15)
    await page.wait_for_timeout(300)
    await hand.click("#runStart")
    await wait_arm(page, rig, lambda s: s.get("run", {}).get("state") in ("running", "done"), 10)
    await wait_arm(page, rig, lambda s: s.get("run", {}).get("state") == "done", 60)
    clip = await film.cut(1.0)
    return [media.encode(clip, os.path.join(out, "teach.gif"), fps=12.5, note=NOTE)]


async def scene_hero(page, rig, film, hand, out):
    await set_fader(page, "jointStep", 10)
    await zoom(page, 150)
    await enable(page, rig, hand)
    for _ in range(3):
        await hand.click('button[aria-label="Joint 1 plus one step"]')
        await page.wait_for_timeout(250)
    await wait_arm(page, rig)
    await hand.turn_ring(1, -35.0)
    await hand.turn_ring(2, 125.0)
    await hand.glide(VIEWPORT["width"] * 0.5, VIEWPORT["height"] - 20)
    await page.evaluate("() => { document.getElementById('media-pointer').style.display = 'none'; }")
    await page.wait_for_timeout(600)
    path = os.path.join(out, "hero.png")
    media.still(await page.screenshot(), path)
    return [os.path.getsize(path)]


async def film_scene(name, browser, rig, out):
    viewport, scale = LAYOUTS[name]
    context = await browser.new_context(viewport=viewport, device_scale_factor=scale)
    await context.add_init_script(POINTER)
    page = await context.new_page()
    await page.goto(rig.base + "/")
    await page.wait_for_function("() => window.jogPanel && jogPanel.view() && jogPanel.status() "
                                 "&& jogPanel.status().live", timeout=30000)
    await page.wait_for_timeout(800)
    cdp = await context.new_cdp_session(page)
    try:
        scene = globals()[f"scene_{name}"]
        return await scene(page, rig, Film(page, cdp), Hand(page, viewport), out)
    finally:
        await context.close()

async def main(args):
    from playwright.async_api import async_playwright
    logs = tempfile.mkdtemp(prefix="media-logs-")
    async with async_playwright() as p:
        # The full Chromium in its new headless mode draws WebGL on the GPU; the headless shell would use SwiftShader.
        browser = await p.chromium.launch(headless=True, channel="chromium")
        try:
            for name in args.scenes:
                rig = Rig(args.host, args.port, logs)  # a fresh arm, at START, for every scene
                try:
                    print(f"== {name}")
                    await film_scene(name, browser, rig, args.out)
                finally:
                    rig.close()
        finally:
            await browser.close()
    shutil.rmtree(logs, ignore_errors=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("scenes", nargs="*", help=f"what to film (default all: {', '.join(SCENES)})")
    parser.add_argument("--host", default="127.0.0.200", help="loopback address for the simulator (127.0.0.200)")
    parser.add_argument("--port", type=int, default=8960, help="the jog panel's port (8960)")
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "media"), help="folder for the GIFs (docs/media)")
    a = parser.parse_args()
    unknown = [s for s in a.scenes if s not in SCENES]
    if unknown:
        parser.error(f"no scene {', '.join(unknown)}: pick from {', '.join(SCENES)}")
    a.scenes = a.scenes or list(SCENES)
    asyncio.run(main(a))
