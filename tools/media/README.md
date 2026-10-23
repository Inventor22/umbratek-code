# The README's pictures

The GIFs and the still in `docs/media/` are filmed from the real software, never the real arm: the jog panel and its
server against the simulator (`tools/utra_sim.py`). The scripts here make them, so they can be made again whenever the
panel changes.

| File | What it shows | Made by |
| ---- | ------------- | ------- |
| `hero.png` | The panel with a pose planned by dragging J2's and J3's rings | `capture_panel.py hero` |
| `jog.gif` | Hold Enable; three clicks of J1's +; J2's and J3's rings dragged to plan a pose; Move | `capture_panel.py jog` |
| `tool.gif` | Tool mode: the gizmo's Z and X arrows dragged, a turn about Z, then Move | `capture_panel.py tool` |
| `teach.gif` | Hold for freedrive; the arm moved by the simulator's "hand" and recorded twice; leave freedrive; Run | `capture_panel.py teach` |

## The environment

Playwright for Python and Pillow, in a venv of their own here (git ignores it), and ffmpeg on the PATH (or named by
`FFMPEG`). Nothing in the spine or the brain uses them. On Windows, from the repo's root:

```
py -3.12 -m venv tools\media\.venv
tools\media\.venv\Scripts\python -m pip install -r tools\media\requirements.txt
tools\media\.venv\Scripts\python -m playwright install chromium
```

## Making them

```
tools\media\.venv\Scripts\python tools\media\capture_panel.py          # hero.png, jog.gif, tool.gif and teach.gif (a few minutes)
tools\media\.venv\Scripts\python tools\media\capture_panel.py tool     # or just the ones named
```

Each prints its pictures' sizes, and flags a GIF over 3 MB. Look at them before committing: a change to the panel's
layout can move what a scene clicks on (the scripts find keys by their ids and labels, and the rings and the gizmo
from the page).

## How they're made

- **The panel's** (`capture_panel.py`): each scene starts its own simulator, on `127.0.0.200`, and its own jog server,
  on port 8960, with a cell folder of its own in a temporary folder and the brain off (`--host` and `--port` pick
  others), so it leaves every other panel, simulator and the repo's `cell/` alone. Playwright drives Chromium, in its
  new headless mode, which draws the 3D view on the GPU (its headless shell falls back to software WebGL, at about
  8 frames a second). The mouse moves, clicks, holds and drags as a person's would; where a joint's ring or a gizmo
  handle is comes from the page (`ringPoint` and `gizmoPoint` in `scripts/web/arm_view.js`), and each drag, with
  Shift held for the coarse snaps (5 degrees, 10 mm), carries on until the page's own readout shows the angle or
  distance asked for. Freedrive's hand is the simulator's control port (`"hand"`, see `control()` in
  `tools/utra_sim.py`), which moves the arm only while the controller is in freedrive. Headless Chromium draws no
  cursor, so the page is given one, with an amber ring while a button is down.
- **Filming**: the DevTools screencast sends the page's frames as it draws them, with their times. `media.py` samples
  them at 12.5 frames a second, adds a caption bar in the panel's colours and type, and has ffmpeg make the GIF with
  one palette for the clip and ordered dither, so what doesn't move costs almost nothing. The desktop layout is
  filmed at 1440 x 820 CSS pixels drawn at two thirds, 960 pixels wide; the tool clip at 960 CSS pixels, one column,
  cropped to the 3D view.
