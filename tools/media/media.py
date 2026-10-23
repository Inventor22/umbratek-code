"""Shared by the capture scripts: timed frames to a looping GIF, with a caption bar in the jog panel's colours.

A clip is a list of frames, each with the time it was shown (s, any clock), and captions by time. encode() samples it
at a steady frame rate (holding each frame until the next), crops and scales it, adds the caption bar and makes the
GIF with ffmpeg's palettegen and paletteuse: one palette for the clip and ordered (Bayer) dither, so what doesn't move
stays the same from frame to frame and costs almost nothing.

Needs Pillow, and ffmpeg on the PATH (or named by FFMPEG). Nothing in the spine or the brain imports this.
"""
import io
import os
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

# The jog panel's colours (DESIGN.md).
DESK = (0x18, 0x19, 0x1B)
WELL = (0x0E, 0x0F, 0x10)
SEAM = (0x33, 0x36, 0x3B)
INK = (0xEC, 0xE8, 0xDF)
INK_2 = (0xAA, 0xA6, 0x9D)
AMBER = (0xF2, 0xAA, 0x3C)

FONT_DIRS = [r"C:\Windows\Fonts", "/mnt/c/Windows/Fonts", "/usr/share/fonts/truetype/dejavu",
             "/usr/share/fonts/TTF", "/Library/Fonts", "/System/Library/Fonts/Supplemental"]
FONTS = {"bold": ["seguisb.ttf", "segoeuisb.ttf", "DejaVuSans-Bold.ttf", "Arial Bold.ttf"],
         "regular": ["segoeui.ttf", "DejaVuSans.ttf", "Arial.ttf"]}
CAPTION_H = 40  # px, the caption bar under the picture
GIF_BUDGET = 3_000_000  # bytes: about the most a README GIF should weigh


def font(kind, size):
    """The panel's typeface (Segoe UI) where it's installed, else DejaVu Sans or Arial, else Pillow's own."""
    for name in FONTS[kind]:
        for folder in FONT_DIRS:
            path = os.path.join(folder, name)
            if os.path.isfile(path):
                return ImageFont.truetype(path, size)
    return ImageFont.load_default(size)


def ffmpeg():
    exe = os.environ.get("FFMPEG") or shutil.which("ffmpeg")
    if not exe:
        raise SystemExit("ffmpeg isn't on the PATH (or named by FFMPEG): it makes the GIFs")
    return exe


class Clip:
    """Frames (PNG bytes, or a path) at times in seconds, and the caption to show from each time on."""

    def __init__(self):
        self.frames = []  # (t, png bytes or path), in time order
        self.captions = []  # (t, text)
        self.stop = None  # when filming stopped: the last frame shows until then

    def add(self, t, image):
        self.frames.append((t, image))

    def caption(self, t, text):
        self.captions.append((t, text))

    @property
    def start(self):
        return self.frames[0][0]

    @property
    def end(self):
        return max(self.frames[-1][0], self.stop or 0.0)

    def caption_at(self, t):
        """The caption shown at time t: the last one given by then (the first, before it)."""
        text = self.captions[0][1] if self.captions else ""
        for at, words in self.captions:
            if at <= t + 1e-6:
                text = words
        return text


def sample(times, ticks):
    """For each tick, the index of the last frame shown at or before it (the first frame before the clip starts)."""
    out, i = [], 0
    for tick in ticks:
        while i + 1 < len(times) and times[i + 1] <= tick:
            i += 1
        out.append(i)
    return out


def load(image):
    if isinstance(image, (bytes, bytearray)):
        return Image.open(io.BytesIO(image)).convert("RGB")
    return Image.open(image).convert("RGB")


def caption_bar(width, text, note):
    """The caption under the picture: Legend Ink on the desk, with a dim note on the right (where it was filmed)."""
    bar = Image.new("RGB", (width, CAPTION_H), DESK)
    draw = ImageDraw.Draw(bar)
    draw.line([(0, 0), (width, 0)], fill=SEAM, width=1)
    big, small = font("bold", 17), font("regular", 13)
    right = width - 14
    if note:
        w = draw.textlength(note, font=small)
        draw.text((right - w, CAPTION_H / 2 + 1), note, fill=INK_2, font=small, anchor="lm")
        right -= w + 18
    if text:
        draw.rectangle([0, 1, 4, CAPTION_H], fill=AMBER)
        draw.text((16, CAPTION_H / 2 + 1), text, fill=INK, font=big, anchor="lm")
    return bar


def compose(image, crop, width, text, note):
    if crop:
        x, y, w, h = crop
        image = image.crop((x, y, x + w, y + h))
    if width and image.width != width:
        image = image.resize((width, round(image.height * width / image.width)), Image.LANCZOS)
    if note is None and text is None:
        return image
    out = Image.new("RGB", (image.width, image.height + CAPTION_H), DESK)
    out.paste(image, (0, 0))
    out.paste(caption_bar(image.width, text, note), (0, image.height))
    return out


def encode(clip, out, fps=12.5, start=None, end=None, speed=1.0, crop=None, width=None, note="", colors=256,
           dither="bayer:bayer_scale=4", stats="full", hold=1.5, keep=None):
    """Writes `clip` to the GIF `out`, sampled at `fps` from `start` to `end` (the clip's times), played `speed` times
    faster than it was filmed, cropped to `crop` (x, y, w, h in the frames' pixels) and scaled to `width`, with the
    caption bar (note=None leaves it off), the last frame held `hold` seconds, looping. `keep` names a folder to keep
    the composed frames in. Returns the GIF's size in bytes."""
    start = clip.start if start is None else start
    end = clip.end if end is None else end
    n = max(1, int((end - start) / speed * fps) + 1)
    ticks = [start + k * speed / fps for k in range(n)] + [end] * int(round(hold * fps))
    times = [t for t, _ in clip.frames]
    picks = sample(times, ticks)
    folder = keep or tempfile.mkdtemp(prefix="media-")
    os.makedirs(folder, exist_ok=True)
    try:
        cache = {}
        for k, (tick, i) in enumerate(zip(ticks, picks)):
            text = clip.caption_at(tick) if note is not None else None
            key = (i, text)
            if key not in cache:
                cache.clear()
                cache[key] = compose(load(clip.frames[i][1]), crop, width, text, note)
            cache[key].save(os.path.join(folder, f"{k:05d}.png"), compress_level=1)
        graph = (f"split[a][b];[a]palettegen=max_colors={colors}:stats_mode={stats}[p];"
                 f"[b][p]paletteuse=dither={dither}:diff_mode=rectangle")
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        subprocess.run([ffmpeg(), "-v", "error", "-y", "-framerate", str(fps), "-i", os.path.join(folder, "%05d.png"),
                        "-vf", graph, "-loop", "0", out], check=True)
    finally:
        if keep is None:
            shutil.rmtree(folder, ignore_errors=True)
    size = os.path.getsize(out)
    flag = "" if size <= GIF_BUDGET else f"  (over the {GIF_BUDGET / 1e6:.0f} MB budget)"
    print(f"{out}: {Image.open(out).size[0]} x {Image.open(out).size[1]}, {len(ticks)} frames at {fps:g} fps, "
          f"{size / 1e6:.2f} MB{flag}")
    return size


def still(image, out, crop=None, width=None):
    """A PNG still from one frame, cropped and scaled, without a caption."""
    picture = compose(load(image), crop, width, None, None)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    picture.save(out, optimize=True)
    print(f"{out}: {picture.width} x {picture.height}, {os.path.getsize(out) / 1e6:.2f} MB")
