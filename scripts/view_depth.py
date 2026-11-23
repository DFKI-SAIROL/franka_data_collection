#!/usr/bin/env python3
"""
View the 16-bit depth videos written by the data collector.

Depth is stored losslessly as uint16 millimetres in FFV1/MKV.  A robot
workspace sits at roughly 0.3-1.5 m, which is under 3% of the uint16
range, so every ordinary player -- and ``cv2.VideoCapture`` in its default
mode -- stretches 0..65535 onto 0..255 and renders the scene as black.
Nothing is wrong with the data; it just needs windowing.

This script reads the file as true 16-bit, windows it to the depth range
actually present, and either shows it in a window or writes an 8-bit
preview you can play anywhere.

Examples
--------
    # window, auto range (needs cv2)
    ./view_depth.py <episode>/videos/depth_zed_left_arm.mkv

    # whole episode directory, colour map, fixed 0.2-1.5 m window
    ./view_depth.py <episode> --cmap turbo --range 200 1500

    # no display available: write a preview mp4 next to the source
    ./view_depth.py <episode> --save preview.mp4

Sentinels
---------
``0`` means "no reading" and ``65535`` means "beyond max range"; both are
excluded from the auto range and drawn as black.
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

INVALID_NEAR = 0
INVALID_FAR = 65535


def probe(path: Path) -> tuple[int, int, float]:
    """Return (width, height, fps) via ffprobe."""
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=width,height,r_frame_rate",
         "-of", "json", str(path)],
        capture_output=True, text=True, check=True,
    )
    s = json.loads(out.stdout)["streams"][0]
    num, _, den = s["r_frame_rate"].partition("/")
    fps = float(num) / float(den or 1)
    return int(s["width"]), int(s["height"]), (fps or 30.0)


def read_frames(path: Path, w: int, h: int):
    """Yield uint16 H×W frames, decoding as true 16-bit greyscale."""
    proc = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", str(path),
         "-f", "rawvideo", "-pix_fmt", "gray16le", "-"],
        stdout=subprocess.PIPE,
        # Callers stop reading early (range sampling, stats), which makes
        # ffmpeg complain about a broken pipe.  That is expected, not a fault.
        stderr=subprocess.DEVNULL,
    )
    nbytes = w * h * 2
    try:
        while True:
            buf = proc.stdout.read(nbytes)
            if len(buf) < nbytes:
                break
            yield np.frombuffer(buf, dtype="<u2").reshape(h, w)
    finally:
        proc.stdout.close()
        if proc.poll() is None:
            proc.kill()
        proc.wait()


def auto_range(path: Path, w: int, h: int, lo_pct: float, hi_pct: float,
               sample: int = 20) -> tuple[float, float]:
    """Percentile window over real measurements, sampled across the clip."""
    vals = []
    for i, frame in enumerate(read_frames(path, w, h)):
        if i % max(1, sample // 4) == 0:
            f = frame.reshape(-1)
            f = f[(f > INVALID_NEAR) & (f < INVALID_FAR)]
            if f.size:
                vals.append(f[::37])          # thin out, keep it cheap
        if len(vals) >= sample:
            break
    if not vals:
        raise SystemExit(f"{path.name}: no valid depth pixels found.")
    allv = np.concatenate(vals)
    return float(np.percentile(allv, lo_pct)), float(np.percentile(allv, hi_pct))


def window(frame: np.ndarray, lo: float, hi: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Map a uint16 mm frame onto 0..255.

    Returns (grey, valid).  Sentinel pixels are set to 0 in *grey*, but the
    mask is returned separately so a colour map can black them out without
    confusing them with the nearest valid depth.
    """
    valid = (frame > INVALID_NEAR) & (frame < INVALID_FAR)
    out = (frame.astype(np.float32) - lo) * (255.0 / max(hi - lo, 1e-6))
    out = np.clip(out, 0, 255).astype(np.uint8)
    out[~valid] = 0
    return out, valid


def turbo_lut() -> np.ndarray:
    """
    256×3 uint8 RGB lookup table for the Turbo colour map.

    Uses the standard published polynomial fit rather than ffmpeg's
    ``pseudocolor`` filter, whose ``preset`` option is silently ignored on
    ffmpeg 4.x (it emits pure greyscale), and rather than
    ``cv2.applyColorMap`` so the window and the saved preview agree exactly.
    """
    x = np.linspace(0.0, 1.0, 256)
    v = np.stack([np.ones_like(x), x, x ** 2, x ** 3], axis=1)
    w = np.stack([x ** 4, x ** 5], axis=1)

    coeffs = {
        "r": ([0.13572138, 4.61539260, -42.66032258, 132.13108234],
              [-152.94239396, 59.28637943]),
        "g": ([0.09140261, 2.19418839, 4.84296658, -14.18503333],
              [4.27729857, 2.82956604]),
        "b": ([0.10667330, 12.64194608, -60.58204836, 110.36276771],
              [-89.90310912, 27.34824973]),
    }
    channels = [
        v @ np.array(c4) + w @ np.array(c2) for c4, c2 in coeffs.values()
    ]
    return (np.clip(np.stack(channels, axis=1), 0.0, 1.0) * 255).astype(np.uint8)


def colorize(grey: np.ndarray, valid: np.ndarray, lut: np.ndarray) -> np.ndarray:
    """Apply a colour map, keeping invalid pixels black.  Returns H×W×3 RGB."""
    rgb = lut[grey]
    rgb[~valid] = 0
    return rgb


def find_sources(target: Path) -> list[Path]:
    if target.is_file():
        return [target]
    for cand in (target, target / "videos"):
        if cand.is_dir():
            hits = sorted(cand.glob("depth_*.mkv"))
            if hits:
                return hits
    raise SystemExit(f"No depth_*.mkv found under {target}")


def render(frame: np.ndarray, lo: float, hi: float,
           lut: np.ndarray | None) -> np.ndarray:
    """One uint16 mm frame → display-ready 8-bit grey or RGB."""
    grey, valid = window(frame, lo, hi)
    return grey if lut is None else colorize(grey, valid, lut)


def stats(path: Path, w: int, h: int, lo: float, hi: float) -> None:
    first = next(read_frames(path, w, h), None)
    if first is None:
        return
    f = first.astype(np.int64)
    real = f[(f > INVALID_NEAR) & (f < INVALID_FAR)]
    print(f"  {w}x{h}   no-reading {np.mean(f == INVALID_NEAR) * 100:.1f}%   "
          f"too-far {np.mean(f == INVALID_FAR) * 100:.1f}%   "
          f"valid {real.size / f.size * 100:.1f}%")
    if real.size:
        print(f"  measured {real.min()}..{real.max()} mm "
              f"(median {np.median(real):.0f})   "
              f"display window {lo:.0f}..{hi:.0f} mm")


def save_preview(path: Path, out_path: Path, w: int, h: int, fps: float,
                 lo: float, hi: float, lut: np.ndarray | None) -> None:
    """Write an 8-bit preview that plays correctly in any ordinary player."""
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-v", "error",
         "-f", "rawvideo",
         "-pix_fmt", "gray" if lut is None else "rgb24", "-s", f"{w}x{h}",
         "-r", str(fps), "-i", "pipe:0",
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
         str(out_path)],
        stdin=subprocess.PIPE,
    )
    n = 0
    try:
        for frame in read_frames(path, w, h):
            proc.stdin.write(render(frame, lo, hi, lut).tobytes())
            n += 1
    except BrokenPipeError:
        pass
    finally:
        proc.stdin.close()
    proc.wait()
    if proc.returncode != 0:
        raise SystemExit(f"ffmpeg failed writing {out_path}")
    print(f"  wrote {out_path}  ({n} frames)")


def show(path: Path, w: int, h: int, fps: float, lo: float, hi: float,
         lut: np.ndarray | None) -> bool:
    """Interactive playback.  Returns False if cv2/display is unavailable."""
    try:
        import cv2
    except Exception as e:
        print(f"  (cv2 unavailable: {e})")
        return False

    frames = [render(f, lo, hi, lut) for f in read_frames(path, w, h)]
    if not frames:
        return False

    delay = max(1, int(1000 / fps))
    title = f"{path.name}  [{lo:.0f}-{hi:.0f} mm]  space=pause  a/d=step  q=quit"
    idx, paused = 0, False
    try:
        while True:
            img = frames[idx]
            if lut is not None:
                img = img[:, :, ::-1]     # cv2 windows expect BGR
            cv2.imshow(title, img)

            key = cv2.waitKey(0 if paused else delay) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord(" "):
                paused = not paused
            elif key == ord("d"):
                paused, idx = True, min(idx + 1, len(frames) - 1)
            elif key == ord("a"):
                paused, idx = True, max(idx - 1, 0)
            elif not paused:
                idx = (idx + 1) % len(frames)
        cv2.destroyAllWindows()
    except cv2.error as e:
        print(f"  (no display: {e})")
        return False
    return True


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Visualise 16-bit depth .mkv files with a usable grey range.",
    )
    ap.add_argument("target", type=Path,
                    help="a depth_*.mkv, an episode dir, or its videos/ dir")
    ap.add_argument("--range", nargs=2, type=float, metavar=("MIN_MM", "MAX_MM"),
                    help="fixed display window in mm (default: auto percentiles)")
    ap.add_argument("--percentiles", nargs=2, type=float, default=(2.0, 98.0),
                    metavar=("LO", "HI"),
                    help="percentiles for the auto window (default 2 98)")
    ap.add_argument("--cmap", default="gray", choices=["gray", "turbo"],
                    help="grey (default) or the turbo colour map")
    ap.add_argument("--save", nargs="?", const="", metavar="OUT.mp4",
                    help="write an 8-bit preview instead of opening a window")
    args = ap.parse_args()

    for tool in ("ffmpeg", "ffprobe"):
        if shutil.which(tool) is None:
            raise SystemExit(f"{tool} not found on PATH.")

    lut = None if args.cmap == "gray" else turbo_lut()

    for src in find_sources(args.target):
        print(f"\n{src}")
        w, h, fps = probe(src)

        if args.range:
            lo, hi = args.range
        else:
            lo, hi = auto_range(src, w, h, *args.percentiles)
        stats(src, w, h, lo, hi)

        if args.save is not None:
            out = Path(args.save) if args.save else src.with_name(
                src.stem + "_preview.mp4")
            save_preview(src, out, w, h, fps, lo, hi, lut)
        elif not show(src, w, h, fps, lo, hi, lut):
            out = src.with_name(src.stem + "_preview.mp4")
            print("  falling back to a preview file")
            save_preview(src, out, w, h, fps, lo, hi, lut)


if __name__ == "__main__":
    sys.exit(main())
