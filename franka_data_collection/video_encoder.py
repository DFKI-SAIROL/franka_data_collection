import subprocess
from pathlib import Path

import numpy as np

from .image_buffer import CameraFrame


class VideoEncoder:
    """
    Encodes a list of :class:`CameraFrame` objects into per-camera video files
    and writes a ``timestamps.npy`` sidecar alongside them.

    Directory layout produced
    -------------------------
    ::

        <output_dir>/
            videos/
                timestamps.npy          # int64 nanoseconds, shape (N,)
                rgb_<camera_name>.mp4   # H.264 via ffmpeg
                depth_<camera_name>.mkv # FFV1 lossless (when depth present)

    Parameters
    ----------
    camera_names:
        Ordered list of camera names matching the order of the arrays inside
        each :class:`CameraFrame`.
    codec_config:
        Dict with keys ``rgb`` (``"h264"``), ``depth`` (``"ffv1"``),
        ``crf`` (int, H.264 quality — lower = better, 23 is a good default).
    fps:
        Frame rate used for the container timestamps.  Does not affect the
        per-frame ROS timestamps stored in ``timestamps.npy``.
    """

    # Supported RGB codecs and their ffmpeg parameters
    _RGB_CODEC_PARAMS: dict[str, list[str]] = {
        "h264": ["-c:v", "libx264", "-pix_fmt", "yuv420p"],
        "h265": ["-c:v", "libx265", "-pix_fmt", "yuv420p"],
    }

    def __init__(
        self,
        camera_names: list[str],
        codec_config: dict,
        fps: float = 30.0,
    ) -> None:
        self.camera_names = camera_names
        self.fps = fps

        self.rgb_codec: str = codec_config.get("rgb", "h264")
        self.depth_codec: str = codec_config.get("depth", "ffv1")
        self.crf: int = int(codec_config.get("crf", 23))

        if self.rgb_codec not in self._RGB_CODEC_PARAMS:
            raise ValueError(
                f"Unsupported rgb codec '{self.rgb_codec}'. "
                f"Choose from {list(self._RGB_CODEC_PARAMS)}."
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def encode(self, frames: list[CameraFrame], output_dir: Path) -> None:
        """
        Encode *frames* and write all output files under *output_dir/videos/*.

        Parameters
        ----------
        frames:
            Ordered list of frames as returned by :meth:`ImageBuffer.drain`.
        output_dir:
            Episode root directory.  A ``videos/`` subdirectory is created.
        """
        if not frames:
            raise ValueError("Cannot encode an empty frame list.")

        video_dir = output_dir / "videos"
        video_dir.mkdir(parents=True, exist_ok=True)

        # 1. Save timestamps
        timestamps = np.array([f.timestamp_ns for f in frames], dtype=np.int64)
        np.save(video_dir / "timestamps.npy", timestamps)

        has_depth = any(len(f.depths) > 0 for f in frames)

        # 2. Encode per-camera RGB streams
        for cam_idx, cam_name in enumerate(self.camera_names):
            rgb_frames = [f.rgbs[cam_idx] for f in frames]
            out_path = video_dir / f"rgb_{cam_name}.mp4"
            self._encode_rgb(rgb_frames, out_path)

        # 3. Encode per-camera depth streams (when present)
        if has_depth:
            for cam_idx, cam_name in enumerate(self.camera_names):
                depth_frames = [f.depths[cam_idx] for f in frames if f.depths]
                out_path = video_dir / f"depth_{cam_name}.mkv"
                self._encode_depth(depth_frames, out_path)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _encode_rgb(self, frames: list[np.ndarray], out_path: Path) -> None:
        """Encode a list of H×W×3 uint8 frames to H.264 mp4 via ffmpeg."""
        if not frames:
            raise ValueError(f"No RGB frames to encode for {out_path.name}.")

        h, w = frames[0].shape[:2]
        codec_params = self._RGB_CODEC_PARAMS[self.rgb_codec]

        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-s", f"{w}x{h}",
            "-pix_fmt", "bgr24",      # OpenCV default channel order
            "-r", str(self.fps),
            "-i", "pipe:0",
            *codec_params,
            "-crf", str(self.crf),
            "-movflags", "+faststart",
            str(out_path),
        ]

        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        for frame in frames:
            proc.stdin.write(frame.tobytes())
        proc.stdin.close()
        proc.wait()

        if proc.returncode != 0:
            stderr = proc.stderr.read().decode(errors="replace")
            raise RuntimeError(
                f"ffmpeg failed encoding {out_path.name} (rc={proc.returncode}):\n{stderr}"
            )

    def _encode_depth(self, frames: list[np.ndarray], out_path: Path) -> None:
        """
        Encode a list of H×W uint16 depth frames as FFV1 in an MKV container.

        FFV1 is a lossless codec that natively handles 16-bit grayscale,
        preserving the full depth range without any normalization.
        """
        if not frames:
            raise ValueError(f"No depth frames to encode for {out_path.name}.")

        h, w = frames[0].shape[:2]

        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-s", f"{w}x{h}",
            "-pix_fmt", "gray16le",   # 16-bit little-endian grayscale
            "-r", str(self.fps),
            "-i", "pipe:0",
            "-c:v", "ffv1",
            "-level", "3",            # FFV1 v3: supports multithreaded encoding
            str(out_path),
        ]

        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        for frame in frames:
            # Ensure native byte order for ffmpeg
            proc.stdin.write(frame.astype("<u2").tobytes())
        proc.stdin.close()
        proc.wait()

        if proc.returncode != 0:
            stderr = proc.stderr.read().decode(errors="replace")
            raise RuntimeError(
                f"ffmpeg failed encoding {out_path.name} (rc={proc.returncode}):\n{stderr}"
            )