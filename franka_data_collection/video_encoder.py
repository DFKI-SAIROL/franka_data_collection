import subprocess
from pathlib import Path

import numpy as np

from .image_buffer import CameraStream


class VideoEncoder:
    """
    Encodes per-camera :class:`CameraStream` objects into video files
    plus per-camera ``timestamps_<cam>.npy`` sidecars.

    Directory layout produced
    -------------------------
    ::

        <output_dir>/
            videos/
                rgb_<camera_name>.mp4         # H.264 via ffmpeg
                timestamps_<camera_name>.npy  # int64 nanoseconds, shape (N,)

    Per-camera timestamps make alignment a downstream concern: the dataset
    builder pairs frames across cameras using these files.

    Parameters
    ----------
    camera_names:
        Ordered list of camera names.  One video file per name will be
        written when that camera has frames.
    codec_config:
        Dict with keys:
          - ``rgb`` : ``"h264"``, ``"h265"``, or ``"h264_nvenc"``
          - ``crf`` : H.264/H.265 quality (lower = better, 23 default)
          - ``preset`` : ffmpeg preset, e.g. ``"veryfast"``, ``"medium"``
    fps:
        Frame rate written into the video container.  Per-frame ROS
        timestamps in ``timestamps_<cam>.npy`` are the source of truth
        for actual timing — the container fps is approximate and only
        affects playback in standard video players.
    """

    _RGB_CODEC_PARAMS: dict[str, list[str]] = {
        "h264":       ["-c:v", "libx264",     "-pix_fmt", "yuv420p"],
        "h265":       ["-c:v", "libx265",     "-pix_fmt", "yuv420p"],
        "h264_nvenc": ["-c:v", "h264_nvenc",  "-pix_fmt", "yuv420p"],
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
        self.crf: int = int(codec_config.get("crf", 23))
        self.preset: str = codec_config.get("preset", "medium")

        if self.rgb_codec not in self._RGB_CODEC_PARAMS:
            raise ValueError(
                f"Unsupported rgb codec '{self.rgb_codec}'. "
                f"Choose from {list(self._RGB_CODEC_PARAMS)}."
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def encode(
        self,
        streams: dict[str, CameraStream],
        output_dir: Path,
    ) -> None:
        """
        Encode each camera's stream into its own mp4 + timestamps file.

        Parameters
        ----------
        streams:
            Mapping of camera name → :class:`CameraStream` as returned
            by :meth:`ImageBuffer.drain`.
        output_dir:
            Episode root directory.  A ``videos/`` subdirectory is created.
        """
        video_dir = output_dir / "videos"
        video_dir.mkdir(parents=True, exist_ok=True)

        for cam_name in self.camera_names:
            stream = streams.get(cam_name)
            if stream is None or len(stream) == 0:
                continue

            ts = np.asarray(stream.timestamps_ns, dtype=np.int64)
            np.save(video_dir / f"timestamps_{cam_name}.npy", ts)

            self._encode_rgb(
                stream.frames,
                video_dir / f"rgb_{cam_name}.mp4",
            )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _encode_rgb(self, frames: list[np.ndarray], out_path: Path) -> None:
        """Encode H×W×3 uint8 BGR frames to video via ffmpeg."""
        if not frames:
            return

        h, w = frames[0].shape[:2]
        codec_params = self._RGB_CODEC_PARAMS[self.rgb_codec]

        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-s", f"{w}x{h}",
            "-pix_fmt", "bgr24",
            "-r", str(self.fps),
            "-i", "pipe:0",
            *codec_params,
            "-preset", self.preset,
            "-crf", str(self.crf),
            "-movflags", "+faststart",
            str(out_path),
        ]

        proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE
        )
        try:
            for frame in frames:
                proc.stdin.write(frame.tobytes())
        finally:
            proc.stdin.close()
        proc.wait()

        if proc.returncode != 0:
            stderr = proc.stderr.read().decode(errors="replace")
            raise RuntimeError(
                f"ffmpeg failed encoding {out_path.name} "
                f"(rc={proc.returncode}):\n{stderr}"
            )