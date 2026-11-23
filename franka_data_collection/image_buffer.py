from dataclasses import dataclass
from threading import Lock

import numpy as np


@dataclass
class CameraStream:
    """Accumulated frames + timestamps for one camera."""
    timestamps_ns: list[int]
    frames: list[np.ndarray]   # H×W×3 uint8 (BGR)

    def __len__(self) -> int:
        return len(self.frames)


class ImageBuffer:
    """
    Thread-safe per-camera accumulator for RGB frames.

    Each camera has its own independent stream of (timestamp, frame) pairs.
    Cameras are NOT aligned in this buffer — alignment is a dataset-build
    step performed later using the saved per-camera timestamp files.

    Lifecycle
    ---------
    start()  → begin accepting frames
    push()   → called from a ROS subscriber callback (one camera at a time)
    stop()   → stop accepting frames
    drain()  → retrieve all per-camera streams and clear

    Encoding to video is handled separately by VideoEncoder so that the
    ROS spin-loop is never blocked by I/O.
    """

    def __init__(self, camera_names: list[str]) -> None:
        self.camera_names = list(camera_names)
        self._streams: dict[str, CameraStream] = {
            name: CameraStream(timestamps_ns=[], frames=[])
            for name in self.camera_names
        }
        self._lock = Lock()
        self._active = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        with self._lock:
            for stream in self._streams.values():
                stream.timestamps_ns.clear()
                stream.frames.clear()
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False

    def reset(self) -> None:
        """Drop all buffered frames without returning them."""
        with self._lock:
            for stream in self._streams.values():
                stream.timestamps_ns.clear()
                stream.frames.clear()
            self._active = False

    # ------------------------------------------------------------------
    # Data ingestion
    # ------------------------------------------------------------------

    def push(
        self,
        camera_name: str,
        timestamp_ns: int,
        frame: np.ndarray,
    ) -> None:
        """
        Store one frame from one camera.  Silently dropped when inactive.

        Parameters
        ----------
        camera_name:
            Name of the camera the frame came from.  Must match one of the
            names passed to ``__init__``.
        timestamp_ns:
            Header stamp in nanoseconds.
        frame:
            H×W×3 uint8 BGR array.
        """
        if not self._active:
            return
        if camera_name not in self._streams:
            raise KeyError(
                f"Unknown camera '{camera_name}'. Known: {self.camera_names}"
            )

        with self._lock:
            if not self._active:        # re-check under lock
                return
            stream = self._streams[camera_name]
            stream.timestamps_ns.append(timestamp_ns)
            stream.frames.append(frame)

    def drain(self) -> dict[str, CameraStream]:
        """
        Return all per-camera streams and clear the internal buffers.

        Safe to call after ``stop()``.  The buffer is left empty.
        """
        with self._lock:
            out = {
                name: CameraStream(
                    timestamps_ns=list(stream.timestamps_ns),
                    frames=list(stream.frames),
                )
                for name, stream in self._streams.items()
            }
            for stream in self._streams.values():
                stream.timestamps_ns.clear()
                stream.frames.clear()
        return out

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def sizes(self) -> dict[str, int]:
        """Per-camera frame counts (for logging)."""
        with self._lock:
            return {name: len(s.frames) for name, s in self._streams.items()}

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._active