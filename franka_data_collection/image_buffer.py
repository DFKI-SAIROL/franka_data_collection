from dataclasses import dataclass, field
from threading import Lock
from typing import Optional

import numpy as np


@dataclass
class CameraFrame:
    """One time-stamped snapshot from all cameras in a synced message."""
    timestamp_ns: int
    rgbs: list[np.ndarray]        # list of H×W×3 uint8, one per camera
    depths: list[np.ndarray]      # list of H×W uint16, one per camera (may be empty)


class ImageBuffer:
    """
    Thread-safe accumulator for synced camera frames.

    Lifecycle
    ---------
    start()  → begin accepting frames
    push()   → called from the ROS subscriber callback
    stop()   → stop accepting frames
    drain()  → retrieve all accumulated frames and reset

    The buffer stores frames as raw numpy arrays.  Encoding to video is
    handled separately by VideoEncoder so that the ROS spin-loop is never
    blocked by I/O.
    """

    def __init__(self) -> None:
        self._frames: list[CameraFrame] = []
        self._lock = Lock()
        self._active = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        with self._lock:
            self._frames.clear()
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False

    def reset(self) -> None:
        """Drop all buffered frames without returning them."""
        with self._lock:
            self._frames.clear()
            self._active = False

    # ------------------------------------------------------------------
    # Data ingestion
    # ------------------------------------------------------------------

    def push(
        self,
        timestamp_ns: int,
        rgbs: list[np.ndarray],
        depths: Optional[list[np.ndarray]] = None,
    ) -> None:
        """
        Store one synced frame.  Silently dropped when the buffer is not active.

        Parameters
        ----------
        timestamp_ns:
            Header stamp in nanoseconds (sec * 1e9 + nanosec).
        rgbs:
            One HxWx3 uint8 array per camera, in the same order as
            ``camera_names`` in the ROS message.
        depths:
            One HxW uint16 array per camera.  Pass None or an empty list
            when depth is not recorded.
        """
        if not self._active:
            return

        frame = CameraFrame(
            timestamp_ns=timestamp_ns,
            rgbs=rgbs,
            depths=depths or [],
        )
        with self._lock:
            if self._active:          # re-check under lock
                self._frames.append(frame)

    # ------------------------------------------------------------------
    # Data retrieval
    # ------------------------------------------------------------------

    def drain(self) -> list[CameraFrame]:
        """
        Return all accumulated frames and clear the internal list.

        Safe to call after ``stop()``.  The buffer is left empty but not
        reset — call ``reset()`` explicitly if you want to discard frames
        without inspecting them.
        """
        with self._lock:
            frames = list(self._frames)
            self._frames.clear()
        return frames

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def size(self) -> int:
        with self._lock:
            return len(self._frames)

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._active