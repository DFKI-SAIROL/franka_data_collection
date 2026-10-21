"""One recorded episode in the data collector's layout, shared by every writer so they
all produce exactly the same thing:

    <storage_path>/<task><suffix>/episode_<YYYYmmdd_HHMMSS>/
        mcap/      ros2 bag (mcap) of the configured topics + lwk_metadata.yaml (task, label)
        videos/    rgb_<cam>.mp4 + timestamps*.npy from the camera-rig snapshots

Episodes are recorded into <storage_path>/_unlabeled and moved to their task folder once labeled and
their videos are written (same filesystem, so the move is an atomic rename).
"""
import shutil
import signal
import subprocess
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


from franka_data_collection.image_buffer import ImageBuffer
from franka_data_collection.video_encoder import VideoEncoder

# Defaults; override per deployment via storage_path / tasks / labels in the collector config.
DEFAULT_TASKS: list[str] = []
DEFAULT_LABELS = ['Perfect', 'Good', 'Recovery', 'Failure']


def load_collector_config(config_file: str) -> dict:
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    # The task is picked when labeling, so a config must not pre-empt it.
    if 'task_name' in config:
        raise ValueError(f'{config_file}: "task_name" is not supported; the task is chosen when labeling.')
    if not config.get('storage_path'):
        raise ValueError(f'{config_file}: "storage_path" is required (dataset root).')
    if not config.get('videos'):
        raise ValueError(f'{config_file}: no "videos" section')
    return config


class EpisodeWriter:
    """Records one episode at a time: start() -> stop() -> finish(task, label) or discard().

    node: the ROS node the camera-rig subscription is created on.
    arms: arm namespaces the per-arm topic templates ("{arm}") are expanded for.
    suffix: appended to the task folder (e.g. '_dagger').
    subscribe_always: False subscribes to the camera rig only between start() and stop(),
        so the 7 MB snapshots are not received (and deserialized) while nothing is recorded.
    """

    def __init__(self, node, config: dict, arms: list[str], suffix: str = '',
                 subscribe_always: bool = True, callback_group=None):
        self.node = node
        self.logger = node.get_logger()
        self.suffix = suffix
        self.callback_group = callback_group
        arm_topics = config.get('arm_topics') or []
        self.topics = [t.format(arm=arm) for arm in arms for t in arm_topics] + list(config.get('topics') or [])
        if not self.topics:
            raise ValueError('No topics to record provided!')

        video_cfg = config['videos']
        self.cameras: list[dict] = video_cfg['cameras']
        self.camera_names: list[str] = [c['name'] for c in self.cameras]
        self.image_buffer = ImageBuffer(camera_names=self.camera_names)
        self.video_encoder = VideoEncoder(
            camera_names=self.camera_names,
            codec_config=video_cfg.get('codec', {}),
            fps=video_cfg['camera_fps'],
        )

        self.dataset_root = Path(config['storage_path']).expanduser()
        self.unlabeled_dir = self.dataset_root / '_unlabeled'
        self.tasks: list[str] = list(config.get('tasks') or DEFAULT_TASKS)
        self.labels: list[str] = list(config.get('labels') or DEFAULT_LABELS)

        self.record_process = None
        self.episode_path: Path | None = None   # episode being recorded, or stopped and not yet labeled
        # Background video encoding of the last stopped episode; the episode is only moved to
        # its task folder after this has finished writing.
        self.encode_thread: threading.Thread | None = None
        self._sub: list = []
        self._subscribe_always = subscribe_always
        if subscribe_always:
            self._subscribe()
        self.unlabeled_dir.mkdir(parents=True, exist_ok=True)

    @property
    def is_recording(self) -> bool:
        return self.record_process is not None

    # ---- cameras ---------------------------------------------------------------------------

    def _subscribe(self):
        if self._sub:
            return
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        # One RGB topic per camera; timestamps stay independent and alignment is a
        # dataset-build step using the saved per-camera timestamp files.
        for cam in self.cameras:
            name = cam['name']
            self._sub.append(self.node.create_subscription(
                Image, cam['rgb_topic'],
                lambda msg, n=name: self._image_callback(msg, n),
                qos, callback_group=self.callback_group))

    def _unsubscribe(self):
        if self._sub and not self._subscribe_always:
            for sub in self._sub:
                self.node.destroy_subscription(sub)
            self._sub = []

    def _image_callback(self, msg: Image, camera_name: str) -> None:
        if not self.image_buffer.is_active:
            return
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.image_buffer.push(camera_name, timestamp_ns, self._ros_image_to_numpy(msg))

    # ---- episode lifecycle -----------------------------------------------------------------

    def start(self) -> Path:
        """Starts the bag and the frame buffer; raises if the bag cannot be started."""
        if self.is_recording:
            raise RuntimeError(f'Already recording {self.episode_path}')
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.episode_path = self.unlabeled_dir / f'episode_{timestamp}'
        # Rosbag goes into a dedicated subdirectory so it never collides with
        # the videos/ folder written by the encoder.
        mcap_path = self.episode_path / 'mcap'
        command = ['ros2', 'bag', 'record', '-o', str(mcap_path), '-s', 'mcap'] + self.topics
        self.logger.info(f'Starting recording to: {self.episode_path}')
        self._subscribe()
        try:
            self.record_process = subprocess.Popen(command)
            self.image_buffer.start()
        except Exception as e:
            self.logger.error(f'Failed to start recording: {e}')
            self.record_process = None
            self.image_buffer.reset()
            self._unsubscribe()
            raise
        return self.episode_path

    def stop(self) -> int:
        """Stops the bag and starts encoding the buffered frames in the background.
        Returns the number of camera frames."""
        if self.record_process is None:
            raise RuntimeError('No active recording process found.')
        self.logger.info('Stopping recording...')
        # Stop the rosbag process gracefully
        self.record_process.send_signal(signal.SIGINT)
        self.record_process.wait()
        self.record_process = None
        # Stop the image buffer — no more frames will be pushed
        self.image_buffer.stop()
        self._unsubscribe()
        # Drain frames and kick off background encoding
        frames = self.image_buffer.drain()
        self.logger.info(f'Encoding {len(frames)} frames to video...')
        if frames:
            self.encode_thread = self._encode_async(frames, self.episode_path)
        else:
            self.encode_thread = None
            self.logger.warn('No camera frames were buffered — skipping video encoding.')
        self.logger.info('Recording stopped. Video encoding running in background.')
        return len(frames)

    def finish(self, task: str, label: str, extra_files: dict[str, dict] | None = None) -> Path:
        """Labels the stopped episode (mcap/lwk_metadata.yaml, plus extra_files: name -> dict
        written as YAML next to it) and moves it to <storage_path>/<task><suffix> once its videos
        are written. Returns the target folder."""
        episode_path, encode_thread = self._take_stopped()
        metadata_dir = episode_path / 'mcap'
        with open(metadata_dir / 'lwk_metadata.yaml', 'w') as f:
            yaml.dump({'task': task, 'label': label}, f)
        for name, content in (extra_files or {}).items():
            with open(metadata_dir / name, 'w') as f:
                yaml.safe_dump(content, f, sort_keys=False, allow_unicode=True)
        target = self.dataset_root / f'{task}{self.suffix}' / episode_path.name
        threading.Thread(target=self._move_to_task_dir, args=(episode_path, encode_thread, target),
                         daemon=True).start()
        return target

    def discard(self) -> Path:
        """Deletes the stopped episode once its video encoding is done."""
        episode_path, encode_thread = self._take_stopped()

        def _delete():
            if encode_thread is not None:
                encode_thread.join()
            shutil.rmtree(episode_path, ignore_errors=True)
            self.logger.info(f'Episode discarded: {episode_path}')
        threading.Thread(target=_delete, daemon=True).start()
        return episode_path

    def _take_stopped(self):
        if self.is_recording or self.episode_path is None:
            raise RuntimeError('No stopped episode to label.')
        stopped = self.episode_path, self.encode_thread
        self.episode_path, self.encode_thread = None, None
        return stopped

    def _encode_async(self, frames, episode_path: Path) -> threading.Thread:
        """Encode in a background thread so the next episode can start recording immediately
        without waiting for ffmpeg."""
        def _encode():
            try:
                self.video_encoder.encode(frames, episode_path)
                self.logger.info(f'Video encoding complete: {episode_path / "videos"}')
            except Exception as e:
                self.logger.error(f'Video encoding failed: {e}')

        t = threading.Thread(target=_encode, daemon=True)
        t.start()
        return t

    def _move_to_task_dir(self, episode_path: Path, encode_thread, target: Path) -> None:
        """Move a labeled episode to its task folder once its videos are written."""
        if encode_thread is not None:
            encode_thread.join()
        if target.exists():
            self.logger.error(f'{target} already exists — episode left in {episode_path}')
            return
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            episode_path.rename(target)
            self.logger.info(f'Episode saved to {target}')
        except OSError as e:
            self.logger.error(f'Failed to move episode to {target} ({e}) — left in {episode_path}')

    # ---- image conversion --------------------------------------------------------------------

    @staticmethod
    def _ros_image_to_numpy(img: Image) -> np.ndarray:
        """
        Convert a sensor_msgs/Image to a H×W×3 uint8 numpy array (BGR).

        Supports encoding strings: ``rgb8``, ``bgr8``, ``rgba8``, ``bgra8``.
        """
        data = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        encoding = img.encoding.lower()
        if encoding in ('rgb8', 'rgb'):
            # OpenCV / ffmpeg expect BGR
            return data[:, :, ::-1].copy()
        elif encoding in ('rgba8',):
            return data[:, :, 2::-1].copy()   # RGBA → BGR
        elif encoding in ('bgra8',):
            return data[:, :, :3].copy()       # drop alpha
        elif encoding in ('bgr8', 'bgr'):
            return data
        else:
            raise ValueError(f"Unsupported RGB encoding: {img.encoding!r}")

    @staticmethod
    def _ros_image_to_numpy_depth(img: Image) -> np.ndarray:
        """
        Convert a sensor_msgs/Image depth frame to a HxW uint16 numpy array.

        Supports ``16UC1`` (millimetres, native ZED output) and
        ``32FC1`` (metres, converted to mm uint16).
        """
        encoding = img.encoding.lower()
        if encoding == '16uc1':
            return np.frombuffer(img.data, dtype=np.uint16).reshape(img.height, img.width).copy()
        elif encoding == '32fc1':
            # Convert metres → millimetres, clamp to uint16 range
            f32 = np.frombuffer(img.data, dtype=np.float32).reshape(img.height, img.width)
            return (f32 * 1000.0).clip(0, 65535).astype(np.uint16)
        else:
            raise ValueError(f"Unsupported depth encoding: {img.encoding!r}")
