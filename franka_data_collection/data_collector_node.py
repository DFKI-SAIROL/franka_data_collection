import os
import signal
import subprocess
import shutil
from datetime import datetime
from pathlib import Path
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

from franka_custom_msgs.msg import RigSnapshot

from franka_data_collection.image_buffer import ImageBuffer
from franka_data_collection.video_encoder import VideoEncoder


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        default_config = Path(__file__).resolve().parents[1] / 'config' / 'data_collector.yaml'
        config_file = Path(self.declare_parameter('config_path', str(default_config)).value).expanduser()

        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        self.config_dir = config_file.resolve().parent

        # 1. Configuration Setup
        self.storage_path = Path(self.config.get('storage_path', './data'))
        self.storage_path.mkdir(parents=True, exist_ok=True)
        
        # Assuming your YAML parser just returns a flat list of topic names now
        self.topics_to_record = self.config.get('topics', None)
        if self.topics_to_record is None:
            self.get_logger().error('No topics to record provided!')
            return

        video_cfg = self.config.get('videos', None)
        if video_cfg is None:
            self.get_logger().error('No "videos" section found in config!')
            return
 
        self.video_topic: str = video_cfg['topic']
        self.camera_fps: float = video_cfg['camera_fps']
        self.camera_names: list[str] = [c['name'] for c in video_cfg['cameras']]
        self.codec_config: dict = video_cfg.get('codec', {})
        # Calibrated cameras: every camera with a non-empty videos.extrinsics entry. Their extrinsics and CameraInfo
        # are stored with each episode; a camera without an entry is recorded without calibration.
        extrinsics_cfg = video_cfg.get('extrinsics') or {}
        self.pose_camera_names = tuple(name for name, path in extrinsics_cfg.items() if path)
        if not self.pose_camera_names:
            raise ValueError('videos.extrinsics must name at least one calibrated camera')
        unknown = set(self.pose_camera_names) - set(self.camera_names)
        if unknown:
            raise ValueError(f'videos.extrinsics names cameras missing from videos.cameras: {sorted(unknown)}')
        self.extrinsic_paths = {
            name: self._configured_extrinsic_path(extrinsics_cfg[name]) for name in self.pose_camera_names
        }
        self.get_logger().info(f'Calibrated cameras: {list(self.pose_camera_names)}')

        # 2. State Variables
        self.is_recording = False
        self.record_process = None
        self.episode_path = None
        self.saved_camera_infos = set()
        self.expected_camera_frames = {}
        self.calibration_error = None

        self.image_buffer = ImageBuffer()
        self.video_encoder = VideoEncoder(
            camera_names=self.camera_names,
            codec_config=self.codec_config,
            fps=self.camera_fps,
        )

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(
            RigSnapshot,
            self.video_topic,
            self._camera_sync_callback,
            qos_profile=input_qos,
        )

        # 3. Services
        self.srv_trigger = self.create_service(
            Trigger, '~/record_data_trigger', self.recording_callback
        )

        self.get_logger().info('Data collector ready. Waiting for trigger...')

    def recording_callback(self, request, response):
        """Toggles the recording state."""
        if not self.is_recording:
            success, message = self.start_recording()
        else:
            success, message = self.stop_recording()

        response.success = success
        response.message = message
        return response

    def _camera_sync_callback(self, msg) -> None:
        """
        Deserialize the synced snapshot message and push frames into the buffer.
 
        Expected message fields (from your custom type):
            header          std_msgs/Header
            camera_names    string[]
            cam_infos       sensor_msgs/CameraInfo[]
            rgbs            sensor_msgs/Image[]
            depths          sensor_msgs/Image[]   (may be empty)
        """
        if not self.image_buffer.is_active:
            return
        if (len(msg.camera_names) != len(msg.rgbs)
                or len(msg.camera_names) != len(msg.cam_infos)
                or len(msg.camera_names) != len(msg.depths)):
            self.calibration_error = 'RigSnapshot camera_names, rgbs, depths and cam_infos lengths differ'
            self.get_logger().error(self.calibration_error)
            return
        if tuple(msg.camera_names) != tuple(self.camera_names):
            self.calibration_error = 'RigSnapshot camera order differs from videos.cameras configuration'
            self.get_logger().error(self.calibration_error)
            return
        for name in self.pose_camera_names:
            index = msg.camera_names.index(name)
            info, rgb = msg.cam_infos[index], msg.rgbs[index]
            depth = msg.depths[index]
            if (info.width, info.height) != (rgb.width, rgb.height) or not info.header.frame_id:
                self.calibration_error = f'{name}: CameraInfo size/frame_id does not match the RGB image'
                self.get_logger().error(self.calibration_error)
                return
            if (depth.width, depth.height) != (rgb.width, rgb.height):
                self.calibration_error = f'{name}: depth dimensions do not match RGB image'
                self.get_logger().error(self.calibration_error)
                return
            if info.header.frame_id != self.expected_camera_frames[name]:
                self.calibration_error = f'{name}: CameraInfo frame_id differs from calibrated child_frame'
                self.get_logger().error(self.calibration_error)
                return
            if not all(np.isfinite(info.k)) or info.k[0] <= 0 or info.k[4] <= 0:
                self.calibration_error = f'{name}: invalid CameraInfo K matrix'
                self.get_logger().error(self.calibration_error)
                return
            if name not in self.saved_camera_infos:
                try:
                    self._save_camera_info(name, info)
                except OSError as exc:
                    self.calibration_error = f'{name}: failed to save CameraInfo: {exc}'
                    self.get_logger().error(self.calibration_error)
                    return
                self.saved_camera_infos.add(name)
 
        timestamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )
 
        rgbs = [self._ros_image_to_numpy(img) for img in msg.rgbs]
 
        depths = (
            [self._ros_image_to_numpy_depth(img) for img in msg.depths]
            if msg.depths
            else []
        )
 
        self.image_buffer.push(timestamp_ns, rgbs, depths)

    def start_recording(self):
        try:
            extrinsics = {name: self._load_extrinsic(name) for name in self.pose_camera_names}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return False, f'Camera extrinsic calibration is unavailable: {exc}'
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.episode_path = self.storage_path / f'episode_{timestamp}'
        if self.episode_path.exists():
            return False, f'Episode directory already exists: {self.episode_path}'
        calibration_dir = self.episode_path / 'calibration'
        calibration_dir.mkdir(parents=True)
        try:
            for name in self.pose_camera_names:
                shutil.copyfile(self.extrinsic_paths[name], calibration_dir / f'{name}_extrinsic.yaml')
            with (calibration_dir / 'depth_format.yaml').open('w') as stream:
                yaml.safe_dump({
                    'stored_encoding': 'gray16le_uint16_mm',
                    'scale_to_meters': 0.001,
                    'invalid_values': [0, 65535],
                    'registered_to_rgb': True,
                }, stream, sort_keys=False)
        except OSError as exc:
            return False, f'Failed to save episode calibration: {exc}'
        self.expected_camera_frames = {
            name: extrinsics[name]['child_frame'] for name in self.pose_camera_names
        }
        self.saved_camera_infos = set()
        self.calibration_error = None
 
        # Rosbag goes into a dedicated subdirectory so it never collides with
        # the videos/ folder written by the encoder.
        mcap_path = self.episode_path / 'mcap'
 
        command = [
            'ros2', 'bag', 'record',
            '-o', str(mcap_path),
            '-s', 'mcap',
        ] + self.topics_to_record
 
        self.get_logger().info(f'Starting recording to: {self.episode_path}')
 
        try:
            self.record_process = subprocess.Popen(command)
            self.image_buffer.start()
            self.is_recording = True
            return True, f"Started recording to {self.episode_path}"
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")
            self.image_buffer.reset()
            return False, str(e)
 
    def stop_recording(self):
        if self.record_process is None:
            return False, "No active recording process found."
 
        self.get_logger().info("Stopping recording...")
 
        # Stop the rosbag process gracefully
        self.record_process.send_signal(signal.SIGINT)
        self.record_process.wait()
        self.record_process = None
 
        # Stop the image buffer — no more frames will be pushed
        self.image_buffer.stop()
        missing = set(self.pose_camera_names) - self.saved_camera_infos
 
        # Drain frames and kick off background encoding
        frames = self.image_buffer.drain()
        self.get_logger().info(f"Encoding {len(frames)} frames to video...")
 
        if frames:
            self._encode_async(frames, self.episode_path)
        else:
            self.get_logger().warn("No camera frames were buffered — skipping video encoding.")
 
        self.is_recording = False
        if missing or self.calibration_error:
            problem = self.calibration_error or f'Missing CameraInfo for: {sorted(missing)}'
            self.get_logger().error(f'Episode calibration incomplete: {problem}')
            return False, f'Recording stopped; episode calibration incomplete: {problem}'
        self.get_logger().info("Recording stopped. Video encoding running in background.")
        return True, f"Stopped recording. Encoding {len(frames)} frames."

    def _configured_extrinsic_path(self, value):
        if not value:
            return None
        path = Path(os.path.expandvars(value)).expanduser()
        return path if path.is_absolute() else self.config_dir / path

    def _load_extrinsic(self, name):
        path = self.extrinsic_paths[name]
        if path is None or not path.is_file():
            raise ValueError(f'{name}: set videos.extrinsics.{name} to an existing calibrated YAML file')
        with path.open() as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or not data.get('parent_frame') or not data.get('child_frame'):
            raise ValueError(f'{path}: parent_frame and child_frame are required')
        translation = np.asarray(data.get('translation_m'), dtype=float)
        quaternion = np.asarray(data.get('quaternion_xyzw'), dtype=float)
        if translation.shape != (3,) or quaternion.shape != (4,):
            raise ValueError(f'{path}: expected translation_m[3] and quaternion_xyzw[4]')
        if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
            raise ValueError(f'{path}: transform contains non-finite values')
        if not np.isclose(np.linalg.norm(quaternion), 1.0, atol=0.01):
            raise ValueError(f'{path}: quaternion_xyzw must have unit length')
        return data

    def _save_camera_info(self, name, info):
        roi = info.roi
        record = {
            'frame_id': info.header.frame_id,
            'width': int(info.width), 'height': int(info.height),
            'distortion_model': info.distortion_model,
            'd': [float(v) for v in info.d],
            'k': [float(v) for v in info.k],
            'r': [float(v) for v in info.r],
            'p': [float(v) for v in info.p],
            'binning_x': int(info.binning_x), 'binning_y': int(info.binning_y),
            'roi': {
                'x_offset': int(roi.x_offset), 'y_offset': int(roi.y_offset),
                'height': int(roi.height), 'width': int(roi.width),
                'do_rectify': bool(roi.do_rectify),
            },
        }
        path = self.episode_path / 'calibration' / f'{name}_camera_info.yaml'
        with path.open('w') as stream:
            yaml.safe_dump(record, stream, sort_keys=False)
        # Plain 3x3 intrinsic matrix, the camera_matrix_file format of 6dposeestimationtracking
        np.savetxt(self.episode_path / 'calibration' / f'{name}_cam_K.txt',
                   np.asarray(info.k, dtype=float).reshape(3, 3), fmt='%.10g')

    def _encode_async(self, frames, episode_path: Path) -> None:
        """
        Run video encoding in a background subprocess so the next episode
        can start recording immediately without waiting for ffmpeg.
 
        We serialize the frames to a temporary numpy file and spawn a small
        helper script.  This avoids any shared-memory or pickle complexity.
        """
        import threading
 
        def _encode():
            try:
                self.video_encoder.encode(frames, episode_path)
                self.get_logger().info(
                    f"Video encoding complete: {episode_path / 'videos'}"
                )
            except Exception as e:
                self.get_logger().error(f"Video encoding failed: {e}")
 
        t = threading.Thread(target=_encode, daemon=True)
        t.start()

    @staticmethod
    def _ros_image_to_numpy(img: Image) -> 'np.ndarray':
        """
        Convert a sensor_msgs/Image to a H×W×3 uint8 numpy array (BGR).
 
        Supports encoding strings: ``rgb8``, ``bgr8``, ``rgba8``, ``bgra8``.
        """
 
        data = np.frombuffer(img.data, dtype=np.uint8).reshape(
            img.height, img.width, -1
        )
 
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
    def _ros_image_to_numpy_depth(img: Image) -> 'np.ndarray':
        """
        Convert a sensor_msgs/Image depth frame to a HxW uint16 numpy array.
 
        Supports ``16UC1`` (millimetres, native ZED output) and
        ``32FC1`` (metres, converted to mm uint16).
        """
 
        encoding = img.encoding.lower()
        if encoding == '16uc1':
            return np.frombuffer(img.data, dtype=np.uint16).reshape(
                img.height, img.width
            ).copy()
        elif encoding == '32fc1':
            # Convert metres → millimetres, clamp to uint16 range
            f32 = np.frombuffer(img.data, dtype=np.float32).reshape(
                img.height, img.width
            )
            mm = (f32 * 1000.0).clip(0, 65535).astype(np.uint16)
            return mm
        else:
            raise ValueError(f"Unsupported depth encoding: {img.encoding!r}")


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Catch Ctrl+C to ensure we don't leave a dangling rosbag process
        if node.is_recording:
            node.stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
