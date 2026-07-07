import os
import signal
import subprocess
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

        config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'data_collector.yaml')

        if not config_file:
            self.get_logger().error('No config_file parameter provided!')
            return

        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

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
        self.camera_fps : flaot = video_cfg['camera_fps']
        self.camera_names: list[str] = [c['name'] for c in video_cfg['cameras']]
        self.codec_config: dict = video_cfg.get('codec', {})

        # 2. State Variables
        self.is_recording = False
        self.record_process = None
        self.episode_path = None

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
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.episode_path = self.storage_path / f'episode_{timestamp}'
 
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
 
        # Drain frames and kick off background encoding
        frames = self.image_buffer.drain()
        self.get_logger().info(f"Encoding {len(frames)} frames to video...")
 
        if frames:
            self._encode_async(frames, self.episode_path)
        else:
            self.get_logger().warn("No camera frames were buffered — skipping video encoding.")
 
        self.is_recording = False
        self.get_logger().info("Recording stopped. Video encoding running in background.")
        return True, f"Stopped recording. Encoding {len(frames)} frames."

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