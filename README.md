- [1. Franka Data Collection](#1-franka-data-collection)
  - [1.1. Overview](#11-overview)
  - [1.2. Configuration](#12-configuration)
  - [1.3. Running](#13-running)
    - [1.3.1. Launch the Node](#131-launch-the-node)
    - [1.3.2. Start/Stop a Recording](#132-startstop-a-recording)
  - [1.4. Output Layout](#14-output-layout)

# 1. Franka Data Collection

A ROS 2 package that records synchronized robot and camera data.

## 1.1. Overview

The `data_collector` node buffers synced RGB(-D) frames from a `RigSnapshot` topic in memory and, on trigger, simultaneously:
- records the configured ROS topics (joint states, poses, etc.) to an `mcap` rosbag via `ros2 bag record`, and
- encodes the buffered camera frames to per-camera video files via `ffmpeg` in a background thread, so the next episode can start immediately without waiting for encoding to finish.

## 1.2. Configuration

All settings live in [`config/data_collector.yaml`](config/data_collector.yaml):

```yaml
storage_path: "/path/to/dataset"

topics:
  - "/franka_right/franka/joint_states"
  - "/franka_right/target_pose"
  # ...

videos:
  topic: "/zed_rig/synced_raw_snapshot"
  camera_fps: 30.0
  extrinsics:
    zed_middle: "/path/to/measured/zed_middle.yaml"
    zed_right: "/path/to/measured/zed_right.yaml"
  cameras:
    - name: zed_middle
      type: rgb
    - name: zed_right
      type: rgb
    - name: zed_right_arm
      type: rgb
  codec:
    rgb: h264
    depth: ffv1
    crf: 23
```

- `topics`: plain ROS topics recorded straight into the rosbag.
- `videos.topic`: the `franka_custom_msgs/RigSnapshot` topic providing the synced camera frames to buffer and encode.
- `videos.extrinsics.<camera>`: paths to measured fixed camera transforms. Every camera with a non-empty entry is treated as calibrated: recording refuses to start until its file exists, and its extrinsic and CameraInfo are stored with each episode. Cameras with an empty or no entry are recorded without calibration; at least one entry is required. Relative paths are resolved from the config file directory.

Each extrinsic YAML must describe `T_parent_child`, mapping points from the camera optical frame into the robot base frame, with metres and ROS `xyzw` quaternion order:

```yaml
parent_frame: franka_right_link0  # replace with the actual calibrated base frame
child_frame: zed_middle_left_camera_optical_frame  # match CameraInfo.header.frame_id
translation_m: [0.0, 0.0, 0.0]  # replace with measured values
quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]  # replace with measured values
```

The collector checks the CameraInfo frame ID and dimensions against the calibration and RGB stream. These example numeric values are only a format illustration, not a camera calibration.

## 1.3. Running

### 1.3.1. Launch the Node

```bash
ros2 launch franka_data_collection start.launch.py
```

> [!NOTE]
> Pass `config_path:=/path/to/your.yaml` to override the default config.

### 1.3.2. Start/Stop a Recording

Recording is toggled by calling the trigger service — the first call starts it, the second stops it:

```bash
ros2 service call /data_collector/record_data_trigger std_srvs/srv/Trigger
```

## 1.4. Output Layout

Each triggered episode is written under `storage_path` as:

```
episode_<timestamp>/
├── mcap/                     # rosbag with the configured topics
├── calibration/              # for every camera with a videos.extrinsics entry:
│   ├── <camera>_camera_info.yaml
│   ├── <camera>_cam_K.txt     # 3x3 intrinsic matrix (camera_matrix_file of 6dposeestimationtracking)
│   ├── <camera>_extrinsic.yaml
│   └── depth_format.yaml
└── videos/
    ├── timestamps.npy        # int64 ns, one per synced frame
    ├── rgb_<camera_name>.mp4
    └── depth_<camera_name>.mkv   # only when depth cameras are configured
```
