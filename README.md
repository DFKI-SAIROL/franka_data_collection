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
  cameras:
    - name: zed_middle
      type: rgb
  codec:
    rgb: h264
    depth: ffv1
    crf: 23
```

- `topics`: plain ROS topics recorded straight into the rosbag.
- `videos.topic`: the `franka_custom_msgs/RigSnapshot` topic providing the synced camera frames to buffer and encode.

## 1.3. Running

### 1.3.1. Launch the Node

```bash
ros2 launch franka_data_collection start.launch.py
```

> [!NOTE]
> Pass `config_path:=/path/to/your.yaml` to override the default config, and
> `storage_path:=/path/to/datasets` to record somewhere other than the config's `storage_path`
> (keeps machine-specific paths out of the shared config).

### 1.3.2. Start/Stop a Recording

Recording is toggled by calling the trigger service — the first call starts it, the second stops it:

```bash
ros2 service call /data_collector/record_data_trigger std_srvs/srv/Trigger
```

Stopping opens a window to choose the episode's task and label. With `label_dialog:=false` the
node runs headless, for a client that labels the episodes itself: the stopped episode waits
for one of

```bash
# file it below storage_path/<task> (task may be nested, e.g. "<dataset>/<task>"); metadata is
# an optional YAML mapping stored as mcap/episode_info.yaml
ros2 service call /data_collector/finish_episode franka_custom_msgs/srv/FinishEpisode \
  "{task: 'my_dataset/pick-cup', label: 'Good', metadata: 'operator: jane'}"
ros2 service call /data_collector/discard_episode std_srvs/srv/Trigger
```

and a new recording is refused until then. The valid labels are the node's read-only `labels`
parameter (from the config). An episode labeled `garbage_label` (`Garbage`) is filed below
`garbage_dir` (default `<storage_path>/_garbage/<task>`) instead of `<storage_path>/<task>`: kept,
but out of the datasets.

## 1.4. Output Layout

Each triggered episode is written under `storage_path` as:

```
episode_<timestamp>/
├── mcap/                     # rosbag with the configured topics
└── videos/
    ├── timestamps.npy        # int64 ns, one per synced frame
    ├── rgb_<camera_name>.mp4
    └── depth_<camera_name>.mkv   # only when depth cameras are configured
```
