import json
import os
import subprocess
import time
from pathlib import PurePosixPath

import rclpy
import yaml
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from franka_custom_msgs.srv import FinishEpisode

# Episodes are recorded into <storage_path>/_unlabeled and moved to <storage_path>/<task> once labeled.
from franka_data_collection.episode_writer import (
    EpisodeWriter, load_collector_config,
)

from tkinter import StringVar, Tk
from tkinter import font as tkfont
from tkinter import ttk


def display_zoom() -> float:
    """The desktop zoom (GNOME on X11 publishes it as Xft.dpi = 96 x zoom, e.g. 192 at 200 %).
    Read when a dialog opens, so a changed zoom applies to the next one; 1.0 if unknown."""
    try:
        out = subprocess.run(['xrdb', '-query'], capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            if line.startswith('Xft.dpi:'):
                return max(0.5, min(4.0, float(line.split(':', 1)[1]) / 96.0))
    except Exception:
        pass
    return 1.0


def apply_zoom(master: Tk, zoom: float) -> None:
    """Scale a Tk window by the desktop zoom: Tk ignores it and draws at the X server's DPI."""
    master.tk.call('tk', 'scaling', 96.0 * zoom / 72.0)  # pixels per point: sizes given in points
    for name in tkfont.names(master):  # Tk's built-in fonts are sized in pixels (negative): scale them too
        f = tkfont.nametofont(name, root=master)
        if f.cget('size') < 0:
            f.configure(size=round(f.cget('size') * zoom))


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        self.declare_parameter('config_path', '')
        self.declare_parameter('arms', ['franka_right'])

        config_file = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_file or not os.path.isfile(config_file):
            fallback = os.path.join(os.path.dirname(__file__), '..', 'config', 'data_collector.yaml')
            if config_file:
                self.get_logger().warn(f'Config not found at {config_file}, falling back to {fallback}')
            config_file = fallback

        # Bag, camera frames, labeling files and the move to the task folder: EpisodeWriter,
        # shared by every writer so they all produce the same layout.
        arms = list(self.get_parameter('arms').get_parameter_value().string_array_value)
        self.writer = EpisodeWriter(self, load_collector_config(config_file), arms)
        self.get_logger().info(
            f'Recording to {self.writer.unlabeled_dir}; labeled episodes are moved to '
            f'{self.writer.dataset_root}/<task>'
            + (f', {self.writer.garbage_label} ones to {self.writer.garbage_root}/<task>.'
               if self.writer.garbage_label else '.')
        )
        self.get_logger().info(f'Recording arms: {arms}')

        # label_dialog false = headless: stopping opens no window, and the stopped episode waits
        # for ~/finish_episode (task, label, metadata) or ~/discard_episode, e.g. from Franka Studio.
        self.declare_parameter('label_dialog', True)
        self.label_dialog = self.get_parameter('label_dialog').get_parameter_value().bool_value
        # The labels a finish_episode request may use, for clients to offer
        self.declare_parameter('labels', self.writer.labels, ParameterDescriptor(read_only=True))

        # 3. Services
        self.srv_trigger = self.create_service(
            Trigger, '~/record_data_trigger', self.recording_callback
        )
        self.srv_finish = self.create_service(FinishEpisode, '~/finish_episode', self.finish_callback)
        self.srv_discard = self.create_service(Trigger, '~/discard_episode', self.discard_callback)

        # ~/status (latched): JSON {state: idle | recording | stopped (waits for its label),
        # episode, stamp}, on every change. Lets a client follow recordings toggled by anyone else
        # (the teleop's A button calls record_data_trigger directly).
        self.status_pub = self.create_publisher(String, '~/status', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish_status()

        mode ='labelling window' if self.label_dialog else 'headless (finish_episode / discard_episode)'
        self.get_logger().info(f'Data collector ready ({mode}). Waiting for trigger...')

    @property
    def is_recording(self) -> bool:
        return self.writer.is_recording

    def publish_status(self):
        w = self.writer
        state = 'recording' if w.is_recording else 'stopped' if w.pending else 'idle'
        self.status_pub.publish(String(data=json.dumps(
            {'state': state, 'episode': str(w.episode_path or ''), 'stamp': time.time()})))

    def recording_callback(self, request, response):
        """Toggles the recording state."""
        if not self.is_recording:
            success, message = self.start_recording()
        else:
            success, message = self.stop_recording()
            if self.label_dialog:
                self.label_episode()

        response.success = success
        response.message = message
        return response

    def finish_callback(self, request, response):
        """Labels the stopped episode and files it below <storage_path>/<task> (headless)."""
        try:
            task = self._task_folder(request.task)
            if request.label not in self.writer.labels:
                raise ValueError(f'Unknown label {request.label!r} (one of {", ".join(self.writer.labels)}).')
            metadata = yaml.safe_load(request.metadata) if request.metadata.strip() else None
            if metadata is not None and not isinstance(metadata, dict):
                raise ValueError('metadata must be a YAML mapping.')
            target = self.writer.finish(task, request.label,
                                        {'episode_info.yaml': metadata} if metadata else None)
        except (ValueError, RuntimeError, yaml.YAMLError) as e:
            response.success, response.message = False, str(e)
            return response
        self.publish_status()
        response.success, response.path = True, str(target)
        response.message = f'Labeled {request.label}; moving to {target} once its videos are written.'
        return response

    def discard_callback(self, request, response):
        """Deletes the stopped, not yet labeled episode (once its videos are written)."""
        try:
            path = self.writer.discard()
        except RuntimeError as e:
            response.success, response.message = False, str(e)
            return response
        self.publish_status()
        response.success, response.message = True, f'Discarding {path}.'
        return response

    @staticmethod
    def _task_folder(task: str) -> str:
        """The task as a relative folder below storage_path; ValueError if it would leave it."""
        path = PurePosixPath(task.strip())
        if not path.parts or path.is_absolute() or any(p in ('.', '..') for p in path.parts):
            raise ValueError(f'Invalid task folder {task!r}: a relative path below storage_path.')
        return str(path)

    def start_recording(self):
        # Headless, nobody else would ever label the stopped episode: it must not be replaced
        if not self.label_dialog and self.writer.pending:
            return False, f'Label or discard the last episode first ({self.writer.episode_path}).'
        try:
            path = self.writer.start()
        except Exception as e:
            return False, str(e)
        self.publish_status()
        return True, f"Started recording to {path}"

    def stop_recording(self):
        try:
            n_cameras = self.writer.stop()
        except RuntimeError as e:
            return False, str(e)
        self.publish_status()
        return True, f"Stopped recording. Encoding the videos of {n_cameras} cameras."

    def label_episode(self):
        """ Open a simple GUI to label the recorded episode with its task and outcome. """
        print("Labeling episode...", flush=True)
        episode_path = self.writer.episode_path

        master = Tk()
        master.title("Label Episode")
        zoom = display_zoom()
        apply_zoom(master, zoom)

        def px(n):  # paddings follow the zoom as well
            return round(n * zoom)

        # Submit lives in its own bar, packed first at the bottom, so it stays visible even
        # when the window is smaller than its content.
        bottom = ttk.Frame(master, padding=(px(25), px(10), px(25), px(20)))
        bottom.pack(side="bottom", fill="x")
        main_frame = ttk.Frame(master, padding=(px(25), px(25), px(25), 0))
        main_frame.pack(side="top", fill="both", expand=True)

        ttk.Label(main_frame, text="Please Label the Recorded Episode", font=("Sans-Serif", 16, "bold")).pack(anchor="w", pady=(0, px(15)))
        ttk.Label(main_frame, text="Please select the task:", font=("Sans-Serif", 11)).pack(anchor="w", pady=(px(5), px(5)))

        # No preselection: the task decides the folder the episode lands in,
        # so it must be an explicit choice every time.
        task_var = StringVar(value="")
        task_grid = ttk.Frame(main_frame)
        task_grid.pack(anchor="w", padx=px(10))
        columns = 3  # the task list is long: a grid keeps the dialog compact
        for i, task in enumerate(self.writer.tasks):
            ttk.Radiobutton(task_grid, text=task, variable=task_var, value=task).grid(
                row=i // columns, column=i % columns, sticky="w", padx=(0, px(20)), pady=px(3))

        ttk.Separator(main_frame, orient="horizontal").pack(fill="x", pady=px(15))
        ttk.Label(main_frame, text="Please select the label for the task:", font=("Sans-Serif", 11)).pack(anchor="w", pady=(px(5), px(5)))

        label_var = StringVar(value="")
        label_row = ttk.Frame(main_frame)
        label_row.pack(anchor="w", padx=px(10))
        for label in self.writer.labels:
            ttk.Radiobutton(label_row, text=label, variable=label_var, value=label).pack(side="left", padx=(0, px(20)), pady=px(3))

        submit = ttk.Button(
            bottom, text="Submit", state="disabled",
            command=lambda: self.fill_metadata(master, task_var.get(), label_var.get()),
        )
        submit.pack(anchor="center", ipadx=px(15), ipady=px(5))

        # Size the window to its content (no fixed geometry that cuts off widgets), capped to the screen
        master.update_idletasks()
        width = min(master.winfo_reqwidth(), master.winfo_screenwidth() - 40)
        height = min(master.winfo_reqheight(), master.winfo_screenheight() - 80)
        master.geometry(f"{width}x{height}")
        master.minsize(min(width, px(400)), min(height, px(300)))

        def _update_submit(*_):
            submit.configure(state="normal" if task_var.get() and label_var.get() else "disabled")
        task_var.trace_add("write", _update_submit)
        label_var.trace_add("write", _update_submit)

        def _on_close():
            self.get_logger().warn(f"Episode not labeled — left in {episode_path}")
            master.destroy()
        master.protocol("WM_DELETE_WINDOW", _on_close)

        master.mainloop()

    def fill_metadata(self, master, task, label):
        """
        Save the metadata for the recorded episode to a YAML file and move the
        episode into its task folder."""

        print(f"Submitted -> Task: {task} | Label: {label}", flush=True)
        master.destroy()
        self.writer.finish(task, label)
        self.publish_status()
        return 0

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