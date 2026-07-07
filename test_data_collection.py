#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from franka_custom_msgs.msg import RigSnapshot
from std_msgs.msg import Header
import subprocess
import time
import threading
import numpy as np
import sys


class TestDataCollector(Node):
    def __init__(self):
        super().__init__('test_data_collector')
        
        # Service client for trigger
        self.trigger_client = self.create_client(Trigger, '/data_collector/record_data_trigger')
        
        # Publishers
        self.pub_franka_js = self.create_publisher(JointState, '/franka_right/franka/joint_states', 10)
        self.pub_gripper_js = self.create_publisher(JointState, '/franka_right/franka_gripper/joint_states', 10)
        self.pub_target_pose = self.create_publisher(PoseStamped, '/franka_right/target_pose', 10)
        self.pub_camera = self.create_publisher(RigSnapshot, '/zed_rig/synced_raw_snapshot', 10)
        
        # Timer for publishing data (30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.data_count = 0

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # 1. Franka JointStates
        js_msg = JointState()
        js_msg.header.stamp = now
        js_msg.name = [f'joint_{i}' for i in range(1, 8)]
        js_msg.position = [0.1 * (self.data_count % 10)] * 7
        js_msg.velocity = [0.01 * (self.data_count % 10)] * 7
        js_msg.effort = [0.5] * 7
        self.pub_franka_js.publish(js_msg)
        
        # 2. Gripper JointStates
        gjs_msg = JointState()
        gjs_msg.header.stamp = now
        gjs_msg.name = ['finger_joint1']
        gjs_msg.position = [0.04]
        gjs_msg.velocity = [0.0]
        gjs_msg.effort = [10.0]
        self.pub_gripper_js.publish(gjs_msg)
        
        # 3. Camera RigSnapshot
        rig_msg = RigSnapshot()
        rig_msg.header.stamp = now
        rig_msg.camera_names = ['cam_1', 'cam_2']
        
        # Dummy Image
        dummy_img = Image()
        dummy_img.header.stamp = now
        dummy_img.height = 480
        dummy_img.width = 640
        dummy_img.encoding = 'rgb8'
        dummy_img.is_bigendian = 0
        dummy_img.step = 640 * 3
        # Use simple constant array to save time
        img_data = np.full((480, 640, 3), self.data_count % 255, dtype=np.uint8).tobytes()
        dummy_img.data = img_data
        
        rig_msg.rgbs = [dummy_img, dummy_img]
        self.pub_camera.publish(rig_msg)
        
        # 4. Target Pose (Also acts as the trigger for recording)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = 0.5
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.5
        pose_msg.pose.orientation.w = 1.0
        self.pub_target_pose.publish(pose_msg)
        
        self.data_count += 1

def main():
    rclpy.init()
    
    # Launch data collector node in subprocess
    print("Launching data collector node...")
    process = subprocess.Popen(
        ['ros2', 'run', 'franka_data_collection', 'data_collector_main'],
        stdout=sys.stdout,
        stderr=sys.stderr
    )
    
    # Wait for node to initialize
    time.sleep(3.0)
    
    tester_node = TestDataCollector()
    
    # Run node in a separate thread so we can script the sequence
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tester_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        tester_node.get_logger().info('Waiting for /data_collector/record_data_trigger service...')
        # Wait up to 5 seconds
        if not tester_node.trigger_client.wait_for_service(timeout_sec=5.0):
            tester_node.get_logger().error('Service not available, exiting.')
            return
            
        tester_node.get_logger().info('Calling trigger to START recording...')
        req = Trigger.Request()
        future = tester_node.trigger_client.call_async(req)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            tester_node.get_logger().info(f'Start response: {future.result().success}, {future.result().message}')
        
        tester_node.get_logger().info('Publishing dummy data for 10 seconds...')
        time.sleep(10.0)
        
        tester_node.get_logger().info('Calling trigger to STOP recording...')
        future = tester_node.trigger_client.call_async(req)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            tester_node.get_logger().info(f'Stop response: {future.result().success}, {future.result().message}')
            
    except KeyboardInterrupt:
        pass
    finally:
        tester_node.get_logger().info('Terminating data collector node...')
        process.terminate()
        process.wait()
        
        executor.shutdown()
        tester_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
