from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_data_collector_node(context):
    nodes = []

    config_path = LaunchConfiguration('config_path').perform(context)

    nodes.append(Node(
        package='franka_data_collection',
        executable='data_collector_main',
        name='data_collector',
        output='screen',
        parameters=[{'config_path': config_path}],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare argument
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_data_collection'), 'config', 'data_collection_config.yaml'
            ]),
            description="Path to the data collection config file",
        ),
        OpaqueFunction(function=generate_data_collector_node),
    ])