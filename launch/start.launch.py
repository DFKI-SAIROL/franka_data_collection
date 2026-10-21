from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_data_collector_node(context):
    config_path = LaunchConfiguration('config_path').perform(context)

    arms = []
    if LaunchConfiguration('spawn_franka_main').perform(context).lower() == 'true':
        arms.append('franka_main')
    if LaunchConfiguration('spawn_franka_left').perform(context).lower() == 'true':
        arms.append('franka_left')
    if LaunchConfiguration('spawn_franka_right').perform(context).lower() == 'true':
        arms.append('franka_right')

    if not arms:
        raise RuntimeError(
            'No arms selected — set at least one of spawn_franka_main, '
            'spawn_franka_left, spawn_franka_right to true.'
        )

    return [Node(
        package='franka_data_collection',
        executable='data_collector_main',
        name='data_collector',
        output='screen',
        parameters=[{'config_path': config_path, 'arms': arms}],
    )]


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_data_collection'), 'config', 'data_collector.yaml'
            ]),
            description="Path to the data collection config file",
        ),
        DeclareLaunchArgument(
            "spawn_franka_main",
            default_value="false",
            description="Record franka main",
        ),
        DeclareLaunchArgument(
            "spawn_franka_left",
            default_value="true",
            description="Record franka left",
        ),
        DeclareLaunchArgument(
            "spawn_franka_right",
            default_value="true",
            description="Record franka right",
        ),
        OpaqueFunction(function=generate_data_collector_node),
    ])
