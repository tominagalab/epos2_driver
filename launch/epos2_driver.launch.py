from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                FindPackageShare('epos2_driver'), 'config', 'config.yaml',
            ]),
            description='Path to the EPOS driver ROS parameters file',
        ),
        Node(
            package='epos2_driver',
            executable='epos_driver_node',
            name='epos_driver_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
