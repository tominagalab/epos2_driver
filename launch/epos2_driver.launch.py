from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='epos2_driver',
            executable='epos2_driver_node',
            name='epos2_driver',
            output='screen',
            parameters=[{
                'device_name': 'EPOS2',
                'protocol_stack_name': 'MAXON SERIAL V2',
                'interface_name': 'USB',
                'port_name': 'USB0',
                'node_id': 1,
                'baudrate': 1000000,
                'timeout_ms': 500,
                'profile_acceleration': 1000,
                'profile_deceleration': 1000,
            }],
        ),
    ])
