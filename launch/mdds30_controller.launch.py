from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mdds30_ros2_controller',
            executable='mdds30_controller',
            name='mdds30_controller',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 9600
            }]
        )
    ])