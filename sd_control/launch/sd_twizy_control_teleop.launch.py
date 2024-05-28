import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_button',
            default_value='0',
            description='Enable button'
        ),
        DeclareLaunchArgument(
            'throttle_axis',
            default_value='1',
            description='Throttle axis'
        ),
        DeclareLaunchArgument(
            'steer_axis',
            default_value='0',
            description='Steer axis'
        ),
        
        Node(
            package='joy',
            executable='joy_node',
            name='joy'
        ),
        
        Node(
            package='sd_control',
            executable='sd_teleop_joy',
            name='sd_teleop_joy',
            parameters=[{
                'enable_button': LaunchConfiguration('enable_button'),
                'throttle_axis': LaunchConfiguration('throttle_axis'),
                'steer_axis': LaunchConfiguration('steer_axis')
            }]
        )
    ])
