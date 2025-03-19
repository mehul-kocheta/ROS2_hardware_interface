import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='encoder_reader',
            executable='encoder_serial_node',
            name='encoder_serial_node',
            output='screen'
        ),
        Node(
            package='encoder_reader',
            executable='encoder_odom',
            name='encoder_odom',
            output='screen'
        ),
        Node(
            package='encoder_reader',
            executable='motor_interface',
            name='motor_interface',
            output='screen'
        ),
    ])
