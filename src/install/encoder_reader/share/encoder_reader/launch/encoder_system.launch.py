import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare the serial_port launch argument
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for motor interface'
        ),
        # Encoder serial node (unchanged)
        Node(
            package='encoder_reader',
            executable='encoder_serial_node',
            name='encoder_serial_node',
            output='screen'
        ),
        # Encoder odom node (unchanged)
        Node(
            package='encoder_reader',
            executable='encoder_odom',
            name='encoder_odom',
            output='screen'
        ),
        # Motor interface node with serial_port parameter
        Node(
            package='encoder_reader',
            executable='motor_interface',
            name='motor_interface',
            output='screen',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}]
        ),
    ])