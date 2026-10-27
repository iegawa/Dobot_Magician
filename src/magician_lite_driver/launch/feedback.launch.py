from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Dobot Magician Lite'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    rate_arg = DeclareLaunchArgument(
        'poll_rate_hz',
        default_value='10.0',
        description='Polling frequency in Hz'
    )

    speed_arg = DeclareLaunchArgument(
        'speed_ratio',
        default_value='20.0',
        description='Motion speed ratio percentage (default 20% for safety)'
    )

    feedback_node = Node(
        package='magician_lite_driver',
        executable='feedback_node',
        name='magician_lite_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'poll_rate_hz': LaunchConfiguration('poll_rate_hz'),
            'speed_ratio': LaunchConfiguration('speed_ratio'),
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        rate_arg,
        speed_arg,
        feedback_node
    ])
