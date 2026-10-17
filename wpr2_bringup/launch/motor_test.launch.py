"""逐个测试底盘电机；实机会转动。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 直接通过串口测试电机，不同时启动 wpr2_core。
    motor_test_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_test_motors',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('serial_port', default_value='/dev/ftdi'))
    ld.add_action(motor_test_cmd)

    return ld
