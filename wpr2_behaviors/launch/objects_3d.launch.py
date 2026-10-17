"""Standalone detection, with the same algorithm and command protocol as WPB."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(get_package_share_directory('wpr2_behaviors'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(package / 'config/behaviors.yaml')),
        DeclareLaunchArgument('auto_start', default_value='false'),
        Node(package='wpr2_behaviors', executable='wpr2_objects_3d', name='wpr2_objects_3d',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool)}]),
    ])
