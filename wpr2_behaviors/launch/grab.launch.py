"""Coordinate-driven grasp controller; no camera or hardware is started here."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = Path(get_package_share_directory('wpr2_behaviors')) / 'launch/behaviors.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
        launch_arguments={'start_objects': 'false'}.items())])
