import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_file = os.path.join(
        get_package_share_directory('wp_speech'), 'launch', 'tts.launch.py')
    include_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
    )

    ld = LaunchDescription()

    ld.add_action(include_cmd)

    return ld
