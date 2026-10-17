"""MoveIt planning and trajectory execution against a simulated joint interface."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    main = Path(get_package_share_directory('wpr2_moveit_config')) / 'launch/move_group.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('pose_gui', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(main)),
                                 launch_arguments={'mode': 'demo', 'rviz': LaunchConfiguration('rviz'),
                                                   'pose_gui': LaunchConfiguration('pose_gui'),
                                                   'allow_execution': 'true'}.items()),
    ])
