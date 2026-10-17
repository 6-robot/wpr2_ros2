"""Start reusable behaviors only; existing bringup owns the robot and camera."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = Path(get_package_share_directory('wpr2_behaviors'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(package / 'config/behaviors.yaml')),
        DeclareLaunchArgument('start_objects', default_value='true'),
        DeclareLaunchArgument('start_grab', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='wpr2_behaviors', executable='wpr2_objects_3d', name='wpr2_objects_3d',
             output='screen', parameters=[LaunchConfiguration('config')],
             condition=IfCondition(LaunchConfiguration('start_objects'))),
        Node(package='wpr2_behaviors', executable='wpr2_grab', name='wpr2_grab',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('start_grab'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(package / 'rviz/objects.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
