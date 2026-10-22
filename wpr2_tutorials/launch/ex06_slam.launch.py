import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tutorials_dir = get_package_share_directory('wpr2_tutorials')
    config_file = os.path.join(tutorials_dir, 'config', 'slam.yaml')
    rviz_file = os.path.join(tutorials_dir, 'rviz', 'slam.rviz')


    slam_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file, {'use_sim_time': False}],
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()

    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)

    return ld
