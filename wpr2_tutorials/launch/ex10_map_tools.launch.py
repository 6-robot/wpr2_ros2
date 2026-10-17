import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    tutorials_dir = get_package_share_directory('wpr2_tutorials')
    map_file = os.path.join(tutorials_dir, 'maps', 'map.yaml')
    waypoint_file = os.path.expanduser('~/waypoint.xml')
    config_file = os.path.join(tutorials_dir, 'config', 'nav2_params.yaml')
    rviz_file = os.path.join(tutorials_dir, 'rviz', 'navi_waypoint.rviz')
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': map_file,
            'params_file': config_file,
            'use_sim_time': 'False',
            'use_composition': 'False',
        }.items(),
    )


    wp_edit_cmd = Node(
        package='wp_map_tools',
        executable='wp_edit_node',
        name='wp_edit_node',
        parameters=[{'load': waypoint_file, 'use_sim_time': False}],
        output='screen',
    )

    wp_navi_server_cmd = Node(
        package='wp_map_tools',
        executable='wp_navi_server',
        name='wp_navi_server',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()

    ld.add_action(navigation_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)
    ld.add_action(rviz_cmd)

    return ld
