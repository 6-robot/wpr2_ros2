import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, OpaqueFunction
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_files(context, map_file, waypoint_file):

    if not os.path.isfile(map_file):
        raise RuntimeError('Map YAML does not exist: ' + map_file)
    with open(map_file, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    image = data.get('image') if isinstance(data, dict) else None
    if not isinstance(image, str) or not image or not os.path.isfile(
            os.path.join(os.path.dirname(map_file), image)):
        raise RuntimeError('Map image is missing or invalid in ' + map_file)
    config_file = os.path.join(
        get_package_share_directory('wpr2_tutorials'), 'config', 'nav2_params.yaml')
    if not os.path.isfile(config_file):
        raise RuntimeError('Nav2 parameter file does not exist: ' + config_file)
    if not os.path.isfile(waypoint_file):
        raise RuntimeError('Waypoint XML does not exist: ' + waypoint_file)

    behaviors_file = os.path.join(
        get_package_share_directory('wpr2_behaviors'), 'config', 'behaviors.yaml')
    if not os.path.isfile(behaviors_file):
        raise RuntimeError('Behavior parameter file does not exist: ' + behaviors_file)


def generate_launch_description():
    tutorials_dir = get_package_share_directory('wpr2_tutorials')
    map_file = os.path.join(tutorials_dir, 'maps', 'map.yaml')
    waypoint_file = os.path.expanduser('~/waypoint.xml')
    bringup_dir = get_package_share_directory('wpr2_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'wpr2.yaml')
    model_file = os.path.join(
        get_package_share_directory('wpr2_description'), 'urdf', 'wpr2.urdf.xacro')
    rviz_file = os.path.join(
        get_package_share_directory('wpr2_tutorials'), 'rviz', 'fetch.rviz')
    behaviors_file = os.path.join(
        get_package_share_directory('wpr2_behaviors'), 'config', 'behaviors.yaml')

    nav_params_file = os.path.join(
        get_package_share_directory('wpr2_tutorials'), 'config', 'nav2_params.yaml')
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    check_files_cmd = OpaqueFunction(
        function=validate_files, args=[map_file, waypoint_file])


    wpr2_core_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_core',
        name='wpr2_core',
        output='screen',
        parameters=[config_file, {
            'serial_port': '/dev/ftdi',
            'publish_odom_tf': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'imu_frame': 'imu',
            'use_sim_time': False,
        }],
        on_exit=[EmitEvent(event=Shutdown(reason='wpr2_core exited'))],
    )


    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', model_file]), value_type=str),
            'use_sim_time': False,
        }],
    )


    lidar_cmd = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_frequency': 10.0,
            'scan_mode': 'Standard',
            'use_sim_time': False,
        }],
        remappings=[('scan', 'scan_raw')],
    )


    lidar_filter_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_lidar_filter',
        output='screen',
        parameters=[{'pub_topic': '/scan', 'use_sim_time': False}],
    )


    femto_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'include', 'start_femto_bolt.launch.py')),
    )


    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': map_file,
            'params_file': nav_params_file,
            'use_sim_time': 'False',
            'use_composition': 'False',
        }.items(),
    )

    wp_edit_cmd = Node(
        package='wp_map_tools',
        executable='wp_edit_node',
        name='wp_edit_node',
        output='screen',
        parameters=[{'load': waypoint_file, 'use_sim_time': False}],
    )
    wp_navi_server_cmd = Node(
        package='wp_map_tools',
        executable='wp_navi_server',
        name='wp_navi_server',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )


    objects_3d_cmd = Node(
        package='wpr2_behaviors',
        executable='wpr2_objects_3d',
        name='wpr2_objects_3d',
        output='screen',
        parameters=[behaviors_file, {'use_sim_time': False, 'auto_start': False}],
    )


    grab_cmd = Node(
        package='wpr2_behaviors',
        executable='wpr2_grab',
        name='wpr2_grab',
        output='screen',
        parameters=[behaviors_file, {'use_sim_time': False, 'auto_start': False}],
    )


    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()

    ld.add_action(check_files_cmd)
    ld.add_action(wpr2_core_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(lidar_filter_cmd)
    ld.add_action(femto_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)
    ld.add_action(objects_3d_cmd)
    ld.add_action(grab_cmd)
    ld.add_action(rviz_cmd)

    return ld
