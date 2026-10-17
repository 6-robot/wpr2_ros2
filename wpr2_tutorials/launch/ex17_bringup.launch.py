import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('wpr2_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'wpr2.yaml')
    model_file = os.path.join(
        get_package_share_directory('wpr2_description'), 'urdf', 'wpr2.urdf.xacro')
    rviz_file = os.path.join(
        get_package_share_directory('wpr2_behaviors'), 'rviz', 'objects.rviz')
    behaviors_file = os.path.join(
        get_package_share_directory('wpr2_behaviors'), 'config', 'behaviors.yaml')


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


    objects_3d_cmd = Node(
        package='wpr2_behaviors',
        executable='wpr2_objects_3d',
        name='wpr2_objects_3d',
        output='screen',
        parameters=[behaviors_file, {'use_sim_time': False, 'auto_start': True}],
    )


    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()

    ld.add_action(wpr2_core_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(lidar_filter_cmd)
    ld.add_action(femto_cmd)
    ld.add_action(objects_3d_cmd)
    ld.add_action(rviz_cmd)

    return ld
