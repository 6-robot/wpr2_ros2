import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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
        get_package_share_directory('wpr2_tutorials'), 'rviz', 'sensor.rviz')


    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', model_file]), value_type=str),
            'use_sim_time': False,
        }],
    )


    joint_state_publisher_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[config_file, {'use_sim_time': False}],
    )


    femto_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'include', 'start_femto_bolt.launch.py')),
    )


    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(femto_cmd)
    ld.add_action(rviz_cmd)

    return ld
