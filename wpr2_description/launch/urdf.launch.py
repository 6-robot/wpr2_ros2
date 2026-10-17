"""Display the WPR2 model, or publish its description and TF without a GUI."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def launch_nodes(context):
    share = Path(get_package_share_directory('wpr2_description'))
    model = LaunchConfiguration('model').perform(context)
    description = xacro.process_file(model).toxml()
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    joint_parameters = [
        str(share / 'config' / 'joint_states.yaml'),
        {'use_sim_time': use_sim_time},
    ]
    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            parameters=joint_parameters, condition=IfCondition(LaunchConfiguration('gui')),
            output='screen',
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            parameters=joint_parameters, condition=UnlessCondition(LaunchConfiguration('gui')),
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen',
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('wpr2_description'))
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(share / 'urdf' / 'wpr2.urdf.xacro'),
                              description='Absolute path to the robot Xacro or URDF.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Use sliders instead of fixed initial joint positions.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2.'),
        DeclareLaunchArgument('rvizconfig', default_value=str(share / 'rviz' / 'urdf.rviz')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])
