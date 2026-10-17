"""Start MoveIt 2 with the WPR2 model and either simulated or real joint I/O."""

from ctypes import CDLL
from pathlib import Path

from ament_index_python.packages import (PackageNotFoundError, get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('demo', 'real'):
        raise ValueError('mode must be demo or real')
    missing = []
    for package in ('moveit_planners_ompl', 'moveit_simple_controller_manager'):
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            missing.append('ros-humble-' + package.replace('_', '-'))
    if missing:
        raise RuntimeError('Missing MoveIt plugins: ' + ', '.join(missing) +
                           '. Install them with wpr2_bringup/scripts/install_deps.sh.')
    plugin = Path(get_package_prefix('moveit_planners_ompl')) / 'lib/libmoveit_ompl_planner_plugin.so'
    try:
        CDLL(str(plugin))
    except OSError as error:
        raise RuntimeError('OMPL plugin cannot load: ' + str(error) +
                           '. Upgrade installed MoveIt packages together with '
                           'wpr2_bringup/scripts/install_deps.sh.') from error
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1')
    allow_execution = LaunchConfiguration('allow_execution').perform(context).lower() in ('true', '1')
    share = Path(get_package_share_directory('wpr2_moveit_config'))
    bringup = Path(get_package_share_directory('wpr2_bringup'))
    moveit = (MoveItConfigsBuilder('wpr2')
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .trajectory_execution(file_path='config/moveit_controllers.yaml',
                                    moveit_manage_controllers=False)
              .to_moveit_configs())
    parameters = moveit.to_dict()
    parameters.update({
        'allow_trajectory_execution': allow_execution,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    })
    actions = [Node(package='moveit_ros_move_group', executable='move_group',
                    output='screen', parameters=[parameters])]

    if mode == 'real':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/base.launch.py')),
            launch_arguments={'serial_port': LaunchConfiguration('serial_port'),
                              'rviz': 'false', 'start_core': 'true'}.items()))
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            arguments=['--frame-id', 'world', '--child-frame-id', 'odom']))
    else:
        actions.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 output='screen', parameters=[moveit.robot_description]),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--frame-id', 'world', '--child-frame-id', 'base_footprint']),
        ])

    if mode == 'demo' or allow_execution:
        actions.append(Node(package='wpr2_moveit_config', executable='trajectory_to_wpr2.py',
                            output='screen', parameters=[{'fake_hardware': mode == 'demo'}]))
    if use_rviz:
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', str(share / 'config/moveit.rviz')],
                            parameters=[moveit.robot_description,
                                        moveit.robot_description_semantic,
                                        moveit.robot_description_kinematics,
                                        moveit.planning_pipelines,
                                        moveit.joint_limits]))
    if LaunchConfiguration('pose_gui').perform(context).lower() in ('true', '1'):
        actions.append(Node(package='wpr2_moveit_config', executable='get_pose.py',
                            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='demo'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('allow_execution', default_value='false'),
        DeclareLaunchArgument('pose_gui', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ftdi'),
        OpaqueFunction(function=setup),
    ])
