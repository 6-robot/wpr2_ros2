"""在独立的 real 和 plan 坐标树中对照实机关节与计划关节。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('wpr2_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'wpr2.yaml')
    model_file = os.path.join(
        get_package_share_directory('wpr2_description'), 'urdf', 'wpr2.urdf.xacro')
    model = ParameterValue(Command(['xacro ', model_file]), value_type=str)
    rviz_file = os.path.join(bringup_dir, 'rviz', 'plan.rviz')

    # 实机驱动：只发布 real 坐标树内的关节状态。
    wpr2_core_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_core',
        namespace='real',
        output='screen',
        parameters=[config_file, {
            'serial_port': LaunchConfiguration('serial_port'),
            'publish_odom_tf': False,
            'odom_frame': 'real/odom',
            'base_frame': 'real/base_footprint',
        }],
        condition=IfCondition(LaunchConfiguration('start_core')),
        on_exit=[EmitEvent(event=Shutdown(reason='wpr2_core exited'))],
    )

    # 计划关节控制面板，发布 plan/joint_states。
    joints_panel_cmd = Node(
        package='wpr2_bringup',
        executable='joints_panel.py',
        namespace='plan',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # 关闭面板时，用配置中的零位发布计划关节状态。
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='plan',
        parameters=[config_file],
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    # 将面板中的计划关节值转换成实机控制指令。
    arms_test_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_arms_test',
        output='screen',
    )

    # 在 RViz 中对照显示两套模型。
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # 实机模型 TF。
    real_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='real',
        parameters=[{'robot_description': model, 'frame_prefix': 'real/'}],
    )

    # 固定实机模型在 world 中的位置。
    world_to_real_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_real',
        arguments=['--y', '0', '--frame-id', 'world',
                   '--child-frame-id', 'real/base_footprint'],
    )

    # 计划模型 TF。
    plan_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='plan',
        parameters=[{'robot_description': model, 'frame_prefix': 'plan/'}],
    )

    # 将计划模型沿 y 轴平移 1 米，便于与实机模型对照。
    world_to_plan_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_plan',
        arguments=['--y', '1', '--frame-id', 'world',
                   '--child-frame-id', 'plan/base_footprint'],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('serial_port', default_value='/dev/ftdi'))
    ld.add_action(DeclareLaunchArgument('start_core', default_value='true'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='true'))

    ld.add_action(wpr2_core_cmd)
    ld.add_action(joints_panel_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(arms_test_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(real_state_publisher_cmd)
    ld.add_action(world_to_real_cmd)
    ld.add_action(plan_state_publisher_cmd)
    ld.add_action(world_to_plan_cmd)

    return ld
