"""底盘、机器人模型和速度控制面板启动。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    # 运行时解析命名空间和可选 IMU 参数；不指定 imu_pub 时沿用 YAML 配置。
    bringup_dir = get_package_share_directory('wpr2_bringup')
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    frame_prefix = namespace + '/' if namespace else ''
    core_parameters = [LaunchConfiguration('config'), {
        'serial_port': LaunchConfiguration('serial_port'),
        'publish_odom_tf': ParameterValue(LaunchConfiguration('publish_odom_tf'), value_type=bool),
        'odom_frame': frame_prefix + 'odom',
        'base_frame': frame_prefix + 'base_footprint',
        'imu_frame': frame_prefix + 'imu',
    }]
    if LaunchConfiguration('imu_pub').perform(context).strip():
        core_parameters.append({
            'imu_pub': ParameterValue(LaunchConfiguration('imu_pub'), value_type=bool),
        })

    # 底盘和上肢驱动；退出时结束本次启动。
    wpr2_core_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_core',
        namespace=namespace,
        output='screen',
        parameters=core_parameters,
        condition=IfCondition(LaunchConfiguration('start_core')),
        on_exit=[EmitEvent(event=Shutdown(reason='wpr2_core exited'))],
    )

    # 根据模型和关节状态发布 TF。
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]), value_type=str),
            'frame_prefix': frame_prefix,
        }],
    )

    # 不接底盘时可启用，用 YAML 中的零位显示机器人。
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        parameters=[LaunchConfiguration('config')],
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
    )

    # RViz 机器人和传感器显示。
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # 读取手柄；摇杆不变时也按设定频率重复发布。
    joy_cmd = Node(
        package='joy',
        executable='joy_node',
        namespace=namespace,
        parameters=[{
            'device_id': 0,
            'deadzone': 0.12,
            'autorepeat_rate': ParameterValue(
                LaunchConfiguration('joy_autorepeat_rate'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('joystick')),
    )

    # 将手柄输入转换为速度指令。
    js_vel_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_js_vel',
        namespace=namespace,
        parameters=[{
            'joy_timeout': ParameterValue(LaunchConfiguration('joy_timeout'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('joystick')),
    )

    # 图形化速度控制面板。
    vel_panel_cmd = Node(
        package='wpr2_bringup',
        executable='vel_panel.py',
        namespace=namespace,
        condition=IfCondition(LaunchConfiguration('panel')),
    )

    # C1 激光雷达：原始扫描发布到 scan_raw。
    lidar_cmd = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': ParameterValue(LaunchConfiguration('lidar_baudrate'), value_type=int),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_frequency': 10.0,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', 'scan_raw')],
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

    # 过滤机器人自身遮挡，发布 scan。
    lidar_filter_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_lidar_filter',
        output='screen',
        parameters=[{'pub_topic': 'scan'}],
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

    # Femto Bolt 图像和点云，使用厂家驱动的启动入口。
    femto_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'include', 'start_femto_bolt.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )

    ld = LaunchDescription()

    ld.add_action(wpr2_core_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(joy_cmd)
    ld.add_action(js_vel_cmd)
    ld.add_action(vel_panel_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(lidar_filter_cmd)
    ld.add_action(femto_cmd)
    return [ld]


def generate_launch_description():
    bringup_dir = get_package_share_directory('wpr2_bringup')
    description_dir = get_package_share_directory('wpr2_description')

    ld = LaunchDescription()

    # 启动参数：可在命令行用 参数名:=值 覆盖。
    ld.add_action(DeclareLaunchArgument('serial_port', default_value='/dev/ftdi'))
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('start_core', default_value='true'))
    ld.add_action(DeclareLaunchArgument('model', default_value=os.path.join(description_dir, 'urdf', 'wpr2.urdf.xacro')))
    ld.add_action(DeclareLaunchArgument('config', default_value=os.path.join(bringup_dir, 'config', 'wpr2.yaml')))
    ld.add_action(DeclareLaunchArgument('publish_odom_tf', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_joint_state_publisher', default_value='false'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false'))
    ld.add_action(DeclareLaunchArgument('panel', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rvizconfig', default_value=os.path.join(bringup_dir, 'rviz', 'sensor.rviz')))
    ld.add_action(DeclareLaunchArgument('joystick', default_value='false'))
    ld.add_action(DeclareLaunchArgument('joy_autorepeat_rate', default_value='20.0'))
    ld.add_action(DeclareLaunchArgument('joy_timeout', default_value='1.0'))
    ld.add_action(DeclareLaunchArgument('enable_lidar', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_camera', default_value='false'))
    ld.add_action(DeclareLaunchArgument('imu_pub', default_value=''))
    ld.add_action(DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'))
    ld.add_action(DeclareLaunchArgument('lidar_baudrate', default_value='460800'))

    # 延迟解析运行时参数，具体启动项均在上方逐项列出。
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
