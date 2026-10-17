"""C1 雷达驱动和扫描滤波节点。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # C1 雷达：将原始扫描输出到 scan_raw。
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
    )

    # 过滤机器人自身遮挡，发布 scan。
    lidar_filter_cmd = Node(
        package='wpr2_bringup',
        executable='wpr2_lidar_filter',
        output='screen',
        parameters=[{'pub_topic': 'scan'}],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'))
    ld.add_action(DeclareLaunchArgument('lidar_baudrate', default_value='460800'))
    ld.add_action(lidar_cmd)
    ld.add_action(lidar_filter_cmd)

    return ld
