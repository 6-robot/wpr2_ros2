"""Femto Bolt 图像和彩色点云启动。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 调用 Orbbec 官方 Femto Bolt 驱动，保持原生图像、TF 和点云话题。
    femto_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orbbec_camera'), 'launch', 'femto_bolt.launch.py'])),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_number': LaunchConfiguration('serial_number'),
            'enable_point_cloud': LaunchConfiguration('enable_point_cloud'),
            'enable_colored_point_cloud': LaunchConfiguration('enable_colored_point_cloud'),
            'depth_registration': LaunchConfiguration('depth_registration'),
            'color_width': LaunchConfiguration('color_width'),
            'color_height': LaunchConfiguration('color_height'),
            'color_fps': LaunchConfiguration('color_fps'),
            'depth_width': LaunchConfiguration('depth_width'),
            'depth_height': LaunchConfiguration('depth_height'),
            'depth_fps': LaunchConfiguration('depth_fps'),
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_ir': LaunchConfiguration('enable_ir'),
            'align_mode': LaunchConfiguration('align_mode'),
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('camera_name', default_value='camera'))
    ld.add_action(DeclareLaunchArgument('serial_number', default_value=''))
    ld.add_action(DeclareLaunchArgument('enable_point_cloud', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_colored_point_cloud', default_value='true'))
    ld.add_action(DeclareLaunchArgument('depth_registration', default_value='true'))
    ld.add_action(DeclareLaunchArgument('color_width', default_value='1280'))
    ld.add_action(DeclareLaunchArgument('color_height', default_value='720'))
    ld.add_action(DeclareLaunchArgument('color_fps', default_value='30'))
    ld.add_action(DeclareLaunchArgument('depth_width', default_value='640'))
    ld.add_action(DeclareLaunchArgument('depth_height', default_value='576'))
    ld.add_action(DeclareLaunchArgument('depth_fps', default_value='30'))
    ld.add_action(DeclareLaunchArgument('enable_color', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_depth', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_ir', default_value='true'))
    ld.add_action(DeclareLaunchArgument('align_mode', default_value='SW'))
    ld.add_action(femto_cmd)

    return ld
