"""Run ex20's real detector and grabber with fast simulated-robot timing only."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import normalize_parameters


def main():
    context = LaunchContext()
    source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('wpr2_tutorials'), 'launch', 'ex20_bringup.launch.py'))
    actions = []
    for action in source.get_launch_description(context).entities:
        if not isinstance(action, Node):
            continue
        executable = perform_substitutions(context, normalize_to_list_of_substitutions(action.node_executable))
        if executable not in ['wpr2_objects_3d', 'wpr2_grab']:
            continue
        if executable == 'wpr2_grab':
            action._Node__parameters += tuple(normalize_parameters([{
                'joint_min_wait': 0.02, 'grip_min_wait': 0.03, 'grip_timeout': 0.2,
                'reset_wait': 0.03, 'stop_wait': 0.03, 'ready_timeout': 2.0,
                'joint_timeout': 1.0, 'joint_feedback_timeout': 0.3, 'pose_timeout': 0.3,
                'motion_timeout': 8.0, 'total_timeout': 25.0,
                'align_speed': 0.2, 'forward_speed': 0.2, 'move_gain': 3.0,
            }]))
        actions.append(action)
    if len(actions) != 2:
        raise RuntimeError('ex20 must directly define the detector and grabber')
    service = LaunchService(argv=[], noninteractive=True)
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


if __name__ == '__main__':
    raise SystemExit(main())
