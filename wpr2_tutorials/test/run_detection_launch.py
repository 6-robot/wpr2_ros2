"""Run only the real ex17 launch's detector for hardware-free integration tests."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


def main():
    context = LaunchContext()
    source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('wpr2_tutorials'), 'launch', 'ex17_bringup.launch.py'))
    description = source.get_launch_description(context)
    detectors = [action for action in description.entities
                 if isinstance(action, Node) and
                 perform_substitutions(context, normalize_to_list_of_substitutions(
                     action.node_executable)) == 'wpr2_objects_3d']
    if len(detectors) != 1:
        raise RuntimeError('ex17_bringup must directly define exactly one object detector')
    service = LaunchService(argv=[], noninteractive=True)
    service.include_launch_description(LaunchDescription(detectors))
    return service.run()


if __name__ == '__main__':
    raise SystemExit(main())
