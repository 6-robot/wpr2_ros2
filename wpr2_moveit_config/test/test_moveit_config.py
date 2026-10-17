"""Check that the installed MoveIt data agrees with the installed robot model."""

from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
import xacro


def test_robot_groups_and_controllers():
    share = Path(get_package_share_directory('wpr2_moveit_config'))
    description = Path(get_package_share_directory('wpr2_description'))
    urdf = ET.fromstring(xacro.process_file(str(description / 'urdf/wpr2.urdf.xacro')).toxml())
    srdf = ET.parse(share / 'config/wpr2.srdf').getroot()
    joints = {joint.get('name') for joint in urdf.findall('joint')}
    links = {link.get('name') for link in urdf.findall('link')}
    groups = {group.get('name') for group in srdf.findall('group')}
    for chain in srdf.findall('group/chain'):
        assert chain.get('base_link') in links
        assert chain.get('tip_link') in links
    for state in srdf.findall('group_state'):
        assert state.get('group') in groups
        assert {joint.get('name') for joint in state.findall('joint')} <= joints
    controllers = yaml.safe_load((share / 'config/moveit_controllers.yaml').read_text())
    names = controllers['moveit_simple_controller_manager']['controller_names']
    assert set(names) == {'left_arm_controller', 'right_arm_controller',
                          'left_arm_lift_controller', 'right_arm_lift_controller'}
    for name in names:
        assert set(controllers['moveit_simple_controller_manager'][name]['joints']) <= joints


def test_moveit_builder_loads_humble_parameters():
    config = (MoveItConfigsBuilder('wpr2')
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .to_moveit_configs())
    params = config.to_dict()
    assert params['planning_pipelines'] == ['ompl']
    assert params['ompl']['planning_plugin'] == 'ompl_interface/OMPLPlanner'
    assert 'Left_Arm_Lift' in params['robot_description_kinematics']
    assert 'wpr2' in params['robot_description_semantic']


def test_rviz_shows_live_robot_and_initial_arm_goal():
    share = Path(get_package_share_directory('wpr2_moveit_config'))
    rviz = yaml.safe_load((share / 'config/moveit.rviz').read_text())
    displays = rviz['Visualization Manager']['Displays']
    robot = next(item for item in displays if item['Class'] == 'rviz_default_plugins/RobotModel')
    planning = next(item for item in displays if item['Class'] == 'moveit_rviz_plugin/MotionPlanning')
    assert robot['Enabled']
    assert robot['Description Topic']['Value'] == '/robot_description'
    assert planning['Planning Request']['Planning Group'] == 'Left_Arm_Lift'
    assert planning['Planning Request']['Query Goal State']
