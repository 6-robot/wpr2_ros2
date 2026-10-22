"""Validate launch composition and configuration without starting hardware."""
import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from rclpy.qos import (DurabilityPolicy, QoSCompatibility, QoSProfile, ReliabilityPolicy,
                      qos_check_compatible, qos_profile_sensor_data)

ROOT = Path(__file__).resolve().parents[1]


def module(path):
    spec = importlib.util.spec_from_file_location('launch_under_test', str(path))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context_for(mod, **overrides):
    context = LaunchContext()
    for entity in mod.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            context.launch_configurations[entity.name] = perform_substitutions(context, entity.default_value)
    context.launch_configurations.update({k: str(v).lower() if isinstance(v, bool) else str(v)
                                         for k, v in overrides.items()})
    return context


def executable(action, context):
    value = action.node_executable
    return value if isinstance(value, str) else perform_substitutions(context, value)


def test_launch_python_and_expected_experiments():
    for path in (ROOT / 'launch').glob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    assert not (ROOT / 'launch/experiment.launch.py').exists()
    for path in (ROOT / 'launch').glob('*.py'):
        assert 'experiment.launch.py' not in path.read_text()
        assert 'base.launch.py' not in path.read_text()
    expected = set(range(1, 19))
    actual = {int(p.name[2:4]) for p in (ROOT / 'launch').glob('ex??_bringup.launch.py')}
    assert actual == expected
    manifest = ET.parse(ROOT / 'package.xml')
    assert manifest.findtext('name') == 'wpr2_tutorials'
    dependencies = [node.text for node in manifest.getroot() if node.tag.endswith('depend')]
    assert not any(p.startswith('wpb_home') or p == 'kinect2_bridge' for p in dependencies)


def parameters(action, context):
    evaluated = evaluate_parameters(context, action._Node__parameters)
    return {key: value for entry in evaluated if isinstance(entry, dict)
            for key, value in entry.items()}


@pytest.mark.parametrize('number', range(2, 16))
def test_independent_experiment_nodes_and_configuration(number):
    mod = module(ROOT / f'launch/ex{number:02d}_bringup.launch.py')
    context = context_for(mod)
    actions = mod.generate_launch_description().entities
    nodes = {executable(action, context): action for action in actions if isinstance(action, Node)}
    expected = {'robot_state_publisher'}
    expected.add('joint_state_publisher_gui' if number == 9 else 'wpr2_core')
    if number not in (6, 7, 8):
        expected.add('rviz2')
    if number in (4, 5, 6, 7, 8, 15):
        expected.update(['sllidar_node', 'wpr2_lidar_filter'])
    if number == 11:
        expected.add('face_detector.py')
    if number == 15:
        expected.add('wpr2_objects_3d')
    assert set(nodes) == expected
    assert len(nodes) == sum(isinstance(action, Node) for action in actions)
    assert not any(isinstance(action, DeclareLaunchArgument) for action in actions)
    assert all(action.condition is None for action in actions)

    if number != 9:
        core = parameters(nodes['wpr2_core'], context)
        assert core['serial_port'] == '/dev/ftdi'
        assert core['publish_odom_tf'] is True and core['use_sim_time'] is False
        assert core['odom_frame'] == 'odom' and core['base_frame'] == 'base_footprint'
        assert 'imu_pub' not in core  # Preserve the driver's YAML setting.
    rsp = parameters(nodes['robot_state_publisher'], context)
    assert ET.fromstring(rsp['robot_description']).find("link[@name='base_footprint']") is not None
    assert rsp['use_sim_time'] is False
    if 'sllidar_node' in nodes:
        lidar = parameters(nodes['sllidar_node'], context)
        assert lidar['serial_port'] == '/dev/rplidar' and lidar['serial_baudrate'] == 460800
        assert lidar['frame_id'] == 'laser' and lidar['scan_mode'] == 'Standard'
        assert lidar['scan_frequency'] == 10.0 and lidar['angle_compensate'] is True
        assert lidar['inverted'] is False
        assert parameters(nodes['wpr2_lidar_filter'], context)['pub_topic'] == '/scan'
        # These remaps are resolved when the driver node is executed.
        assert [('scan', 'scan_raw')] == [
            (perform_substitutions(context, src), perform_substitutions(context, dst))
            for src, dst in nodes['sllidar_node']._Node__remappings]
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    if number in (9, 10, 11, 12, 13, 15):
        assert len(includes) == 1
        includes[0].launch_description_source.get_launch_description(context)
        assert includes[0].launch_description_source.location.endswith('include/start_femto_bolt.launch.py')
    else:
        assert includes == []
    if number == 15:
        assert parameters(nodes['wpr2_objects_3d'], context)['auto_start'] is True
        assert 'wpr2_grab' not in nodes


@pytest.mark.parametrize('filename, target', [
    ('ex01_bringup.launch.py', 'ex01_intro.launch.py'),
    ('ex01_intro.launch.py', 'urdf.launch.py'),
    ('ex16_bringup.launch.py', 'tts.launch.py'),
    ('ex17_bringup.launch.py', 'sr_cn.launch.py'),
    ('slam.launch.py', 'ex06_slam.launch.py'),
    ('navigation.launch.py', 'ex07_navigation.launch.py'),
])
def test_compatibility_and_external_launch_entries(filename, target):
    mod = module(ROOT / 'launch' / filename)
    context = context_for(mod)
    actions = mod.generate_launch_description().entities
    assert len(actions) == 1 and isinstance(actions[0], IncludeLaunchDescription)
    actions[0].launch_description_source.get_launch_description(context)
    assert actions[0].launch_description_source.location.endswith('/' + target)
    if filename == 'ex01_intro.launch.py':
        assert dict(actions[0].launch_arguments) == {
            'gui': 'true', 'rviz': 'true', 'use_sim_time': 'false'}


def test_teleop_and_slam_launches():
    mod = module(ROOT / 'launch/ex02_teleop.launch.py')
    context = context_for(mod)
    actions = mod.generate_launch_description().entities
    nodes = {executable(action, context): action for action in actions}
    assert set(nodes) == {'joy_node', 'wpr2_js_vel'}
    assert parameters(nodes['joy_node'], context)['autorepeat_rate'] == 20.0
    assert parameters(nodes['wpr2_js_vel'], context)['joy_timeout'] == 1.0
    mod = module(ROOT / 'launch/ex06_slam.launch.py')
    context = context_for(mod)
    actions = mod.generate_launch_description().entities
    nodes = {executable(action, context): action for action in actions}
    assert set(nodes) == {'async_slam_toolbox_node', 'rviz2'}
    assert parameters(nodes['async_slam_toolbox_node'], context)['use_sim_time'] is False


def test_fetch_launch_composition_and_files(tmp_path, monkeypatch):
    mod = module(ROOT / 'launch/ex18_bringup.launch.py')
    context = context_for(mod, map=tmp_path / 'ignored.yaml', waypoints=tmp_path / 'ignored.xml')
    map_file = tmp_path / 'map.yaml'
    waypoint_file = tmp_path / 'waypoints.xml'
    with pytest.raises(RuntimeError, match='Map YAML'):
        mod.validate_files(context, str(map_file), str(waypoint_file))
    map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    with pytest.raises(RuntimeError, match='Map image'):
        mod.validate_files(context, str(map_file), str(waypoint_file))
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    with pytest.raises(RuntimeError, match='Waypoint XML'):
        mod.validate_files(context, str(map_file), str(waypoint_file))
    (tmp_path / 'waypoints.xml').write_text('<Waypoints/>')
    mod.validate_files(context, str(map_file), str(waypoint_file))
    actions = mod.generate_launch_description().entities
    nodes = {executable(action, context): action for action in actions if isinstance(action, Node)}
    assert len(nodes) == sum(isinstance(action, Node) for action in actions)
    assert set(nodes) == {'wpr2_core', 'robot_state_publisher', 'sllidar_node', 'wpr2_lidar_filter',
                          'wp_edit_node', 'wp_navi_server', 'wpr2_objects_3d', 'wpr2_grab', 'rviz2'}
    assert parameters(nodes['wpr2_core'], context)['serial_port'] == '/dev/ftdi'
    assert parameters(nodes['sllidar_node'], context)['serial_baudrate'] == 460800
    assert parameters(nodes['wpr2_lidar_filter'], context)['pub_topic'] == '/scan'
    for name in ['wpr2_objects_3d', 'wpr2_grab']:
        assert parameters(nodes[name], context)['auto_start'] is False
    assert parameters(nodes['wp_edit_node'], context)['load'] == str(Path.home() / 'waypoints.xml')
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 2
    for action in includes:
        action.launch_description_source.get_launch_description(context)
    assert {Path(action.launch_description_source.location).name for action in includes} == {
        'start_femto_bolt.launch.py', 'bringup_launch.py'}
    assert not any(isinstance(action, DeclareLaunchArgument) for action in actions)
    navigation = next(action for action in includes
                      if Path(action.launch_description_source.location).name == 'bringup_launch.py')
    assert dict(navigation.launch_arguments)['map'] == str(
        Path(mod.get_package_share_directory('wpr2_tutorials')) / 'maps/map.yaml')
    # Missing grasp calibration must be reported before hardware is started.
    real_share = mod.get_package_share_directory
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda name:
                        str(tmp_path / 'missing_behaviors') if name == 'wpr2_behaviors' else real_share(name))
    with pytest.raises(RuntimeError, match='Behavior parameter file'):
        mod.validate_files(context, str(map_file), str(waypoint_file))


def test_imu_launch_override_preserves_default():
    mod = module(ROOT.parent / 'wpr2_bringup/launch/base.launch.py')
    for override in ['', 'true', 'false']:
        context = context_for(mod, imu_pub=override)
        actions = mod.launch_setup(context)[0].entities
        core = next(a for a in actions if isinstance(a, Node) and executable(a, context) == 'wpr2_core')
        params = evaluate_parameters(context, core._Node__parameters)
        values = {k: v for p in params if isinstance(p, dict) for k, v in p.items()}
        if override:
            assert values['imu_pub'] == (override == 'true')
        else:
            assert 'imu_pub' not in values  # selected YAML retains control


def test_navigation_uses_package_map_without_arguments_or_file_checks(tmp_path, monkeypatch):
    mod = module(ROOT / 'launch/ex07_navigation.launch.py')
    share = tmp_path / 'installed_tutorials'
    real_share = mod.get_package_share_directory
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda name:
                        str(share) if name == 'wpr2_tutorials' else real_share(name))
    # 构造启动描述不预读或检查地图，也不接受外部 map 值覆盖固定路径。
    context = context_for(mod, map=tmp_path / 'other.yaml')
    actions = mod.generate_launch_description().entities
    assert len(actions) == 2
    navigation, rviz = actions
    assert isinstance(navigation, IncludeLaunchDescription) and isinstance(rviz, Node)
    arguments = dict(navigation.launch_arguments)
    assert arguments['map'] == str(share / 'maps/map.yaml')
    assert arguments['params_file'] == str(share / 'config/nav2_params.yaml')
    assert arguments['use_sim_time'] == 'False' and arguments['use_composition'] == 'False'
    assert executable(rviz, context) == 'rviz2'
    assert parameters(rviz, context)['use_sim_time'] is False


def test_waypoint_navigation_uses_fixed_files_without_preflight_checks(tmp_path, monkeypatch):
    mod = module(ROOT / 'launch/ex08_map_tools.launch.py')
    share = tmp_path / 'installed_tutorials'
    real_share = mod.get_package_share_directory
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda name:
                        str(share) if name == 'wpr2_tutorials' else real_share(name))
    context = context_for(mod, map=tmp_path / 'override.yaml', waypoints=tmp_path / 'override.xml')
    actions = mod.generate_launch_description().entities
    assert len(actions) == 4
    navigation = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    arguments = dict(navigation.launch_arguments)
    assert arguments['map'] == str(share / 'maps/map.yaml')
    assert arguments['params_file'] == str(share / 'config/nav2_params.yaml')
    assert arguments['use_sim_time'] == 'False' and arguments['use_composition'] == 'False'
    nodes = {executable(action, context): action for action in actions if isinstance(action, Node)}
    assert set(nodes) == {'rviz2', 'wp_edit_node', 'wp_navi_server'}
    assert parameters(nodes['wp_edit_node'], context)['load'] == str(Path.home() / 'waypoints.xml')
    assert all(isinstance(action, (Node, IncludeLaunchDescription)) for action in actions)


def test_nav2_and_rviz_configuration():
    data = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    assert data['amcl']['ros__parameters']['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    for name in ['local_costmap', 'global_costmap']:
        params = data[name][name]['ros__parameters']
        assert params['robot_base_frame'] == 'base_footprint'
        assert len(yaml.safe_load(params['footprint'])) == 4
        assert 'robot_radius' not in params
    for path in (ROOT / 'rviz').glob('*.rviz'):
        text = path.read_text()
        assert 'kinect2' not in text and 'wpb_home' not in text
        assert yaml.safe_load(text)['Visualization Manager']['Global Options']['Fixed Frame'] in ['map', 'base_footprint']


def test_sensor_rviz_qos_matches_publishers():
    publishers = {
        '/scan': qos_profile_sensor_data,
        '/camera/color/image_raw': QoSProfile(depth=10),
        '/camera/depth_registered/points': QoSProfile(depth=10),
    }
    checked = set()
    for path in ROOT.parent.glob('*/rviz/*.rviz'):
        text = path.read_text()
        assert '/camera/depth/color/points' not in text, str(path)
        for display in yaml.safe_load(text)['Visualization Manager']['Displays']:
            topic = display.get('Topic')
            if not isinstance(topic, dict) or topic.get('Value') not in publishers:
                continue
            name = topic['Value']
            reliability = {'Reliable': ReliabilityPolicy.RELIABLE,
                           'Best Effort': ReliabilityPolicy.BEST_EFFORT}[topic['Reliability Policy']]
            durability = {'Volatile': DurabilityPolicy.VOLATILE,
                          'Transient Local': DurabilityPolicy.TRANSIENT_LOCAL}[topic['Durability Policy']]
            subscription = QoSProfile(depth=topic['Depth'], reliability=reliability,
                                      durability=durability)
            compatibility, reason = qos_check_compatible(publishers[name], subscription)
            assert compatibility != QoSCompatibility.ERROR, f'{path}: {name}: {reason}'
            if name.startswith('/camera/'):
                compatibility, reason = qos_check_compatible(qos_profile_sensor_data, subscription)
                assert compatibility != QoSCompatibility.ERROR, f'{path}: SENSOR_DATA: {reason}'
            checked.add(name)
    assert checked == set(publishers)
