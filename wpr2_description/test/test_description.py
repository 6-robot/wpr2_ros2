"""Check complete models, mesh resources, mimic joints and display defaults."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(params=['wpr2.urdf.xacro', 'wpr2_gripper.urdf.xacro'])
def model(request):
    return ET.fromstring(xacro.process_file(str(PACKAGE / 'urdf' / request.param)).toxml())


def test_urdf_tree_and_resources(model, tmp_path):
    urdf = tmp_path / 'robot.urdf'
    ET.ElementTree(model).write(urdf, encoding='unicode')
    result = subprocess.run(['check_urdf', str(urdf)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'Successfully Parsed XML' in result.stdout
    assert len(model.findall('link')) == 39
    assert len(model.findall('joint')) == 38
    for mesh in model.iter('mesh'):
        uri = mesh.attrib['filename']
        assert uri.startswith('package://wpr2_description/')
        assert (PACKAGE / uri.removeprefix('package://wpr2_description/')).is_file(), uri
    assert not model.findall('.//plugin')
    assert not model.findall('.//hardwareInterface')


def test_grippers_follow_driver_joints(model):
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}
    for side in ['left', 'right']:
        driver = f'{side}_joint_finger'
        assert joints[driver].attrib['type'] == 'revolute'
        for suffix in ['joint_tip_1', 'joint_finger_2', 'joint_tip_2']:
            mimic = joints[f'{side}_{suffix}'].find('mimic')
            assert mimic.attrib == {'joint': driver, 'multiplier': '1', 'offset': '0'}


def test_initial_positions_are_valid(model):
    config = yaml.safe_load((PACKAGE / 'config' / 'joint_states.yaml').read_text())
    zeros = config['/**']['ros__parameters']['zeros']
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}
    for name, position in zeros.items():
        joint = joints[name]
        assert joint.attrib['type'] not in ['fixed']
        assert joint.find('mimic') is None
        limit = joint.find('limit')
        if limit is not None and joint.attrib['type'] != 'continuous':
            assert float(limit.attrib['lower']) <= position <= float(limit.attrib['upper'])
