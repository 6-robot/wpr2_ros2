"""Run real executables on isolated DDS topics, without any hardware driver."""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Twist
from sensor_msgs.msg import Imu, JointState, LaserScan, Image, RegionOfInterest, PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

TOPICS = ['/camera/depth_registered/points', '/objects_marker', '/wpr2/objects',
          '/cmd_vel', '/scan', '/imu/data_raw', '/joint_states',
          '/wpr2/chest_height', '/wpr2/left_arm', '/wpr2/right_arm',
          '/wpr2/left_gripper', '/wpr2/right_gripper',
          '/camera/color/image_raw', '/face_detector_input', '/face_position',
          '/waterplus/navi_waypoint', '/waterplus/navi_result', '/tts/text', '/speech/text']


class Harness:
    def __init__(self, tmp_path):
        self.prefix = '/test_' + uuid.uuid4().hex
        self.context = Context()
        rclpy.init(context=self.context, domain_id=175 + os.getpid() % 40)
        self.node = rclpy.create_node('test_node', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(175 + os.getpid() % 40),
                        ROS_LOCALHOST_ONLY='1', QT_QPA_PLATFORM='offscreen')
        self.processes = []
        self.tmp_path = tmp_path

    def start(self, executable, **params):
        path = Path(os.environ['TUTORIAL_BIN_DIR']) / executable
        args = [str(path), '--ros-args']
        for topic in TOPICS:
            args += ['-r', topic + ':=' + self.prefix + topic]
        for name, value in params.items():
            args += ['-p', name + ':=' + str(value).lower()]
        log = open(self.tmp_path / (Path(executable).name + '.log'), 'w+')
        process = subprocess.Popen(args, env=self.env, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append((process, log))
        return process

    def publisher(self, cls, topic, sensor=False):
        return self.node.create_publisher(cls, self.prefix + topic,
                                          qos_profile_sensor_data if sensor else 10)

    def collect(self, cls, topic):
        messages = []
        self.node.create_subscription(cls, self.prefix + topic, messages.append, 10)
        return messages

    def wait(self, predicate, timeout=5, tick=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if tick:
                tick()
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        logs = []
        for _, log in self.processes:
            log.flush()
            logs.append(Path(log.name).read_text())
        pytest.fail('Timed out\n' + '\n'.join(logs))

    def pump(self, seconds=0.2):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def close(self):
        for process, log in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def h(tmp_path):
    harness = Harness(tmp_path)
    yield harness
    harness.close()


def stopped(msg):
    return msg.linear.x == msg.linear.y == msg.angular.z == 0


def test_velocity_continuous_and_normal_exit(h):
    values = h.collect(Twist, '/cmd_vel')
    process = h.start('ex03_velocity')
    h.wait(lambda: len(values) > 8)
    assert all(m.linear.x == 0.1 for m in values)
    process.send_signal(signal.SIGINT)
    h.wait(lambda: process.poll() is not None)
    h.pump()
    assert process.returncode == 0


def scan_with_front(distance, layout='signed', rear=0.4):
    # WPR2 mounts laser at yaw=pi: laser +/-pi is base forward; laser 0 is rear.
    start, increment, front, back = {
        'signed': (-math.pi, math.pi / 4, 0, 4),
        'unsigned': (0.0, math.pi / 4, 4, 0),
        'reverse': (math.pi, -math.pi / 4, 0, 4),
    }[layout]
    ranges = [3.0] * 8
    ranges[front], ranges[back] = distance, rear
    scan = LaserScan(angle_min=start, angle_max=start + 7 * increment,
                     angle_increment=increment, range_min=0.05, range_max=12.0,
                     ranges=ranges)
    scan.header.frame_id = 'laser'
    return scan


@pytest.mark.parametrize('layout', ['signed', 'unsigned', 'reverse'])
def test_lidar_front_turn_and_refresh(h, layout):
    pub = h.publisher(LaserScan, '/scan', sensor=True)
    values = h.collect(Twist, '/cmd_vel')
    h.start('ex05_lidar_behavior')
    h.wait(lambda: pub.get_subscription_count() > 0)
    h.wait(lambda: values and values[-1].linear.x == 0.1,
           tick=lambda: pub.publish(scan_with_front(3.0, layout)))
    values.clear()
    h.wait(lambda: len(values) >= 10 and values[-1].angular.z == 0.3,
           tick=lambda: pub.publish(scan_with_front(0.4, layout, rear=3.0)))
    h.wait(lambda: values[-1].linear.x == 0.1,
           tick=lambda: pub.publish(scan_with_front(3.0, layout)))
    # 没有新扫描时不重发缓存速度，交给驱动自己的指令超时处理。
    h.pump(0.1)
    count = len(values)
    h.pump(0.2)
    assert len(values) == count


@pytest.mark.parametrize('layout', ['signed', 'unsigned', 'reverse'])
def test_lidar_readout_uses_wpr2_forward_heading(h, layout):
    pub = h.publisher(LaserScan, '/scan', sensor=True)
    h.start('ex04_lidar')
    log = Path(h.processes[-1][1].name)
    h.wait(lambda: pub.get_subscription_count() > 0)
    h.wait(lambda: 'Forward distance = 0.400 m' in log.read_text(),
           tick=lambda: pub.publish(scan_with_front(0.4, layout, rear=3.0)))


def test_imu_readout_and_wrapped_control(h):
    pub = h.publisher(Imu, '/imu/data_raw', sensor=True)
    values = h.collect(Twist, '/cmd_vel')
    h.start('ex06_imu_data')
    h.start('ex07_imu_behavior')
    h.wait(lambda: pub.get_subscription_count() == 2)
    msg = Imu()
    # 目标90度，当前-179度：环绕后误差-91度，应向负方向转动。
    msg.orientation.z = math.sin(math.radians(-179) / 2)
    msg.orientation.w = math.cos(math.radians(-179) / 2)
    h.wait(lambda: values and values[-1].angular.z == pytest.approx(-0.91),
           tick=lambda: pub.publish(msg))
    msg.orientation.z = math.sin(math.radians(89) / 2)
    msg.orientation.w = math.cos(math.radians(89) / 2)
    h.wait(lambda: values[-1].angular.z == pytest.approx(0.01, abs=1e-6),
           tick=lambda: pub.publish(msg))
    assert values[-1].linear.x == 0.1
    assert 'yaw=89.0 deg' in Path(h.processes[-2][1].name).read_text()
    h.pump(0.1)
    count = len(values)
    h.pump(0.2)
    assert len(values) == count


def test_joint_demo_matches_ros1_two_poses(h):
    topics = ['chest_height', 'left_arm', 'left_gripper', 'right_arm', 'right_gripper']
    commands = {name: h.collect(JointState, '/wpr2/' + name) for name in topics}
    process = h.start('ex16_manipulator')
    # 不提供关节反馈；教学示例直接以5秒周期发两组指令。
    h.wait(lambda: all(len(v) >= 2 for v in commands.values()), timeout=8)
    assert commands['left_arm'][0].position == pytest.approx([0.5, -1.57, 0, -1.16, 0, 0])
    assert commands['left_arm'][1].position == pytest.approx([0.3, -1.4, 0.1, -0.6, 0.4, -0.2])
    assert commands['right_arm'][0].position == pytest.approx([-0.5, 1.57, 0, 1.16, 0, 0])
    assert commands['right_arm'][1].position == pytest.approx([-0.3, 1.4, -0.1, 0.6, -0.4, 0.2])
    for index in range(2):
        assert commands['chest_height'][index].position == pytest.approx([0.65 if index == 0 else 0.7])
        for name in ['left_gripper', 'right_gripper']:
            assert commands[name][index].position == pytest.approx([0.05 if index == 0 else 0.1])
        for name in topics:
            msg = commands[name][index]
            assert list(msg.velocity) == [15000.0 if name == 'chest_height' else 2000.0] * len(msg.position)
    process.send_signal(signal.SIGINT)
    h.wait(lambda: process.poll() is not None)
    assert process.returncode == 0


def test_face_image_forwarding_and_rectangle(h):
    images = h.publisher(Image, '/camera/color/image_raw', sensor=True)
    rois = h.publisher(RegionOfInterest, '/face_position')
    forwarded = h.collect(Image, '/face_detector_input')
    process = h.start('ex13_face')
    h.wait(lambda: images.get_subscription_count() > 0 and rois.get_subscription_count() > 0)
    msg = Image(height=8, width=8, encoding='bgr8', step=24, data=bytes(192))
    h.wait(lambda: forwarded, tick=lambda: images.publish(msg))
    rois.publish(RegionOfInterest(x_offset=1, y_offset=1, width=4, height=4))
    h.pump()
    assert forwarded[0].width == 8 and process.poll() is None


@pytest.mark.parametrize('executable', ['ex12_image', 'ex12_hsv'])
def test_image_display_with_valid_frame(h, executable):
    images = h.publisher(Image, '/camera/color/image_raw', sensor=True)
    process = h.start(executable)
    h.wait(lambda: images.get_subscription_count() > 0)
    images.publish(Image(height=8, width=8, encoding='bgr8', step=24, data=bytes(192)))
    h.pump()
    assert process.poll() is None


def test_waypoint_request_and_result(h):
    requests = h.collect(String, '/waterplus/navi_waypoint')
    results = h.publisher(String, '/waterplus/navi_result')
    process = h.start('ex10_waypoint')
    h.wait(lambda: requests)
    assert requests[0].data == '1'
    h.wait(lambda: results.get_subscription_count() > 0)
    results.publish(String(data='navi done'))
    h.wait(lambda: 'Arrived !' in Path(h.processes[-1][1].name).read_text())
    assert process.poll() is None


def make_cloud(frame='base_footprint'):
    points = [(0.55 + i * 0.01, -0.45 + j * 0.01, 0.7)
              for i in range(85) for j in range(85)]
    points += [(0.9 + i * 0.01, -0.05 + j * 0.01, 0.85) for i in range(15) for j in range(15)]
    fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(['x', 'y', 'z'])]
    cloud = PointCloud2(height=1, width=len(points), fields=fields, point_step=12,
                        row_step=12 * len(points), is_dense=True,
                        data=b''.join(struct.pack('<fff', *p) for p in points))
    cloud.header.frame_id = frame
    return cloud


def test_pointcloud_readout_uses_driver_topic(h):
    pub = h.publisher(PointCloud2, '/camera/depth_registered/points', sensor=True)
    process = h.start('ex14_pointcloud')
    log = Path(h.processes[-1][1].name)
    cloud = make_cloud()
    h.wait(lambda: pub.get_subscription_count() > 0)
    h.wait(lambda: f'Point count = {cloud.width}' in log.read_text(),
           tick=lambda: pub.publish(cloud))
    assert process.poll() is None


def test_pointcloud_detection_and_wpb_tf_check(h):
    pub = h.publisher(PointCloud2, '/camera/depth_registered/points', sensor=True)
    results = h.collect(PoseArray, '/wpr2/objects')
    markers = h.collect(MarkerArray, '/objects_marker')
    process = h.start('ex15_objects')
    cloud = make_cloud()
    h.wait(lambda: pub.get_subscription_count() > 0)
    h.wait(lambda: any(m.poses for m in results), tick=lambda: pub.publish(cloud))
    result = next(m for m in results if m.poses)
    assert result.header.frame_id == 'base_footprint'
    assert result.poses[0].position.x == pytest.approx(0.97, abs=0.02)
    assert result.poses[0].position.z == pytest.approx(0.85, abs=0.02)
    # 与 WPB 一样，缺少 TF 时不处理该帧，不发布替代结果。
    h.pump()
    results.clear()
    markers.clear()
    pub.publish(make_cloud('missing_camera_frame'))
    h.pump(0.2)
    assert not results and not markers
    assert process.poll() is None


def test_speech_interfaces(h):
    text = h.collect(String, '/tts/text')
    process = h.start('ex18_tts')
    h.wait(lambda: text)
    assert 'WPR2' in text[0].data
    h.wait(lambda: process.poll() is not None)
    assert process.returncode == 0
    recognition = h.publisher(String, '/speech/text')
    h.start('ex19_recognition')
    h.wait(lambda: recognition.get_subscription_count() > 0)
    recognition.publish(String(data='测试识别'))
    h.wait(lambda: '测试识别' in Path(h.processes[-1][1].name).read_text())
