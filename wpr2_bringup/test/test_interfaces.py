"""Exercise real ROS 2 executables with DDS and a pseudo-terminal robot."""
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan, Joy, Imu
from std_msgs.msg import String


class Harness:
    def __init__(self, tmp_path):
        self.context = Context()
        self.domain = 150 + os.getpid() % 30
        rclpy.init(context=self.context, domain_id=self.domain)
        self.prefix = '/test_' + uuid.uuid4().hex
        self.node = rclpy.create_node('check', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.processes = []
        self.tmp_path = tmp_path
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.buffer = b''
        self.frames = []

    def start(self, name, **params):
        args = [str(Path(os.environ['WPR2_BIN_DIR']) / name), '--ros-args']
        topics = ['cmd_vel', 'joint_states', 'odom', 'odom_delta', 'odom_delta_reset', 'imu/data_raw',
                  'wpr2/left_arm', 'wpr2/right_arm', 'wpr2/left_gripper', 'wpr2/right_gripper',
                  'wpr2/chest_height', 'wpr2/joints_result', 'scan_raw', 'scan_filtered', 'joy',
                  'plan/joint_states', 'plan/wpr2_cmd', 'real/wpr2/left_arm', 'real/wpr2/right_arm',
                  'real/wpr2/chest_height', 'tf', 'tf_static']
        for topic in topics:
            args += ['-r', '/' + topic + ':=' + self.prefix + '/' + topic]
        for key, value in params.items():
            args += ['-p', key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))]
        log = open(self.tmp_path / (name + '.log'), 'w+')
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                   env=dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_LOCALHOST_ONLY='1'))
        self.processes.append((process, log))
        return process

    def core(self, **params):
        return self.start('wpr2_core', serial_port=os.ttyname(self.slave), **params)

    def pub(self, msg_type, topic):
        return self.node.create_publisher(msg_type, self.prefix + '/' + topic, 10)

    def collect(self, msg_type, topic, sensor=False):
        values = []
        self.node.create_subscription(msg_type, self.prefix + '/' + topic, values.append,
                                      qos_profile_sensor_data if sensor else 10)
        return values

    def read(self):
        while True:
            try:
                self.buffer += os.read(self.master, 65536)
            except BlockingIOError:
                break
        while len(self.buffer) >= 7:
            start = self.buffer.find(b'\x55\xaa')
            if start < 0:
                self.buffer = self.buffer[-1:]
                break
            self.buffer = self.buffer[start:]
            if len(self.buffer) < 7:
                break
            length = self.buffer[3] + 7
            if len(self.buffer) < length:
                break
            frame, self.buffer = self.buffer[:length], self.buffer[length:]
            assert sum(frame[:-1]) % 256 == frame[-1]
            self.frames.append(frame)

    def wait(self, predicate, timeout=5, tick=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if tick:
                tick()
            self.executor.spin_once(timeout_sec=0.03)
            self.read()
            if predicate():
                return
        logs = []
        for _, log in self.processes:
            log.flush()
            logs.append(Path(log.name).read_text())
        pytest.fail('Timed out\n' + '\n'.join(logs))

    def pump(self, seconds=0.25):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)
            self.read()

    def feedback(self, module, method, payload, subtype=0):
        frame = bytes([0x55, 0xaa, 0x40, len(payload), module, method, subtype]) + payload
        os.write(self.master, frame + bytes([sum(frame) % 256]))

    def joints(self, positions):
        self.feedback(8, 0x63, b''.join(struct.pack('>ii', 0, p) for p in positions))

    def motors(self):
        return [struct.unpack('>4i', frame[6:-1]) for frame in self.frames if frame[4:6] == b'\x08\x60']

    def commands(self):
        return [struct.unpack('>30i', frame[6:-1]) for frame in self.frames if frame[4:6] == b'\x08\x63']

    def close(self):
        for process, log in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def h(tmp_path):
    harness = Harness(tmp_path)
    try:
        yield harness
    finally:
        harness.close()


def test_velocity_timeout_and_shutdown(h):
    process = h.core()
    pub = h.pub(Twist, 'cmd_vel')
    odom = h.collect(Odometry, 'odom')
    h.wait(lambda: odom and pub.get_subscription_count() > 0)
    assert odom[-1].pose.pose.orientation.w == 1.0
    command = Twist()
    command.linear.x = 0.1
    h.wait(lambda: any(m[0] > 0 and m[1] < 0 for m in h.motors()), tick=lambda: pub.publish(command))
    h.pump(0.6)
    assert h.motors()[-1] != (0, 0, 0, 0)
    h.wait(lambda: h.motors() and h.motors()[-1] == (0, 0, 0, 0), timeout=1.2)
    command.linear.x = float('nan')
    pub.publish(command)
    h.pump()
    assert h.motors()[-1] == (0, 0, 0, 0)
    process.send_signal(signal.SIGINT)
    process.wait(timeout=5)
    h.read()
    assert process.returncode == 0
    assert h.motors()[-1] == (0, 0, 0, 0)


def test_joint_commands_validation_and_arrival(h):
    process = h.core()
    states = h.collect(JointState, 'joint_states')
    arm = h.pub(JointState, 'wpr2/left_arm')
    gripper = h.pub(JointState, 'wpr2/right_gripper')
    torso = h.pub(JointState, 'wpr2/chest_height')
    results = h.collect(String, 'wpr2/joints_result')
    h.wait(lambda: arm.get_subscription_count() and gripper.get_subscription_count() and torso.get_subscription_count())
    h.joints([0] * 15)
    h.pump()
    arm.publish(JointState())
    arm.publish(JointState(position=[1.0] * 6, velocity=[1.0]))
    gripper.publish(JointState(position=[float('nan')]))
    h.pump()
    assert not h.commands()
    arm.publish(JointState(position=[0.1] * 6))
    h.wait(lambda: h.commands())
    frame = h.commands()[-1]
    assert all(frame[i * 2] == 2000 for i in range(1, 7))
    assert all(frame[i * 2 + 1] == int(0.1 * 18000 / math.pi) for i in range(1, 7))
    gripper.publish(JointState(position=[0.0], velocity=[1500.0]))
    torso.publish(JointState(position=[0.5], velocity=[18000.0]))
    h.wait(lambda: h.commands()[-1][29] == 21900 and abs(h.commands()[-1][1] - 200000) <= 1)
    h.pump(1.1)
    assert not results  # stationary feedback alone must not report success
    targets = list(h.commands()[-1][1::2])
    h.wait(lambda: results, tick=lambda: h.joints(targets))
    assert results[-1].data == 'done'
    h.wait(lambda: states and abs(states[-1].position[18] - 1.57) < 0.01)
    assert process.poll() is None


def test_feedback_odometry_imu_and_malformed_frames(h):
    process = h.core(imu_pub=True)
    states = h.collect(JointState, 'joint_states')
    odom = h.collect(Odometry, 'odom')
    delta = h.collect(Pose2D, 'odom_delta')
    imu = h.collect(Imu, 'imu/data_raw')
    reset = h.pub(String, 'odom_delta_reset')
    h.wait(lambda: states and odom and imu and reset.get_subscription_count())
    assert len(states[-1].name) == 21
    assert imu[-1].orientation_covariance[0] == -1
    h.feedback(9, 0, struct.pack('>ii', 1, 0), subtype=3)
    h.feedback(9, 0, struct.pack('>ii', 0, 0), subtype=4)
    h.wait(lambda: imu[-1].orientation_covariance[0] == 0)
    assert imu[-1].orientation.w == 1.0
    # Oversize header, motor id zero, and short joint packet must not corrupt memory.
    os.write(h.master, b'\x55\xaa\x40\xff')
    h.feedback(8, 0x60, struct.pack('>Bii', 0, 0, 123))
    h.feedback(8, 0x63, b'')
    for motor, position in [(1, -950), (2, 950), (3, 0), (4, 0)]:
        h.feedback(8, 0x60, struct.pack('>Bii', motor, 0, position))
    h.wait(lambda: delta and delta[-1].x > 0)
    assert delta[-1].x == pytest.approx(0.01)
    h.pump()
    assert odom[-1].twist.twist.linear.x == 0
    assert odom[-1].twist.twist.angular.z == 0
    reset.publish(String(data='odom_delta reset'))
    h.wait(lambda: delta[-1].x == 0)
    assert process.poll() is None


def test_lidar_large_empty_and_intensity_free_scan(h):
    h.start('wpr2_lidar_filter', ignore_count=0)
    pub = h.pub(LaserScan, 'scan_raw')
    scans = h.collect(LaserScan, 'scan_filtered', sensor=True)
    h.wait(lambda: pub.get_subscription_count())
    msg = LaserScan(angle_min=-math.pi, angle_max=math.pi, angle_increment=2 * math.pi / 1499,
                    range_min=0.02, range_max=10.0, scan_time=0.1, ranges=[0.1] * 1500)
    msg.ranges[750] = 2.0
    h.wait(lambda: scans, tick=lambda: pub.publish(msg))
    assert len(scans[-1].ranges) == 1500
    assert scans[-1].ranges[0] == 11.0
    assert scans[-1].ranges[750] == 2.0
    assert not scans[-1].intensities
    assert scans[-1].scan_time == pytest.approx(0.1)
    pub.publish(LaserScan())
    h.wait(lambda: not scans[-1].ranges)


def test_joystick_short_message_and_timeout(h):
    process = h.start('wpr2_js_vel')
    pub = h.pub(Joy, 'joy')
    commands = h.collect(Twist, 'cmd_vel')
    h.wait(lambda: pub.get_subscription_count())
    pub.publish(Joy(axes=[]))
    h.pump()
    h.wait(lambda: commands and commands[-1].linear.x == pytest.approx(0.2),
           tick=lambda: pub.publish(Joy(axes=[0.5, 1.0, 0.0, -0.5])))
    assert commands[-1].linear.y == pytest.approx(0.1)
    assert commands[-1].angular.z == pytest.approx(-0.25)
    h.pump(0.6)
    assert commands[-1].linear.x == pytest.approx(0.2)
    h.wait(lambda: commands[-1].linear.x == 0)
    assert process.poll() is None


def test_arm_plan_named_mapping_and_incomplete_input(h):
    h.start('wpr2_arms_test')
    plans = h.pub(JointState, 'plan/joint_states')
    go = h.pub(String, 'plan/wpr2_cmd')
    left = h.collect(JointState, 'real/wpr2/left_arm')
    right = h.collect(JointState, 'real/wpr2/right_arm')
    torso = h.collect(JointState, 'real/wpr2/chest_height')
    h.wait(lambda: plans.get_subscription_count() and go.get_subscription_count())
    plans.publish(JointState(name=['chest_height'], position=[0.5]))
    h.pump()
    go.publish(String(data='go'))
    h.pump()
    assert not left and not right and not torso
    names = ['chest_height'] + [f'{side}_joint_{i}' for side in ['right', 'left'] for i in range(6, 0, -1)]
    values = [0.5] + [float(i) / 10 for i in range(1, 13)]
    plans.publish(JointState(name=names, position=values))
    h.pump()
    h.wait(lambda: left and right and torso, tick=lambda: go.publish(String(data='go')))
    expected = dict(zip(names, values))
    assert list(left[-1].position) == [expected[f'left_joint_{i}'] for i in range(1, 7)]
    assert torso[-1].position[0] == 0.5


def test_missing_serial_port_fails_cleanly(h):
    process = h.start('wpr2_core', serial_port='/nonexistent/wpr2-test-port')
    assert process.wait(timeout=5) == 1
