"""Run experiment 15 against synthetic perception and a simulated WPR2 driver."""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Pose, PoseStamped, Pose2D, Twist
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String
from wpr2_msgs.msg import Object

TOPICS = ['/camera/depth_registered/points', '/objects_marker', '/wpr2/objects_3d',
          '/wpr2/behavior', '/wpr2/behaviors', '/wpr2/grab_bottle', '/wpr2/grab_pose',
          '/wpr2/grab_result', '/wpr2/grab_status', '/cmd_vel', '/odom_delta', '/odom_delta_reset',
          '/joint_states', '/wpr2/chest_height', '/wpr2/left_arm', '/wpr2/right_arm',
          '/wpr2/right_gripper']


class Harness:
    def __init__(self, tmp_path):
        self.prefix = '/test_' + uuid.uuid4().hex
        self.context = Context()
        rclpy.init(context=self.context, domain_id=175 + os.getpid() % 40)
        self.node = rclpy.create_node('test_node', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(175 + os.getpid() % 40),
                        ROS_LOCALHOST_ONLY='1')
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
        process = subprocess.Popen(args, env=self.env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((process, log))
        return process

    def publisher(self, cls, topic, sensor=False):
        return self.node.create_publisher(cls, self.prefix + topic,
                                          qos_profile_sensor_data if sensor else 10)

    def collect(self, cls, topic):
        messages = []
        self.node.create_subscription(cls, self.prefix + topic, messages.append, 10)
        return messages

    def wait(self, predicate, timeout=10, tick=None):
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
                    os.killpg(process.pid, signal.SIGKILL)
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


class Robot:
    """Closed-loop fake driver: integrate velocity, acknowledge resets, echo joint positions."""
    def __init__(self, h, follow=True):
        self.h = h
        self.pose = Pose2D(x=0.4, y=-0.2, theta=0.1)
        self.world_x = self.world_y = self.world_theta = 0.0
        self.velocity = Twist()
        self.feedback = self.pose_feedback = self.ack_reset = True
        self.integrate = True
        self.follow = follow
        self.follow_grip = True
        self.freeze_holding = False
        self.detect = False
        self.fake_detection = True
        self.world_target = (0.9, 0.08, 0.9)
        self.joints = {'chest_height': 0.7, 'right_joint_finger': 0.0}
        self.joints.update({f'{side}_joint_{i}': 0.0 for side in ['left', 'right'] for i in range(1, 7)})
        self.joint_pub = h.publisher(JointState, '/joint_states')
        self.pose_pub = h.publisher(Pose2D, '/odom_delta')
        self.object_pub = h.publisher(Object, '/wpr2/objects_3d')
        self.goal_pub = h.publisher(Pose, '/wpr2/grab_bottle')
        self.stamped_pub = h.publisher(PoseStamped, '/wpr2/grab_pose')
        self.behavior_pub = h.publisher(String, '/wpr2/behavior')
        self.legacy_behavior_pub = h.publisher(String, '/wpr2/behaviors')
        self.velocities = h.collect(Twist, '/cmd_vel')
        self.results = h.collect(String, '/wpr2/grab_result')
        self.statuses = h.collect(String, '/wpr2/grab_status')
        self.resets = h.collect(String, '/odom_delta_reset')
        self.commands = h.collect(String, '/wpr2/behavior')
        self.arm_commands = {name: [] for name in ['chest_height', 'right_arm', 'left_arm', 'right_gripper']}
        self.subscriptions = []
        for name in self.arm_commands:
            self.subscriptions.append(h.node.create_subscription(JointState, h.prefix + '/wpr2/' + name,
                lambda msg, name=name: self.joint_command(name, msg), 10))
        h.node.create_subscription(Twist, h.prefix + '/cmd_vel', lambda msg: setattr(self, 'velocity', msg), 10)
        h.node.create_subscription(String, h.prefix + '/odom_delta_reset', self.reset, 10)
        h.node.create_subscription(String, h.prefix + '/wpr2/behavior', self.command, 10)
        self.last_tick = time.monotonic()
        self.timer = h.node.create_timer(0.02, self.tick)

    def joint_command(self, name, msg):
        self.arm_commands[name].append(msg)
        if not self.follow:
            return
        if self.freeze_holding and name == 'right_arm' and abs(msg.position[0] + 0.1593) < 1e-5:
            return
        if name == 'right_gripper':
            if self.follow_grip or msg.position[0] > 0.1:
                self.joints['right_joint_finger'] = (0.155 - msg.position[0]) / 0.155 * 1.57
        else:
            self.joints.update(zip(msg.name, msg.position))

    def command(self, msg):
        if msg.data == 'start objects':
            self.detect = True
        elif msg.data == 'stop objects':
            self.detect = False

    def reset(self, msg):
        if msg.data == 'odom_delta reset' and self.ack_reset:
            self.pose = Pose2D()

    def target(self):
        x, y, z = self.world_target
        dx, dy = x - self.world_x, y - self.world_y
        return (dx * math.cos(self.world_theta) + dy * math.sin(self.world_theta),
                -dx * math.sin(self.world_theta) + dy * math.cos(self.world_theta), z)

    def tick(self):
        stamp = time.monotonic()
        dt = min(stamp - self.last_tick, 0.05)
        self.last_tick = stamp
        if self.integrate:
            vx, vy, wz = self.velocity.linear.x, self.velocity.linear.y, self.velocity.angular.z
            self.pose.x += (vx * math.cos(self.pose.theta) - vy * math.sin(self.pose.theta)) * dt
            self.pose.y += (vx * math.sin(self.pose.theta) + vy * math.cos(self.pose.theta)) * dt
            self.pose.theta += wz * dt
            self.world_x += (vx * math.cos(self.world_theta) - vy * math.sin(self.world_theta)) * dt
            self.world_y += (vx * math.sin(self.world_theta) + vy * math.cos(self.world_theta)) * dt
            self.world_theta += wz * dt
        if self.feedback:
            joints = JointState(name=list(self.joints), position=list(self.joints.values()))
            joints.header.stamp = self.h.node.get_clock().now().to_msg()
            self.joint_pub.publish(joints)
        if self.pose_feedback:
            self.pose_pub.publish(self.pose)
        if self.detect and self.fake_detection:
            self.publish_object()

    def publish_object(self, **overrides):
        x, y, z = self.target()
        msg = Object(x=[x], y=[y], z=[z], name=['object_0'], probability=[1.0])
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = self.h.node.get_clock().now().to_msg()
        for key, value in overrides.items():
            setattr(msg, key, value)
        self.object_pub.publish(msg)

    def start(self):
        self.process = self.h.start('ex15_grab')
        self.h.wait(lambda: self.object_pub.get_subscription_count() > 0)

    def goal(self, x=0.9, y=0.08, z=0.9):
        self.goal_pub.publish(Pose(position=Point(x=x, y=y, z=z)))

    def at(self, step):
        return any(msg.data == step for msg in self.statuses)

    def done(self):
        return any(msg.data == 'grab done' for msg in self.results)

    def failed(self):
        return any(msg.data == 'grab failed' for msg in self.results)


def assert_released(h, robot):
    h.pump(0.25)
    assert robot.velocities and stopped(robot.velocities[-1])
    count = len(robot.velocities)
    h.pump(0.15)
    assert len(robot.velocities) == count


def make_cloud(x=0.9, y=0.0, z=0.9, frame='base_footprint', include_plane=True):
    points = [(0.52 + i * 0.02, -0.48 + j * 0.02, 0.72, 0)
              for i in range(48) for j in range(48)] if include_plane else []
    points += [(x - 0.04 + i * 0.008, y - 0.02 + j * 0.008, z + k * 0.008, 0)
               for i in range(6) for j in range(6) for k in range(8)]
    fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(['x', 'y', 'z'])]
    fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    msg = PointCloud2(height=1, width=len(points), fields=fields, point_step=16,
                     row_step=16 * len(points), is_dense=True,
                     data=b''.join(struct.pack('<fffI', *p) for p in points))
    msg.header.frame_id = frame
    return msg


def publish_cloud(h, pub, cloud):
    cloud.header.stamp = h.node.get_clock().now().to_msg()
    pub.publish(cloud)


def start_support_launch(h):
    log = open(h.tmp_path / 'ex15_bringup.log', 'w+')
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).with_name('run_detection_launch.py'))],
        env=h.env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    h.processes.append((process, log))
    return process


def test_ex15_launch_and_autonomous_grab_sequence(h):
    # The launch has absolute topic names; a private DDS domain isolates this test.
    h.prefix = ''
    robot = Robot(h)
    robot.fake_detection = False
    robot.feedback = False
    clouds = h.publisher(PointCloud2, '/camera/depth_registered/points', sensor=True)
    support = start_support_launch(h)
    h.wait(lambda: clouds.get_subscription_count() > 0)
    robot.start()  # 无需开始指令，默认直接运行一次教学流程。

    def frame():
        x, y, z = robot.target()
        publish_cloud(h, clouds, make_cloud(x=x, y=y, z=z))

    h.wait(robot.done, timeout=200, tick=frame)
    assert not robot.failed() and support.poll() is None
    names = h.node.get_node_names()
    assert 'ex15_grab' in names and 'wpr2_objects_3d' in names
    assert 'wpr2_grab' not in names
    assert [m.data for m in robot.results] == ['grab done']
    assert sum(m.data == 'start objects' for m in robot.commands) >= 2
    assert robot.arm_commands['right_arm'][1].position == pytest.approx(
        [1.4813, 1.5723, -1.5426, 1.3623, 1.4895, -1.3667])
    assert robot.arm_commands['right_arm'][-1].position == pytest.approx(
        [-0.1593, 1.5128, -0.0382, 1.7225, 0.0637, 1.5130])
    assert any(m.linear.y > 0 for m in robot.velocities)
    assert any(m.linear.x < 0 for m in robot.velocities)
    assert_released(h, robot)


