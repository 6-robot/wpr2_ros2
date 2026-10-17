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
from geometry_msgs.msg import Point, Pose, PoseStamped, Pose2D, Twist
from sensor_msgs.msg import Imu, JointState, LaserScan, Image, RegionOfInterest, PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
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
        path = Path(os.environ['BEHAVIORS_BIN_DIR']) / executable
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

    def start(self, **params):
        defaults = dict(joint_min_wait=0.02, grip_min_wait=0.03, grip_timeout=0.2,
                        reset_wait=0.03, stop_wait=0.03, ready_timeout=2.0,
                        joint_timeout=1.0, joint_feedback_timeout=0.3, pose_timeout=0.3,
                        motion_timeout=8.0, total_timeout=25.0,
                        align_speed=0.2, forward_speed=0.2, move_gain=3.0)
        defaults.update(params)
        self.process = self.h.start('wpr2_grab', **defaults)
        self.h.wait(lambda: self.goal_pub.get_subscription_count() > 0 and
                    self.behavior_pub.get_subscription_count() >= 2)

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


def test_direct_coordinate_full_sequence_and_exact_ros1_poses(h):
    robot = Robot(h)
    robot.start()
    h.pump(0.1)
    assert not robot.velocities and not any(robot.arm_commands.values())
    robot.goal()
    h.wait(robot.done)
    assert not robot.failed()
    assert [msg.data for msg in robot.results] == ['grab done']
    right = robot.arm_commands['right_arm']
    assert len(right) == 4
    assert right[0].position == pytest.approx([-0.6, 1.57, 0, 1.5, 0, 0])
    assert right[1].position == pytest.approx([1.4813, 1.5723, -1.5426, 1.3623, 1.4895, -1.3667])
    assert right[2].position == pytest.approx([1.6673, 1.5699, -1.5818, 1.3601, 1.6793, -1.3631])
    assert right[3].position == pytest.approx([-0.1593, 1.5128, -0.0382, 1.7225, 0.0637, 1.5130])
    assert all(list(msg.velocity) == [2000.0] * 6 for msg in right)
    lifts = robot.arm_commands['chest_height']
    assert [m.position[0] for m in lifts] == pytest.approx([0.5, 0.7])
    assert all(list(m.velocity) == [15000.0] for m in lifts)
    assert [m.position[0] for m in robot.arm_commands['right_gripper']] == pytest.approx([0.15, 0.06])
    assert len(robot.resets) == 3
    assert all(m.data == 'odom_delta reset' for m in robot.resets)
    assert any(m.linear.y > 0 for m in robot.velocities)
    assert any(m.linear.x < 0 for m in robot.velocities)
    assert all(abs(m.linear.x) <= 0.2 and abs(m.linear.y) <= 0.2 for m in robot.velocities)
    # The terminal status and result are on separate DDS topics and can arrive in either order.
    h.wait(lambda: robot.at('done'))
    assert robot.at('hold_arm')
    assert_released(h, robot)


def test_detector_input_rechecks_after_alignment_and_adjusts_lift(h):
    robot = Robot(h)
    robot.world_target = (0.88, -0.07, 0.85)
    robot.start(auto_start=True)
    h.wait(robot.done)
    assert sum(m.data == 'start objects' for m in robot.commands) >= 2
    assert any(m.position[0] == pytest.approx(0.45) for m in robot.arm_commands['chest_height'])
    assert not robot.failed()
    assert_released(h, robot)


def test_already_aligned_still_adjusts_chest(h):
    robot = Robot(h)
    robot.start()
    robot.goal(x=0.7, y=0.0, z=0.85)
    h.wait(lambda: robot.at('forward_reset'))
    assert not robot.at('align_move')
    assert [m.position[0] for m in robot.arm_commands['chest_height']][:2] == pytest.approx([0.5, 0.45])
    robot.behavior_pub.publish(String(data='stop grab'))
    h.wait(robot.failed)


@pytest.mark.parametrize('stop_topic', ['behavior', 'legacy'])
def test_cancel_moving_hold_and_restart(h, stop_topic):
    robot = Robot(h)
    robot.start()
    robot.goal()
    h.wait(lambda: robot.at('align_move') and any(m.linear.x > 0 for m in robot.velocities))
    # A new invalid coordinate cannot replace the active goal or report another result.
    robot.goal(x=-1.0)
    # Different DDS topics are unordered: observe goal rejection before sending cancellation.
    h.wait(lambda: 'duplicate start ignored' in Path(h.processes[-1][1].name).read_text())
    pub = robot.behavior_pub if stop_topic == 'behavior' else robot.legacy_behavior_pub
    pub.publish(String(data='grab stop'))
    h.wait(robot.failed)
    assert [m.data for m in robot.results] == ['grab failed']
    assert robot.arm_commands['right_arm'][-1].position == pytest.approx([-0.6, 1.57, 0, 1.5, 0, 0])
    assert_released(h, robot)
    robot.statuses.clear()
    robot.goal(x=0.7, y=0.0)
    h.wait(robot.done)
    assert [m.data for m in robot.results] == ['grab failed', 'grab done']


@pytest.mark.parametrize('fault', ['odom', 'joints', 'invalid_odom'])
def test_feedback_loss_during_motion_fails_and_stops(h, fault):
    robot = Robot(h)
    robot.start()
    robot.goal()
    h.wait(lambda: robot.at('align_move') and any(m.linear.x > 0 for m in robot.velocities))
    if fault == 'odom':
        robot.pose_feedback = False
    elif fault == 'joints':
        robot.feedback = False
    else:
        robot.pose_feedback = False
        robot.pose_pub.publish(Pose2D(x=float('nan')))
    h.wait(robot.failed)
    assert not robot.done()
    assert_released(h, robot)


def test_reset_requires_new_zero_feedback(h):
    robot = Robot(h)
    robot.ack_reset = False
    robot.start(motion_timeout=0.25)
    robot.goal()
    h.wait(robot.failed)
    assert robot.at('align_reset') and not robot.at('align_move')
    assert all(stopped(m) for m in robot.velocities)


def test_stuck_chassis_motion_timeout(h):
    robot = Robot(h)
    robot.integrate = False
    robot.start(motion_timeout=0.3)
    robot.goal()
    h.wait(robot.failed)
    assert robot.at('align_move') and not robot.done()
    assert_released(h, robot)


def test_joint_motion_timeout_and_no_false_success(h):
    robot = Robot(h, follow=False)
    robot.start(joint_timeout=0.25)
    robot.goal(x=0.7, y=0.0)
    h.wait(robot.failed)
    assert not robot.done() and all(stopped(m) for m in robot.velocities)


def test_final_holding_pose_must_arrive(h):
    robot = Robot(h)
    robot.freeze_holding = True
    robot.start(joint_timeout=0.25)
    robot.goal(x=0.7, y=0.0)
    h.wait(robot.failed)
    assert robot.at('hold_arm') and not robot.done()
    assert_released(h, robot)


def test_gripper_contact_timeout_allows_raise(h):
    robot = Robot(h)
    robot.follow_grip = False
    robot.start()
    robot.goal(x=0.7, y=0.0)
    h.wait(robot.done)
    assert robot.at('grip') and robot.at('raise_arm') and not robot.failed()


def test_invalid_targets_are_rejected_without_motion(h):
    robot = Robot(h)
    robot.start()
    for x, y, z in [(-1.0, 0.0, 0.9), (0.9, 1.0, 0.9), (0.9, 0.0, 0.5),
                    (0.9, 0.0, 1.3), (float('nan'), 0.0, 0.9)]:
        count = len(robot.results)
        robot.goal(x=x, y=y, z=z)
        h.wait(lambda: len(robot.results) > count)
        assert robot.results[-1].data == 'grab failed'
    assert not robot.velocities and not any(robot.arm_commands.values())


def test_stamped_goal_frame_age_and_valid_goal(h):
    robot = Robot(h)
    robot.start()
    msg = PoseStamped(pose=Pose(position=Point(x=0.7, y=0.0, z=0.9)))
    msg.header.frame_id = 'camera_link'
    robot.stamped_pub.publish(msg)
    h.wait(robot.failed)
    assert not any(robot.arm_commands.values())
    robot.results.clear()
    msg.header.frame_id = 'base_footprint'
    msg.header.stamp.sec = 1
    robot.stamped_pub.publish(msg)
    h.wait(robot.failed)
    assert not any(robot.arm_commands.values())
    robot.results.clear()
    msg.header.stamp.sec = 0
    robot.stamped_pub.publish(msg)
    h.wait(robot.done)
    assert not robot.failed()


def test_detection_timeout_no_success_and_old_target_ignored(h):
    robot = Robot(h)
    robot.fake_detection = False
    robot.start(auto_start=True, detection_timeout=0.3)
    h.wait(lambda: robot.detect)
    msg = Object(x=[0.7], y=[0.0], z=[0.9])
    msg.header.frame_id = 'base_footprint'
    robot.object_pub.publish(msg)  # A zero/old timestamp is not a fresh detection.
    h.wait(robot.failed)
    assert not robot.done()
    assert [m.data for m in robot.results] == ['grab failed']
    assert_released(h, robot)


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


def test_detector_stability_lock_tracking_and_loss(h):
    clouds = h.publisher(PointCloud2, '/camera/depth_registered/points', sensor=True)
    behavior = h.publisher(String, '/wpr2/behavior')
    objects = h.collect(Object, '/wpr2/objects_3d')
    markers = h.collect(MarkerArray, '/objects_marker')
    process = h.start('wpr2_objects_3d', auto_start=True, object_min_height=0.02, min_cluster_size=30)
    h.wait(lambda: clouds.get_subscription_count() > 0 and behavior.get_subscription_count() > 0)
    cloud = make_cloud()
    publish_cloud(h, clouds, cloud)
    h.pump(0.15)
    assert not objects
    publish_cloud(h, clouds, cloud)
    h.pump(0.15)
    assert not objects
    h.wait(lambda: objects, tick=lambda: publish_cloud(h, clouds, cloud))
    assert objects[-1].header.frame_id == 'base_footprint'
    assert objects[-1].x[0] == pytest.approx(0.9, abs=0.002)  # xMax, not centroid
    assert objects[-1].y[0] == pytest.approx(0.0, abs=0.002)
    assert objects[-1].z[0] == pytest.approx(0.9, abs=0.002)  # zMin
    # Locked mode retains its plane height and follows a target using a local sphere.
    local = make_cloud(x=0.92, include_plane=False)
    objects.clear()
    h.wait(lambda: objects, tick=lambda: publish_cloud(h, clouds, local))
    assert objects[-1].x[0] == pytest.approx(0.92, abs=0.002)
    h.pump()
    objects.clear()
    markers.clear()
    h.wait(lambda: markers and len(markers[-1].markers) == 1,
           tick=lambda: clouds.publish(PointCloud2()))
    h.pump()
    assert not objects
    assert markers[-1].markers[0].action == 3
    # Restart forces search/stability again; stopped detector never republishes old coordinates.
    behavior.publish(String(data='stop objects'))
    h.pump()
    for _ in range(4):
        publish_cloud(h, clouds, cloud)
        h.pump(0.03)
    assert not objects and process.poll() is None


def test_detector_and_grab_end_to_end(h):
    robot = Robot(h)
    robot.fake_detection = False
    robot.world_target = (0.9, 0.08, 0.9)
    clouds = h.publisher(PointCloud2, '/camera/depth_registered/points', sensor=True)
    h.start('wpr2_objects_3d', object_min_height=0.02, min_cluster_size=30)
    robot.start(auto_start=True)
    def frame():
        x, y, z = robot.target()
        publish_cloud(h, clouds, make_cloud(x=x, y=y, z=z))
    h.wait(robot.done, timeout=15, tick=frame)
    assert not robot.failed()
    assert sum(m.data == 'start objects' for m in robot.commands) >= 2
    assert_released(h, robot)


def test_local_message_dependencies_and_launch_defaults():
    import ast
    import xml.etree.ElementTree as ET
    import yaml
    root = Path(__file__).resolve().parents[1]
    for name in ['wpr2_behaviors', 'wpr2_msgs']:
        manifest = ET.parse(root.parent / name / 'package.xml')
        assert manifest.findtext('name') == name
        assert not any('wpb_home' in (e.text or '') or 'wpr_simulation' in (e.text or '')
                       for e in manifest.getroot())
    for launch in (root / 'launch').glob('*.py'):
        ast.parse(launch.read_text())
        assert "executable='wpr2_core'" not in launch.read_text()
    params = yaml.safe_load((root / 'config/behaviors.yaml').read_text())
    assert params['wpr2_grab']['ros__parameters']['auto_start'] is False
    assert params['wpr2_objects_3d']['ros__parameters']['stable_frame_count'] == 3


def test_shutdown_active_task_stops_and_holds(h):
    robot = Robot(h)
    robot.start()
    robot.goal()
    h.wait(lambda: robot.at('align_move') and any(m.linear.x > 0 for m in robot.velocities))
    robot.process.send_signal(signal.SIGINT)
    h.wait(lambda: robot.process.poll() is not None)
    h.pump(0.2)
    assert robot.process.returncode == 0
    assert robot.failed() and not robot.done()
    assert stopped(robot.velocities[-1])
    assert len(robot.arm_commands['right_arm']) == 2  # observe, then measured-position hold
