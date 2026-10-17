"""Exercise fake trajectory execution without sending real motor commands."""

import importlib.util
import os
from pathlib import Path
import threading
import time

os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


spec = importlib.util.spec_from_file_location(
    'trajectory_to_wpr2',
    Path(__file__).resolve().parents[1] / 'scripts/trajectory_to_wpr2.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_fake_trajectory_reaches_action_and_driver_topics():
    rclpy.init()
    bridge = module.TrajectoryBridge(
        parameter_overrides=[Parameter('fake_hardware', value=True)])
    client_node = Node('wpr2_trajectory_test')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(bridge)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    received = []
    driver_commands = []
    subscription = client_node.create_subscription(
        JointState, 'joint_states', lambda msg: received.append(msg), 10)
    driver_subscription = client_node.create_subscription(
        JointState, 'wpr2/left_arm', lambda msg: driver_commands.append(msg), 10)
    action = ActionClient(client_node, FollowJointTrajectory,
                          'left_arm_controller/follow_joint_trajectory')

    def wait_for(future):
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done()
        return future.result()

    try:
        assert action.wait_for_server(timeout_sec=5)
        deadline = time.monotonic() + 2
        while not received and time.monotonic() < deadline:
            time.sleep(0.01)
        assert received
        initial = dict(zip(received[-1].name, received[-1].position))
        assert initial['chest_height'] == 0.7
        assert initial['femto_height'] == 0.15
        assert initial['left_joint_1'] == 0.5
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(module.ARMS['left'])
        point = JointTrajectoryPoint()
        point.positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        point.time_from_start.nanosec = 100_000_000
        goal.trajectory.points = [point]
        sent = action.send_goal_async(goal)
        handle = wait_for(sent)
        assert handle.accepted
        result = wait_for(handle.get_result_async())
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        deadline = time.monotonic() + 2
        while (not received or received[-1].position[received[-1].name.index('left_joint_1')] != 0.1) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert received
        positions = dict(zip(received[-1].name, received[-1].position))
        assert [positions[name] for name in module.ARMS['left']] == list(point.positions)
        assert not driver_commands
    finally:
        client_node.destroy_subscription(subscription)
        client_node.destroy_subscription(driver_subscription)
        action.destroy()
        executor.shutdown()
        thread.join(timeout=5)
        client_node.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


def test_real_bridge_reorders_joints_and_waits_for_feedback():
    rclpy.init()
    bridge = module.TrajectoryBridge()
    client_node = Node('wpr2_real_bridge_test')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(bridge)
    executor.add_node(client_node)
    lock = threading.Lock()
    positions = {'chest_height': 0.7}
    positions.update({name: 0.0 for name in module.ARMS['left']})
    seen = {'arm': None, 'lift': None}
    state_pub = client_node.create_publisher(JointState, 'joint_states', 10)

    def publish_state():
        msg = JointState()
        msg.header.stamp = client_node.get_clock().now().to_msg()
        with lock:
            msg.name = list(positions)
            msg.position = list(positions.values())
        state_pub.publish(msg)

    def on_arm(msg):
        with lock:
            seen['arm'] = msg
            positions.update(zip(msg.name, msg.position))

    def on_lift(msg):
        with lock:
            seen['lift'] = msg
            positions.update(zip(msg.name, msg.position))

    timer = client_node.create_timer(0.05, publish_state)
    arm_sub = client_node.create_subscription(JointState, 'wpr2/left_arm', on_arm, 10)
    lift_sub = client_node.create_subscription(JointState, 'wpr2/chest_height', on_lift, 10)
    action = ActionClient(client_node, FollowJointTrajectory,
                          'left_arm_lift_controller/follow_joint_trajectory')
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def wait_for(future):
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done()
        return future.result()

    try:
        assert action.wait_for_server(timeout_sec=5)
        deadline = time.monotonic() + 3
        while bridge.last_state == 0.0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert bridge.last_state > 0.0
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(reversed(('chest_height',) + module.ARMS['left']))
        point = JointTrajectoryPoint()
        point.positions = [0.1 * (i + 1) for i in range(6)] + [0.6]
        point.time_from_start.nanosec = 100_000_000
        goal.trajectory.points = [point]
        handle = wait_for(action.send_goal_async(goal))
        assert handle.accepted
        result = wait_for(handle.get_result_async())
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        with lock:
            assert list(seen['arm'].name) == list(module.ARMS['left'])
            assert list(seen['arm'].position) == list(reversed(point.positions[:6]))
            assert list(seen['lift'].position) == [0.6]
    finally:
        action.destroy()
        client_node.destroy_subscription(arm_sub)
        client_node.destroy_subscription(lift_sub)
        client_node.destroy_timer(timer)
        executor.shutdown()
        thread.join(timeout=5)
        client_node.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
