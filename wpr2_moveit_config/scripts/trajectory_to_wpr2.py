#!/usr/bin/env python3
"""FollowJointTrajectory actions backed by WPR2 JointState command topics."""

import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARMS = {
    'left': tuple(f'left_joint_{i}' for i in range(1, 7)),
    'right': tuple(f'right_joint_{i}' for i in range(1, 7)),
}
CONTROLLERS = {
    f'{side}_arm_controller': joints
    for side, joints in ARMS.items()
}
CONTROLLERS.update({
    f'{side}_arm_lift_controller': ('chest_height',) + joints
    for side, joints in ARMS.items()
})


class TrajectoryBridge(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('trajectory_action_bridge', parameter_overrides=parameter_overrides)
        self.fake = self.declare_parameter('fake_hardware', False).value
        self.tolerance = self.declare_parameter('goal_tolerance', 0.1).value
        self.lock = threading.Lock()
        self.active_joints = set()
        self.positions = {name: 0.0 for joints in ARMS.values() for name in joints}
        self.positions.update(chest_height=0.7, femto_height=0.15, femto_pitch=0.02,
                              left_joint_finger=0.05, right_joint_finger=0.05)
        if self.fake:
            self.positions.update(zip(ARMS['left'], (0.5, -1.57, 0.0, -1.0, 0.0, 0.0)))
            self.positions.update(zip(ARMS['right'], (-0.5, 1.57, 0.0, 1.0, 0.0, 0.0)))
        self.positions.update({f'{front}_{side}_wheel_joint': 0.0
                               for front in ('front', 'back') for side in ('left', 'right')})
        self.last_state = 0.0
        self.command_pubs = {} if self.fake else {
            name: self.create_publisher(JointState, f'wpr2/{name}', 10)
            for name in ('left_arm', 'right_arm', 'chest_height')
        }
        self.callback_group = ReentrantCallbackGroup()
        if self.fake:
            self.state_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.create_timer(0.05, self.publish_fake_state,
                              callback_group=self.callback_group)
        else:
            self.create_subscription(JointState, 'joint_states', self.on_state, 10,
                                     callback_group=self.callback_group)
        self.servers = [
            ActionServer(self, FollowJointTrajectory, f'{name}/follow_joint_trajectory',
                         execute_callback=lambda handle, n=name: self.execute(n, handle),
                         goal_callback=lambda request, n=name: self.accept(n, request),
                         cancel_callback=lambda handle: CancelResponse.ACCEPT,
                         callback_group=self.callback_group)
            for name in CONTROLLERS
        ]
        self.get_logger().info('WPR2 trajectory bridge: %s' %
                               ('fake hardware' if self.fake else 'real hardware'))

    def on_state(self, state):
        with self.lock:
            self.positions.update(zip(state.name, state.position))
            self.last_state = time.monotonic()

    def publish_fake_state(self):
        with self.lock:
            names = list(self.positions)
            positions = [self.positions[name] for name in names]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.state_pub.publish(msg)

    def accept(self, controller, request):
        expected = CONTROLLERS[controller]
        names = request.trajectory.joint_names
        points = request.trajectory.points
        if len(names) != len(expected) or set(names) != set(expected) or not points:
            return GoalResponse.REJECT
        previous = -1.0
        for point in points:
            when = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if (len(point.positions) != len(names) or
                    any(not math.isfinite(x) for x in point.positions) or when < previous):
                return GoalResponse.REJECT
            previous = when
        with self.lock:
            if self.active_joints.intersection(expected):
                return GoalResponse.REJECT
            if not self.fake and time.monotonic() - self.last_state > 1.0:
                self.get_logger().warn('Rejecting trajectory: no recent joint feedback')
                return GoalResponse.REJECT
            self.active_joints.update(expected)
        return GoalResponse.ACCEPT

    def command(self, controller, names, positions):
        requested = dict(zip(names, positions))
        if self.fake:
            with self.lock:
                self.positions.update(requested)
            return
        side = controller.split('_')[0]
        arm = JointState()
        arm.header.stamp = self.get_clock().now().to_msg()
        arm.name = list(ARMS[side])
        arm.position = [requested[name] for name in arm.name]
        arm.velocity = [2000.0] * 6  # WPR2 motor speed units, as in the ROS 1 bridge.
        self.command_pubs[f'{side}_arm'].publish(arm)
        if 'chest_height' in requested:
            lift = JointState()
            lift.header.stamp = arm.header.stamp
            lift.name = ['chest_height']
            lift.position = [requested['chest_height']]
            lift.velocity = [10000.0]
            self.command_pubs['chest_height'].publish(lift)

    def hold(self, controller):
        with self.lock:
            names = CONTROLLERS[controller]
            positions = [self.positions[name] for name in names]
        self.command(controller, names, positions)

    def execute(self, controller, handle):
        trajectory = handle.request.trajectory
        names = trajectory.joint_names
        result = FollowJointTrajectory.Result()
        start = time.monotonic()
        try:
            for point in trajectory.points:
                due = start + point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                while time.monotonic() < due:
                    if handle.is_cancel_requested:
                        self.hold(controller)
                        handle.canceled()
                        return result
                    time.sleep(min(0.01, max(0.0, due - time.monotonic())))
                if handle.is_cancel_requested:
                    self.hold(controller)
                    handle.canceled()
                    return result
                self.command(controller, names, point.positions)
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = names
                feedback.desired = point
                with self.lock:
                    feedback.actual = JointTrajectoryPoint()
                    feedback.actual.positions = [self.positions[name] for name in names]
                handle.publish_feedback(feedback)

            final = trajectory.points[-1]
            tolerance = (handle.request.goal_time_tolerance.sec +
                         handle.request.goal_time_tolerance.nanosec * 1e-9) or 3.0
            deadline = time.monotonic() + tolerance
            while time.monotonic() <= deadline:
                if handle.is_cancel_requested:
                    self.hold(controller)
                    handle.canceled()
                    return result
                with self.lock:
                    arrived = (self.fake or time.monotonic() - self.last_state < 1.0) and all(
                        abs(self.positions[name] - target) <=
                        (0.02 if name == 'chest_height' else self.tolerance)
                        for name, target in zip(names, final.positions))
                if arrived:
                    handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                time.sleep(0.02)
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = 'Joint feedback did not reach the final trajectory point'
            handle.abort()
            return result
        finally:
            with self.lock:
                self.active_joints.difference_update(CONTROLLERS[controller])


def main():
    rclpy.init()
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
