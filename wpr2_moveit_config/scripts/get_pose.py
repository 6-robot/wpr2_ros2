#!/usr/bin/env python3
"""Display WPR2 TCP transforms and joint values in the original copyable format."""

import sys
import threading

from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QMessageBox, QPushButton,
                             QTextEdit, QVBoxLayout, QWidget)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException


class PoseProvider(Node):
    def __init__(self):
        super().__init__('wpr2_pose_display')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.positions = {}
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)

    def on_joints(self, msg):
        with self.lock:
            self.positions.update(zip(msg.name, msg.position))

    def snapshot(self, side):
        transform = self.buffer.lookup_transform('base_footprint', f'{side}_tcp_link', Time())
        names = ['chest_height'] + [f'{side}_joint_{i}' for i in range(1, 7)]
        with self.lock:
            missing = [name for name in names if name not in self.positions]
            if missing:
                raise ValueError('缺少关节状态: ' + ', '.join(missing))
            positions = {name: self.positions[name] for name in names}
        return transform, positions


class PoseDisplay(QWidget):
    def __init__(self, provider):
        super().__init__()
        self.provider = provider
        self.pose_text = QTextEdit()
        self.joint_text = QTextEdit()
        self.pose_text.setReadOnly(True)
        self.joint_text.setReadOnly(True)
        layout = QVBoxLayout(self)
        layout.addWidget(self.pose_text)
        layout.addWidget(self.joint_text)
        buttons = QHBoxLayout()
        for side, label in (('left', '左臂姿态'), ('right', '右臂姿态')):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, s=side: self.show_pose(s))
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.setWindowTitle('WPR2 机器人姿态')
        self.resize(570, 570)

    def show_pose(self, side):
        try:
            transform, positions = self.provider.snapshot(side)
        except (TransformException, ValueError) as error:
            QMessageBox.warning(self, '姿态不可用', str(error))
            return
        p = transform.transform.translation
        q = transform.transform.rotation
        self.pose_text.setPlainText(
            'geometry_msgs::Pose target_pose;\n'
            f'target_pose.position.x = {p.x:.4f};\n'
            f'target_pose.position.y = {p.y:.4f};\n'
            f'target_pose.position.z = {p.z:.4f};\n'
            f'target_pose.orientation.x = {q.x:.4f};\n'
            f'target_pose.orientation.y = {q.y:.4f};\n'
            f'target_pose.orientation.z = {q.z:.4f};\n'
            f'target_pose.orientation.w = {q.w:.4f};')
        lines = [f'chest_height = {positions["chest_height"]:.4f};', '']
        for index in range(6):
            lines.append(f'{side}_name[{index}] = "{side}_joint_{index + 1}";')
        lines.append('')
        for index in range(6):
            lines.append(f'{side}_position[{index}] = '
                         f'{positions[f"{side}_joint_{index + 1}"]:.4f};')
        self.joint_text.setPlainText('\n'.join(lines))


def main():
    rclpy.init()
    provider = PoseProvider()
    executor = SingleThreadedExecutor()
    executor.add_node(provider)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    app = QApplication([sys.argv[0]])
    display = PoseDisplay(provider)
    display.show()
    try:
        app.exec_()
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        provider.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
