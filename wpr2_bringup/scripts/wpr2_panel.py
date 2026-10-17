#!/usr/bin/env python3
"""Combined ROS 2 velocity, gripper and planned-arm controls."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QLabel, QSlider, QPushButton
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from vel_panel import VelocityControlPanel


class WPR2Panel(VelocityControlPanel):
    def __init__(self, node):
        super().__init__(node)
        self.setWindowTitle('WPR2 控制面板')
        self.commands = node.create_publisher(String, '/plan/wpr2_cmd', 10)
        self.grippers = {}
        for side in ['left', 'right']:
            self.grippers[side] = node.create_publisher(JointState, f'/real/wpr2/{side}_gripper', 10)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 155)
            slider.setValue(100)
            self.layout().addWidget(QLabel(f'{side} gripper (mm)'))
            self.layout().addWidget(slider)
            slider.valueChanged.connect(lambda value, side=side: self.move_gripper(side, value))
        for command in ['reset', 'go']:
            button = QPushButton(command)
            button.clicked.connect(lambda checked=False, command=command: self.commands.publish(String(data=command)))
            self.layout().addWidget(button)

    def move_gripper(self, side, value):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.position = [value / 1000.0]
        msg.velocity = [2000.0]
        self.grippers[side].publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = Node('wpr2_panel')
    app = QApplication(remove_ros_args(sys.argv))
    panel = WPR2Panel(node)
    app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
