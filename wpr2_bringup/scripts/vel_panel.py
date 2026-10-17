#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import time
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLineEdit, QGridLayout, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer
from geometry_msgs.msg import Twist

class VelocityControlPanel(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.command = Twist()
        self.stop_deadline = 0.0
        
        # 速度话题发布者将在发送速度时动态创建
        self.cmd_vel_pub = None
        # 用于存储自动停止的定时器
        self.stop_timer = None

        # 初始化UI
        self.init_ui()

    def init_ui(self):
        # 窗口标题和大小
        self.setWindowTitle('机器人速度控制面板')
        self.setGeometry(300, 300, 350, 400) # 调整窗口大小

        # === UI 控件 ===
        # 速度话题输入框
        self.topic_label = QLabel('速度话题:')
        self.topic_input = QLineEdit(self)
        self.topic_input.setText("cmd_vel")

        # 3x3 按钮网格
        grid_layout = QGridLayout()
        grid_layout.setSpacing(10)

        # 定义按钮及其位置、文本和回调函数
        buttons_info = {
            'turn_left':    (0, 0, '↶', self.turn_left),
            'forward':      (0, 1, '↑', self.move_forward),
            'turn_right':   (0, 2, '↷', self.turn_right),
            'left':         (1, 0, '←', self.move_left),
            'stop':         (1, 1, '■', self.stop_robot),
            'right':        (1, 2, '→', self.move_right),
            'backward':     (2, 1, '↓', self.move_backward),
        }

        # 创建并添加按钮到网格布局
        for name, (row, col, text, callback) in buttons_info.items():
            button = QPushButton(text, self)
            button.setFixedSize(80, 80)
            button.clicked.connect(callback)
            grid_layout.addWidget(button, row, col)

        # === 整体布局 ===
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.topic_label)
        main_layout.addWidget(self.topic_input)
        main_layout.addLayout(grid_layout)
        
        self.setLayout(main_layout)

        # 创建并启动一个定时器来检查ROS关闭信号
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.check_ros_shutdown)
        self.ros_timer.start(100)  # 设置检查间隔为100毫秒

        self.show()

    def check_ros_shutdown(self):
        """
        处理 ROS 2 回调、速度续发和关闭信号。
        """
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.cmd_vel_pub is not None:
            if time.monotonic() >= self.stop_deadline:
                self.command = Twist()
            self.cmd_vel_pub.publish(self.command)

    def _get_velocity_publisher(self):
        """ 根据输入框中的话题名创建或复用发布者 """
        topic_name = self.topic_input.text()
        if not topic_name:
            self.node.get_logger().warning("速度话题为空,无法发布.")
            return None
            
        if self.cmd_vel_pub is None or self.cmd_vel_pub.topic_name != self.node.resolve_topic_name(topic_name):
            self.node.get_logger().info(f"正在为话题 '{topic_name}' 创建发布者...")
            try:
                if self.cmd_vel_pub is not None:
                    self.cmd_vel_pub.publish(Twist())
                    self.node.destroy_publisher(self.cmd_vel_pub)
                self.cmd_vel_pub = self.node.create_publisher(Twist, topic_name, 10)
            except Exception as e:
                self.node.get_logger().error(f"创建发布者失败: {e}")
                return None
        return self.cmd_vel_pub

    def _publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, auto_stop=True):
        """ 发布速度指令，并按需设置一个3秒后停止的定时器 """
        pub = self._get_velocity_publisher()
        if pub is None:
            return

        if self.stop_timer is not None:
            self.stop_timer.stop()
            self.stop_timer = None
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        
        self.command = twist
        pub.publish(twist)
        self.node.get_logger().info(f"发布速度到话题 '{pub.topic_name}': linear=({linear_x}, {linear_y}), angular={angular_z}")

        self.stop_deadline = time.monotonic() + 3.0 if auto_stop else 0.0

    # --- 速度控制按钮回调函数 ---
    def move_forward(self):
        self._publish_velocity(linear_x=0.1)

    def move_backward(self):
        self._publish_velocity(linear_x=-0.1)

    def move_left(self):
        self._publish_velocity(linear_y=0.1)

    def move_right(self):
        self._publish_velocity(linear_y=-0.1)

    def turn_left(self):
        self._publish_velocity(angular_z=0.5)

    def turn_right(self):
        self._publish_velocity(angular_z=-0.5)

    def stop_robot(self, event=None):
        """ 停止机器人运动 """
        self.node.get_logger().info("发送停止指令")
        self._publish_velocity(auto_stop=False)


    def closeEvent(self, event):
        if rclpy.ok() and self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(Twist())
        event.accept()


if __name__ == '__main__':
    rclpy.init()
    node = Node('velocity_control_panel')
    app = QApplication(remove_ros_args(sys.argv))
    panel = VelocityControlPanel(node)
    app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
