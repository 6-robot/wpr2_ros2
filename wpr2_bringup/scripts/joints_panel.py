#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class TfPublisherGui(QWidget):
    def __init__(self, node):
        super(TfPublisherGui, self).__init__()

        self.node = node

        self.joint_states_pub = self.node.create_publisher(JointState, 'joint_states', 10)
        self.left_gripper_pub = self.node.create_publisher(JointState, '/real/wpr2/left_gripper', 10)
        self.right_gripper_pub = self.node.create_publisher(JointState, '/real/wpr2/right_gripper', 10)
        self.cmd_sub = self.node.create_subscription(String, "wpr2_cmd", self.receive_command, 10)
        self.cmd_pub = self.node.create_publisher(String, 'wpr2_cmd', 10)

        self.femto_height_pos = 0.0
        self.femto_pitch_pos = 0.0

        self.initial_slider_values = {}

        self.init_ui()

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.publish_ros_topics)
        self.ros_timer.start(100)

    def receive_command(self, msg):
        if msg.data == 'reset':
            for name, slider in self.sliders.items():
                slider.setValue(self.initial_slider_values[name])

    def init_ui(self):
        self.sliders = {}
        self.labels = {}
        
        top_level_layout = QVBoxLayout()
        
        slider_layout = QHBoxLayout()
        left_layout = self.create_arm_layout("left")
        middle_layout = self.create_chest_layout()
        right_layout = self.create_arm_layout("right")
        slider_layout.addLayout(left_layout)
        slider_layout.addLayout(middle_layout)
        slider_layout.addLayout(right_layout)

        top_level_layout.addLayout(slider_layout)

        button_layout = QHBoxLayout()
        button_layout.addStretch(1) 
        
        reset_button = QPushButton("重置")
        reset_button.clicked.connect(self.on_reset_clicked)
        button_layout.addWidget(reset_button)

        go_button = QPushButton("执行")
        go_button.clicked.connect(self.on_go_clicked)
        button_layout.addWidget(go_button)

        top_level_layout.addLayout(button_layout)

        self.setLayout(top_level_layout)
        self.setWindowTitle('Joints Panel')
        
        self.set_initial_positions()
        
        self.show()

    def on_reset_clicked(self):
        msg = String()
        msg.data = "reset"
        self.cmd_pub.publish(msg)
        self.node.get_logger().info("发布命令: reset 到 /wpr2_cmd")

        for joint_name, initial_value in self.initial_slider_values.items():
            if joint_name in self.sliders:
                self.sliders[joint_name].setValue(initial_value)
        self.node.get_logger().info("所有滑块已重置到初始位置。")

    def on_go_clicked(self):
        self.publish_grippers()
        msg = String()
        msg.data = "go"
        self.cmd_pub.publish(msg)
        self.node.get_logger().info("发布命令: go 到 /wpr2_cmd")

    def create_arm_layout(self, side):
        layout = QVBoxLayout()
        joint_names = [f"{side}_joint_{i}" for i in range(1, 7)] + [f"{side}_gripper"]

        for name in joint_names:
            label_layout = QHBoxLayout()
            label_name = QLabel(name)
            label_value = QLabel("0.00")
            
            label_layout.addWidget(label_name)
            label_layout.addStretch(1)
            label_layout.addWidget(label_value)
            
            self.labels[name] = label_value
            layout.addLayout(label_layout)
            
            slider = QSlider(Qt.Horizontal)
            slider.setFixedWidth(200)

            if "gripper" in name:
                slider.setRange(0, 155)
            else:
                slider.setRange(-314, 314)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, n=name: self.update_label(n, value))
            layout.addWidget(slider)
            
            self.sliders[name] = slider

        return layout

    def create_chest_layout(self):
        layout = QVBoxLayout()
        name = "chest_height"

        label_name = QLabel(name)
        layout.addWidget(label_name, alignment=Qt.AlignCenter)

        slider = QSlider(Qt.Vertical)
        slider.setFixedHeight(300)

        slider.setRange(0, 70)
        slider.setValue(0)
        layout.addWidget(slider, alignment=Qt.AlignCenter)

        label_value = QLabel("0.00")
        layout.addWidget(label_value, alignment=Qt.AlignCenter)
        self.labels[name] = label_value

        slider.valueChanged.connect(lambda value, n=name: self.update_label(n, value))
        self.sliders[name] = slider

        return layout

    # *** 确保所有滑块都有初始值 ***
    def set_initial_positions(self):
        """
        为所有滑块建立一个完整的初始值记录，然后应用到UI上。
        """
        # 1. 首先，为所有存在的滑块设置一个默认的初始值0。
        for joint_name in self.sliders.keys():
            self.initial_slider_values[joint_name] = 0
        self.node.get_logger().info("已为所有滑块建立默认初始值(0)。")

        # 2. 然后，从ROS参数服务器加载值来覆盖默认值。
        try:
            initial_positions_from_param = {name: param.value for name, param in self.node.get_parameters_by_prefix('zeros').items()}
            self.node.get_logger().info("成功加载初始位置参数。")
        except Exception:
            self.node.get_logger().warning("无法从参数服务器找到'zeros'参数。")
            initial_positions_from_param = {}

        for joint_name, position in initial_positions_from_param.items():
            if joint_name in self.sliders:
                slider_value = 0
                if "gripper" in joint_name: slider_value = int(position * 1000)
                elif joint_name == "chest_height": slider_value = int(position * 100)
                else: slider_value = int(position * 100)
                # 覆盖默认值
                self.initial_slider_values[joint_name] = slider_value
            
            elif joint_name == "femto_height": self.femto_height_pos = position
            elif joint_name == "femto_pitch": self.femto_pitch_pos = position
        
        # 3. 最后，用硬编码的值覆盖Gripper的初始值。
        gripper_initial_float = 0.1 
        gripper_initial_slider = int(gripper_initial_float * 1000)
        self.initial_slider_values['left_gripper'] = gripper_initial_slider
        self.initial_slider_values['right_gripper'] = gripper_initial_slider
        self.node.get_logger().info(f"为Grippers设置了硬编码的初始值: {gripper_initial_float}")

        # 4. 现在字典是完整的，将所有值应用到UI滑块。
        for joint_name, value in self.initial_slider_values.items():
            if joint_name in self.sliders:
                self.sliders[joint_name].setValue(value)
        self.node.get_logger().info("所有滑块的初始位置已设置完毕。")

    def update_label(self, name, value):
        float_value = 0.0
        if "gripper" in name: float_value = value / 1000.0
        elif name == "chest_height": float_value = value / 100.0
        else: float_value = value / 100.0
        self.labels[name].setText(f"{float_value:.2f}")

    def publish_ros_topics(self):
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if len(self.sliders) < 15: return

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        gui_joint_names = [f"left_joint_{i}" for i in range(1, 7)] + \
                          [f"right_joint_{i}" for i in range(1, 7)] + \
                          ["chest_height"]
        
        joint_state_msg.name = gui_joint_names
        joint_state_msg.position = []
        for name in gui_joint_names:
            scale = 100.0
            if name == "chest_height": scale = 100.0
            joint_state_msg.position.append(self.sliders[name].value() / scale)

        non_gui_names = ["front_left_wheel_joint", "front_right_wheel_joint", "back_right_wheel_joint",
                         "back_left_wheel_joint", "femto_height", "femto_pitch"]
        non_gui_positions = [0.0, 0.0, 0.0, 0.0, self.femto_height_pos, self.femto_pitch_pos]
        joint_state_msg.name.extend(non_gui_names)
        joint_state_msg.position.extend(non_gui_positions)

        gripper_names = ["left_joint_finger","right_joint_finger"]
        gripper_positions = [
            (155 - self.sliders[side + "_gripper"].value()) / 155.0 * 1.57
            for side in ["left", "right"]
        ]
        joint_state_msg.name.extend(gripper_names)
        joint_state_msg.position.extend(gripper_positions)

        self.joint_states_pub.publish(joint_state_msg)

    def publish_grippers(self):
        left_gripper_msg = JointState()
        left_gripper_msg.header.stamp = self.node.get_clock().now().to_msg()
        left_gripper_msg.name = ["left_gripper"]
        left_gripper_msg.position = [self.sliders["left_gripper"].value() / 1000.0]
        left_gripper_msg.velocity = [2000.0]
        self.left_gripper_pub.publish(left_gripper_msg)

        right_gripper_msg = JointState()
        right_gripper_msg.header.stamp = self.node.get_clock().now().to_msg()
        right_gripper_msg.name = ["right_gripper"]
        right_gripper_msg.position = [self.sliders["right_gripper"].value() / 1000.0]
        right_gripper_msg.velocity = [2000.0]
        self.right_gripper_pub.publish(right_gripper_msg)

if __name__ == '__main__':
    rclpy.init()
    node = Node('joints_panel', automatically_declare_parameters_from_overrides=True)
    app = QApplication(remove_ros_args(sys.argv))
    panel = TfPublisherGui(node)
    app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
