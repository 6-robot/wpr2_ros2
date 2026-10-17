#!/usr/bin/env bash
set -euo pipefail

# 覆盖 wpr2_ros2 六个包，以及配套雷达、航点和语音包的系统依赖。
# sllidar_ros2、wp_map_tools、wp_speech 源码和语音模型需另行准备。
packages=(
  # 构建工具和 Python 基础环境。
  build-essential cmake git pkg-config python3-colcon-common-extensions
  python3-dev python3-setuptools python3-pip python3-pyqt5
  python3-numpy python3-opencv python3-yaml

  # ROS 基础、标准消息和自定义消息生成工具。
  ros-humble-ament-cmake ros-humble-ament-index-python
  ros-humble-launch ros-humble-launch-ros
  ros-humble-rclcpp ros-humble-rclpy ros-humble-rcl-interfaces
  ros-humble-std-msgs ros-humble-std-srvs ros-humble-geometry-msgs
  ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-visualization-msgs
  ros-humble-control-msgs ros-humble-trajectory-msgs
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-msgs
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime

  # 模型、关节显示、手柄及 RViz。
  ros-humble-xacro ros-humble-urdf liburdfdom-tools
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher
  ros-humble-joint-state-publisher-gui ros-humble-joy ros-humble-rviz2
  ros-humble-rviz-common ros-humble-rviz-default-plugins ros-humble-pluginlib

  # Femto Bolt 相机、图像传输及驱动所需的开发库。
  ros-humble-orbbec-camera ros-humble-orbbec-description
  ros-humble-image-transport ros-humble-image-transport-plugins
  ros-humble-compressed-image-transport ros-humble-image-publisher
  ros-humble-camera-info-manager ros-humble-diagnostic-updater
  ros-humble-diagnostic-msgs ros-humble-statistics-msgs ros-humble-backward-ros
  libgflags-dev nlohmann-json3-dev libgoogle-glog-dev
  libdw-dev libssl-dev mesa-utils libgl1

  # 图像和点云实验。
  ros-humble-cv-bridge ros-humble-pcl-ros ros-humble-pcl-conversions
  libopencv-dev libpcl-dev

  # 建图、导航及 wp_map_tools 的编译和运行依赖。
  ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-nav2-map-server
  ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs
  ros-humble-tinyxml2-vendor libtinyxml2-dev libyaml-cpp-dev
  qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5

  # MoveIt 规划节点、运动学、可视化和轨迹控制插件。
  ros-humble-geometric-shapes ros-humble-moveit-configs-utils
  ros-humble-moveit-kinematics ros-humble-moveit-planners-ompl
  ros-humble-moveit-ros-move-group ros-humble-moveit-ros-visualization
  ros-humble-moveit-simple-controller-manager

  # wp_speech 的系统音频工具；Python 语音库和模型按该包说明安装。
  alsa-utils pulseaudio-utils

  # 自动测试及配套源码包的 ament lint 依赖。
  python3-pytest ros-humble-ament-cmake-pytest
  ros-humble-ament-lint-auto ros-humble-ament-lint-common
)

sudo apt-get update
sudo apt-get install -y "${packages[@]}"
# Keep previously installed MoveIt libraries on the same release as new plugins.
mapfile -t moveit_packages < <(dpkg-query -W -f='${binary:Package}\n' 'ros-humble-moveit*' 2>/dev/null)
if ((${#moveit_packages[@]})); then
  sudo apt-get install -y --only-upgrade "${moveit_packages[@]}"
fi
printf 'WPR2 build, runtime and test dependencies installed. See README.md for external source packages, face recognition and speech setup.\n'
