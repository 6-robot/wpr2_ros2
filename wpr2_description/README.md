# wpr2_description（ROS 2 Humble）

WPR2 双臂机器人的模型描述包，由 ROS 1 同名包移植，包名保持为
`wpr2_description`。适用于 Ubuntu 22.04 / ROS 2 Humble。

## 构建与启动

在工作空间根目录执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/wpr2_ros2/wpr2_description --ignore-src -r -y
colcon build --symlink-install --packages-select wpr2_description
source install/local_setup.bash
ros2 launch wpr2_description urdf.launch.py
```

默认打开 RViz2 和关节滑块窗口，显示麦轮底盘、升降躯干、相机、双臂及双夹爪。
初始姿态来自原 `wpr2_bringup/config/wpr2.yaml`，现位于本包
`config/joint_states.yaml`，不再依赖 `wpr2_bringup`。

无图形界面运行及兼容入口：

```bash
ros2 launch wpr2_description urdf.launch.py gui:=false rviz:=false
ros2 launch wpr2_description gripper.launch.py
```

启动参数：`gui` 控制关节滑块；`rviz` 控制 RViz2；`model` 指定 Xacro/URDF
绝对路径；`rvizconfig` 指定 RViz2 配置；`use_sim_time` 默认 `false`，设为
`true` 时需要外部 `/clock`。`gui:=false` 仍会发布初始关节状态；接入真实驱动时，
应单独启动 `robot_state_publisher`，避免与驱动重复发布 `/joint_states`。

## 目录与移植范围

- `urdf/`：有效的整机 Xacro、底盘、躯干和左右臂组件。
- `meshes/`：原始 DAE 网格，保持几何、关节命名、安装位姿和夹爪 mimic 关系。
- `launch/`、`config/`、`rviz/`：ROS 2 Python 启动文件、关节初始值和 RViz2 配置。
- `test/`：URDF 解析、网格引用、夹爪联动与初始关节限位检查。

构建系统由 catkin 改为 ament_cmake，资源安装到包共享目录。
移除了 ROS 1 的 `gazebo_ros_control`、激光插件及旧 PositionJointInterface
transmission 配置。本包提供模型显示和 TF，不包含 Gazebo 动力学控制、
雷达仿真、ros2_control 驱动或 MoveIt 配置。

`wpr2_gripper.urdf.xacro` 和 `gripper.launch.py` 复用包含双夹爪的完整模型，
不提供独立夹爪变体。

## 验证

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select wpr2_description --event-handlers console_direct+
colcon test-result --test-result-base build/wpr2_description --verbose
```

保留原仓库 BSD 3-Clause 许可证，见 `LICENSE`。
