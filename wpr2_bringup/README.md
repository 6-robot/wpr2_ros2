# wpr2_bringup（ROS 2 Humble）

WPR2 实物驱动与启动包，保留包名 `wpr2_bringup` 和原有控制话题。适用 Ubuntu 22.04、ROS 2 Humble，与同级 `wpr2_description` 配套。

## 安装和编译

```bash
source /opt/ros/humble/setup.bash
bash ~/ros2_ws/src/wpr2_ros2/wpr2_bringup/scripts/install_deps.sh
cd ~/ros2_ws
colcon build --symlink-install --packages-select wpr2_description wpr2_bringup
source install/local_setup.bash
```

需要实物设备时，再运行 `scripts/create_udev_rules.sh` 并重新插拔底盘串口、雷达及相机。设备规则安装会请求 sudo 权限；规则文件位于 `scripts/`。删除规则可用 `scripts/delete_udev_rules.sh`。请勿同时运行 `wpr2_core` 和 `wpr2_test_motors`，两者会争用同一串口并驱动电机。

## 启动入口

各 launch 文件独立列出启动项，先定义带注释的 `*_cmd`，再用
`ld.add_action(...)` 逐项添加，不再通过 `base.launch.py` 间接组合。
相机仍调用 Orbbec 官方启动文件，`launch/include/` 中的传感器入口也可单独复用。

```bash
ros2 launch wpr2_bringup base.launch.py                    # 底盘、关节状态、URDF/TF
ros2 launch wpr2_bringup base_lidar.launch.py              # 加 C1 雷达和滤波
ros2 launch wpr2_bringup base_lidar_camera.launch.py       # 再加 Femto Bolt
ros2 launch wpr2_bringup js_ctrl.launch.py                 # 手柄控制
ros2 launch wpr2_bringup vel_test.launch.py                # 速度控制面板
ros2 launch wpr2_bringup lidar_test.launch.py              # 雷达和 RViz2，不开底盘
ros2 launch wpr2_bringup femto_test.launch.py              # 相机和 RViz2，不开底盘
ros2 launch wpr2_bringup arms_test.launch.py               # real / plan 双机器人视图
ros2 launch wpr2_bringup motor_test.launch.py              # 电机逐个测试，实机会转动
```

`base.launch.py` 的主要参数为 `serial_port`（默认 `/dev/ftdi`）、`config`、`start_core`、`rviz`、`enable_lidar`、`enable_camera`、`joystick`、`panel`、`joy_autorepeat_rate`（默认 20 Hz）和 `joy_timeout`（默认 1 秒）。如仅检查模型：

```bash
ros2 launch wpr2_bringup base.launch.py start_core:=false use_joint_state_publisher:=true
```

单机器人使用默认空命名空间即可。已知限制：非空 `namespace` 尚未同步到相机、雷达的话题和 TF；带传感器启动时请保持默认值。

`calibrate_velocity.launch.py` 启动底盘；在另一终端运行 `ros2 run wpr2_bringup wpr2_calibrate_velocity`，输入 W/S/A/D/Q/E 和回车，X 退出。这样交互程序可以正常读取终端输入。

## 接口及参数

`wpr2_core` 订阅 `/cmd_vel`、`/wpr2/left_arm`、`/wpr2/right_arm`、`/wpr2/left_gripper`、`/wpr2/right_gripper`、`/wpr2/chest_height` 和 `/odom_delta_reset`；发布 `/joint_states`、`/odom`、`/odom_delta`、`/wpr2/joints_result`，可选 `/imu/data_raw`，并广播 `odom → base_footprint`。四轮参数使用 `config/wpr2.yaml` 的 `MODE_MECANUM` 或 `MODE_OMNI`。需要在真实串口收到上肢反馈后才接受关节指令；`joint_state_publisher` 与底盘节点不可同时发布同一组 `/joint_states`。

`base.launch.py imu_pub:=true` 可临时开启 IMU 发布；不指定 `imu_pub` 时沿用所选 YAML 的设置。

未收到新的 `/cmd_vel` 时，默认 1 秒停车；可在 `config/wpr2.yaml` 中调整 `cmd_vel_timeout`。在手柄入口中，`joy_node` 即使摇杆不变也以 20 Hz 重复发布 `/joy`；`wpr2_js_vel` 在收到手柄消息后以约 30 Hz 发布 `/cmd_vel`。若 `/joy` 中断超过 1 秒，`wpr2_js_vel` 会发布零速度；若该节点也中断，底盘的 1 秒超时会停车。可用 `ros2 topic hz /joy` 和 `ros2 topic hz /cmd_vel` 检查消息频率。

相机初始关节值沿用原 `zeros` 配置。`wpr2_lidar_filter` 订阅 `/scan_raw`、发布 `/scan`（启动文件设置 `pub_topic`），支持任意长度、无强度数据的扫描。彩色点云使用当前 Orbbec 驱动原生的 `/camera/depth_registered/points` 话题，图像为 `/camera/color/image_raw`。雷达滤波输出采用 Best Effort / Volatile QoS，配套 RViz 使用兼容设置。实物 IMU 的角速度和加速度沿用原始串口值，单位与标定需在硬件上核实。

## 验证与移植范围

```bash
cd ~/ros2_ws
colcon test --packages-select wpr2_bringup --event-handlers console_direct+
colcon test-result --test-result-base build/wpr2_bringup --verbose
```

自动测试用虚拟串口和独立 DDS 域检查速度、超时停车、关节编码与到位反馈、里程计、雷达滤波、手柄及双臂计划映射。实物上的电机方向、里程计系数、IMU 单位、雷达和相机仍需接设备验证。原包并没有实现 FollowJointTrajectory / GripperCommand action server；ROS 2 移植也没有新增此接口。许可证见 `LICENSE`。
