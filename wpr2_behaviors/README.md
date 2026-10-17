# wpr2_behaviors（ROS 2 Humble）

WPR2 物体检测与右臂抓取。包名按 `wpb_home_behaviors` 的拼写命名为 `wpr2_behaviors`。自定义消息只依赖同目录的 `wpr2_msgs`。

## 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select wpr2_msgs wpr2_behaviors --parallel-workers 2
source install/setup.bash
```

需要 `rclcpp`、ROS2 标准消息、`tf2_ros`、`pcl_ros`、`pcl_conversions` 和 ROSIDL 工具。运行实物时使用现有 `wpr2_bringup`；Femto Bolt 还需要 `orbbec_camera`。本包所有 launch 均只启动行为节点，不打开串口或相机。

## 直接传入目标坐标

```bash
ros2 launch wpr2_bringup base.launch.py
# 第二个终端
ros2 launch wpr2_behaviors grab.launch.py
# 第三个终端；替换成实测物体坐标，发送后机器人会运动
ros2 topic pub --once /wpr2/grab_bottle geometry_msgs/msg/Pose \
  '{position: {x: 0.9, y: 0.08, z: 0.9}}'
```

坐标是发送时刻 `base_footprint` 中的位置，单位米，orientation 不参与动作。也支持 `/wpr2/grab_pose`（`geometry_msgs/PoseStamped`）：frame_id 必须为 `base_footprint`，默认数据有效期 1 秒；零时间戳表示以收到时刻为准。旧 ROS1 `grab_bottle` 只用 Pose 触发任务，本节点实际使用传入的 position。

流程：等待驱动 → 观测姿态并张爪 → 调整升降 → 底盘横纵向对准 → 右臂进入预设抓取姿态 → 底盘前进 → 合爪 → 抬起 → 升降复位 → 后退 → 折臂持物并等待到位 → 返回结果。已经对准时也会调整升降。

单点输入模式通过相对里程计更新移动后的目标坐标，假定物体静止，不会用相机重新确认；里程计误差会影响对准精度。

## 检测后自动抓取

```bash
ros2 launch wpr2_bringup base.launch.py enable_camera:=true
# 第二个终端；rviz:=true 可观察点云和检测标记
ros2 launch wpr2_behaviors behaviors.launch.py
# 第三个终端
ros2 topic pub --once /wpr2/behavior std_msgs/msg/String '{data: "start grab"}'
```

观测姿态到位后请求 `start objects`，获取有效目标后发送 `stop objects`；移动对准后停车并重新检测确认，直到误差落入容差。`auto_start:=true` 可在启动抓取节点后自动开始一次任务。`start_objects:=false` 可不启动检测节点。

独立检测：`ros2 launch wpr2_behaviors objects_3d.launch.py auto_start:=true`。相机驱动与相机到 `base_footprint` 的 TF 需另行提供。

## 接口与控制约定

| 节点 | 输入 | 输出 |
|---|---|---|
| `wpr2_objects_3d` | `/camera/depth_registered/points`、`/wpr2/behavior` | `/wpr2/objects_3d`（`wpr2_msgs/Object`）、`/objects_marker` |
| `wpr2_grab` | `/wpr2/grab_pose`、`/wpr2/grab_bottle`、`/wpr2/objects_3d`、`/wpr2/behavior`、`/odom_delta`、`/joint_states` | `/cmd_vel`、`/odom_delta_reset`、四路上肢控制话题、`/wpr2/grab_result`、`/wpr2/grab_status` |

- 四路上肢指令是 `/wpr2/right_arm`、`/wpr2/left_arm`、`/wpr2/right_gripper`、`/wpr2/chest_height`。角度 rad，升降和手爪开度 m，velocity 字段沿用原控制器速度数值。
- 检测指令为 `start objects`、`stop objects`；抓取指令为 `start grab`、`stop grab`、`grab stop`。抓取节点兼容旧 `/wpr2/behaviors` 的开始/停止指令。
- 每轮终止只发布一次 `grab done` 或 `grab failed`，失败不会再报成功；状态话题发布当前步骤名称。忙时忽略新目标或重复开始。
- 停止时向底盘发零速度，有新鲜关节反馈时发送测量位置保持指令。它不能保证硬件瞬时停下，不等于硬件急停。任务等待不阻塞回调。
- 每段移动前发送 `odom_delta reset`，等待至少 reset_wait 后连续两条近零反馈；按位移闭环移动，速度受限并补偿航向。反馈缺失、驱动订阅断开、清零或动作超时会失败。
- 手臂和升降按 `/joint_states` 实际值判断到位，包括最后折臂；不使用没有任务编号的通用 done 消息。手爪开度转成反馈手指角度后比较。
- 合爪时硬物可能阻止手爪到达目标开度，沿用 ROS1 合爪等待超时后继续抬起的策略；`grab done` 表示流程完成，不表示已经由力传感器确认抓牢。
- 空闲、完成及失败后释放底盘控制，只发送短暂零速度。任务期间需由本节点独占底盘和上肢控制。

## 算法和标定

`objects_3d.cpp` 完整复制当前 WPB 检测算法，只替换机器人包名、消息类型、话题前缀、节点名和相机话题。保留搜索空间、5 mm 体素、水平面分割、聚类、最近物体选择、稳定帧判定、球形局部锁定跟踪和标记显示。锁定后才发布目标；丢失目标时重新搜索并清除标记，不重发旧坐标。

`wpr2_msgs/Object` 保留 Header 和 name/x/y/z/probability 数组，抓取参考点为 **包围框 xMax、y 中点、zMin**，与 WPB 一致，不是质心。默认 launch 的检测参数也与 WPB 一致。

动作使用 ROS1 `wpr2_grab_bottle.cpp` 的右臂预设姿态，没有逆运动学或 MoveIt 规划，适用于标定后的抓取位置和物体。参数位于 `config/behaviors.yaml`，可通过 `config:=/绝对路径/behaviors.yaml` 替换。

| 参数 | 默认值 | 含义 |
|---|---|---|
| align_x / align_y / grab_y_offset | 0.7 / 0 / 0 | 目标对准到 (align_x, align_y − grab_y_offset) |
| lift_reference_height / lift_reference_z / grab_lift_offset | 0.7 / 1.1 / 0 | 升降目标 = 0.7 − (1.1 − 目标z) + 偏移 |
| min_lift / max_lift | 0.4 / 0.7 | 超范围目标被拒绝；默认可用目标 z 为 0.8～1.1 m |
| observe_lift / restore_lift | 0.5 / 0.7 | 观测高度、后退前恢复高度 |
| grab_forward_range | 0.3 m | 对准后前进、抓取后后退的目标距离 |
| open_width / grab_gripper_value | 0.15 / 0.06 m | 张开、夹取开度 |
| align_speed / forward_speed | 0.05 / 0.1 m/s | 对准和前后移动各轴速度上限，move_gain=0.5 |
| object_tolerance_x/y | 0.03 / 0.03 m | 目标对准容差 |
| move_tolerance_x/y | 0.02 / 0.01 m | 里程计到位容差 |
| reset_wait / stop_wait | 0.5 / 0.5 s | 清零等待、到位停车等待 |
| joint_timeout / joint_feedback_timeout | 45 / 1 s | 关节到位超时、反馈有效时间 |
| motion_timeout / pose_timeout | 30 / 1 s | 单段位移或清零超时、位移反馈有效时间 |
| grip_min_wait / grip_timeout | 0.5 / 5 s | 合爪最短等待、硬物接触等待上限 |
| ready_timeout / detection_timeout / total_timeout | 10 / 15 / 180 s | 等待驱动、单轮检测、整次任务超时 |

右臂观测、准备、上抬和持有姿态，以及左臂观测姿态，均保留 ROS1 数值并参数化。升降目标超出可标定范围时拒绝目标，避免把夹紧高度当作已经对准。

## 测试

```bash
cd ~/ros2_ws
colcon test --packages-select wpr2_behaviors --event-handlers console_direct+
colcon test-result --test-result-base build/wpr2_behaviors --verbose
```

自动测试使用独立 DDS 域、合成点云、模拟关节与里程计，不连接机器人。实机相机 TF、抓取姿态、手爪开度和里程计误差仍需单独标定验证。
