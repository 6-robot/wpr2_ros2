# wpr2_tutorials（ROS 2 Humble）

WPR2 实验程序，参考现有 ROS2 启智实验源码和实验功能；实验 16 的 `ex16_manipulator.cpp` 由 ROS1 WPR2 的 `wpr2_joints_demo.cpp` 移植而来。本包不包含新的实验指导书。

## 构建与依赖

```bash
source /opt/ros/humble/setup.bash
bash ~/ros2_ws/src/wpr2_ros2/wpr2_bringup/scripts/install_deps.sh
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run wpr2_tutorials check_runtime.py
```

编译依赖包括 `rclcpp`、`tf2_ros`、`cv_bridge`、OpenCV、`pcl_ros`、`pcl_conversions`、`visualization_msgs` 和同目录的 `wpr2_msgs`。系统依赖由 `install_deps.sh` 安装；外部雷达、航点和语音源码包，以及语音 Python 环境和模型，按各包说明另行准备。

| 实验 | 额外运行依赖 |
|---|---|
| 2 | `joy`、USB 手柄 |
| 4、5、8—10、17、20 | [Slamtec sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)，默认 C1，460800 波特率 |
| 8 | `slam_toolbox`、`nav2_map_server`（保存地图） |
| 9、10、20 | `nav2_bringup`；10、20 还需要 ROS2 `wp_map_tools` |
| 11—15、17、20 | [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) 中的 `orbbec_camera`，Femto Bolt |
| 17、20 | 同目录 `wpr2_behaviors`、`wpr2_msgs` |
| 13 | Python `face_recognition`、`cv2`、`numpy` |
| 18、19 | ROS2 `wp_speech`，对应语音模型、扬声器或麦克风 |

`check_runtime.py --group base|lidar|camera|face|navigation|speech` 可分组检查，不会打开设备。航点和语音包复用当前环境已安装的 ROS2 包；换机器时需单独提供并 source 其安装环境。缺少可选驱动不影响 C++ 示例编译，但对应硬件入口无法运行。

## 实验入口

每个终端先执行 `source ~/ros2_ws/install/setup.bash`。表中的 launch 均属于 `wpr2_tutorials`，使用 `ros2 launch wpr2_tutorials 文件名`；程序使用 `ros2 run wpr2_tutorials 程序名`。先启动配套节点，再在另一终端运行程序。

| 编号 | 内容 | 配套 launch | 程序或第二个 launch |
|---|---|---|---|
| 01 | 初识 ROS2 | `ex01_bringup.launch.py` | `ex01_my_node` |
| 02 | 手柄控制 | `ex02_bringup.launch.py` | `ex02_teleop.launch.py`（launch） |
| 03 | 速度控制 | `ex03_bringup.launch.py` | `ex03_velocity` |
| 04 | 雷达读取 | `ex04_bringup.launch.py` | `ex04_lidar` |
| 05 | 雷达避障 | `ex05_bringup.launch.py` | `ex05_lidar_behavior` |
| 06 | IMU 姿态 | `ex06_bringup.launch.py` | `ex06_imu_data` |
| 07 | 航向锁定 | `ex07_bringup.launch.py` | `ex07_imu_behavior` |
| 08 | 建图 | `ex08_bringup.launch.py` | `ex08_slam.launch.py`（launch），另开手柄入口操控 |
| 09 | 导航 | `ex09_bringup.launch.py` | `ex09_navigation.launch.py`（launch，固定读取包内地图） |
| 10 | 航点导航 | `ex10_bringup.launch.py` | `ex10_map_tools.launch.py`（launch），再运行 `ex10_waypoint` |
| 11 | 相机与安装参数调节 | `ex11_bringup.launch.py` | GUI 调节 `femto_height`、`femto_pitch` |
| 12 | 彩色图像、HSV | `ex12_bringup.launch.py` | `ex12_image` 或 `ex12_hsv` |
| 13 | 人脸检测 | `ex13_bringup.launch.py` | `ex13_face` |
| 14 | 点云读取 | `ex14_bringup.launch.py` | `ex14_pointcloud` |
| 15 | 物体检测 | `ex15_bringup.launch.py` | `ex15_objects` |
| 16 | 双臂、手爪与升降 | `ex16_bringup.launch.py` | `ex16_manipulator` |
| 17 | 物品抓取 | `ex17_bringup.launch.py` | `ex17_grab`，运行后自动开始一轮检测抓取 |
| 18 | 语音输出 | `ex18_bringup.launch.py` | `ex18_tts` |
| 19 | 语音识别 | `ex19_bringup.launch.py` | `ex19_recognition` |
| 20 | 导航取物与递送 | `ex20_bringup.launch.py map:=/绝对路径/map.yaml` | `ex20_fetch`，定位完成后启动 |

实验 17 启动 `wpr2_core`、机器人 TF、Femto Bolt、C1 雷达及滤波、自动物体检测和 RViz；抓取示例单独启动。点云使用 `/camera/depth_registered/points`，检测结果为 `/wpr2/objects_3d`（`wpr2_msgs/Object`）。实验 20 进一步加入 Nav2、航点服务和 `wpr2_grab`，由单独运行的 `ex20_fetch` 调度任务。

各实验 launch 直接列出节点、固定配置和启动顺序，示例代码不含文字注释。串口（底盘 `/dev/ftdi`、雷达 `/dev/rplidar`）、配置文件、传感器及 RViz 设置写在对应文件中；需要调整时修改该文件。原通用启动开关已移除。实验 1、11 默认启动关节滑块；实验 11 不启动底盘；实验 18、19 只启动语音依赖。单机器人保持空命名空间，使用真实时钟 `use_sim_time=False`。

实验 8—10 的 bringup 只启动硬件，第二个 launch 启动算法和 RViz，避免重复打开窗口。`slam.launch.py`、`navigation.launch.py` 分别是 `ex08_slam.launch.py`、`ex09_navigation.launch.py` 的兼容入口，均需先单独启动硬件。实验9和10固定读取 `wpr2_tutorials/maps/map.yaml`，由 YAML 中的 `image: map.pgm` 加载同目录图像，不设置启动参数或额外文件检查；保存或更新地图后重新编译，将地图安装到包共享目录。实验10还固定读取 `~/waypoint.xml`，不设置启动参数或额外文件检查。实验20保留 `map`、`waypoints` 参数及相关文件检查。

## 示例代码约定

示例参考 WPB 同类实验，使用全局变量、独立回调、main 和步骤编号展示流程。速度、距离、阈值、姿态与等待时间集中在各源码顶部，含义和单位见开发指南正文；修改后重新编译 `wpr2_tutorials`。不再使用教程专用 ROS 参数或 `config/ex17_grab.yaml`，话题仍可用 ROS 重映射调整。

- 底盘使用 `/cmd_vel`，驱动的 1 秒指令超时及串口换算保持现状。实验 3 按 30 Hz 连续发送速度；实验 5、7 每收到一帧传感器数据就发送一次，不重发停止更新的测量。手柄由驱动配套节点保持 20 Hz 重发。
- 实验 4、5 按扫描角度寻找车头方向。WPR2 的雷达相对底盘旋转 180°，源码 `forward_angle=M_PI`；更换安装方向需修改该常量。避障距离 1.5 m、直行速度 0.1 m/s、左转速度 0.3 rad/s、转向持续20帧（C1默认10 Hz，约2 s）。该示例只检查正前方，不是完整导航器。
- 实验 6、7 的 launch 开启 `imu_pub`，订阅 `/imu/data_raw`。实验 7 目标航向 90°，前进速度 0.1 m/s，角速度直接由角差乘0.01得到，角差跨 ±180° 环绕，不另加限速。
- 图像使用 `/camera/color/image_raw`，点云使用 `/camera/depth_registered/points`；传感器订阅和配套 RViz 使用 Best Effort / Volatile QoS。实验 12、13 直接打开图像窗口；实验 14 逐点打印当前帧坐标。
- 实验 11 不启动底盘，仅用关节滑块显示标定。先设置实际 `chest_height`，再调节 `femto_height`、`femto_pitch`；它们是相机安装偏移，单位分别为 m、rad。把结果写入所用 `wpr2.yaml` 的 `zeros`，重启复核。标定窗口不能与驱动同时发布 `/joint_states`。
- 实验 15 展示坐标变换、裁剪、平面分割、聚类和质心计算，发布 `/wpr2/objects`（`geometry_msgs/PoseArray`）及 `/objects_marker`。计算坐标系为 `base_footprint`，物体最小桌面高度差为0.02 m；姿态为单位四元数，不代表估计了物体朝向。
- Nav2 使用全向底盘配置。footprint 覆盖底盘并留5 cm边距，运行前按收拢后的双臂和物品实际外廓调整。地图统一保存到 `~/ros2_ws/src/wpr2_ros2/wpr2_tutorials/maps/map.yaml` 和同目录的 `map.pgm`，更新后重新编译并加载环境；航点由 `wp_map_tools` 编辑保存。实验10默认请求航点 `1`，更改源码顶部的 `waypoint` 可换目标。

教程层按 WPB 同类实验去掉额外检查：不校验雷达范围或 IMU 姿态，不捕获图像转换异常，不检查人脸框边界，不另设速度或升降限幅，不添加任务失败处理。仍保留 WPB 已有的算法判断，如 HSV 目标像素计数、点云 TF 可用性和平面点数、抓取步骤与本轮检测结果判断。雷达示例按 C1 正常输出的完整一圈扫描计算前方索引。没有检测结果或里程计清零反馈时会等待，不会自动报超时。退出使用标准 ROS 流程，不保证示例发送最后的停车指令；底盘驱动仍有1秒指令超时，已发送的机械臂动作可能继续完成。不要同时运行多个主动控制程序。

### 实验 16

```bash
ros2 launch wpr2_tutorials ex16_bringup.launch.py
```

另一终端：启动后会交替驱动双臂、双手爪和升降

```bash
ros2 run wpr2_tutorials ex16_manipulator
```

保留 ROS1 两组双臂姿态、升降高度0.65/0.7 m、手爪开度0.05/0.1 m及5秒切换周期。启动后等待1秒，随后直接交替发送五组控制话题，不订阅关节反馈；运行前应先确认配套驱动已正常工作。

关节指令用 rad，升降及手爪开度用 m；`velocity` 为控制器速度数值，升降15000，其他2000，不是标准 rad/s 或 m/s。

### 实验 17

```bash
ros2 launch wpr2_tutorials ex17_bringup.launch.py
```

另一终端：默认自动开始一次抓取，机器人会运动。

```bash
ros2 run wpr2_tutorials ex17_grab
```

程序直接发布 WPR2 控制话题，流程为：观测姿态 → 检测 → 调整升降 → 平移对准 → 停车重新检测 → 准备右臂 → 前进 → 合爪 → 抬物 → 恢复升降 → 后退 → 折臂持物。

源码顶部保留原 WPR2 姿态、升降公式、0.7 m对准距离、0.3 m前后距离、0.06 m合爪开度和控制器速度。升降按标定公式直接计算，不额外限幅；标定目标高度为0.8～1.1 m，应在此范围内摆放物体。

机械臂步骤改为固定等待：观测和准备右臂18秒，调整及恢复升降18秒，合爪、抬物和折臂各5秒。各值均在源码顶部，需按实物调整；等待结束不代表确认到位。底盘仍按 `/odom_delta` 判断移动距离，清零与停车各等待0.5秒，移动后重新检测确认对准。

启动后只执行一轮，完成时发布一次 `/wpr2/grab_result` 的 `grab done`，并停止发送速度。已移除目标坐标、开始/停止指令和状态话题接口；重新运行程序可再做一次。请勿与 `wpr2_behaviors/wpr2_grab` 同时运行。

### 实验 20

先准备当前场地地图及包含 `kitchen`、`guest` 的航点 XML。`kitchen` 应位于可检测和对准桌面物体的位置，目标高度和抓取姿态需沿用实验 17 的实物标定。

```bash
ros2 launch wpr2_tutorials ex20_bringup.launch.py map:=/绝对路径/map.yaml waypoints:=/绝对路径/waypoint.xml
```

在 RViz 中用 2D Pose Estimate 完成定位，确认机器人位置和地图一致。

另一终端：程序运行后自动开始任务，机器人会运动。

```bash
ros2 run wpr2_tutorials ex20_fetch
```

流程为 `kitchen` 导航 → WPR2 对准抓取、抬起、后退、折臂持物 → `guest` 导航 → 停车持物。到达终点不自动张爪。配套 launch 中检测和抓取节点均默认空闲，任务程序只通过 `/waterplus/navi_waypoint`、`/waterplus/navi_result`、`/wpr2/behavior` 和 `/wpr2/grab_result` 调度，不重复实现抓取动作。

源码顶部设置航点名称 `kitchen_waypoint="kitchen"`、`guest_waypoint="guest"`。名称必须与当前场地航点 XML 一致；修改后重新编译。

与 WPB 一样，仅收到当前阶段的 `navi done` / `grab done` 才推进步骤；其他回执不改变步骤。正常结束切换到 `STEP_DONE` 并打印 `fetch done` 日志，不另设结果发布器或失败分支。任务调度不发布 `/cmd_vel`，没有交接等待、连接检测、阶段超时或自动取消。Ctrl+C 仅退出调度程序，不会取消已交给导航或抓取节点的任务。任务期间不要并行发送其他导航、抓取或手柄指令。`grab done` 表示行为节点动作流程完成，不保证已经夹住物品。

## 测试

```bash
cd ~/ros2_ws
colcon test --packages-select wpr2_tutorials --event-handlers console_direct+
colcon test-result --test-result-base build/wpr2_tutorials --verbose
```

自动测试使用独立 DDS 域、重映射话题和模拟输入，不连接机器人。覆盖雷达方向与持续转向、IMU环绕、双臂两组消息、图像及点云处理、航点和语音接口，以及各 launch 的节点、配置和传感器QoS。图像窗口使用离屏环境；实验17从实际 launch 中只启动检测节点，配合虚拟底盘验证默认固定等待、重新检测及整轮抓取；实验20覆盖调度顺序、实际WPR2检测抓取，以及已安装航点服务配合模拟Nav2的两段导航。实验17按真实默认等待时间运行，完整测试需要数分钟。

实物雷达、Femto Bolt、IMU、机械臂、导航和音频效果需要现场联调；自动测试不代替实机验证。
