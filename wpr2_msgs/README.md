# wpr2_msgs

位于 wpr2_ros2 内的 WPR2 ROS2 自定义接口包，不依赖 WPB 或仿真消息包。

Object.msg 包含 Header 和 name/x/y/z/probability 数组。位置以 Header 指定坐标系表示，单位米。检测节点发布 base_footprint 中包围框 xMax、y 中点、zMin。

```bash
ros2 interface show wpr2_msgs/msg/Object
```
