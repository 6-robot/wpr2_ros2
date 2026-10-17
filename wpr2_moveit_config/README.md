# WPR2 MoveIt 2 configuration

This ROS 2 Humble package preserves the ROS 1 WPR2 SRDF groups and named poses, KDL
kinematics, joint limits, OMPL planner settings, and four FollowJointTrajectory
controller names. The robot model comes from `wpr2_description`.

Build from the workspace root after installing the package dependencies:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src/wpr2_ros2 --ignore-src -r -y
colcon build --packages-up-to wpr2_moveit_config --symlink-install
source install/setup.bash
ros2 launch wpr2_moveit_config demo.launch.py
```

The dependency installer also upgrades existing MoveIt packages together. A
newer OMPL plugin cannot load against older MoveIt core libraries; the launch
file reports the missing shared library if versions still differ.

`demo.launch.py` starts MoveIt, RViz, TF, robot state publishing, and a simulated
joint/action interface. It does not open the robot serial port. Use
`rviz:=false` for a headless check. Add `pose_gui:=true` to either launch to
open the TCP pose and joint-value viewer. RViz starts with the `Left_Arm_Lift`
planning group selected; select `Right_Arm_Lift` in the MotionPlanning panel to
drag the right arm. The visible robot follows `/joint_states` through TF, with
the simulated chest and camera lift initialized to 0.7 m and 0.15 m.

```bash
ros2 launch wpr2_moveit_config real_robot.launch.py serial_port:=/dev/ftdi
ros2 launch wpr2_moveit_config real_robot.launch.py allow_execution:=true
```

The real launch starts `wpr2_bringup`. Motion execution and its trajectory bridge
are disabled by default; enable them after checking joint feedback and direction on
the actual robot. The bridge publishes arm and lift `JointState` commands to the
existing `wpr2/*` driver topics and reports success only after joint feedback
reaches the final point. Cancellation commands the measured position to hold;
this is not a hardware emergency stop. The real launch expects one robot with
un-namespaced `wpr2/*` and `joint_states` topics.

The ROS 1 Gazebo, warehouse, CHOMP/STOMP, and joystick planning launches were not
carried over: they depend on ROS 1 APIs or plugins not used by
this Humble setup. Pilz Cartesian limits are retained as data but the default
pipeline is OMPL. The original SRDF collision matrix is unchanged. Validate
planning and execution on the real robot before using it near people or objects.
The source model lacks collision geometry on several links, and MoveIt warns that
the source SRDF end-effector declarations have no identifiable parent group;
collision checking cannot cover geometry that the URDF does not define.
