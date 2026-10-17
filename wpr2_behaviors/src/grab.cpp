// Nonblocking WPB-style behavior flow using the ROS1 WPR2 bottle-grasp poses.
#include "wpr2_behaviors/runtime.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <wpr2_msgs/msg/object.hpp>
#include <array>
#include <map>
#include <optional>
#include <stdexcept>

namespace
{
using Clock = std::chrono::steady_clock;
using JointState = sensor_msgs::msg::JointState;
using Point = geometry_msgs::msg::Point;
using Pose2D = geometry_msgs::msg::Pose2D;

enum class Step
{
  IDLE, WAIT_READY, OBSERVE, DETECT, ADJUST_LIFT,
  ALIGN_RESET, ALIGN_MOVE, ALIGN_STOP, READY_ARM,
  FORWARD_RESET, FORWARD_MOVE, FORWARD_STOP, GRIP, RAISE_ARM,
  RESTORE_LIFT, BACKWARD_RESET, BACKWARD_MOVE, BACKWARD_STOP, HOLD_ARM,
  DONE, FAILED
};
const std::array<const char *, 21> STEP_NAMES = {
  "idle", "wait_ready", "observe", "detect", "adjust_lift",
  "align_reset", "align_move", "align_stop", "ready_arm",
  "forward_reset", "forward_move", "forward_stop", "grip", "raise_arm",
  "restore_lift", "backward_reset", "backward_move", "backward_stop", "hold_arm",
  "done", "failed"};

// 抓取参数存储；下方构造函数集中声明默认值，并逐项说明用途和单位。
// 启动时可通过同名 ROS 参数或 config/behaviors.yaml 覆盖默认值。
struct Parameters
{
  double align_x, align_y, y_offset, lift_reference_z, lift_reference_height, lift_offset;
  double forward_distance, open_width, grip_width, min_lift, max_lift, observe_lift, restore_lift;
  double object_tol_x, object_tol_y, move_tol_x, move_tol_y;
  double gain, align_speed, forward_speed, angular_gain, angular_limit, heading_tol, heading_limit;
  double reset_wait, reset_tolerance, stop_wait, object_timeout, detection_timeout;
  double ready_timeout, joint_timeout, joint_feedback_timeout, pose_timeout, motion_timeout, total_timeout;
  double joint_tolerance, lift_tolerance, gripper_tolerance, joint_min_wait, grip_min_wait, grip_timeout;
  double max_object_x, max_object_y, max_align_distance;
  double joint_speed, lift_speed;
  int max_align_attempts;
  std::vector<double> observe_right, observe_left, ready_right, raise_right, hold_right;
};
}

class Wpr2Grab : public rclcpp::Node
{
public:
  Wpr2Grab() : Node("wpr2_grab")
  {
    auto positive = [this](const char * name, double value) {
      value = declare_parameter(name, value);
      if (!std::isfinite(value) || value <= 0) { throw std::invalid_argument(std::string(name) + " must be positive"); }
      return value;
    };
    auto finite = [this](const char * name, double value) {
      value = declare_parameter(name, value);
      if (!std::isfinite(value)) { throw std::invalid_argument(std::string(name) + " must be finite"); }
      return value;
    };
    // 1、抓取位置标定：坐标位于 base_footprint，x 向前、y 向左、z 向上，单位米。
    // 对准目标为 (align_x, align_y - grab_y_offset)，之后再前进 grab_forward_range 抓取。
    p_.align_x = positive("align_x", 0.7);  // m，抓取前物体应对准的前向距离。
    p_.align_y = finite("align_y", 0.0);  // m，抓取前物体应对准的横向位置，可正可负。
    p_.y_offset = finite("grab_y_offset", 0.0);  // m，横向补偿；正值使对准目标 y 减小。

    // 升降目标 = lift_reference_height - (lift_reference_z - 物体 z) + grab_lift_offset。
    // 升降值是 chest_height 关节位移，不是手爪或相机的离地高度。
    p_.lift_reference_z = positive("lift_reference_z", 1.1);  // m，标定参考物体的 z 坐标。
    p_.lift_reference_height = positive("lift_reference_height", 0.7);  // m，对应参考物体高度的升降值。
    p_.lift_offset = finite("grab_lift_offset", 0.0);  // m，升降补偿；正值抬高，负值降低。
    p_.forward_distance = positive("grab_forward_range", 0.3);  // m，对准后前进及抓取后后退的目标距离，最大 0.5。
    p_.open_width = positive("open_width", 0.15);  // m，张开右爪时的指间距，最大 0.155。
    p_.grip_width = positive("grab_gripper_value", 0.06);  // m，夹取时的目标指间距，须大于 0 且不超过 open_width。
    p_.min_lift = finite("min_lift", 0.4);  // m，允许的最低升降值，不能小于 0。
    p_.max_lift = positive("max_lift", 0.7);  // m，允许的最高升降值，不能超过硬件上限 0.7。
    p_.observe_lift = finite("observe_lift", 0.5);  // m，进入观测姿态时的升降值，须在 min_lift 与 max_lift 之间。
    p_.restore_lift = finite("restore_lift", 0.7);  // m，抓起物品后、底盘后退前恢复的升降值，须在允许范围内。

    // 2、对准容差与底盘速度：物体坐标容差和里程计位移容差分别设置。
    p_.object_tol_x = positive("object_tolerance_x", 0.03);  // m，物体相对目标位置的 x 误差容差。
    p_.object_tol_y = positive("object_tolerance_y", 0.03);  // m，物体相对目标位置的 y 误差容差。
    p_.move_tol_x = positive("move_tolerance_x", 0.02);  // m，单段移动中相对里程计 x 的到位容差。
    p_.move_tol_y = positive("move_tolerance_y", 0.01);  // m，单段移动中相对里程计 y 的到位容差。
    p_.gain = positive("move_gain", 0.5);  // 1/s，平移比例系数；位置误差乘此系数得到限速前的速度。
    p_.align_speed = positive("align_speed", 0.05);  // m/s，对准移动时每个平移轴的速度上限，最大 0.3。
    p_.forward_speed = positive("forward_speed", 0.1);  // m/s，前进和后退时每个平移轴的速度上限，最大 0.3。
    p_.angular_gain = positive("angular_gain", 0.5);  // 1/s，航向纠偏比例系数，角速度 = -系数 × 航向误差，再限速。
    p_.angular_limit = positive("max_angular_speed", 0.2);  // rad/s，航向纠偏的角速度上限。
    p_.heading_tol = positive("heading_tolerance", 0.03);  // rad，移动完成时允许的航向误差，须小于 max_heading_error。
    p_.heading_limit = positive("max_heading_error", 0.35);  // rad，相对本段起始方向的最大航向偏差，超过则任务失败。

    // 3、清零、停车等待与超时：所有时间单位均为秒。
    p_.reset_wait = positive("reset_wait", 0.5);  // s，清零后至少等待此时间，再累计连续两条近零里程计反馈。
    p_.reset_tolerance = positive("reset_tolerance", 0.001);  // 清零判定容差：x/y 用 m，theta 用 rad，共用此数值。
    p_.stop_wait = positive("stop_wait", 0.5);  // s，底盘到位后保持零速度的稳定等待时间。
    p_.object_timeout = positive("object_timeout", 1.0);  // s，检测结果及非零时间戳单点目标允许的最大数据年龄。
    p_.detection_timeout = positive("detection_timeout", 15.0);  // s，每轮等待有效物体检测结果的最长时间。
    p_.ready_timeout = positive("ready_timeout", 10.0);  // s，等待驱动订阅连接及完整、新鲜关节反馈的最长时间。
    p_.joint_timeout = positive("joint_timeout", 45.0);  // s，每个手臂或升降姿态步骤的到位超时，不用于合爪等待。
    p_.joint_feedback_timeout = positive("joint_feedback_timeout", 1.0);  // s，允许的最大关节反馈接收间隔。
    p_.pose_timeout = positive("pose_timeout", 1.0);  // s，判断相对里程计反馈是否新鲜的时间上限。
    p_.motion_timeout = positive("motion_timeout", 30.0);  // s，每段里程计清零或底盘移动的独立超时。
    p_.total_timeout = positive("total_timeout", 180.0);  // s，一轮抓取从开始指令起算的总超时，包含等待驱动等步骤。

    // 4、关节到位与合爪等待：右爪指令用开度，反馈到位判定用手指关节角度。
    p_.joint_tolerance = positive("joint_tolerance", 0.02);  // rad，左右臂各关节的到位角度容差。
    p_.lift_tolerance = positive("lift_tolerance", 0.005);  // m，升降关节到位容差。
    p_.gripper_tolerance = positive("gripper_tolerance", 0.03);  // rad，right_joint_finger 的到位容差，不是手爪开度误差。
    p_.joint_min_wait = positive("joint_min_wait", 0.1);  // s，发送关节指令后，最早允许判定到位的时间。
    p_.grip_min_wait = positive("grip_min_wait", 0.5);  // s，合爪最短等待；到位且满足等待时间才继续，不能超过 grip_timeout。
    p_.grip_timeout = positive("grip_timeout", 5.0);  // s，合爪等待上限；到时即使未完全闭合，也继续抬起以适应硬物接触。

    // 5、目标范围与控制器速度数值。
    p_.max_object_x = positive("max_object_x", 1.5);  // m，可接受目标的 x 上限；x 还必须大于 grab_forward_range。
    p_.max_object_y = positive("max_object_y", 0.5);  // m，可接受目标的 y 绝对值上限。
    p_.max_align_distance = positive("max_align_distance", 0.8);  // m，单轮对准的平移距离上限，按 hypot(dx, dy) 判断。
    p_.joint_speed = positive("joint_speed", 2000.0);  // 控制器原始速度数值，用于双臂和右手爪，非 rad/s，最大 100000。
    p_.lift_speed = positive("lift_speed", 15000.0);  // 升降控制器原始速度数值，非 m/s，最大 100000。
    p_.max_align_attempts = declare_parameter("max_align_attempts", 5);  // 次，允许的最大对准移动轮数，至少为 1。
    auto pose = [this](const char * name, std::vector<double> value) {
      value = declare_parameter(name, value);
      if (value.size() != 6 || std::any_of(value.begin(), value.end(),
        [](double v) { return !std::isfinite(v) || std::abs(v) > 2 * M_PI; })) {
        throw std::invalid_argument(std::string(name) + " needs six finite joint angles");
      }
      return value;
    };
    // 6、预设手臂姿态：每组 6 个角度，单位 rad，依次对应该侧 joint_1～joint_6。
    // 默认值沿用 ROS1 WPR2；每个角度必须有限且绝对值不超过 2π。
    p_.observe_right = pose("observe_right", {-0.6, 1.57, 0, 1.5, 0, 0});  // 右臂观测姿态，任务开始时使用。
    p_.observe_left = pose("observe_left", {0.6, -1.57, 0, -1.5, 0, 0});  // 左臂观测姿态，进入后保持，不参与夹取。
    // 右臂抓取准备姿态：对准完成后摆到此姿态，再由底盘前进使物体进入手爪。
    p_.ready_right = pose("ready_right", {1.4813, 1.5723, -1.5426, 1.3623, 1.4895, -1.3667});
    // 右臂上抬姿态：合爪结束后抬起物体，再恢复升降高度并后退。
    p_.raise_right = pose("raise_right", {1.6673, 1.5699, -1.5818, 1.3601, 1.6793, -1.3631});
    // 右臂持物姿态：底盘后退完成后折臂，确认此姿态到位才报告抓取完成。
    p_.hold_right = pose("hold_right", {-0.1593, 1.5128, -0.0382, 1.7225, 0.0637, 1.5130});
    if (p_.min_lift < 0 || p_.min_lift > p_.max_lift || p_.max_lift > 0.7 ||
      p_.observe_lift < p_.min_lift || p_.observe_lift > p_.max_lift ||
      p_.restore_lift < p_.min_lift || p_.restore_lift > p_.max_lift ||
      p_.open_width > 0.155 || p_.grip_width > p_.open_width ||
      p_.align_x <= p_.forward_distance || p_.align_x >= p_.max_object_x ||
      std::abs(p_.align_y - p_.y_offset) >= p_.max_object_y ||
      p_.forward_distance > 0.5 || p_.joint_speed > 100000 || p_.lift_speed > 100000 ||
      p_.grip_min_wait > p_.grip_timeout || p_.max_align_attempts < 1 ||
      p_.heading_tol >= p_.heading_limit || p_.align_speed > 0.3 || p_.forward_speed > 0.3) {
      throw std::invalid_argument("Inconsistent grab calibration, hardware limits or timing parameters");
    }
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    ctrl_pub_ = create_publisher<std_msgs::msg::String>("/odom_delta_reset", 10);
    command_pub_ = create_publisher<std_msgs::msg::String>("/wpr2/behavior", 10);
    result_pub_ = create_publisher<std_msgs::msg::String>("/wpr2/grab_result", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/wpr2/grab_status", rclcpp::QoS(1).transient_local());
    const std::array<std::string, 4> topics = {"chest_height", "right_arm", "left_arm", "right_gripper"};
    for (size_t i = 0; i < topics.size(); ++i) {
      joint_pubs_[i] = create_publisher<JointState>("/wpr2/" + topics[i], 10);
    }
    behavior_sub_ = create_subscription<std_msgs::msg::String>("/wpr2/behavior", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { behavior(msg->data); });
    // Preserve the legacy plural topic for start/stop callers.
    legacy_behavior_sub_ = create_subscription<std_msgs::msg::String>("/wpr2/behaviors", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { behavior(msg->data); });
    object_sub_ = create_subscription<wpr2_msgs::msg::Object>("/wpr2/objects_3d", 10,
      [this](wpr2_msgs::msg::Object::ConstSharedPtr msg) { object(*msg); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/wpr2/grab_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (active()) { RCLCPP_WARN(get_logger(), "Grab active; new target ignored"); return; }
        if (msg->header.frame_id != "base_footprint" || !recent_stamp(msg->header.stamp, true)) {
          reject("Target must be fresh and expressed in base_footprint"); return;
        }
        start(msg->pose.position);
      });
    // Unlike ROS1's trigger-only callback, this entry actually uses the supplied coordinates.
    legacy_goal_sub_ = create_subscription<geometry_msgs::msg::Pose>("/wpr2/grab_bottle", 10,
      [this](geometry_msgs::msg::Pose::ConstSharedPtr msg) { start(msg->position); });
    pose_sub_ = create_subscription<Pose2D>("/odom_delta", 10,
      [this](Pose2D::ConstSharedPtr msg) {
        if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->theta)) {
          pose_received_ = false; return;
        }
        delta_ = *msg;
        pose_received_ = true;
        pose_observed_ = Clock::now();
        if (resetting() && age(step_started_) >= p_.reset_wait) {
          const bool zero = std::abs(msg->x) <= p_.reset_tolerance &&
            std::abs(msg->y) <= p_.reset_tolerance && std::abs(msg->theta) <= p_.reset_tolerance;
          reset_samples_ = zero ? reset_samples_ + 1 : 0;
        }
      });
    joint_sub_ = create_subscription<JointState>("/joint_states", rclcpp::SensorDataQoS(),
      [this](JointState::ConstSharedPtr msg) {
        joints_.clear();
        for (size_t i = 0; i < std::min(msg->name.size(), msg->position.size()); ++i) {
          if (std::isfinite(msg->position[i])) { joints_[msg->name[i]] = msg->position[i]; }
        }
        joints_observed_ = Clock::now();
        ++joint_sequence_;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this] { tick(); });
    change(Step::IDLE);
    // auto_start：true 表示启动后自动执行一轮检测抓取；false 表示等待开始指令或目标坐标。
    if (declare_parameter("auto_start", false)) { start(std::nullopt); }
  }

  void shutdown()
  {
    if (active()) { finish(false, "Grab interrupted by shutdown"); tutorial::stop(vel_pub_); }
  }

private:
  static double age(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }
  bool active() const { return step_ != Step::IDLE && step_ != Step::DONE && step_ != Step::FAILED; }
  bool resetting() const {
    return step_ == Step::ALIGN_RESET || step_ == Step::FORWARD_RESET || step_ == Step::BACKWARD_RESET;
  }
  bool moving() const {
    return step_ == Step::ALIGN_MOVE || step_ == Step::FORWARD_MOVE || step_ == Step::BACKWARD_MOVE;
  }
  bool recent_stamp(const builtin_interfaces::msg::Time & stamp, bool allow_zero) const
  {
    if (stamp.sec < 0) { return false; }
    if (stamp.sec == 0 && stamp.nanosec == 0) { return allow_zero; }
    const double elapsed = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return elapsed >= -0.1 && elapsed <= p_.object_timeout;
  }
  void change(Step next)
  {
    step_ = next; step_started_ = Clock::now();
    std_msgs::msg::String status; status.data = STEP_NAMES[static_cast<size_t>(next)];
    status_pub_->publish(status);
    RCLCPP_INFO(get_logger(), "Grab step: %s", status.data.c_str());
  }
  void detection_command(const char * value)
  {
    std_msgs::msg::String msg; msg.data = value; command_pub_->publish(msg);
  }
  void reject(const char * reason)
  {
    RCLCPP_ERROR(get_logger(), "%s", reason);
    std_msgs::msg::String msg; msg.data = "grab failed"; result_pub_->publish(msg);
  }
  double lift_for(const Point & point) const {
    return p_.lift_reference_height - (p_.lift_reference_z - point.z) + p_.lift_offset;
  }
  bool valid_target(const Point & point) const
  {
    const double lift = lift_for(point);
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
      point.x > p_.forward_distance && point.x <= p_.max_object_x &&
      std::abs(point.y) <= p_.max_object_y && lift >= p_.min_lift - 1e-6 && lift <= p_.max_lift + 1e-6;
  }
  void start(std::optional<Point> target)
  {
    if (active()) { RCLCPP_WARN(get_logger(), "Grab active; duplicate start ignored"); return; }
    if (target && !valid_target(*target)) { reject("Target outside calibrated grab range"); return; }
    direct_ = target.has_value();
    if (target) { target_ = *target; }
    goal_started_ = Clock::now(); align_attempts_ = 0; detection_requested_ = false;
    touched_.fill(false); pending_.clear(); stop_cycles_ = 0;
    detection_command("stop objects");
    change(Step::WAIT_READY);
  }
  void behavior(const std::string & command)
  {
    if (command == "start grab") { start(std::nullopt); }
    else if ((command == "stop grab" || command == "grab stop") && active()) {
      finish(false, "Grab stopped by command");
    }
  }
  void object(const wpr2_msgs::msg::Object & msg)
  {
    if (step_ != Step::DETECT || !detection_requested_ || direct_) { return; }
    if (msg.header.frame_id != "base_footprint" || msg.x.empty() ||
      msg.x.size() != msg.y.size() || msg.x.size() != msg.z.size() ||
      !recent_stamp(msg.header.stamp, false) ||
      rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()) < detection_started_) { return; }
    Point point; point.x = msg.x[0]; point.y = msg.y[0]; point.z = msg.z[0];
    if (!valid_target(point)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring object outside calibrated grab range");
      return;
    }
    target_ = point;
    detection_command("stop objects");
    prepare_target();
  }
  bool fresh_joints() const
  {
    if (age(joints_observed_) > p_.joint_feedback_timeout) { return false; }
    if (!joints_.count("chest_height") || !joints_.count("right_joint_finger")) { return false; }
    for (int i = 1; i <= 6; ++i) {
      if (!joints_.count("right_joint_" + std::to_string(i)) ||
        !joints_.count("left_joint_" + std::to_string(i))) { return false; }
    }
    return true;
  }
  bool connected() const
  {
    if (vel_pub_->get_subscription_count() == 0 || ctrl_pub_->get_subscription_count() == 0) { return false; }
    for (const auto & pub : joint_pubs_) { if (pub->get_subscription_count() == 0) { return false; } }
    return true;
  }
  void send_joint(size_t group, const std::vector<double> & values)
  {
    JointState msg;
    if (group == 0) { msg.name = {"chest_height"}; }
    else if (group == 3) { msg.name = {"right_gripper"}; }
    else {
      for (int i = 1; i <= 6; ++i) {
        msg.name.push_back(std::string(group == 1 ? "right_joint_" : "left_joint_") + std::to_string(i));
      }
    }
    msg.header.stamp = now(); msg.position = values;
    // WPR2 velocity fields use controller speed numbers, not SI joint speeds.
    msg.velocity.assign(values.size(), group == 0 ? p_.lift_speed : p_.joint_speed);
    joint_pubs_[group]->publish(msg);
    touched_[group] = true;
    if (group == 3) { pending_["right_joint_finger"] = (0.155 - values[0]) / 0.155 * 1.57; }
    else { for (size_t i = 0; i < msg.name.size(); ++i) { pending_[msg.name[i]] = values[i]; } }
  }
  void issue(Step next, std::optional<std::vector<double>> right = {},
    std::optional<double> lift = {}, std::optional<double> width = {}, bool observe_left = false)
  {
    pending_.clear(); command_sequence_ = joint_sequence_;
    if (right) { send_joint(1, *right); }
    if (lift) { send_joint(0, {std::clamp(*lift, p_.min_lift, p_.max_lift)}); }
    if (width) { send_joint(3, {*width}); }
    if (observe_left) { send_joint(2, p_.observe_left); }
    change(next);
  }
  bool reached() const
  {
    if (joint_sequence_ <= command_sequence_ || age(step_started_) < p_.joint_min_wait) { return false; }
    for (const auto & [name, value] : pending_) {
      const auto it = joints_.find(name);
      const double tolerance = name == "chest_height" ? p_.lift_tolerance :
        name == "right_joint_finger" ? p_.gripper_tolerance : p_.joint_tolerance;
      if (it == joints_.end() || std::abs(it->second - value) > tolerance) { return false; }
    }
    return true;
  }
  void hold_measured()
  {
    if (!fresh_joints()) { return; }
    const auto touched = touched_;
    for (size_t group = 0; group < 4; ++group) {
      if (!touched[group]) { continue; }
      if (group == 0) { send_joint(0, {joints_.at("chest_height")}); }
      else if (group == 3) {
        const double width = 0.155 * (1 - std::clamp(joints_.at("right_joint_finger") / 1.57, 0.0, 1.0));
        send_joint(3, {width});
      } else {
        std::vector<double> values;
        for (int i = 1; i <= 6; ++i) {
          values.push_back(joints_.at(std::string(group == 1 ? "right_joint_" : "left_joint_") + std::to_string(i)));
        }
        send_joint(group, values);
      }
    }
  }
  void finish(bool success, const char * reason)
  {
    if (!success) { hold_measured(); RCLCPP_ERROR(get_logger(), "%s", reason); }
    else { RCLCPP_INFO(get_logger(), "%s", reason); }
    detection_command("stop objects");
    change(success ? Step::DONE : Step::FAILED);
    vel_pub_->publish(geometry_msgs::msg::Twist{}); stop_cycles_ = 2;
    std_msgs::msg::String result; result.data = success ? "grab done" : "grab failed";
    result_pub_->publish(result);
  }
  void begin_detection()
  {
    detection_requested_ = false;
    change(Step::DETECT);
  }
  void prepare_target()
  {
    if (!fresh_joints() || !connected()) { finish(false, "Driver feedback lost before target preparation"); return; }
    const double lift = std::clamp(lift_for(target_), p_.min_lift, p_.max_lift);
    if (std::abs(joints_.at("chest_height") - lift) > p_.lift_tolerance) {
      issue(Step::ADJUST_LIFT, {}, lift);
    } else { plan_alignment(); }
  }
  void plan_alignment()
  {
    const double dx = target_.x - p_.align_x;
    const double dy = target_.y - p_.align_y + p_.y_offset;
    if (std::abs(dx) <= p_.object_tol_x && std::abs(dy) <= p_.object_tol_y) {
      issue(Step::READY_ARM, p_.ready_right);
    } else if (++align_attempts_ > p_.max_align_attempts ||
      std::hypot(dx, dy) > p_.max_align_distance) {
      finish(false, "Alignment exceeds configured range or attempt limit");
    } else {
      move_x_ = std::abs(dx) > p_.object_tol_x ? dx : 0;
      move_y_ = std::abs(dy) > p_.object_tol_y ? dy : 0;
      reset(Step::ALIGN_RESET);
    }
  }
  void reset(Step next)
  {
    pose_received_ = false; reset_samples_ = 0;
    change(next);
    std_msgs::msg::String command; command.data = "odom_delta reset";
    ctrl_pub_->publish(command);
  }
  void motion(geometry_msgs::msg::Twist & velocity)
  {
    if (!pose_received_ || age(pose_observed_) > p_.pose_timeout) {
      finish(false, "Odometry feedback timed out or invalid"); return;
    }
    const double theta = std::remainder(delta_.theta, 2 * M_PI);
    if (std::abs(theta) > p_.heading_limit) { finish(false, "Unexpected chassis heading change"); return; }
    const bool align = step_ == Step::ALIGN_MOVE;
    const double dx = (align ? move_x_ : step_ == Step::FORWARD_MOVE ? p_.forward_distance : -p_.forward_distance) - delta_.x;
    const double dy = (align ? move_y_ : 0.0) - delta_.y;
    if (std::abs(dx) <= p_.move_tol_x && std::abs(dy) <= p_.move_tol_y && std::abs(theta) <= p_.heading_tol) {
      change(align ? Step::ALIGN_STOP : step_ == Step::FORWARD_MOVE ? Step::FORWARD_STOP : Step::BACKWARD_STOP);
      return;
    }
    const double ex = std::abs(dx) <= p_.move_tol_x ? 0.0 : dx;
    const double ey = std::abs(dy) <= p_.move_tol_y ? 0.0 : dy;
    const double speed = align ? p_.align_speed : p_.forward_speed;
    // Delta is expressed in the reset frame; cmd_vel is expressed in the current base frame.
    velocity.linear.x = std::clamp(p_.gain * (ex * std::cos(theta) + ey * std::sin(theta)), -speed, speed);
    velocity.linear.y = std::clamp(p_.gain * (-ex * std::sin(theta) + ey * std::cos(theta)), -speed, speed);
    velocity.angular.z = std::clamp(-p_.angular_gain * theta, -p_.angular_limit, p_.angular_limit);
  }
  void tick()
  {
    if (!active()) {
      if (stop_cycles_ > 0) { vel_pub_->publish(geometry_msgs::msg::Twist{}); --stop_cycles_; }
      return;
    }
    if (age(goal_started_) > p_.total_timeout) { finish(false, "Grab total timeout"); return; }
    if (step_ != Step::WAIT_READY && (!fresh_joints() || !connected())) {
      finish(false, "Joint feedback stale/incomplete or driver disconnected"); return;
    }
    geometry_msgs::msg::Twist velocity;
    const double elapsed = age(step_started_);
    if (step_ == Step::WAIT_READY) {
      if (fresh_joints() && connected()) {
        issue(Step::OBSERVE, p_.observe_right, p_.observe_lift, p_.open_width, true);
      } else if (elapsed > p_.ready_timeout) { finish(false, "Driver or joint feedback not ready"); }
    } else if (resetting()) {
      if (elapsed > p_.motion_timeout) { finish(false, "Odometry reset timed out"); }
      else if (reset_samples_ >= 2 && pose_received_ && age(pose_observed_) <= p_.pose_timeout) {
        change(step_ == Step::ALIGN_RESET ? Step::ALIGN_MOVE :
          step_ == Step::FORWARD_RESET ? Step::FORWARD_MOVE : Step::BACKWARD_MOVE);
      }
    } else if (moving()) {
      if (elapsed > p_.motion_timeout) { finish(false, "Chassis movement timed out"); }
      else { motion(velocity); }
    } else if (step_ == Step::DETECT) {
      if (!detection_requested_ && command_pub_->get_subscription_count() > 1) {
        detection_started_ = now(); detection_requested_ = true;
        detection_command("start objects");
      }
      if (elapsed > p_.detection_timeout) { finish(false, "No valid target before detection timeout"); }
    } else if (step_ == Step::ALIGN_STOP && elapsed >= p_.stop_wait) {
      if (!pose_received_ || age(pose_observed_) > p_.pose_timeout) {
        finish(false, "Odometry feedback lost while stopping");
      } else if (direct_) {
        const double x = target_.x - delta_.x, y = target_.y - delta_.y;
        target_.x = x * std::cos(delta_.theta) + y * std::sin(delta_.theta);
        target_.y = -x * std::sin(delta_.theta) + y * std::cos(delta_.theta);
        plan_alignment();
      } else { begin_detection(); }
    } else if (step_ == Step::FORWARD_STOP && elapsed >= p_.stop_wait) {
      issue(Step::GRIP, {}, {}, p_.grip_width);
    } else if (step_ == Step::BACKWARD_STOP && elapsed >= p_.stop_wait) {
      issue(Step::HOLD_ARM, p_.hold_right);
    } else if (step_ == Step::GRIP) {
      if ((elapsed >= p_.grip_min_wait && reached()) || elapsed >= p_.grip_timeout) {
        if (!reached()) { RCLCPP_INFO(get_logger(), "Gripper contact wait elapsed; continuing without full closure confirmation"); }
        issue(Step::RAISE_ARM, p_.raise_right);
      }
    } else if (step_ == Step::OBSERVE || step_ == Step::ADJUST_LIFT || step_ == Step::READY_ARM ||
      step_ == Step::RAISE_ARM || step_ == Step::RESTORE_LIFT || step_ == Step::HOLD_ARM) {
      if (elapsed > p_.joint_timeout) { finish(false, "Joint motion timed out"); }
      else if (reached()) {
        switch (step_) {
          case Step::OBSERVE:
            if (direct_) { prepare_target(); } else { begin_detection(); }
            break;
          case Step::ADJUST_LIFT: plan_alignment(); break;
          case Step::READY_ARM: reset(Step::FORWARD_RESET); break;
          case Step::RAISE_ARM: issue(Step::RESTORE_LIFT, {}, p_.restore_lift); break;
          case Step::RESTORE_LIFT: reset(Step::BACKWARD_RESET); break;
          case Step::HOLD_ARM: finish(true, "Grab sequence completed"); break;
          default: break;
        }
      }
    }
    // Only the active task owns cmd_vel. Terminal states send a short stop burst then release it.
    if (active()) { vel_pub_->publish(velocity); }
  }

  Parameters p_;
  Step step_ = Step::IDLE;
  bool direct_ = false, pose_received_ = false, detection_requested_ = false;
  int align_attempts_ = 0, reset_samples_ = 0, stop_cycles_ = 0;
  size_t joint_sequence_ = 0, command_sequence_ = 0;
  double move_x_ = 0, move_y_ = 0;
  Point target_;
  Pose2D delta_;
  Clock::time_point step_started_{}, goal_started_{}, joints_observed_{}, pose_observed_{};
  rclcpp::Time detection_started_{0, 0, RCL_ROS_TIME};
  std::map<std::string, double> joints_, pending_;
  std::array<bool, 4> touched_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ctrl_pub_, command_pub_, result_pub_, status_pub_;
  std::array<rclcpp::Publisher<JointState>::SharedPtr, 4> joint_pubs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_sub_, legacy_behavior_sub_;
  rclcpp::Subscription<wpr2_msgs::msg::Object>::SharedPtr object_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr legacy_goal_sub_;
  rclcpp::Subscription<Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  tutorial::init(argc, argv);
  try {
    auto node = std::make_shared<Wpr2Grab>();
    rclcpp::WallRate rate(100);
    while (tutorial::running()) { rclcpp::spin_some(node); rate.sleep(); }
    node->shutdown();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("wpr2_grab"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
