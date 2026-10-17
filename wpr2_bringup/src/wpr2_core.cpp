/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2025-2035, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include "driver/wpr2_driver.h"

class Wpr2Core : public rclcpp::Node
{
public:
  Wpr2Core() : Node("wpr2_core"), broadcaster_(*this)
  {
    const auto mode = declare_parameter<std::string>("chassis_type", "MODE_MECANUM");
    if (mode != "MODE_MECANUM" && mode != "MODE_OMNI")
      throw std::runtime_error("chassis_type must be MODE_MECANUM or MODE_OMNI");
    driver_.nBaseMode = mode == "MODE_MECANUM" ? MODE_MECANUM : MODE_OMNI;
    imu_enabled_ = declare_parameter("imu_pub", false);
    publish_tf_ = declare_parameter("publish_odom_tf", true);
    timeout_ = declare_parameter("cmd_vel_timeout", 1.0);
    if (!std::isfinite(timeout_) || timeout_ <= 0)
      throw std::runtime_error("cmd_vel_timeout must be positive");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
    camera_height_ = declare_parameter("zeros.femto_height", 0.15);
    camera_pitch_ = declare_parameter("zeros.femto_pitch", 0.02);
    const auto port = declare_parameter<std::string>("serial_port", "/dev/ftdi");
    driver_.Open(port.c_str(), declare_parameter("serial_baudrate", 115200));
    driver_.Velocity(0, 0, 0);
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    delta_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("odom_delta", 10);
    result_pub_ = create_publisher<std_msgs::msg::String>("wpr2/joints_result", 10);
    if (imu_enabled_) imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z) || std::abs(msg->linear.x) > 10 ||
            std::abs(msg->linear.y) > 10 || std::abs(msg->angular.z) > 20) {
          RCLCPP_WARN(get_logger(), "Ignoring invalid cmd_vel");
          return;
        }
        target_ = *msg;
        last_command_ = std::chrono::steady_clock::now();
      });
    const std::array<std::string, 5> topics = {
      "left_arm", "right_arm", "left_gripper", "right_gripper", "chest_height"};
    for (size_t group = 0; group < topics.size(); ++group)
      joint_subs_.push_back(create_subscription<sensor_msgs::msg::JointState>(
        "wpr2/" + topics[group], 10,
        [this, group](sensor_msgs::msg::JointState::ConstSharedPtr msg) { joint_command(*msg, group); }));
    reset_sub_ = create_subscription<std_msgs::msg::String>("odom_delta_reset", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data.find("odom_delta reset") != std::string::npos) delta_ = geometry_msgs::msg::Pose2D();
      });
    previous_time_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { update(); });
    RCLCPP_INFO(get_logger(), "WPR2 ROS 2 driver: %s, %s", port.c_str(), mode.c_str());
  }

  ~Wpr2Core() override { stop(); }
  void stop() { driver_.Velocity(0, 0, 0); }

private:
  void joint_command(const sensor_msgs::msg::JointState &msg, size_t group)
  {
    const size_t count = group < 2 ? 6 : 1;
    if (msg.position.size() != count ||
        (!msg.velocity.empty() && msg.velocity.size() != count)) {
      RCLCPP_WARN(get_logger(), "Joint command needs %zu positions and zero or %zu velocities", count, count);
      return;
    }
    for (size_t i = 0; i < count; ++i) {
      if (!std::isfinite(msg.position[i]) || std::abs(msg.position[i]) > 100 ||
          (!msg.velocity.empty() && (!std::isfinite(msg.velocity[i]) ||
           msg.velocity[i] < 0 || msg.velocity[i] > 100000))) {
        RCLCPP_WARN(get_logger(), "Ignoring invalid joint command");
        return;
      }
    }
    if (!driver_.has_joint_feedback) {
      RCLCPP_WARN(get_logger(), "Waiting for joint feedback before accepting joint commands");
      return;
    }
    // Hold all unspecified groups at measured positions on the first command.
    if (!joints_enabled_)
      std::copy(std::begin(driver_.arJointPosRecv), std::end(driver_.arJointPosRecv), driver_.arJointPos);
    const int offset = group == 0 ? 1 : group == 1 ? 8 : group == 2 ? 7 : group == 3 ? 14 : 0;
    for (size_t i = 0; i < count; ++i) {
      double value = msg.position[i];
      if (group < 2) value *= 18000.0 / M_PI;
      else if (group < 4) value = (0.155 - std::clamp(value, 0.0, 0.155)) * 21900.0 / 0.155;
      else value = (0.7 - std::clamp(value, 0.0, 0.7)) * 1000000.0;
      driver_.arJointPos[offset + i] = static_cast<int>(value);
      driver_.arJointSpeed[offset + i] = msg.velocity.empty() ? (group == 4 ? 18000 : 2000) : msg.velocity[i];
    }
    joints_enabled_ = true;
    joint_pending_ = true;
    action_cycles_ = 0;
  }

  static geometry_msgs::msg::Quaternion quaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2);
    q.w = std::cos(yaw / 2);
    return q;
  }

  void update()
  {
    driver_.ReadNewData();
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_).count();
    if (elapsed > timeout_) {
      target_ = geometry_msgs::msg::Twist();
      current_ = geometry_msgs::msg::Twist();
    } else {
      current_.linear.x += std::clamp(target_.linear.x - current_.linear.x, -0.02, 0.02);
      current_.linear.y += std::clamp(target_.linear.y - current_.linear.y, -0.02, 0.02);
      current_.angular.z += std::clamp(target_.angular.z - current_.angular.z, -0.1, 0.1);
    }
    driver_.Velocity(current_.linear.x, current_.linear.y, current_.angular.z);
    if (joints_enabled_) driver_.JointAction();
    const auto stamp = now();
    const double dt = (stamp - previous_time_).seconds();
    previous_time_ = stamp;
    if (++driver_.nParseCount > 100) {
      std::fill(std::begin(driver_.arMotorPos), std::end(driver_.arMotorPos), 0);
      previous_encoders_.fill(0);
      driver_.nParseCount = 0;
      driver_.has_joint_feedback = false;
      joints_enabled_ = false;
      joint_pending_ = false;
      target_ = geometry_msgs::msg::Twist();
      current_ = geometry_msgs::msg::Twist();
      stop();
    }
    std::array<double, 4> diff{};
    for (size_t i = 0; i < 4; ++i) {
      // Unsigned subtraction handles signed 32-bit encoder rollover.
      diff[i] = static_cast<int32_t>(static_cast<uint32_t>(driver_.arMotorPos[i]) -
                                    static_cast<uint32_t>(previous_encoders_[i]));
      previous_encoders_[i] = driver_.arMotorPos[i];
    }
    const bool mecanum = driver_.nBaseMode == MODE_MECANUM;
    const double dx = (diff[1] - diff[0]) / (mecanum ? 190000.0 : 143000.0);
    const double dy = (diff[1] - diff[2]) / (mecanum ? 210000.0 : 141000.0);
    const double da = (diff[0] + diff[1] + diff[2] + diff[3]) / (mecanum ? 173000.0 : 119000.0);
    for (auto *pose : {&pose_, &delta_}) {
      pose->x += dx * std::cos(pose->theta) - dy * std::sin(pose->theta);
      pose->y += dx * std::sin(pose->theta) + dy * std::cos(pose->theta);
      pose->theta += da;
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.orientation = quaternion(pose_.theta);
    if (dt > 0) {
      odom.twist.twist.linear.x = dx / dt;
      odom.twist.twist.linear.y = dy / dt;
      odom.twist.twist.angular.z = da / dt;
    }
    odom_pub_->publish(odom);
    delta_pub_->publish(delta_);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose_.x;
      transform.transform.translation.y = pose_.y;
      transform.transform.rotation = odom.pose.pose.orientation;
      broadcaster_.sendTransform(transform);
    }
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {"front_left_wheel_joint", "front_right_wheel_joint", "back_right_wheel_joint", "back_left_wheel_joint",
      "chest_height", "left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6",
      "left_joint_finger", "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5",
      "right_joint_6", "right_joint_finger", "femto_height", "femto_pitch"};
    joints.position.resize(21);
    for (size_t i = 0; i < 4; ++i) joints.position[i] = driver_.arMotorPos[i] * (-0.1 * M_PI / 180);
    joints.position[4] = 0.7 - driver_.arJointPosRecv[0] * 0.000001;
    for (size_t i = 1; i < 15; ++i) joints.position[i + 4] = driver_.arJointPosRecv[i] * (M_PI / 18000);
    // The gripper encoder uses 0..21900 counts; the URDF finger joint uses 0..1.57 rad.
    joints.position[11] = std::clamp(driver_.arJointPosRecv[7] / 21900.0, 0.0, 1.0) * 1.57;
    joints.position[18] = std::clamp(driver_.arJointPosRecv[14] / 21900.0, 0.0, 1.0) * 1.57;
    joints.position[19] = camera_height_;
    joints.position[20] = camera_pitch_;
    joints_pub_->publish(joints);
    if (joint_pending_ && ++action_cycles_ >= 20) {
      action_cycles_ = 0;
      bool arrived = driver_.has_joint_feedback;
      for (size_t i = 0; i < 15; ++i)
        arrived = arrived && std::abs(static_cast<double>(driver_.arJointPosRecv[i]) - driver_.arJointPos[i]) <= 100;
      if (arrived) {
        std_msgs::msg::String result;
        result.data = "done";
        result_pub_->publish(result);
        joint_pending_ = false;
      }
    }
    if (imu_enabled_) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = imu_frame_;
      const double norm = std::sqrt(driver_.fQuatW * driver_.fQuatW + driver_.fQuatX * driver_.fQuatX +
                                    driver_.fQuatY * driver_.fQuatY + driver_.fQuatZ * driver_.fQuatZ);
      if (norm > 0 && std::isfinite(norm)) {
        imu.orientation.w = driver_.fQuatW / norm;
        imu.orientation.x = driver_.fQuatX / norm;
        imu.orientation.y = driver_.fQuatY / norm;
        imu.orientation.z = driver_.fQuatZ / norm;
      } else {
        imu.orientation.w = 1;
        imu.orientation_covariance[0] = -1;
      }
      imu.angular_velocity.x = driver_.fGyroX;
      imu.angular_velocity.y = driver_.fGyroY;
      imu.angular_velocity.z = driver_.fGyroZ;
      imu.linear_acceleration.x = driver_.fAccX;
      imu.linear_acceleration.y = driver_.fAccY;
      imu.linear_acceleration.z = driver_.fAccZ;
      imu_pub_->publish(imu);
    }
  }

  CWPR2_driver driver_;
  tf2_ros::TransformBroadcaster broadcaster_;
  bool imu_enabled_, publish_tf_, joints_enabled_ = false, joint_pending_ = false;
  int action_cycles_ = 0;
  double timeout_, camera_height_, camera_pitch_;
  std::string odom_frame_, base_frame_, imu_frame_;
  std::array<int, 4> previous_encoders_{};
  geometry_msgs::msg::Twist target_, current_;
  geometry_msgs::msg::Pose2D pose_, delta_;
  rclcpp::Time previous_time_;
  std::chrono::steady_clock::time_point last_command_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr delta_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_subs_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Wpr2Core>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("wpr2_core"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
