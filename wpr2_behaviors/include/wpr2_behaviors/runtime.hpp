#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace tutorial
{
// Keep the ROS context alive until the final stop commands have been sent.
inline volatile std::sig_atomic_t interrupted = 0;
inline void signal_handler(int) { interrupted = 1; }
inline void init(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
}
inline bool running() { return rclcpp::ok() && !interrupted; }
using Clock = std::chrono::steady_clock;
inline double age(Clock::time_point t)
{
  return std::chrono::duration<double>(Clock::now() - t).count();
}
inline void stop(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr & pub)
{
  if (!rclcpp::ok()) { return; }
  for (int i = 0; i < 3; ++i) {
    pub->publish(geometry_msgs::msg::Twist{});
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
  }
}
inline double positive(const rclcpp::Node::SharedPtr & node, const char * name, double value)
{
  const auto result = node->declare_parameter<double>(name, value);
  if (!std::isfinite(result) || result <= 0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
  return result;
}
}  // namespace tutorial
