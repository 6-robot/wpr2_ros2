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
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/string.hpp>
#include <poll.h>
#include <iostream>
#include <chrono>
#include <cctype>
#include <cmath>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wpr2_calibrate_velocity");
  auto velocity = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto reset = node->create_publisher<std_msgs::msg::String>("odom_delta_reset", 10);
  geometry_msgs::msg::Pose2D delta;
  auto sub = node->create_subscription<geometry_msgs::msg::Pose2D>("odom_delta", 10,
    [&delta](geometry_msgs::msg::Pose2D::ConstSharedPtr msg) { delta = *msg; });
  rclcpp::WallRate rate(20);
  std::cout << "W/S: 前后1米, A/D: 左右1米, Q/E: 转动360度, X: 退出（输入后回车）" << std::endl;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    pollfd input{0, POLLIN, 0};
    if (poll(&input, 1, 0) <= 0) { rate.sleep(); continue; }
    std::string text;
    if (!std::getline(std::cin, text)) break;
    if (text.empty()) continue;
    char key = std::tolower(text[0]);
    if (key == 'x') break;
    geometry_msgs::msg::Twist command;
    double seconds = 4;
    if (key == 'w') command.linear.x = 0.25;
    else if (key == 's') command.linear.x = -0.25;
    else if (key == 'a') command.linear.y = 0.25;
    else if (key == 'd') command.linear.y = -0.25;
    else if (key == 'q' || key == 'e') { command.angular.z = (key == 'q' ? 1 : -1) * M_PI / 4; seconds = 8; }
    else continue;
    std_msgs::msg::String message;
    message.data = "odom_delta reset";
    reset->publish(message);
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
      velocity->publish(command);
      rclcpp::spin_some(node);
      rate.sleep();
    }
    if (!rclcpp::ok()) break;
    velocity->publish(geometry_msgs::msg::Twist());
    for (int i = 0; i < 10 && rclcpp::ok(); ++i) { rclcpp::spin_some(node); rate.sleep(); }
    std::cout << "里程计: x=" << delta.x << " y=" << delta.y << " yaw=" << delta.theta * 180 / M_PI << std::endl;
  }
  if (rclcpp::ok()) velocity->publish(geometry_msgs::msg::Twist());
  rclcpp::shutdown();
}
