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
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>

class TeleopJoy : public rclcpp::Node
{
public:
  TeleopJoy() : Node("wpr2_js_vel")
  {
    linear_ = declare_parameter("scale_linear", 0.2);
    angular_ = declare_parameter("scale_angular", 0.5);
    timeout_ = declare_parameter("joy_timeout", 1.0);
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        if (msg->axes.size() < 4 || !std::isfinite(msg->axes[0]) ||
            !std::isfinite(msg->axes[1]) || !std::isfinite(msg->axes[3])) {
          command_ = geometry_msgs::msg::Twist();
          return;
        }
        command_.linear.x = msg->axes[1] * linear_;
        command_.linear.y = msg->axes[0] * linear_;
        command_.angular.z = msg->axes[3] * angular_;
        received_ = true;
        last_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this] {
      if (!received_) return;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_).count() > timeout_)
        command_ = geometry_msgs::msg::Twist();
      publisher_->publish(command_);
    });
  }
private:
  double linear_, angular_, timeout_;
  bool received_ = false;
  geometry_msgs::msg::Twist command_;
  std::chrono::steady_clock::time_point last_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopJoy>());
  rclcpp::shutdown();
}
