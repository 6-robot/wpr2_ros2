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
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <cmath>

class ArmsTest : public rclcpp::Node
{
public:
  ArmsTest() : Node("wpr2_arms_test")
  {
    for (const auto &group : {"chest_height", "left_arm", "right_arm"})
      publishers_.push_back(create_publisher<sensor_msgs::msg::JointState>(std::string("/real/wpr2/") + group, 10));
    states_ = create_subscription<sensor_msgs::msg::JointState>("/plan/joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        positions_.clear();
        if (msg->name.size() != msg->position.size()) return;
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (!std::isfinite(msg->position[i]) || positions_.count(msg->name[i])) { positions_.clear(); return; }
          positions_[msg->name[i]] = msg->position[i];
        }
      });
    command_ = create_subscription<std_msgs::msg::String>("/plan/wpr2_cmd", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data != "go") return;
        std::vector<sensor_msgs::msg::JointState> commands(3);
        commands[0].name = {"chest_height"};
        for (int i = 1; i <= 6; ++i) {
          commands[1].name.push_back("left_joint_" + std::to_string(i));
          commands[2].name.push_back("right_joint_" + std::to_string(i));
        }
        for (size_t group = 0; group < commands.size(); ++group) {
          auto &command = commands[group];
          command.header.stamp = now();
          for (const auto &name : command.name) {
            if (!positions_.count(name)) { RCLCPP_WARN(get_logger(), "Missing planned joint: %s", name.c_str()); return; }
            command.position.push_back(positions_.at(name));
            command.velocity.push_back(group == 0 ? 18000 : 2000);
          }
        }
        for (size_t i = 0; i < commands.size(); ++i) publishers_[i]->publish(commands[i]);
      });
  }
private:
  std::map<std::string, double> positions_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> publishers_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmsTest>());
  rclcpp::shutdown();
}
