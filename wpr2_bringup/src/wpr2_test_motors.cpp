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
#include "driver/wpr2_driver.h"
#include <array>
#include <chrono>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wpr2_test_motors");
  CWPR2_driver driver;
  try {
    driver.Open(node->declare_parameter<std::string>("serial_port", "/dev/ftdi").c_str(), 115200);
    const int speed = node->declare_parameter("motor_speed", 3000);
    rclcpp::WallRate rate(20);
    int state = 0;
    auto next = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (std::chrono::steady_clock::now() >= next) {
        std::array<int, 4> motors{};
        if (state > 0) motors[(state - 1) / 2] = state % 2 ? speed : -speed;
        driver.SendMotors(motors[0], motors[1], motors[2], motors[3]);
        RCLCPP_INFO(node->get_logger(), "Motor test state %d", state);
        state = (state + 1) % 9;
        next = std::chrono::steady_clock::now() + std::chrono::milliseconds(3333);
      }
      rclcpp::spin_some(node);
      rate.sleep();
    }
    driver.SendMotors(0, 0, 0, 0);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
