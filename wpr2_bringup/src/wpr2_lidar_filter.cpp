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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter() : Node("wpr2_lidar_filter")
  {
    remaining_ = declare_parameter("ignore_count", 20);
    front_ = declare_parameter("front", 0.24);
    back_ = declare_parameter("back", -0.20);
    side_ = declare_parameter("side", 0.29);
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      declare_parameter<std::string>("pub_topic", "scan_filtered"), rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("scan_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        if (remaining_ > 0) { --remaining_; return; }
        auto output = *msg;
        for (size_t i = 0; i < output.ranges.size(); ++i) {
          // The description mounts laser with yaw=pi relative to base_link.
          const double angle = msg->angle_min + i * msg->angle_increment;
          const double x = -msg->ranges[i] * std::cos(angle);
          const double y = -msg->ranges[i] * std::sin(angle);
          if (x > back_ && x < front_ && y > -side_ && y < side_)
            output.ranges[i] = msg->range_max + 1.0f;
        }
        publisher_->publish(output);
      });
  }
private:
  int remaining_;
  double front_, back_, side_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
}
