#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

const double forward_angle = M_PI;
const double obstacle_distance = 1.5;
const double forward_speed = 0.1;
const double turn_speed = 0.3;
const int turn_scan_count = 20;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
int turn_count = 0;

void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    geometry_msgs::msg::Twist vel_msg;

    int count = msg->ranges.size();
    int index = std::lround((forward_angle - msg->angle_min) / msg->angle_increment);
    index = (index % count + count) % count;
    double distance = msg->ranges[index];
    if (turn_count == 0 && distance < obstacle_distance)
    {
        turn_count = turn_scan_count;
    }
    if (turn_count > 0)
    {
        vel_msg.angular.z = turn_speed;
        --turn_count;
    }
    else { vel_msg.linear.x = forward_speed; }

    vel_pub->publish(vel_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("lidar_behavior_node");
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), LidarCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
