#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

const double forward_angle = M_PI;
std::shared_ptr<rclcpp::Node> node;

void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

    int count = msg->ranges.size();
    int index = std::lround((forward_angle - msg->angle_min) / msg->angle_increment);
    index = (index % count + count) % count;
    double distance = msg->ranges[index];
    RCLCPP_INFO(node->get_logger(), "Forward distance = %.3f m", distance);

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("lidar_data_node");
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), LidarCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
