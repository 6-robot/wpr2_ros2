#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

std::shared_ptr<rclcpp::Node> node;

void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const auto & q = msg->orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;
    RCLCPP_INFO(node->get_logger(), "roll=%.1f pitch=%.1f yaw=%.1f deg", roll, pitch, yaw);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("imu_data_node");
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw", rclcpp::SensorDataQoS(), IMUCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
