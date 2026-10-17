#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>

const double target_yaw = 90.0;
const double turn_gain = 0.01;
const double forward_speed = 0.1;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

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

    geometry_msgs::msg::Twist vel_msg;

    double diff_angle = std::remainder(target_yaw - yaw, 360.0);
    vel_msg.angular.z = diff_angle * turn_gain;
    vel_msg.linear.x = forward_speed;
    vel_pub->publish(vel_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("imu_behavior_node");
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw", rclcpp::SensorDataQoS(), IMUCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
