#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


const double linear_x = 0.1;
const double linear_y = 0.0;
const double angular_z = 0.0;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("velocity_command_node");
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = linear_x;
    vel_msg.linear.y = linear_y;
    vel_msg.angular.z = angular_z;

    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok())
    {
        vel_pub->publish(vel_msg);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
