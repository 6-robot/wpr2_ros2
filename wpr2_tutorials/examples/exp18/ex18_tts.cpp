#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

const std::string text = "你好，欢迎使用六部工坊 WPR2 机器人";

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("speak_node");
    auto pub = node->create_publisher<std_msgs::msg::String>("/tts/text", 10);

    rclcpp::sleep_for(std::chrono::seconds(1));
    std_msgs::msg::String message;
    message.data = text;
    pub->publish(message);
    RCLCPP_INFO(node->get_logger(), "已发送朗读文字：%s", text.c_str());

    rclcpp::sleep_for(std::chrono::milliseconds(200));
    rclcpp::shutdown();
    return 0;
}
