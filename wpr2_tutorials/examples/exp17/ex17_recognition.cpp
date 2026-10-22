#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

std::shared_ptr<rclcpp::Node> node;

void SpeechCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "识别文字：%s", msg->data.c_str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("sr_node");

    auto speech_sub = node->create_subscription<std_msgs::msg::String>("/speech/text", 10, SpeechCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
