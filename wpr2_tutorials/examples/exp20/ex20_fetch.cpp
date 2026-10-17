#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

const std::string kitchen_waypoint = "kitchen";
const std::string guest_waypoint = "guest";
enum FetchStep { STEP_WAIT, STEP_GOTO_KITCHEN, STEP_GRAB, STEP_GOTO_GUEST, STEP_DONE };
int fetch_step = STEP_WAIT;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navi_pub, behavior_pub;

void NaviResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Navigation: %s", msg->data.c_str());
    if (fetch_step == STEP_GOTO_KITCHEN && msg->data == "navi done")
    {
        std_msgs::msg::String command;
        command.data = "start grab";
        behavior_pub->publish(command);
        fetch_step = STEP_GRAB;
    }
    if (fetch_step == STEP_GOTO_GUEST && msg->data == "navi done")
    {
        fetch_step = STEP_DONE;
        RCLCPP_INFO(node->get_logger(), "fetch done");
    }
}

void GrabResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Grab: %s", msg->data.c_str());
    if (fetch_step == STEP_GRAB && msg->data == "grab done")
    {
        std_msgs::msg::String command;
        command.data = guest_waypoint;
        navi_pub->publish(command);
        fetch_step = STEP_GOTO_GUEST;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("ex20_fetch");
    navi_pub = node->create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);
    behavior_pub = node->create_publisher<std_msgs::msg::String>("/wpr2/behavior", 10);
    auto navi_sub = node->create_subscription<std_msgs::msg::String>("/waterplus/navi_result", 10, NaviResultCallback);
    auto grab_sub = node->create_subscription<std_msgs::msg::String>("/wpr2/grab_result", 10, GrabResultCallback);

    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok())
    {
        if (fetch_step == STEP_WAIT)
        {
            std_msgs::msg::String command;
            command.data = kitchen_waypoint;
            navi_pub->publish(command);
            fetch_step = STEP_GOTO_KITCHEN;
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
