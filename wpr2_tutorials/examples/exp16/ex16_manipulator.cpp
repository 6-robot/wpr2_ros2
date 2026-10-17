#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

const double pose_period = 5.0;

const double joint_speed = 2000.0;
const double lift_speed = 15000.0;

const std::vector<double> left_pose_1 = {0.5, -1.57, 0, -1.16, 0, 0};
const std::vector<double> left_pose_2 = {0.3, -1.4, 0.1, -0.6, 0.4, -0.2};
const std::vector<double> right_pose_1 = {-0.5, 1.57, 0, 1.16, 0, 0};
const std::vector<double> right_pose_2 = {-0.3, 1.4, -0.1, 0.6, -0.4, 0.2};
const double lift_pose_1 = 0.65, lift_pose_2 = 0.7;
const double grip_pose_1 = 0.05, grip_pose_2 = 0.1;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ex16_manipulator");
    auto chest_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/chest_height", 10);
    auto left_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/left_arm", 10);
    auto right_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/right_arm", 10);
    auto left_gripper_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/left_gripper", 10);
    auto right_gripper_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/right_gripper", 10);

    sensor_msgs::msg::JointState chest, left, right, left_gripper, right_gripper;
    chest.name = {"chest_height"};
    left.name = {"left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6"};
    right.name = {"right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"};
    left_gripper.name = {"left_gripper"};
    right_gripper.name = {"right_gripper"};
    chest.velocity = {lift_speed};
    left.velocity = right.velocity = std::vector<double>(6, joint_speed);
    left_gripper.velocity = right_gripper.velocity = {joint_speed};

    rclcpp::Rate loop_rate(1.0 / pose_period);

    rclcpp::sleep_for(std::chrono::seconds(1));
    int pose = 0;
    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Pose %d", pose + 1);
        chest.position = {pose == 0 ? lift_pose_1 : lift_pose_2};
        left.position = pose == 0 ? left_pose_1 : left_pose_2;
        right.position = pose == 0 ? right_pose_1 : right_pose_2;
        left_gripper.position = right_gripper.position = {pose == 0 ? grip_pose_1 : grip_pose_2};
        chest_pub->publish(chest);
        left_pub->publish(left);
        right_pub->publish(right);
        left_gripper_pub->publish(left_gripper);
        right_gripper_pub->publish(right_gripper);
        pose = 1 - pose;
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
