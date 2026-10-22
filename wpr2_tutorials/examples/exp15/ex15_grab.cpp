#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <wpr2_msgs/msg/object.hpp>


const double align_x = 0.7;
const double align_y = 0.0;
const double grab_y_offset = 0.0;
const double lift_reference_z = 1.1;
const double lift_reference_height = 0.7;
const double grab_lift_offset = 0.0;
const double observe_lift = 0.5;
const double restore_lift = 0.7;
const double forward_distance = 0.3;
const double open_width = 0.15;
const double grip_width = 0.06;
const double object_tolerance_x = 0.03;
const double object_tolerance_y = 0.03;
const double move_tolerance_x = 0.02;
const double move_tolerance_y = 0.01;
const double align_speed = 0.05;
const double forward_speed = 0.1;
const double reset_wait = 0.5;
const double stop_wait = 0.5;
const double arm_wait = 18.0;
const double lift_wait = 18.0;
const double grip_wait = 5.0;
const double raise_wait = 5.0;
const double hold_wait = 5.0;
const double joint_speed = 2000.0;
const double lift_speed = 15000.0;

const std::vector<double> observe_right = {-0.6, 1.57, 0, 1.5, 0, 0};
const std::vector<double> observe_left = {0.6, -1.57, 0, -1.5, 0, 0};
const std::vector<double> ready_right = {1.4813, 1.5723, -1.5426, 1.3623, 1.4895, -1.3667};
const std::vector<double> raise_right = {1.6673, 1.5699, -1.5818, 1.3601, 1.6793, -1.3631};
const std::vector<double> hold_right = {-0.1593, 1.5128, -0.0382, 1.7225, 0.0637, 1.5130};


enum GrabStep
{
    STEP_INIT, STEP_OBSERVE, STEP_DETECT, STEP_ADJUST_LIFT, STEP_ALIGN_OBJ,
    STEP_ALIGN_RESET, STEP_ALIGN_MOVE, STEP_ALIGN_STOP, STEP_READY_ARM,
    STEP_FORWARD_RESET, STEP_FORWARD_MOVE, STEP_FORWARD_STOP, STEP_GRIP,
    STEP_RAISE_ARM, STEP_RESTORE_LIFT, STEP_BACKWARD_RESET, STEP_BACKWARD_MOVE,
    STEP_BACKWARD_STOP, STEP_HOLD_ARM, STEP_DONE
};
int grab_step = STEP_INIT;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub, ctrl_pub, result_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr chest_pub, right_pub, left_pub, gripper_pub;
geometry_msgs::msg::Pose2D pose_diff;
bool pose_diff_ready = false;
double object_x = 0, object_y = 0, object_z = 0;
double move_x = 0, move_y = 0;
std::chrono::steady_clock::time_point step_start;
rclcpp::Time detection_start;

void SetStep(int step)
{
    grab_step = step;
    step_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(node->get_logger(), "Grab step: %d", step);
}

double StepTime()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start).count();
}

void DetectObjects()
{
    SetStep(STEP_DETECT);
    detection_start = node->now();
    std_msgs::msg::String msg;
    msg.data = "start objects";
    cmd_pub->publish(msg);
}

void ObjectCallback(const wpr2_msgs::msg::Object::SharedPtr msg)
{

    if (grab_step != STEP_DETECT || msg->x.empty() || msg->y.empty() || msg->z.empty() ||
        rclcpp::Time(msg->header.stamp, node->get_clock()->get_clock_type()) < detection_start) { return; }
    object_x = msg->x[0];
    object_y = msg->y[0];
    object_z = msg->z[0];
    std_msgs::msg::String stop_msg;
    stop_msg.data = "stop objects";
    cmd_pub->publish(stop_msg);
    SetStep(STEP_ADJUST_LIFT);
}

void PoseDiffCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    pose_diff = *msg;
    if (StepTime() >= reset_wait && std::fabs(msg->x) <= 0.001 &&
        std::fabs(msg->y) <= 0.001 && std::fabs(msg->theta) <= 0.001)
    {
        pose_diff_ready = true;
    }
}


void JointCommand(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & pub,
                  const std::string & name, const std::vector<double> & position, double speed)
{
    sensor_msgs::msg::JointState msg;
    if (name == "right" || name == "left")
    {
        for (int i = 1; i <= 6; ++i) { msg.name.push_back(name + "_joint_" + std::to_string(i)); }
    }
    else { msg.name = {name}; }
    msg.position = position;
    msg.velocity.assign(position.size(), speed);
    pub->publish(msg);
}

void ResetOdometry(int next_step)
{
    pose_diff_ready = false;
    SetStep(next_step);
    std_msgs::msg::String msg;
    msg.data = "odom_delta reset";
    ctrl_pub->publish(msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("ex15_grab");
    cmd_pub = node->create_publisher<std_msgs::msg::String>("/wpr2/behavior", 10);
    ctrl_pub = node->create_publisher<std_msgs::msg::String>("/odom_delta_reset", 10);
    result_pub = node->create_publisher<std_msgs::msg::String>("/wpr2/grab_result", 10);
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    chest_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/chest_height", 10);
    right_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/right_arm", 10);
    left_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/left_arm", 10);
    gripper_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpr2/right_gripper", 10);
    auto object_sub = node->create_subscription<wpr2_msgs::msg::Object>("/wpr2/objects_3d", 10, ObjectCallback);
    auto pose_sub = node->create_subscription<geometry_msgs::msg::Pose2D>("/odom_delta", 10, PoseDiffCallback);
    rclcpp::sleep_for(std::chrono::seconds(1));
    SetStep(STEP_INIT);

    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        geometry_msgs::msg::Twist vel_msg;
        switch (grab_step)
        {
        case STEP_INIT:
            JointCommand(right_pub, "right", observe_right, joint_speed);
            JointCommand(left_pub, "left", observe_left, joint_speed);
            JointCommand(chest_pub, "chest_height", {observe_lift}, lift_speed);
            JointCommand(gripper_pub, "right_gripper", {open_width}, joint_speed);
            SetStep(STEP_OBSERVE);
            break;
        case STEP_OBSERVE:
            if (StepTime() >= arm_wait) { DetectObjects(); }
            break;
        case STEP_ADJUST_LIFT:

            JointCommand(chest_pub, "chest_height",
                {lift_reference_height - (lift_reference_z - object_z) + grab_lift_offset}, lift_speed);
            SetStep(STEP_ALIGN_OBJ);
            break;
        case STEP_ALIGN_OBJ:
            if (StepTime() < lift_wait) { break; }
            move_x = object_x - align_x;
            move_y = object_y - align_y + grab_y_offset;
            if (std::fabs(move_x) <= object_tolerance_x && std::fabs(move_y) <= object_tolerance_y)
            {
                JointCommand(right_pub, "right", ready_right, joint_speed);
                SetStep(STEP_READY_ARM);
            }
            else
            {
                if (std::fabs(move_x) <= object_tolerance_x) { move_x = 0; }
                if (std::fabs(move_y) <= object_tolerance_y) { move_y = 0; }
                ResetOdometry(STEP_ALIGN_RESET);
            }
            break;
        case STEP_ALIGN_RESET:
        case STEP_FORWARD_RESET:
        case STEP_BACKWARD_RESET:
            if (pose_diff_ready) { SetStep(grab_step + 1); }
            break;
        case STEP_ALIGN_MOVE:
        case STEP_FORWARD_MOVE:
        case STEP_BACKWARD_MOVE:
        {

            double target_x = grab_step == STEP_ALIGN_MOVE ? move_x :
                              grab_step == STEP_FORWARD_MOVE ? forward_distance : -forward_distance;
            double target_y = grab_step == STEP_ALIGN_MOVE ? move_y : 0;
            double diff_x = target_x - pose_diff.x;
            double diff_y = target_y - pose_diff.y;
            double speed = grab_step == STEP_ALIGN_MOVE ? align_speed : forward_speed;
            if (std::fabs(diff_x) > move_tolerance_x) { vel_msg.linear.x = diff_x > 0 ? speed : -speed; }
            if (std::fabs(diff_y) > move_tolerance_y) { vel_msg.linear.y = diff_y > 0 ? speed : -speed; }
            if (std::fabs(diff_x) <= move_tolerance_x && std::fabs(diff_y) <= move_tolerance_y)
            {
                SetStep(grab_step + 1);
            }
            break;
        }
        case STEP_ALIGN_STOP:
            if (StepTime() >= stop_wait) { DetectObjects(); }
            break;
        case STEP_READY_ARM:
            if (StepTime() >= arm_wait) { ResetOdometry(STEP_FORWARD_RESET); }
            break;
        case STEP_FORWARD_STOP:
            if (StepTime() >= stop_wait)
            {
                JointCommand(gripper_pub, "right_gripper", {grip_width}, joint_speed);
                SetStep(STEP_GRIP);
            }
            break;
        case STEP_GRIP:
            if (StepTime() >= grip_wait)
            {
                JointCommand(right_pub, "right", raise_right, joint_speed);
                SetStep(STEP_RAISE_ARM);
            }
            break;
        case STEP_RAISE_ARM:
            if (StepTime() >= raise_wait)
            {
                JointCommand(chest_pub, "chest_height", {restore_lift}, lift_speed);
                SetStep(STEP_RESTORE_LIFT);
            }
            break;
        case STEP_RESTORE_LIFT:
            if (StepTime() >= lift_wait) { ResetOdometry(STEP_BACKWARD_RESET); }
            break;
        case STEP_BACKWARD_STOP:
            if (StepTime() >= stop_wait)
            {
                JointCommand(right_pub, "right", hold_right, joint_speed);
                SetStep(STEP_HOLD_ARM);
            }
            break;
        case STEP_HOLD_ARM:
            if (StepTime() >= hold_wait)
            {
                vel_pub->publish(geometry_msgs::msg::Twist());
                std_msgs::msg::String result;
                result.data = "grab done";
                result_pub->publish(result);
                SetStep(STEP_DONE);
            }
            break;
        default:
            break;
        }

        if (grab_step != STEP_DONE) { vel_pub->publish(vel_msg); }
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
