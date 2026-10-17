#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

std::shared_ptr<rclcpp::Node> node;

void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat imgOriginal = cv_ptr->image;
    cv::imshow("RGB", imgOriginal);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("cv_image_node");

    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::SensorDataQoS(), CamRGBCallback);

    cv::namedWindow("RGB");

    rclcpp::spin(node);

    cv::destroyAllWindows();

    rclcpp::shutdown();

    return 0;
}
