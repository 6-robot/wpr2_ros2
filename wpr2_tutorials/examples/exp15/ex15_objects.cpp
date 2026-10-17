#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_ros/transform_listener.h>


const std::string frame = "base_footprint";
const std::array<double, 3> minimum = {0.5, -0.5, 0.5};
const std::array<double, 3> maximum = {1.5, 0.5, 1.5};
const double plane_threshold = 0.05;
const double above_plane = 0.02;
const double tolerance = 0.1;
const int min_cluster = 100;
const int max_cluster = 25000;
std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<tf2_ros::Buffer> buffer;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr objects;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers;

void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!buffer->canTransform(frame, msg->header.frame_id, msg->header.stamp)) { return; }
    sensor_msgs::msg::PointCloud2 transformed;
    pcl_ros::transformPointCloud(frame, *msg, transformed, *buffer);

    geometry_msgs::msg::PoseArray result;
    result.header = transformed.header;
    visualization_msgs::msg::MarkerArray drawing;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    drawing.markers.push_back(clear);
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(transformed, *cloud);
    pcl::PassThrough<pcl::PointXYZ> pass;
    const std::array<std::string, 3> axes = {"x", "y", "z"};
    for (size_t i = 0; i < 3; ++i) {
      pass.setInputCloud(cloud);
      pass.setFilterFieldName(axes[i]);
      pass.setFilterLimits(minimum[i], maximum[i]);
      pass.filter(*cloud);
    }
    pcl::SACSegmentation<pcl::PointXYZ> segment;
    segment.setInputCloud(cloud);
    segment.setModelType(pcl::SACMODEL_PLANE);
    segment.setMethodType(pcl::SAC_RANSAC);
    segment.setDistanceThreshold(plane_threshold);
    segment.setOptimizeCoefficients(true);
    pcl::PointIndices plane;
    pcl::ModelCoefficients coefficients;
    segment.segment(plane, coefficients);
    if (plane.indices.empty()) { return; }
    double height = 0;
    for (int i : plane.indices) { height += cloud->points[i].z; }
    height /= plane.indices.size();
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(height + above_plane, maximum[2]);
    pass.filter(*cloud);
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusters;
    clusters.setInputCloud(cloud);
    clusters.setSearchMethod(tree);
    clusters.setMinClusterSize(min_cluster);
    clusters.setMaxClusterSize(max_cluster);
    clusters.setClusterTolerance(tolerance);
    std::vector<pcl::PointIndices> indices;
    clusters.extract(indices);
    for (const auto & cluster : indices) {
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1;
      for (int i : cluster.indices) {
        pose.position.x += cloud->points[i].x;
        pose.position.y += cloud->points[i].y;
        pose.position.z += cloud->points[i].z;
      }
      pose.position.x /= cluster.indices.size();
      pose.position.y /= cluster.indices.size();
      pose.position.z /= cluster.indices.size();
      result.poses.push_back(pose);
      visualization_msgs::msg::Marker marker;
      marker.header = result.header;
      marker.ns = "objects";
      marker.id = static_cast<int>(result.poses.size());
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.06;
      marker.color.g = marker.color.a = 1;
      drawing.markers.push_back(marker);
      RCLCPP_INFO(node->get_logger(), "Object: (%.3f, %.3f, %.3f)",
        pose.position.x, pose.position.y, pose.position.z);
    }
    objects->publish(result);
    markers->publish(drawing);

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("pointcloud_objects_node");
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_ros::TransformListener listener(*buffer);
    objects = node->create_publisher<geometry_msgs::msg::PoseArray>("/wpr2/objects", 10);
    markers = node->create_publisher<visualization_msgs::msg::MarkerArray>("/objects_marker", 10);
    auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth_registered/points", rclcpp::SensorDataQoS(), PointcloudCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
