// Adapted from wpr_simulation2/src/objects_publisher.cpp.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <wpr2_msgs/msg/object.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace std;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

class ObjectsPublisher : public rclcpp::Node
{
public:
  explicit ObjectsPublisher(const char * node_name)
  : Node(node_name)
  {
    declare_parameter("object_min_height", 0.05);
    declare_parameter("cluster_tolerance", 0.03);
    declare_parameter("min_cluster_size", 50);
    stable_frame_count_ = declare_parameter("stable_frame_count", 3);
    stable_distance_threshold_ = declare_parameter("stable_distance_threshold", 0.05);
    lock_radius_ = declare_parameter("lock_radius", 0.10);
    if(stable_frame_count_ < 1 || !std::isfinite(stable_distance_threshold_) ||
       stable_distance_threshold_ <= 0.0 || !std::isfinite(lock_radius_) || lock_radius_ <= 0.0)
    {
      throw std::invalid_argument("stable_frame_count must be >= 1; stability threshold and lock radius must be finite and positive");
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth_registered/points",
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ObjectsPublisher::pointcloudCallback, this, std::placeholders::_1));
    objects_pub_ = this->create_publisher<wpr2_msgs::msg::Object>("/wpr2/objects_3d", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/objects_marker", 10);
    behavior_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/wpr2/behavior", 10, std::bind(&ObjectsPublisher::behaviorCallback, this, std::placeholders::_1));

    this->declare_parameter<bool>("auto_start",false);
    this->get_parameter("auto_start", auto_start_param);
    RCLCPP_INFO(this->get_logger(), "auto_start = %d", auto_start_param);
  }

private:
  enum class Mode { SEARCH, LOCKED };

  static geometry_msgs::msg::Point boxCenter(const stBoxMarker & box)
  {
    geometry_msgs::msg::Point center;
    center.x = (static_cast<double>(box.xMin) + box.xMax) / 2.0;
    center.y = (static_cast<double>(box.yMin) + box.yMax) / 2.0;
    center.z = (static_cast<double>(box.zMin) + box.zMax) / 2.0;
    return center;
  }

  static double squaredDistance(
    const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
  }

  void resetSearch()
  {
    if(mode_ == Mode::LOCKED)
    {
      RCLCPP_INFO(get_logger(), "Target unlocked; returning to search mode");
    }
    mode_ = Mode::SEARCH;
    stable_centers_.clear();
    locked_center_ = geometry_msgs::msg::Point();
    locked_plane_height_ = 0.0;
  }

  void clearObjects()
  {
    RemoveBoxes();
    marker_pub_->publish(marker_array);
  }

  void detectionFailed()
  {
    resetSearch();
    clearObjects();
    // 不发布空数组，也不重新发布上一帧的目标位置。
  }

  void behaviorCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if(msg->data == "start objects")
    {
      // 每次启动指令都重新搜索，包括已经锁定或正在累计稳定帧的情况。
      resetSearch();
      clearObjects();
      auto_start_param = true;
    }
    else if(msg->data == "stop objects")
    {
      auto_start_param = false;
      resetSearch();
      clearObjects();
    }
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    if(!auto_start_param)
      return;
    if(input->width == 0 || input->height == 0 || input->data.empty())
    {
      detectionFailed();
      return;
    }

    // 两种模式都先将点云转换到机器人坐标系。
    sensor_msgs::msg::PointCloud2 pc_footprint;
    try
    {
      if(!tf_buffer_->canTransform("base_footprint", input->header.frame_id, input->header.stamp) ||
         !pcl_ros::transformPointCloud("base_footprint", *input, pc_footprint, *tf_buffer_))
      {
        detectionFailed();
        return;
      }
    }
    catch(const tf2::TransformException & error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Point cloud TF failed: %s", error.what());
      detectionFailed();
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint, cloud_src);
    cloud_src.erase(std::remove_if(cloud_src.begin(), cloud_src.end(),
      [&](const pcl::PointXYZRGB & p) {
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          return true;
        if(mode_ == Mode::SEARCH)
          return false;
        geometry_msgs::msg::Point point;
        point.x = p.x; point.y = p.y; point.z = p.z;
        return squaredDistance(point, locked_center_) > lock_radius_ * lock_radius_;
      }), cloud_src.end());

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    if(mode_ == Mode::SEARCH)
    {
      // 搜索模式：前方0.5～1.5米、左右各0.5米、高度0.5～1.5米。
      pass.setInputCloud(cloud_src.makeShared());
      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.5, 1.5);
      pass.filter(cloud_src);
      pass.setInputCloud(cloud_src.makeShared());
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-0.5, 0.5);
      pass.filter(cloud_src);
      pass.setInputCloud(cloud_src.makeShared());
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.5, 1.5);
      pass.filter(cloud_src);
    }
    if(cloud_src.empty())
    {
      detectionFailed();
      return;
    }

    // 球形截取或搜索范围截取后，两种模式均使用5毫米体素降采样。
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud_src.makeShared());
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel.filter(cloud_src);
    if(cloud_src.empty())
    {
      detectionFailed();
      return;
    }

    double plane_height = locked_plane_height_;
    if(mode_ == Mode::SEARCH)
    {
      // 仅在搜索模式重新检测水平面。
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
      segmentation.setInputCloud(cloud_src.makeShared());
      segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      segmentation.setAxis(Eigen::Vector3f::UnitZ());
      segmentation.setEpsAngle(0.15);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setDistanceThreshold(0.05);
      segmentation.setOptimizeCoefficients(true);
      pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
      segmentation.segment(*plane_indices, *coefficients);
      if(plane_indices->indices.empty())
      {
        detectionFailed();
        return;
      }
      double height_sum = 0.0;
      for(int index : plane_indices->indices)
        height_sum += cloud_src.points[index].z;
      plane_height = height_sum / plane_indices->indices.size();
      RCLCPP_INFO(get_logger(), "Search plane height = %.2f", plane_height);
    }

    // 锁定模式沿用进入锁定时的水平面高度，其余过滤与聚类参数不变。
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(plane_height + get_parameter("object_min_height").as_double(), 1.5);
    pass.filter(cloud_src);
    if(cloud_src.empty())
    {
      detectionFailed();
      return;
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_src.makeShared());
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(get_parameter("cluster_tolerance").as_double());
    ec.setMinClusterSize(get_parameter("min_cluster_size").as_int());
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_src.makeShared());
    ec.extract(cluster_indices);

    std::vector<stBoxMarker> boxes;
    for(const auto & cluster : cluster_indices)
    {
      const auto & first = cloud_src.points[cluster.indices.front()];
      stBoxMarker box{first.x, first.x, first.y, first.y, first.z, first.z};
      for(int index : cluster.indices)
      {
        const auto & point = cloud_src.points[index];
        box.xMin = std::min(box.xMin, point.x);
        box.xMax = std::max(box.xMax, point.x);
        box.yMin = std::min(box.yMin, point.y);
        box.yMax = std::max(box.yMax, point.y);
        box.zMin = std::min(box.zMin, point.z);
        box.zMax = std::max(box.zMax, point.z);
      }
      // 锁定后使用局部球形范围，不再套用搜索范围的包围框边界。
      if(mode_ == Mode::LOCKED ||
         (box.xMin < 1.5 && box.yMin > -0.5 && box.yMax < 0.5))
        boxes.push_back(box);
    }
    if(boxes.empty())
    {
      detectionFailed();
      return;
    }

    // 搜索时选离机器人原点最近的物体；锁定时选离上一帧中心最近的聚类。
    const geometry_msgs::msg::Point reference =
      mode_ == Mode::SEARCH ? geometry_msgs::msg::Point() : locked_center_;
    size_t selected = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for(size_t i = 0; i < boxes.size(); ++i)
    {
      const double distance = squaredDistance(boxCenter(boxes[i]), reference);
      if(distance < nearest_distance)
      {
        nearest_distance = distance;
        selected = i;
      }
    }
    const auto center = boxCenter(boxes[selected]);
    if(mode_ == Mode::SEARCH)
    {
      // 连续观测中的任意两个中心都必须位于稳定阈值以内。
      const double threshold_squared = stable_distance_threshold_ * stable_distance_threshold_;
      if(std::any_of(stable_centers_.begin(), stable_centers_.end(),
          [&](const geometry_msgs::msg::Point & previous) {
            return squaredDistance(center, previous) > threshold_squared;
          }))
        stable_centers_.clear();
      stable_centers_.push_back(center);
      if(stable_centers_.size() >= static_cast<size_t>(stable_frame_count_))
      {
        mode_ = Mode::LOCKED;
        locked_plane_height_ = plane_height;
        stable_centers_.clear();
        RCLCPP_INFO(get_logger(), "Target locked at (%.3f, %.3f, %.3f), plane height %.3f",
          center.x, center.y, center.z, locked_plane_height_);
      }
    }

    RemoveBoxes();
    if(mode_ == Mode::LOCKED)
    {
      locked_center_ = center;
      drawObject(boxes[selected], 0);
      publishObject(boxes[selected], input->header);
    }
    else
    {
      for(size_t i = 0; i < boxes.size(); ++i)
        drawObject(boxes[i], static_cast<int>(i));
    }
    marker_pub_->publish(marker_array);
  }

  void drawObject(const stBoxMarker & box, int index)
  {
    DrawBox(box.xMin, box.xMax, box.yMin, box.yMax, box.zMin, box.zMax,
      0, 1, 0, 2 * index);
    DrawText("object_" + std::to_string(index), 0.06, box.xMax,
      (box.yMin + box.yMax) / 2, box.zMax + 0.02, 1, 0, 1, 2 * index + 1);
  }

  void publishObject(const stBoxMarker & box, const std_msgs::msg::Header & header)
  {
    wpr2_msgs::msg::Object object_msg;
    object_msg.header = header;
    object_msg.header.frame_id = "base_footprint";
    object_msg.name.push_back("object_0");
    // 保留抓取接口的参考点定义，不能用跟踪用的包围框中心替代。
    object_msg.x.push_back(box.xMax);
    object_msg.y.push_back((box.yMin + box.yMax) / 2);
    object_msg.z.push_back(box.zMin);
    object_msg.probability.push_back(1.0);
    objects_pub_->publish(object_msg);
  }

  void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB, int inID)
  {
      line_box.header.frame_id = "base_footprint";
      line_box.header.stamp = this->now();
      line_box.ns = "line_box";
      line_box.action = visualization_msgs::msg::Marker::ADD;
      line_box.id = inID;
      line_box.type = visualization_msgs::msg::Marker::LINE_LIST;
      line_box.scale.x = 0.005;
      line_box.color.r = inR;
      line_box.color.g = inG;
      line_box.color.b = inB;
      line_box.color.a = 1.0;

      line_box.points.clear();
      geometry_msgs::msg::Point p;
      p.z = inMinZ;
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

      p.z = inMaxZ;
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

      marker_array.markers.push_back(line_box);
  }

  void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB, int inID)
  {
      text_marker.header.frame_id = "base_footprint";
      text_marker.header.stamp = this->now();
      text_marker.id = inID;
      text_marker.ns = "line_obj";
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = inScale;
      text_marker.color.r = inR;
      text_marker.color.g = inG;
      text_marker.color.b = inB;
      text_marker.color.a = 1.0;

      text_marker.pose.position.x = inX;
      text_marker.pose.position.y = inY;
      text_marker.pose.position.z = inZ;

      text_marker.text = inText;

      marker_array.markers.push_back(text_marker);
  }

  void RemoveBoxes()
  {
      marker_array.markers.clear();
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear);
  }

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_sub_;
  rclcpp::Publisher<wpr2_msgs::msg::Object>::SharedPtr objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::Marker line_box;
  visualization_msgs::msg::Marker text_marker;

  visualization_msgs::msg::MarkerArray marker_array;
  bool auto_start_param;
  Mode mode_ = Mode::SEARCH;
  int stable_frame_count_;
  double stable_distance_threshold_;
  double lock_radius_;
  double locked_plane_height_ = 0.0;
  geometry_msgs::msg::Point locked_center_;
  std::vector<geometry_msgs::msg::Point> stable_centers_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectsPublisher>("wpr2_objects_3d");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
