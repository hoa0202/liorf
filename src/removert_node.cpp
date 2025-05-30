// src/removert_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h> 

#include <unordered_map>
#include <string>
#include <cmath>
#include <unordered_set>
#include <deque>
#include <vector>

using PointT = pcl::PointXYZ; // For static map and internal processing
using PointTRGB = pcl::PointXYZRGB; // For dynamic points visualization (optional, can use PointT too)

class RemovertNode : public rclcpp::Node
{
public:
  RemovertNode()
  : Node("removert_node")
  {
    // 파라미터 선언
    this->declare_parameter<std::string>("local_cloud_topic", "/liorf/mapping/cloud_registered_raw");
    this->declare_parameter<double>("voxel_size", 0.1);
    this->declare_parameter<int>("frame_threshold", 5);      // 이만큼 연속으로 보여야 static
    this->declare_parameter<std::string>("fixed_frame_id", "map"); // TF 변환 및 게시될 클라우드의 기준 프레임
    // this->declare_parameter<std::string>("robot_base_frame", "base_link"); // Not actively used in this logic version


    // 파라미터 읽기
    this->get_parameter("local_cloud_topic", local_cloud_topic_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("frame_threshold", frame_threshold_);
    this->get_parameter("fixed_frame_id", fixed_frame_id_);
    // this->get_parameter("robot_base_frame", robot_base_frame_);

    RCLCPP_INFO(this->get_logger(), "Parameters: voxel_size=%.2f, frame_threshold=%d, fixed_frame_id=%s",
                voxel_size_, frame_threshold_, fixed_frame_id_.c_str());

    if (voxel_size_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "voxel_size parameter must be positive.");
        rclcpp::shutdown();
        return;
    }
    if (frame_threshold_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "frame_threshold parameter must be positive.");
        rclcpp::shutdown();
        return;
    }

    global_static_map_pcl_ = pcl::make_shared<pcl::PointCloud<PointT>>();
    global_static_map_pcl_->header.frame_id = fixed_frame_id_;


    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    local_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      local_cloud_topic_,
      rclcpp::SensorDataQoS(), 
      std::bind(&RemovertNode::localCloudCB, this, std::placeholders::_1)
    );

    dynamic_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_points", 10);
    static_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("remove_global_map", rclcpp::QoS(1).reliable().transient_local()); // Latched for static map

    RCLCPP_INFO(get_logger(),
      "RemovertNode (accumulating static map logic) initialized. Waiting for local clouds on `%s`.",
      local_cloud_topic_.c_str());
  }

private:
  inline std::string voxelKey(const PointT &p) const {
    int ix = static_cast<int>(std::floor(p.x / voxel_size_));
    int iy = static_cast<int>(std::floor(p.y / voxel_size_));
    int iz = static_cast<int>(std::floor(p.z / voxel_size_));
    return std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iz);
  }
  
void localCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud_tf_msg;

  try {
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(
          fixed_frame_id_, msg->header.frame_id,
          msg->header.stamp, tf2::durationFromSec(0.05));  // 짧은 timeout으로 시도

    tf2::doTransform(*msg, cloud_tf_msg, transform_stamped);

  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
      "[TF WARN] Cannot transform from %s to %s at time %f. Skip frame. Reason: %s",
      msg->header.frame_id.c_str(), fixed_frame_id_.c_str(),
      rclcpp::Time(msg->header.stamp).seconds(), ex.what());
    return;
  }

  auto current_local_scan_transformed = pcl::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(cloud_tf_msg, *current_local_scan_transformed);

  if (current_local_scan_transformed->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Transformed local cloud is empty. Skipping.");
    return;
  }

  // --- Voxel Mapping ---
  std::unordered_map<std::string, PointT> current_frame_voxel_points;
  for (const auto& pt : *current_local_scan_transformed) {
    std::string key = voxelKey(pt);
    if (current_frame_voxel_points.find(key) == current_frame_voxel_points.end()) {
      current_frame_voxel_points[key] = pt;
    }
  }

  std::unordered_map<std::string, int> next_voxel_observed_streak;
  bool new_static_points_added = false;

  for (const auto& pair : current_frame_voxel_points) {
    const std::string& key = pair.first;
    const PointT& pt_repr = pair.second;

    int current_streak = voxel_observed_streak_[key];
    next_voxel_observed_streak[key] = current_streak + 1;

    if (next_voxel_observed_streak[key] >= frame_threshold_ && !static_voxel_keys_.count(key)) {
      static_voxel_keys_.insert(key);
      global_static_map_pcl_->push_back(pt_repr);
      new_static_points_added = true;
    }
  }
  voxel_observed_streak_ = next_voxel_observed_streak;

  // --- Dynamic Point 추출 ---
  auto dynamic_points_pcl = pcl::make_shared<pcl::PointCloud<PointTRGB>>();
  dynamic_points_pcl->header.frame_id = fixed_frame_id_;
  pcl_conversions::toPCL(this->get_clock()->now(), dynamic_points_pcl->header.stamp);

  for (const auto& pt : *current_local_scan_transformed) {
    if (!static_voxel_keys_.count(voxelKey(pt))) {
      PointTRGB p_rgb;
      p_rgb.x = pt.x; p_rgb.y = pt.y; p_rgb.z = pt.z;
      p_rgb.r = 255; p_rgb.g = 0; p_rgb.b = 0;
      dynamic_points_pcl->push_back(p_rgb);
    }
  }

  // Dynamic Point publish
  sensor_msgs::msg::PointCloud2 dynamic_msg;
  pcl::toROSMsg(*dynamic_points_pcl, dynamic_msg);
  dynamic_pub_->publish(dynamic_msg);

  // Static Map publish → 조건부로 수행
  if (new_static_points_added) {
    sensor_msgs::msg::PointCloud2 static_map_msg;
    pcl_conversions::toPCL(this->get_clock()->now(), global_static_map_pcl_->header.stamp);
    pcl::toROSMsg(*global_static_map_pcl_, static_map_msg);
    static_map_pub_->publish(static_map_msg);
    // RCLCPP_INFO(this->get_logger(), "Static map updated. Size: %zu", global_static_map_pcl_->size());
  }
}

  // Parameters
  std::string local_cloud_topic_;
  double voxel_size_;
  int    frame_threshold_;
  std::string fixed_frame_id_;

  // State
  std::unordered_map<std::string, int> voxel_observed_streak_; // Tracks consecutive observations for each voxel
  std::unordered_set<std::string> static_voxel_keys_;         // Set of voxel keys deemed globally static
  pcl::PointCloud<PointT>::Ptr global_static_map_pcl_;        // Accumulates points from static voxels

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_pub_, static_map_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<RemovertNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}