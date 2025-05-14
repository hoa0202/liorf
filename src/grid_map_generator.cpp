#include <memory>
#include <vector>
#include <limits>
#include <cmath>
#include <string>
#include <mutex> // For std::mutex

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h> // Optional: for downsampling accumulated map

using namespace std::chrono_literals;

class GridMapGenerator : public rclcpp::Node {
public:
  GridMapGenerator()
  : Node("grid_map_generator"),
    accumulated_map_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
    current_min_x_(std::numeric_limits<double>::max()),
    current_min_y_(std::numeric_limits<double>::max()),
    current_max_x_(std::numeric_limits<double>::lowest()),
    current_max_y_(std::numeric_limits<double>::lowest())
  {
    // 파라미터 선언 및 가져오기
    this->declare_parameter<std::string>("pointcloud_topic", "/remove_global_map");
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<double>("angle_increment", M_PI / 360.0);
    this->declare_parameter<double>("height_min", 1.0);   // 지상 1.0m
    this->declare_parameter<double>("height_max", 1.2);   // 지상 1.2m
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("robot_base_frame", "base_link");
    this->declare_parameter<double>("raycasting_range", 20.0);
    this->declare_parameter<double>("map_update_interval", 1.0);

    this->get_parameter("pointcloud_topic", pointcloud_topic_);
    this->get_parameter("resolution", resolution_);
    this->get_parameter("angle_increment", angle_increment_);
    this->get_parameter("height_min", height_min_);
    this->get_parameter("height_max", height_max_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("raycasting_range", raycasting_range_);
    double map_update_interval_sec;
    this->get_parameter("map_update_interval", map_update_interval_sec);

    RCLCPP_INFO(this->get_logger(), "Parameters: resolution=%.2f, pc_topic=%s, map_frame=%s, robot_frame=%s, ray_range=%.1f",
                resolution_, pointcloud_topic_.c_str(), map_frame_.c_str(), robot_base_frame_.c_str(), raycasting_range_);

    // TF Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", qos);

    // Subscriber
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GridMapGenerator::pointCloudCallback, this, std::placeholders::_1));

    // Timer for publishing grid map
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(map_update_interval_sec),
      std::bind(&GridMapGenerator::generateAndPublishGrid, this));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Received PointCloud2 in frame '%s', but expected frame '%s'. Assuming it's in map_frame.",
                           msg->header.frame_id.c_str(), map_frame_.c_str());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_raw);
    if (cloud_raw->empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_raw);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(height_min_, height_max_);
    pass.filter(*cloud_filtered);
    if (cloud_filtered->empty()) return;

    std::lock_guard<std::mutex> lock(map_mutex_);
    *accumulated_map_cloud_ += *cloud_filtered;

    // Update map bounds
    for (const auto& pt : cloud_filtered->points) {
      current_min_x_ = std::min(current_min_x_, (double)pt.x);
      current_min_y_ = std::min(current_min_y_, (double)pt.y);
      current_max_x_ = std::max(current_max_x_, (double)pt.x);
      current_max_y_ = std::max(current_max_y_, (double)pt.y);
    }
  }

  void generateAndPublishGrid() {
    geometry_msgs::msg::PoseStamped robot_pose_map;
    try {
      auto t = tf_buffer_->lookupTransform(map_frame_, robot_base_frame_, tf2::TimePointZero);
      robot_pose_map.header.frame_id = map_frame_;
      robot_pose_map.header.stamp = t.header.stamp;
      robot_pose_map.pose.position.x = t.transform.translation.x;
      robot_pose_map.pose.position.y = t.transform.translation.y;
      robot_pose_map.pose.position.z = t.transform.translation.z;
      robot_pose_map.pose.orientation = t.transform.rotation;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Could not get transform from %s to %s: %s",
                           robot_base_frame_.c_str(), map_frame_.c_str(), ex.what());
      return;
    }

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (accumulated_map_cloud_->empty()) return;
      if (current_min_x_ > current_max_x_ || current_min_y_ > current_max_y_) return;
      grid = generateGrid(robot_pose_map.pose);
    }

    if (grid) {
      grid->header.stamp = this->now();
      grid_pub_->publish(*grid);
      RCLCPP_INFO(this->get_logger(),
                  "Published grid map (%u x %u) with %zu points. Origin: (%.2f, %.2f)",
                  grid->info.width, grid->info.height,
                  accumulated_map_cloud_->size(),
                  grid->info.origin.position.x, grid->info.origin.position.y);
    }
  }

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> generateGrid(const geometry_msgs::msg::Pose& robot_pose_in_map) {
    double padded_min_x = current_min_x_ - resolution_ * 5;
    double padded_min_y = current_min_y_ - resolution_ * 5;
    double padded_max_x = current_max_x_ + resolution_ * 5;
    double padded_max_y = current_max_y_ + resolution_ * 5;

    uint32_t width  = std::ceil((padded_max_x - padded_min_x) / resolution_);
    uint32_t height = std::ceil((padded_max_y - padded_min_y) / resolution_);
    if (width == 0 || height == 0 || width * height > 25000000) return nullptr;

    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = map_frame_;
    grid->info.resolution = resolution_;
    grid->info.width  = width;
    grid->info.height = height;
    grid->info.origin.position.x = padded_min_x;
    grid->info.origin.position.y = padded_min_y;
    grid->info.origin.orientation.w = 1.0;

    // persistent data init or reuse
    if (!persistent_initialized_
        || width != persistent_width_
        || height != persistent_height_
        || std::fabs(padded_min_x - persistent_min_x_) > 1e-6
        || std::fabs(padded_min_y - persistent_min_y_) > 1e-6)
    {
      persistent_width_ = width;
      persistent_height_ = height;
      persistent_min_x_ = padded_min_x;
      persistent_min_y_ = padded_min_y;
      persistent_data_.assign(width * height, -1);
      persistent_initialized_ = true;
    }
    grid->data = persistent_data_;

    // Mark occupied (do not overwrite free cells)
    for (const auto& pt : accumulated_map_cloud_->points) {
      int ix = static_cast<int>((pt.x - padded_min_x) / resolution_);
      int iy = static_cast<int>((pt.y - padded_min_y) / resolution_);
      if (ix >= 0 && ix < (int)width && iy >= 0 && iy < (int)height) {
        size_t idx = iy * width + ix;
        if (grid->data[idx] != 0) {  // free(0)를 덮어쓰지 않음
          grid->data[idx] = 100;
        }
      }
    }

    // Raycast free
    double rx = robot_pose_in_map.position.x;
    double ry = robot_pose_in_map.position.y;
    int ix_r = static_cast<int>((rx - padded_min_x) / resolution_);
    int iy_r = static_cast<int>((ry - padded_min_y) / resolution_);
    if (ix_r >= 0 && ix_r < (int)width && iy_r >= 0 && iy_r < (int)height) {
      int max_steps = std::ceil(raycasting_range_ / resolution_);
      for (double ang = 0; ang < 2*M_PI; ang += angle_increment_) {
        for (int s = 1; s < max_steps; ++s) {
          double x = rx + s * resolution_ * std::cos(ang);
          double y = ry + s * resolution_ * std::sin(ang);
          int ix = static_cast<int>((x - padded_min_x) / resolution_);
          int iy = static_cast<int>((y - padded_min_y) / resolution_);
          if (ix < 0 || ix >= (int)width || iy < 0 || iy >= (int)height) break;
          size_t idx = iy * width + ix;
          if (grid->data[idx] == 100) break;
          if (grid->data[idx] == -1) grid->data[idx] = 0;
        }
      }
    }

    // update persistent
    persistent_data_ = grid->data;
    return grid;
  }

  // Parameters
  std::string pointcloud_topic_;
  double resolution_;
  double angle_increment_;
  double height_min_, height_max_;
  std::string map_frame_;
  std::string robot_base_frame_;
  double raycasting_range_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Accumulated map
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_map_cloud_;
  std::mutex map_mutex_;
  double current_min_x_, current_min_y_, current_max_x_, current_max_y_;

  // Persistent grid data
  std::vector<int8_t> persistent_data_;
  uint32_t persistent_width_, persistent_height_;
  double persistent_min_x_, persistent_min_y_;
  bool persistent_initialized_ = false;

  // ROS Interfaces
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapGenerator>());
  rclcpp::shutdown();
  return 0;
}
