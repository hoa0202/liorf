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
    this->declare_parameter<std::string>("pointcloud_topic", "/remove_global_map");
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<double>("height_min", 1.0);
    this->declare_parameter<double>("height_max", 1.2);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("robot_base_frame", "base_link");
    this->declare_parameter<double>("map_update_interval", 1.0);

    this->get_parameter("pointcloud_topic", pointcloud_topic_);
    this->get_parameter("resolution", resolution_);
    this->get_parameter("height_min", height_min_);
    this->get_parameter("height_max", height_max_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    double interval; this->get_parameter("map_update_interval", interval);

    RCLCPP_INFO(this->get_logger(), "Parameters: res=%.2f, topic=%s, map_frame=%s, robot_frame=%s",
                resolution_, pointcloud_topic_.c_str(), map_frame_.c_str(), robot_base_frame_.c_str());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos(rclcpp::KeepLast(1)); qos.transient_local();
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", qos);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GridMapGenerator::pointCloudCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(interval),
      std::bind(&GridMapGenerator::generateAndPublishGrid, this));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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

    for (const auto& pt : cloud_filtered->points) {
      current_min_x_ = std::min(current_min_x_, (double)pt.x);
      current_min_y_ = std::min(current_min_y_, (double)pt.y);
      current_max_x_ = std::max(current_max_x_, (double)pt.x);
      current_max_y_ = std::max(current_max_y_, (double)pt.y);
    }
  }

  void generateAndPublishGrid() {
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
      auto t = tf_buffer_->lookupTransform(map_frame_, robot_base_frame_, tf2::TimePointZero);
      robot_pose.pose.position.x = t.transform.translation.x;
      robot_pose.pose.position.y = t.transform.translation.y;
      robot_pose.pose.position.z = t.transform.translation.z;
      robot_pose.pose.orientation = t.transform.rotation;
    } catch (...) { return; }

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (accumulated_map_cloud_->empty()) return;
      grid = generateGrid(robot_pose.pose);
    }

    if (grid) {
      grid->header.stamp = this->now();
      grid_pub_->publish(*grid);
      RCLCPP_INFO(this->get_logger(), "Published grid (%u x %u)", grid->info.width, grid->info.height);
    }
  }

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> generateGrid(const geometry_msgs::msg::Pose& robot_pose) {
    double pad = resolution_ * 5;
    double min_x = current_min_x_ - pad, min_y = current_min_y_ - pad;
    double max_x = current_max_x_ + pad, max_y = current_max_y_ + pad;
    uint32_t width  = std::ceil((max_x - min_x) / resolution_);
    uint32_t height = std::ceil((max_y - min_y) / resolution_);
    size_t   new_size = (size_t)width * height;
    if (width == 0 || height == 0 || width * height > 25000000) return nullptr;

    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = map_frame_;
    grid->info.resolution = resolution_;
    grid->info.width = width; grid->info.height = height;
    grid->info.origin.position.x = min_x;
    grid->info.origin.position.y = min_y;
    grid->info.origin.orientation.w = 1.0;

    // 2) 영역이 달라지면 리사이즈 & 기존 데이터 재매핑
    if (!persistent_initialized_ ||
        width  != persistent_width_ ||
        height != persistent_height_ ||
        fabs(min_x - persistent_min_x_) > 1e-6 ||
        fabs(min_y - persistent_min_y_) > 1e-6)
    {
      std::vector<int8_t> new_data(new_size, -1);

      if (persistent_initialized_) {
        // 예전 맵(old_w × old_h) 데이터를 world 좌표 → new 맵 인덱스로 복사
        for (uint32_t y=0; y<persistent_height_; ++y) {
          for (uint32_t x=0; x<persistent_width_; ++x) {
            size_t old_idx = y * persistent_width_ + x;
            int8_t val = persistent_data_[old_idx];
            if (val < 0) continue;
            double wx = persistent_min_x_ + (x + 0.5)*resolution_;
            double wy = persistent_min_y_ + (y + 0.5)*resolution_;
            int nx = int((wx - min_x)/resolution_);
            int ny = int((wy - min_y)/resolution_);
            if (nx>=0 && nx<(int)width && ny>=0 && ny<(int)height) {
              new_data[ny*width + nx] = val;
            }
          }
        }
      }

      // 교체
      persistent_data_.swap(new_data);
      persistent_width_     = width;
      persistent_height_    = height;
      persistent_min_x_     = min_x;
      persistent_min_y_     = min_y;
      persistent_initialized_ = true;
    }
    grid->data = persistent_data_;

    int ix_r = static_cast<int>((robot_pose.position.x - min_x) / resolution_);
    int iy_r = static_cast<int>((robot_pose.position.y - min_y) / resolution_);

    // Raycasting per point using Bresenham
    for (const auto& pt : accumulated_map_cloud_->points) {
      int ix = static_cast<int>((pt.x - min_x) / resolution_);
      int iy = static_cast<int>((pt.y - min_y) / resolution_);
      if (ix < 0 || ix >= (int)width || iy < 0 || iy >= (int)height) continue;

      int x0 = ix_r, y0 = iy_r, x1 = ix, y1 = iy;
      int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
      int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
      int err = dx - dy;

      int x = x0, y = y0;
      while (true) {
        int idx = y * width + x;
        if (x == x1 && y == y1) {
          grid->data[idx] = 100;  // Occupied
          break;
        } else {
          if (grid->data[idx] != 100) grid->data[idx] = 0;  // Free
        }
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx)  { err += dx; y += sy; }
      }
    }

    // Raycast 후 grid->data 에 0/100 마킹을 마쳤다면
    // unknown(-1)을 건드리지 않고, 오직 0 또는 100만 덮어쓰기
    for (size_t i = 0; i < grid->data.size(); ++i) {
      if (grid->data[i] == 0 || grid->data[i] == 100) {
        persistent_data_[i] = grid->data[i];
      }
      else if (persistent_data_[i] == 100 && grid->data[i] == -1) {
        // 이전에 occupied였다가, 지금은 unknown → free 처리
        persistent_data_[i] = 0;
      }
    }
    return grid;
  }

  std::string pointcloud_topic_, map_frame_, robot_base_frame_;
  double resolution_, height_min_, height_max_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_map_cloud_;
  std::mutex map_mutex_;
  double current_min_x_, current_min_y_, current_max_x_, current_max_y_;

  std::vector<int8_t> persistent_data_;
  uint32_t persistent_width_, persistent_height_;
  double persistent_min_x_, persistent_min_y_;
  bool persistent_initialized_ = false;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapGenerator>());
  rclcpp::shutdown();
  return 0;
}
