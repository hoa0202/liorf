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
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h> // For pcl::search::KdTree

#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

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
    this->declare_parameter<std::string>("pointcloud_topic");

    // 나머지 파라미터도 동일하게 기본값 인자를 지우고 선언만
    this->declare_parameter<double>("resolution");
    this->declare_parameter<double>("height_min");
    this->declare_parameter<double>("height_max");
    this->declare_parameter<std::string>("map_frame");
    this->declare_parameter<std::string>("robot_base_frame");
    this->declare_parameter<double>("map_update_interval");
    this->declare_parameter<bool>("enable_line_fitting", false);
    this->declare_parameter<int>("min_cluster_size_for_line_fitting", 5); //10
    this->declare_parameter<double>("cluster_tolerance_factor", 4.0); // resolution * factor 2.5
    this->declare_parameter<double>("line_ransac_distance_threshold_factor", 0.8); // resolution * factor 0.5
    this->get_parameter("pointcloud_topic", pointcloud_topic_);
    this->get_parameter("resolution", resolution_);
    this->get_parameter("height_min", height_min_);
    this->get_parameter("height_max", height_max_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("enable_line_fitting", enable_line_fitting_);
    this->get_parameter("min_cluster_size_for_line_fitting", min_cluster_size_for_line_fitting_);
    this->get_parameter("cluster_tolerance_factor", cluster_tolerance_factor_);
    this->get_parameter("line_ransac_distance_threshold_factor", line_ransac_distance_threshold_factor_);
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
    
    const size_t MAX_SIZE = 300000;       // 예시값, 상황에 따라 조절
    const size_t REMOVE_SIZE = 50000;     // 한 번에 버릴 수
    if (accumulated_map_cloud_->size() > MAX_SIZE) {
      accumulated_map_cloud_->erase(accumulated_map_cloud_->begin(),
                                    accumulated_map_cloud_->begin() + REMOVE_SIZE);
    }

    for (const auto& pt : cloud_filtered->points) {
      current_min_x_ = std::min(current_min_x_, (double)pt.x);
      current_min_y_ = std::min(current_min_y_, (double)pt.y);
      current_max_x_ = std::max(current_max_x_, (double)pt.x);
      current_max_y_ = std::max(current_max_y_, (double)pt.y);
    }
  }


  // 내부 함수: OccupancyGrid를 PGM + YAML로 저장 (Nav2 호환)
  void saveGridToPGMYAML(const nav_msgs::msg::OccupancyGrid& grid, const std::string& pgm_path, const std::string& yaml_path) {
    int width = grid.info.width;
    int height = grid.info.height;
    float resolution = grid.info.resolution;
    float origin_x = grid.info.origin.position.x;
    float origin_y = grid.info.origin.position.y;

    // OccupancyGrid 데이터를 OpenCV 이미지로 변환 (0: occupied, 254: free, 205: unknown)
    cv::Mat image(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int val = grid.data[(height - y - 1) * width + x]; // Y-axis flip for image
        if (val == 0)        image.at<uchar>(y, x) = 254;   // free
        else if (val == 100) image.at<uchar>(y, x) = 0;     // occupied
        else                 image.at<uchar>(y, x) = 205;   // unknown
      }
    }
    cv::imwrite(pgm_path, image);

    // YAML 파일 작성
    std::ofstream yaml_out(yaml_path);
    yaml_out << "image: " << fs::path(pgm_path).filename() << "\n";
    yaml_out << "resolution: " << resolution << "\n";
    yaml_out << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    yaml_out << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    yaml_out.close();

    // RCLCPP_INFO(rclcpp::get_logger("grid_map_generator"), "Saved grid map as %s and %s", pgm_path.c_str(), yaml_path.c_str());
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
      // RCLCPP_INFO(this->get_logger(), "Published grid (%u x %u)", grid->info.width, grid->info.height);
    }
    // generateAndPublishGrid 내부 마지막에 grid 저장 추가
    if (grid) {
      grid->header.stamp = this->now();
      grid_pub_->publish(*grid);
      // RCLCPP_INFO(this->get_logger(), "Published grid (%u x %u)", grid->info.width, grid->info.height);

      // 저장 위치 설정 (liorf_ws/src/liorf/GRID)
      std::string map_name = "saved_grid_map";
      std::string dir = "/root/dblio_ws/src/liorf/GRID";
      fs::create_directories(dir);
      saveGridToPGMYAML(*grid, dir + "/" + map_name + ".pgm", dir + "/" + map_name + ".yaml");
    }  
  }

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> generateGrid(const geometry_msgs::msg::Pose& robot_pose) {
    double pad = resolution_ * 5;
    double min_x = current_min_x_ - pad, min_y = current_min_y_ - pad;
    double max_x = current_max_x_ + pad, max_y = current_max_y_ + pad;
    uint32_t width  = std::ceil((max_x - min_x) / resolution_);
    uint32_t height = std::ceil((max_y - min_y) / resolution_);
    size_t   new_size = (size_t)width * height;
    if (width == 0 || height == 0 || width * height > 25000000) return nullptr; // Max 5000x5000 map

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
      std::vector<int8_t> new_data(new_size, -1); // Initialize with unknown

      if (persistent_initialized_) {
        for (uint32_t y_old=0; y_old<persistent_height_; ++y_old) {
          for (uint32_t x_old=0; x_old<persistent_width_; ++x_old) {
            size_t old_idx = y_old * persistent_width_ + x_old;
            int8_t val = persistent_data_[old_idx];
            if (val < 0) continue; // Skip unknown cells
            double wx = persistent_min_x_ + (x_old + 0.5)*resolution_;
            double wy = persistent_min_y_ + (y_old + 0.5)*resolution_;
            int nx = static_cast<int>((wx - min_x)/resolution_);
            int ny = static_cast<int>((wy - min_y)/resolution_);
            if (nx>=0 && nx<(int)width && ny>=0 && ny<(int)height) {
              new_data[ny*width + nx] = val;
            }
          }
        }
      }
      persistent_data_.swap(new_data);
      persistent_width_     = width;
      persistent_height_    = height;
      persistent_min_x_     = min_x;
      persistent_min_y_     = min_y;
      persistent_initialized_ = true;
    }
    grid->data = persistent_data_; // Start with the remapped persistent data

    int ix_r = static_cast<int>((robot_pose.position.x - min_x) / resolution_);
    int iy_r = static_cast<int>((robot_pose.position.y - min_y) / resolution_);

    // Raycasting per point using Bresenham
    // This updates grid->data with current sensor information (0 for free, 100 for occupied)
    for (const auto& pt : accumulated_map_cloud_->points) { // accumulated_map_cloud_ is in map_frame_
      int ix = static_cast<int>((pt.x - min_x) / resolution_);
      int iy = static_cast<int>((pt.y - min_y) / resolution_);
      
      // Check bounds for the target point itself
      if (ix < 0 || ix >= (int)width || iy < 0 || iy >= (int)height) continue;

      // Raycasting from robot pose to the point
      int x0 = ix_r, y0 = iy_r, x1 = ix, y1 = iy;
      int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
      int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1; // dy is negative
      int err = dx + dy; // error value e_xy

      int x = x0, y = y0;
      while (true) {
        if (x < 0 || x >= (int)width || y < 0 || y >= (int)height) {
             // Ray went out of bounds before reaching the target point
             // This can happen if robot is outside current map bounds, or point is far.
             break;
        }
        int idx = y * width + x;
        // if (idx < 0 || idx >= (int)grid->data.size()) break; // Should be caught by x,y checks

        if (x == x1 && y == y1) {
          grid->data[idx] = 100;  // Occupied
          break;
        } else {
          // Mark as free only if not already marked as occupied by another ray's endpoint
          if (grid->data[idx] != 100) grid->data[idx] = 0;  // Free
        }
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; } // e_xy+e_x > 0
        if (e2 <= dx) { err += dx; y += sy; } // e_xy+e_y < 0
      }
    }
    // At this point, grid->data contains raycasted information (0s and 100s) potentially overwriting persistent_data.
    // Unknown cells (-1) from persistent_data are still -1 if no ray touched them.

    // --- START LINE FITTING ---
    if (enable_line_fitting_ && width > 0 && height > 0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_cells_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (uint32_t r_idx = 0; r_idx < height; ++r_idx) {
            for (uint32_t c_idx = 0; c_idx < width; ++c_idx) {
                if (grid->data[r_idx * width + c_idx] == 100) {
                    double world_x = min_x + (c_idx + 0.5) * resolution_;
                    double world_y = min_y + (r_idx + 0.5) * resolution_;
                    occupied_cells_cloud->points.emplace_back(world_x, world_y, 0.0);
                }
            }
        }
        occupied_cells_cloud->width = occupied_cells_cloud->points.size();
        occupied_cells_cloud->height = 1;
        occupied_cells_cloud->is_dense = true;

        if (occupied_cells_cloud->points.size() >= static_cast<size_t>(min_cluster_size_for_line_fitting_)) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(occupied_cells_cloud);

            std::vector<pcl::PointIndices> cluster_indices_vec;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(resolution_ * cluster_tolerance_factor_); 
            ec.setMinClusterSize(min_cluster_size_for_line_fitting_);
            ec.setMaxClusterSize(10000); // Adjust as needed
            ec.setSearchMethod(tree);
            ec.setInputCloud(occupied_cells_cloud);
            ec.extract(cluster_indices_vec);
            
            // Create a temporary grid to draw lines on, initialized from current grid->data
            // This way, we preserve free space markings from raycasting.
            std::vector<int8_t> line_drawn_data = grid->data; 

            for (const auto& ECI : cluster_indices_vec) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& idx : ECI.indices) {
                    cloud_cluster->points.push_back(occupied_cells_cloud->points[idx]);
                }
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                if (cloud_cluster->points.size() < static_cast<size_t>(min_cluster_size_for_line_fitting_)) continue;

                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_LINE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(resolution_ * line_ransac_distance_threshold_factor_);
                seg.setInputCloud(cloud_cluster);
                seg.segment(*inliers, *coefficients);

                if (inliers->indices.size() < 2 || coefficients->values.size() < 6) { // Need at least 2 points for a line segment
                    continue; 
                }

                // Get line parameters: point (x,y,z) and direction (dx,dy,dz)
                Eigen::Vector3f line_pt_eigen(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
                Eigen::Vector3f line_dir_eigen(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
                line_dir_eigen.normalize();


                // Project all points in the cluster onto the line to find the extent
                double min_proj = std::numeric_limits<double>::max();
                double max_proj = std::numeric_limits<double>::lowest();

                for (const auto& pt_idx : inliers->indices) { // Iterate over inliers of the line
                    Eigen::Vector3f p(cloud_cluster->points[pt_idx].x, cloud_cluster->points[pt_idx].y, cloud_cluster->points[pt_idx].z);
                    double proj = (p - line_pt_eigen).dot(line_dir_eigen);
                    min_proj = std::min(min_proj, proj);
                    max_proj = std::max(max_proj, proj);
                }
                
                if (max_proj < min_proj + resolution_) continue; // Line segment is too short

                Eigen::Vector3f start_eigen = line_pt_eigen + min_proj * line_dir_eigen;
                Eigen::Vector3f end_eigen   = line_pt_eigen + max_proj * line_dir_eigen;

                // Convert world endpoints to grid cell coordinates
                int x0_l = static_cast<int>((start_eigen.x() - min_x) / resolution_);
                int y0_l = static_cast<int>((start_eigen.y() - min_y) / resolution_);
                int x1_l = static_cast<int>((end_eigen.x() - min_x) / resolution_);
                int y1_l = static_cast<int>((end_eigen.y() - min_y) / resolution_);

                // Draw this line segment onto line_drawn_data using Bresenham
                int dx_l = std::abs(x1_l - x0_l), sx_l = x0_l < x1_l ? 1 : -1;
                int dy_l = -std::abs(y1_l - y0_l), sy_l = y0_l < y1_l ? 1 : -1;
                int err_l = dx_l + dy_l;
                int cur_x_l = x0_l, cur_y_l = y0_l;

                while (true) {
                    if (cur_x_l >= 0 && cur_x_l < (int)width && cur_y_l >= 0 && cur_y_l < (int)height) {
                        line_drawn_data[cur_y_l * width + cur_x_l] = 100; // Mark as occupied
                    }
                    if (cur_x_l == x1_l && cur_y_l == y1_l) break;
                    int e2_l = 2 * err_l;
                    if (e2_l >= dy_l) { err_l += dy_l; cur_x_l += sx_l; }
                    if (e2_l <= dx_l) { err_l += dx_l; cur_y_l += sy_l; }
                }
            }
            // Update grid->data with the line-fitted data
            grid->data = line_drawn_data;
        }
    }
    // --- END LINE FITTING ---



    std::vector<bool> seen_cells(grid->data.size(), false);

    for (const auto& pt : accumulated_map_cloud_->points) {
      int ix = static_cast<int>((pt.x - min_x) / resolution_);
      int iy = static_cast<int>((pt.y - min_y) / resolution_);
      if (ix < 0 || ix >= (int)width || iy < 0 || iy >= (int)height) continue;

      int x0 = ix_r, y0 = iy_r, x1 = ix, y1 = iy;
      int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
      int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
      int err = dx + dy;

      int x = x0, y = y0;
      while (true) {
        if (x < 0 || x >= (int)width || y < 0 || y >= (int)height) break;
        int idx = y * width + x;
        seen_cells[idx] = true;

        if (x == x1 && y == y1) {
          grid->data[idx] = 100;
          break;
        } else {
          if (grid->data[idx] != 100) grid->data[idx] = 0;
        }
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
      }
    }

    // 이전 persistent_data와 비교하며 occupied였는데 ray로 관측되지 않은 경우 → free 처리
    for (size_t i = 0; i < grid->data.size(); ++i) {
      if (!seen_cells[i]) {
        if (persistent_data_[i] == 100) {
          grid->data[i] = 0; // Free로 전환
        } else {
          grid->data[i] = persistent_data_[i]; // 유지
        }
      }
      // Free나 Occupied는 덮어씀
      if (grid->data[i] == 0 || grid->data[i] == 100) {
        persistent_data_[i] = grid->data[i];
      }
    }

    // 결과 저장
    grid->data = persistent_data_;

    return grid;
  }


  std::string pointcloud_topic_, map_frame_, robot_base_frame_;
  double resolution_, height_min_, height_max_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool enable_line_fitting_;
  int min_cluster_size_for_line_fitting_;
  double cluster_tolerance_factor_; // Multiplied by resolution
  double line_ransac_distance_threshold_factor_; // Multiplied by resolution
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
