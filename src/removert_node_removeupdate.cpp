// src/removert_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // PointCloud2 변환에 필요

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h> // For pcl::getMinMax3D (not currently used, but good to have for PCL)

#include <unordered_map>
#include <string>
#include <cmath>
#include <unordered_set>
#include <deque>

using PointT = pcl::PointXYZ;

class RemovertNode : public rclcpp::Node
{
public:
  RemovertNode()
  : Node("removert_node")
  {
    // 파라미터 선언
    this->declare_parameter<std::string>("global_map_topic", "/liorf/mapping/map_global");
    this->declare_parameter<std::string>("local_cloud_topic", "/liorf/mapping/cloud_registered_raw");
    this->declare_parameter<double>("voxel_size", 0.5);
    this->declare_parameter<double>("interest_radius", 5.0);
    this->declare_parameter<int>("frame_threshold", 10);
    this->declare_parameter<int>("accumulation_frames", 10);
    this->declare_parameter<std::string>("robot_base_frame", "base_link");
    this->declare_parameter<bool>("accumulate_dynamic_points_globally", true);

    // 파라미터 읽기
    this->get_parameter("global_map_topic", global_map_topic_);
    this->get_parameter("local_cloud_topic", local_cloud_topic_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("interest_radius", interest_radius_);
    this->get_parameter("frame_threshold", frame_threshold_);
    this->get_parameter("accumulation_frames", accumulation_frames_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("accumulate_dynamic_points_globally", accumulate_dynamic_points_globally_);

    RCLCPP_INFO(this->get_logger(), "Parameters: voxel_size=%.2f, interest_radius=%.2f, frame_threshold=%d, accumulation_frames=%d, robot_base_frame=%s, accumulate_dynamic_globally=%s",
                voxel_size_, interest_radius_, frame_threshold_, accumulation_frames_, robot_base_frame_.c_str(), accumulate_dynamic_points_globally_ ? "true" : "false");

    if (voxel_size_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "voxel_size parameter must be positive.");
        rclcpp::shutdown();
        return;
    }
    if (interest_radius_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "interest_radius parameter must be positive.");
        rclcpp::shutdown();
        return;
    }
    if (accumulation_frames_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "accumulation_frames parameter must be positive.");
        rclcpp::shutdown();
        return;
    }

    accumulated_dyn_cloud_pcl_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    rclcpp::QoS global_map_qos(1);
    global_map_qos.reliable();
    global_map_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE); // VOLATILE로 변경

    global_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      global_map_topic_,
      global_map_qos,
      std::bind(&RemovertNode::globalMapCB, this, std::placeholders::_1)
    );

    local_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      local_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RemovertNode::localCloudCB, this, std::placeholders::_1)
    );

    dyn_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_points", 10);
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("remove_global_map", 10);

    RCLCPP_INFO(get_logger(),
      "RemovertNode initialized. Waiting for global_map on `%s`. Interest radius set to %.2f m.",
      global_map_topic_.c_str(), interest_radius_);
  }

private:
  void globalMapCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // if (global_map_received_) {
    //     RCLCPP_INFO(get_logger(), "Global map already received and processed. Ignoring new map.");
    //     return;
    // }
    auto temp = std::make_shared<pcl::PointCloud<PointT>>();

    pcl::fromROSMsg(*msg, *temp);

    if (!temp->empty()) {
      RCLCPP_WARN(get_logger(), "Received empty global_map. Ignoring.");
      return;
    }
      
      // global_map_ = temp_global_map;
      // RCLCPP_INFO(get_logger(),
      //   "Loaded global_map (%zu pts). Frame ID: %s", global_map_->points.size(), msg->header.frame_id.c_str());
      // global_map_frame_id_ = msg->header.frame_id;
      // global_map_received_ = true;

      // if (accumulated_dyn_cloud_pcl_ && global_map_ && !global_map_->empty()) {
      //     accumulated_dyn_cloud_pcl_->header = global_map_->header; // Copy PCL header
      // }

      // global_sub_.reset(); 
      // RCLCPP_INFO(get_logger(), "Global map subscription has been shut down.");
      // 새로운 global map 으로 매번 갱신
      // global_map_ = temp_global_map;
      // global_map_frame_id_ = msg->header.frame_id;
      // RCLCPP_INFO(get_logger(),
      //   "Updated global_map (%zu pts). Frame ID: %s", global_map_->points.size(), global_map_frame_id_.c_str());
      // → 기존에 제거된 voxel은 제외하고 새 맵 갱신
      // auto filtered = std::make_shared<pcl::PointCloud<PointT>>();
      // filtered->header = temp_global_map->header;
      // for (const auto &pt : *temp_global_map) {
      //   if (removed_voxel_keys_.count(voxelKey(pt)) == 0)
      //     filtered->push_back(pt);
      // }
      // global_map_ = filtered;
      global_map_ = temp;
      global_map_frame_id_ = msg->header.frame_id;
      RCLCPP_INFO(get_logger(), "Received new global_map (%zu pts).", global_map_->size());
      // 동적 포인트 누적 클라우드 헤더도 업데이트
      // if (accumulated_dyn_cloud_pcl_) {
      //     accumulated_dyn_cloud_pcl_->header = global_map_->header;
      // }      
  }

  void localCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // if (!global_map_received_ || !global_map_ || global_map_->empty()) {
    if (!global_map_ || global_map_->empty()) {    
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
        "Global_map not ready or empty, skipping local cloud processing.");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_base;
    tf2::TimePoint target_lookup_time;
    bool use_latest_tf_for_lookup = false;

    try {
      target_lookup_time = tf2_ros::fromMsg(msg->header.stamp);
      tf_base = tf_buffer_->lookupTransform(
        global_map_frame_id_,
        robot_base_frame_,
        target_lookup_time, // Use converted message timestamp
        tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException &ex){
      RCLCPP_WARN(get_logger(),
        "TF lookup %s to %s failed for time %f: %s. Current ROS time: %.3f. Retrying with TimePointZero.",
        robot_base_frame_.c_str(), global_map_frame_id_.c_str(), target_lookup_time.time_since_epoch().count() / 1e9, ex.what(),
        this->get_clock()->now().seconds());
      try {
        target_lookup_time = tf2::TimePointZero; // Fallback to latest available
        use_latest_tf_for_lookup = true; // Flag that we are using latest
        tf_base = tf_buffer_->lookupTransform(
          global_map_frame_id_,
          robot_base_frame_,
          target_lookup_time,
          tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException &ex_retry) {
         RCLCPP_ERROR(get_logger(), "TF lookup retry %s to %s failed: %s.",
         robot_base_frame_.c_str(), global_map_frame_id_.c_str(), ex_retry.what());
        return;
      }
    }
    if(use_latest_tf_for_lookup) {
        RCLCPP_INFO(get_logger(), "Successfully used TimePointZero for TF lookup of robot pose.");
    }

    double bx = tf_base.transform.translation.x;
    double by = tf_base.transform.translation.y;
    double bz = tf_base.transform.translation.z;
    RCLCPP_DEBUG(get_logger(), "Robot pose in %s: (%.2f, %.2f, %.2f)", global_map_frame_id_.c_str(), bx, by, bz);

    sensor_msgs::msg::PointCloud2 cloud_tf_msg;
    tf2::TimePoint target_transform_time;
    bool use_latest_tf_for_transform = false;

    try {
      // Try to transform using the message's original timestamp
      target_transform_time = tf2_ros::fromMsg(msg->header.stamp);
      if (!tf_buffer_->canTransform(global_map_frame_id_, msg->header.frame_id, target_transform_time, tf2::durationFromSec(0.1))) {
          RCLCPP_WARN(get_logger(), "Cannot transform local scan from '%s' to '%s' at time %f. Trying with TimePointZero.",
                      msg->header.frame_id.c_str(), global_map_frame_id_.c_str(), target_transform_time.time_since_epoch().count() / 1e9);
          
          target_transform_time = tf2::TimePointZero; // Fallback to latest
          use_latest_tf_for_transform = true;
          if (!tf_buffer_->canTransform(global_map_frame_id_, msg->header.frame_id, target_transform_time, tf2::durationFromSec(0.1))) {
             RCLCPP_ERROR(get_logger(), "Cannot transform local scan from '%s' to '%s' even with TimePointZero.",
                         msg->header.frame_id.c_str(), global_map_frame_id_.c_str());
             return;
          }
      }
      
      if(use_latest_tf_for_transform) {
          RCLCPP_INFO(get_logger(), "Attempting transform of local scan with TimePointZero, fixed_frame '%s', and 0.1s timeout.", msg->header.frame_id.c_str());
      } else {
          RCLCPP_INFO(get_logger(), "Attempting transform of local scan with msg timestamp, fixed_frame '%s', and 0.1s timeout.", msg->header.frame_id.c_str());
      }

      cloud_tf_msg = tf_buffer_->transform(
        *msg,                     // source PointCloud2 message
        global_map_frame_id_,     // target_frame
        target_transform_time,    // target_time (either original or TimePointZero)
        msg->header.frame_id,     // fixed_frame (original frame_id of the cloud)
        tf2::durationFromSec(0.1) // timeout
      );

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(),
        "Failed to transform local scan from '%s' to '%s' frame: %s. Check TF and timestamps.",
        msg->header.frame_id.c_str(), global_map_frame_id_.c_str(), ex.what());
      return;
    }
     if(use_latest_tf_for_transform) {
        RCLCPP_INFO(get_logger(), "Successfully transformed local scan using TimePointZero.");
    }


    auto current_local_cloud_transformed = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromROSMsg(cloud_tf_msg, *current_local_cloud_transformed);
    if (current_local_cloud_transformed->empty()) {
        RCLCPP_WARN(get_logger(), "Transformed local cloud is empty. Skipping.");
        return;
    }

    local_scan_accumulator_.push_back(current_local_cloud_transformed);
    while (local_scan_accumulator_.size() > static_cast<size_t>(accumulation_frames_)) {
      local_scan_accumulator_.pop_front();
    }
    
    if (local_scan_accumulator_.size() < static_cast<size_t>(accumulation_frames_)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
        "Accumulating local scans, currently have %zu out of %d.",
        local_scan_accumulator_.size(), accumulation_frames_);
    }

    std::unordered_set<std::string> accumulated_local_voxels;
    size_t total_accumulated_points = 0;
    for (const auto& scan_ptr : local_scan_accumulator_) {
      if (!scan_ptr) continue; // Safety check
      accumulated_local_voxels.reserve(accumulated_local_voxels.size() + scan_ptr->size());
      total_accumulated_points += scan_ptr->size();
      for (const auto &pt : *scan_ptr) {
        accumulated_local_voxels.insert(voxelKey(pt));
      }
    }
    RCLCPP_DEBUG(get_logger(), "Generated %zu unique voxels from %zu accumulated scans (total %zu points).",
                 accumulated_local_voxels.size(), local_scan_accumulator_.size(), total_accumulated_points);

    pcl::PointCloud<PointT>::Ptr next_global_map(new pcl::PointCloud<PointT>());
    if (global_map_ && !global_map_->empty()) {
        next_global_map->header = global_map_->header;
    } else { 
        pcl_conversions::toPCL(this->get_clock()->now(), next_global_map->header.stamp);
        next_global_map->header.frame_id = global_map_frame_id_;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_frame_dyn_cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (global_map_ && !global_map_->empty()) {
      current_frame_dyn_cloud_pcl->header = global_map_->header;
    } else {
        pcl_conversions::toPCL(this->get_clock()->now(), current_frame_dyn_cloud_pcl->header.stamp);
        current_frame_dyn_cloud_pcl->header.frame_id = global_map_frame_id_;
    }

    size_t dynamic_points_found_this_frame = 0;
    size_t points_in_interest_radius = 0;
    size_t points_kept_static_in_radius = 0;
    size_t points_kept_dynamic_pending_in_radius = 0;
    size_t original_global_map_size = global_map_ ? global_map_->size() : 0;

    const double interest_radius_sq = interest_radius_ * interest_radius_;

    // if (global_map_) { 
    //     for (const auto &pt_map : global_map_->points)
    //     {
    //       double dx = pt_map.x - bx;
    //       double dy = pt_map.y - by;
    //       double dz = pt_map.z - bz;
    //       double dist_sq_to_robot = dx * dx + dy * dy + dz * dz;

    //       if (dist_sq_to_robot > interest_radius_sq) {
    //         next_global_map->push_back(pt_map);
    //         continue;
    //       }
    //       points_in_interest_radius++;

    //       std::string key = voxelKey(pt_map);
    //       bool seen_in_accumulated_local_scan = accumulated_local_voxels.count(key);

    //       if (seen_in_accumulated_local_scan) {
    //         miss_counter_[key] = 0;
    //         next_global_map->push_back(pt_map);
    //         points_kept_static_in_radius++;
    //       } else {
    //         int cnt = ++miss_counter_[key];
    //         if (cnt >= frame_threshold_) {
    //           pcl::PointXYZRGB p_dyn;
    //           p_dyn.x = pt_map.x; p_dyn.y = pt_map.y; p_dyn.z = pt_map.z;
    //           p_dyn.r = 255; p_dyn.g = 0; p_dyn.b = 0; 
              
    //           if (accumulate_dynamic_points_globally_) {
    //             if(accumulated_dyn_cloud_pcl_) accumulated_dyn_cloud_pcl_->push_back(p_dyn);
    //           } else {
    //             current_frame_dyn_cloud_pcl->push_back(p_dyn);
    //           }
    //           dynamic_points_found_this_frame++;
    //         } else {
    //           next_global_map->push_back(pt_map);
    //           points_kept_dynamic_pending_in_radius++;
    //         }
    //       }
    //     }
    // }
    if (global_map_) {
      for (const auto &pt_map : global_map_->points)
      {
        // 1) 이미 영구 제거된 voxel이면 무조건 건너뛴다
        std::string key = voxelKey(pt_map);
        if (removed_voxel_keys_.count(key)) {
          continue;
        }

        // 2) 로봇으로부터 거리 확인
        double dx = pt_map.x - bx, dy = pt_map.y - by, dz = pt_map.z - bz;
        double dist_sq_to_robot = dx*dx + dy*dy + dz*dz;
        if (dist_sq_to_robot > interest_radius_sq) {
          next_global_map->push_back(pt_map);
          continue;
        }

        // 3) 관심 반경 내: local scan에 보이면 static, 아니면 miss 카운트
        bool seen = accumulated_local_voxels.count(key);
        if (seen) {
          miss_counter_[key] = 0;
          next_global_map->push_back(pt_map);
          points_kept_static_in_radius++;
        } else {
          int cnt = ++miss_counter_[key];
          if (cnt >= frame_threshold_) {
            // → 한 번 동적으로 판단되면 영구 제거 리스트에 추가
            removed_voxel_keys_.insert(key);
            pcl::PointXYZRGB p_dyn;
            p_dyn.x = pt_map.x; p_dyn.y = pt_map.y; p_dyn.z = pt_map.z;
            p_dyn.r = 255; p_dyn.g = 0; p_dyn.b = 0; 
            
            if (accumulate_dynamic_points_globally_) {
              if(accumulated_dyn_cloud_pcl_) accumulated_dyn_cloud_pcl_->push_back(p_dyn);
            } else {
              current_frame_dyn_cloud_pcl->push_back(p_dyn);
            }            
            dynamic_points_found_this_frame++;
          } else {
            next_global_map->push_back(pt_map);
            points_kept_dynamic_pending_in_radius++;
          }
        }
      }
    }
    global_map_ = next_global_map;

    RCLCPP_INFO(get_logger(),
                "Update: OrigMap %zu -> NewMap %zu. InRadius(%.1fm): %zu. Static: %zu. PendingDyn: %zu. DynFoundNow: %zu. TotalDynAccum: %zu.",
                original_global_map_size, (global_map_ ? global_map_->size() : 0),
                interest_radius_, points_in_interest_radius,
                points_kept_static_in_radius,
                points_kept_dynamic_pending_in_radius,
                dynamic_points_found_this_frame,
                accumulate_dynamic_points_globally_ ? (accumulated_dyn_cloud_pcl_ ? accumulated_dyn_cloud_pcl_->size() : 0) : (current_frame_dyn_cloud_pcl ? current_frame_dyn_cloud_pcl->size() : 0)
                );
    
    if (points_in_interest_radius > 0 && points_kept_static_in_radius == 0 && points_kept_dynamic_pending_in_radius == 0 && dynamic_points_found_this_frame == points_in_interest_radius) {
        RCLCPP_WARN(get_logger(), "All points within interest_radius (%.2f m) were marked as dynamic! Check TF, voxel_size, thresholds.", interest_radius_);
    }

    rclcpp::Time now = this->get_clock()->now();
    sensor_msgs::msg::PointCloud2 dyn_msg, filt_msg;

    if (accumulate_dynamic_points_globally_) {
        if(accumulated_dyn_cloud_pcl_) {
            pcl::toROSMsg(*accumulated_dyn_cloud_pcl_, dyn_msg);
            dyn_msg.header.stamp = now; 
            dyn_msg.header.frame_id = global_map_frame_id_;
        } else { 
            dyn_msg.header.stamp = now;
            dyn_msg.header.frame_id = global_map_frame_id_;
            // dyn_msg will be an empty cloud if accumulated_dyn_cloud_pcl_ is null
        }
    } else {
        if(current_frame_dyn_cloud_pcl) {
            pcl::toROSMsg(*current_frame_dyn_cloud_pcl, dyn_msg);
            dyn_msg.header.stamp = now; 
            dyn_msg.header.frame_id = global_map_frame_id_;
        } else {
            dyn_msg.header.stamp = now;
            dyn_msg.header.frame_id = global_map_frame_id_;
        }
    }
    dyn_pub_->publish(dyn_msg);

    if (global_map_) { 
        pcl::toROSMsg(*global_map_, filt_msg);
        filt_msg.header.stamp = now; 
        filt_msg.header.frame_id = global_map_frame_id_;
        filtered_pub_->publish(filt_msg);
    }
  }

  std::string voxelKey(const PointT &p) const {
    int ix = static_cast<int>(std::floor(p.x / voxel_size_));
    int iy = static_cast<int>(std::floor(p.y / voxel_size_));
    int iz = static_cast<int>(std::floor(p.z / voxel_size_));
    return std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iz);
  }

  // Parameters
  std::string global_map_topic_, local_cloud_topic_, robot_base_frame_;
  double interest_radius_;
  int    frame_threshold_;
  int    accumulation_frames_;
  double voxel_size_;
  bool   accumulate_dynamic_points_globally_;

  // State
  pcl::PointCloud<PointT>::Ptr global_map_{nullptr};        // 원본 global map
  pcl::PointCloud<PointT>::Ptr remove_map_{nullptr}; 
  std::string global_map_frame_id_ = "map";
  // bool global_map_received_ = false;
  std::unordered_map<std::string, int> miss_counter_;
  std::deque<pcl::PointCloud<PointT>::Ptr> local_scan_accumulator_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_dyn_cloud_pcl_ = nullptr;
  std::unordered_set<std::string> removed_voxel_keys_;
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_sub_, local_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dyn_pub_, filtered_pub_;

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