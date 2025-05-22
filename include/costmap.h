#ifndef COSTMAP_H
#define COSTMAP_H

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <numeric>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <memory>
#include <atomic>

using PointType = pcl::PointXYZI;

// PointTypePose 정의 (mapOptimization.h와 호환성 유지)
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

// 클라우드 데이터 구조체 - 코스트맵 생성에 필요한 데이터 보관
struct CloudData {
    pcl::PointCloud<PointType>::Ptr keyPoses3D;
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudFrames;
    std::vector<PointTypePose> keyPoses6D;
    
    CloudData() {
        keyPoses3D.reset(new pcl::PointCloud<PointType>());
    }
};

class CostmapGenerator
{
public:
    CostmapGenerator(rclcpp::Node* node);
    ~CostmapGenerator();

    // 파라미터 설정
    void setParameters(double resolution, double width, double height,
                      double min_height = 0.15, double max_height = 2.0,
                      int obstacle_threshold = 2, int point_threshold = 1,
                      double height_diff_threshold = 0.01,
                      const std::string& base_frame_id = "map",
                      bool auto_resize = true);

    // 맵 크기 업데이트 (포인트 클라우드 범위에 따라)
    void updateMapSize(const pcl::PointCloud<PointType>::Ptr& cloud);

    // 코스트맵 생성 (메인 함수)
    nav_msgs::msg::OccupancyGrid::UniquePtr generateCostmap(
        const pcl::PointCloud<PointType>::Ptr& cloud);
        
    // Getter 메소드들
    double getCostmapResolution() const { return costmap_resolution_; }
    double getCostmapWidth() const { return costmap_width_; }
    double getCostmapHeight() const { return costmap_height_; }
    double getMinHeightThreshold() const { return min_height_threshold_; }
    double getMaxHeightThreshold() const { return max_height_threshold_; }
    int getObstacleThreshold() const { return obstacle_threshold_; }
    int getPointThreshold() const { return point_threshold_; }
    double getHeightDiffThreshold() const { return height_diff_threshold_; }
    std::string getBaseFrameId() const { return base_frame_id_; }
    bool getAutoResizeMap() const { return auto_resize_map_; }
    
    int getCostmapWidthCells() const { return costmap_width_cells_; }
    int getCostmapHeightCells() const { return costmap_height_cells_; }
    float getCostmapOriginX() const { return costmap_origin_x_; }
    float getCostmapOriginY() const { return costmap_origin_y_; }
    
    float getLowHeightMin() const { return low_height_min_; }
    float getLowHeightMax() const { return low_height_max_; }
    float getMidHeightMin() const { return mid_height_min_; }
    float getMidHeightMax() const { return mid_height_max_; }
    float getHighHeightMin() const { return high_height_min_; }
    float getHighHeightMax() const { return high_height_max_; }
    
    bool isOriginInitialized() const { return origin_initialized_; }
    
    // 맵 최적화 노드로부터 데이터 수신
    void processClouds(const pcl::PointCloud<PointType>::Ptr& keyPoses3D,
                      const std::vector<pcl::PointCloud<PointType>::Ptr>& surfCloudFrames,
                      const std::vector<PointTypePose>& keyPoses6D);
                      
    // 코스트맵 스레드 시작/종료
    void startCostmapThread();
    void stopCostmapThread();
    
    // 글로벌 맵 경계 업데이트
    void updateGlobalMapBounds(const pcl::PointCloud<PointType>::Ptr& cloud);
    
    // 포인트 클라우드 변환 유틸리티 함수
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, 
                                                       const PointTypePose* transformIn);

    // 로봇 위치 마커 추가
    void addRobotMarker(std::unique_ptr<nav_msgs::msg::OccupancyGrid>& grid_msg);

private:
    // 코스트맵 스레드 함수
    void costmapThreadFunc();
    
    // 높이 기반 클라우드 분류 함수
    void classifyCloudByHeight(const pcl::PointCloud<PointType>::Ptr& cloud,
                               pcl::PointCloud<PointType>::Ptr& high_obstacles,
                               pcl::PointCloud<PointType>::Ptr& mid_obstacles,
                               pcl::PointCloud<PointType>::Ptr& low_obstacles);

    // 포인트를 그리드로 투영하는 함수
    void projectPointsToGrid(const pcl::PointCloud<PointType>::Ptr& cloud,
                            std::vector<std::vector<std::vector<float>>>& grid_points,
                            std::vector<std::vector<bool>>& cells);

    // 노드 포인터 (로깅용)
    rclcpp::Node* node_;
    
    // 코스트맵 발행기
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // 코스트맵 파라미터
    double costmap_resolution_;
    double costmap_width_;
    double costmap_height_;
    double min_height_threshold_;
    double max_height_threshold_;
    int obstacle_threshold_;
    int point_threshold_;
    double height_diff_threshold_;
    std::string base_frame_id_;
    bool auto_resize_map_;

    // 그리드 크기 관련 변수
    int costmap_width_cells_;
    int costmap_height_cells_;
    float costmap_origin_x_;
    float costmap_origin_y_;

    // 높이 범위 정의
    float low_height_min_;
    float low_height_max_; 
    float mid_height_min_;
    float mid_height_max_;
    float high_height_min_;
    float high_height_max_;

    // 맵 초기화 플래그
    bool origin_initialized_;
    
    // 글로벌 맵 경계 변수
    float global_min_x_;
    float global_max_x_;
    float global_min_y_;
    float global_max_y_;
    
    // 코스트맵 데이터
    nav_msgs::msg::OccupancyGrid costmap_;
    
    // 스레드 관련 변수
    std::thread thread_;
    std::atomic<bool> running_;
    
    // 클라우드 데이터 관리
    CloudData cloud_data_;
    std::mutex cloud_mutex_;
    std::atomic<bool> data_ready_;
};

#endif // COSTMAP_H 