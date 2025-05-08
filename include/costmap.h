#ifndef COSTMAP_H
#define COSTMAP_H

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <numeric>
#include <vector>

using PointType = pcl::PointXYZI;

class CostmapGenerator
{
public:
    CostmapGenerator(rclcpp::Node* node);
    ~CostmapGenerator() = default;

    // 파라미터 설정
    void setParameters(double resolution, double width, double height,
                      double min_height, double max_height,
                      int obstacle_threshold, int point_threshold,
                      double height_diff_threshold,
                      const std::string& base_frame_id,
                      bool auto_resize);

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

private:
    // 높이 기반 클라우드 분류 함수
    void classifyCloudByHeight(const pcl::PointCloud<PointType>::Ptr& cloud,
                               pcl::PointCloud<PointType>::Ptr& high_obstacles,
                               pcl::PointCloud<PointType>::Ptr& mid_obstacles,
                               pcl::PointCloud<PointType>::Ptr& low_obstacles);

    // 포인트를 그리드로 투영하는 함수
    void projectPointsToGrid(const pcl::PointCloud<PointType>::Ptr& cloud,
                            std::vector<std::vector<std::vector<float>>>& grid_points,
                            std::vector<std::vector<bool>>& cells);

    // 로봇 위치 마커 추가 (그리드 맵에 로봇 위치 표시)
    void addRobotMarker(nav_msgs::msg::OccupancyGrid::UniquePtr& grid_msg);

    // 노드 포인터 (로깅용)
    rclcpp::Node* node_;

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
    
    // 코스트맵 데이터
    nav_msgs::msg::OccupancyGrid costmap_;
};

#endif // COSTMAP_H 