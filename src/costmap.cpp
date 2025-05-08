#include "costmap.h"
#include <pcl/filters/voxel_grid.h>

CostmapGenerator::CostmapGenerator(rclcpp::Node* node) 
    : node_(node),
    costmap_resolution_(0.1),
    costmap_width_(1.0),  // 크기를 20미터로 감소 (기존 50m)
    costmap_height_(1.0), // 크기를 20미터로 감소 (기존 50m)
    min_height_threshold_(0.15),
    max_height_threshold_(2.0),
    obstacle_threshold_(2),
    point_threshold_(1),
    height_diff_threshold_(0.01),
    base_frame_id_("map"),  // base_frame_id를 map으로 설정 (기본값)
    auto_resize_map_(true),
    origin_initialized_(false)
{
    // 그리드 크기 계산
    costmap_width_cells_ = static_cast<int>(costmap_width_ / costmap_resolution_);
    costmap_height_cells_ = static_cast<int>(costmap_height_ / costmap_resolution_);
    
    // 맵 원점 초기화 (로봇 중심)
    costmap_origin_x_ = -costmap_width_ / 2;
    costmap_origin_y_ = -costmap_height_ / 2;
    
    // 높이 범위 초기화
    low_height_min_ = min_height_threshold_;
    low_height_max_ = 0.5f;  // 낮은 장애물 (지면~0.5m)
    mid_height_min_ = 0.5f;  
    mid_height_max_ = 1.5f;  // 중간 장애물 (0.5~1.5m)
    high_height_min_ = 1.5f;
    high_height_max_ = 2.0f; // 높은 장애물 (1.5~2.0m)
    
    RCLCPP_INFO(node_->get_logger(), "Costmap integration initialized with resolution: %.2f, size: %.1fm x %.1fm, origin: (%.2f, %.2f)", 
               costmap_resolution_, costmap_width_, costmap_height_, costmap_origin_x_, costmap_origin_y_);
}

void CostmapGenerator::setParameters(double resolution, double width, double height,
                                   double min_height, double max_height,
                                   int obstacle_threshold, int point_threshold,
                                   double height_diff_threshold,
                                   const std::string& base_frame_id,
                                   bool auto_resize)
{
    costmap_resolution_ = resolution;
    costmap_width_ = width;
    costmap_height_ = height;
    min_height_threshold_ = min_height;
    max_height_threshold_ = max_height;
    obstacle_threshold_ = obstacle_threshold;
    point_threshold_ = point_threshold;
    height_diff_threshold_ = height_diff_threshold;
    base_frame_id_ = base_frame_id;
    auto_resize_map_ = auto_resize;
    
    // 그리드 크기 재계산
    costmap_width_cells_ = static_cast<int>(costmap_width_ / costmap_resolution_);
    costmap_height_cells_ = static_cast<int>(costmap_height_ / costmap_resolution_);
    
    // 맵 원점 초기화 (로봇 중심)
    costmap_origin_x_ = -costmap_width_ / 2;
    costmap_origin_y_ = -costmap_height_ / 2;
    
    // 높이 범위 업데이트
    low_height_min_ = min_height_threshold_;
    low_height_max_ = 0.5f;
    mid_height_min_ = 0.5f;
    mid_height_max_ = 1.5f;
    high_height_min_ = 1.5f;
    high_height_max_ = max_height_threshold_ > 2.0f ? 2.0f : max_height_threshold_;
    
    // RCLCPP_INFO(node_->get_logger(), "Costmap parameters updated - resolution: %.2f, size: %.1fm x %.1fm", 
    //            costmap_resolution_, costmap_width_, costmap_height_);
}

void CostmapGenerator::updateMapSize(const pcl::PointCloud<PointType>::Ptr& cloud)
{
    if (!auto_resize_map_) return;
    
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    
    if (cloud->points.empty()) {
        // 클라우드가 비어있으면, 기본 값으로 설정
        min_x = -10.0f;
        max_x = 10.0f;
        min_y = -10.0f;
        max_y = 10.0f;
    } else {
        // 포인트 클라우드의 min-max 계산
        for (const auto& point : cloud->points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
    }
    
    // 버퍼 적용 - 더 작게 설정 (기존 10m에서 5m로 축소)
    float buffer = 1.0f;
    min_x -= buffer;
    max_x += buffer;
    min_y -= buffer;
    max_y += buffer;
    
    bool need_update = false;
    
    // 맵 원점 초기화 (최초 한 번만)
    if (!origin_initialized_) {
        // 로봇 주변으로 작은 영역만 유지
        costmap_origin_x_ = -costmap_width_ / 2;  // 중앙 기준
        costmap_origin_y_ = -costmap_height_ / 2;  // 중앙 기준
        origin_initialized_ = true;
        
        RCLCPP_INFO(node_->get_logger(), "Map origin initialized at (%.2f, %.2f) with small size %.1fm x %.1fm", 
                   costmap_origin_x_, costmap_origin_y_, costmap_width_, costmap_height_);
        
        need_update = true;
    } else {
        // 확장이 필요한지 검사
        bool expand_x_min = min_x < costmap_origin_x_;
        bool expand_y_min = min_y < costmap_origin_y_;
        bool expand_x_max = max_x > (costmap_origin_x_ + costmap_width_);
        bool expand_y_max = max_y > (costmap_origin_y_ + costmap_height_);
        
        if (expand_x_min || expand_y_min || expand_x_max || expand_y_max) {
            
            float new_origin_x = costmap_origin_x_;
            float new_origin_y = costmap_origin_y_;
            float new_width = costmap_width_;
            float new_height = costmap_height_;
            
            // X 음수 방향으로 확장 필요
            if (expand_x_min) {
                float diff = costmap_origin_x_ - min_x;
                new_origin_x = min_x;
                new_width += diff;
            }
            
            // Y 음수 방향으로 확장 필요
            if (expand_y_min) {
                float diff = costmap_origin_y_ - min_y;
                new_origin_y = min_y;
                new_height += diff;
            }
            
            // X 양수 방향으로 확장 필요
            if (expand_x_max) {
                new_width = max_x - new_origin_x;
            }
            
            // Y 양수 방향으로 확장 필요
            if (expand_y_max) {
                new_height = max_y - new_origin_y;
            }
            
            // 크기 변경이 필요한 경우만 업데이트 (최소 1m 이상 차이가 있을 때)
            if (std::abs(new_width - costmap_width_) > 1.0 || 
                std::abs(new_height - costmap_height_) > 1.0 ||
                std::abs(new_origin_x - costmap_origin_x_) > 0.1 ||
                std::abs(new_origin_y - costmap_origin_y_) > 0.1) {
                
                costmap_origin_x_ = new_origin_x;
                costmap_origin_y_ = new_origin_y;
                costmap_width_ = new_width;
                costmap_height_ = new_height;
                
                RCLCPP_INFO(node_->get_logger(), "Map size updated: width=%.2fm (cells=%d), height=%.2fm (cells=%d), origin=(%.2f, %.2f)",
                           costmap_width_, costmap_width_cells_, costmap_height_, costmap_height_cells_,
                           costmap_origin_x_, costmap_origin_y_);
                
                need_update = true;
            }
        }
    }
    
    // 그리드 크기 업데이트
    costmap_width_cells_ = static_cast<int>(costmap_width_ / costmap_resolution_);
    costmap_height_cells_ = static_cast<int>(costmap_height_ / costmap_resolution_);
    
    // 맵이 업데이트되었으면 그리드 데이터도 업데이트
    if (need_update) {
        int grid_width = costmap_width_cells_;
        int grid_height = costmap_height_cells_;
        
        // 안전 처리
        grid_width = std::max(10, grid_width);
        grid_height = std::max(10, grid_height);
        
        // 코스트맵 그리드 확장 (데이터 보존)
        nav_msgs::msg::OccupancyGrid new_costmap;
        new_costmap.info.resolution = costmap_resolution_;
        new_costmap.info.width = grid_width;
        new_costmap.info.height = grid_height;
        new_costmap.info.origin.position.x = costmap_origin_x_;
        new_costmap.info.origin.position.y = costmap_origin_y_;
        new_costmap.info.origin.position.z = 0.0;
        new_costmap.info.origin.orientation.w = 1.0;
        new_costmap.header.frame_id = "map";  // 항상 map 프레임 사용
        
        // 새 그리드 크기에 맞게 데이터 할당
        new_costmap.data.resize(grid_width * grid_height, -1);  // 초기값을 -1(알 수 없음)으로 설정
        
        // 이전 데이터가 있으면 복사
        if (!costmap_.data.empty()) {
            for (int y = 0; y < std::min(grid_height, static_cast<int>(costmap_.info.height)); ++y) {
                for (int x = 0; x < std::min(grid_width, static_cast<int>(costmap_.info.width)); ++x) {
                    int old_index = y * costmap_.info.width + x;
                    int new_index = y * grid_width + x;
                    
                    if (old_index < static_cast<int>(costmap_.data.size()) && new_index < static_cast<int>(new_costmap.data.size())) {
                        new_costmap.data[new_index] = costmap_.data[old_index];
                    }
                }
            }
        }
        
        // 새 코스트맵으로 교체
        costmap_ = new_costmap;
    }
}

void CostmapGenerator::classifyCloudByHeight(const pcl::PointCloud<PointType>::Ptr& cloud,
                                           pcl::PointCloud<PointType>::Ptr& high_obstacles,
                                           pcl::PointCloud<PointType>::Ptr& mid_obstacles,
                                           pcl::PointCloud<PointType>::Ptr& low_obstacles)
{
    for (const auto& point : cloud->points)
    {
        if (point.z >= high_height_min_ && point.z <= high_height_max_) {
            high_obstacles->push_back(point);
        } else if (point.z >= mid_height_min_ && point.z < high_height_min_) {
            mid_obstacles->push_back(point);
        } else if (point.z >= low_height_min_ && point.z < mid_height_min_) {
            low_obstacles->push_back(point);
        }
    }
    
    // RCLCPP_INFO(node_->get_logger(), "Height-based points - Low: %ld, Mid: %ld, High(%.1f-%.1fm): %ld", 
    //            low_obstacles->size(), mid_obstacles->size(), high_height_min_, high_height_max_, high_obstacles->size());
}

void CostmapGenerator::projectPointsToGrid(const pcl::PointCloud<PointType>::Ptr& cloud,
                                         std::vector<std::vector<std::vector<float>>>& grid_points,
                                         std::vector<std::vector<bool>>& cells)
{
    for (const auto& point : cloud->points)
    {
        // 포인트 좌표를 그리드 인덱스로 변환
        int grid_x = static_cast<int>((point.x - costmap_origin_x_) / costmap_resolution_);
        int grid_y = static_cast<int>((point.y - costmap_origin_y_) / costmap_resolution_);
        
        // 그리드 범위 내 확인
        if (grid_x >= 0 && grid_x < costmap_width_cells_ && 
            grid_y >= 0 && grid_y < costmap_height_cells_)
        {
            // 포인트의 높이 값 저장
            grid_points[grid_x][grid_y].push_back(point.z);
            cells[grid_x][grid_y] = true;
        }
    }
}

void CostmapGenerator::addRobotMarker(nav_msgs::msg::OccupancyGrid::UniquePtr& grid_msg)
{
    // 로봇 위치를 표시 (맵 중앙)
    // 로봇은 중앙 좌표인 (0,0)에 있다고 가정
    float robot_x = 0.0;
    float robot_y = 0.0;

    // 로봇 좌표를 그리드 인덱스로 변환
    int robot_grid_x = static_cast<int>((robot_x - costmap_origin_x_) / costmap_resolution_);
    int robot_grid_y = static_cast<int>((robot_y - costmap_origin_y_) / costmap_resolution_);
    
    // 로봇 주변에 마커 표시 (중심에 있는지 확인용)
    if (robot_grid_x >= 0 && robot_grid_x < costmap_width_cells_ && 
        robot_grid_y >= 0 && robot_grid_y < costmap_height_cells_)
    {
        int idx = robot_grid_y * grid_msg->info.width + robot_grid_x;
        
        if (idx >= 0 && idx < static_cast<int>(grid_msg->data.size())) {
            grid_msg->data[idx] = 100; // 로봇 위치에 장애물 표시
            
            // 로봇 주변 십자가 표시 (더 크게)
            for (int offset = 1; offset <= 3; offset++) {
                // 우측
                if (robot_grid_x + offset < costmap_width_cells_) {
                    int cross_idx = robot_grid_y * grid_msg->info.width + (robot_grid_x + offset);
                    if (cross_idx >= 0 && cross_idx < static_cast<int>(grid_msg->data.size())) {
                        grid_msg->data[cross_idx] = 100;
                    }
                }
                // 좌측
                if (robot_grid_x - offset >= 0) {
                    int cross_idx = robot_grid_y * grid_msg->info.width + (robot_grid_x - offset);
                    if (cross_idx >= 0 && cross_idx < static_cast<int>(grid_msg->data.size())) {
                        grid_msg->data[cross_idx] = 100;
                    }
                }
                // 상단
                if (robot_grid_y + offset < costmap_height_cells_) {
                    int cross_idx = (robot_grid_y + offset) * grid_msg->info.width + robot_grid_x;
                    if (cross_idx >= 0 && cross_idx < static_cast<int>(grid_msg->data.size())) {
                        grid_msg->data[cross_idx] = 100;
                    }
                }
                // 하단
                if (robot_grid_y - offset >= 0) {
                    int cross_idx = (robot_grid_y - offset) * grid_msg->info.width + robot_grid_x;
                    if (cross_idx >= 0 && cross_idx < static_cast<int>(grid_msg->data.size())) {
                        grid_msg->data[cross_idx] = 100;
                    }
                }
            }
        }
        
        RCLCPP_INFO(node_->get_logger(), "Added robot marker at grid coords: (%d, %d), world: (%.2f, %.2f)",
                  robot_grid_x, robot_grid_y, robot_x, robot_y);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Robot position (%.2f, %.2f) is outside map bounds!", robot_x, robot_y);
    }
}

nav_msgs::msg::OccupancyGrid::UniquePtr CostmapGenerator::generateCostmap(
    const pcl::PointCloud<PointType>::Ptr& cloud)
{
    // 맵 크기 업데이트
    if (auto_resize_map_) {
        updateMapSize(cloud);
    }
    
    // 다운샘플링된 클라우드 생성
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    downSizeFilterMap.setLeafSize(0.2, 0.2, 0.2); // 20cm 해상도로 필터링
    downSizeFilterMap.setInputCloud(cloud);
    downSizeFilterMap.filter(*downsampled_cloud);
    
    // 높이 기준으로 필터링된 클라우드 생성
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>());
    for (const auto& point : downsampled_cloud->points)
    {
        if (point.z >= min_height_threshold_ && point.z <= max_height_threshold_)
        {
            filtered_cloud->push_back(point);
        }
    }
    
    // 높이 별로 포인트 클라우드 분류
    pcl::PointCloud<PointType>::Ptr high_obstacles(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr mid_obstacles(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr low_obstacles(new pcl::PointCloud<PointType>());
    
    classifyCloudByHeight(filtered_cloud, high_obstacles, mid_obstacles, low_obstacles);
    
    // 그리드 셀 초기화 - 크기 확인 및 안전 처리
    int width_cells = costmap_width_cells_;
    int height_cells = costmap_height_cells_;
    
    width_cells = std::max(10, width_cells);
    height_cells = std::max(10, height_cells);
    
    std::vector<std::vector<std::vector<float>>> grid_points(
        width_cells, std::vector<std::vector<float>>(
            height_cells, std::vector<float>()));
            
    // 높이 범위별 그리드 셀
    std::vector<std::vector<bool>> high_cells(width_cells, std::vector<bool>(height_cells, false));
    std::vector<std::vector<bool>> mid_cells(width_cells, std::vector<bool>(height_cells, false));
    std::vector<std::vector<bool>> low_cells(width_cells, std::vector<bool>(height_cells, false));
    
    // 포인트 클라우드를 그리드로 투영 (높이 범위별)
    projectPointsToGrid(high_obstacles, grid_points, high_cells);
    projectPointsToGrid(mid_obstacles, grid_points, mid_cells);
    projectPointsToGrid(low_obstacles, grid_points, low_cells);
    
    // 점유 그리드 메시지 생성
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid_msg->header.stamp = node_->now();
    grid_msg->header.frame_id = "map";  // 항상 map 프레임 사용
    grid_msg->info.resolution = costmap_resolution_;
    grid_msg->info.width = width_cells;
    grid_msg->info.height = height_cells;
    
    // 로봇 중심 좌표계로 설정
    grid_msg->info.origin.position.x = costmap_origin_x_;
    grid_msg->info.origin.position.y = costmap_origin_y_;
    grid_msg->info.origin.position.z = 0.0;
    grid_msg->info.origin.orientation.w = 1.0;
    
    // 데이터 크기 초기화 (-1: 알 수 없음)
    grid_msg->data.resize(width_cells * height_cells, -1);
    
    // 그리드 셀의 점유 확률 결정
    int high_cells_count = 0;
    int mid_cells_count = 0;
    int low_cells_count = 0;
    
    for (int i = 0; i < width_cells; ++i)
    {
        for (int j = 0; j < height_cells; ++j)
        {
            auto& cell_points = grid_points[i][j];
            int idx = j * width_cells + i;
            
            if (cell_points.empty())
            {
                // 포인트가 없으면 알 수 없음(-1)으로 유지
                continue;
            }
            
            // 높이 기반 색상 코드 - 높이에 따라 다른 값으로 시각화
            // 높은 장애물 (1.5~2.0m): 100 (검은색)
            // 중간 장애물 (0.5~1.5m): 80 (짙은 회색)
            // 낮은 장애물 (지면~0.5m): 50 (중간 회색)
            
            if (high_cells[i][j]) {
                // 높은 장애물 (1.5~2.0m) - 사용자가 원하는 장애물
                grid_msg->data[idx] = 100; // 검은색 (장애물)
                high_cells_count++;
            } 
            else if (mid_cells[i][j]) {
                // 중간 장애물 (0.5~1.5m)
                grid_msg->data[idx] = 80; // 짙은 회색
                mid_cells_count++;
            }
            else if (low_cells[i][j]) {
                // 낮은 장애물 (지면~0.5m) 
                grid_msg->data[idx] = 50; // 중간 회색
                low_cells_count++;
            } 
            else {
                // 포인트가 있지만 분류되지 않음 (예외 처리)
                grid_msg->data[idx] = 0; // 자유 공간
            }
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Costmap cells - Low: %d, Mid: %d, High: %d, Total grid: %dx%d (%.1fx%.1fm)", 
               low_cells_count, mid_cells_count, high_cells_count,
               width_cells, height_cells, costmap_width_, costmap_height_);
    
    // 로봇 위치 마커 추가
    addRobotMarker(grid_msg);
    
    return grid_msg;
} 