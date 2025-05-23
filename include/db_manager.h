#pragma once

#include <sqlite3.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <memory>
#include <vector>
#include <deque>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <atomic>
#include <unordered_map>
#include "point_types.h"

/**
 * @brief SLAM 데이터 관리를 위한 SQLite 데이터베이스 관리자
 * 
 * 이 클래스는 SLAM 시스템의 키프레임, 포인트 클라우드, 팩터 그래프 데이터를
 * 효율적으로 관리하기 위한 SQLite 데이터베이스 인터페이스를 제공합니다.
 * 슬라이딩 윈도우 방식으로 메모리 사용량을 최적화합니다.
 */
class DBManager {
public:
    /**
     * @brief 데이터베이스 관리자 생성자
     * 
     * @param node ROS 2 노드 포인터 (로깅용)
     * @param db_path 데이터베이스 파일 경로
     * @param active_window_size 메모리에 유지할 활성 키프레임 수
     * @param spatial_radius 공간 쿼리 기본 반경 (미터)
     */
    DBManager(rclcpp::Node* node, 
              const std::string& db_path = "slam_map.db", 
              int active_window_size = 100,
              double spatial_radius = 20.0);

    /**
     * @brief 소멸자
     */
    ~DBManager();

    /**
     * @brief 데이터베이스 초기화
     * 
     * @return 성공 여부
     */
    bool initialize();

    /**
     * @brief 키프레임 추가
     * 
     * @param id 키프레임 ID
     * @param timestamp 타임스탬프
     * @param pose 6D 포즈
     * @param cloud 포인트 클라우드
     * @return 성공 여부
     */
    bool addKeyFrame(int id, double timestamp, const PointTypePose& pose, 
                    const pcl::PointCloud<PointType>::Ptr& cloud);

    /**
     * @brief 팩터 추가
     * 
     * @param id 팩터 ID
     * @param type 팩터 타입 ("odometry", "loop", "prior")
     * @param from_id 시작 키프레임 ID
     * @param to_id 끝 키프레임 ID
     * @param transform 변환 행렬
     * @param covariance 공분산 행렬
     * @return 성공 여부
     */
    bool addFactor(int id, const std::string& type, int from_id, int to_id,
                  const Eigen::Matrix4f& transform, const Eigen::MatrixXd& covariance);

    /**
     * @brief 공간 쿼리를 통한 키프레임 로드
     * 
     * @param center_pose 중심 포즈
     * @param radius 검색 반경 (미터)
     * @param max_frames 최대 로드할 프레임 수
     * @return 검색된 키프레임 ID 목록
     */
    std::vector<int> loadKeyFramesByRadius(const PointTypePose& center_pose, 
                                          double radius = -1.0, 
                                          int max_frames = -1);

    /**
     * @brief 키프레임 포인트 클라우드 로드
     * 
     * @param id 키프레임 ID
     * @return 포인트 클라우드 (nullptr if not found)
     */
    pcl::PointCloud<PointType>::Ptr loadCloud(int id);

    /**
     * @brief 특정 키프레임 관련 팩터 로드
     * 
     * @param id 키프레임 ID
     * @return 관련 팩터 목록
     */
    std::vector<std::pair<int, Eigen::Matrix4f>> loadRelatedFactors(int id);

    /**
     * @brief 루프 클로저 후보 키프레임 검색
     * 
     * @param current_id 현재 키프레임 ID
     * @param min_distance 최소 유클리드 거리
     * @param max_frames 최대 후보 수
     * @return 후보 키프레임 ID 목록
     */
    std::vector<int> findLoopClosureCandidates(int current_id, double min_distance, int max_frames);

    /**
     * @brief GTSAM 팩터 그래프와 초기값 로드
     * 
     * @param ids 키프레임 ID 목록
     * @param graph 출력 팩터 그래프
     * @param initial 출력 초기값
     * @return 성공 여부
     */
    bool loadFactorGraphValues(const std::vector<int>& ids, 
                              gtsam::NonlinearFactorGraph& graph,
                              gtsam::Values& initial);

    /**
     * @brief 슬라이딩 윈도우 업데이트
     * 최신 키프레임 기준으로 활성 윈도우 업데이트
     * 
     * @param current_pose 현재 로봇 포즈
     */
    void updateActiveWindow(const PointTypePose& current_pose);

    /**
     * @brief 데이터베이스에 저장된 전체 맵 로드
     * 
     * @param downsampling_leaf_size 다운샘플링 크기 (메모리 절약)
     * @return 병합된 포인트 클라우드 맵
     */
    pcl::PointCloud<PointType>::Ptr loadGlobalMap(float downsampling_leaf_size = 0.1f);

    /**
     * @brief 메모리 사용량 모니터링 시작
     * 
     * @param check_interval_ms 체크 간격 (밀리초)
     * @param memory_limit_mb 메모리 한계치 (MB)
     */
    void startMemoryMonitoring(int check_interval_ms = 5000, int memory_limit_mb = 2000);

    /**
     * @brief 메모리 모니터링 중지
     */
    void stopMemoryMonitoring();

    /**
     * @brief 활성 키프레임 ID 목록 반환
     * 
     * @return 현재 메모리에 로드된 활성 키프레임 ID 목록
     */
    std::vector<int> getActiveKeyFrameIds() const;

    /**
     * @brief 포인트 클라우드 추가
     * 
     * @param keyframe_id 키프레임 ID
     * @param cloud 포인트 클라우드
     * @param resolution 해상도
     * @return 성공 여부
     */
    bool addPointCloud(int keyframe_id, const pcl::PointCloud<PointType>::Ptr& cloud, int resolution);

private:
    // SQLite 관련 변수
    sqlite3* db_;
    std::string db_path_;
    
    // 핵심 파라미터
    int active_window_size_;
    double spatial_radius_;
    
    // 쿼리 준비문
    sqlite3_stmt* stmt_add_keyframe_;
    sqlite3_stmt* stmt_add_pointcloud_;
    sqlite3_stmt* stmt_add_factor_;
    sqlite3_stmt* stmt_load_cloud_;
    sqlite3_stmt* stmt_load_keyframe_;
    sqlite3_stmt* stmt_spatial_query_;
    sqlite3_stmt* stmt_loop_candidates_;
    
    // 메모리 관리
    std::deque<int> active_keyframe_ids_;
    std::unordered_map<int, pcl::PointCloud<PointType>::Ptr> cloud_cache_;
    mutable std::mutex cache_mutex_;
    
    // 메모리 모니터링
    std::atomic<bool> monitoring_running_;
    std::unique_ptr<std::thread> monitoring_thread_;
    
    // 로깅용 ROS 노드
    rclcpp::Node* node_;

    // 내부 헬퍼 함수들
    bool prepareStatements();
    void finalizeStatements();
    void monitorMemoryUsage(int check_interval_ms, int memory_limit_mb);
    bool isKeyFrameInActiveWindow(int id) const;
    void removeOldestFromActiveWindow();
    bool serializePointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, std::vector<uint8_t>& data);
    bool deserializePointCloud(const std::vector<uint8_t>& data, pcl::PointCloud<PointType>::Ptr& cloud);
    bool createTables();
    bool beginTransaction();
    bool commitTransaction();
    bool rollbackTransaction();
    double getDistanceBetweenPoses(const PointTypePose& pose1, const PointTypePose& pose2);
}; 