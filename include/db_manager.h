#ifndef DB_MANAGER_H
#define DB_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <sqlite3.h>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <memory>
#include <unordered_map>
#include <filesystem>
#include <Eigen/Dense>
#include <chrono>
#include <unistd.h>
#include <fstream>
#include <map>

// utility.h 파일 포함 이동 (모든 타입 정의 확인을 위해)
#include "utility.h"

class DBManager {
public:
    DBManager(rclcpp::Node* node, 
              const std::string& db_path, 
              int max_memory_keyframes = 100,
              double spatial_query_radius = 50.0,
              const std::string& clouds_directory = "",
              bool reset_on_start = false,
              bool localization_mode = false);
    
    ~DBManager();
    
    // 초기화 함수
    bool initialize();
    
    // 키프레임 관련 함수
    bool addKeyFrame(int keyframe_id, double timestamp, const PointTypePose& pose, 
                    pcl::PointCloud<PointType>::Ptr cloud);
    pcl::PointCloud<PointType>::Ptr loadCloud(int keyframe_id);
    bool deleteKeyFrame(int keyframe_id);
    
    // 루프 클로저 특징점 관련 함수
    bool addLoopFeature(int feature_id, double timestamp, const PointTypePose& pose,
                      pcl::PointCloud<PointType>::Ptr cloud);
    pcl::PointCloud<PointType>::Ptr loadLoopFeature(int feature_id);
    bool deleteLoopFeature(int feature_id);
    std::vector<int> loadLoopFeaturesByRadius(const PointTypePose& current_pose,
                                           double radius,
                                           int max_features = 20);
    
    // 공간 쿼리 함수
    std::vector<int> loadKeyFramesByRadius(const PointTypePose& current_pose, 
                                          double radius, 
                                          int max_keyframes = 20);
    
    // 전체 맵 로드 함수
    pcl::PointCloud<PointType>::Ptr loadGlobalMap(float leaf_size = 0.1);
    
    // 메모리 관리 함수
    void updateActiveWindow(const PointTypePose& current_pose);
    void updateLoopFeatureActiveWindow(const PointTypePose& current_pose);
    void startMemoryMonitoring();
    void stopMemoryMonitoring();
    
    // 유틸리티 함수
    void printStats();
    bool isInitialized() const { return initialized_; }
    int getNumKeyFrames() const;
    int getNumLoopFeatures() const;
    
    // 데이터베이스 핸들 반환 함수
    sqlite3* getDB() const { return db_; }
    
    // 메모리 제한 설정/조회 함수
    void setMaxMemoryKeyframes(int max_frames) { max_memory_keyframes_ = max_frames; }
    int getMaxMemoryKeyframes() const { return max_memory_keyframes_; }
    
    // 루프 특징점 메모리 제한 설정/조회 함수
    void setMaxMemoryLoopFeatures(int max_features) { max_memory_loop_features_ = max_features; }
    int getMaxMemoryLoopFeatures() const { return max_memory_loop_features_; }
    
    // 로컬라이제이션 모드 설정/조회 함수
    void setLocalizationMode(bool mode) { localization_mode_ = mode; }
    bool getLocalizationMode() const { return localization_mode_; }
    
private:
    // 데이터베이스 관련 변수
    sqlite3* db_;
    std::string db_path_;
    std::string clouds_directory_;
    std::string loop_features_directory_;
    bool initialized_;
    bool reset_on_start_;
    bool localization_mode_;
    
    // 노드 포인터
    rclcpp::Node* node_;
    
    // 메모리 관리 관련 변수
    int max_memory_keyframes_;
    int max_memory_loop_features_;
    double spatial_query_radius_;
    std::mutex mutex_;
    std::thread memory_monitor_thread_;
    bool stop_thread_;
    
    // 메모리 캐시
    std::unordered_map<int, pcl::PointCloud<PointType>::Ptr> cloud_cache_;
    std::vector<int> active_keyframe_ids_;
    
    // 루프 특징점 메모리 캐시
    std::unordered_map<int, pcl::PointCloud<PointType>::Ptr> loop_feature_cache_;
    std::vector<int> active_loop_feature_ids_;
    
    // 데이터베이스 초기화 함수
    bool createTables();
    bool resetDatabase();
    
    // SQL 실행 유틸리티 함수
    bool executeSql(const std::string& sql);
    
    // 포즈 변환 유틸리티 함수
    Eigen::Matrix4f poseToMatrix(const PointTypePose& pose);
    PointTypePose matrixToPose(const Eigen::Matrix4f& matrix);
    
    // 포인트 클라우드 저장/로드 함수
    bool saveCloudToFile(int keyframe_id, pcl::PointCloud<PointType>::Ptr cloud);
    pcl::PointCloud<PointType>::Ptr loadCloudFromFile(int keyframe_id);
    std::string getCloudFilePath(int keyframe_id);
    
    // 루프 특징점 클라우드 저장/로드 함수
    bool saveLoopFeatureToFile(int feature_id, pcl::PointCloud<PointType>::Ptr cloud);
    pcl::PointCloud<PointType>::Ptr loadLoopFeatureFromFile(int feature_id);
    std::string getLoopFeatureFilePath(int feature_id);
    
    // 메모리 모니터링 함수
    void memoryMonitoringThread();
    void enforceMemoryLimit();
    void enforceLoopFeatureMemoryLimit();
    
    // 시스템 메모리 사용량 확인 함수
    std::map<std::string, size_t> getProcessMemoryUsage();
    
    // 파일 시스템 유틸리티
    void createDirectoryIfNotExists(const std::string& directory);
};

#endif // DB_MANAGER_H 