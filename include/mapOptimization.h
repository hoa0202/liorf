#pragma once

#include "utility.h"
#include "costmap.h" // 코스트맵 생성기 클래스 헤더 추가
#include "loopclosure.h" // Loop Closure 클래스 헤더 추가
#include "db_manager.h" // 데이터베이스 관리자 헤더 추가
#include "liorf/msg/cloud_info.hpp"
#include "liorf/srv/save_map.hpp"
// <!-- liorf_yjz_lucky_boy -->
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "Scancontext.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <numeric> // for std::accumulate
#include <thread> // 스레드 지원을 위한 헤더 추가

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

enum class SCInputType 
{ 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 

class mapOptimization : public ParamServer
{

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Subscription<liorf::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<liorf::msg::CloudInfo>::SharedPtr pubSLAMInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGpsOdom;

    rclcpp::Service<liorf::srv::SaveMap>::SharedPtr srvSaveMap;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    liorf::msg::CloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    GeographicLib::LocalCartesian gps_trans_;

    // scancontext loop closure는 LoopClosure 클래스에서 관리
    // SCManager scManager;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    // TF Buffer와 Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 코스트맵 관련 변수
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    std::unique_ptr<CostmapGenerator> costmap_generator_; // 코스트맵 생성기

    // Loop Closure 관련 변수
    std::unique_ptr<LoopClosure> loop_closure_; // 루프 클로저 모듈

    // 데이터베이스 관련 변수
    std::unique_ptr<DBManager> db_manager_; // 데이터베이스 관리자
    bool use_database_mode_; // 데이터베이스 모드 활성화 여부
    bool localization_mode_ = false;              // 로컬라이제이션 모드 여부
    int active_keyframes_window_size_; // 메모리에 유지할 키프레임 수
    int active_loop_features_window_size_; // 메모리에 유지할 루프 특징점 수
    double spatial_query_radius_; // 공간 쿼리 반경

    // --- Localization Mode를 위한 변수 추가 ---
    bool localization_initialized_ = false;       // 로컬라이제이션 초기화 여부
    bool has_new_initial_pose_ = false;          // 새로운 초기 위치 수신 여부
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg_;  // 초기 위치 메시지
    std::mutex initial_pose_mutex_;              // 초기 위치 관련 뮤텍스
    float globalMapLeafSize_ = 0.2;              // 전역 지도 다운샘플링 크기
    float surroundingKeyframeSearchRadius_ = 50.0;  // 주변 키프레임 검색 반경
    float surroundingKeyframeMapLeafSize_ = 0.2;  // 주변 키프레임 지도 다운샘플링 크기
    std::string localizationInitialPoseTopic_ = "/initialpose";  // 초기 위치 토픽
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subInitialPose_;  // 초기 위치 구독자

    // 전역 지도를 저장할 포인트 클라우드
    pcl::PointCloud<PointType>::Ptr globalMapCloud_;  // 전역 지도 클라우드
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap_;

    // 전역 지도 시각화를 위한 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalMap_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;

    // 전역 지도 다운샘플링을 위한 필터
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;

    mapOptimization(const rclcpp::NodeOptions & options);

    void allocateMemory();
    void laserCloudInfoHandler(const liorf::msg::CloudInfo::SharedPtr msgIn);
    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg);
    void pointAssociateToMap(PointType const * const pi, PointType * const po);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
    pcl::PointCloud<PointType>::Ptr transformPointCloudWithLidarOffset(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
    gtsam::Pose3 trans2gtsamPose(float transformIn[]);
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
    Eigen::Affine3f trans2Affine3f(float transformIn[]);
    PointTypePose trans2PointTypePose(float transformIn[]);
    bool saveMapService(const std::shared_ptr<liorf::srv::SaveMap::Request> req, std::shared_ptr<liorf::srv::SaveMap::Response> res);
    void visualizeGlobalMapThread();
    void publishGlobalMap();
    void addLoopFactor();
    void updateInitialGuess();
    void extractNearby();
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void extractSurroundingKeyFrames();
    void downsampleCurrentScan();
    void updatePointAssociateToMap();
    void surfOptimization();
    void combineOptimizationCoeffs();
    bool LMOptimization(int iterCount);
    void scan2MapOptimization();
    void transformUpdate();
    float constraintTransformation(float value, float limit);
    bool saveFrame();
    void addOdomFactor();
    void addGPSFactor();
    void saveKeyFramesAndFactor();
    void correctPoses();
    void updatePath(const PointTypePose& pose_in);
    void publishOdometry();
    void publishFrames();
    void updateMapSize(const pcl::PointCloud<PointType>::Ptr& cloud);
    void clearOldFrames();

    // 데이터베이스 관련 함수
    void initializeDBManager();
    void loadKeyFramesFromDB();
    void updateActiveWindow(const PointTypePose& current_pose);

    // --- Localization Mode를 위한 함수들 추가 ---
    void initializeForSLAM();
    void initializeForLocalization();
    void initialPoseHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void runSLAM();
    void runLocalization();
    void extractGlobalMapForLocalization();
    bool initializeLocalization();
    void resetOptimization();
    bool loadGlobalMap();
};

int main(int argc, char** argv); 