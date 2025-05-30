#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include "utility.h"
#include "Scancontext.h"
#include "costmap.h"
#include "db_manager.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace gtsam;

using PointType = pcl::PointXYZI;

// costmap.h에서 PointXYZIRPYT를 이미 정의했으므로 여기서는 정의하지 않음

// SCInputType 또한 중복으로 정의하지 않고 같은 값을 사용
// enum class SCInputType
// {
//     SINGLE_SCAN_FULL, // original SC-LIO-SAM uses this (radius filter -> registration)
//     SINGLE_SCAN_FEAT, // SC-A-LOAM uses this (feature extraction -> radius filter -> registration)
//     MULTI_SCAN_FEAT, // use in new ver. for better performance, recommended. (feature extraction -> voxel filter -> radius filter -> feature to map registration)
// };

using SCInputType = enum class SCInputType;

// forward declaration
void PublishCloudMsg(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
                    const pcl::PointCloud<PointType>& cloud,
                    const rclcpp::Time& stamp,
                    const std::string& frame_id);

class LoopClosure
{
public:
    // 수정된 생성자 - DB 관리자 추가
    LoopClosure(rclcpp::Node* node, 
                double historyKeyframeSearchRadius,
                int historyKeyframeSearchNum,
                double historyKeyframeSearchTimeDiff,
                double historyKeyframeFitnessScore,
                DBManager* db_manager = nullptr);
    ~LoopClosure();
    
    // 루프 클로저 스레드 함수
    void loopClosureThread();
    
    // 데이터 설정 함수 - DB 모드 지원 추가
    void setInputData(pcl::PointCloud<PointType>::Ptr& keyPoses3D, 
                     pcl::PointCloud<PointTypePose>::Ptr& keyPoses6D,
                     std::vector<pcl::PointCloud<PointType>::Ptr>& surfKeyFrames,
                     double currentTimestamp,
                     pcl::PointCloud<PointType>::Ptr currentScan = nullptr);
    
    // DB 모드용 데이터 설정 함수 추가
    void setInputDataWithDB(pcl::PointCloud<PointType>::Ptr& keyPoses3D, 
                           pcl::PointCloud<PointTypePose>::Ptr& keyPoses6D, 
                     double currentTimestamp,
                     pcl::PointCloud<PointType>::Ptr currentScan = nullptr);
    
    // 루프 감지 및 처리
    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg);
    void performRSLoopClosure();
    void performSCLoopClosure();
    bool detectLoopClosureDistance(int *latestID, int *closestID);
    bool detectLoopClosureExternal(int *latestID, int *closestID);
    void loopFindNearKeyFrames(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int& loop_index);
    void visualizeLoopClosure();
    
    // 유틸리티 함수
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
    
    // Getter 함수들
    bool isLoopClosed() const { return aLoopIsClosed; }
    const std::vector<std::pair<int, int>>& getLoopIndexQueue() const { return loopIndexQueue; }
    const std::vector<gtsam::Pose3>& getLoopPoseQueue() const { return loopPoseQueue; }
    const std::vector<gtsam::SharedNoiseModel>& getLoopNoiseQueue() const { return loopNoiseQueue; }

    // 메모리 관리 함수 추가
    void clearTemporaryCache();
    
    // DB 모드 기능 추가
    void saveLoopFeatureToDB(int feature_id, pcl::PointCloud<PointType>::Ptr cloud);
    pcl::PointCloud<PointType>::Ptr loadLoopFeatureFromDB(int feature_id);
    void updateActiveLoopFeatureWindow(const PointTypePose& current_pose);
    
    // DB 설정 함수
    void setDBMode(bool use_db) { use_db_mode_ = use_db; }
    bool isUsingDBMode() const { return use_db_mode_; }

private:
    rclcpp::Node* node_;
    std::unique_ptr<SCManager> sc_manager_; // Scan Context 관리자
    DBManager* db_manager_; // DB 관리자 참조 추가 (소유권 없음)
    bool use_db_mode_; // DB 모드 사용 여부
    
    // Loop Closure 관련 변수
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    bool aLoopIsClosed;
    bool isRunning;
    double timeLaserInfoCur;
    
    // Loop 관련 변수
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;
    std::deque<std_msgs::msg::Float64MultiArray> loopInfoVec;
    std::map<int, int> loopIndexContainer; // from new to old
    std::vector<std::pair<int, int>> loopIndexQueue;
    std::vector<gtsam::Pose3> loopPoseQueue;
    std::vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    
    // 다운샘플링 필터
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    
    // 파라미터
    double historyKeyframeSearchRadius;
    int historyKeyframeSearchNum;
    double historyKeyframeSearchTimeDiff;
    double historyKeyframeFitnessScore;
    
    // 뮤텍스
    std::mutex mtx;
    std::mutex mtxLoopInfo;
    
    // 스레드 종료 플래그
    bool isThreadRunning;
    
    // 구독
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;
    
    // 발행
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    
    // DB 모드용 헬퍼 함수
    pcl::PointCloud<PointType>::Ptr getKeyFrameFromDB(int keyframe_id);
    
    // 임시 클라우드 캐시 추가 - 루프 클로저 작업 중에만 사용
    std::unordered_map<int, pcl::PointCloud<PointType>::Ptr> tempCloudCache;
};

#endif // LOOPCLOSURE_H 