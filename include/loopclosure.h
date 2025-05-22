#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include "utility.h"
#include "Scancontext.h"
#include "costmap.h"

typedef pcl::PointXYZI PointType;
// utility.h에서 이미 PointTypePose가 정의되어 있음

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
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mutex>
#include <thread>
#include <atomic>
#include <deque>

using namespace gtsam;
using SCDetectionResult = std::pair<int, float>;

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

/**
 * Loop Closure 클래스
 * 
 * 이 클래스는 LIORF SLAM에서 루프 클로저를 담당합니다.
 * Scan Context 기반의 loop detection 및 ICP 정합을 수행합니다.
 */
class LoopClosure
{
public:
    /**
     * 생성자
     * @param node ROS2 노드 포인터
     * @param historyKeyframeSearchRadius 이전 키프레임 검색 반경
     * @param historyKeyframeSearchNum 검색할 이전 키프레임 수
     * @param historyKeyframeSearchTimeDiff 이전 키프레임 검색 시간 차이
     * @param historyKeyframeFitnessScore ICP 정합 적합도 점수 임계값
     */
    LoopClosure(rclcpp::Node* node,
               double historyKeyframeSearchRadius = 10.0,
               int historyKeyframeSearchNum = 25,
               double historyKeyframeSearchTimeDiff = 30.0,
               double historyKeyframeFitnessScore = 0.3);
    
    /**
     * 소멸자
     */
    ~LoopClosure();
    
    /**
     * 스레드 시작
     */
    void startThread();
    
    /**
     * 스레드 종료
     */
    void stopThread();
    
    /**
     * 입력 데이터 설정
     * @param keyPoses3D 3D 키포즈
     * @param keyPoses6D 6D 키포즈
     * @param surfClouds 표면 포인트 클라우드
     * @param currentTime 현재 타임스탬프
     * @param currentCloudRaw 현재 원시 포인트 클라우드
     */
    void setInputData(const pcl::PointCloud<PointType>::Ptr& keyPoses3D,
                     const pcl::PointCloud<PointTypePose>::Ptr& keyPoses6D,
                     const std::vector<pcl::PointCloud<PointType>::Ptr>& surfClouds,
                     double currentTime,
                     const pcl::PointCloud<PointType>::Ptr& currentCloudRaw = nullptr);
    
    /**
     * 루프 클로저 스레드 함수
     */
    void loopClosureThread();
    
    /**
     * Scan Context 기반 루프 클로저 수행
     */
    void performSCLoopClosure();
    
    /**
     * 루프가 닫혔는지 확인
     * @return 루프 닫힘 여부
     */
    bool isLoopClosed();
    
    /**
     * 루프 인덱스 큐 반환
     * @return 루프 인덱스 큐 (pair<int, int> 형태의 벡터)
     */
    const std::vector<std::pair<int, int>>& getLoopIndexQueue() const;
    
    /**
     * 루프 포즈 큐 반환
     * @return 루프 포즈 큐 (gtsam::Pose3 형태의 벡터)
     */
    const std::vector<gtsam::Pose3>& getLoopPoseQueue() const;
    
    /**
     * 루프 노이즈 큐 반환
     * @return 루프 노이즈 큐 (gtsam::noiseModel::Diagonal::shared_ptr 형태의 벡터)
     */
    const std::vector<gtsam::noiseModel::Diagonal::shared_ptr>& getLoopNoiseQueue() const;
    
    /**
     * 루프 큐 초기화
     */
    void clearLoopQueue();

private:
    // 좌표 변환 함수들
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
    
    // 노드 포인터
    rclcpp::Node* node_;
    
    // 루프 클로저 파라미터
    double historyKeyframeSearchRadius;
    int historyKeyframeSearchNum;
    double historyKeyframeSearchTimeDiff;
    double historyKeyframeFitnessScore;
    
    // Scan Context 관리자
    std::unique_ptr<SCManager> scManager;
    
    // 뮤텍스
    std::mutex mtx;
    std::mutex mtxLoopInfo;
    
    // 스레드 제어
    std::atomic<bool> isThreadRunning;
    bool loopClosureEnabled;
    
    // 루프 클로저 데이터
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    double timeLaserInfoCur;
    
    // 다운샘플링 필터
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    
    // 루프 인덱스 큐
    std::vector<std::pair<int, int>> loopIndexQueue;
    std::vector<gtsam::Pose3> loopPoseQueue;
    std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    
    // 처리 상태
    int lastProcessedPoseIdx = -1;
    bool icpClosed = false;
};

#endif // LOOPCLOSURE_H 