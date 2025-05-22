#include "loopclosure.h"
#include "utility.h"

LoopClosure::LoopClosure(rclcpp::Node* node,
                        double historyKeyframeSearchRadius,
                        int historyKeyframeSearchNum,
                        double historyKeyframeSearchTimeDiff,
                        double historyKeyframeFitnessScore)
    : node_(node),
      historyKeyframeSearchRadius(historyKeyframeSearchRadius),
      historyKeyframeSearchNum(historyKeyframeSearchNum),
      historyKeyframeSearchTimeDiff(historyKeyframeSearchTimeDiff),
      historyKeyframeFitnessScore(historyKeyframeFitnessScore),
      isThreadRunning(false)
{
    // Scan Context 초기화
    scManager = std::make_unique<SCManager>();
    
    // 루프 큐 초기화
    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    
    // ICP 설정
    icpClosed = false;
    downSizeFilterICP.setLeafSize(0.3, 0.3, 0.3);
    
    // 루프 클로저 스레드 생성 플래그
    loopClosureEnabled = node_->declare_parameter<bool>("loop_closure_enabled", true);
    
    // 로깅
    RCLCPP_INFO(node_->get_logger(), "Loop Closure Module Initialized. Enabled: %s", 
               loopClosureEnabled ? "true" : "false");
}

LoopClosure::~LoopClosure()
{
    stopThread();
}

// 스레드 시작
void LoopClosure::startThread()
{
    if (!isThreadRunning && loopClosureEnabled) {
        RCLCPP_INFO(node_->get_logger(), "Starting Loop Closure Thread");
        isThreadRunning = true;
    }
}

// 스레드 종료
void LoopClosure::stopThread()
{
    if (isThreadRunning) {
        RCLCPP_INFO(node_->get_logger(), "Stopping Loop Closure Thread");
        isThreadRunning = false;
    }
}

void LoopClosure::setInputData(const pcl::PointCloud<PointType>::Ptr& keyPoses3D,
                              const pcl::PointCloud<PointTypePose>::Ptr& keyPoses6D,
                              const std::vector<pcl::PointCloud<PointType>::Ptr>& surfClouds,
                              double currentTime,
                              const pcl::PointCloud<PointType>::Ptr& currentCloudRaw)
{
    std::lock_guard<std::mutex> lock(mtx);
    
    cloudKeyPoses3D = keyPoses3D;
    cloudKeyPoses6D = keyPoses6D;
    surfCloudKeyFrames = surfClouds;
    timeLaserInfoCur = currentTime;
    
    // 새로운 키프레임이 추가되면 Scan Context 생성
    if (cloudKeyPoses3D->size() > 0 && currentCloudRaw && !currentCloudRaw->empty()) {
        int currentPoseIdx = cloudKeyPoses3D->size() - 1;
        
        // 이미 처리된 데이터인지 확인
        if (lastProcessedPoseIdx < currentPoseIdx) {
            // Scan Context 저장
            scManager->makeAndSaveScancontextAndKeys(*currentCloudRaw);
            lastProcessedPoseIdx = currentPoseIdx;
        }
    }
}

void LoopClosure::loopClosureThread()
{
    RCLCPP_INFO(node_->get_logger(), "Loop Closure Thread Started");
    
    while (rclcpp::ok()) {
        // 스레드 중지 신호 확인
        if (!isThreadRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // 루프 클로저 수행
        if (cloudKeyPoses3D && cloudKeyPoses6D && !cloudKeyPoses3D->empty()) {
            performSCLoopClosure();
            
            // ICP가 닫혔는지 확인하고 루프 요소 추가
            if (icpClosed) {
                icpClosed = false;
            }
        }
        
        // 스레드 리소스 절약
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void LoopClosure::performSCLoopClosure()
{
    std::lock_guard<std::mutex> lock(mtx);
    
    if (cloudKeyPoses3D->empty() || scManager->polarcontexts_.size() < 10) {
        return;
    }
    
    // 최신 키프레임의 인덱스
    int latestFrameIDX = cloudKeyPoses3D->size() - 1;
    
    // 최신 키프레임의 포즈
    auto& latestPose = cloudKeyPoses6D->points[latestFrameIDX];
    
    // 자기 자신과 매칭되는 것 방지를 위한 시간 차이 확인
    if (timeLaserInfoCur - latestPose.time < historyKeyframeSearchTimeDiff) {
        return;
    }
    
    // SC Loop 감지
    SCDetectionResult detectResult = scManager->detectLoopClosureID();
    int loopKeyCur = detectResult.first;
    int loopKeyPre = detectResult.second;
    float yawDiffRad = detectResult.second; // v1에서는 사용하지 않음
    
    if (loopKeyCur == -1) {
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Loop detected between %d and %d", latestFrameIDX, loopKeyPre);
    
    // 검출된 루프 위치에서 ICP 정합 수행
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    
    // 현재 프레임 클라우드
    *cureKeyframeCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDX], &cloudKeyPoses6D->points[latestFrameIDX]);
    
    // 루프 대상 프레임 클라우드
    *prevKeyframeCloud += *transformPointCloud(surfCloudKeyFrames[loopKeyPre], &cloudKeyPoses6D->points[loopKeyPre]);
    
    // ICP 정합
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    
    // 다운샘플링
    pcl::PointCloud<PointType>::Ptr cureKeyframeDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeDS(new pcl::PointCloud<PointType>());
    
    downSizeFilterICP.setInputCloud(cureKeyframeCloud);
    downSizeFilterICP.filter(*cureKeyframeDS);
    
    downSizeFilterICP.setInputCloud(prevKeyframeCloud);
    downSizeFilterICP.filter(*prevKeyframeDS);
    
    // ICP 설정 및 정합
    icp.setInputSource(cureKeyframeDS);
    icp.setInputTarget(prevKeyframeDS);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
    
    // ICP 정합 결과 확인
    if (icp.hasConverged() && icp.getFitnessScore() < historyKeyframeFitnessScore) {
        RCLCPP_INFO(node_->get_logger(), "ICP fitness score: %f", icp.getFitnessScore());
        
        // 상대 변환 행렬
        Eigen::Matrix4f correctionLidarFrame = icp.getFinalTransformation();
        
        // 루프 제약 추가
        Eigen::Affine3f tWrong = pclPointToAffine3f(cloudKeyPoses6D->points[latestFrameIDX]);
        Eigen::Affine3f tCorrect = pclPointToAffine3f(cloudKeyPoses6D->points[loopKeyPre]);
        Eigen::Matrix4f poseFinal = tCorrect.matrix() * correctionLidarFrame * tWrong.matrix().inverse();
        
        Eigen::Affine3f transFinal(poseFinal);
        
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transFinal, x, y, z, roll, pitch, yaw);
        
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
        
        // Loop Closure 노이즈 설정
        gtsam::Vector Vector6(6);
        Vector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);
        
        // 루프 큐에 추가
        mtxLoopInfo.lock();
        loopIndexQueue.push_back(std::make_pair(loopKeyPre, latestFrameIDX));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtxLoopInfo.unlock();
        
        // 상태 설정
        icpClosed = true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "ICP failed, score: %f > %f", icp.getFitnessScore(), historyKeyframeFitnessScore);
    }
}

// 루프 폐쇄 확인
bool LoopClosure::isLoopClosed()
{
    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    return !loopIndexQueue.empty();
}

// 루프 큐 반환
const std::vector<std::pair<int, int>>& LoopClosure::getLoopIndexQueue() const
{
    return loopIndexQueue;
}

const std::vector<gtsam::Pose3>& LoopClosure::getLoopPoseQueue() const
{
    return loopPoseQueue;
}

const std::vector<gtsam::noiseModel::Diagonal::shared_ptr>& LoopClosure::getLoopNoiseQueue() const
{
    return loopNoiseQueue;
}

// 루프 큐 지우기
void LoopClosure::clearLoopQueue()
{
    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
}

// 좌표 변환 함수
Eigen::Affine3f LoopClosure::pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

pcl::PointCloud<PointType>::Ptr LoopClosure::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
} 