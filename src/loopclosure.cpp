#include "loopclosure.h"
#include "utility.h"

LoopClosure::LoopClosure(rclcpp::Node* node, 
                        double historyKeyframeSearchRadius,
                        int historyKeyframeSearchNum,
                        double historyKeyframeSearchTimeDiff,
                        double historyKeyframeFitnessScore,
                        DBManager* db_manager) 
    : node_(node),
    aLoopIsClosed(false),
    isRunning(false),
    isThreadRunning(true),
    historyKeyframeSearchRadius(historyKeyframeSearchRadius),
    historyKeyframeSearchNum(historyKeyframeSearchNum),
    historyKeyframeSearchTimeDiff(historyKeyframeSearchTimeDiff),
    historyKeyframeFitnessScore(historyKeyframeFitnessScore),
    db_manager_(db_manager),
    use_db_mode_(db_manager != nullptr)
{
    // 파라미터 로깅
    RCLCPP_INFO(node_->get_logger(), "Loop closure parameters: historyKeyframeSearchRadius=%f, historyKeyframeSearchNum=%d, historyKeyframeSearchTimeDiff=%f, historyKeyframeFitnessScore=%f", 
        historyKeyframeSearchRadius, historyKeyframeSearchNum, historyKeyframeSearchTimeDiff, historyKeyframeFitnessScore);
    
    if (use_db_mode_) {
        RCLCPP_INFO(node_->get_logger(), "Loop closure is using DB mode");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Loop closure is using memory-only mode");
    }

    // 포인트 클라우드 초기화
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    // ICP 다운샘플링 필터 설정
    downSizeFilterICP.setLeafSize(0.3, 0.3, 0.3);

    // SC 관리자 초기화
    try {
        sc_manager_ = std::make_unique<SCManager>();
        RCLCPP_INFO(node_->get_logger(), "SC Manager initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize SC Manager: %s", e.what());
    }

    // 구독/발행자 설정
    subLoop = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "lio_loop/loop_closure_detection", 
        rclcpp::QoS(100),
        std::bind(&LoopClosure::loopInfoHandler, this, std::placeholders::_1));

    pubHistoryKeyFrames = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "liorf/mapping/icp_loop_closure_history_cloud", 
        rclcpp::QoS(100));

    pubIcpKeyFrames = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "liorf/mapping/icp_loop_closure_corrected_cloud", 
        rclcpp::QoS(100));

    pubLoopConstraintEdge = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/liorf/mapping/loop_closure_constraints", 
        rclcpp::QoS(100));

    RCLCPP_INFO(node_->get_logger(), "Loop closure module initialized");
}

LoopClosure::~LoopClosure()
{
    isThreadRunning = false;
}

void LoopClosure::loopClosureThread()
{
    rclcpp::Rate rate(2);  // 2Hz로 실행
    
    while (rclcpp::ok() && isThreadRunning)
    {
        rate.sleep();
        
        performRSLoopClosure();
        performSCLoopClosure();
        visualizeLoopClosure();
        
        // 임시 캐시 정리
        clearTemporaryCache();
        
        // Clear loop flag
        aLoopIsClosed = false;
    }
}

// DB 모드에서 키프레임 가져오기
pcl::PointCloud<PointType>::Ptr LoopClosure::getKeyFrameFromDB(int keyframe_id) {
    if (!use_db_mode_ || !db_manager_ || !db_manager_->isInitialized()) {
        RCLCPP_ERROR(node_->get_logger(), "DB 모드가 아니거나 DB 관리자가 초기화되지 않았습니다");
        return nullptr;
    }
    
    // 임시 캐시에 이미 있는지 확인
    auto cache_it = tempCloudCache.find(keyframe_id);
    if (cache_it != tempCloudCache.end()) {
        return cache_it->second;
    }
    
    // DB에서 로드 (디버그 레벨로 변경)
    RCLCPP_DEBUG(node_->get_logger(), "루프 클로저: DB에서 키프레임 ID=%d 로드", keyframe_id);
    pcl::PointCloud<PointType>::Ptr cloud = db_manager_->loadCloud(keyframe_id);
    if (!cloud || cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 ID %d를 DB에서 로드하지 못했습니다", keyframe_id);
        return nullptr;
    }
    
    // 임시 캐시에 저장
    tempCloudCache[keyframe_id] = cloud;
    
    return cloud;
}

// 임시 캐시 정리 함수
void LoopClosure::clearTemporaryCache() {
    std::lock_guard<std::mutex> lock(mtx);
    if (!tempCloudCache.empty()) {
        size_t count = tempCloudCache.size();
        tempCloudCache.clear();
        // RCLCPP_INFO(node_->get_logger(), "★★★ 루프 클로저 임시 캐시 정리: %zu 키프레임 해제됨 ★★★", count);
    }
}

// 기존 메모리 모드 setInputData
void LoopClosure::setInputData(pcl::PointCloud<PointType>::Ptr& keyPoses3D, 
                              pcl::PointCloud<PointTypePose>::Ptr& keyPoses6D,
                              std::vector<pcl::PointCloud<PointType>::Ptr>& surfKeyFrames,
                              double currentTimestamp,
                              pcl::PointCloud<PointType>::Ptr currentScan)
{
    std::lock_guard<std::mutex> lock(mtx);

    // 데이터 복사
    *cloudKeyPoses3D = *keyPoses3D;
    *cloudKeyPoses6D = *keyPoses6D;
    
    // DB 모드가 아닐 때만 서페이스 클라우드 키프레임 복사
    if (!use_db_mode_) {
        surfCloudKeyFrames = surfKeyFrames;
    }
    
    timeLaserInfoCur = currentTimestamp;
    
    // Scan Context 생성
    if (sc_manager_ && currentScan) {
        try {
            // Scan Context 생성 - SINGLE_SCAN_FULL 방식 사용
            sc_manager_->makeAndSaveScancontextAndKeys(*currentScan);
            
            // 디버그 메시지
            RCLCPP_DEBUG(node_->get_logger(), "SC Manager processed current scan successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Error in SC Manager while processing scan: %s", e.what());
        }
    }
}

// DB 모드용 setInputData - surfCloudKeyFrames 참조 없이 DB에서 직접 로드
void LoopClosure::setInputDataWithDB(pcl::PointCloud<PointType>::Ptr& keyPoses3D, 
                                    pcl::PointCloud<PointTypePose>::Ptr& keyPoses6D, 
                                    double currentTimestamp,
                                    pcl::PointCloud<PointType>::Ptr currentScan)
{
    std::lock_guard<std::mutex> lock(mtx);

    if (!use_db_mode_ || !db_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "DB 모드가 아니거나 DB 관리자가 설정되지 않았습니다");
        return;
    }

    // 포즈 데이터 복사
    *cloudKeyPoses3D = *keyPoses3D;
    *cloudKeyPoses6D = *keyPoses6D;
    timeLaserInfoCur = currentTimestamp;
    
    // surfCloudKeyFrames는 필요할 때마다 DB에서 로드하므로 여기서는 복사하지 않음
    
    // Scan Context 처리
    if (sc_manager_ && currentScan) {
        try {
            sc_manager_->makeAndSaveScancontextAndKeys(*currentScan);
            RCLCPP_DEBUG(node_->get_logger(), "SC Manager processed current scan successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Error in SC Manager while processing scan: %s", e.what());
        }
    }
}

void LoopClosure::loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg)
{
    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopMsg->data.size() != 2)
        return;

    loopInfoVec.push_back(*loopMsg);

    while (loopInfoVec.size() > 5)
        loopInfoVec.pop_front();
}

void LoopClosure::performRSLoopClosure()
{
    std::lock_guard<std::mutex> lock(mtx);
    
    // 이미 루프 클로저가 발생했으면 이번 호출에서는 처리하지 않음
    if (aLoopIsClosed) {
        return;
    }
    
    if (cloudKeyPoses3D->points.empty())
        return;

    // find keys
    int loopKeyCur;
    int loopKeyPre;
    if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;

    // 루프 클로저 후보 발견 - 로그 출력
    RCLCPP_DEBUG(node_->get_logger(), "루프 클로저 매칭 시도: 현재=%d, 과거=%d, 간격=%d", 
                loopKeyCur, loopKeyPre, std::abs(loopKeyCur - loopKeyPre));

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    {
        // DB 모드에 따라 다른 방식으로 키프레임 로드
        if (use_db_mode_ && db_manager_) {
            // 현재 키프레임 로드
            auto curCloud = getKeyFrameFromDB(loopKeyCur);
            if (curCloud) {
                loopFindNearKeyFrames(cureKeyframeCloud, loopKeyCur, 0, -1);
            } else {
                RCLCPP_WARN(node_->get_logger(), "현재 키프레임 로드 실패: ID=%d", loopKeyCur);
                return;
            }
            
            // 이전 키프레임 로드
            auto prevCloud = getKeyFrameFromDB(loopKeyPre);
            if (prevCloud) {
                loopFindNearKeyFrames(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
            } else {
                RCLCPP_WARN(node_->get_logger(), "과거 키프레임 로드 실패: ID=%d", loopKeyPre);
                return;
            }
        } else {
            // 기존 메모리 방식
            loopFindNearKeyFrames(cureKeyframeCloud, loopKeyCur, 0, -1);
            loopFindNearKeyFrames(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
        }
        
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000) {
            RCLCPP_WARN(node_->get_logger(), "포인트 수 부족 - 현재: %zu, 과거: %zu", 
                       cureKeyframeCloud->size(), prevKeyframeCloud->size());
            return;
        }
        
        if (pubHistoryKeyFrames->get_subscription_count() != 0)
            PublishCloudMsg(pubHistoryKeyFrames, *prevKeyframeCloud, node_->now(), "odom");
    }

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
    //     RCLCPP_WARN(node_->get_logger(), "ICP 매칭 실패 - 수렴: %s, 점수: %.2f", 
    //                icp.hasConverged() ? "성공" : "실패", icp.getFitnessScore());
    //     return;
    // }

    // RCLCPP_INFO(node_->get_logger(), "ICP 매칭 성공 - 점수: %.4f (현재=%d, 과거=%d)", 
    //            icp.getFitnessScore(), loopKeyCur, loopKeyPre);

    // publish corrected cloud
    if (pubIcpKeyFrames->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        PublishCloudMsg(pubIcpKeyFrames, *closed_cloud, node_->now(), "odom");
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong = pclPointToAffine3f(cloudKeyPoses6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    
    // DB 모드에서는 루프 특징점을 DB에 저장
    if (use_db_mode_ && db_manager_ && db_manager_->isInitialized()) {
        // 현재 키프레임 특징점 저장
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 저장: 현재 ID=%d", loopKeyCur);
        saveLoopFeatureToDB(loopKeyCur, cureKeyframeCloud);
        
        // 매칭된 이전 키프레임 특징점 저장
        // prevKeyframeCloud는 이미 여러 키프레임이 합쳐진 상태이므로 개별 키프레임 특징점만 저장
        auto singlePrevCloud = getKeyFrameFromDB(loopKeyPre);
        if (singlePrevCloud && !singlePrevCloud->empty()) {
            RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 저장: 과거 ID=%d", loopKeyPre);
            saveLoopFeatureToDB(loopKeyPre, singlePrevCloud);
        }
        
        // 루프 특징점 활성 윈도우 업데이트
        PointTypePose currentPose = cloudKeyPoses6D->points[loopKeyCur];
        updateActiveLoopFeatureWindow(currentPose);
    }

    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    auto robustConstraintNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), 
        gtsam::noiseModel::Diagonal::Variances(Vector6)
    );
    
    // Add this constraint to graph optimization
    mtxLoopInfo.lock();
    loopIndexQueue.push_back(std::make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(robustConstraintNoise);
    mtxLoopInfo.unlock();

    // 루프 클로저 성공 설정 및 컨테이너에 추가
    aLoopIsClosed = true;
    loopIndexContainer[loopKeyCur] = loopKeyPre;
    
    RCLCPP_DEBUG(node_->get_logger(), "루프 클로저 성공 - 현재=%d, 과거=%d, 거리=%.2f m, 간격=%d", 
               loopKeyCur, loopKeyPre, 
               sqrt(pow(cloudKeyPoses3D->points[loopKeyCur].x - cloudKeyPoses3D->points[loopKeyPre].x, 2) +
                    pow(cloudKeyPoses3D->points[loopKeyCur].y - cloudKeyPoses3D->points[loopKeyPre].y, 2)),
               std::abs(loopKeyCur - loopKeyPre));
}

void LoopClosure::performSCLoopClosure()
{
    std::lock_guard<std::mutex> lock(mtx);
    
    // 이미 RS 루프 클로저가 발생했으면 이번 호출에서는 SC 루프 클로저 처리하지 않음
    if (aLoopIsClosed) {
        return;
    }
    
    if (cloudKeyPoses3D->points.empty())
        return;

    // find keys
    // first: nn index, second: yaw diff 
    auto detectResult = sc_manager_->detectLoopClosureID(); 
    int loopKeyCur = cloudKeyPoses3D->size() - 1;
    int loopKeyPre = detectResult.first;
    float yawDiffRad = detectResult.second; // v1에서는 사용하지 않음
    
    if (loopKeyPre == -1)
        return;

    // 이미 이 키프레임에 대한 루프 클로저가 등록되어 있는지 확인
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return;
    
    // 너무 가까운 키프레임은 루프 클로저로 간주하지 않음
    if (std::abs(loopKeyCur - loopKeyPre) < 10) {
        return;
    }
    
    // 루프 클로저 후보 발견 - 로그 출력
    RCLCPP_DEBUG(node_->get_logger(), "SC 루프 클로저 후보 발견: 현재=%d, 과거=%d, 간격=%d", 
                loopKeyCur, loopKeyPre, std::abs(loopKeyCur - loopKeyPre));

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    {
        int base_key = 0;
        loopFindNearKeyFrames(cureKeyframeCloud, loopKeyCur, 0, base_key);
        loopFindNearKeyFrames(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key);

        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000) {
            RCLCPP_WARN(node_->get_logger(), "SC 포인트 수 부족 - 현재: %zu, 과거: %zu", 
                       cureKeyframeCloud->size(), prevKeyframeCloud->size());
            return;
        }
        
        if (pubHistoryKeyFrames->get_subscription_count() != 0)
            PublishCloudMsg(pubHistoryKeyFrames, *prevKeyframeCloud, node_->now(), "odom");
    }

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
        RCLCPP_WARN(node_->get_logger(), "SC ICP 매칭 실패 - 수렴: %s, 점수: %.2f", 
                   icp.hasConverged() ? "성공" : "실패", icp.getFitnessScore());
        return;
    }
    
    // RCLCPP_INFO(node_->get_logger(), "SC ICP 매칭 성공 - 점수: %.4f (현재=%d, 과거=%d)", 
    //            icp.getFitnessScore(), loopKeyCur, loopKeyPre);

    // publish corrected cloud
    if (pubIcpKeyFrames->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        PublishCloudMsg(pubIcpKeyFrames, *closed_cloud, node_->now(), "odom");
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // giseop 
    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    // giseop, robust kernel for a SC loop
    float robustNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); 
    robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
    noiseModel::Base::shared_ptr robustConstraintNoise; 
    robustConstraintNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
    ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

    // Add pose constraint
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(robustConstraintNoise);

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
    
    // Set flag
    aLoopIsClosed = true;
    
    // 루프 클로저 성공 로그
    RCLCPP_DEBUG(node_->get_logger(), "SC 루프 클로저 성공 - 현재=%d, 과거=%d, 거리=%.2f m, 간격=%d", 
               loopKeyCur, loopKeyPre, 
               sqrt(pow(cloudKeyPoses3D->points[loopKeyCur].x - cloudKeyPoses3D->points[loopKeyPre].x, 2) +
                    pow(cloudKeyPoses3D->points[loopKeyCur].y - cloudKeyPoses3D->points[loopKeyPre].y, 2)),
               std::abs(loopKeyCur - loopKeyPre));
}

bool LoopClosure::detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // 이미 루프 클로저가 적용된 키프레임은 처리하지 않음
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    // DB 모드와 메모리 모드에 따라 다른 방식으로 처리
    if (use_db_mode_ && db_manager_ && db_manager_->isInitialized()) {
        // DB 모드에서는 공간 검색 쿼리 사용
        PointTypePose currentPose = cloudKeyPoses6D->points[loopKeyCur];
        
        // 원본 알고리즘과 동일하게 historyKeyframeSearchNum 파라미터 사용
        std::vector<int> candidateKeyframes = db_manager_->loadKeyFramesByRadius(
            currentPose, 
            historyKeyframeSearchRadius, 
            historyKeyframeSearchNum  // 원본 로직과 동일하게 historyKeyframeSearchNum 사용
        );
        
        // 쿼리 결과 로그
        RCLCPP_DEBUG(node_->get_logger(), "루프 클로저 검색: %zu개의 후보 발견 (최대: %d)",
                    candidateKeyframes.size(), historyKeyframeSearchNum);
        
        // 키프레임 인덱스 기준으로 정렬 - 가장 가까운 것부터
        std::sort(candidateKeyframes.begin(), candidateKeyframes.end(), 
            [loopKeyCur](int a, int b) {
                return std::abs(a - loopKeyCur) < std::abs(b - loopKeyCur);
            });
        
        // 원본 알고리즘과 동일하게 시간 차이가 충분히 큰 첫 번째 후보만 선택
        for (int i = 0; i < (int)candidateKeyframes.size(); ++i) {
            int id = candidateKeyframes[i];
            // 자기 자신과는 루프 클로저 하지 않음
            if (id == loopKeyCur) 
                continue;
            
            // 키프레임 간의 거리가 일정 이상이어야 함 (너무 가까운 키프레임은 제외)
            // 최소 10개 이상의 키프레임 차이가 있어야 루프 클로저로 처리
            if (std::abs(id - loopKeyCur) < 10)
                continue;
                
            // 시간 차이가 충분히 커야만 루프 클로저로 간주
            if (abs(cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff) {
                loopKeyPre = id;
                break; // 첫 번째 적합한 후보를 찾으면 중단
            }
        }
    } else {
        // 원래 메모리 모드: KDTree 기반 검색
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        // 검색 결과 정렬 - 거리 기준
        std::vector<std::pair<int, float>> candidates;
        for (size_t i = 0; i < pointSearchIndLoop.size(); i++) {
            candidates.push_back(std::make_pair(pointSearchIndLoop[i], pointSearchSqDisLoop[i]));
        }
        
        std::sort(candidates.begin(), candidates.end(), 
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second < b.second;  // 거리 기준 오름차순 정렬
            });
        
        // 원본 로직과 동일하게 시간 차이가 충분히 큰 첫 번째 후보만 선택
        for (const auto& candidate : candidates) {
            int id = candidate.first;
            
            // 자기 자신과는 루프 클로저 하지 않음
            if (id == loopKeyCur) 
                continue;
            
            // 키프레임 간의 거리가 일정 이상이어야 함 (너무 가까운 키프레임은 제외)
            if (std::abs(id - loopKeyCur) < 10)
                continue;
                
            // 시간 차이가 충분히 커야만 루프 클로저로 간주
            if (abs(cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff) {
                loopKeyPre = id;
                break; // 첫 번째 적합한 후보를 찾으면 중단
            }
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
        
    RCLCPP_DEBUG(node_->get_logger(), "루프 클로저 후보 감지: 현재=%d, 과거=%d (간격: %d)", 
               loopKeyCur, loopKeyPre, std::abs(loopKeyCur - loopKeyPre));

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

bool LoopClosure::detectLoopClosureExternal(int *latestID, int *closestID)
{
    // this function is not used yet, please ignore it
    int loopKeyCur = -1;
    int loopKeyPre = -1;

    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopInfoVec.empty())
        return false;

    double loopTimeCur = loopInfoVec.front().data[0];
    double loopTimePre = loopInfoVec.front().data[1];
    loopInfoVec.pop_front();

    if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
        return false;

    int cloudSize = cloudKeyPoses6D->size();
    if (cloudSize < 2)
        return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i)
    {
        if (cloudKeyPoses6D->points[i].time >= loopTimeCur)
            loopKeyCur = round(cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i)
    {
        if (cloudKeyPoses6D->points[i].time <= loopTimePre)
            loopKeyPre = round(cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    if (loopKeyCur == loopKeyPre)
        return false;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

void LoopClosure::loopFindNearKeyFrames(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int& loop_index)
{
    // 대상 클라우드 초기화
    nearKeyframes->clear();
    
    // 검색 할 키프레임 ID 및 개수 설정
    int cloudSize = cloudKeyPoses6D->size();
    
    // loop_index가 -1이면 자체 검색, 그렇지 않으면 외부 검출된 루프 인덱스 사용
    int start_idx = key;
    int search_count = searchNum;
    
    // 루프 인덱스 유효성 검사
    if (loop_index >= 0 && loop_index < cloudSize) {
        // 해당 루프 인덱스를 사용하여 검색 시작점 설정
        start_idx = loop_index;
        // 자기 자신(1개)만 사용
        search_count = 1;
    }
    
    // 키프레임 ID 유효성 검사
    if (key < 0 || key >= cloudSize) {
        RCLCPP_ERROR(node_->get_logger(), "유효하지 않은 키프레임 ID: %d (총 포즈 수: %d)", key, cloudSize);
        return;
    }
    
    // 타겟 키프레임 설정
    PointTypePose targetPose = cloudKeyPoses6D->points[start_idx];
    
    // DB 모드인 경우와 아닌 경우에 따라 다르게 처리
    if (use_db_mode_ && db_manager_ && db_manager_->isInitialized()) {
        // DB 모드: 데이터베이스에서 키프레임 로드
        
        // 자기 자신인 경우: 직접 로드
        if (search_count == 0 || search_count == 1) {
            pcl::PointCloud<PointType>::Ptr cloud;
            
            // 루프 클로저 특징점으로 먼저 시도
            cloud = loadLoopFeatureFromDB(start_idx);
            
            // 없으면 일반 키프레임 로드 시도
            if (!cloud || cloud->empty()) {
                cloud = db_manager_->loadCloud(start_idx);
            }
            
            if (cloud && !cloud->empty()) {
                *nearKeyframes += *cloud;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "키프레임 ID %d: DB에서 로드 실패", start_idx);
            }
            
            return;
        }
        
        // 검색 범위 내 키프레임 추가
        std::vector<int> keyframe_ids;
        
        // 반경 내 키프레임 ID 로드
        if (loop_index < 0) {
            // 반경 기반 검색
            keyframe_ids = db_manager_->loadKeyFramesByRadius(targetPose, historyKeyframeSearchRadius, search_count);
        } else {
            // 시간 차이 기반 검색
            // 데이터베이스 검색 쿼리 생성 (복잡하므로 간단하게 구현)
            int startID = std::max(0, start_idx - search_count / 2);
            int endID = std::min(cloudSize - 1, start_idx + search_count / 2);
            
            for (int i = startID; i <= endID; i++) {
                keyframe_ids.push_back(i);
            }
        }
        
        // 각 키프레임 로드 및 합치기
        for (int id : keyframe_ids) {
            if (id == start_idx) continue; // 시작점은 이미 추가했으므로 스킵
            
            pcl::PointCloud<PointType>::Ptr cloud;
            
            // 루프 클로저 특징점으로 먼저 시도
            cloud = loadLoopFeatureFromDB(id);
            
            // 없으면 일반 키프레임 로드 시도
            if (!cloud || cloud->empty()) {
                cloud = db_manager_->loadCloud(id);
            }
            
            if (cloud && !cloud->empty()) {
                *nearKeyframes += *cloud;
            }
        }
    } else {
        // 메모리 모드: 메모리에 있는 키프레임 사용
        
        // 벡터 유효성 검사
        if (surfCloudKeyFrames.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "빈 서페이스 클라우드 키프레임 벡터");
            return;
        }
        
        // 자기 자신인 경우: 해당 키프레임만 사용
        if (search_count == 0 || search_count == 1) {
            if (start_idx < static_cast<int>(surfCloudKeyFrames.size()) && surfCloudKeyFrames[start_idx]) {
                *nearKeyframes += *surfCloudKeyFrames[start_idx];
            } else {
                RCLCPP_ERROR(node_->get_logger(), "키프레임 ID %d: 메모리에서 로드 실패", start_idx);
            }
            return;
        }
        
        // 검색 범위 내 키프레임 추가
        for (int i = -search_count / 2; i <= search_count / 2; ++i) {
            int keyNear = start_idx + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;
                
            if (keyNear < static_cast<int>(surfCloudKeyFrames.size()) && surfCloudKeyFrames[keyNear]) {
                *nearKeyframes += *surfCloudKeyFrames[keyNear];
            }
        }
    }
    
    // 다운샘플링
    if (nearKeyframes->points.size() > 1000) {
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }
}

void LoopClosure::visualizeLoopClosure()
{
    std::lock_guard<std::mutex> lock(mtx);
    
    if (loopIndexContainer.empty())
        return;
    
    visualization_msgs::msg::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = "odom";
    markerNode.header.stamp = node_->now();
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = "odom";
    markerEdge.header.stamp = node_->now();
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::msg::Point p;
        p.x = cloudKeyPoses6D->points[key_cur].x;
        p.y = cloudKeyPoses6D->points[key_cur].y;
        p.z = cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = cloudKeyPoses6D->points[key_pre].x;
        p.y = cloudKeyPoses6D->points[key_pre].y;
        p.z = cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge->publish(markerArray);
}

Eigen::Affine3f LoopClosure::pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

gtsam::Pose3 LoopClosure::pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

void PublishCloudMsg(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
                    const pcl::PointCloud<PointType>& cloud,
                    const rclcpp::Time& stamp,
                    const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = frame_id;
    publisher->publish(cloud_msg);
}

// DB 관련 기능 추가
void LoopClosure::saveLoopFeatureToDB(int feature_id, pcl::PointCloud<PointType>::Ptr cloud) {
    if (!use_db_mode_ || !db_manager_ || !db_manager_->isInitialized()) {
        RCLCPP_ERROR(node_->get_logger(), "DB 모드가 아니거나 DB 관리자가 초기화되지 않았습니다");
        return;
    }

    if (feature_id < 0 || !cloud || cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "유효하지 않은 특징점 또는 빈 클라우드: ID=%d", feature_id);
        return;
    }

    // 키프레임 포즈 가져오기
    if (feature_id >= cloudKeyPoses6D->size()) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 ID(%d)가 범위를 벗어남: 크기=%zu", 
                    feature_id, cloudKeyPoses6D->size());
        return;
    }
    
    // 키프레임 포즈 가져오기
    PointTypePose pose = cloudKeyPoses6D->points[feature_id];
    double timestamp = cloudKeyPoses6D->points[feature_id].time;
    
    RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: SQL 실행 시도", feature_id);
    
    // DB에 저장
    if (db_manager_->addLoopFeature(feature_id, timestamp, pose, cloud)) {
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: DB 저장 성공", feature_id);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "루프 특징점 ID %d: DB 저장 실패", feature_id);
    }
}

// DB에서 루프 특징점 로드
pcl::PointCloud<PointType>::Ptr LoopClosure::loadLoopFeatureFromDB(int feature_id) {
    if (!use_db_mode_ || !db_manager_ || !db_manager_->isInitialized()) {
        RCLCPP_WARN(node_->get_logger(), "DB 모드가 활성화되지 않았거나 DB 관리자가 초기화되지 않았습니다");
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }
    
    // 임시 캐시에 이미 있는지 확인
    auto cache_it = tempCloudCache.find(feature_id);
    if (cache_it != tempCloudCache.end()) {
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: 메모리 캐시에서 로드됨", feature_id);
        return cache_it->second;
    }
    
    // DB에서 로드
    RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: DB에서 로드 시도", feature_id);
    pcl::PointCloud<PointType>::Ptr cloud = db_manager_->loadLoopFeature(feature_id);
    if (!cloud || cloud->empty()) {
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: DB에서 로드 실패", feature_id);
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }
    
    // 임시 캐시에 저장
    tempCloudCache[feature_id] = cloud;
    RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: DB에서 로드 성공 (포인트 수: %zu)", feature_id, cloud->size());
    
    return cloud;
}

// 활성 루프 특징점 윈도우 업데이트
void LoopClosure::updateActiveLoopFeatureWindow(const PointTypePose& current_pose) {
    if (!use_db_mode_ || !db_manager_ || !db_manager_->isInitialized()) {
        return;
    }
    
    db_manager_->updateLoopFeatureActiveWindow(current_pose);
} 