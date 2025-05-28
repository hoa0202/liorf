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
    
    if (cloudKeyPoses3D->points.empty())
        return;

    // find keys
    int loopKeyCur;
    int loopKeyPre;
    if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;

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
                return;
            }
            
            // 이전 키프레임 로드
            auto prevCloud = getKeyFrameFromDB(loopKeyPre);
            if (prevCloud) {
                loopFindNearKeyFrames(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
            } else {
                return;
            }
        } else {
            // 기존 메모리 방식
            loopFindNearKeyFrames(cureKeyframeCloud, loopKeyCur, 0, -1);
            loopFindNearKeyFrames(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
        }
        
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
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

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

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
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
    
    // Set flag
    aLoopIsClosed = true;
}

void LoopClosure::performSCLoopClosure()
{
    std::lock_guard<std::mutex> lock(mtx);
    
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

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return;

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    {
        int base_key = 0;
        loopFindNearKeyFrames(cureKeyframeCloud, loopKeyCur, 0, base_key);
        loopFindNearKeyFrames(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key);

        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
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

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

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
}

bool LoopClosure::detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

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
    // 수정: 외부에서 재사용하기 위해 변환 함수 추가
    auto transformPointCloud = [this](const pcl::PointCloud<PointType>::Ptr& cloudIn, PointTypePose* transformIn) 
                              -> pcl::PointCloud<PointType>::Ptr {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        Eigen::Affine3f transCur = pcl::getTransformation(
            transformIn->x, transformIn->y, transformIn->z, 
            transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        pcl::transformPointCloud(*cloudIn, *cloudOut, transCur);
        return cloudOut;
    };

    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = cloudKeyPoses6D->size();
    
    // DB 모드일 경우 DB에서 키프레임 로드
    if (use_db_mode_ && db_manager_) {
        std::vector<int> keys_to_load;
        
        // 먼저 로드할 키 ID 목록 수집
        for (int i = -searchNum; i <= searchNum; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;
                
            keys_to_load.push_back(keyNear);
        }
        
        // 각 키에 대해 DB에서 포인트 클라우드 로드 및 변환
        for (int keyNear : keys_to_load) {
            int select_loop_index = (loop_index != -1) ? loop_index : keyNear;
            
            // DB에서 포인트 클라우드 로드 (임시 캐시 사용)
            pcl::PointCloud<PointType>::Ptr cloud = getKeyFrameFromDB(keyNear);
            if (!cloud || cloud->empty()) {
                RCLCPP_ERROR(node_->get_logger(), "키프레임 ID %d를 DB에서 로드하지 못했습니다", keyNear);
                continue;
            }
            
            // 변환 및 추가
            pcl::PointCloud<PointType>::Ptr transformed_cloud = transformPointCloud(cloud, &cloudKeyPoses6D->points[select_loop_index]);
            *nearKeyframes += *transformed_cloud;
        }
    } else {
        // 기존 메모리 모드
        for (int i = -searchNum; i <= searchNum; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize || keyNear >= (int)surfCloudKeyFrames.size())
                continue;

            int select_loop_index = (loop_index != -1) ? loop_index : keyNear;
            pcl::PointCloud<PointType>::Ptr transformed_cloud = transformPointCloud(surfCloudKeyFrames[keyNear], &cloudKeyPoses6D->points[select_loop_index]);
            *nearKeyframes += *transformed_cloud;
        }
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
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