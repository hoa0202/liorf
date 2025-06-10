#include "PGOManager.h"

// PGOManager::PGOManager() {
//     setupISAM2();
// }

PGOManager::~PGOManager() {
    if (isam) {
        delete isam;
    }
}

void PGOManager::setupISAM2() {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);
}

int PGOManager::addKeyframeNode(const gtsam::Pose3& initialPose, int db_keyframe_id) {
    int pgo_idx = getNumPoses();
    initialEstimate.insert(gtsam::Symbol('x', pgo_idx), initialPose);
    // 매핑 테이블 갱신
    if (db_keyframe_id >= 0) {
        dbIdToPGOIdx_[db_keyframe_id] = pgo_idx;
        pgoIdxToDbId_[pgo_idx] = db_keyframe_id;
    }
    // DB에 포즈 저장
    if (db_manager_ && db_manager_->isInitialized() && db_keyframe_id >= 0) {
        PointTypePose pose;
        pose.x = initialPose.translation().x();
        pose.y = initialPose.translation().y();
        pose.z = initialPose.translation().z();
        pose.roll = initialPose.rotation().roll();
        pose.pitch = initialPose.rotation().pitch();
        pose.yaw = initialPose.rotation().yaw();
        pose.time = 0.0;
        db_manager_->addKeyFrame(db_keyframe_id, 0.0, pose, nullptr);
    }
    return pgo_idx;
}

void PGOManager::loadKeyframesFromDB() {
    if (!db_manager_ || !db_manager_->isInitialized()) {
        return;
    }
    
    // 매핑 테이블 초기화
    dbIdToPGOIdx_.clear();
    pgoIdxToDbId_.clear();
    
    // 모든 키프레임 로드
    auto keyframe_ids = db_manager_->loadKeyFramesByRadius(PointTypePose(), std::numeric_limits<double>::max());
    
    // ID 기준으로 정렬
    std::sort(keyframe_ids.begin(), keyframe_ids.end());
    
    int pgo_idx = 0;
    for (int db_id : keyframe_ids) {
        auto cloud = db_manager_->loadCloud(db_id);
        if (cloud && !cloud->empty()) {
            // 포즈 정보 추출
            PointTypePose pose;
            pose.x = cloud->points[0].x;
            pose.y = cloud->points[0].y;
            pose.z = cloud->points[0].z;
            float intensity = cloud->points[0].intensity;
            pose.roll = (intensity - floor(intensity)) * 360.0f;
            pose.pitch = (floor(intensity) - floor(intensity/100.0f) * 100.0f) * 3.6f;
            pose.yaw = floor(intensity/100.0f) * 3.6f;
            
            // GTSAM 포즈 생성
            gtsam::Pose3 gtsam_pose(
                gtsam::Rot3::RzRyRx(pose.roll * M_PI/180.0f, pose.pitch * M_PI/180.0f, pose.yaw * M_PI/180.0f),
                gtsam::Point3(pose.x, pose.y, pose.z)
            );
            
            // 매핑 테이블 업데이트
            dbIdToPGOIdx_[db_id] = pgo_idx;
            pgoIdxToDbId_[pgo_idx] = db_id;
            
            // 초기 추정값 추가
            initialEstimate.insert(gtsam::Symbol('x', pgo_idx), gtsam_pose);
            updatePoseCache(db_id, gtsam_pose);
            
            pgo_idx++;
        }
    }
    
    // 로드된 키프레임 수 로깅
    RCLCPP_INFO(logger_, "DB에서 %d개의 키프레임 로드됨", pgo_idx);
}

void PGOManager::saveOptimizedPosesToDB() {
    if (!db_manager_ || !db_manager_->isInitialized()) {
        return;
    }
    
    RCLCPP_INFO(logger_, "최적화된 포즈 DB에 저장 시작");
    
    // 최적화된 포즈를 DB에 저장
    for (const auto& pair : pgoIdxToDbId_) {
        int pgo_idx = pair.first;
        int db_id = pair.second;
        
        // 최적화된 포즈 가져오기
        auto pose = getOptimizedPose(db_id);
        if (dbIdToPGOIdx_.count(db_id)) {
            // DB에 저장
            PointTypePose pose6D;
            pose6D.x = pose.x();
            pose6D.y = pose.y();
            pose6D.z = pose.z();
            
            // 회전 행렬에서 오일러 각도 추출
            auto rot = pose.rotation();
            auto euler = rot.toQuaternion().toRotationMatrix().eulerAngles(0, 1, 2);
            pose6D.roll = euler[0] * 180.0f / M_PI;
            pose6D.pitch = euler[1] * 180.0f / M_PI;
            pose6D.yaw = euler[2] * 180.0f / M_PI;
            
            // DB에 저장
            if (db_manager_->addKeyFrame(db_id, 0.0, pose6D, nullptr)) {
                RCLCPP_DEBUG(logger_, "키프레임 ID %d 포즈 업데이트 성공", db_id);
            } else {
                RCLCPP_ERROR(logger_, "키프레임 ID %d 포즈 업데이트 실패", db_id);
            }
        }
    }
    
    RCLCPP_INFO(logger_, "최적화된 포즈 DB 저장 완료");
}

void PGOManager::updatePoseCache(int db_keyframe_id, const gtsam::Pose3& pose) {
    pose_cache_[db_keyframe_id] = pose;
}

gtsam::Pose3 PGOManager::getPoseFromCache(int db_keyframe_id) {
    auto it = pose_cache_.find(db_keyframe_id);
    if (it != pose_cache_.end()) {
        return it->second;
    }
    return gtsam::Pose3();
}

void PGOManager::clearPoseCache() {
    pose_cache_.clear();
}

void PGOManager::addPriorFactor(int nodeIdx, const gtsam::Pose3& pose, const gtsam::noiseModel::Base::shared_ptr& noise) {
    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', nodeIdx),
        pose,
        noise
    ));
}

void PGOManager::addOdometryFactor(int prevKeyframeIdx, int currentKeyframeIdx,
                                 const gtsam::Pose3& relativePose,
                                 const gtsam::noiseModel::Diagonal::shared_ptr& noise) {
    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', prevKeyframeIdx),
        gtsam::Symbol('x', currentKeyframeIdx),
        relativePose,
        noise
    ));
}

void PGOManager::addGPSFactor(int keyframeIdx,
                            const gtsam::Point3& gpsMeasurement,
                            const gtsam::noiseModel::Diagonal::shared_ptr& noise) {
    gtSAMgraph.add(gtsam::GPSFactor(
        gtsam::Symbol('x', keyframeIdx),
        gpsMeasurement,
        noise
    ));
}

void PGOManager::addLoopFactor(int fromKeyframeIdx, int toKeyframeIdx,
                             const gtsam::Pose3& relativePose,
                             const gtsam::noiseModel::Base::shared_ptr& noise) {
    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', fromKeyframeIdx),
        gtsam::Symbol('x', toKeyframeIdx),
        relativePose,
        noise
    ));
}

bool PGOManager::optimizeGraph() {
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    
    isamCurrentEstimate = isam->calculateEstimate();
    
    // 최신 포즈의 공분산 계산
    latestPoseCovariance = isam->marginalCovariance(gtsam::Symbol('x', getNumPoses() - 1));
    
    // DB에 최적화된 포즈 저장
    saveOptimizedPosesToDB();
    
    return true;
}

gtsam::Pose3 PGOManager::getOptimizedPose(int db_keyframe_id) const {
    auto it = dbIdToPGOIdx_.find(db_keyframe_id);
    if (it == dbIdToPGOIdx_.end()) {
        // 유효하지 않은 ID
        return gtsam::Pose3();
    }
    int pgo_idx = it->second;
    if (!isamCurrentEstimate.exists(gtsam::Symbol('x', pgo_idx))) {
        return gtsam::Pose3();
    }
    return isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', pgo_idx));
}

const gtsam::Values& PGOManager::getAllOptimizedPoses() const {
    return isamCurrentEstimate;
}

gtsam::Matrix PGOManager::getLatestPoseCovariance() const {
    return latestPoseCovariance;
}

size_t PGOManager::getNumPoses() const {
    return isamCurrentEstimate.size();
} 