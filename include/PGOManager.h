#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include "db_manager.h"

#include <memory>
#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

class PGOManager {
public:
    PGOManager(rclcpp::Node::SharedPtr node) : logger_(node->get_logger()) {}
    ~PGOManager();

    // DB 매니저 설정
    void setDBManager(DBManager* db_manager) { db_manager_ = db_manager; }
    DBManager* getDBManager() const { return db_manager_; }

    // 키프레임 노드 추가 (DB ID 지정 가능)
    int addKeyframeNode(const gtsam::Pose3& initialPose, int db_keyframe_id = -1);

    // 오도메트리 팩터 추가
    void addOdometryFactor(int prevKeyframeIdx, int currentKeyframeIdx, 
                          const gtsam::Pose3& relativePose, 
                          const gtsam::noiseModel::Diagonal::shared_ptr& noise);

    // GPS 팩터 추가
    void addGPSFactor(int keyframeIdx, 
                     const gtsam::Point3& gpsMeasurement, 
                     const gtsam::noiseModel::Diagonal::shared_ptr& noise);

    // 루프 클로저 팩터 추가
    void addLoopFactor(int fromKeyframeIdx, int toKeyframeIdx, 
                      const gtsam::Pose3& relativePose, 
                      const gtsam::noiseModel::Base::shared_ptr& noise);

    // 그래프 최적화 수행
    bool optimizeGraph();

    // 외부에서 DB 키프레임 ID로 접근
    gtsam::Pose3 getOptimizedPose(int db_keyframe_id) const;

    // 모든 최적화된 포즈 가져오기
    const gtsam::Values& getAllOptimizedPoses() const;

    // 최신 포즈의 공분산 가져오기
    gtsam::Matrix getLatestPoseCovariance() const;

    // 그래프의 포즈 개수 가져오기
    size_t getNumPoses() const;

    // ISAM2 인스턴스 가져오기
    gtsam::ISAM2* getISAM2() const { return isam; }

    // Prior Factor 추가
    void addPriorFactor(int nodeIdx, const gtsam::Pose3& pose, const gtsam::noiseModel::Base::shared_ptr& noise);

    // DB에서 키프레임 로드
    void loadKeyframesFromDB();

    // DB에 최적화된 포즈 저장
    void saveOptimizedPosesToDB();

    // 포즈 캐시 관리
    void updatePoseCache(int db_keyframe_id, const gtsam::Pose3& pose);
    gtsam::Pose3 getPoseFromCache(int db_keyframe_id);
    void clearPoseCache();

    // 내부 인덱스 -> DB ID 변환 getter
    int getDbIdFromPGOIdx(int pgo_idx) const {
        auto it = pgoIdxToDbId_.find(pgo_idx);
        if (it != pgoIdxToDbId_.end()) return it->second;
        return pgo_idx;
    }

private:
    rclcpp::Logger logger_;
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::ISAM2* isam;
    gtsam::Values isamCurrentEstimate;
    gtsam::Matrix latestPoseCovariance;
    
    DBManager* db_manager_;
    std::unordered_map<int, gtsam::Pose3> pose_cache_;
    // 매핑 테이블
    std::unordered_map<int, int> dbIdToPGOIdx_;
    std::unordered_map<int, int> pgoIdxToDbId_;

    // ISAM2 파라미터 설정
    void setupISAM2();
}; 