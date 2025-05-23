#include "db_manager.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <sstream>
#include <sys/resource.h>
#include <unistd.h>
#include <chrono>

// 메모리 사용량 얻기 함수 (Linux 전용)
size_t getCurrentRSS() {
    long rss = 0L;
    FILE* fp = nullptr;
    if ((fp = fopen("/proc/self/statm", "r")) == nullptr) {
        return 0L;
    }
    if (fscanf(fp, "%*s%ld", &rss) != 1) {
        fclose(fp);
        return 0L;
    }
    fclose(fp);
    return (size_t)rss * (size_t)sysconf(_SC_PAGESIZE);
}

DBManager::DBManager(rclcpp::Node* node, 
                     const std::string& db_path, 
                     int active_window_size,
                     double spatial_radius)
    : node_(node),
      db_path_(db_path),
      active_window_size_(active_window_size),
      spatial_radius_(spatial_radius),
      db_(nullptr),
      stmt_add_keyframe_(nullptr),
      stmt_add_pointcloud_(nullptr),
      stmt_add_factor_(nullptr),
      stmt_load_cloud_(nullptr),
      stmt_load_keyframe_(nullptr),
      stmt_spatial_query_(nullptr),
      stmt_loop_candidates_(nullptr),
      monitoring_running_(false)
{
    RCLCPP_INFO(node_->get_logger(), "DBManager 초기화 중: %s", db_path_.c_str());
}

DBManager::~DBManager() {
    stopMemoryMonitoring();
    finalizeStatements();
    
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
        RCLCPP_INFO(node_->get_logger(), "데이터베이스 연결 닫힘");
    }
}

bool DBManager::initialize() {
    // SQLite 데이터베이스 열기
    int rc = sqlite3_open(db_path_.c_str(), &db_);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 열기 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    // 성능 최적화 설정
    sqlite3_exec(db_, "PRAGMA journal_mode = WAL;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_, "PRAGMA synchronous = NORMAL;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_, "PRAGMA temp_store = MEMORY;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_, "PRAGMA cache_size = 10000;", nullptr, nullptr, nullptr);
    
    // 테이블 생성
    if (!createTables()) {
        RCLCPP_ERROR(node_->get_logger(), "테이블 생성 실패");
        return false;
    }
    
    // prepared statements 준비
    if (!prepareStatements()) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비문 생성 실패");
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "데이터베이스 초기화 성공");
    return true;
}

bool DBManager::createTables() {
    const char* create_keyframes_sql = 
        "CREATE TABLE IF NOT EXISTS keyframes ("
        "id INTEGER PRIMARY KEY, "
        "timestamp REAL, "
        "pose_x REAL, pose_y REAL, pose_z REAL, "
        "pose_qw REAL, pose_qx REAL, pose_qy REAL, pose_qz REAL, "
        "pose_roll REAL, pose_pitch REAL, pose_yaw REAL, "
        "used_recently INTEGER DEFAULT 0);";
    
    const char* create_pointclouds_sql = 
        "CREATE TABLE IF NOT EXISTS pointclouds ("
        "keyframe_id INTEGER PRIMARY KEY, "
        "cloud_data BLOB, "
        "resolution INTEGER, "
        "FOREIGN KEY(keyframe_id) REFERENCES keyframes(id));";
    
    const char* create_factors_sql = 
        "CREATE TABLE IF NOT EXISTS factors ("
        "id INTEGER PRIMARY KEY, "
        "factor_type TEXT, "
        "from_id INTEGER, "
        "to_id INTEGER, "
        "transform_data BLOB, "
        "covariance BLOB, "
        "FOREIGN KEY(from_id) REFERENCES keyframes(id), "
        "FOREIGN KEY(to_id) REFERENCES keyframes(id));";
    
    const char* create_spatial_index_sql = 
        "CREATE INDEX IF NOT EXISTS spatial_idx ON keyframes(pose_x, pose_y, pose_z);";
    
    char* err_msg = nullptr;
    if (sqlite3_exec(db_, create_keyframes_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "keyframes 테이블 생성 오류: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    if (sqlite3_exec(db_, create_pointclouds_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "pointclouds 테이블 생성 오류: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    if (sqlite3_exec(db_, create_factors_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "factors 테이블 생성 오류: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    if (sqlite3_exec(db_, create_spatial_index_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "공간 인덱스 생성 오류: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    return true;
}

bool DBManager::prepareStatements() {
    const char* add_keyframe_sql = 
        "INSERT OR REPLACE INTO keyframes "
        "(id, timestamp, pose_x, pose_y, pose_z, pose_qw, pose_qx, pose_qy, pose_qz, pose_roll, pose_pitch, pose_yaw, used_recently) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 1);";
    
    const char* add_pointcloud_sql = 
        "INSERT OR REPLACE INTO pointclouds "
        "(keyframe_id, cloud_data, resolution) "
        "VALUES (?, ?, ?);";
    
    const char* add_factor_sql = 
        "INSERT OR REPLACE INTO factors "
        "(id, factor_type, from_id, to_id, transform_data, covariance) "
        "VALUES (?, ?, ?, ?, ?, ?);";
    
    const char* load_cloud_sql = 
        "SELECT cloud_data FROM pointclouds WHERE keyframe_id = ?;";
    
    const char* load_keyframe_sql = 
        "SELECT timestamp, pose_x, pose_y, pose_z, pose_qw, pose_qx, pose_qy, pose_qz, pose_roll, pose_pitch, pose_yaw "
        "FROM keyframes WHERE id = ?;";
    
    const char* spatial_query_sql = 
        "SELECT id FROM keyframes "
        "WHERE (pose_x - ?)*(pose_x - ?) + (pose_y - ?)*(pose_y - ?) + (pose_z - ?)*(pose_z - ?) <= ? * ? "
        "ORDER BY ((pose_x - ?)*(pose_x - ?) + (pose_y - ?)*(pose_y - ?) + (pose_z - ?)*(pose_z - ?)) "
        "LIMIT ?;";
    
    const char* loop_candidates_sql = 
        "SELECT k.id FROM keyframes k "
        "WHERE k.id != ? "
        "AND ((k.pose_x - (SELECT pose_x FROM keyframes WHERE id = ?))*(k.pose_x - (SELECT pose_x FROM keyframes WHERE id = ?)) + "
        "     (k.pose_y - (SELECT pose_y FROM keyframes WHERE id = ?))*(k.pose_y - (SELECT pose_y FROM keyframes WHERE id = ?)) + "
        "     (k.pose_z - (SELECT pose_z FROM keyframes WHERE id = ?))*(k.pose_z - (SELECT pose_z FROM keyframes WHERE id = ?))) > ? * ? "
        "LIMIT ?;";
    
    if (sqlite3_prepare_v2(db_, add_keyframe_sql, -1, &stmt_add_keyframe_, nullptr) != SQLITE_OK ||
        sqlite3_prepare_v2(db_, add_pointcloud_sql, -1, &stmt_add_pointcloud_, nullptr) != SQLITE_OK ||
        sqlite3_prepare_v2(db_, add_factor_sql, -1, &stmt_add_factor_, nullptr) != SQLITE_OK ||
        sqlite3_prepare_v2(db_, load_cloud_sql, -1, &stmt_load_cloud_, nullptr) != SQLITE_OK ||
        sqlite3_prepare_v2(db_, load_keyframe_sql, -1, &stmt_load_keyframe_, nullptr) != SQLITE_OK ||
        sqlite3_prepare_v2(db_, spatial_query_sql, -1, &stmt_spatial_query_, nullptr) != SQLITE_OK ||
        sqlite3_prepare_v2(db_, loop_candidates_sql, -1, &stmt_loop_candidates_, nullptr) != SQLITE_OK) {
        
        RCLCPP_ERROR(node_->get_logger(), "준비문 생성 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    return true;
}

void DBManager::finalizeStatements() {
    if (stmt_add_keyframe_) {
        sqlite3_finalize(stmt_add_keyframe_);
        stmt_add_keyframe_ = nullptr;
    }
    
    if (stmt_add_pointcloud_) {
        sqlite3_finalize(stmt_add_pointcloud_);
        stmt_add_pointcloud_ = nullptr;
    }
    
    if (stmt_add_factor_) {
        sqlite3_finalize(stmt_add_factor_);
        stmt_add_factor_ = nullptr;
    }
    
    if (stmt_load_cloud_) {
        sqlite3_finalize(stmt_load_cloud_);
        stmt_load_cloud_ = nullptr;
    }
    
    if (stmt_load_keyframe_) {
        sqlite3_finalize(stmt_load_keyframe_);
        stmt_load_keyframe_ = nullptr;
    }
    
    if (stmt_spatial_query_) {
        sqlite3_finalize(stmt_spatial_query_);
        stmt_spatial_query_ = nullptr;
    }
    
    if (stmt_loop_candidates_) {
        sqlite3_finalize(stmt_loop_candidates_);
        stmt_loop_candidates_ = nullptr;
    }
}

bool DBManager::addKeyFrame(int id, double timestamp, const PointTypePose& pose, 
                           const pcl::PointCloud<PointType>::Ptr& cloud) {
    RCLCPP_INFO(node_->get_logger(), "[DBManager] addKeyFrame called, id=%d, timestamp=%.3f", id, timestamp);
    
    beginTransaction();
    
    // 키프레임 메타데이터 추가
    sqlite3_reset(stmt_add_keyframe_);
    sqlite3_bind_int(stmt_add_keyframe_, 1, id);
    sqlite3_bind_double(stmt_add_keyframe_, 2, timestamp);
    sqlite3_bind_double(stmt_add_keyframe_, 3, pose.x);
    sqlite3_bind_double(stmt_add_keyframe_, 4, pose.y);
    sqlite3_bind_double(stmt_add_keyframe_, 5, pose.z);
    
    // 쿼터니언 값을 채울 수 있는 경우 (형식에 따라 다름)
    // 회전을 표현하는 쿼터니언 계산
    Eigen::AngleAxisf yawAngle(pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(pose.roll, Eigen::Vector3f::UnitX());
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    
    sqlite3_bind_double(stmt_add_keyframe_, 6, q.w());
    sqlite3_bind_double(stmt_add_keyframe_, 7, q.x());
    sqlite3_bind_double(stmt_add_keyframe_, 8, q.y());
    sqlite3_bind_double(stmt_add_keyframe_, 9, q.z());
    
    // Roll, Pitch, Yaw 값
    sqlite3_bind_double(stmt_add_keyframe_, 10, pose.roll);
    sqlite3_bind_double(stmt_add_keyframe_, 11, pose.pitch);
    sqlite3_bind_double(stmt_add_keyframe_, 12, pose.yaw);
    
    if (sqlite3_step(stmt_add_keyframe_) != SQLITE_DONE) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 추가 실패: %s", sqlite3_errmsg(db_));
        rollbackTransaction();
        return false;
    }
    
    // 포인트 클라우드 직렬화 및 저장
    std::vector<uint8_t> serialized_cloud;
    if (!serializePointCloud(cloud, serialized_cloud)) {
        RCLCPP_ERROR(node_->get_logger(), "클라우드 직렬화 실패");
        rollbackTransaction();
        return false;
    }
    
    sqlite3_reset(stmt_add_pointcloud_);
    sqlite3_bind_int(stmt_add_pointcloud_, 1, id);
    sqlite3_bind_blob(stmt_add_pointcloud_, 2, serialized_cloud.data(), serialized_cloud.size(), SQLITE_STATIC);
    sqlite3_bind_int(stmt_add_pointcloud_, 3, 1); // 해상도 플래그 (향후 다중 해상도 지원용)
    
    if (sqlite3_step(stmt_add_pointcloud_) != SQLITE_DONE) {
        RCLCPP_ERROR(node_->get_logger(), "포인트 클라우드 추가 실패: %s", sqlite3_errmsg(db_));
        rollbackTransaction();
        return false;
    }
    
    commitTransaction();
    
    // 활성 윈도우에 추가
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (active_keyframe_ids_.size() >= active_window_size_) {
            removeOldestFromActiveWindow();
        }
        active_keyframe_ids_.push_back(id);
        cloud_cache_[id] = cloud;
    }
    
    RCLCPP_INFO(node_->get_logger(), "addKeyFrame 성공: id=%d", id);
    return true;
}

bool DBManager::serializePointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, std::vector<uint8_t>& data) {
    if (!cloud) {
        return false;
    }
    
    // PCL 클라우드를 바이너리 형식으로 직렬화
    std::ostringstream oss;
    
    // 헤더 정보 저장
    uint32_t cloud_size = cloud->size();
    oss.write(reinterpret_cast<const char*>(&cloud_size), sizeof(cloud_size));
    
    // 포인트 정보 직접 저장 (더 효율적인 방법)
    if (cloud_size > 0) {
        oss.write(reinterpret_cast<const char*>(&cloud->points[0]), cloud_size * sizeof(PointType));
    }
    
    // 결과 복사
    std::string str = oss.str();
    data.resize(str.size());
    std::copy(str.begin(), str.end(), data.begin());
    
    return true;
}

bool DBManager::deserializePointCloud(const std::vector<uint8_t>& data, pcl::PointCloud<PointType>::Ptr& cloud) {
    if (data.empty()) {
        return false;
    }
    
    // 새 클라우드 생성
    cloud.reset(new pcl::PointCloud<PointType>());
    
    std::istringstream iss(std::string(data.begin(), data.end()));
    
    // 헤더 읽기
    uint32_t cloud_size;
    iss.read(reinterpret_cast<char*>(&cloud_size), sizeof(cloud_size));
    
    // 클라우드 크기 설정 및 포인트 데이터 읽기
    cloud->resize(cloud_size);
    if (cloud_size > 0) {
        iss.read(reinterpret_cast<char*>(&cloud->points[0]), cloud_size * sizeof(PointType));
    }
    
    // PCL 클라우드 속성 설정
    cloud->width = cloud_size;
    cloud->height = 1;
    cloud->is_dense = false;
    
    return true;
}

bool DBManager::addPointCloud(int keyframe_id, const pcl::PointCloud<PointType>::Ptr& cloud, int resolution) {
    RCLCPP_INFO(node_->get_logger(), "[DBManager] addPointCloud called, keyframe_id=%d", keyframe_id);
    // ... existing code ...
    return true;
}

bool DBManager::addFactor(int id, const std::string& type, int from_id, int to_id,
                         const Eigen::Matrix4f& transform, const Eigen::MatrixXd& covariance) {
    RCLCPP_INFO(node_->get_logger(), "[DBManager] addFactor called, id=%d, type=%s", id, type.c_str());
    
    // 변환 행렬 직렬화
    std::vector<uint8_t> transform_data(sizeof(float) * 16);
    memcpy(transform_data.data(), transform.data(), sizeof(float) * 16);
    
    // 공분산 행렬 직렬화
    size_t covariance_size = covariance.rows() * covariance.cols() * sizeof(double);
    std::vector<uint8_t> covariance_data(covariance_size);
    memcpy(covariance_data.data(), covariance.data(), covariance_size);
    
    // SQL 실행
    sqlite3_reset(stmt_add_factor_);
    sqlite3_bind_int(stmt_add_factor_, 1, id);
    sqlite3_bind_text(stmt_add_factor_, 2, type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt_add_factor_, 3, from_id);
    sqlite3_bind_int(stmt_add_factor_, 4, to_id);
    sqlite3_bind_blob(stmt_add_factor_, 5, transform_data.data(), transform_data.size(), SQLITE_STATIC);
    sqlite3_bind_blob(stmt_add_factor_, 6, covariance_data.data(), covariance_data.size(), SQLITE_STATIC);
    
    if (sqlite3_step(stmt_add_factor_) != SQLITE_DONE) {
        RCLCPP_ERROR(node_->get_logger(), "팩터 추가 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    return true;
}

pcl::PointCloud<PointType>::Ptr DBManager::loadCloud(int id) {
    RCLCPP_INFO(node_->get_logger(), "[DBManager] loadCloud called, id=%d", id);
    // 먼저 캐시에서 확인
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        auto it = cloud_cache_.find(id);
        if (it != cloud_cache_.end()) {
            // 캐시 히트
            return it->second;
        }
    }
    
    // 데이터베이스에서 로드
    sqlite3_reset(stmt_load_cloud_);
    sqlite3_bind_int(stmt_load_cloud_, 1, id);
    
    int rc = sqlite3_step(stmt_load_cloud_);
    if (rc != SQLITE_ROW) {
        RCLCPP_ERROR(node_->get_logger(), "클라우드 로드 실패 (ID: %d): %s", id, sqlite3_errmsg(db_));
        return nullptr;
    }
    
    // BLOB 데이터 가져오기
    const void* blob_data = sqlite3_column_blob(stmt_load_cloud_, 0);
    int blob_size = sqlite3_column_bytes(stmt_load_cloud_, 0);
    
    std::vector<uint8_t> data(static_cast<const uint8_t*>(blob_data), 
                              static_cast<const uint8_t*>(blob_data) + blob_size);
    
    // 역직렬화
    pcl::PointCloud<PointType>::Ptr cloud;
    if (!deserializePointCloud(data, cloud)) {
        RCLCPP_ERROR(node_->get_logger(), "클라우드 역직렬화 실패 (ID: %d)", id);
        return nullptr;
    }
    
    // 캐시에 저장
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (cloud_cache_.size() >= active_window_size_) {
            // 가장 오래된 항목 제거
            if (!active_keyframe_ids_.empty()) {
                int oldest_id = active_keyframe_ids_.front();
                cloud_cache_.erase(oldest_id);
            }
        }
        cloud_cache_[id] = cloud;
    }
    
    return cloud;
}

void DBManager::updateActiveWindow(const PointTypePose& current_pose) {
    RCLCPP_INFO(node_->get_logger(), "[DBManager] updateActiveWindow called, pose=(%.2f,%.2f,%.2f)", current_pose.x, current_pose.y, current_pose.z);
    // 현재 위치 주변의 키프레임 검색
    std::vector<int> nearby_ids = loadKeyFramesByRadius(current_pose, spatial_radius_, active_window_size_);
    
    // 활성 윈도우 업데이트
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    // 기존 활성 윈도우 클리어
    cloud_cache_.clear();
    active_keyframe_ids_.clear();
    
    // 메모리에 로드
    for (int id : nearby_ids) {
        pcl::PointCloud<PointType>::Ptr cloud = loadCloud(id);
        if (cloud) {
            active_keyframe_ids_.push_back(id);
        }
    }
    
    // 데이터베이스의 used_recently 필드 업데이트
    beginTransaction();
    
    std::string update_sql = "UPDATE keyframes SET used_recently = 0;";
    char* err_msg = nullptr;
    sqlite3_exec(db_, update_sql.c_str(), nullptr, nullptr, &err_msg);
    
    if (!nearby_ids.empty()) {
        std::string id_list;
        for (size_t i = 0; i < nearby_ids.size(); ++i) {
            id_list += std::to_string(nearby_ids[i]);
            if (i < nearby_ids.size() - 1) {
                id_list += ",";
            }
        }
        
        std::string update_recent_sql = "UPDATE keyframes SET used_recently = 1 WHERE id IN (" + id_list + ");";
        sqlite3_exec(db_, update_recent_sql.c_str(), nullptr, nullptr, &err_msg);
    }
    
    commitTransaction();
    
    RCLCPP_INFO(node_->get_logger(), "활성 윈도우 업데이트: %zu 키프레임 로드됨", active_keyframe_ids_.size());
}

std::vector<int> DBManager::loadKeyFramesByRadius(const PointTypePose& center_pose, 
                                                double radius, 
                                                int max_frames) {
    RCLCPP_INFO(node_->get_logger(), "[DBManager] loadKeyFramesByRadius called, center=(%.2f,%.2f,%.2f), radius=%.2f, max_frames=%d", center_pose.x, center_pose.y, center_pose.z, radius, max_frames);
    // 기본값 설정
    if (radius < 0) {
        radius = spatial_radius_;
    }
    
    if (max_frames < 0) {
        max_frames = 50;  // 기본값
    }
    
    std::vector<int> result;
    
    sqlite3_reset(stmt_spatial_query_);
    
    // 중심점 바인딩 (x, y, z) 여러 번
    for (int i = 0; i < 14; i += 3) {
        sqlite3_bind_double(stmt_spatial_query_, i + 1, center_pose.x);
        sqlite3_bind_double(stmt_spatial_query_, i + 2, center_pose.y);
        sqlite3_bind_double(stmt_spatial_query_, i + 3, center_pose.z);
    }
    
    // 반경과 최대 프레임 수 바인딩
    sqlite3_bind_double(stmt_spatial_query_, 7, radius);
    sqlite3_bind_double(stmt_spatial_query_, 8, radius);
    sqlite3_bind_int(stmt_spatial_query_, 15, max_frames);
    
    while (sqlite3_step(stmt_spatial_query_) == SQLITE_ROW) {
        int id = sqlite3_column_int(stmt_spatial_query_, 0);
        result.push_back(id);
    }
    
    return result;
}

std::vector<int> DBManager::findLoopClosureCandidates(int current_id, double min_distance, int max_frames) {
    if (max_frames <= 0) {
        max_frames = 10;  // 기본값
    }
    
    std::vector<int> candidates;
    
    sqlite3_reset(stmt_loop_candidates_);
    sqlite3_bind_int(stmt_loop_candidates_, 1, current_id);
    
    // 참조할 키프레임 ID 바인딩 (여러 번)
    for (int i = 0; i < 6; i += 2) {
        sqlite3_bind_int(stmt_loop_candidates_, i + 2, current_id);
        sqlite3_bind_int(stmt_loop_candidates_, i + 3, current_id);
    }
    
    // 최소 거리와 최대 프레임 수 바인딩
    sqlite3_bind_double(stmt_loop_candidates_, 9, min_distance);
    sqlite3_bind_double(stmt_loop_candidates_, 10, min_distance);
    sqlite3_bind_int(stmt_loop_candidates_, 11, max_frames);
    
    while (sqlite3_step(stmt_loop_candidates_) == SQLITE_ROW) {
        int id = sqlite3_column_int(stmt_loop_candidates_, 0);
        candidates.push_back(id);
    }
    
    return candidates;
}

std::vector<std::pair<int, Eigen::Matrix4f>> DBManager::loadRelatedFactors(int id) {
    std::vector<std::pair<int, Eigen::Matrix4f>> factors;
    
    const char* sql = 
        "SELECT id, transform_data FROM factors WHERE from_id = ? OR to_id = ?;";
    
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "팩터 로드 준비 실패: %s", sqlite3_errmsg(db_));
        return factors;
    }
    
    // 파라미터 바인딩
    sqlite3_bind_int(stmt, 1, id);
    sqlite3_bind_int(stmt, 2, id);
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        int factor_id = sqlite3_column_int(stmt, 0);
        
        // 변환 행렬 디시리얼라이즈
        const void* blob_data = sqlite3_column_blob(stmt, 1);
        int blob_size = sqlite3_column_bytes(stmt, 1);
        
        if (blob_size == sizeof(float) * 16) {
            Eigen::Matrix4f transform;
            memcpy(transform.data(), blob_data, blob_size);
            
            factors.emplace_back(factor_id, transform);
        }
    }
    
    sqlite3_finalize(stmt);
    return factors;
}

bool DBManager::loadFactorGraphValues(const std::vector<int>& ids, 
                                     gtsam::NonlinearFactorGraph& graph,
                                     gtsam::Values& initial) {
    if (ids.empty()) {
        return false;
    }
    
    // SQL 문 준비
    std::string id_list;
    for (size_t i = 0; i < ids.size(); ++i) {
        id_list += std::to_string(ids[i]);
        if (i < ids.size() - 1) {
            id_list += ",";
        }
    }
    
    std::string keyframes_sql = "SELECT id, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw "
                             "FROM keyframes WHERE id IN (" + id_list + ");";
    
    std::string factors_sql = "SELECT id, factor_type, from_id, to_id, transform_data, covariance "
                           "FROM factors WHERE from_id IN (" + id_list + ") AND to_id IN (" + id_list + ");";
    
    // 키프레임 로드 및 초기값 설정
    sqlite3_stmt* stmt_keyframes = nullptr;
    if (sqlite3_prepare_v2(db_, keyframes_sql.c_str(), -1, &stmt_keyframes, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 로드 준비 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    while (sqlite3_step(stmt_keyframes) == SQLITE_ROW) {
        int id = sqlite3_column_int(stmt_keyframes, 0);
        double x = sqlite3_column_double(stmt_keyframes, 1);
        double y = sqlite3_column_double(stmt_keyframes, 2);
        double z = sqlite3_column_double(stmt_keyframes, 3);
        double roll = sqlite3_column_double(stmt_keyframes, 4);
        double pitch = sqlite3_column_double(stmt_keyframes, 5);
        double yaw = sqlite3_column_double(stmt_keyframes, 6);
        
        // GTSAM 포즈 생성
        gtsam::Rot3 rotation = gtsam::Rot3::RzRyRx(roll, pitch, yaw);
        gtsam::Point3 translation(x, y, z);
        gtsam::Pose3 pose(rotation, translation);
        
        // 초기값에 추가
        initial.insert(gtsam::Symbol('x', id), pose);
    }
    
    sqlite3_finalize(stmt_keyframes);
    
    // 팩터 로드 및 그래프 구성
    sqlite3_stmt* stmt_factors = nullptr;
    if (sqlite3_prepare_v2(db_, factors_sql.c_str(), -1, &stmt_factors, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "팩터 로드 준비 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    while (sqlite3_step(stmt_factors) == SQLITE_ROW) {
        std::string type = reinterpret_cast<const char*>(sqlite3_column_text(stmt_factors, 1));
        int from_id = sqlite3_column_int(stmt_factors, 2);
        int to_id = sqlite3_column_int(stmt_factors, 3);
        
        // 변환 데이터 로드
        const void* transform_blob = sqlite3_column_blob(stmt_factors, 4);
        int transform_size = sqlite3_column_bytes(stmt_factors, 4);
        
        // 공분산 데이터 로드
        const void* cov_blob = sqlite3_column_blob(stmt_factors, 5);
        int cov_size = sqlite3_column_bytes(stmt_factors, 5);
        
        if (transform_size != sizeof(float) * 16 || cov_size <= 0) {
            continue;  // 잘못된 데이터 건너뛰기
        }
        
        // 변환 행렬 변환
        Eigen::Matrix4f transform_mat;
        memcpy(transform_mat.data(), transform_blob, transform_size);
        
        // 회전과 변환 추출
        Eigen::Matrix3f rot = transform_mat.block<3, 3>(0, 0);
        Eigen::Vector3f trans = transform_mat.block<3, 1>(0, 3);
        
        gtsam::Rot3 rotation(rot.cast<double>());
        gtsam::Point3 translation(trans.cast<double>());
        gtsam::Pose3 relativePose(rotation, translation);
        
        // 공분산 행렬 변환
        int cov_dim = std::sqrt(cov_size / sizeof(double));
        Eigen::MatrixXd cov_mat(cov_dim, cov_dim);
        memcpy(cov_mat.data(), cov_blob, cov_size);
        
        gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Covariance(cov_mat);
        
        // 팩터 타입에 따라 추가
        if (type == "odometry") {
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                gtsam::Symbol('x', from_id), gtsam::Symbol('x', to_id), relativePose, noise));
        }
        else if (type == "loop") {
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                gtsam::Symbol('x', from_id), gtsam::Symbol('x', to_id), relativePose, noise));
        }
        else if (type == "prior" && from_id == to_id) {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                gtsam::Symbol('x', from_id), relativePose, noise));
        }
    }
    
    sqlite3_finalize(stmt_factors);
    return true;
}

double DBManager::getDistanceBetweenPoses(const PointTypePose& pose1, const PointTypePose& pose2) {
    double dx = pose1.x - pose2.x;
    double dy = pose1.y - pose2.y;
    double dz = pose1.z - pose2.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

pcl::PointCloud<PointType>::Ptr DBManager::loadGlobalMap(float downsampling_leaf_size) {
    pcl::PointCloud<PointType>::Ptr global_map(new pcl::PointCloud<PointType>());
    
    // 모든 키프레임 ID 가져오기
    const char* sql = "SELECT id FROM keyframes;";
    sqlite3_stmt* stmt = nullptr;
    
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "전역 맵 로드 준비 실패: %s", sqlite3_errmsg(db_));
        return global_map;
    }
    
    // 포인트 클라우드들 병합
    std::vector<int> all_ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        int id = sqlite3_column_int(stmt, 0);
        all_ids.push_back(id);
    }
    
    sqlite3_finalize(stmt);
    
    // 메모리 효율성을 위해 배치 처리
    const int batch_size = 10;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(downsampling_leaf_size, downsampling_leaf_size, downsampling_leaf_size);
    
    for (size_t i = 0; i < all_ids.size(); i += batch_size) {
        pcl::PointCloud<PointType>::Ptr batch_cloud(new pcl::PointCloud<PointType>());
        
        size_t end_idx = std::min(i + batch_size, all_ids.size());
        for (size_t j = i; j < end_idx; ++j) {
            pcl::PointCloud<PointType>::Ptr cloud = loadCloud(all_ids[j]);
            if (cloud && !cloud->empty()) {
                *batch_cloud += *cloud;
            }
        }
        
        // 배치 다운샘플링
        if (!batch_cloud->empty()) {
            pcl::PointCloud<PointType>::Ptr filtered_batch(new pcl::PointCloud<PointType>());
            voxel_filter.setInputCloud(batch_cloud);
            voxel_filter.filter(*filtered_batch);
            
            // 글로벌 맵에 추가
            *global_map += *filtered_batch;
            
            // 중간 진행 보고
            RCLCPP_INFO(node_->get_logger(), "글로벌 맵 로드 중: %.1f%% 완료", 
                       100.0f * end_idx / all_ids.size());
        }
    }
    
    // 최종 다운샘플링
    if (!global_map->empty()) {
        pcl::PointCloud<PointType>::Ptr filtered_map(new pcl::PointCloud<PointType>());
        voxel_filter.setInputCloud(global_map);
        voxel_filter.filter(*filtered_map);
        global_map = filtered_map;
    }
    
    RCLCPP_INFO(node_->get_logger(), "글로벌 맵 로드 완료: %zu 포인트", global_map->size());
    return global_map;
}

void DBManager::startMemoryMonitoring(int check_interval_ms, int memory_limit_mb) {
    if (monitoring_running_) {
        return;  // 이미 실행 중
    }
    
    monitoring_running_ = true;
    monitoring_thread_ = std::make_unique<std::thread>(
        &DBManager::monitorMemoryUsage, this, check_interval_ms, memory_limit_mb);
    
    RCLCPP_INFO(node_->get_logger(), "메모리 모니터링 시작 (한계: %d MB)", memory_limit_mb);
}

void DBManager::stopMemoryMonitoring() {
    if (monitoring_running_) {
        monitoring_running_ = false;
        if (monitoring_thread_ && monitoring_thread_->joinable()) {
            monitoring_thread_->join();
        }
        monitoring_thread_.reset();
        RCLCPP_INFO(node_->get_logger(), "메모리 모니터링 중지됨");
    }
}

void DBManager::monitorMemoryUsage(int check_interval_ms, int memory_limit_mb) {
    while (monitoring_running_) {
        // 현재 메모리 사용량 확인
        size_t current_memory_bytes = getCurrentRSS();
        double current_memory_mb = current_memory_bytes / (1024.0 * 1024.0);
        
        if (current_memory_mb > memory_limit_mb) {
            RCLCPP_WARN(node_->get_logger(), "메모리 사용량 과다: %.1f MB/%.1f MB", 
                       current_memory_mb, static_cast<double>(memory_limit_mb));
            
            // 캐시에서 항목 제거
            std::lock_guard<std::mutex> lock(cache_mutex_);
            
            // 캐시 크기 절반으로 줄이기
            int to_remove = active_keyframe_ids_.size() / 2;
            for (int i = 0; i < to_remove && !active_keyframe_ids_.empty(); ++i) {
                int oldest_id = active_keyframe_ids_.front();
                active_keyframe_ids_.pop_front();
                cloud_cache_.erase(oldest_id);
            }
            
            RCLCPP_INFO(node_->get_logger(), "메모리 관리: %d 키프레임 캐시에서 제거됨", to_remove);
        }
        
        // 지정된 간격만큼 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
    }
}

bool DBManager::isKeyFrameInActiveWindow(int id) const {
    return std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), id) != active_keyframe_ids_.end();
}

void DBManager::removeOldestFromActiveWindow() {
    if (!active_keyframe_ids_.empty()) {
        int oldest_id = active_keyframe_ids_.front();
        cloud_cache_.erase(oldest_id);
        active_keyframe_ids_.pop_front();
    }
}

std::vector<int> DBManager::getActiveKeyFrameIds() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return std::vector<int>(active_keyframe_ids_.begin(), active_keyframe_ids_.end());
}

bool DBManager::beginTransaction() {
    char* err_msg = nullptr;
    if (sqlite3_exec(db_, "BEGIN TRANSACTION;", nullptr, nullptr, &err_msg) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "트랜잭션 시작 실패: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    return true;
}

bool DBManager::commitTransaction() {
    char* err_msg = nullptr;
    if (sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, &err_msg) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "트랜잭션 커밋 실패: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    return true;
}

bool DBManager::rollbackTransaction() {
    char* err_msg = nullptr;
    if (sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, &err_msg) != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "트랜잭션 롤백 실패: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    return true;
} 