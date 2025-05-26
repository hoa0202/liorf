#include "db_manager.h"

DBManager::DBManager(rclcpp::Node* node, 
                     const std::string& db_path, 
                     int max_memory_keyframes,
                     double spatial_query_radius,
                     const std::string& clouds_directory,
                     bool reset_on_start)
    : node_(node), 
      max_memory_keyframes_(max_memory_keyframes),
      spatial_query_radius_(spatial_query_radius),
      db_(nullptr), 
      initialized_(false),
      stop_thread_(false),
      reset_on_start_(reset_on_start)
{
    // 상대 경로인 경우 절대 경로로 변환
    if (db_path.empty()) {
        db_path_ = std::string(std::getenv("HOME")) + "/liorf_maps/slam_map.db";
    } else if (db_path[0] != '/') {
        // 상대 경로인 경우 HOME 기준으로 절대 경로로 변환
        db_path_ = std::string(std::getenv("HOME")) + "/" + db_path;
    } else {
        // 이미 절대 경로면 그대로 사용
        db_path_ = db_path;
    }
    
    // 클라우드 디렉토리 경로도 같은 방식으로 처리
    if (clouds_directory.empty()) {
        clouds_directory_ = std::string(std::getenv("HOME")) + "/liorf_maps/clouds";
    } else if (clouds_directory[0] != '/') {
        clouds_directory_ = std::string(std::getenv("HOME")) + "/" + clouds_directory;
    } else {
        clouds_directory_ = clouds_directory;
    }
    
    // DB 저장 디렉토리 생성
    std::string db_dir = db_path_.substr(0, db_path_.find_last_of('/'));
    if (!db_dir.empty()) {
        createDirectoryIfNotExists(db_dir);
    }
    
    // PCD 파일 저장할 디렉토리 생성
    createDirectoryIfNotExists(clouds_directory_);
    
    RCLCPP_INFO(node_->get_logger(), "DBManager 초기화: 데이터베이스 경로 = %s", db_path_.c_str());
    RCLCPP_INFO(node_->get_logger(), "포인트 클라우드 저장 경로 = %s", clouds_directory_.c_str());
    RCLCPP_INFO(node_->get_logger(), "메모리 키프레임 제한: %d, 공간 쿼리 반경: %.2f", 
               max_memory_keyframes_, spatial_query_radius_);
    RCLCPP_INFO(node_->get_logger(), "시작 시 데이터베이스 초기화: %s", 
               reset_on_start_ ? "활성화 (완전 초기화 모드)" : "비활성화 (연속 매핑 모드)");
}

DBManager::~DBManager() {
    stopMemoryMonitoring();
    
    // 데이터베이스 연결 닫기
    if (db_ != nullptr) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
    
    RCLCPP_INFO(node_->get_logger(), "DBManager 종료됨");
}

bool DBManager::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 추가 디버깅 정보 출력
    RCLCPP_INFO(node_->get_logger(), "데이터베이스 초기화 시작: 경로=%s", db_path_.c_str());
    RCLCPP_INFO(node_->get_logger(), "현재 작업 디렉토리 확인");
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        RCLCPP_INFO(node_->get_logger(), "현재 작업 디렉토리: %s", cwd);
    }
    
    // // 직접 파일 생성 테스트
    // RCLCPP_INFO(node_->get_logger(), "직접 파일 생성 테스트 시작");
    // std::string test_file = db_path_ + ".test";
    // std::ofstream test_out(test_file);
    // if (test_out.is_open()) {
    //     test_out << "test file creation succeeded" << std::endl;
    //     test_out.close();
    //     RCLCPP_INFO(node_->get_logger(), "테스트 파일 생성 성공: %s", test_file.c_str());
    // } else {
    //     RCLCPP_ERROR(node_->get_logger(), "테스트 파일 생성 실패: %s", test_file.c_str());
    // }
    
    // DB 저장 디렉토리 다시 확인 및 생성
    std::string db_dir = db_path_.substr(0, db_path_.find_last_of('/'));
    if (!db_dir.empty()) {
        RCLCPP_INFO(node_->get_logger(), "데이터베이스 디렉토리 생성 시도: %s", db_dir.c_str());
        createDirectoryIfNotExists(db_dir);
    }
    
    // 완전 초기화 모드인 경우 기존 데이터베이스 및 클라우드 파일 삭제
    if (reset_on_start_) {
        RCLCPP_INFO(node_->get_logger(), "완전 초기화 모드: 기존 데이터베이스 및 클라우드 파일 삭제 시작");
        if (!resetDatabase()) {
            RCLCPP_WARN(node_->get_logger(), "데이터베이스 초기화 중 일부 파일 삭제 실패");
            // 경고만 하고 계속 진행 (파일이 없었을 수도 있음)
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "연속 매핑 모드: 기존 데이터베이스 유지");
    }
    
    // 파일이 이미 존재하는지 확인
    if (std::filesystem::exists(db_path_)) {
        RCLCPP_INFO(node_->get_logger(), "데이터베이스 파일이 이미 존재합니다: %s", db_path_.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "데이터베이스 파일이 존재하지 않습니다. 새로 생성합니다: %s", db_path_.c_str());
    }
    
    // SQLite 데이터베이스 열기 - sqlite3_open_v2 사용
    int rc;
    
    // SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE 플래그 사용
    rc = sqlite3_open_v2(db_path_.c_str(), &db_, 
                        SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, 
                        NULL);
    
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 열기 실패: %s (오류 코드: %d)", sqlite3_errmsg(db_), rc);
        
        // 추가 디버깅 정보
        RCLCPP_ERROR(node_->get_logger(), "파일 디렉토리 권한 확인: %s", db_dir.c_str());
        system(("ls -la " + db_dir + " >> /tmp/dbdebug.txt").c_str());
        
        return false;
    }
    
    // 추가 확인: 데이터베이스 파일이 생성되었는지 확인
    if (std::filesystem::exists(db_path_)) {
        RCLCPP_INFO(node_->get_logger(), "데이터베이스 파일이 성공적으로 생성/열렸습니다: %s", db_path_.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 파일이 여전히 존재하지 않습니다: %s", db_path_.c_str());
    }
    
    // 테이블 생성
    if (!createTables()) {
        RCLCPP_ERROR(node_->get_logger(), "테이블 생성 실패");
        return false;
    }
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "데이터베이스 초기화 완료");
    return true;
}

bool DBManager::resetDatabase() {
    RCLCPP_INFO(node_->get_logger(), "데이터베이스 초기화 진행 중...");
    
    // SQLite 데이터베이스 파일 삭제
    bool success = true;
    if (std::filesystem::exists(db_path_)) {
        try {
            if (std::filesystem::remove(db_path_)) {
                RCLCPP_INFO(node_->get_logger(), "데이터베이스 파일 삭제 성공: %s", db_path_.c_str());
            } else {
                RCLCPP_WARN(node_->get_logger(), "데이터베이스 파일 삭제 실패: %s", db_path_.c_str());
                success = false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "데이터베이스 파일 삭제 예외: %s", e.what());
            success = false;
        }
    }
    
    // 저널 파일 삭제 (있을 경우)
    std::string journal_file = db_path_ + "-journal";
    if (std::filesystem::exists(journal_file)) {
        try {
            if (std::filesystem::remove(journal_file)) {
                RCLCPP_INFO(node_->get_logger(), "저널 파일 삭제 성공: %s", journal_file.c_str());
            } else {
                RCLCPP_WARN(node_->get_logger(), "저널 파일 삭제 실패: %s", journal_file.c_str());
                // 저널 파일 삭제 실패는 심각한 오류가 아님
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "저널 파일 삭제 예외: %s", e.what());
            // 저널 파일 삭제 실패는 심각한 오류가 아님
        }
    }
    
    // 테스트 파일 삭제 (있을 경우)
    std::string test_file = db_path_ + ".test";
    if (std::filesystem::exists(test_file)) {
        try {
            if (std::filesystem::remove(test_file)) {
                RCLCPP_INFO(node_->get_logger(), "테스트 파일 삭제 성공: %s", test_file.c_str());
            } else {
                RCLCPP_WARN(node_->get_logger(), "테스트 파일 삭제 실패: %s", test_file.c_str());
                // 테스트 파일 삭제 실패는 심각한 오류가 아님
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "테스트 파일 삭제 예외: %s", e.what());
            // 테스트 파일 삭제 실패는 심각한 오류가 아님
        }
    }
    
    // 클라우드 디렉토리 내의 모든 PCD 파일 삭제
    if (std::filesystem::exists(clouds_directory_)) {
        try {
            RCLCPP_INFO(node_->get_logger(), "클라우드 디렉토리 내 PCD 파일 삭제 중: %s", clouds_directory_.c_str());
            int deleted_count = 0;
            for (const auto& entry : std::filesystem::directory_iterator(clouds_directory_)) {
                if (entry.path().extension() == ".pcd") {
                    std::filesystem::remove(entry.path());
                    deleted_count++;
                }
            }
            RCLCPP_INFO(node_->get_logger(), "총 %d개의 PCD 파일 삭제 완료", deleted_count);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "PCD 파일 삭제 예외: %s", e.what());
            success = false;
        }
    }
    
    return success;
}

bool DBManager::createTables() {
    // 키프레임 테이블 생성
    std::string sql = "CREATE TABLE IF NOT EXISTS keyframes ("
                      "id INTEGER PRIMARY KEY, "
                      "timestamp REAL, "
                      "x REAL, y REAL, z REAL, "
                      "roll REAL, pitch REAL, yaw REAL, "
                      "cloud_file TEXT, "
                      "num_points INTEGER"
                      ");";
    
    if (!executeSql(sql)) {
        return false;
    }
    
    // 공간 인덱스 생성 (R-Tree는 SQLite 3.8.0 이상 지원)
    sql = "CREATE VIRTUAL TABLE IF NOT EXISTS keyframes_rtree USING rtree("
          "id, "          // ID
          "min_x, max_x, " // X 범위
          "min_y, max_y, " // Y 범위
          "min_z, max_z"   // Z 범위
          ");";
    
    return executeSql(sql);
}

bool DBManager::executeSql(const std::string& sql) {
    char* error_msg = nullptr;
    int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &error_msg);
    
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 실행 오류: %s", error_msg);
        sqlite3_free(error_msg);
        return false;
    }
    
    return true;
}

bool DBManager::addKeyFrame(int keyframe_id, double timestamp, const PointTypePose& pose, 
                           pcl::PointCloud<PointType>::Ptr cloud) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 초기화되지 않음");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 포인트 클라우드 파일 저장
    std::string cloud_file;
    if (cloud && cloud->size() > 0) {
        if (!saveCloudToFile(keyframe_id, cloud)) {
            RCLCPP_ERROR(node_->get_logger(), "포인트 클라우드 파일 저장 실패: ID=%d", keyframe_id);
            return false;
        }
        cloud_file = getCloudFilePath(keyframe_id);
    }
    
    // 키프레임 정보 저장
    std::string sql = "INSERT OR REPLACE INTO keyframes (id, timestamp, x, y, z, roll, pitch, yaw, cloud_file, num_points) "
                      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    // 매개변수 바인딩
    sqlite3_bind_int(stmt, 1, keyframe_id);
    sqlite3_bind_double(stmt, 2, timestamp);
    sqlite3_bind_double(stmt, 3, pose.x);
    sqlite3_bind_double(stmt, 4, pose.y);
    sqlite3_bind_double(stmt, 5, pose.z);
    sqlite3_bind_double(stmt, 6, pose.roll);
    sqlite3_bind_double(stmt, 7, pose.pitch);
    sqlite3_bind_double(stmt, 8, pose.yaw);
    sqlite3_bind_text(stmt, 9, cloud_file.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_int(stmt, 10, cloud ? cloud->size() : 0);
    
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (rc != SQLITE_DONE) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 삽입 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    // R-tree 인덱스 업데이트
    // 간단히 구현하기 위해 위치를 기준으로 작은 박스(1m x 1m x 1m) 생성
    float box_size = 1.0f; // 1미터 박스
    sql = "INSERT OR REPLACE INTO keyframes_rtree (id, min_x, max_x, min_y, max_y, min_z, max_z) "
          "VALUES (?, ?, ?, ?, ?, ?, ?);";
    
    rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "R-tree SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    sqlite3_bind_int(stmt, 1, keyframe_id);
    sqlite3_bind_double(stmt, 2, pose.x - box_size/2);
    sqlite3_bind_double(stmt, 3, pose.x + box_size/2);
    sqlite3_bind_double(stmt, 4, pose.y - box_size/2);
    sqlite3_bind_double(stmt, 5, pose.y + box_size/2);
    sqlite3_bind_double(stmt, 6, pose.z - box_size/2);
    sqlite3_bind_double(stmt, 7, pose.z + box_size/2);
    
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (rc != SQLITE_DONE) {
        RCLCPP_ERROR(node_->get_logger(), "R-tree 인덱스 업데이트 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    // 메모리 캐시 업데이트
    if (cloud && cloud->size() > 0) {
        cloud_cache_[keyframe_id] = cloud;
    }
    
    // 활성 키프레임 목록에 추가
    if (std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), keyframe_id) == active_keyframe_ids_.end()) {
        active_keyframe_ids_.push_back(keyframe_id);
    }
    
    // 메모리 제한 적용
    enforceMemoryLimit();
    
    return true;
}

pcl::PointCloud<PointType>::Ptr DBManager::loadCloud(int keyframe_id) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 초기화되지 않음");
        return nullptr;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 먼저 메모리 캐시에서 확인
    auto it = cloud_cache_.find(keyframe_id);
    if (it != cloud_cache_.end() && it->second) {
        return it->second;
    }
    
    // 캐시에 없으면 데이터베이스에서 조회
    std::string sql = "SELECT cloud_file FROM keyframes WHERE id = ?;";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return nullptr;
    }
    
    sqlite3_bind_int(stmt, 1, keyframe_id);
    
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_ROW) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 ID %d 찾을 수 없음", keyframe_id);
        sqlite3_finalize(stmt);
        return nullptr;
    }
    
    const char* cloud_file = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    if (!cloud_file || strlen(cloud_file) == 0) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 ID %d의 클라우드 파일 경로가 없음", keyframe_id);
        sqlite3_finalize(stmt);
        return nullptr;
    }
    
    sqlite3_finalize(stmt);
    
    // 파일에서 포인트 클라우드 로드
    auto cloud = loadCloudFromFile(keyframe_id);
    if (!cloud || cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "포인트 클라우드 로드 실패: ID=%d", keyframe_id);
        return nullptr;
    }
    
    // 캐시에 저장
    cloud_cache_[keyframe_id] = cloud;
    
    // 최근 사용 목록 업데이트
    auto it_active = std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), keyframe_id);
    if (it_active == active_keyframe_ids_.end()) {
        active_keyframe_ids_.push_back(keyframe_id);
    }
    
    // 메모리 제한 적용
    enforceMemoryLimit();
    
    return cloud;
}

bool DBManager::deleteKeyFrame(int keyframe_id) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 초기화되지 않음");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 메모리 캐시에서 제거
    cloud_cache_.erase(keyframe_id);
    
    auto it = std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), keyframe_id);
    if (it != active_keyframe_ids_.end()) {
        active_keyframe_ids_.erase(it);
    }
    
    // 포인트 클라우드 파일 삭제
    std::string file_path = getCloudFilePath(keyframe_id);
    if (std::filesystem::exists(file_path)) {
        std::filesystem::remove(file_path);
    }
    
    // 데이터베이스에서 삭제
    std::string sql = "DELETE FROM keyframes WHERE id = ?;";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    sqlite3_bind_int(stmt, 1, keyframe_id);
    
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (rc != SQLITE_DONE) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 삭제 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    // R-tree 인덱스에서도 삭제
    sql = "DELETE FROM keyframes_rtree WHERE id = ?;";
    
    rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "R-tree SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    sqlite3_bind_int(stmt, 1, keyframe_id);
    
    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (rc != SQLITE_DONE) {
        RCLCPP_ERROR(node_->get_logger(), "R-tree 인덱스 삭제 실패: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    return true;
}

std::vector<int> DBManager::loadKeyFramesByRadius(const PointTypePose& current_pose, 
                                                 double radius, 
                                                 int max_keyframes) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 초기화되지 않음");
        return {};
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<int> result;
    
    // R-tree를 사용한 공간 쿼리
    std::string sql = "SELECT k.id FROM keyframes k "
                      "JOIN keyframes_rtree r ON k.id = r.id "
                      "WHERE r.min_x <= ? AND r.max_x >= ? "
                      "AND r.min_y <= ? AND r.max_y >= ? "
                      "AND r.min_z <= ? AND r.max_z >= ? "
                      "ORDER BY (k.x - ?)*(k.x - ?) + (k.y - ?)*(k.y - ?) + (k.z - ?)*(k.z - ?) ASC "
                      "LIMIT ?;";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return result;
    }
    
    // 공간 검색 범위 설정
    sqlite3_bind_double(stmt, 1, current_pose.x + radius);
    sqlite3_bind_double(stmt, 2, current_pose.x - radius);
    sqlite3_bind_double(stmt, 3, current_pose.y + radius);
    sqlite3_bind_double(stmt, 4, current_pose.y - radius);
    sqlite3_bind_double(stmt, 5, current_pose.z + radius);
    sqlite3_bind_double(stmt, 6, current_pose.z - radius);
    
    // 거리 정렬을 위한 현재 위치
    sqlite3_bind_double(stmt, 7, current_pose.x);
    sqlite3_bind_double(stmt, 8, current_pose.x);
    sqlite3_bind_double(stmt, 9, current_pose.y);
    sqlite3_bind_double(stmt, 10, current_pose.y);
    sqlite3_bind_double(stmt, 11, current_pose.z);
    sqlite3_bind_double(stmt, 12, current_pose.z);
    
    // 결과 제한
    sqlite3_bind_int(stmt, 13, max_keyframes);
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        int keyframe_id = sqlite3_column_int(stmt, 0);
        result.push_back(keyframe_id);
    }
    
    sqlite3_finalize(stmt);
    
    return result;
}

pcl::PointCloud<PointType>::Ptr DBManager::loadGlobalMap(float leaf_size) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스 초기화되지 않음");
        return nullptr;
    }
    
    RCLCPP_INFO(node_->get_logger(), "전체 맵 로드 중...");
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    pcl::PointCloud<PointType>::Ptr global_map(new pcl::PointCloud<PointType>());
    
    // 모든 키프레임 ID 가져오기
    std::string sql = "SELECT id FROM keyframes ORDER BY id;";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return global_map;
    }
    
    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        int keyframe_id = sqlite3_column_int(stmt, 0);
        
        auto cloud = loadCloud(keyframe_id);
        if (cloud && !cloud->empty()) {
            *global_map += *cloud;
            count++;
        }
        
        if (count % 10 == 0) {
            RCLCPP_INFO(node_->get_logger(), "키프레임 %d개 로드됨...", count);
        }
    }
    
    sqlite3_finalize(stmt);
    
    // 다운샘플링
    if (leaf_size > 0 && !global_map->empty()) {
        pcl::PointCloud<PointType>::Ptr global_map_ds(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setLeafSize(leaf_size, leaf_size, leaf_size);
        downSizeFilter.setInputCloud(global_map);
        downSizeFilter.filter(*global_map_ds);
        
        RCLCPP_INFO(node_->get_logger(), "전체 맵 로드 완료: %lu -> %lu 포인트 (다운샘플링 적용)",
                   global_map->size(), global_map_ds->size());
                   
        return global_map_ds;
    }
    
    RCLCPP_INFO(node_->get_logger(), "전체 맵 로드 완료: %lu 포인트", global_map->size());
    
    return global_map;
}

void DBManager::updateActiveWindow(const PointTypePose& current_pose) {
    if (!initialized_) return;
    
    // 공간 쿼리로 현재 위치 주변의 키프레임 로드
    std::vector<int> nearby_keyframes = loadKeyFramesByRadius(
        current_pose, 
        spatial_query_radius_, 
        max_memory_keyframes_
    );
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 활성 키프레임 목록 업데이트
    active_keyframe_ids_ = nearby_keyframes;
    
    // 활성 키프레임에 없는 클라우드는 캐시에서 제거
    std::vector<int> keys_to_remove;
    
    for (const auto& pair : cloud_cache_) {
        int keyframe_id = pair.first;
        if (std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), keyframe_id) == active_keyframe_ids_.end()) {
            keys_to_remove.push_back(keyframe_id);
        }
    }
    
    for (int key : keys_to_remove) {
        cloud_cache_.erase(key);
    }
    
    // 메모리 제한 적용
    enforceMemoryLimit();
}

void DBManager::startMemoryMonitoring() {
    if (memory_monitor_thread_.joinable()) {
        stopMemoryMonitoring();
    }
    
    stop_thread_ = false;
    memory_monitor_thread_ = std::thread(&DBManager::memoryMonitoringThread, this);
}

void DBManager::stopMemoryMonitoring() {
    stop_thread_ = true;
    if (memory_monitor_thread_.joinable()) {
        memory_monitor_thread_.join();
    }
}

void DBManager::memoryMonitoringThread() {
    // 5초마다 메모리 제한 검사
    const int check_interval_ms = 5000;
    
    while (!stop_thread_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            enforceMemoryLimit();
        }
        
        // 메모리 사용량 출력 (매 10번째 반복마다)
        static int counter = 0;
        if (++counter % 10 == 0) {
            printStats();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
    }
}

void DBManager::enforceMemoryLimit() {
    // 메모리 제한 적용
    if (cloud_cache_.size() <= static_cast<size_t>(max_memory_keyframes_)) {
        return;
    }
    
    // 캐시 크기가 제한을 초과하면 LRU 방식으로 제거
    // active_keyframe_ids_에 없는 항목부터 제거
    std::vector<int> non_active_keys;
    
    for (const auto& pair : cloud_cache_) {
        int keyframe_id = pair.first;
        if (std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), keyframe_id) == active_keyframe_ids_.end()) {
            non_active_keys.push_back(keyframe_id);
        }
    }
    
    // 제거할 항목 수 계산
    int num_to_remove = cloud_cache_.size() - max_memory_keyframes_;
    
    // 비활성 키를 먼저 제거
    for (int i = 0; i < std::min(num_to_remove, static_cast<int>(non_active_keys.size())); ++i) {
        cloud_cache_.erase(non_active_keys[i]);
    }
    
    // 비활성 키를 모두 제거해도 여전히 제한을 초과하면 활성 키에서도 제거
    if (cloud_cache_.size() > static_cast<size_t>(max_memory_keyframes_)) {
        num_to_remove = cloud_cache_.size() - max_memory_keyframes_;
        
        // 가장 오래된 활성 키프레임부터 제거
        for (int i = 0; i < std::min(num_to_remove, static_cast<int>(active_keyframe_ids_.size())); ++i) {
            // 가장 오래된 항목은 리스트의 앞쪽에 있음
            cloud_cache_.erase(active_keyframe_ids_[i]);
        }
    }
}

void DBManager::printStats() {
    if (!initialized_) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 키프레임 수 조회
    int total_keyframes = getNumKeyFrames();
    
    // 메모리 사용량 계산 (대략적인 추정)
    size_t memory_usage = 0;
    for (const auto& pair : cloud_cache_) {
        if (pair.second) {
            memory_usage += pair.second->size() * sizeof(PointType);
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "DB 통계: 총 키프레임=%d, 메모리 캐시=%zu, 활성 키프레임=%zu, 메모리 사용=%.2f MB",
               total_keyframes, cloud_cache_.size(), active_keyframe_ids_.size(), memory_usage / (1024.0 * 1024.0));
}

int DBManager::getNumKeyFrames() const {
    if (!initialized_) return 0;
    
    std::string sql = "SELECT COUNT(*) FROM keyframes;";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return 0;
    }
    
    rc = sqlite3_step(stmt);
    
    int count = 0;
    if (rc == SQLITE_ROW) {
        count = sqlite3_column_int(stmt, 0);
    }
    
    sqlite3_finalize(stmt);
    
    return count;
}

Eigen::Matrix4f DBManager::poseToMatrix(const PointTypePose& pose) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    // 회전 행렬 계산
    Eigen::AngleAxisf rotX(pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotY(pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotZ(pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotation = (rotZ * rotY * rotX).toRotationMatrix();
    
    // 변환 행렬 설정
    matrix.block<3, 3>(0, 0) = rotation;
    matrix.block<3, 1>(0, 3) = Eigen::Vector3f(pose.x, pose.y, pose.z);
    
    return matrix;
}

PointTypePose DBManager::matrixToPose(const Eigen::Matrix4f& matrix) {
    PointTypePose pose;
    
    // 위치 추출
    pose.x = matrix(0, 3);
    pose.y = matrix(1, 3);
    pose.z = matrix(2, 3);
    
    // 회전 추출
    Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
    Eigen::Vector3f euler = rotation.eulerAngles(0, 1, 2); // XYZ 순서
    
    pose.roll = euler[0];
    pose.pitch = euler[1];
    pose.yaw = euler[2];
    
    return pose;
}

bool DBManager::saveCloudToFile(int keyframe_id, pcl::PointCloud<PointType>::Ptr cloud) {
    if (!cloud || cloud->empty()) {
        return false;
    }
    
    std::string file_path = getCloudFilePath(keyframe_id);
    
    // 디렉토리 확인
    std::string dir_path = file_path.substr(0, file_path.find_last_of('/'));
    createDirectoryIfNotExists(dir_path);
    
    // 파일 저장
    if (pcl::io::savePCDFileBinary(file_path, *cloud) == -1) {
        RCLCPP_ERROR(node_->get_logger(), "PCD 파일 저장 실패: %s", file_path.c_str());
        return false;
    }
    
    return true;
}

pcl::PointCloud<PointType>::Ptr DBManager::loadCloudFromFile(int keyframe_id) {
    std::string file_path = getCloudFilePath(keyframe_id);
    
    if (!std::filesystem::exists(file_path)) {
        RCLCPP_ERROR(node_->get_logger(), "PCD 파일 없음: %s", file_path.c_str());
        return nullptr;
    }
    
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    
    if (pcl::io::loadPCDFile(file_path, *cloud) == -1) {
        RCLCPP_ERROR(node_->get_logger(), "PCD 파일 로드 실패: %s", file_path.c_str());
        return nullptr;
    }
    
    return cloud;
}

std::string DBManager::getCloudFilePath(int keyframe_id) {
    return clouds_directory_ + "/keyframe_" + std::to_string(keyframe_id) + ".pcd";
}

void DBManager::createDirectoryIfNotExists(const std::string& directory) {
    if (!std::filesystem::exists(directory)) {
        try {
            std::filesystem::create_directories(directory);
            RCLCPP_INFO(node_->get_logger(), "디렉토리 생성됨: %s", directory.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "디렉토리 생성 실패: %s - %s", directory.c_str(), e.what());
        }
    }
} 