#include "db_manager.h"

DBManager::DBManager(rclcpp::Node* node, 
                     const std::string& db_path, 
                     int max_memory_keyframes,
                     double spatial_query_radius,
                     const std::string& clouds_directory,
                     bool reset_on_start,
                     bool localization_mode)
    : node_(node), 
      max_memory_keyframes_(max_memory_keyframes),
      max_memory_loop_features_(max_memory_keyframes), // 루프 특징점 메모리 제한을 키프레임과 동일하게 설정
      spatial_query_radius_(spatial_query_radius),
      db_(nullptr), 
      initialized_(false),
      stop_thread_(false),
      reset_on_start_(reset_on_start),
      localization_mode_(localization_mode)
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
        loop_features_directory_ = std::string(std::getenv("HOME")) + "/liorf_maps/loop_features";
    } else if (clouds_directory[0] != '/') {
        clouds_directory_ = std::string(std::getenv("HOME")) + "/" + clouds_directory;
        loop_features_directory_ = std::string(std::getenv("HOME")) + "/" + clouds_directory + "_loop";
    } else {
        clouds_directory_ = clouds_directory;
        loop_features_directory_ = clouds_directory + "_loop";
    }
    
    // DB 저장 디렉토리 생성
    std::string db_dir = db_path_.substr(0, db_path_.find_last_of('/'));
    if (!db_dir.empty()) {
        createDirectoryIfNotExists(db_dir);
    }
    
    // PCD 파일 저장할 디렉토리 생성
    createDirectoryIfNotExists(clouds_directory_);
    createDirectoryIfNotExists(loop_features_directory_);
    
    RCLCPP_INFO(node_->get_logger(), "DBManager 초기화: 데이터베이스 경로 = %s", db_path_.c_str());
    RCLCPP_INFO(node_->get_logger(), "포인트 클라우드 저장 경로 = %s", clouds_directory_.c_str());
    RCLCPP_INFO(node_->get_logger(), "루프 특징점 저장 경로 = %s", loop_features_directory_.c_str());
    
    // 메모리 제한 로그 - 일관된 형식 유지
    RCLCPP_INFO(node_->get_logger(), "메모리 키프레임 제한: %d, 메모리 루프 특징점 제한: %d, 공간 쿼리 반경: %.2f", 
               max_memory_keyframes_, max_memory_loop_features_, spatial_query_radius_);
               
    RCLCPP_INFO(node_->get_logger(), "시작 시 데이터베이스 초기화: %s", 
               reset_on_start_ ? "활성화 (완전 초기화 모드)" : "비활성화 (연속 매핑 모드)");
               
    RCLCPP_INFO(node_->get_logger(), "로컬라이제이션 모드: %s", 
               localization_mode_ ? "활성화 (데이터베이스 보존)" : "비활성화 (일반 매핑 모드)");
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
    
    // 로컬라이제이션 모드 확인 (localization_mode_ 변수 사용)
    if (localization_mode_) {
        RCLCPP_INFO(node_->get_logger(), "로컬라이제이션 모드에서는 데이터베이스와 PCD 파일을 보존합니다.");
        return true;
    }
    
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
            RCLCPP_INFO(node_->get_logger(), "총 %d개의 키프레임 PCD 파일 삭제 완료", deleted_count);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "키프레임 PCD 파일 삭제 예외: %s", e.what());
            success = false;
        }
    }
    
    // 루프 클로저 특징점 디렉토리 내의 모든 PCD 파일 삭제 (추가된 부분)
    if (std::filesystem::exists(loop_features_directory_)) {
        try {
            RCLCPP_INFO(node_->get_logger(), "루프 특징점 디렉토리 내 PCD 파일 삭제 중: %s", loop_features_directory_.c_str());
            int deleted_count = 0;
            for (const auto& entry : std::filesystem::directory_iterator(loop_features_directory_)) {
                if (entry.path().extension() == ".pcd") {
                    std::filesystem::remove(entry.path());
                    deleted_count++;
                }
            }
            RCLCPP_INFO(node_->get_logger(), "총 %d개의 루프 특징점 PCD 파일 삭제 완료", deleted_count);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "루프 특징점 PCD 파일 삭제 예외: %s", e.what());
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
    
    if (!executeSql(sql)) {
        return false;
    }
    
    // 루프 특징점 테이블 생성
    sql = "CREATE TABLE IF NOT EXISTS loop_features ("
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
    
    // 루프 특징점 공간 인덱스 생성
    sql = "CREATE VIRTUAL TABLE IF NOT EXISTS loop_features_rtree USING rtree("
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
        // 캐시 히트 로그 제거
        return it->second;
    }
    
    // 캐시에 없으면 데이터베이스에서 조회
    // 로그 레벨을 ERROR에서 DEBUG로 변경 - 로그가 너무 많이 출력되는 것 방지
    RCLCPP_DEBUG(node_->get_logger(), "DB 로드: 키프레임 ID=%d", keyframe_id);
    
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
    
    // 로그 추가 - DB에서 성공적으로 로드됨 (DEBUG 레벨로 변경)
    RCLCPP_DEBUG(node_->get_logger(), "★★★ DB 로드 완료: 키프레임 ID=%d (포인트 수: %zu) ★★★", 
               keyframe_id, cloud->size());
    
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
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<int> keyframe_ids;
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스가 초기화되지 않았습니다");
        return keyframe_ids;
    }
    
    try {
        // 쿼리 생성 - X-Y 평면 기준으로만 검색 (Z축 무시)
        char sql[512];
        sprintf(sql, "SELECT id FROM keyframes_rtree WHERE "
                     "min_x <= %f AND max_x >= %f AND "
                     "min_y <= %f AND max_y >= %f "
                     "ORDER BY (min_x - %f)*(min_x - %f) + (min_y - %f)*(min_y - %f) ASC "  // Z축 무시, X-Y 평면만 고려
                     "LIMIT %d;",
                current_pose.x + radius, current_pose.x - radius,
                current_pose.y + radius, current_pose.y - radius,
                current_pose.x, current_pose.x, current_pose.y, current_pose.y,
                max_keyframes);
        
        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, 0);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
            return keyframe_ids;
        }
        
        // 결과 처리
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            int keyframe_id = sqlite3_column_int(stmt, 0);
            keyframe_ids.push_back(keyframe_id);
        }
        
        sqlite3_finalize(stmt);
        
        // 순차적 데이터 정렬: 인덱스 기준으로 정렬하여 가까운 키프레임끼리 연속되게 함
        if (!keyframe_ids.empty()) {
            // 원래 로직처럼 index 기준으로 정렬하여 연속적인 키프레임들이 함께 처리되도록 함
            std::sort(keyframe_ids.begin(), keyframe_ids.end(), 
                      [&current_pose, this](int a, int b) {
                          // 현재 키프레임 ID와의 인덱스 차이를 계산
                          return std::abs(a - static_cast<int>(current_pose.intensity)) < 
                                 std::abs(b - static_cast<int>(current_pose.intensity));
                      });
        }
        
        RCLCPP_DEBUG(node_->get_logger(), "공간 쿼리: 반경 %.2f m 내에서 %zu개 키프레임 발견", 
                   radius, keyframe_ids.size());
        
        return keyframe_ids;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "키프레임 공간 쿼리 중 예외 발생: %s", e.what());
        return keyframe_ids;
    }
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
    
    // 새로 로드된 키프레임 수 계산 (활성 윈도우 변경 추적)
    int new_keyframes = 0;
    for (int id : nearby_keyframes) {
        if (std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), id) == active_keyframe_ids_.end()) {
            new_keyframes++;
        }
    }
    
    // 활성 키프레임 목록 업데이트
    active_keyframe_ids_ = nearby_keyframes;
    
    // 메모리 상태 로깅
    size_t memory_usage = 0;
    for (const auto& pair : cloud_cache_) {
        if (pair.second) {
            memory_usage += pair.second->size() * sizeof(PointType);
        }
    }
    
    // 루프 특징점 메모리 사용량 계산
    size_t loop_memory_usage = 0;
    size_t loop_total_points = 0;
    for (const auto& pair : loop_feature_cache_) {
        if (pair.second) {
            loop_memory_usage += pair.second->size() * sizeof(PointType);
            loop_total_points += pair.second->size();
        }
    }
    
    // 통계 로깅 - 5초마다 출력
    static auto last_log_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_log_time).count() >= 5) {
        size_t total_points = 0;
        for (const auto& pair : cloud_cache_) {
            if (pair.second) {
                total_points += pair.second->size();
            }
        }
        
        // 로그 메시지를 INFO 레벨로 변경하고 요약 정보 제공
        RCLCPP_INFO(node_->get_logger(), "메모리 캐시: %zu개의 키프레임, %.2fMB 사용 중 (총 포인트: %zu개)", 
                   cloud_cache_.size(), 
                   static_cast<double>(memory_usage) / (1024 * 1024),
                   total_points);
        
        // 루프 특징점 캐시 정보 출력
        RCLCPP_INFO(node_->get_logger(), "루프 특징점 캐시: %zu개의 특징점, %.2fMB 사용 중 (총 포인트: %zu개)",
                   loop_feature_cache_.size(),
                   static_cast<double>(loop_memory_usage) / (1024 * 1024),
                   loop_total_points);
                   
        // 활성 키프레임 목록 로깅 (최대 10개까지만)
        std::string active_ids = "";
        int count = 0;
        for (int id : active_keyframe_ids_) {
            if (count++ < 10) {
                active_ids += std::to_string(id) + " ";
            } else {
                active_ids += "...";
                break;
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "활성 키프레임 ID: %s (총 %zu개)", 
        //             active_ids.c_str(), active_keyframe_ids_.size());
                    
        last_log_time = current_time;
    }
    
    // 활성 키프레임에 없는 클라우드는 캐시에서 제거 (모든 클라우드 캐시 제거는 하지 않음)
    std::vector<int> keys_to_remove;
    
    for (const auto& pair : cloud_cache_) {
        int keyframe_id = pair.first;
        if (std::find(active_keyframe_ids_.begin(), active_keyframe_ids_.end(), keyframe_id) == active_keyframe_ids_.end()) {
            keys_to_remove.push_back(keyframe_id);
        }
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
    // 5초마다 메모리 제한 검사 (3초에서 5초로 변경)
    const int check_interval_ms = 5000;
    
    // 메모리 사용량 로깅 간격 (30초로 늘림)
    const int log_interval_ms = 30000;
    auto last_log_time = std::chrono::steady_clock::now();
    
    while (!stop_thread_) {
        try {
            auto current_time = std::chrono::steady_clock::now();
            bool should_log = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_log_time).count() >= log_interval_ms;
            
            {
                std::lock_guard<std::mutex> lock(mutex_);
                // 캐시 관리는 매번 수행
                enforceMemoryLimit();
                
                // 로깅은 지정된 간격으로만 수행 (부하 감소)
                if (should_log) {
                    try {
                        printStats();
                        last_log_time = current_time;
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(node_->get_logger(), "메모리 통계 출력 중 예외 발생: %s", e.what());
                    }
                }
            }
            
            // 짧은 간격으로 여러 번 sleep 호출 (중간에 종료 가능하도록)
            const int sleep_chunk = 500; // 0.5초
            int remaining = check_interval_ms;
            while (remaining > 0 && !stop_thread_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(std::min(sleep_chunk, remaining)));
                remaining -= sleep_chunk;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "메모리 모니터링 스레드에서 예외 발생: %s", e.what());
            // 짧은 대기 후 계속 실행
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}

void DBManager::enforceMemoryLimit() {
    // active_keyframe_ids_가 비어 있거나 max_memory_keyframes_가 0 이하이면 제한 적용 안함
    if (active_keyframe_ids_.empty() || max_memory_keyframes_ <= 0) {
        return;
    }
    
    // 특수 케이스: 활성 키프레임 수가 메모리 제한보다 많은 경우
    // 이 경우 활성 키프레임만 유지하고 나머지는 제거
    if (active_keyframe_ids_.size() > static_cast<size_t>(max_memory_keyframes_)) {
        // 활성 키프레임만 유지하고 나머지 제거
        std::unordered_map<int, pcl::PointCloud<PointType>::Ptr> new_cache;
        
        // 가장 최근 max_memory_keyframes_ 개의 활성 키프레임만 유지
        size_t start_idx = active_keyframe_ids_.size() - max_memory_keyframes_;
        for (size_t i = start_idx; i < active_keyframe_ids_.size(); i++) {
            int key = active_keyframe_ids_[i];
            auto it = cloud_cache_.find(key);
            if (it != cloud_cache_.end()) {
                new_cache[key] = it->second;
            }
        }
        
        // 제거된 키프레임 수 계산
        int removed_count = cloud_cache_.size() - new_cache.size();
        
        // 캐시 교체
        cloud_cache_ = std::move(new_cache);
        
        // INFO 레벨로 변경하고 메시지 간결화
        // if (removed_count > 0) {
        //     RCLCPP_INFO(node_->get_logger(), "메모리 정리: %d개 키프레임 제거됨, 현재 캐시 크기: %zu", 
        //               removed_count, cloud_cache_.size());
        // }
        return;
    }
    
    // 일반 케이스: 캐시 크기가 제한을 초과하면 비활성 키프레임부터 제거
    if (cloud_cache_.size() <= static_cast<size_t>(max_memory_keyframes_)) {
        return;
    }
    
    // 비활성 키 식별
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
    int removed_count = 0;
    for (int i = 0; i < std::min(num_to_remove, static_cast<int>(non_active_keys.size())); ++i) {
        cloud_cache_.erase(non_active_keys[i]);
        removed_count++;
    }
    
    // 제거 로그 추가 (INFO 레벨로 변경)
    // if (removed_count > 0) {
    //     RCLCPP_INFO(node_->get_logger(), "메모리 정리: %d개 키프레임 제거됨, 현재 캐시 크기: %zu", 
        //                removed_count, cloud_cache_.size());
    // }
    
    // 비활성 키를 모두 제거해도 여전히 제한을 초과하는 특수한 경우
    // 이 경우 시스템이 루프 클로저와 같은 중요 작업을 수행 중일 수 있으므로
    // 활성 키는 최대한 유지하고 로그만 출력
    // if (cloud_cache_.size() > static_cast<size_t>(max_memory_keyframes_)) {
    //     RCLCPP_WARN(node_->get_logger(), "메모리 제한 초과: 캐시=%zu, 제한=%d, 활성 키프레임=%zu", 
    //               cloud_cache_.size(), max_memory_keyframes_, active_keyframe_ids_.size());
    // }
}

void DBManager::printStats() {
    if (!initialized_) return;
    
    try {
        // 키프레임 수 조회
        int total_keyframes = getNumKeyFrames();
        
        // 루프 특징점 수 조회
        int total_loop_features = getNumLoopFeatures();
        
        // 메모리 사용량 계산 (대략적인 추정)
        size_t memory_usage = 0;
        size_t total_points = 0;
        for (const auto& pair : cloud_cache_) {
            if (pair.second) {
                memory_usage += pair.second->size() * sizeof(PointType);
                total_points += pair.second->size();
            }
        }
        
        // 루프 특징점 메모리 사용량 계산
        size_t loop_memory_usage = 0;
        size_t loop_total_points = 0;
        for (const auto& pair : loop_feature_cache_) {
            if (pair.second) {
                loop_memory_usage += pair.second->size() * sizeof(PointType);
                loop_total_points += pair.second->size();
            }
        }
        
        // 시스템 메모리 사용량 확인 - 간소화된 로깅을 위해 필수 정보만 가져오기
        std::map<std::string, size_t> process_memory = getProcessMemoryUsage();
        
        // 필수 통계 정보만 출력 (로그 양 감소)
        RCLCPP_INFO(node_->get_logger(), "메모리 상태: 캐시=%zu/총=%d 키프레임, 메모리=%.2f MB, RSS=%.2f MB, VSZ=%.2f MB",
                   cloud_cache_.size(), total_keyframes,
                   memory_usage / (1024.0 * 1024.0),
                   process_memory["VmRSS"] / (1024.0 * 1024.0),
                   process_memory["VmSize"] / (1024.0 * 1024.0));
                   
        // 루프 특징점 통계 정보 출력
        RCLCPP_INFO(node_->get_logger(), "루프 특징점 상태: 캐시=%zu/총=%d 특징점, 메모리=%.2f MB, 포인트=%zu개",
                   loop_feature_cache_.size(), total_loop_features,
                   loop_memory_usage / (1024.0 * 1024.0),
                   loop_total_points);
        
        // 자세한 로그는 DEBUG 레벨로 변경 (INFO 레벨 로그 감소)
        // 단, 디버그 로그는 기본적으로 비활성화
        RCLCPP_DEBUG(node_->get_logger(), "자세한 메모리: 데이터=%.2f MB, 스택=%.2f MB, 최대 RSS=%.2f MB",
                   process_memory["VmData"] / (1024.0 * 1024.0),
                   process_memory["VmStk"] / (1024.0 * 1024.0),
                   process_memory["VmHWM"] / (1024.0 * 1024.0));
        
        // 활성 키프레임 목록 로깅 (최대 5개까지만) - DEBUG 레벨로 변경
        std::string active_ids = "";
        int count = 0;
        for (int id : active_keyframe_ids_) {
            if (count++ < 5) {
                active_ids += std::to_string(id) + " ";
            } else {
                active_ids += "...";
                break;
            }
        }
        RCLCPP_DEBUG(node_->get_logger(), "활성 키프레임 ID: %s (총 %zu개)", 
                    active_ids.c_str(), active_keyframe_ids_.size());
                    
        // 활성 루프 특징점 목록 로깅 (최대 5개까지만)
        std::string active_loop_ids = "";
        int loop_count = 0;
        for (int id : active_loop_feature_ids_) {
            if (loop_count++ < 5) {
                active_loop_ids += std::to_string(id) + " ";
            } else {
                active_loop_ids += "...";
                break;
            }
        }
        RCLCPP_DEBUG(node_->get_logger(), "활성 루프 특징점 ID: %s (총 %zu개)", 
                    active_loop_ids.c_str(), active_loop_feature_ids_.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "통계 정보 계산 중 오류 발생: %s", e.what());
    }
}

// 시스템 메모리 사용량을 확인하는 함수 추가
std::map<std::string, size_t> DBManager::getProcessMemoryUsage() {
    std::map<std::string, size_t> result;
    result["VmRSS"] = 0;   // 물리적 메모리 사용량 (Resident Set Size)
    result["VmSize"] = 0;  // 가상 메모리 크기 (Virtual Memory Size)
    result["VmData"] = 0;  // 데이터 세그먼트 크기
    result["VmStk"] = 0;   // 스택 크기
    result["VmPeak"] = 0;  // 최대 가상 메모리 사용량
    result["VmHWM"] = 0;   // 최대 상주 메모리 사용량 (High Water Mark)
    
    try {
        // /proc/self/status 파일에서 메모리 정보 읽기
        std::ifstream status_file("/proc/self/status");
        if (!status_file.is_open()) {
            RCLCPP_WARN(node_->get_logger(), "프로세스 메모리 정보를 읽을 수 없습니다");
            return result;
        }
        
        std::string line;
        while (std::getline(status_file, line)) {
            try {
                // 필요한 메모리 정보만 처리
                if (line.find("VmRSS:") != std::string::npos || 
                    line.find("VmSize:") != std::string::npos ||
                    line.find("VmData:") != std::string::npos ||
                    line.find("VmStk:") != std::string::npos ||
                    line.find("VmPeak:") != std::string::npos ||
                    line.find("VmHWM:") != std::string::npos) {
                    
                    std::istringstream iss(line);
                    std::string key, value, unit;
                    iss >> key >> value >> unit;
                    
                    // 잘못된 형식이면 건너뛰기
                    if (key.empty() || value.empty()) continue;
                    
                    // 숫자가 아닌 값이면 건너뛰기
                    for (char c : value) {
                        if (!std::isdigit(c)) {
                            continue;
                        }
                    }
                    
                    // 단위 변환 (kB를 바이트로)
                    size_t memory_value = 0;
                    try {
                        memory_value = std::stoul(value);
                        if (unit == "kB") {
                            memory_value *= 1024; // kB -> bytes
                        }
                    } catch (const std::exception& e) {
                        continue; // 변환 실패 시 건너뛰기
                    }
                    
                    // 콜론 제거
                    if (!key.empty() && key.back() == ':') {
                        key.pop_back();
                    }
                    result[key] = memory_value;
                }
            } catch (const std::exception& e) {
                // 라인 파싱 중 예외 발생 시 다음 라인으로 계속
                continue;
            }
        }
        status_file.close();
    } catch (const std::exception& e) {
        // 전체 함수에서 예외 발생 시 안전하게 기본값 반환
        RCLCPP_ERROR(node_->get_logger(), "메모리 정보 읽기 중 예외 발생: %s", e.what());
    }
    
    return result;
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

bool DBManager::addLoopFeature(int feature_id, double timestamp, const PointTypePose& pose, 
                               pcl::PointCloud<PointType>::Ptr cloud) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스가 초기화되지 않았습니다");
        return false;
    }
    
    if (!cloud || cloud->empty()) {
        RCLCPP_WARN(node_->get_logger(), "루프 특징점 ID %d: 빈 포인트 클라우드가 전달되었습니다", feature_id);
        return false;
    }
    
    try {
        // 루프 특징점 테이블에 추가
        char sql[1024];
        sprintf(sql, "INSERT OR REPLACE INTO loop_features "
                     "(id, timestamp, x, y, z, roll, pitch, yaw, cloud_file, num_points) "
                     "VALUES (%d, %f, %f, %f, %f, %f, %f, %f, '%s', %zu);",
                feature_id, timestamp,
                pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw,
                getLoopFeatureFilePath(feature_id).c_str(), cloud->size());
        
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: SQL 실행 시도", feature_id);
        if (!executeSql(sql)) {
            RCLCPP_ERROR(node_->get_logger(), "루프 특징점 ID %d: SQL 실행 실패", feature_id);
            return false;
        }
        
        // R-Tree 인덱스 업데이트 (미세한 범위 추가)
        double offset = 0.1; // 미세한 범위 추가 (10cm)
        
        sprintf(sql, "INSERT OR REPLACE INTO loop_features_rtree "
                     "(id, min_x, max_x, min_y, max_y, min_z, max_z) "
                     "VALUES (%d, %f, %f, %f, %f, %f, %f);",
                feature_id,
                pose.x - offset, pose.x + offset,
                pose.y - offset, pose.y + offset,
                pose.z - offset, pose.z + offset);
        
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: R-Tree 인덱스 업데이트 시도", feature_id);
        if (!executeSql(sql)) {
            RCLCPP_ERROR(node_->get_logger(), "루프 특징점 ID %d: R-Tree 인덱스 업데이트 실패", feature_id);
            return false;
        }
        
        // 포인트 클라우드 파일 저장
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: 파일 저장 시도 - 경로: %s", 
                  feature_id, getLoopFeatureFilePath(feature_id).c_str());
        if (!saveLoopFeatureToFile(feature_id, cloud)) {
            RCLCPP_ERROR(node_->get_logger(), "루프 특징점 ID %d: 클라우드 파일 저장 실패", feature_id);
            return false;
        }
        
        // 캐시에 추가
        loop_feature_cache_[feature_id] = cloud;
        
        // 활성 ID 목록에 추가 (맨 앞에 추가)
        auto it = std::find(active_loop_feature_ids_.begin(), active_loop_feature_ids_.end(), feature_id);
        if (it != active_loop_feature_ids_.end()) {
            active_loop_feature_ids_.erase(it);
        }
        active_loop_feature_ids_.insert(active_loop_feature_ids_.begin(), feature_id);
        
        // 메모리 제한 적용
        enforceLoopFeatureMemoryLimit();
        
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: DB에 성공적으로 추가됨", feature_id);
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "루프 특징점 추가 중 예외 발생: %s", e.what());
        return false;
    }
}

// 루프 특징점 파일 경로 생성 함수
std::string DBManager::getLoopFeatureFilePath(int feature_id) {
    char filename[100];
    sprintf(filename, "loop_feature_%d.pcd", feature_id);
    return loop_features_directory_ + "/" + filename;
}

// 루프 특징점 클라우드 파일 저장 함수
bool DBManager::saveLoopFeatureToFile(int feature_id, pcl::PointCloud<PointType>::Ptr cloud) {
    if (!cloud || cloud->empty()) {
        RCLCPP_WARN(node_->get_logger(), "루프 특징점 ID %d: 저장할 빈 포인트 클라우드", feature_id);
        return false;
    }
    
    std::string filepath = getLoopFeatureFilePath(feature_id);
    
    try {
        // 디렉토리 존재 확인 및 생성
        createDirectoryIfNotExists(loop_features_directory_);
        
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: PCD 파일 저장 시도 (%zu 포인트) - %s", 
                  feature_id, cloud->size(), filepath.c_str());
        
        // PCD 파일 저장
        if (pcl::io::savePCDFileBinary(filepath, *cloud) == -1) {
            RCLCPP_ERROR(node_->get_logger(), "루프 특징점 ID %d: PCD 파일 저장 실패 - %s", feature_id, filepath.c_str());
            return false;
        }
        
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: PCD 파일 저장 성공 - %s", feature_id, filepath.c_str());
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "루프 특징점 PCD 파일 저장 중 예외 발생: %s", e.what());
        return false;
    }
}

// 루프 특징점 클라우드 파일 로드 함수
pcl::PointCloud<PointType>::Ptr DBManager::loadLoopFeature(int feature_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스가 초기화되지 않았습니다");
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }
    
    // 캐시에 있는지 확인
    auto it = loop_feature_cache_.find(feature_id);
    if (it != loop_feature_cache_.end()) {
        // 캐시에 있으면 활성 ID 목록 최신화
        auto active_it = std::find(active_loop_feature_ids_.begin(), active_loop_feature_ids_.end(), feature_id);
        if (active_it != active_loop_feature_ids_.end()) {
            active_loop_feature_ids_.erase(active_it);
        }
        active_loop_feature_ids_.insert(active_loop_feature_ids_.begin(), feature_id);
        
        return it->second;
    }
    
    // 데이터베이스에서 정보 조회
    char sql[256];
    sprintf(sql, "SELECT cloud_file FROM loop_features WHERE id = %d;", feature_id);
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, 0);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }
    
    pcl::PointCloud<PointType>::Ptr cloud;
    
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        // PCD 파일 로드
        cloud = loadLoopFeatureFromFile(feature_id);
        
        // 캐시에 추가
        if (cloud && !cloud->empty()) {
            loop_feature_cache_[feature_id] = cloud;
            
            // 활성 ID 목록에 추가
            active_loop_feature_ids_.insert(active_loop_feature_ids_.begin(), feature_id);
            
            // 메모리 제한 적용
            enforceLoopFeatureMemoryLimit();
        }
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: 데이터베이스에 존재하지 않습니다", feature_id);
        cloud.reset(new pcl::PointCloud<PointType>());
    }
    
    sqlite3_finalize(stmt);
    return cloud;
}

// 루프 특징점 삭제 함수
bool DBManager::deleteLoopFeature(int feature_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스가 초기화되지 않았습니다");
        return false;
    }
    
    try {
        // 데이터베이스에서 삭제
        char sql[256];
        sprintf(sql, "DELETE FROM loop_features WHERE id = %d;", feature_id);
        if (!executeSql(sql)) {
            return false;
        }
        
        sprintf(sql, "DELETE FROM loop_features_rtree WHERE id = %d;", feature_id);
        if (!executeSql(sql)) {
            return false;
        }
        
        // 파일 삭제
        std::string filepath = getLoopFeatureFilePath(feature_id);
        if (std::filesystem::exists(filepath)) {
            std::filesystem::remove(filepath);
        }
        
        // 캐시에서 삭제
        loop_feature_cache_.erase(feature_id);
        
        // 활성 ID 목록에서 삭제
        auto it = std::find(active_loop_feature_ids_.begin(), active_loop_feature_ids_.end(), feature_id);
        if (it != active_loop_feature_ids_.end()) {
            active_loop_feature_ids_.erase(it);
        }
        
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "루프 특징점 삭제 중 예외 발생: %s", e.what());
        return false;
    }
}

void DBManager::enforceLoopFeatureMemoryLimit() {
    // 루프 특징점 메모리 제한 적용
    if (loop_feature_cache_.size() > static_cast<size_t>(max_memory_loop_features_)) {
        // 비활성 특징점 제거
        std::vector<int> non_active_keys;
        for (const auto& pair : loop_feature_cache_) {
            int feature_id = pair.first;
            if (std::find(active_loop_feature_ids_.begin(), active_loop_feature_ids_.end(), feature_id) == active_loop_feature_ids_.end()) {
                non_active_keys.push_back(feature_id);
            }
        }
        
        // 제거할 항목 수 계산
        int num_to_remove = loop_feature_cache_.size() - max_memory_loop_features_;
        
        // 비활성 특징점 제거
        int removed_count = 0;
        for (int i = 0; i < std::min(num_to_remove, static_cast<int>(non_active_keys.size())); ++i) {
            loop_feature_cache_.erase(non_active_keys[i]);
            removed_count++;
        }
        
        // 제거 로그 추가 (INFO 레벨로 변경)
        // if (removed_count > 0) {
        //     RCLCPP_INFO(node_->get_logger(), "메모리 정리: %d개 특징점 제거됨, 현재 캐시 크기: %zu", 
        //                removed_count, loop_feature_cache_.size());
        // }
        
        // 비활성 특징점을 모두 제거해도 여전히 제한을 초과하는 특수한 경우
        // 이 경우 시스템이 루프 클로저와 같은 중요 작업을 수행 중일 수 있으므로
        // 활성 특징점은 최대한 유지하고 로그만 출력
        // if (loop_feature_cache_.size() > static_cast<size_t>(max_memory_loop_features_)) {
        //     RCLCPP_WARN(node_->get_logger(), "메모리 제한 초과: 캐시=%zu, 제한=%d, 활성 특징점=%zu", 
        //               loop_feature_cache_.size(), max_memory_loop_features_, active_loop_feature_ids_.size());
        // }
    }
}

// 루프 특징점 클라우드 파일 로드 함수
pcl::PointCloud<PointType>::Ptr DBManager::loadLoopFeatureFromFile(int feature_id) {
    std::string filepath = getLoopFeatureFilePath(feature_id);
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    
    try {
        if (!std::filesystem::exists(filepath)) {
            RCLCPP_ERROR(node_->get_logger(), "루프 특징점 ID %d: PCD 파일이 존재하지 않습니다 - %s", feature_id, filepath.c_str());
            return cloud; // 빈 클라우드 반환
        }
        
        if (pcl::io::loadPCDFile<PointType>(filepath, *cloud) == -1) {
            RCLCPP_ERROR(node_->get_logger(), "루프 특징점 ID %d: PCD 파일 로드 실패 - %s", feature_id, filepath.c_str());
            return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>()); // 빈 클라우드 반환
        }
        
        // RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 ID %d: PCD 파일 로드 성공 - %s (%zu 포인트)", 
        //               feature_id, filepath.c_str(), cloud->size());
        return cloud;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "루프 특징점 PCD 파일 로드 중 예외 발생: %s", e.what());
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>()); // 빈 클라우드 반환
    }
}

// 루프 특징점 공간 쿼리 함수
std::vector<int> DBManager::loadLoopFeaturesByRadius(const PointTypePose& current_pose,
                                                   double radius,
                                                   int max_features) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<int> feature_ids;
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스가 초기화되지 않았습니다");
        return feature_ids;
    }
    
    try {
        // 쿼리 생성 - X-Y 평면 기준으로만 검색 (Z축 무시)
        char sql[512];
        sprintf(sql, "SELECT id FROM loop_features_rtree WHERE "
                     "min_x <= %f AND max_x >= %f AND "
                     "min_y <= %f AND max_y >= %f "
                     "ORDER BY (min_x - %f)*(min_x - %f) + (min_y - %f)*(min_y - %f) ASC "
                     "LIMIT %d;",
                current_pose.x + radius, current_pose.x - radius,
                current_pose.y + radius, current_pose.y - radius,
                current_pose.x, current_pose.x, current_pose.y, current_pose.y,
                max_features);
        
        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, 0);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
            return feature_ids;
        }
        
        // 결과 처리
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            int feature_id = sqlite3_column_int(stmt, 0);
            feature_ids.push_back(feature_id);
        }
        
        sqlite3_finalize(stmt);
        
        // 순차적 데이터 정렬: 인덱스 기준으로 정렬하여 가까운 특징점끼리 연속되게 함
        if (!feature_ids.empty()) {
            // 원래 로직처럼 index 기준으로 정렬하여 연속적인 특징점들이 함께 처리되도록 함
            std::sort(feature_ids.begin(), feature_ids.end(), 
                      [&current_pose, this](int a, int b) {
                          // 현재 특징점 ID와의 인덱스 차이를 계산
                          return std::abs(a - static_cast<int>(current_pose.intensity)) < 
                                 std::abs(b - static_cast<int>(current_pose.intensity));
                      });
        }
        
        RCLCPP_DEBUG(node_->get_logger(), "공간 쿼리: 반경 %.2f m 내에서 %zu개 루프 특징점 발견", 
                   radius, feature_ids.size());
        
        return feature_ids;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "루프 특징점 공간 쿼리 중 예외 발생: %s", e.what());
        return feature_ids;
    }
}

// 루프 특징점 활성 윈도우 업데이트 함수
void DBManager::updateLoopFeatureActiveWindow(const PointTypePose& current_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "데이터베이스가 초기화되지 않았습니다");
        return;
    }
    
    try {
        // 현재 위치 주변의 루프 특징점 ID 로드 - X-Y 평면 기준으로만 검색 (Z축 무시)
        char sql[512];
        sprintf(sql, "SELECT id FROM loop_features_rtree WHERE "
                     "min_x <= %f AND max_x >= %f AND "
                     "min_y <= %f AND max_y >= %f "
                     "ORDER BY (min_x - %f)*(min_x - %f) + (min_y - %f)*(min_y - %f) ASC "
                     "LIMIT %d;",
                current_pose.x + spatial_query_radius_, current_pose.x - spatial_query_radius_,
                current_pose.y + spatial_query_radius_, current_pose.y - spatial_query_radius_,
                current_pose.x, current_pose.x, current_pose.y, current_pose.y,
                max_memory_loop_features_);
        
        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, 0);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
            return;
        }
        
        // 새로운 활성 ID 목록 생성
        std::vector<int> new_active_ids;
        
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            int feature_id = sqlite3_column_int(stmt, 0);
            new_active_ids.push_back(feature_id);
        }
        
        sqlite3_finalize(stmt);
        
        // 순차적 데이터 정렬: 인덱스 기준으로 정렬하여 가까운 특징점끼리 연속되게 함
        if (!new_active_ids.empty()) {
            // 원래 로직처럼 index 기준으로 정렬하여 연속적인 특징점들이 함께 처리되도록 함
            std::sort(new_active_ids.begin(), new_active_ids.end(), 
                      [&current_pose, this](int a, int b) {
                          // 현재 특징점 ID와의 인덱스 차이를 계산
                          return std::abs(a - static_cast<int>(current_pose.intensity)) < 
                                 std::abs(b - static_cast<int>(current_pose.intensity));
                      });
        }
        
        // 기존 활성 ID 중 새 목록에 없는 것들을 제거하고 캐시도 정리
        for (auto it = active_loop_feature_ids_.begin(); it != active_loop_feature_ids_.end(); ) {
            int feature_id = *it;
            if (std::find(new_active_ids.begin(), new_active_ids.end(), feature_id) == new_active_ids.end()) {
                // 캐시에서 제거
                loop_feature_cache_.erase(feature_id);
                // 활성 ID 목록에서 제거
                it = active_loop_feature_ids_.erase(it);
            } else {
                ++it;
            }
        }
        
        // 새 ID들을 활성 ID 목록에 추가 (중복 제거)
        for (int feature_id : new_active_ids) {
            if (std::find(active_loop_feature_ids_.begin(), active_loop_feature_ids_.end(), feature_id) == active_loop_feature_ids_.end()) {
                active_loop_feature_ids_.push_back(feature_id);
                
                // 아직 캐시에 없는 경우 로드
                if (loop_feature_cache_.find(feature_id) == loop_feature_cache_.end()) {
                    pcl::PointCloud<PointType>::Ptr cloud = loadLoopFeatureFromFile(feature_id);
                    if (cloud && !cloud->empty()) {
                        loop_feature_cache_[feature_id] = cloud;
                    }
                }
            }
        }
        
        // 메모리 제한 적용
        enforceLoopFeatureMemoryLimit();
        
        RCLCPP_DEBUG(node_->get_logger(), "루프 특징점 활성 윈도우 업데이트: %zu개 특징점 활성화, 캐시 크기: %zu", 
                   active_loop_feature_ids_.size(), loop_feature_cache_.size());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "루프 특징점 활성 윈도우 업데이트 중 예외 발생: %s", e.what());
    }
}

// 루프 특징점 통계 출력용 함수
int DBManager::getNumLoopFeatures() const {
    if (!initialized_) {
        return 0;
    }
    
    char sql[] = "SELECT COUNT(*) FROM loop_features;";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, 0);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_));
        return 0;
    }
    
    int count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        count = sqlite3_column_int(stmt, 0);
    }
    
    sqlite3_finalize(stmt);
    return count;
} 