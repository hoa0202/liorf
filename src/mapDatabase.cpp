#include "mapDatabase.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <rcpputils/filesystem_helper.hpp>

namespace fs = rcpputils::fs;

MapDatabase::MapDatabase(rclcpp::Node* node, const std::string& base_path, const std::string& cloud_storage_path)
    : node_(node), base_path_(base_path), cloud_storage_path_(cloud_storage_path)
{
    pose_file_path_ = base_path_ + "/poses.txt";
    
    // 시스템에서 PCD 디렉토리 경로 가져오기 (이미 선언된 파라미터)
    std::string default_pcd_path = base_path_ + "/PCD";
    if (node->has_parameter("savePCDDirectory")) {
        node->get_parameter("savePCDDirectory", pcd_dir_path_);
    } else {
        pcd_dir_path_ = default_pcd_path;
    }
}

MapDatabase::~MapDatabase()
{
}

bool MapDatabase::initialize()
{
    // 맵 초기화 옵션 확인 (이미 선언된 파라미터)
    bool clear_map_on_start = false;
    if (node_->has_parameter("clear_map_on_start")) {
        node_->get_parameter("clear_map_on_start", clear_map_on_start);
    }
    
    // 맵 초기화 옵션이 활성화된 경우 기존 맵 데이터 삭제
    if (clear_map_on_start) {
        RCLCPP_INFO(node_->get_logger(), "맵 초기화 옵션이 활성화되어 있습니다. 기존 맵 데이터를 삭제합니다.");
        clearMapData(true, true, true);
    }
    
    // 기본 디렉토리 생성
    try {
        if (!fs::exists(base_path_)) {
            fs::create_directories(base_path_);
        }
        
        if (!fs::exists(cloud_storage_path_)) {
            fs::create_directories(cloud_storage_path_);
        }
        
        if (!fs::exists(pcd_dir_path_)) {
            fs::create_directories(pcd_dir_path_);
        }
        
        RCLCPP_INFO(node_->get_logger(), "맵 데이터베이스 초기화: 저장 경로 = %s", base_path_.c_str());
        return true;
    } catch (const std::exception& e) {
        logError("디렉토리 생성 실패: " + std::string(e.what()));
        return false;
    }
}

bool MapDatabase::clearMapData(bool clear_cloud_dir, bool clear_pcd_dir, bool clear_pose_file)
{
    try {
        // 클라우드 디렉토리 초기화
        if (clear_cloud_dir && fs::exists(cloud_storage_path_)) {
            RCLCPP_INFO(node_->get_logger(), "클라우드 디렉토리 초기화 중: %s", cloud_storage_path_.c_str());
            
            // 디렉토리 삭제 후 다시 생성 (모든 내용 삭제)
            fs::remove_all(cloud_storage_path_);
            fs::create_directories(cloud_storage_path_);
            
            RCLCPP_INFO(node_->get_logger(), "클라우드 디렉토리 초기화 완료");
        }
        
        // PCD 디렉토리 초기화
        if (clear_pcd_dir && fs::exists(pcd_dir_path_)) {
            RCLCPP_INFO(node_->get_logger(), "PCD 디렉토리 초기화 중: %s", pcd_dir_path_.c_str());
            
            // 디렉토리 삭제 후 다시 생성 (모든 내용 삭제)
            fs::remove_all(pcd_dir_path_);
            fs::create_directories(pcd_dir_path_);
            
            RCLCPP_INFO(node_->get_logger(), "PCD 디렉토리 초기화 완료");
        }
        
        // 포즈 파일 초기화
        if (clear_pose_file && fs::exists(pose_file_path_)) {
            RCLCPP_INFO(node_->get_logger(), "포즈 파일 초기화 중: %s", pose_file_path_.c_str());
            
            // 파일 삭제 또는 내용 비우기
            std::ofstream file(pose_file_path_, std::ios::trunc);
            file.close();
            
            RCLCPP_INFO(node_->get_logger(), "포즈 파일 초기화 완료");
        }
        
        return true;
    } catch (const std::exception& e) {
        logError("맵 데이터 초기화 실패: " + std::string(e.what()));
        return false;
    }
}

bool MapDatabase::saveKeyframe(int keyframe_id, const PointTypePose& pose, double timestamp, 
                             const pcl::PointCloud<PointType>::Ptr& cloud)
{
    try {
        // 키프레임 포즈 저장
        std::ofstream poseFile(pose_file_path_, std::ios::app);
        if (!poseFile.is_open()) {
            logError("포즈 파일 열기 실패: " + pose_file_path_);
            return false;
        }

        // ID, X, Y, Z, Roll, Pitch, Yaw, Timestamp 형식으로 저장
        poseFile << keyframe_id << " "
                << pose.x << " " << pose.y << " " << pose.z << " "
                << pose.roll << " " << pose.pitch << " " << pose.yaw << " "
                << timestamp << std::endl;
        poseFile.close();

        // 포인트 클라우드 저장
        std::string cloud_filename = generateCloudFilename(keyframe_id);
        std::string full_cloud_path = cloud_storage_path_ + "/" + cloud_filename;
        
        if (!savePointCloudToPCD(full_cloud_path, cloud)) {
            logError("클라우드 저장 실패: " + full_cloud_path);
            return false;
        }

        // 간소화된 로그 메시지 (상위 함수에서 더 자세한 로그를 출력하므로 최소화)
        RCLCPP_DEBUG(node_->get_logger(), "[DB] 키프레임 %d 저장 완료 (%zu 포인트)", 
                    keyframe_id, cloud->size());
        return true;
    }
    catch (const std::exception& e) {
        logError("키프레임 저장 실패: " + std::string(e.what()));
        return false;
    }
}

bool MapDatabase::loadKeyframe(int keyframe_id, PointTypePose& pose, pcl::PointCloud<PointType>::Ptr& cloud)
{
    try {
        // 포즈 파일에서 키프레임 정보 찾기
        std::ifstream poseFile(pose_file_path_);
        if (!poseFile.is_open()) {
            logError("포즈 파일 열기 실패: " + pose_file_path_);
            return false;
        }
        
        bool found = false;
        std::string line;
        int id;
        double time;
        
        while (std::getline(poseFile, line)) {
            std::istringstream iss(line);
            if (iss >> id >> pose.x >> pose.y >> pose.z >> pose.roll >> pose.pitch >> pose.yaw >> time) {
                if (id == keyframe_id) {
                    pose.intensity = static_cast<float>(keyframe_id);
                    pose.time = static_cast<float>(time);
                    found = true;
                    break;
                }
            }
        }
        
        poseFile.close();
        
        if (!found) {
            logError("키프레임 ID " + std::to_string(keyframe_id) + " 를 찾을 수 없음");
            return false;
        }
        
        // 포인트 클라우드 로드
        std::string cloud_filename = generateCloudFilename(keyframe_id);
        std::string full_cloud_path = cloud_storage_path_ + "/" + cloud_filename;
        
        if (!loadPointCloudFromPCD(full_cloud_path, cloud)) {
            logError("클라우드 로드 실패: " + full_cloud_path);
            return false;
        }
        
        // 간소화된 로그 메시지
        RCLCPP_DEBUG(node_->get_logger(), "[DB] 키프레임 %d 로드 완료 (%zu 포인트)", 
                    keyframe_id, cloud->size());
        return true;
    }
    catch (const std::exception& e) {
        logError("키프레임 로드 실패: " + std::string(e.what()));
        return false;
    }
}

int MapDatabase::querySurroundingKeyframes(float center_x, float center_y, float center_z, 
                                         float radius, std::vector<int>& result_ids)
{
    result_ids.clear();
    
    try {
        std::vector<PointTypePose> poses;
        loadAllKeyframePoses(poses);
        
        for (const auto& pose : poses) {
            float dx = pose.x - center_x;
            float dy = pose.y - center_y;
            float dz = pose.z - center_z;
            float distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance <= radius) {
                result_ids.push_back(static_cast<int>(pose.intensity));
            }
        }
        
        return static_cast<int>(result_ids.size());
    }
    catch (const std::exception& e) {
        logError("주변 키프레임 검색 실패: " + std::string(e.what()));
        return 0;
    }
}

bool MapDatabase::deleteKeyframe(int keyframe_id)
{
    try {
        // 포인트 클라우드 파일 삭제
        std::string cloud_filename = generateCloudFilename(keyframe_id);
        std::string full_cloud_path = cloud_storage_path_ + "/" + cloud_filename;
        
        if (fs::exists(full_cloud_path)) {
            fs::remove(full_cloud_path);
        }
        
        // 포즈 파일에서 해당 키프레임 정보 삭제 (임시 파일 생성 방식)
        std::ifstream inputFile(pose_file_path_);
        if (!inputFile.is_open()) {
            logError("포즈 파일 열기 실패: " + pose_file_path_);
            return false;
        }
        
        std::string tmpFilePath = pose_file_path_ + ".tmp";
        std::ofstream outputFile(tmpFilePath);
        
        std::string line;
        int id;
        
        while (std::getline(inputFile, line)) {
            std::istringstream iss(line);
            if (iss >> id) {
                if (id != keyframe_id) {
                    outputFile << line << std::endl;
                }
            }
        }
        
        inputFile.close();
        outputFile.close();
        
        // 임시 파일을 원본 파일로 대체 (std::rename 사용)
        if (std::rename(tmpFilePath.c_str(), pose_file_path_.c_str()) != 0) {
            logError("임시 파일 이름 변경 실패: " + tmpFilePath);
            return false;
        }
        
        return true;
    }
    catch (const std::exception& e) {
        logError("키프레임 삭제 실패: " + std::string(e.what()));
        return false;
    }
}

int MapDatabase::pruneOldKeyframes(int max_frames)
{
    try {
        std::vector<PointTypePose> poses;
        loadAllKeyframePoses(poses);
        
        int total_count = static_cast<int>(poses.size());
        if (total_count <= max_frames) {
            return 0;
        }
        
        // 타임스탬프 기준으로 정렬
        std::sort(poses.begin(), poses.end(), [](const PointTypePose& a, const PointTypePose& b) {
            return a.time < b.time;
        });
        
        int to_delete = total_count - max_frames;
        int deleted_count = 0;
        
        for (int i = 0; i < to_delete; ++i) {
            int keyframe_id = static_cast<int>(poses[i].intensity);
            if (deleteKeyframe(keyframe_id)) {
                deleted_count++;
            }
        }
        
        return deleted_count;
    }
    catch (const std::exception& e) {
        logError("오래된 키프레임 삭제 실패: " + std::string(e.what()));
        return 0;
    }
}

int MapDatabase::getKeyframeCount()
{
    try {
        std::ifstream poseFile(pose_file_path_);
        if (!poseFile.is_open()) {
            return 0;
        }
        
        int count = 0;
        std::string line;
        
        while (std::getline(poseFile, line)) {
            count++;
        }
        
        poseFile.close();
        return count;
    }
    catch (const std::exception& e) {
        logError("키프레임 개수 계산 실패: " + std::string(e.what()));
        return 0;
    }
}

int MapDatabase::loadAllKeyframePoses(std::vector<PointTypePose>& poses)
{
    poses.clear();
    
    try {
        std::ifstream poseFile(pose_file_path_);
        if (!poseFile.is_open()) {
            logError("포즈 파일 열기 실패: " + pose_file_path_);
            return 0;
        }
        
        std::string line;
        int id;
        double time;
        
        while (std::getline(poseFile, line)) {
            std::istringstream iss(line);
            PointTypePose pose;
            
            if (iss >> id >> pose.x >> pose.y >> pose.z >> pose.roll >> pose.pitch >> pose.yaw >> time) {
                pose.intensity = static_cast<float>(id);
                pose.time = static_cast<float>(time);
                poses.push_back(pose);
            }
        }
        
        poseFile.close();
        return static_cast<int>(poses.size());
    }
    catch (const std::exception& e) {
        logError("모든 키프레임 포즈 로드 실패: " + std::string(e.what()));
        return 0;
    }
}

int MapDatabase::getRecentKeyframeIds(int count, std::vector<int>& result_ids)
{
    result_ids.clear();
    
    try {
        std::vector<PointTypePose> poses;
        loadAllKeyframePoses(poses);
        
        // 타임스탬프 기준으로 내림차순 정렬
        std::sort(poses.begin(), poses.end(), [](const PointTypePose& a, const PointTypePose& b) {
            return a.time > b.time;
        });
        
        int n = std::min(count, static_cast<int>(poses.size()));
        
        for (int i = 0; i < n; ++i) {
            result_ids.push_back(static_cast<int>(poses[i].intensity));
        }
        
        return static_cast<int>(result_ids.size());
    }
    catch (const std::exception& e) {
        logError("최근 키프레임 ID 가져오기 실패: " + std::string(e.what()));
        return 0;
    }
}

std::string MapDatabase::generateCloudFilename(int keyframe_id)
{
    std::stringstream ss;
    ss << "cloud_" << std::setw(6) << std::setfill('0') << keyframe_id << ".pcd";
    return ss.str();
}

bool MapDatabase::savePointCloudToPCD(const std::string& filename, const pcl::PointCloud<PointType>::Ptr& cloud)
{
    try {
        pcl::io::savePCDFileBinary(filename, *cloud);
        return true;
    } catch (const std::exception& e) {
        logError("PCD 파일 저장 실패: " + std::string(e.what()));
        return false;
    }
}

bool MapDatabase::loadPointCloudFromPCD(const std::string& filename, pcl::PointCloud<PointType>::Ptr& cloud)
{
    if (!cloud) {
        cloud.reset(new pcl::PointCloud<PointType>());
    }
    
    try {
        if (pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1) {
            logError("PCD 파일 로드 실패: " + filename);
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        logError("PCD 파일 로드 실패: " + std::string(e.what()));
        return false;
    }
}

void MapDatabase::logError(const std::string& message)
{
    if (node_) {
        RCLCPP_ERROR(node_->get_logger(), "MapDatabase: %s", message.c_str());
    } else {
        std::cerr << "MapDatabase Error: " << message << std::endl;
    }
}

bool MapDatabase::isKeyframeInMemory(int keyframe_id, 
        const std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map)
{
    return keyframe_map.find(keyframe_id) != keyframe_map.end();
}

int MapDatabase::loadSurroundingKeyframes(
    float center_x, float center_y, float center_z, float radius,
    std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map,
    int max_keyframes)
{
    try {
        RCLCPP_INFO(node_->get_logger(), "[주변검색] 위치(%.2f, %.2f, %.2f) 반경 %.2f 내 키프레임 검색", 
                   center_x, center_y, center_z, radius);
        
        // 주변 키프레임 ID 검색
        std::vector<int> nearby_ids;
        int found = querySurroundingKeyframes(center_x, center_y, center_z, radius, nearby_ids);
        
        if (found == 0) {
            RCLCPP_INFO(node_->get_logger(), "[주변검색] 반경 내 키프레임 없음");
            return 0;
        }
        
        // 로드할 최대 개수 제한
        if (max_keyframes > 0 && found > max_keyframes) {
            RCLCPP_INFO(node_->get_logger(), "[주변검색] 키프레임 %d개 중 최대 %d개로 제한", 
                       found, max_keyframes);
            nearby_ids.resize(max_keyframes);
        } else {
            RCLCPP_INFO(node_->get_logger(), "[주변검색] 반경 내 키프레임 %d개 발견", found);
        }
        
        // 이미 메모리에 있는 키프레임은 건너뛰고 필요한 키프레임만 로드
        std::vector<int> needed_ids;
        for (int id : nearby_ids) {
            if (!isKeyframeInMemory(id, keyframe_map)) {
                needed_ids.push_back(id);
            }
        }
        
        int memory_count = nearby_ids.size() - needed_ids.size();
        if (memory_count > 0) {
            RCLCPP_INFO(node_->get_logger(), "[주변검색] %d개 키프레임은 이미 메모리에 있음, %zu개 로드 필요", 
                       memory_count, needed_ids.size());
        }
        
        return loadRequestedKeyframes(needed_ids, keyframe_map);
    }
    catch (const std::exception& e) {
        logError("주변 키프레임 로드 실패: " + std::string(e.what()));
        return 0;
    }
}

int MapDatabase::loadRequestedKeyframes(
    const std::vector<int>& keyframe_ids,
    std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map)
{
    int loaded_count = 0;
    
    if (keyframe_ids.empty()) {
        return 0;
    }
    
    RCLCPP_INFO(node_->get_logger(), "[요청] %zu개 키프레임 로드 요청 받음", keyframe_ids.size());
    
    for (int id : keyframe_ids) {
        // 이미 메모리에 있는지 확인
        if (isKeyframeInMemory(id, keyframe_map)) {
            RCLCPP_DEBUG(node_->get_logger(), "키프레임 %d 이미 메모리에 있음, 스킵", id);
            continue;
        }
        
        // 디스크에서 키프레임 로드
        PointTypePose pose;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        
        if (loadKeyframe(id, pose, cloud)) {
            // 맵에 추가
            pcl::PointCloud<PointType> cloud_copy;
            pcl::copyPointCloud(*cloud, cloud_copy);
            keyframe_map[id] = std::make_pair(cloud_copy, pose);
            loaded_count++;
        }
    }
    
    if (loaded_count > 0) {
        RCLCPP_INFO(node_->get_logger(), "[요청완료] 디스크에서 %d/%zu개 키프레임 로드됨", 
                   loaded_count, keyframe_ids.size());
    } else if (!keyframe_ids.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[로드실패] 요청된 %zu개 키프레임 중 로드된 것 없음", keyframe_ids.size());
    }
    
    return loaded_count;
}

int MapDatabase::updateKeyframeMap(
    const std::map<int, PointTypePose>& keyframe_poses,
    std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map)
{
    int updated_count = 0;
    
    for (const auto& pose_entry : keyframe_poses) {
        int id = pose_entry.first;
        const PointTypePose& new_pose = pose_entry.second;
        
        // 메모리에 있는 키프레임인지 확인
        auto it = keyframe_map.find(id);
        if (it != keyframe_map.end()) {
            // 포즈 업데이트
            it->second.second = new_pose;
            updated_count++;
        }
    }
    
    return updated_count;
} 