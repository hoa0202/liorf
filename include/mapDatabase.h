#pragma once

#include <string>
#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include "utility.h"
#include "costmap.h"

// 빌드 문제 해결을 위해 명시적으로 사용하는 타입 정의
using PointType = pcl::PointXYZI;
// PointTypePose는 costmap.h에서 이미 정의됨

/**
 * @brief 맵 데이터베이스 관리 클래스
 * 
 * 이 클래스는 LIORF SLAM 시스템에서 생성된 맵 데이터(키프레임, 포인트 클라우드)를
 * 파일 시스템에 저장하고 불러오는 기능을 제공합니다.
 */
class MapDatabase {
public:
    /**
     * @brief 생성자
     * @param node ROS2 노드 포인터 (로깅용)
     * @param base_path 기본 저장 경로
     * @param cloud_storage_path 포인트 클라우드 저장 디렉토리 경로
     */
    MapDatabase(rclcpp::Node* node, const std::string& base_path, const std::string& cloud_storage_path);
    
    /**
     * @brief 소멸자
     */
    ~MapDatabase();
    
    /**
     * @brief 데이터베이스 초기화
     * @return 성공 여부
     */
    bool initialize();
    
    /**
     * @brief 맵 데이터 초기화
     * 
     * 저장된 모든 맵 데이터(포즈 파일, 포인트 클라우드 파일, PCD 파일)를 삭제합니다.
     * 
     * @param clear_cloud_dir 클라우드 디렉토리 초기화 여부
     * @param clear_pcd_dir PCD 디렉토리 초기화 여부
     * @param clear_pose_file 포즈 파일 초기화 여부
     * @return 성공 여부
     */
    bool clearMapData(bool clear_cloud_dir, bool clear_pcd_dir, bool clear_pose_file);
    
    /**
     * @brief 키프레임 저장
     * @param keyframe_id 키프레임 ID
     * @param pose 키프레임 포즈
     * @param timestamp 타임스탬프
     * @param cloud 포인트 클라우드
     * @return 성공 여부
     */
    bool saveKeyframe(int keyframe_id, const PointTypePose& pose, double timestamp, 
                     const pcl::PointCloud<PointType>::Ptr& cloud);
    
    /**
     * @brief 키프레임 로드
     * @param keyframe_id 키프레임 ID
     * @param pose 로드된 포즈를 저장할 변수
     * @param cloud 로드된 포인트 클라우드를 저장할 변수
     * @return 성공 여부
     */
    bool loadKeyframe(int keyframe_id, PointTypePose& pose, pcl::PointCloud<PointType>::Ptr& cloud);
    
    /**
     * @brief 주어진 위치 주변의 키프레임 ID 검색
     * @param center_x 중심 X 좌표
     * @param center_y 중심 Y 좌표
     * @param center_z 중심 Z 좌표
     * @param radius 검색 반경
     * @param result_ids 검색된 키프레임 ID를 저장할 벡터
     * @return 검색된 키프레임 수
     */
    int querySurroundingKeyframes(float center_x, float center_y, float center_z, 
                                 float radius, std::vector<int>& result_ids);
    
    /**
     * @brief 특정 키프레임 삭제
     * @param keyframe_id 삭제할 키프레임 ID
     * @return 성공 여부
     */
    bool deleteKeyframe(int keyframe_id);
    
    /**
     * @brief 오래된 키프레임 삭제
     * @param max_frames 유지할 최대 키프레임 수
     * @return 삭제된 키프레임 수
     */
    int pruneOldKeyframes(int max_frames);
    
    /**
     * @brief 전체 키프레임 개수 반환
     * @return 키프레임 개수
     */
    int getKeyframeCount();
    
    /**
     * @brief 모든 키프레임 포즈 불러오기
     * @param poses 불러온 포즈를 저장할 벡터
     * @return 불러온 포즈 개수
     */
    int loadAllKeyframePoses(std::vector<PointTypePose>& poses);
    
    /**
     * @brief 최근 N개 키프레임 ID 가져오기
     * @param count 가져올 키프레임 수
     * @param result_ids 키프레임 ID를 저장할 벡터
     * @return 가져온 키프레임 수
     */
    int getRecentKeyframeIds(int count, std::vector<int>& result_ids);

    /**
     * @brief 주변 키프레임 로드
     * 
     * 주어진 위치 주변의 키프레임을 디스크에서 로드하여 제공된 맵에 추가합니다.
     * 이미 맵에 존재하는 키프레임은 건너뜁니다.
     * 
     * @param center_x 중심 X 좌표
     * @param center_y 중심 Y 좌표
     * @param center_z 중심 Z 좌표
     * @param radius 검색 반경
     * @param keyframe_map 로드된 키프레임을 저장할 맵 (ID -> 클라우드, 포즈)
     * @param max_keyframes 최대 로드할 키프레임 수 (0=제한 없음)
     * @return 로드된 키프레임 수
     */
    int loadSurroundingKeyframes(
        float center_x, float center_y, float center_z, float radius,
        std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map,
        int max_keyframes = 0);
    
    /**
     * @brief 메모리에 있는 키프레임인지 확인
     * @param keyframe_id 확인할 키프레임 ID
     * @param keyframe_map 메모리 내 키프레임 맵
     * @return 메모리에 있으면 true, 없으면 false
     */
    bool isKeyframeInMemory(int keyframe_id, 
        const std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map);
    
    /**
     * @brief 필요한 키프레임 로드
     * 
     * 주어진 ID의 키프레임을 로드하여 맵에 추가합니다.
     * 이미 맵에 존재하는 키프레임은 건너뜁니다.
     * 
     * @param keyframe_ids 로드할 키프레임 ID 목록
     * @param keyframe_map 로드된 키프레임을 저장할 맵 (ID -> 클라우드, 포즈)
     * @return 로드된 키프레임 수
     */
    int loadRequestedKeyframes(
        const std::vector<int>& keyframe_ids,
        std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map);
    
    /**
     * @brief 키프레임 맵 업데이트
     * 
     * 메모리에 있는 키프레임 맵을 최적화된 포즈로 업데이트합니다.
     * 
     * @param keyframe_poses 최적화된 키프레임 포즈 맵
     * @param keyframe_map 업데이트할 키프레임 맵
     * @return 업데이트된 키프레임 수
     */
    int updateKeyframeMap(
        const std::map<int, PointTypePose>& keyframe_poses,
        std::map<int, std::pair<pcl::PointCloud<PointType>, PointTypePose>>& keyframe_map);

private:
    rclcpp::Node* node_;
    std::string base_path_;
    std::string cloud_storage_path_;
    std::string pose_file_path_;
    std::string pcd_dir_path_;
    
    /**
     * @brief 클라우드 파일 이름 생성
     * @param keyframe_id 키프레임 ID
     * @return 클라우드 파일 이름
     */
    std::string generateCloudFilename(int keyframe_id);
    
    /**
     * @brief 포인트 클라우드를 PCD 파일로 저장
     * @param filename 파일 이름
     * @param cloud 저장할 포인트 클라우드
     * @return 성공 여부
     */
    bool savePointCloudToPCD(const std::string& filename, const pcl::PointCloud<PointType>::Ptr& cloud);
    
    /**
     * @brief PCD 파일에서 포인트 클라우드 로드
     * @param filename 파일 이름
     * @param cloud 로드된 포인트 클라우드를 저장할 변수
     * @return 성공 여부
     */
    bool loadPointCloudFromPCD(const std::string& filename, pcl::PointCloud<PointType>::Ptr& cloud);
    
    /**
     * @brief 에러 로그 출력
     * @param message 에러 메시지
     */
    void logError(const std::string& message);
}; 