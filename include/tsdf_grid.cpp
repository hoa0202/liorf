// tsdf_grid.cpp
#include "tsdf_grid.hpp"
#include <limits>
#include <iostream>

void TSDFGrid::updateVoxel(VoxelData& vd, float sdf_observed, bool is_static_integration) {
    float tsdf_val = std::max(-truncation_distance_, std::min(truncation_distance_, sdf_observed));
    if (is_static_integration) {
        vd.static_tsdf = (vd.static_tsdf * vd.weight_static + tsdf_val) / (vd.weight_static + 1.0f);
        vd.weight_static += 1.0f;
    } else {
        vd.current_tsdf = (vd.current_tsdf * vd.weight_current + tsdf_val) / (vd.weight_current + 1.0f);
        vd.weight_current += 1.0f;
    }
}

void TSDFGrid::integrateGlobalCloud(const pcl::PointCloud<PointT>& cloud, const PointT* sensor_origin) {
  std::lock_guard<std::mutex> lk(mutex_);
  // sensor_origin이 제공되지 않으면, cloud의 포인트들이 이미 표면이라고 가정 (SDF=0)
  // 실제로는 SLAM 시스템이 TSDF를 직접 생성하거나, 더 정교한 통합 방법이 필요할 수 있음
  for (const auto& pt : cloud.points) {
    std::string key = keyFromPoint(pt);
    VoxelData& vd = voxels_[key];
    updateVoxel(vd, 0.0f, true); // static TSDF 업데이트
  }
}

void TSDFGrid::integrateLocalCloud(const pcl::PointCloud<PointT>& cloud, const PointT& sensor_origin) {
  std::lock_guard<std::mutex> lk(mutex_);
  Eigen::Vector3f origin(sensor_origin.x, sensor_origin.y, sensor_origin.z);

  for (const auto& surface_pt_pcl : cloud.points) {
    Eigen::Vector3f surface_pt(surface_pt_pcl.x, surface_pt_pcl.y, surface_pt_pcl.z);
    Eigen::Vector3f ray_direction = (surface_pt - origin).normalized();
    float surface_depth = (surface_pt - origin).norm();
    float step_size = static_cast<float>(voxel_size_ / 2.0);

    for (float current_depth_along_ray = step_size;
         current_depth_along_ray < surface_depth + truncation_distance_;
         current_depth_along_ray += step_size)
    {
        Eigen::Vector3f current_pt_on_ray = origin + ray_direction * current_depth_along_ray;
        PointT voxel_query_pt;
        voxel_query_pt.x = current_pt_on_ray.x();
        voxel_query_pt.y = current_pt_on_ray.y();
        voxel_query_pt.z = current_pt_on_ray.z();
        std::string key = keyFromPoint(voxel_query_pt);
        VoxelData& vd = voxels_[key];
        float sdf_observed = current_depth_along_ray - surface_depth;
        updateVoxel(vd, sdf_observed, false); // current TSDF 업데이트
    }
  }
}

void TSDFGrid::resetCurrentTsdf() {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& pair : voxels_) {
        pair.second.current_tsdf = 0.0f; // 또는 truncation_distance_와 같은 미관측 값
        pair.second.weight_current = 0.0f;
    }
}

void TSDFGrid::resetStaticTsdf() { // 새로운 함수
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& pair : voxels_) {
        pair.second.static_tsdf = 0.0f; // 또는 truncation_distance_
        pair.second.weight_static = 0.0f;
    }
    // 또는 voxels_.clear(); 를 사용하여 맵 자체를 비울 수도 있습니다.
    // 여기서는 가중치만 리셋하여 기존 voxel 키는 유지합니다.
    // 완전히 새로 시작하려면 voxels_.clear();가 더 적합할 수 있습니다.
}


float TSDFGrid::getTsdfDifference(const PointT& pt) const {
  std::string key = keyFromPoint(pt);
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = voxels_.find(key);
  if (it == voxels_.end() || it->second.weight_static < 0.1f ) { // Static 정보가 없으면 비교 불가
    return 0.0f; // 또는 매우 큰 값을 반환하여 무조건 동적으로 처리 (상황에 따라 다름)
                 // 여기서는 static 정보가 없으면 동적이라 판단하지 않음
  }
  if (it->second.weight_current < 0.1f) { // Current 정보가 없으면 (예: 현재 시야 밖에 있음)
      // Static에는 있지만 Current에 없으면, 동적일 가능성 (사라짐)
      // current_tsdf가 unobserved 값(truncation_distance_)을 가질 것이므로 차이가 크게 나올 것
      // 또는 static_tsdf가 0에 가깝고, current_tsdf가 free space(양수)라면 그 차이로 판단
      return std::abs(truncation_distance_ - it->second.static_tsdf); // 현재 관측 안됨을 가정
  }
  const VoxelData& vd = it->second;
  return std::abs(vd.current_tsdf - vd.static_tsdf);
}

float TSDFGrid::getStaticTsdf(const PointT& pt) const {
    std::string key = keyFromPoint(pt);
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = voxels_.find(key);
    if (it == voxels_.end() || it->second.weight_static < 0.1f) {
        return truncation_distance_;
    }
    return it->second.static_tsdf;
}

float TSDFGrid::getCurrentTsdf(const PointT& pt) const {
    std::string key = keyFromPoint(pt);
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = voxels_.find(key);
    if (it == voxels_.end() || it->second.weight_current < 0.1f) {
        return truncation_distance_;
    }
    return it->second.current_tsdf;
}