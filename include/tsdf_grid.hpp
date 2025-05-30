// tsdf_grid.hpp
#ifndef TSDF_GRID_HPP
#define TSDF_GRID_HPP

#include <unordered_map>
#include <string>
#include <mutex>
#include <cmath>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

using PointT = pcl::PointXYZ;

struct VoxelData {
  float static_tsdf   = 0.0f;
  float current_tsdf  = 0.0f;
  float weight_static = 0.0f;
  float weight_current= 0.0f;
};

class TSDFGrid {
public:
  TSDFGrid(double voxel_size, double trunc_dist)
  : voxel_size_(voxel_size), truncation_distance_(static_cast<float>(trunc_dist)) {}

  void integrateGlobalCloud(const pcl::PointCloud<PointT>& cloud, const PointT* sensor_origin = nullptr);
  void integrateLocalCloud(const pcl::PointCloud<PointT>& cloud, const PointT& sensor_origin);
  float getTsdfDifference(const PointT& pt) const;
  float getStaticTsdf(const PointT& pt) const;
  float getCurrentTsdf(const PointT& pt) const;
  void resetCurrentTsdf();
  void resetStaticTsdf(); // Static TSDF를 리셋하는 함수 추가

  Eigen::Vector3i getVoxelIndices(const PointT& pt) const {
    return Eigen::Vector3i(
        static_cast<int>(std::floor(pt.x / voxel_size_)),
        static_cast<int>(std::floor(pt.y / voxel_size_)),
        static_cast<int>(std::floor(pt.z / voxel_size_))
    );
  }

private:
  std::string keyFromIndices(const Eigen::Vector3i& indices) const {
    return std::to_string(indices.x()) + "_" + std::to_string(indices.y()) + "_" + std::to_string(indices.z());
  }
  std::string keyFromPoint(const PointT& pt) const {
    return keyFromIndices(getVoxelIndices(pt));
  }
  void updateVoxel(VoxelData& vd, float sdf_observed, bool is_static_integration);

  double voxel_size_;
  float truncation_distance_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, VoxelData> voxels_;
};

#endif // TSDF_GRID_HPP