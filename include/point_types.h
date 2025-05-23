#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 기본 포인트 타입 정의
using PointType = pcl::PointXYZI;

// 6D 포즈를 포함하는 포인트 타입 정의
struct PointTypePose
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointTypePose,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw) (float, time, time)
) 