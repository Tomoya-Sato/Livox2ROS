#ifndef CUSTOM_POINT_TYPE_H
#define CUSTOM_POINT_TYPE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

#endif // CUSTOM_POINT_TYPE_H
