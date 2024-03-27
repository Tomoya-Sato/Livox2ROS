#include <iostream>
#include <fstream>

#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include "livox_def.h"
#include "point_type.h"

const float gyro_scale = 9.80665;

double convertTimestamp(uint8_t* data)
{
  uint64_t timestamp = 0;
  for (int i = 0; i < 8; i++)
  {
    timestamp |= (uint64_t)data[i] << (i * 8);
  }
  return timestamp / 1000000000.0;
}

int main(int argc, char** argv)
{
  std::string input_file(argv[1]);
  std::string output_file(argv[2]);

  std::ifstream ifs(input_file, std::ios::binary);

  rosbag::Bag bag;
  bag.open(output_file, rosbag::bagmode::Write);

  CompactHeader header;
  LivoxLidarCartesianHighRawPoint high_point;
  LivoxLidarImuRawPoint imu;

  pcl::PointCloud<PointXYZIRT> cloud;
  double base_stamp = -1;

  const double points_interval = 0.1;
  unsigned int lidar_seq = 0;
  unsigned int imu_seq = 0;

  while (ifs.read(reinterpret_cast<char*>(&header), sizeof(header)))
  {
    double current_stamp = header.stamp;
    std::cout << std::setprecision(15) << current_stamp << std::endl;

    if (base_stamp < 0)
    {
      base_stamp = current_stamp;
    }

    if (header.data_type == kLivoxLidarCartesianCoordinateHighData)
    {
      cloud.reserve(cloud.size() + header.dot_num);
      if (current_stamp - base_stamp > points_interval)
      {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = ros::Time(base_stamp);
        msg.header.frame_id = "livox";
        msg.header.seq = lidar_seq++;
        bag.write("livox/lidar", ros::Time(base_stamp), msg);
        base_stamp = current_stamp;
        cloud.clear();
        cloud.reserve(header.dot_num);
      }

      for (int i = 0; i < header.dot_num; i++)
      {
        ifs.read(reinterpret_cast<char*>(&high_point), sizeof(high_point));
        PointXYZIRT point;
        point.x = high_point.x / 1000.0;
        point.y = high_point.y / 1000.0;
        point.z = high_point.z / 1000.0;
        point.intensity = high_point.reflectivity;
        point.ring = high_point.tag;
        point.time = current_stamp - base_stamp;
        cloud.push_back(point);
      }
    }

    else if (header.data_type == kLivoxLidarImuData)
    {
      LivoxLidarImuRawPoint imu;
      ifs.read(reinterpret_cast<char*>(&imu), sizeof(imu));

      sensor_msgs::Imu msg;
      msg.header.stamp = ros::Time(current_stamp);
      msg.header.frame_id = "livox";
      msg.header.seq = imu_seq++;
      msg.angular_velocity.x = imu.gyro_x;
      msg.angular_velocity.y = imu.gyro_y;
      msg.angular_velocity.z = imu.gyro_z;
      msg.linear_acceleration.x = imu.acc_x * gyro_scale;
      msg.linear_acceleration.y = imu.acc_y * gyro_scale;
      msg.linear_acceleration.z = imu.acc_z * gyro_scale;

      bag.write("livox/imu", ros::Time(current_stamp), msg);
    }
  }

  bag.close();

  return 0;
}
