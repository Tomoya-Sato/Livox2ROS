#ifndef LIVOX_DEF_H
#define LIVOX_DEF_H

#pragma pack(1)

typedef struct {
  uint16_t dot_num;
  uint8_t data_type;
  double stamp;
} CompactHeader;

typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;      /**< unit: 0.1 us */
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;
  uint8_t timestamp[8];
  uint8_t data[1];             /**< Point cloud data. */
} LivoxLidarEthernetPacket;

typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
} LivoxLidarImuRawPoint;

typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarCartesianHighRawPoint;

typedef struct {
  int16_t x;            /**< X axis, Unit:cm */
  int16_t y;            /**< Y axis, Unit:cm */
  int16_t z;            /**< Z axis, Unit:cm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxLidarCartesianLowRawPoint;

typedef struct {
  uint32_t depth;
  uint16_t theta;
  uint16_t phi;
  uint8_t reflectivity;
  uint8_t tag;
} LivoxLidarSpherPoint;

typedef enum {
  kLivoxLidarImuData = 0,
  kLivoxLidarCartesianCoordinateHighData = 0x01,
  kLivoxLidarCartesianCoordinateLowData = 0x02,
  kLivoxLidarSphericalCoordinateData = 0x03
} LivoxLidarPointDataTy;

#endif // LIVOX_DEF_H
