#ifndef DRIVERS_LIDAR_LIDAR_H_
#define DRIVERS_LIDAR_LIDAR_H_

#include "pdl.h"

class LidarDriver
{
public:
  virtual ~LidarDriver() {};
  virtual bool setup() = 0;
  virtual bool isDataReady() = 0;
  virtual float getRange() = 0;
  virtual float getFOV() = 0;
};

#endif /* DRIVERS_LIDAR_LIDAR_H_ */
