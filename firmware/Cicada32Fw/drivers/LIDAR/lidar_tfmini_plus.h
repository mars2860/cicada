#ifndef DRIVERS_LIDAR_LIDAR_TFMINI_PLUS_H_
#define DRIVERS_LIDAR_LIDAR_TFMINI_PLUS_H_

#include "lidar.h"
#include "tfmini_plus.h"

class TFminiPlusDriver: public LidarDriver
{
private:
  const float TFMINI_FOV = 3.6f * PI/180.f;   // field of view for tfmini in rad
  TFminiPlus tf;
  tfminiplus_data_t data;
public:
  bool setup()
  {
    tf.begin();

    tfminiplus_version_t version = tf.get_version();

    if(version.major == 0)
    {
      return false;
    }

    LOG_INFO("TFmini v%i.%i.%i",version.major,version.minor,version.revision);

    return true;
  }

  bool isDataReady()
  {
    return tf.read_data(data, true);
  }

  float getRange()
  {
    float range = -1.f;
    if(data.strength > 100 && data.strength != 0XFFFF)
    {
      range = data.distance;
      range /= 1000.f;
    }
    return range;
  }

  float getFOV()
  {
    return TFMINI_FOV;
  }
};


#endif /* DRIVERS_LIDAR_LIDAR_TFMINI_PLUS_H_ */
