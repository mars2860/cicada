#ifndef DRIVERS_LIDAR_SONAR_HCSR04_H_
#define DRIVERS_LIDAR_SONAR_HCSR04_H_

#include "lidar.h"
#include "hcsr04_i2c.h"

class HCSR04Driver: public LidarDriver
{
private:
  const float HCSR04_FOV = 15.f * PI/180.f;   // field of view for tfmini in rad
  HCSR04_I2C sonar;
public:
  bool setup()
  {
    bool result = false;
    for(uint8_t i = 0; i < 5; i++)
    {
      if(getRange() > 0)
      {
        result = true;
        break;
      }
    }
    if(result)
    {
      LOG_INFO("HCSR04");
    }
    return result;
  }

  bool isDataReady()
  {
    return true;
  }

  float getRange()
  {
    Wire.flush();
    Wire.setClock(100000UL);

    float range = sonar.getRange_m();
    sonar.measure();

    Wire.setClock(400000UL);
    Wire.flush();

    return range;
  }

  float getFOV()
  {
    return HCSR04_FOV;
  }
};


#endif /* DRIVERS_LIDAR_SONAR_HCSR04_H_ */
