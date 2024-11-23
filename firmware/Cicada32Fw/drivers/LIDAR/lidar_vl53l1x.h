#ifndef DRIVERS_LIDAR_LIDAR_VL53L1X_H_
#define DRIVERS_LIDAR_LIDAR_VL53L1X_H_

#include "lidar.h"
#include "vl53l1x_class.h"

class VL53L1xDriver: public LidarDriver
{
private:
  const float VL53L1X_FOV = 27.f * PI/180.f;   // field of view for vl53l1x in rad
  VL53L1X vl;
public:
  VL53L1xDriver(): vl(&Wire,-1) {};

  bool setup()
  {
    if(vl.Init() != 0)
      return false;

    uint16_t sensorId;
    vl.VL53L1X_GetSensorId(&sensorId);

    if(sensorId == 0)
      return false;

    LOG_INFO("VL53L1X id=%i",sensorId);

    /*  Timing budget (TB)
        The VL53L1X timing budget can be set from 20 ms up to 1000 ms.
         • 20 ms is the minimum timing budget and can be used only in Short distance mode.
         • 33 ms is the minimum timing budget which can work for all distance modes.
         • 140 ms is the timing budget which allows the maximum distance of 4 m (in the dark on
        a white chart) to be reached under Long distance mode
        Increasing the timing budget increases the maximum distance the device can range and improves
        the repeatability error. However, average power consumption augments accordingly.
     */
     /* Ranging is continuous, with a programmable delay between two ranging operations
        (called an inter-measurement period). Ranging duration (timing budget) is also
        programmable.
     */

    // Long mode 1
    vl.VL53L1X_SetDistanceMode(2);
    vl.VL53L1X_SetTimingBudgetInMs(100);
    vl.VL53L1X_SetInterMeasurementInMs(100);

    // Short mode 1
    //vl.VL53L1X_SetDistanceMode(1);
    //vl.VL53L1X_SetTimingBudgetInMs(100);
    //vl.VL53L1X_SetInterMeasurementInMs(100);

    /*
    // Long mode 2
    vl.VL53L1X_SetDistanceMode(2);
    vl.VL53L1X_SetTimingBudgetInMs(200);
    vl.VL53L1X_SetInterMeasurementInMs(200);
    */

    /*
    // Short mode 2
    vl.VL53L1X_SetDistanceMode(1);
    vl.VL53L1X_SetTimingBudgetInMs(50);
    vl.VL53L1X_SetInterMeasurementInMs(50);
    */
    // work in single shot mode, because update period is not constant
    vl.VL53L1X_ClearInterrupt();
    vl.VL53L1X_StartSingle();

    return true;
  }

  bool isDataReady()
  {
    uint8_t dataReady = 0;
    uint32_t startTimeout = micros();

    while(micros() - startTimeout < 2000)
    {
      vl.VL53L1X_CheckForDataReady(&dataReady);
      if(dataReady)
      {
        return true;
      }
    }

    return false;
  }

  float getRange()
  {
    float range = -1.f;
    uint8_t rangeStatus = 0;
    uint16_t distance = 0;

    vl.VL53L1X_GetDistance(&distance);
    vl.VL53L1X_GetRangeStatus(&rangeStatus);

    if(rangeStatus == 0)
    {
      range = distance;
      range /= 1000.f;
    }

    vl.VL53L1X_ClearInterrupt();
    vl.VL53L1X_StartSingle();

    return range;
  }

  float getFOV()
  {
    return VL53L1X_FOV;
  }
};

#endif /* DRIVERS_LIDAR_LIDAR_VL53L1X_H_ */
