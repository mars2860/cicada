#include "pdl.h"
#include "VL53L1X.h"
#include <math.h>

VL53L1X sensor;
const float fov = 27.f * 3.14f/180.f; // field of view for vl53l1x in rad

void pdlSetupLidar(pdlDroneState *ds)
{
  delay(500);

  if(sensor.init())
  {
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.setTimeout(500);
    sensor.startContinuous(50);
  }
  else
  {
    ds->lidarRange = -1.f;
  }
}

uint8_t pdlReadLidar(pdlDroneState *ds)
{
  static float oldRange = 0;
  static uint32_t lastUpdate = 0;


  if(sensor.dataReady())
  {
    sensor.read(false);

    if(sensor.ranging_data.range_status == VL53L1X::RangeValid)
    {
      ds->lidarRange = sensor.ranging_data.range_mm;
      ds->lidarRange /= 1000.f;
      // tilt compensation
      float a = acosf(cosf(ds->pitch) * cosf(ds->roll));
      a = fabsf(a);
      if(a >= fov / 2.f)
      {
        ds->lidarRange = ds->lidarRange * cosf(a - fov/2.f);
      }
      // calc vertical velocity
      float dt = pdlGetDeltaTime(pdlMicros(), lastUpdate);
      lastUpdate = pdlMicros();
      dt /= 1000000.f;
      ds->velocity[PDL_Z] = (ds->lidarRange - oldRange) / dt;
      oldRange = ds->lidarRange;

      if(ds->lidarRange > -0.1f && ds->lidarRange < PDL_LIDAR_MAX_RANGE)
      {
        ds->altitude = ds->lidarRange;
        return 1;
      }
    }
    else
    {
      ds->lidarRange = -1.f;
    }
  }

  return 0;
}
