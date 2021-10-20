#include "pdl.h"
#include "VL53L1X.h"

VL53L1X sensor;

void pdlSetupLidar(pdlDroneState *ds)
{
  pdlSetLidarMaxRange(3.f);
  pdlSetLidarUpdatePeriod(100000);

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

void pdlReadLidar(pdlDroneState *ds)
{
  if(sensor.dataReady())
  {
    sensor.read(false);

    if(sensor.ranging_data.range_status == VL53L1X::RangeValid)
    {
      ds->lidarRange = sensor.ranging_data.range_mm;
      ds->lidarRange /= 1000.f;
    }
    else
    {
      ds->lidarRange = -1.f;
    }
  }
}
