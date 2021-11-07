#include "pdl.h"

#include <ESP8266WiFi.h>
#include <Wire.h>
#include "RTFusion.h"
#include "RTFusionRTQF.h"
#include "RTFusionKalman4.h"
#include "movingAvg.h"

//-----------------------------------------------------------------------------
// INSTALL
// 1. https://github.com/enjoyneering/ESP8266-I2C-Driver

//---------------------------------------------------------------
// VARIABLES

pdlDroneState droneState;
RTFusion* imuFusion;
RTIMU_DATA fusionData;
movingAvg avgLoopTime(10);

uint32_t pdlMicros()
{
  return micros();
}

//--------------------------------------------------------------
// INTERRUPTS

//--------------------------------------------------------------

void setup()
{
  memset(&droneState, 0, sizeof(droneState));

  Wire.begin();
  Wire.setClock(200000UL);  // line capacity is too high, can't run at higher frequency

  pdlSetup(&droneState);

  //imuFusion = new RTFusionRTQF();
  imuFusion = new RTFusionKalman4();
  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  imuFusion->setSlerpPower(0.02f);
  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  imuFusion->setGyroEnable(true);
  imuFusion->setAccelEnable(true);
  imuFusion->setCompassEnable(false);

  avgLoopTime.begin();
}

//-----------------------------------------------------------------------------

void loop()
{
  static uint32_t loopStartTime = 0;
  uint32_t loopTime = 0;

  loopStartTime = micros();

  pdlUpdate(&droneState);

  loopTime = micros() - loopStartTime;

  if(loopTime > droneState.maxLoopTime)
    droneState.maxLoopTime = loopTime;

  droneState.avgLoopTime = avgLoopTime.reading(loopTime);
}

void pdlSetupMagneto(pdlDroneState*)
{
  // nothing
}

void pdlReadMagneto(pdlDroneState*)
{
  // nothing
}

void pdlTripleAxisSensorFusion(pdlDroneState *ds)
{
  // sort out axes as in RTIMUMPU9250

  fusionData.fusionPoseValid = false;
  fusionData.fusionQPoseValid = false;
  fusionData.timestamp = ds->timestamp;

  fusionData.gyro = RTVector3(ds->gyro.pure[PDL_X], ds->gyro.pure[PDL_Y], ds->gyro.pure[PDL_Z]);
  fusionData.accelValid = true;
  fusionData.accel = RTVector3(ds->accel.pure[PDL_X], ds->accel.pure[PDL_Y], ds->accel.pure[PDL_Z]);
  fusionData.compassValid = false;
  fusionData.compass = RTVector3(ds->magneto.pure[PDL_X], ds->magneto.pure[PDL_Y], -ds->magneto.pure[PDL_Z]);
  fusionData.pressureValid = false;
  fusionData.temperatureValid = false;
  fusionData.humidityValid = false;
  imuFusion->newIMUData(fusionData, 0);
  const RTVector3 &pose = fusionData.fusionPose;
  ds->yaw = pose.z();
  ds->pitch = pose.y();
  ds->roll = pose.x();

  if(fusionData.compassValid)
  {
    if(imuFusion->getCompassEnable())
      ds->heading = pose.z() + RTMATH_PI;
  }
  else
  {
    ds->heading = 0;
  }
}
