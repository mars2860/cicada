#include "pdl.h"
#include <Math.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#include "RTFusion.h"
#include "RTFusionRTQF.h"
#include "RTFusionKalman4.h"

MPU6050 mpu;

RTFusion* imuFusion;
RTIMU_DATA fusionData;

void imuReadAccelOffset(pdlDroneState *ds);
void imuReadGyroOffset(pdlDroneState *ds);

void pdlSetupAccel(pdlDroneState *ds)
{
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setSleepEnabled(false);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setRate(4); // 200 Hz
  // I2CDev has a bug. Take a look at mpu6050.cpp
  // if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= Gravity; //remove Gravity
  // MPU6050::PID
  // It causes wrong calibration of accel
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  mpu.setXAccelOffset(ds->accel.offset[PDL_X]);
  mpu.setYAccelOffset(ds->accel.offset[PDL_Y]);
  mpu.setZAccelOffset(ds->accel.offset[PDL_Z]);

  imuReadAccelOffset(ds);

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
}

void pdlSetupGyro(pdlDroneState *ds)
{
  mpu.setXGyroOffset(ds->gyro.offset[PDL_X]);
  mpu.setYGyroOffset(ds->gyro.offset[PDL_Y]);
  mpu.setZGyroOffset(ds->gyro.offset[PDL_Z]);

  imuReadGyroOffset(ds);
}

void imuReadAccelOffset(pdlDroneState *ds)
{
  ds->accel.offset[PDL_X] = mpu.getXAccelOffset();
  ds->accel.offset[PDL_Y] = mpu.getYAccelOffset();
  ds->accel.offset[PDL_Z] = mpu.getZAccelOffset();
}

void imuReadGyroOffset(pdlDroneState *ds)
{
  ds->gyro.offset[PDL_X] = mpu.getXGyroOffset();
  ds->gyro.offset[PDL_Y] = mpu.getYGyroOffset();
  ds->gyro.offset[PDL_Z] = mpu.getZGyroOffset();
}

void pdlCalibrateAccel(pdlDroneState *ds)
{
  mpu.CalibrateAccel(25);

  imuReadAccelOffset(ds);
}

void pdlCalibrateGyro(pdlDroneState *ds)
{
  mpu.CalibrateGyro(25);

  imuReadGyroOffset(ds);
}

void pdlReadAccel(pdlDroneState *ds)
{
  // disadvantage of this method is no temperature compensation for gyro and accel
  // and result is not so clean as in DMP, there are more fluctuations.
  // values are invalid when motors run high speed. it is very sensetive to vibrations
  // advantage of this method is fast execute. it takes 5ms to execute to read all data
  // read DMP results takes more time

  mpu.getMotion6( &ds->accel.raw[PDL_X],
                  &ds->accel.raw[PDL_Y],
                  &ds->accel.raw[PDL_Z],
                  &ds->gyro.raw[PDL_X],
                  &ds->gyro.raw[PDL_Y],
                  &ds->gyro.raw[PDL_Z]);

  for(uint8_t i = 0; i < 3; i++)
  {
    ds->accel.pure[i] = (float)(ds->accel.raw[i]) / 8192.f;
    ds->gyro.pure[i] = (float)(ds->gyro.raw[i]) / 16.4f;
    ds->gyro.pure[i] *= 3.1415926535f / 180.f;
  }
}

void pdlReadGyro(pdlDroneState*) {}

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