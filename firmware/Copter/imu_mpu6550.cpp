#include "pdl.h"
#include <Math.h>

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

//SampleFilter axFilter, ayFilter, azFilter, gxFilter, gyFilter, gzFilter;
/*Filter axFilter(20.0, 0.005, IIR::ORDER::OD3);
Filter ayFilter(20.0, 0.005, IIR::ORDER::OD3);
Filter azFilter(20.0, 0.005, IIR::ORDER::OD3);
Filter gxFilter(20.0, 0.005, IIR::ORDER::OD3);
Filter gyFilter(20.0, 0.005, IIR::ORDER::OD3);
Filter gzFilter(20.0, 0.005, IIR::ORDER::OD3);*/

void imuReadAccelOffset(pdlDroneState *ds);
void imuReadGyroOffset(pdlDroneState *ds);

void pdlSetupAccel(pdlDroneState *ds)
{
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setSleepEnabled(false);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setRate(4); pdlSetImuReadPeriod(5000);  // 200 Hz
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

void imuCalibrateAccel(pdlDroneState *ds)
{
  mpu.CalibrateAccel(25);

  imuReadAccelOffset(ds);
}

void imuCalibrateGyro(pdlDroneState *ds)
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
    /*SampleFilter_put(&axFilter, accel[0]);
    SampleFilter_put(&ayFilter, accel[1]);
    SampleFilter_put(&azFilter, accel[2]);

    SampleFilter_put(&gxFilter, gyro[0]);
    SampleFilter_put(&gyFilter, gyro[1]);
    SampleFilter_put(&gzFilter, gyro[2]);

    accel[0] = (int16_t)SampleFilter_get(&axFilter);
    accel[1] = (int16_t)SampleFilter_get(&ayFilter);
    accel[2] = (int16_t)SampleFilter_get(&azFilter);

    gyro[0] = (int16_t)SampleFilter_get(&gxFilter);
    gyro[1] = (int16_t)SampleFilter_get(&gyFilter);
    gyro[2] = (int16_t)SampleFilter_get(&gzFilter);*/

    /*accel[0] = axFilter.filterIn(accel[0]);
    accel[1] = ayFilter.filterIn(accel[1]);
    accel[2] = azFilter.filterIn(accel[2]);

    gyro[0] = gxFilter.filterIn(gyro[0]);
    gyro[1] = gyFilter.filterIn(gyro[1]);
    gyro[2] = gzFilter.filterIn(gyro[2]);*/
  }
}

void pdlReadGyro(pdlDroneState*) {}
