#include "pdl.h"

#include <Wire.h>
#include <Math.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

Quaternion q;               // [w, x, y, z] quaternion container
VectorFloat gravity;        // [x, y, z] gravity vector
//VectorInt16 aa;             // [x, y, z] accel sensor measurements
//VectorInt16 aaReal;         // [x, y, z] gravity-free accel sensor measurements
//VectorInt16 aaWorld;        // [x, y, z] world-frame accel sensor measurements

void imuReadAccelOffset(pdlDroneState *ds);
void imuReadGyroOffset(pdlDroneState *ds);

void pdlSetupAccel(pdlDroneState *ds)
{
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  pdlSetImuReadPeriod(5000);

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
  // this approach takes 10-12 ms to read sensors and update pids state
  // it is slow because we have slow I2C. Reading 42 bytes of fifo takes to much time

  // holds actual interrupt status byte from MPU
  //uint8_t mpuIntStatus = mpu.getIntStatus();
  // FIFO storage buffer
  uint8_t fifoBuffer[64];
  uint16_t packetSize = mpu.dmpGetFIFOPacketSize();

  //if((mpuIntStatus & MPU6050_INTERRUPT_DMP_INT_BIT) && mpu.dmpPacketAvailable())
  if(mpu.dmpPacketAvailable())
  {
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // reset fifo to get fresh data in next cycle
    mpu.resetFIFO();

    // calc ypr (< 1 millis to execute)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&ds->accel.raw[0], fifoBuffer);
    mpu.dmpGetGyro(&ds->gyro.raw[0], fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    for(uint8_t i = 0; i < 3; i++)
    {
      ds->accel.filtered[i] = ds->accel.raw[i];
      ds->accel.pure[i] = (float)(ds->accel.filtered[i]) / 8192.f;

      ds->gyro.filtered[i] = ds->gyro.raw[i];
      ds->gyro.pure[i] = (float)(ds->gyro.filtered[i]) / 16.4f;
      ds->gyro.pure[i] *= 3.1415926535f/180.f;
    }
  }

  /*if((mpuIntStatus & MPU6050_INTERRUPT_FIFO_OFLOW_BIT) || mpu.getFIFOCount() == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("MPU FIFO overflow!"));
  }*/
}

void pdlReadGyro(pdlDroneState *ds) {}

void pdlTripleAxisSensorFusion(pdlDroneState *ds)
{
  float ypr[3];

  mpu.dmpGetYawPitchRoll(&ypr[0], &q, &gravity);

  ds->yaw = ypr[0];
  ds->pitch = ypr[1];
  ds->roll = ypr[2];

  pdlCalcHeading(ds);
}
