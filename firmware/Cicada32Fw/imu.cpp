#include "pdl.h"
#include "pdl_utils.h"
#include "main.h"

#include "imu_mpu9250.h"
#include "imu_mpu6050.h"
#include "imu_qmc5883.h"

#include <math.h>

static ImuDriver *pAccel = NULL;
static ImuDriver *pGyro = NULL;
static ImuDriver *pMagneto = NULL;

//-----------------------------------------------------------------------------
// PDL IMU FUNCTIONS IMPLEMENTATION

bool mpu9250Available = false;

void pdlSetupGyro(pdlDroneState *ds)
{
  if(pGyro)  // perform remote command to calibrate gyro
  {
    pGyro->setupGyro(ds);
    return;
  }
  else
  {
    return; // TODO Gyro
  }


  delay(500); // wait for power up of imu module
  // scan for MPU9250
  MPU9250Driver *mpu9250 = new MPU9250Driver();
  pAccel = mpu9250;
  pGyro = mpu9250;
  pMagneto = NULL;
  if(pGyro->setupGyro(ds))
  {
    // check for MPU6500
    if(mpu9250->isMPU6500())
    {
      // MPU6500 has no magnetometer
      pMagneto = NULL;
    }
    else
    {
      mpu9250Available = true;
    }
    return;
  }
  // scan for MPU6050
  if(pAccel)
    delete pAccel;
  pAccel = new MPU6050Driver();
  pGyro = pAccel;
  pMagneto = NULL;
  if(pGyro->setupGyro(ds))
    return;
  // imu is not found
  if(pAccel)
    delete pAccel;
  pAccel = NULL;
  pGyro = NULL;
  pMagneto = NULL;
  pdlSetError(ds,ERR_IMU_DATA_NOT_READY);
  LOG_INFO("Gyroscope is not found");
}

void pdlSetupAccel(pdlDroneState *ds)
{
  if(pAccel)  // perform remote command to calibrate accel
  {
    pAccel->setupAccel(ds);
    return;
  }
  else
  {
    return; // TODO accel
  }
  pdlSetError(ds,ERR_IMU_DATA_NOT_READY);
  LOG_INFO("Accelerometer is not found");
}

void pdlSetupMagneto(pdlDroneState* ds)
{
  if(pMagneto)  // perform remote command to calibrate magneto
  {
    pMagneto->setupMagneto(ds);
    return;
  }
  else
  {
    return; // TODO Magneto
  }
  // scan for QMC5883
  pMagneto = new QMC5883Driver();
  if(pMagneto->setupMagneto(ds))
    return;
  // apply MPU9250 if no other magneto is available
  if(pMagneto)
    delete pMagneto;
  pMagneto = NULL;
  if(mpu9250Available)
  {
    pMagneto = pGyro;
    return;
  }
  pMagneto = NULL;
  // no magneto
  pdlSetError(ds,ERR_MAG_DATA_NOT_READY);
  LOG_INFO("Magnetometer is not found");
}

void pdlCalibrateAccel(pdlDroneState *ds)
{
  if(pAccel)
  {
    pAccel->calibrateAccel(ds);
  }
}

void pdlCalibrateGyro(pdlDroneState *ds)
{
  if(pGyro)
  {
    pGyro->calibrateGyro(ds);
  }
}

uint8_t pdlReadGyro(pdlDroneState *ds)
{
  // if host is not set, there are invalid gyro & accel data because offsets are not set
  if(hostIsSet() == false || !pGyro || !pAccel)
    return 0;

  if(pGyro->readGyro() == false)
  {
    pdlSetError(ds,ERR_IMU_DATA_NOT_READY);
    return 0;
  }

  pdlResetError(ds,ERR_IMU_DATA_NOT_READY);

  // provide new data to PDL
  pdlNewGyroData(ds, pGyro->getGyroRaw(PDL_X),  pGyro->getGyroRaw(PDL_Y),  pGyro->getGyroRaw(PDL_Z),
                     pGyro->getGyroPure(PDL_X), pGyro->getGyroPure(PDL_Y), pGyro->getGyroPure(PDL_Z));

  pdlNewAccelData(ds, pAccel->getAccelRaw(PDL_X),  pAccel->getAccelRaw(PDL_Y),  pAccel->getAccelRaw(PDL_Z),
                      pAccel->getAccelPure(PDL_X), pAccel->getAccelPure(PDL_Y), pAccel->getAccelPure(PDL_Z));

  // calc euler angles with complementary filter to compare with used Kalman filter
  static uint32_t oldMicros = 0;
  float dt = pdlGetDeltaTime(pdlMicros(),oldMicros);
  oldMicros = pdlMicros();
  dt /= 1000000.f;

  //LOG_INFO("h=%f,r=%f,y=%f\n",ds->magneto.heading,ds->gyro.pure[PDL_Z],ds->refYaw);

  if(!isnan(ds->accel.pitch))
  {
    ds->refPitch = pdlComplementaryFilter(0.98f,ds->refPitch,ds->gyro.pure[PDL_Y],ds->accel.pitch,dt);
  }

  if(!isnan(ds->accel.roll))
  {
    ds->refRoll = pdlComplementaryFilter(0.98f,ds->refRoll,ds->gyro.pure[PDL_X],ds->accel.roll,dt);
  }

  if(!isnan(ds->magneto.heading)) // sometimes it's NaN ?!
  {
    ds->refYaw = pdlComplementaryFilter(0.98f,ds->refYaw,ds->gyro.pure[PDL_Z],ds->magneto.heading,dt);
  }

  return 1;
}

uint8_t pdlReadAccel(pdlDroneState*)
{
  // if host is not set, there are invalid gyro & accel data because offsets are not set
  if(hostIsSet() == false || !pAccel)
    return 0;

  return 1;
}

uint8_t pdlReadMagneto(pdlDroneState* ds)
{
  // if host is not set, there are invalid gyro & accel data because offsets are not set
  if(hostIsSet() == false || !pMagneto)
    return 0;

  if(pMagneto->readMagneto() == false)
  {
    pdlSetError(ds,ERR_MAG_DATA_NOT_READY);
    return 0;
  }

  pdlResetError(ds,ERR_MAG_DATA_NOT_READY);

  // provide magneto data to PDL
  pdlNewMagnetoData(ds, pMagneto->getMagnetoRaw(PDL_X),  pMagneto->getMagnetoRaw(PDL_Y),  pMagneto->getMagnetoRaw(PDL_Z),
                        pMagneto->getMagnetoPure(PDL_X), pMagneto->getMagnetoPure(PDL_Y), pMagneto->getMagnetoPure(PDL_Z));

  return 1;
}
