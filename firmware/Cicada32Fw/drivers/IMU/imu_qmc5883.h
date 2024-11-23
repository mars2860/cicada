#ifndef DRIVERS_IMU_IMU_QMC5883_H_
#define DRIVERS_IMU_IMU_QMC5883_H_

#include "RunningMedian.h"

#include "imu.h"
#include "QMC5883L.h"

class QMC5883Driver: public ImuDriver
{
private:
  QMC5883L mag;
  bool isInitialized;
  // We need median filters because there is spikes in imu data
  // I guess this caused by TF mini plus with 5V power supply
  RunningMedian mf[3];
  // to wait some time for accel data stabilization after offset has been changed
  uint8_t imuStabCounter;
  int16_t magRaw[3];
  float magPure[3];
  float magOffset[3];
  float magScale[3];
  float magResolution;
private:
  void resetCalibration()
  {
    for(int i = 0; i < 3; i++)
    {
      magOffset[i] = 0;
      magScale[i] = 1.f;
    }
  }
public:
  QMC5883Driver():
    mf {RunningMedian(3),
        RunningMedian(3),
        RunningMedian(3)}
  {
    isInitialized = false;
    imuStabCounter = 0;
    magResolution = 1.0f;
    resetCalibration();
  }

  bool setupAccel(pdlDroneState *ds)
  {
    (void)ds;
    return false;
  }

  bool setupGyro(pdlDroneState *ds)
  {
    (void)ds;
    return false;
  }

  bool setupMagneto(pdlDroneState *ds)
  {
    (void)ds;
    if(isInitialized == false)
    {
      isInitialized = mag.init( QMC5883L_CONFIG_100HZ,
                                QMC5883L_CONFIG_2GAUSS,
                                QMC5883L_CONFIG_OS256);
      imuStabCounter = 0;
      magResolution = 2000.f/32768.f;

      if(isInitialized)
      {
        LOG_INFO("QMC5883 is ok");
      }
    }

    calibrateMagneto(ds);

    return isInitialized;
  }

  void calibrateAccel(pdlDroneState *ds)
  {
    (void)ds;
  }

  void calibrateGyro(pdlDroneState *ds)
  {
    (void)ds;
  }

  void calibrateMagneto(pdlDroneState *ds)
  {
    magOffset[PDL_X] = ds->magneto.offset[PDL_X];
    magOffset[PDL_Y] = ds->magneto.offset[PDL_Y];
    magOffset[PDL_Z] = ds->magneto.offset[PDL_Z];

    magScale[PDL_X] = ds->magneto.scale[PDL_X];
    magScale[PDL_Y] = ds->magneto.scale[PDL_Y];
    magScale[PDL_Z] = ds->magneto.scale[PDL_Z];
  }

  bool readAccel()
  {
    return false;
  }

  bool readGyro()
  {
    return false;
  }

  bool readMagneto()
  {
    uint8_t i;

    if(isInitialized == false)
      return false;

    // wait for imu gets stabilized
    if(imuStabCounter < 100)
    {
      imuStabCounter++;
      return false;
    }

    if(!mag.readRaw(&magRaw[0],&magRaw[1],&magRaw[2]))
    {
      return false;
    }

    // store measurements
    for(i = 0; i < 3; i++)
    {
      float val = ((float)magRaw[i] - magOffset[i])*magScale[i]*magResolution;
      mf[i].add(val);
      magPure[i] = mf[i].getMedian();
    }

    // correct pure values to be according PDL coordinate system
    float magY = magPure[PDL_Y];
    magPure[PDL_Y] = magPure[PDL_X];
    magPure[PDL_X] = -magY;

    return true;
  }

  int16_t getAccelRaw(uint8_t i)
  {
    (void)i;
    return 0;
  }

  float getAccelPure(uint8_t i)
  {
    (void)i;
    return 0;
  }

  int16_t getGyroRaw(uint8_t i)
  {
    (void)i;
    return 0;
  }

  float getGyroPure(uint8_t i)
  {
    (void)i;
    return 0;
  }

  int16_t getMagnetoRaw(uint8_t i)
  {
    if(i < 3)
      return magRaw[i];
    return 0;
  }

  float getMagnetoPure(uint8_t i)
  {
    if(i < 3)
      return magPure[i];
    return 0;
  }
};

#endif /* DRIVERS_IMU_IMU_QMC5883_H_ */
