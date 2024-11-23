#ifndef DRIVERS_IMU_IMU_MPU6050_H_
#define DRIVERS_IMU_IMU_MPU6050_H_

#include "RunningMedian.h"

#include "imu.h"
#include "I2Cdev.h"
#include "MPU6050.h"

class MPU6050Driver: public ImuDriver
{
private:
  MPU6050 mpu;
  bool mpuInitialized;
  // We need median filters because there is spikes in imu data
  // I guess this caused by TF mini plus with 5V power supply
  RunningMedian af[3];
  RunningMedian gf[3];
  RunningMedian mf[3];
  // to wait some time for accel data stabilization after offset has been changed
  uint8_t imuStabCounter;
  int16_t accRaw[3];
  float accPure[3];
  int16_t gyroRaw[3];
  float gyroPure[3];
  int16_t magRaw[3];
  float magPure[3];
  float acc_resolution;
  float gyro_resolution;
private:
  void readAccelOffset(pdlDroneState *ds)
  {
    ds->accel.offset[PDL_X] = mpu.getXAccelOffset();
    ds->accel.offset[PDL_Y] = mpu.getYAccelOffset();
    ds->accel.offset[PDL_Z] = mpu.getZAccelOffset();
  }
  void readGyroOffset(pdlDroneState *ds)
  {
    ds->gyro.offset[PDL_X] = mpu.getXGyroOffset();
    ds->gyro.offset[PDL_Y] = mpu.getYGyroOffset();
    ds->gyro.offset[PDL_Z] = mpu.getZGyroOffset();
  }
  float get_acc_resolution(const MPU6050_IMU::ACCEL_FS accel_af_sel) const
  {
    switch (accel_af_sel)
    {
      case MPU6050_IMU::ACCEL_FS::MPU6050_ACCEL_FS_2:
        return 2.0 / 32768.0;
      case MPU6050_IMU::ACCEL_FS::MPU6050_ACCEL_FS_4:
        return 4.0 / 32768.0;
      case MPU6050_IMU::ACCEL_FS::MPU6050_ACCEL_FS_8:
        return 8.0 / 32768.0;
      case MPU6050_IMU::ACCEL_FS::MPU6050_ACCEL_FS_16:
        return 16.0 / 32768.0;
      default:
        return 0.;
    }
  }
  float get_gyro_resolution(const MPU6050_IMU::GYRO_FS gyro_fs_sel) const
  {
    switch (gyro_fs_sel)
    {
      case MPU6050_IMU::GYRO_FS::MPU6050_GYRO_FS_250:
        return 250.0 / 32768.0;
      case MPU6050_IMU::GYRO_FS::MPU6050_GYRO_FS_500:
        return 500.0 / 32768.0;
      case MPU6050_IMU::GYRO_FS::MPU6050_GYRO_FS_1000:
        return 1000.0 / 32768.0;
      case MPU6050_IMU::GYRO_FS::MPU6050_GYRO_FS_2000:
        return 2000.0 / 32768.0;
      default:
        return 0.;
    }
  }
public:
  MPU6050Driver():
    af {RunningMedian(3),
        RunningMedian(3),
        RunningMedian(3)},
    gf {RunningMedian(3),
        RunningMedian(3),
        RunningMedian(3)},
    mf {RunningMedian(3),
        RunningMedian(3),
        RunningMedian(3)}
  {
    mpuInitialized = false;
    imuStabCounter = 0;
    acc_resolution = 0;
    gyro_resolution = 0;
  }

  bool setupAccel(pdlDroneState *ds)
  {
    mpu.setXAccelOffset(ds->accel.offset[PDL_X]);
    mpu.setYAccelOffset(ds->accel.offset[PDL_Y]);
    mpu.setZAccelOffset(ds->accel.offset[PDL_Z]);
    readAccelOffset(ds);
    imuStabCounter = 0;
    return mpuInitialized;
  }

  bool setupGyro(pdlDroneState *ds)
  {
    // prevent to restart mpu9250 after CMD_SET_ACCEL command received
    if(mpuInitialized == false)
    {
      mpu.initialize();
      mpu.setRate(0);
      MPU6050_IMU::GYRO_FS gyro_fs = MPU6050_IMU::GYRO_FS::MPU6050_GYRO_FS_2000;
      gyro_resolution = get_gyro_resolution(gyro_fs);
      mpu.setFullScaleGyroRange(gyro_fs);
      MPU6050_IMU::ACCEL_FS accel_fs = MPU6050_IMU::ACCEL_FS::MPU6050_ACCEL_FS_16;
      acc_resolution = get_acc_resolution(accel_fs);
      mpu.setFullScaleAccelRange(accel_fs);
      //          |   ACCELEROMETER    |           GYROSCOPE
      // DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
      // ---------+-----------+--------+-----------+--------+-------------
      // 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
      // 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
      // 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
      // 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
      // 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
      // 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
      // 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
      // 7        |   -- Reserved --   |   -- Reserved --   | Reserved
      mpu.setDLPFMode(MPU6050_IMU::DLPF_CFG::MPU6050_DLPF_BW_10);

      ds->gyro.dlpf = MPU6050_IMU::DLPF_CFG::MPU6050_DLPF_BW_10;
      ds->accel.dlpf = MPU6050_IMU::DLPF_CFG::MPU6050_DLPF_BW_10;

      mpuInitialized = mpu.testConnection();

      if(mpuInitialized)
      {
        LOG_INFO("MPU6050 is ok");
      }
    }

    if(mpuInitialized && ds->gyro.dlpf <= 6 && mpu.getDLPFMode() != ds->gyro.dlpf)
    {
      mpu.setDLPFMode(ds->gyro.dlpf);
      LOG_INFO("New DLPF=%i has been applied to MPU6050",mpu.getDLPFMode());
    }

    mpu.setXGyroOffset(ds->gyro.offset[PDL_X]);
    mpu.setYGyroOffset(ds->gyro.offset[PDL_Y]);
    mpu.setZGyroOffset(ds->gyro.offset[PDL_Z]);
    readGyroOffset(ds);
    imuStabCounter = 0;
    return mpuInitialized;
  }

  bool setupMagneto(pdlDroneState *ds)
  {
    (void)ds;
    return false;
  }

  void calibrateAccel(pdlDroneState *ds)
  {
    mpu.CalibrateAccel(6);
    readAccelOffset(ds);
    imuStabCounter = 0;
  }

  void calibrateGyro(pdlDroneState *ds)
  {
    mpu.CalibrateGyro(6);
    readGyroOffset(ds);
    imuStabCounter = 0;
  }

  void calibrateMagneto(pdlDroneState*)
  {
    // it is done by host
  }

  bool readAccel()
  {
    return true;
  }

  bool readGyro()
  {
    uint8_t i;

    // wait for imu gets stabilized
    if(imuStabCounter < 100)
    {
      imuStabCounter++;
      return false;
    }

    if(mpu.getIntDataReadyStatus() == false)
    {
      return false;
    }

    mpu.getMotion6( &accRaw[PDL_X],
                    &accRaw[PDL_Y],
                    &accRaw[PDL_Z],
                    &gyroRaw[PDL_X],
                    &gyroRaw[PDL_Y],
                    &gyroRaw[PDL_Z]);



    // save measurements
    for(i = 0; i < 3; i++)
    {
      af[i].add((float)accRaw[i]*acc_resolution*PDL_G);
      accPure[i] = af[i].getMedian();

      gf[i].add((float)gyroRaw[i]*gyro_resolution*DEG_TO_RAD);
      gyroPure[i] = gf[i].getMedian();
    }

    // correct pure values to be according PDL coordinate system
    gyroPure[PDL_Y] = -gyroPure[PDL_Y];
    gyroPure[PDL_Z] = -gyroPure[PDL_Z];

    accPure[PDL_Y] = -accPure[PDL_Y];
    accPure[PDL_Z] = -accPure[PDL_Z];

    return true;
  }

  bool readMagneto()
  {
    return false;
  }

  int16_t getAccelRaw(uint8_t i)
  {
    if(i < 3)
      return accRaw[i];
    return 0;
  }

  float getAccelPure(uint8_t i)
  {
    if(i < 3)
      return accPure[i];
    return 0;
  }

  int16_t getGyroRaw(uint8_t i)
  {
    if(i < 3)
      return gyroRaw[i];
    return 0;
  }

  float getGyroPure(uint8_t i)
  {
    if(i < 3)
      return gyroPure[i];
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




#endif /* DRIVERS_IMU_IMU_MPU6050_H_ */
