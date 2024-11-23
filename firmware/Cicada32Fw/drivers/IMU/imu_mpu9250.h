#ifndef DRIVERS_IMU_IMU_MPU9250_H_
#define DRIVERS_IMU_IMU_MPU9250_H_

#include "RunningMedian.h"

#include "imu.h"
#include "mpu9250ex.h"

class MPU9250Driver: public ImuDriver
{
private:
  MPU9250Ex mpu;
  MPU9250Setting setting;
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
private:
  void readAccelOffset(pdlDroneState *ds)
  {
    mpu.readAccelOffset(&ds->accel.offset[PDL_X], &ds->accel.offset[PDL_Y], &ds->accel.offset[PDL_Z]);
  }
  void readGyroOffset(pdlDroneState *ds)
  {
    mpu.readGyroOffset(&ds->gyro.offset[PDL_X], &ds->gyro.offset[PDL_Y], &ds->gyro.offset[PDL_Z]);
    //LOG_INFO("gox=%i,goy=%i,goz=%i",ds->gyro.offset[PDL_X],ds->gyro.offset[PDL_Y],ds->gyro.offset[PDL_Z]);
  }
  GYRO_DLPF_CFG castGyroDlpfCfg(uint32_t val)
  {
    switch(val)
    {
    case 0:
      return GYRO_DLPF_CFG::DLPF_250HZ;
    case 1:
      return GYRO_DLPF_CFG::DLPF_184HZ;
    case 2:
      return GYRO_DLPF_CFG::DLPF_92HZ;
    case 3:
      return GYRO_DLPF_CFG::DLPF_41HZ;
    case 4:
      return GYRO_DLPF_CFG::DLPF_20HZ;
    case 5:
      return GYRO_DLPF_CFG::DLPF_10HZ;
    case 6:
      return GYRO_DLPF_CFG::DLPF_5HZ;
    case 7:
      return GYRO_DLPF_CFG::DLPF_3600HZ;
    }
    return GYRO_DLPF_CFG::DLPF_10HZ;
  }
  ACCEL_DLPF_CFG castAccelDlpfCfg(uint32_t val)
  {
    switch(val)
    {
    case 0:
      return ACCEL_DLPF_CFG::DLPF_218HZ_0;
    case 1:
      return ACCEL_DLPF_CFG::DLPF_218HZ_1;
    case 2:
      return ACCEL_DLPF_CFG::DLPF_99HZ;
    case 3:
      return ACCEL_DLPF_CFG::DLPF_45HZ;
    case 4:
      return ACCEL_DLPF_CFG::DLPF_21HZ;
    case 5:
      return ACCEL_DLPF_CFG::DLPF_10HZ;
    case 6:
      return ACCEL_DLPF_CFG::DLPF_5HZ;
    case 7:
      return ACCEL_DLPF_CFG::DLPF_420HZ;
    }
    return ACCEL_DLPF_CFG::DLPF_5HZ;
  }
public:
  MPU9250Driver():
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
  }

  bool isMPU6500()
  {
    if(mpu.whoAmI() == MPU6500_WHOAMI_DEFAULT_VALUE)
      return true;
    return false;
  }

  bool setupAccel(pdlDroneState *ds)
  {
    if( mpuInitialized &&
          (ds->gyro.dlpf != (uint32_t)setting.gyro_dlpf_cfg ||
           ds->accel.dlpf != (uint32_t)setting.accel_dlpf_cfg) )
    {
      setting.gyro_dlpf_cfg = castGyroDlpfCfg(ds->gyro.dlpf);
      setting.accel_dlpf_cfg = castAccelDlpfCfg(ds->accel.dlpf);
      mpuInitialized = mpu.setup(0x68, setting);
      if(mpuInitialized)
      {
        LOG_INFO("New DLPF=%i,%i has been applied to IMU",setting.gyro_dlpf_cfg,setting.accel_dlpf_cfg);
      }
    }

    // updates gyro offsets and accel offsets because after changing DLPF settings offsets get wrong

    mpu.writeGyroOffset(ds->gyro.offset[PDL_X], ds->gyro.offset[PDL_Y], ds->gyro.offset[PDL_Z]);
    readGyroOffset(ds);

    mpu.writeAccelOffset(ds->accel.offset[PDL_X], ds->accel.offset[PDL_Y], ds->accel.offset[PDL_Z]);
    readAccelOffset(ds);

    imuStabCounter = 0;
    return mpuInitialized;
  }

  bool setupGyro(pdlDroneState *ds)
  {
    // prevent to restart mpu9250 after CMD_SET_GYRO command received
    if(mpuInitialized == false)
    {
      setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
      setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
      setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
      // Sampling frequency affects on DLPF output
      // It seems that DLPF is included after sampling divider block inside MPU9250
      //setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_50HZ;
      //setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_100HZ;
      //setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ; (default in pixhawk for i2c mode)
      // It is better to set sampling frequence 1000 Hz
      // In this case we can be sure that DLPF performance is as described in datasheet
      setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
      // max sample rate for ak8963 is 100 hz (it is hardcoded in MPU9250 class line 98)
      setting.gyro_fchoice = 0x03;
      // This is very important setting
      // WARN: Changing of DLPF setting leads to new PID koefs
      //setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_184HZ; // Data delay 2.9ms
      //setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_92HZ; // Data delay 3.9ms (default in pixhawk)
      //setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ; // Data delay 5.9ms
      //setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ; // Data delay 9.9ms
      setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_10HZ; // Data delay 17.85ms
      //setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_5HZ;  // Data delay 33.48ms
      setting.accel_fchoice = 0x01;
      //setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_99HZ; // Data delay 2.88ms
      //setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ; // Data delay 4.88ms (default in pixhawk)
      //setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_21HZ; // Data delay 8.87ms
      //setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_10HZ; // Data delay 16.83ms
      // This is very important setting
      // If motors are very noisy and we set higher DLPF cut-off frequency
      // AHRS filter will give wrong values for pitch and roll when drone flies
      // If drone flies in right or left or other side without your command
      // And you see in telemetry data that pitch & roll have zero values
      // In this case your motors very noisy and AHRS gives wrong values of pitch&roll
      // It is better to set DLPF cut-off for accel as lower as it possible
      // Data delay for accel is not important as gyro data
      // But gyro data delay have to be as low as possible
      setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_5HZ;  // Data delay 32.48ms

      // Playing with higher Accel DLPF cut-off gave not good results
      // Higher accel dlpf give good results for AHRS but bad results for vertical speed estimation

      // Playing with scale ragne of Accel/Gyro also gave not good results

      ds->gyro.dlpf = (uint32_t)setting.gyro_dlpf_cfg;
      ds->accel.dlpf = (uint32_t)setting.accel_dlpf_cfg;

      mpu.selectFilter(QuatFilterSel::NONE);
      mpu.setFilterIterations(0);
      mpu.ahrs(false);
      mpuInitialized = mpu.setup(0x68, setting);

      if(mpuInitialized)
      {
        if(isMPU6500())
        {
          LOG_INFO("MPU6500 is ok, id=0x%x",mpu.whoAmI());
        }
        else
        {
          mpu.setMagMode(1);
          LOG_INFO("MPU9250 is ok, id=0x%x",mpu.whoAmI());
        }
      }
    }

    if( mpuInitialized &&
        (ds->gyro.dlpf != (uint32_t)setting.gyro_dlpf_cfg ||
         ds->accel.dlpf != (uint32_t)setting.accel_dlpf_cfg) )
    {
      setting.gyro_dlpf_cfg = castGyroDlpfCfg(ds->gyro.dlpf);
      setting.accel_dlpf_cfg = castAccelDlpfCfg(ds->accel.dlpf);
      mpuInitialized = mpu.setup(0x68, setting);
      if(mpuInitialized)
      {
        LOG_INFO("New DLPF=%i,%i has been applied to IMU",setting.gyro_dlpf_cfg,setting.accel_dlpf_cfg);
      }
    }

    // updates gyro offsets and accel offsets because after changing DLPF settings offsets get wrong
    mpu.writeGyroOffset(ds->gyro.offset[PDL_X], ds->gyro.offset[PDL_Y], ds->gyro.offset[PDL_Z]);
    readGyroOffset(ds);

    mpu.writeAccelOffset(ds->accel.offset[PDL_X], ds->accel.offset[PDL_Y], ds->accel.offset[PDL_Z]);
    readAccelOffset(ds);

    imuStabCounter = 0;
    return mpuInitialized;
  }

  bool setupMagneto(pdlDroneState *ds)
  {
    mpu.writeMagOffset(ds->magneto.offset[PDL_X], ds->magneto.offset[PDL_Y], ds->magneto.offset[PDL_Z]);
    mpu.setMagScale(ds->magneto.scale[PDL_X],ds->magneto.scale[PDL_Y],ds->magneto.scale[PDL_Z]);
    return true;
  }

  void calibrateAccel(pdlDroneState *ds)
  {
    mpu.calibrateAccelGyro();
    readAccelOffset(ds);
    readGyroOffset(ds);
    imuStabCounter = 0;
  }

  void calibrateGyro(pdlDroneState *ds)
  {
    mpu.calibrateAccelGyro();
    readAccelOffset(ds);
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

    if(!(mpu.readAccGyroData() & 0x1))
    {
      return false;
    }

    // save measurements
    for(i = 0; i < 3; i++)
    {
      accRaw[i] = mpu.getAccRaw(i);
      af[i].add(mpu.getAcc(i) * PDL_G);
      accPure[i] = af[i].getMedian();
      //ds->accel.pure[i] = mpu.getAcc(i) * PDL_G;

      gyroRaw[i] = mpu.getGyroRaw(i);
      gf[i].add(mpu.getGyro(i) * DEG_TO_RAD);
      gyroPure[i] = gf[i].getMedian();
      //ds->gyro.pure[i] = mpu.getGyro(i) * DEG_TO_RAD;
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
    uint8_t i;

    if(mpu.readMagnetoData() == false)
    {
       return false;
    }

    // use single shot because sometimes measurement doesn't start after measurement period in continuous mode
    mpu.setMagMode(1);

    // store measurements
    for(i = 0; i < 3; i++)
    {
      magRaw[i] = mpu.getMagRaw(i);
      mf[i].add(mpu.getMag(i));
      magPure[i] = mf[i].getMedian();
    }
    // correct pure values to be according PDL coordinate system
    float x = magPure[PDL_X];
    magPure[PDL_X] = magPure[PDL_Y];
    magPure[PDL_Y] = -x;
    return true;
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

#endif /* DRIVERS_IMU_IMU_MPU9250_H_ */
