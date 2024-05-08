#include <arduino.h>
#include "pdl.h"
#include "pdl_utils.h"
#include "main.h"
#include <math.h>

#include "mpu9250ex.h"
#include "RunningMedian.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "QMC5883L.h"

class ImuDriver
{
public:
  virtual ~ImuDriver() {};
  virtual bool setupAccel(pdlDroneState *ds) = 0;
  virtual bool setupGyro(pdlDroneState *ds) = 0;
  virtual bool setupMagneto(pdlDroneState *ds) = 0;
  virtual void calibrateAccel(pdlDroneState *ds) = 0;
  virtual void calibrateGyro(pdlDroneState *ds) = 0;
  virtual void calibrateMagneto(pdlDroneState *ds) = 0;
  virtual bool readAccel() = 0;
  virtual bool readGyro() = 0;
  virtual bool readMagneto() = 0;
  virtual int16_t getAccelRaw(uint8_t i) = 0;
  virtual float getAccelPure(uint8_t i) = 0;
  virtual int16_t getGyroRaw(uint8_t i) = 0;
  virtual float getGyroPure(uint8_t i) = 0;
  virtual int16_t getMagnetoRaw(uint8_t i) = 0;
  virtual float getMagnetoPure(uint8_t i) = 0;
};

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

ImuDriver *pAccel = 0;
ImuDriver *pGyro = 0;
ImuDriver *pMagneto = 0;

//-----------------------------------------------------------------------------
// PDL IMU FUNCTIONS IMPLEMENTATION

bool mpu9250Available = false;

void pdlSetupGyro(pdlDroneState *ds)
{
  if(pGyro)  // perform remote command to calibrate gyro
  {
    if(pGyro->setupGyro(ds))
      return;
  }
  delay(500); // wait for power up of imu module
  // scan for MPU9250
  MPU9250Driver *mpu9250 = new MPU9250Driver();
  pAccel = mpu9250;
  pGyro = mpu9250;
  pMagneto = 0;
  if(pGyro->setupGyro(ds))
  {
    // check for MPU6500
    if(mpu9250->isMPU6500())
    {
      // MPU6500 has no magnetometer
      pMagneto = 0;
    }
    else
    {
      mpu9250Available = true;
    }
    return;
  }
  // scan for MPU6050
  delete pAccel;
  pAccel = new MPU6050Driver();
  pGyro = pAccel;
  pMagneto = 0;
  if(pGyro->setupGyro(ds))
    return;
  // imu is not found
  delete pAccel;
  pAccel = 0;
  pGyro = 0;
  pMagneto = 0;
  pdlSetError(ds,ERR_IMU_DATA_NOT_READY);
  LOG_INFO("Gyroscope is not found");
}

void pdlSetupAccel(pdlDroneState *ds)
{
  if(pAccel)  // perform remote command to calibrate accel
  {
    if(pAccel->setupAccel(ds))
      return;
  }
  pdlSetError(ds,ERR_IMU_DATA_NOT_READY);
  LOG_INFO("Accelerometer is not found");
}

void pdlSetupMagneto(pdlDroneState* ds)
{
  if(pMagneto)  // perform remote command to calibrate magneto
  {
    if(pMagneto->setupMagneto(ds))
      return;
  }
  // scan for QMC5883
  pMagneto = new QMC5883Driver();
  if(pMagneto->setupMagneto(ds))
    return;
  delete pMagneto;
  pMagneto = 0;
  // apply MPU9250 if no other magneto is available
  if(mpu9250Available)
  {
    pMagneto = pGyro;
    return;
  }
  pMagneto = 0;
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
