#include "pdl.h"

#include <arduino.h>
#include "MPU9250.h"

MPU9250 mpu;

bool mpuInitialized = false;

void imuReadAccelOffset(pdlDroneState *ds);
void imuReadGyroOffset(pdlDroneState *ds);

void pdlSetupAccel(pdlDroneState *ds)
{
  // prevent to restart mpu9250 after CMD_SET_ACCEL command received
  if(mpuInitialized == false)
  {
    MPU9250Setting setting;

    setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    mpu.setup(0x68, setting);
    mpu.setMagneticDeclination(0);
    mpu.selectFilter(QuatFilterSel::MADGWICK);
    mpu.setFilterIterations(1);

    mpu.setup(0x68, setting);

    mpuInitialized = true;
  }

  mpu.writeAccelOffset(ds->accel.offset[PDL_X], ds->accel.offset[PDL_Y], ds->accel.offset[PDL_Z]);

  imuReadAccelOffset(ds);
}

void pdlSetupGyro(pdlDroneState *ds)
{
  mpu.writeGyroOffset(ds->gyro.offset[PDL_X], ds->gyro.offset[PDL_Y], ds->gyro.offset[PDL_Z]);

  imuReadGyroOffset(ds);
}

void pdlSetupMagneto(pdlDroneState* ds)
{
  mpu.writeMagOffset(ds->magneto.offset[PDL_X], ds->magneto.offset[PDL_Y], ds->magneto.offset[PDL_Z]);
}

void imuReadAccelOffset(pdlDroneState *ds)
{
  mpu.readAccelOffset(&ds->accel.offset[PDL_X], &ds->accel.offset[PDL_Y], &ds->accel.offset[PDL_Z]);
}

void imuReadGyroOffset(pdlDroneState *ds)
{
  mpu.readGyroOffset(&ds->gyro.offset[PDL_X], &ds->gyro.offset[PDL_Y], &ds->gyro.offset[PDL_Z]);
}

void pdlCalibrateAccel(pdlDroneState *ds)
{
  mpu.calibrateAccelGyro();
  imuReadAccelOffset(ds);
  imuReadGyroOffset(ds);
}

void pdlCalibrateGyro(pdlDroneState *ds)
{
  mpu.calibrateAccelGyro();
  imuReadAccelOffset(ds);
  imuReadGyroOffset(ds);
}

void pdlReadAccel(pdlDroneState *ds)
{
  if(mpu.update())
  {
    for(uint8_t i = 0; i < 3; i++)
    {
      ds->accel.raw[i] = mpu.getAccRaw(i);
      ds->accel.pure[i] = mpu.getAcc(i);

      ds->gyro.raw[i] = mpu.getGyroRaw(i);
      ds->gyro.pure[i] = mpu.getGyro(i) * DEG_TO_RAD;

      ds->magneto.raw[i] = mpu.getMagRaw(i);
      ds->magneto.pure[i] = mpu.getMag(i);
    }
    // correct pure values to be according PDL coordinate system
    ds->gyro.pure[PDL_Y] = -ds->gyro.pure[PDL_Y];
    ds->gyro.pure[PDL_Z] = -ds->gyro.pure[PDL_Z];
    ds->accel.pure[PDL_Y] = -ds->accel.pure[PDL_Y];
    ds->accel.pure[PDL_Z] = -ds->accel.pure[PDL_Z];
  }
}

void pdlReadGyro(pdlDroneState*) {}

void pdlReadMagneto(pdlDroneState*) {}

void pdlTripleAxisSensorFusion(pdlDroneState* ds)
{
  ds->yaw = mpu.getYaw() * DEG_TO_RAD;
  ds->pitch = mpu.getPitch() * DEG_TO_RAD;
  ds->roll = mpu.getRoll() * DEG_TO_RAD;
  ds->heading = (ds->yaw < 0.f)? ds->yaw + TWO_PI : ds->yaw;
}
