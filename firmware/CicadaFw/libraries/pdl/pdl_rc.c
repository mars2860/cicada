#include "pdl.h"
#include "pdl_tasks.h"

#include <string.h>

void pdlCmdSetCameraAngle(pdlDroneState *ds, uint8_t *packet)
{
  int32_t angle;
  memcpy(&angle,&packet[1],sizeof(angle));
  pdlSetCameraAngle(ds,angle);
}

void pdlCmdStartStopVideo(pdlDroneState *ds, uint8_t *packet)
{
  if(packet[1])
    pdlStartVideo(ds);
  else
    pdlStopVideo(ds);
}

void pdlCmdTakePhoto(pdlDroneState *ds, uint8_t *packet)
{
  (void)packet;
  pdlTakePhoto(ds);
}

void pdlCmdSaveDefaultCfg(pdlDroneState *ds, uint8_t *packet)
{
  (void)packet;
  pdlSaveDefaultCfg(ds);
}

void pdlCmdLoadDefaultCfg(pdlDroneState *ds, uint8_t *packet)
{
  (void)packet;
  pdlLoadDefaultCfg(ds);
}

void pdlCmdEnableMotors(pdlDroneState *ds, uint8_t *packet)
{
  pdlEnableMotors(ds, packet[1]);
}

void pdlCmdEnableStabilization(pdlDroneState *ds, uint8_t *packet)
{
  pdlEnableStabilization(ds, packet[1]);
}

void pdlCmdEnableTrickMode(pdlDroneState *ds, uint8_t *packet)
{
  pdlEnableTrickMode(ds, packet[1]);
}

void pdlCmdSetMotorsDir(pdlDroneState *ds, uint8_t *packet)
{
  pdlSetMotorsDir(ds, packet[1]);
}

void pdlCmdSetEsc(pdlDroneState *ds, uint8_t *packet)
{
  pdlSetEscMode(ds, packet[1]);
}

void pdlCmdSetFrame(pdlDroneState *ds, uint8_t *packet)
{
  pdlSetFrameType(ds, packet[1]);
}

void pdlCmdSetBattery(pdlDroneState *ds, uint8_t *packet)
{
  memcpy(&ds->battery.voltScaler, &packet[1], sizeof(float));
  memcpy(&ds->battery.curnScaler, &packet[5], sizeof(float));
  memcpy(&ds->battery.minVoltage, &packet[9], sizeof(float));
  memcpy(&ds->battery.maxVoltage, &packet[13], sizeof(float));
}

void pdlCmdSetBaseGas(pdlDroneState *ds, uint8_t *packet)
{
  int32_t t0;

  memcpy(&t0, &packet[1], sizeof(int32_t));

  pdlSetBaseGas(ds, t0);

  if(!ds->stabilizationEnabled)
  {
    for(uint8_t i = 0; i < PDL_MOTOR_COUNT;i++)
    {
      pdlSetMotorGas(ds,i,ds->baseGas);
    }
  }
}

void pdlCmdSetMotorsGas(pdlDroneState *ds, uint8_t *packet)
{
  int32_t t0,t1,t2,t3;

  memcpy(&t0, &packet[1], sizeof(int32_t));
  memcpy(&t1, &packet[5], sizeof(int32_t));
  memcpy(&t2, &packet[9], sizeof(int32_t));
  memcpy(&t3, &packet[13], sizeof(int32_t));

  pdlSetMotorGas(ds,0,t0);
  pdlSetMotorGas(ds,1,t1);
  pdlSetMotorGas(ds,2,t2);
  pdlSetMotorGas(ds,3,t3);
}

void pdlCmdSetAccel(pdlDroneState *ds, uint8_t *packet)
{
  uint8_t dlpf;
  int16_t dx,dy,dz;

  memcpy(&dx, &packet[1], sizeof(int16_t));
  memcpy(&dy, &packet[3], sizeof(int16_t));
  memcpy(&dz, &packet[5], sizeof(int16_t));
  memcpy(&dlpf, &packet[7], sizeof(uint8_t));

  ds->accel.offset[PDL_X] = dx;
  ds->accel.offset[PDL_Y] = dy;
  ds->accel.offset[PDL_Z] = dz;
  ds->accel.dlpf = dlpf;

  pdlSetupAccel(ds);
}

void pdlCmdSetGyro(pdlDroneState *ds, uint8_t *packet)
{
  uint8_t dlpf;
  int16_t dx,dy,dz;

  memcpy(&dx, &packet[1], sizeof(int16_t));
  memcpy(&dy, &packet[3], sizeof(int16_t));
  memcpy(&dz, &packet[5], sizeof(int16_t));
  memcpy(&dlpf, &packet[7], sizeof(uint8_t));

  ds->gyro.offset[PDL_X] = dx;
  ds->gyro.offset[PDL_Y] = dy;
  ds->gyro.offset[PDL_Z] = dz;
  ds->gyro.dlpf = dlpf;

  pdlSetupGyro(ds);
}

void pdlCmdSetMagneto(pdlDroneState *ds, uint8_t *packet)
{
  int16_t dx,dy,dz;
  float sx,sy,sz;
  float fTemp,fTemp2;

  memcpy(&dx, &packet[1], sizeof(int16_t));
  memcpy(&dy, &packet[3], sizeof(int16_t));
  memcpy(&dz, &packet[5], sizeof(int16_t));
  memcpy(&sx, &packet[7], sizeof(float));
  memcpy(&sy, &packet[11], sizeof(float));
  memcpy(&sz, &packet[15], sizeof(float));
  memcpy(&fTemp, &packet[19], sizeof(float));
  memcpy(&fTemp2, &packet[23], sizeof(float));

  ds->magneto.offset[PDL_X] = dx;
  ds->magneto.offset[PDL_Y] = dy;
  ds->magneto.offset[PDL_Z] = dz;
  ds->magneto.scale[PDL_X] = sx;
  ds->magneto.scale[PDL_Y] = sy;
  ds->magneto.scale[PDL_Z] = sz;
  ds->magneto.declination = fTemp;
  ds->magneto.inflightCorrection = fTemp2;

  pdlSetupMagneto(ds);
}

void pdlCmdSetPidConfig(pdlPidState *pid, uint8_t *packet)
{
  float kp,ki,kd,maxOut,maxErrSum;
  uint8_t enabled = packet[1];

  memcpy(&kp, &packet[2], sizeof(kp));
  memcpy(&ki, &packet[2 + sizeof(kp)], sizeof(ki));
  memcpy(&kd, &packet[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
  memcpy(&maxOut, &packet[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
  memcpy(&maxErrSum, &packet[2 + sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(maxOut)], sizeof(maxErrSum));

  pid->enabled = enabled;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->maxOut = maxOut;
  //pid->maxErrSum = maxErrSum;
}

void pdlCmdSetEulerTargets(pdlDroneState *ds, uint8_t *packet)
{
  float yaw,pitch,roll;

  memcpy(&yaw, &packet[1], sizeof(yaw));
  memcpy(&pitch, &packet[1 + sizeof(pitch)], sizeof(pitch));
  memcpy(&roll, &packet[1 + sizeof(yaw) + sizeof(pitch)], sizeof(roll));

  pdlSetYawRateTarget(ds,yaw);
  pdlSetPitchTarget(ds,pitch);
  pdlSetRollTarget(ds,roll);
}

void pdlCmdSetTelemetryPeriod(pdlDroneState *ds, uint8_t *packet)
{
  (void)ds;

  uint32_t ut0;
  memcpy(&ut0, &packet[1], sizeof(ut0));
  pdlSetTelemetryUpdatePeriod(ut0);
}

void pdlCmdResetAltitude(pdlDroneState *ds, uint8_t *packet)
{
  (void)packet;

  ds->baro.seaLevelPressure = ds->baro.pressure;
}

void pdlCmdSetSeaLevel(pdlDroneState *ds, uint8_t *packet)
{
  memcpy(&ds->baro.seaLevelPressure, &packet[1], sizeof(ds->baro.seaLevelPressure));
}

void pdlCmdSetAltTarget(pdlDroneState *ds, uint8_t *packet)
{
  memcpy(&ds->altPid.target, &packet[1], sizeof(ds->altPid.target));
}

void pdlCmdSetVeloXTarget(pdlDroneState *ds, uint8_t *packet)
{
  float fTemp;
  memcpy(&fTemp, &packet[1], sizeof(float));
  pdlSetVelocityXTarget(ds, fTemp);
}

void pdlCmdSetVeloYTarget(pdlDroneState *ds, uint8_t *packet)
{
  float fTemp;
  memcpy(&fTemp, &packet[1], sizeof(float));
  pdlSetVelocityYTarget(ds, fTemp);
}

void pdlCmdSetVeloZTarget(pdlDroneState *ds, uint8_t *packet)
{
  float fTemp;
  memcpy(&fTemp, &packet[1], sizeof(float));
  pdlSetVelocityZTarget(ds, fTemp);
}

void pdlCmdSetHeadingTarget(pdlDroneState *ds, uint8_t *packet)
{
  float fTemp;
  memcpy(&fTemp, &packet[1], sizeof(float));
  pdlSetHeadingTarget(ds, fTemp);
}

void pdlCmdSetPitchTarget(pdlDroneState *ds, uint8_t *packet)
{
  float pitch;
  memcpy(&pitch, &packet[1], sizeof(pitch));
  pdlSetPitchTarget(ds,pitch);
}

void pdlCmdSetRollTarget(pdlDroneState *ds, uint8_t *packet)
{
  float roll;
  memcpy(&roll, &packet[1], sizeof(roll));
  pdlSetRollTarget(ds,roll);
}

void pdlCmdSetYawRateTarget(pdlDroneState *ds, uint8_t *packet)
{
  float yaw;
  memcpy(&yaw, &packet[1], sizeof(yaw));
  pdlSetYawRateTarget(ds,yaw);
}

void pdlCmdSetPitchRateTarget(pdlDroneState *ds, uint8_t *packet)
{
  float pitch;
  memcpy(&pitch, &packet[1], sizeof(pitch));
  pdlSetPitchRateTarget(ds,pitch);
}

void pdlCmdSetRollRateTarget(pdlDroneState *ds, uint8_t *packet)
{
  float roll;
  memcpy(&roll, &packet[1], sizeof(roll));
  pdlSetRollRateTarget(ds,roll);
}

void pdlCmdSetKalman(pdlDroneState *ds, uint8_t *packet)
{
  memcpy(&ds->kfSettings.navModelNoise,    &packet[1], sizeof(float));
  memcpy(&ds->kfSettings.accVariance,      &packet[5], sizeof(float));
  memcpy(&ds->kfSettings.baroAltVariance,  &packet[9], sizeof(float));
  memcpy(&ds->kfSettings.lidarVelVariance1,&packet[13], sizeof(float));
  memcpy(&ds->kfSettings.ofVelVariance1,   &packet[17], sizeof(float));
  memcpy(&ds->kfSettings.ofVelVariance2,   &packet[21], sizeof(float));
  memcpy(&ds->kfSettings.lidarVelVariance2,&packet[25], sizeof(float));
  memcpy(&ds->kfSettings.poseModelNoise,   &packet[29], sizeof(float));
  memcpy(&ds->kfSettings.yawRateVariance,  &packet[33], sizeof(float));
  memcpy(&ds->kfSettings.pitchRollRateVariance,  &packet[37], sizeof(float));
  memcpy(&ds->kfSettings.magHeadingVariance,     &packet[41], sizeof(float));
  memcpy(&ds->kfSettings.accPitchRollVariance,   &packet[45], sizeof(float));
  memcpy(&ds->kfSettings.gpsHorSpeedVariance,    &packet[49], sizeof(float));
  memcpy(&ds->kfSettings.gpsVerSpeedVariance,    &packet[53], sizeof(float));
  memcpy(&ds->kfSettings.gpsHorPosVariance,      &packet[57], sizeof(float));
  memcpy(&ds->kfSettings.gpsVerPosVariance,      &packet[61], sizeof(float));

  pdlResetKalman(ds);
}

void pdlCmdSetGps(pdlDroneState *ds, uint8_t *packet)
{
  ds->gps.enabled = packet[1];
}

void pdlCmdSetLoad(pdlDroneState *ds, uint8_t *packet)
{
  uint8_t num,enabled;
  uint16_t period;
  num = packet[1];
  if(num < PDL_LOAD_COUNT)
  {
    enabled = packet[2];
    memcpy(&period,&packet[3],sizeof(uint16_t));

    ds->load[num].enabled = enabled;
    ds->load[num].period = period;
    ds->load[num].state = 0;
    pdlSwitchLoad(ds,num,0);
  }
}

void pdlCmdCalibrateAccel(pdlDroneState *ds, uint8_t *packet)
{
  (void)packet;
  pdlCalibrateAccel(ds);
}

void pdlCmdCalibrateGyro(pdlDroneState *ds, uint8_t *packet)
{
  (void)packet;
  pdlCalibrateGyro(ds);
}

void pdlCmdVeloZTakeoff(pdlDroneState *ds, uint8_t *packet)
{
  float err;
  if(!ds->stabilizationEnabled)
  {
    memcpy(&err,&packet[1],sizeof(float));
    ds->stabilizationEnabled = 1;
    ds->velocityZPid.errSum = err;
  }
}

void pdlParseCommand(pdlDroneState *ds, uint8_t *packet)
{
  uint8_t cmd = packet[0];
  switch(cmd)
  {
    case PDL_CMD_SWITCH_MOTORS:
      pdlCmdEnableMotors(ds,packet);
      break;
    case PDL_CMD_SET_MOTORS_DIR:
      pdlCmdSetMotorsDir(ds,packet);
      break;
    case PDL_CMD_SET_BATTERY:
      pdlCmdSetBattery(ds,packet);
      break;
    case PDL_CMD_SET_BASE_GAS:
      pdlCmdSetBaseGas(ds,packet);
      break;
    case PDL_CMD_SET_MOTORS_GAS:
      pdlCmdSetMotorsGas(ds,packet);
      break;
    case PDL_CMD_SET_ACCEL:
      pdlCmdSetAccel(ds,packet);
      break;
    case PDL_CMD_SET_GYRO:
      pdlCmdSetGyro(ds,packet);
      break;
    case PDL_CMD_SET_MAGNETO:
      pdlCmdSetMagneto(ds,packet);
      break;
    case PDL_CMD_SELF_CALIB_ACCEL:
      pdlCmdCalibrateAccel(ds,packet);
      break;
    case PDL_CMD_SELF_CALIB_GYRO:
      pdlCmdCalibrateGyro(ds,packet);
      break;
    case PDL_CMD_SET_YAW_RATE_PID:
      pdlCmdSetPidConfig(&ds->zRatePid, packet);
      break;
    case PDL_CMD_SET_PITCH_RATE_PID:
      pdlCmdSetPidConfig(&ds->yRatePid, packet);
      break;
    case PDL_CMD_SET_ROLL_RATE_PID:
      pdlCmdSetPidConfig(&ds->xRatePid, packet);
      break;
    case PDL_CMD_SET_PITCH_PID:
      pdlCmdSetPidConfig(&ds->pitchPid, packet);
      break;
    case PDL_CMD_SET_ROLL_PID:
      pdlCmdSetPidConfig(&ds->rollPid, packet);
      break;
    case PDL_CMD_SET_ALT_PID:
      pdlCmdSetPidConfig(&ds->altPid, packet);
      break;
    case PDL_CMD_SET_VELOCITY_X_PID:
      pdlCmdSetPidConfig(&ds->velocityXPid, packet);
      break;
    case PDL_CMD_SET_VELOCITY_Y_PID:
      pdlCmdSetPidConfig(&ds->velocityYPid, packet);
      break;
    case PDL_CMD_SET_VELOCITY_Z_PID:
      pdlCmdSetPidConfig(&ds->velocityZPid, packet);
      break;
    case PDL_CMD_SET_YPR:
      pdlCmdSetEulerTargets(ds, packet);
      break;
    case PDL_CMD_SET_TELEMETRY_PERIOD:
      pdlCmdSetTelemetryPeriod(ds, packet);
      break;
    case PDL_CMD_ENABLE_STABILIZATION:
      pdlCmdEnableStabilization(ds,packet);
      break;
    case PDL_CMD_ENABLE_TRICK_MODE:
      pdlCmdEnableTrickMode(ds,packet);
      break;
    case PDL_CMD_SET_ESC:
      pdlCmdSetEsc(ds,packet);
      break;
    case PDL_CMD_SET_FRAME:
      pdlCmdSetFrame(ds,packet);
      break;
    case PDL_CMD_RESET_ALTITUDE:
      pdlCmdResetAltitude(ds,packet);
      break;
    case PDL_CMD_SET_SEA_LEVEL:
      pdlCmdSetSeaLevel(ds,packet);
      break;
    case PDL_CMD_SET_ALTITUDE:
      pdlCmdSetAltTarget(ds,packet);
      break;
    case PDL_CMD_SET_HEADING_PID:
      pdlCmdSetPidConfig(&ds->headingPid, packet);
      break;
    case PDL_CMD_SET_VELOCITY_X:
      pdlCmdSetVeloXTarget(ds, packet);
      break;
    case PDL_CMD_SET_VELOCITY_Y:
      pdlCmdSetVeloYTarget(ds, packet);
      break;
    case PDL_CMD_SET_VELOCITY_Z:
      pdlCmdSetVeloZTarget(ds, packet);
      break;
    case PDL_CMD_SET_HEADING:
      pdlCmdSetHeadingTarget(ds, packet);
      break;
    case PDL_CMD_SET_PITCH:
      pdlCmdSetPitchTarget(ds, packet);
      break;
    case PDL_CMD_SET_ROLL:
      pdlCmdSetRollTarget(ds, packet);
      break;
    case PDL_CMD_SET_YAW_RATE:
      pdlCmdSetYawRateTarget(ds, packet);
      break;
    case PDL_CMD_SET_PITCH_RATE:
      pdlCmdSetPitchRateTarget(ds, packet);
      break;
    case PDL_CMD_SET_ROLL_RATE:
      pdlCmdSetRollRateTarget(ds, packet);
      break;
    /*case PDL_CMD_SET_USER_DATA1:
      memcpy(&ds->fUserData[0],&packet[1],sizeof(ds->fUserData));
      pdlCallbackUserDataReceived(ds);
      break;
      */
    case PDL_CMD_SET_POS_NORTH_PID:
      pdlCmdSetPidConfig(&ds->posNorthPid, packet);
      break;
    case PDL_CMD_SET_POS_EAST_PID:
      pdlCmdSetPidConfig(&ds->posEastPid, packet);
      break;
    case PDL_CMD_SET_KALMAN:
      pdlCmdSetKalman(ds, packet);
      break;
    case PDL_CMD_SET_GPS:
      pdlCmdSetGps(ds, packet);
      break;
    case PDL_CMD_ENABLE_LOAD:
      pdlCmdSetLoad(ds, packet);
      break;
    case PDL_CMD_SET_SSID:
      pdlCmdSetSsid(ds, packet);
      break;
    case PDL_CMD_SET_PSK:
      pdlCmdSetPsk(ds, packet);
      break;
    case PDL_CMD_SET_IP:
      pdlCmdSetIp(ds, packet);
      break;
    case PDL_CMD_SET_GATEWAY:
      pdlCmdSetGateway(ds, packet);
      break;
    case PDL_CMD_SET_SUBNET:
      pdlCmdSetSubnet(ds, packet);
      break;
    case PDL_CMD_VELOZ_TAKEOFF:
      pdlCmdVeloZTakeoff(ds, packet);
      break;
    case PDL_CMD_SAVE_DEFAULT_CFG:
      pdlCmdSaveDefaultCfg(ds, packet);
      break;
    case PDL_CMD_LOAD_DEFAULT_CFG:
      pdlCmdLoadDefaultCfg(ds,packet);
      break;
    case PDL_CMD_SET_CAMERA_ANGLE:
      pdlCmdSetCameraAngle(ds,packet);
      break;
    case PDL_CMD_START_STOP_VIDEO:
      pdlCmdStartStopVideo(ds,packet);
      break;
    case PDL_CMD_TAKE_PHOTO:
      pdlCmdTakePhoto(ds,packet);
      break;
    case PDL_CMD_ENABLE_DYNAMIC_IP:
      pdlCmdEnableDynamicIp(ds,packet);
      break;
  }
}
