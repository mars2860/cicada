#include "pdl.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

typedef struct s_pdlTask
{
  uint32_t lastUpdateTime;
  uint32_t period;
} pdlTaskState;

pdlTaskState pdlRcTS;
pdlTaskState pdlBatteryTS;
pdlTaskState pdlImuTS;
pdlTaskState pdlOpticalFlowTS;
pdlTaskState pdlLidarTS;
pdlTaskState pdlBaroTS;
pdlTaskState pdlUpdateEscaperTS;
pdlTaskState pdlTelemetryTS;

/// Set true if some pid has been modified
uint8_t pdlPidModifiedFlag;
/// Time of system when pdlUpdate has been invoked in last time
uint32_t pdlUpdateStartTime;

void pdlCrossFrameApplyPids(pdlDroneState*);
void pdlXFrameApplyPids(pdlDroneState*);

void pdlUpdateAltPid(pdlDroneState*);
void pdlUpdateLevelsPid(pdlDroneState*);
void pdlUpdateHorVelocityPids(pdlDroneState*);

uint8_t pdlCanTaskRun(pdlTaskState*);

void pdlRcTask(pdlDroneState*);
void pdlBatteryTask(pdlDroneState*);
void pdlImuTask(pdlDroneState*);
void pdlBaroTask(pdlDroneState*);
void pdlLidarTask(pdlDroneState*);
void pdlOpticalFlowTask(pdlDroneState*);
void pdlUpdateEscaperTask(pdlDroneState*);
void pdlTelemetryTask(pdlDroneState*);

void parsePidConfigPacket(pdlPidState *pid, uint8_t *packet);
void parseTripleAxisSensorConfigPacket(pdlTripleAxisSensorState *ps, uint8_t *packet);

//-----------------------------------------------------------------------------

uint32_t pdlGetDeltaTime(uint32_t cur, uint32_t last)
{
  if(cur >= last)
    return cur - last;

  return UINT32_MAX - last + cur;
}

void pdlSetup(pdlDroneState *ds)
{
  memset(ds,0,sizeof(pdlDroneState));

  pdlRcTS.period = PDL_RC_UPDATE_PERIOD;
  pdlBatteryTS.period = PDL_BATTERY_READ_PERIOD;
  pdlImuTS.period = PDL_IMU_READ_PERIOD;
  pdlOpticalFlowTS.period = PDL_OPTICAL_FLOW_READ_PERIOD;
  pdlLidarTS.period = PDL_LIDAR_READ_PERIOD;
  pdlBaroTS.period = PDL_BARO_READ_PERIOD;
  pdlUpdateEscaperTS.period = PDL_ESCAPER_UPDATE_PERIOD;
  pdlTelemetryTS.period = PDL_DEFAULT_TELEMETRY_UPDATE_PERIOD;

  pdlPidModifiedFlag = 0;
  pdlUpdateStartTime = 0;

  ds->version = PDL_VERSION;

  pdlStopMotors(ds);
  pdlSetupEscaper(ds);
  pdlSetupAccel(ds);
  pdlSetupGyro(ds);
  pdlSetupMagneto(ds);
  pdlSetupBaro(ds);
  pdlSetupRc(ds);
  pdlSetupLidar(ds);
  pdlSetupOpticalFlow(ds);
  pdlSetupTelemetry(ds);
}

uint8_t pdlCanTaskRun(pdlTaskState *ts)
{
  uint32_t dt, frameDt;

  dt = pdlGetDeltaTime(pdlMicros(), ts->lastUpdateTime);
  if(dt < ts->period)
    return 0;

  dt -= ts->period;
  frameDt = pdlGetDeltaTime(pdlMicros(), pdlUpdateStartTime);

  if(frameDt >= PDL_DESIRE_UPDATE_TIME && dt < PDL_TASK_MAX_WAIT_TIME)
    return 0;

  ts->lastUpdateTime = pdlMicros();
  return 1;
}

void pdlSetTelemetryUpdatePeriod(uint32_t t)
{
  pdlTelemetryTS.period = t;
}

uint32_t pdlGetTelemetryUpdatePeriod(void)
{
  return pdlTelemetryTS.period;
}

void pdlRcTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlRcTS))
    return;

  pdlRemoteControl(ds);
}

void pdlTelemetryTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlTelemetryTS))
    return;

  pdlUpdateTelemetry(ds);
}

void pdlBatteryTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlBatteryTS))
    return;

  pdlReadBattery(ds);
}

void pdlUpdateLevelsPid(pdlDroneState *ds)
{
  static uint32_t levelPidsLastUpdateTime = 0;
  float dt;

  dt = pdlGetDeltaTime(pdlMicros(), levelPidsLastUpdateTime);
  levelPidsLastUpdateTime = pdlMicros();
  dt = dt / 1000000.f;

  pdlUpdatePid(&ds->pitchPid, ds->pitch, dt);
  pdlUpdatePid(&ds->rollPid, ds->roll, dt);

  // apply level pids to angular rate pids
  if(ds->stabilizationEnabled)
  {
    if(ds->pitchPid.enabled)
      ds->pitchRatePid.target = ds->pitchPid.out;
    if(ds->rollPid.enabled)
      ds->rollRatePid.target = ds->rollPid.out;
  }

  // update angular rates pids
  pdlUpdatePid(&ds->yawRatePid, ds->gyro.pure[PDL_Z], dt);
  pdlUpdatePid(&ds->pitchRatePid, ds->gyro.pure[PDL_Y], dt);
  pdlUpdatePid(&ds->rollRatePid, ds->gyro.pure[PDL_X], dt);
}

void pdlImuTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlImuTS))
    return;

  pdlReadAccel(ds);
  pdlReadGyro(ds);
  pdlReadMagneto(ds);

  pdlTripleAxisSensorFusion(ds);

  pdlUpdateLevelsPid(ds);
}

void pdlUpdateAltPid(pdlDroneState *ds)
{
  static uint32_t lastUpdateTime = 0;
  float dt;

  dt = pdlGetDeltaTime(pdlMicros(), lastUpdateTime);
  lastUpdateTime = pdlMicros();
  dt = dt / 1000000.f;

  // apply alt pid
  if(ds->stabilizationEnabled)
  {
    pdlUpdatePid(&ds->velocityZPid, ds->velocity[PDL_Z], dt);

    if(ds->velocityZPid.enabled)
      ds->altPid.target += ds->velocityZPid.out;

    pdlUpdatePid(&ds->altPid, ds->altitude, dt);

    if(ds->altPid.enabled)
      ds->baseGas = ds->altPid.out;
  }
}

void pdlBaroTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlBaroTS))
    return;

  if(pdlReadBaro(ds))
    pdlUpdateAltPid(ds);
}

void pdlLidarTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlLidarTS))
    return;

  if(pdlReadLidar(ds))
    pdlUpdateAltPid(ds);
}

void pdlUpdateHorVelocityPids(pdlDroneState *ds)
{
  static uint32_t lastUpdateTime = 0;
  float dt;

  dt = pdlGetDeltaTime(pdlMicros(), lastUpdateTime);
  lastUpdateTime = pdlMicros();
  dt = dt / 1000000.f;

  pdlUpdatePid(&ds->velocityXPid,ds->velocity[PDL_X],dt);
  pdlUpdatePid(&ds->velocityYPid,ds->velocity[PDL_Y],dt);

  if(ds->holdPosEnabled && ds->stabilizationEnabled)
  {
    if( ds->velocityYPid.enabled &&
        (ds->holdPosEnabled == PDL_HOLDPOS_Y || ds->holdPosEnabled == PDL_HOLDPOS_BOTH_XY) )
    {
      ds->rollPid.target = ds->velocityYPid.out;
    }
    else
    {
      pdlResetPid(&ds->velocityYPid);
    }

    if( ds->velocityXPid.enabled &&
        (ds->holdPosEnabled == PDL_HOLDPOS_X || ds->holdPosEnabled == PDL_HOLDPOS_BOTH_XY) )
    {
      ds->pitchPid.target = -ds->velocityXPid.out;
    }
    else
    {
      pdlResetPid(&ds->velocityXPid);
    }
  }
  else
  {
    pdlResetPid(&ds->velocityXPid);
    pdlResetPid(&ds->velocityYPid);
  }
}

void pdlOpticalFlowTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlOpticalFlowTS))
    return;

  pdlReadOpticalFlow(ds);
  pdlUpdateHorVelocityPids(ds);
}

void pdlUpdateEscaperTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlUpdateEscaperTS))
    return;

  pdlUpdateEscaper(ds);
}

void pdlUpdate(pdlDroneState *ds)
{
  ds->timestamp = pdlMicros();

  pdlUpdateStartTime = pdlMicros();

  pdlUpdateEscaperTask(ds);
  pdlImuTask(ds);
  pdlLidarTask(ds);
  pdlOpticalFlowTask(ds);
  pdlRcTask(ds);
  pdlBaroTask(ds);
  pdlTelemetryTask(ds);
  pdlBatteryTask(ds);

  if(ds->stabilizationEnabled && ds->motorsEnabled)
  {
    if(pdlPidModifiedFlag)
    {
      pdlPidModifiedFlag = 0;
      // apply pids result to motors
      switch(PDL_DRONE_FRAME)
      {
        case PDL_DRONE_FRAME_CROSS:
          pdlCrossFrameApplyPids(ds);
          break;
        case PDL_DRONE_FRAME_X:
          pdlXFrameApplyPids(ds);
          break;
      }
    }
  }
  else
  {
    pdlResetPid(&ds->yawRatePid);
    pdlResetPid(&ds->pitchRatePid);
    pdlResetPid(&ds->rollRatePid);
    pdlResetPid(&ds->pitchPid);
    pdlResetPid(&ds->rollPid);
    pdlResetPid(&ds->altPid);
    pdlResetPid(&ds->velocityXPid);
    pdlResetPid(&ds->velocityYPid);
    pdlResetPid(&ds->velocityZPid);
  }

  if(!ds->motorsEnabled)
  {
    pdlStopMotors(ds);
  }
}

void pdlStopMotors(pdlDroneState *ds)
{
  ds->motorsEnabled = 0;
  ds->baseGas = PDL_MOTOR_GAS_NULL;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, PDL_MOTOR_GAS_NULL);
}

void pdlCrossFrameApplyPids(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  dg[0] += -ds->yawRatePid.out + ds->pitchRatePid.out;
  dg[1] += ds->yawRatePid.out + ds->rollRatePid.out;
  dg[2] += -ds->yawRatePid.out - ds->pitchRatePid.out;
  dg[3] += ds->yawRatePid.out - ds->rollRatePid.out;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg[i]);
}

void pdlXFrameApplyPids(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  dg[0] += -ds->yawRatePid.out + ds->pitchRatePid.out + ds->rollRatePid.out;
  dg[1] += ds->yawRatePid.out - ds->pitchRatePid.out + ds->rollRatePid.out;
  dg[2] += -ds->yawRatePid.out - ds->pitchRatePid.out - ds->rollRatePid.out;
  dg[3] += ds->yawRatePid.out + ds->pitchRatePid.out - ds->rollRatePid.out;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg[i]);
}

void pdlSetMotorGas(pdlDroneState *ds, uint8_t m, int32_t gas)
{
  if(m >= PDL_MOTOR_COUNT)
    return;

  if(gas < PDL_MOTOR_GAS_MIN)
    gas = PDL_MOTOR_GAS_MIN;

  if(gas > PDL_MOTOR_GAS_MAX)
    gas = PDL_MOTOR_GAS_MAX;

  ds->motorGas[m] = gas;
}

void pdlAddMotorGas(pdlDroneState *ds, uint8_t m, int32_t gas)
{
  if(m >= PDL_MOTOR_COUNT)
    return;

  pdlSetMotorGas(ds, m, ds->motorGas[m] + gas);
}

void pdlUpdatePid(pdlPidState *pid, float input, float dt)
{
  if(!pid->enabled)
  {
    pdlResetPid(pid);
    return;
  }

  float err = pid->target - input;
  float prevErr = pid->target - pid->input;

  //output = Un1 + (kp + kd/dt)*En + (-kp - 2.f*kd/dt)*En1 + kd/dt*En2;
  //output = Un1 + kp*(En - En1) + ki_d*En + kd_d*(En - 2.f*En1 + En2);
  if(fabsf(pid->errSum) * pid->ki < pid->maxOut)
    pid->errSum += dt*(err + prevErr)/2.f;
  /*if(fabsf(pid->errSum) > fabsf(pid->maxErrSum))
  {
    if(pid->errSum > 0)
      pid->errSum = pid->maxErrSum;
    else
      pid->errSum = -pid->maxErrSum;
  }*/

  pid->out = pid->kp*err + pid->ki*pid->errSum + pid->kd*(err - prevErr)/dt;
  pid->input = input;
  if(fabsf(pid->out) > fabsf(pid->maxOut))
  {
    if(pid->out > 0)
      pid->out = pid->maxOut;
    else
      pid->out = -pid->maxOut;
  }

  pdlPidModifiedFlag = 1;
}

void pdlResetPid(pdlPidState *pid)
{
  pid->errSum = 0;
  pid->input = 0;
  pid->out = 0;
}

void pdlParseCommand(pdlDroneState *ds, uint8_t *packet)
{
  int32_t t0,t1,t2,t3;
  uint32_t ut0;
  float yaw,pitch,roll;
  uint8_t cmd = packet[0];
  switch(cmd)
  {
    case PDL_CMD_SWITCH_MOTORS:
      pdlStopMotors(ds);
      ds->motorsEnabled = packet[1];
      if(ds->motorsEnabled)
      {
        ds->yawRatePid.target = 0;
        ds->pitchRatePid.target = 0;
        ds->rollRatePid.target = 0;
        ds->pitchPid.target = 0;
        ds->rollPid.target = 0;
        ds->altPid.target = 0;
        ds->velocityXPid.target = 0;
        ds->velocityYPid.target = 0;
        ds->velocityZPid.target = 0;
        ds->holdPosEnabled = PDL_HOLDPOS_BOTH_XY;
      }
      break;
    case PDL_CMD_SET_BASE_GAS:
      memcpy(&t0, &packet[1], sizeof(int32_t));
      ds->baseGas = t0;
      if(!ds->stabilizationEnabled)
      {
        pdlSetMotorGas(ds,0,ds->baseGas);
        pdlSetMotorGas(ds,1,ds->baseGas);
        pdlSetMotorGas(ds,2,ds->baseGas);
        pdlSetMotorGas(ds,3,ds->baseGas);
      }
      break;
    case PDL_CMD_SET_MOTORS_GAS:
      memcpy(&t0, &packet[1], sizeof(int32_t));
      memcpy(&t1, &packet[5], sizeof(int32_t));
      memcpy(&t2, &packet[9], sizeof(int32_t));
      memcpy(&t3, &packet[13], sizeof(int32_t));

      pdlSetMotorGas(ds,0,t0);
      pdlSetMotorGas(ds,1,t1);
      pdlSetMotorGas(ds,2,t2);
      pdlSetMotorGas(ds,3,t3);

      break;
    case PDL_CMD_SET_ACCEL:
      parseTripleAxisSensorConfigPacket(&ds->accel, packet);
      pdlSetupAccel(ds);
      break;
    case PDL_CMD_SET_GYRO:
      parseTripleAxisSensorConfigPacket(&ds->gyro, packet);
      pdlSetupGyro(ds);
      break;
    case PDL_CMD_SET_MAGNETO:
      parseTripleAxisSensorConfigPacket(&ds->magneto, packet);
      pdlSetupMagneto(ds);
      break;
    case PDL_CMD_SELF_CALIB_ACCEL:
      pdlCalibrateAccel(ds);
      break;
    case PDL_CMD_SELF_CALIB_GYRO:
      pdlCalibrateGyro(ds);
      break;
    case PDL_CMD_SET_YAW_RATE_PID:
      parsePidConfigPacket(&ds->yawRatePid, packet);
      break;
    case PDL_CMD_SET_PITCH_RATE_PID:
      parsePidConfigPacket(&ds->pitchRatePid, packet);
      break;
    case PDL_CMD_SET_ROLL_RATE_PID:
      parsePidConfigPacket(&ds->rollRatePid, packet);
      break;
    case PDL_CMD_SET_PITCH_PID:
      parsePidConfigPacket(&ds->pitchPid, packet);
      break;
    case PDL_CMD_SET_ROLL_PID:
      parsePidConfigPacket(&ds->rollPid, packet);
      break;
    case PDL_CMD_SET_ALT_PID:
      parsePidConfigPacket(&ds->altPid, packet);
      break;
    case PDL_CMD_SET_VELOCITY_X_PID:
      parsePidConfigPacket(&ds->velocityXPid, packet);
      break;
    case PDL_CMD_SET_VELOCITY_Y_PID:
      parsePidConfigPacket(&ds->velocityYPid, packet);
      break;
    case PDL_CMD_SET_VELOCITY_Z_PID:
      parsePidConfigPacket(&ds->velocityZPid, packet);
      break;
    case PDL_CMD_SET_YPR:
      memcpy(&yaw, &packet[1], sizeof(yaw));
      memcpy(&pitch, &packet[1 + sizeof(pitch)], sizeof(pitch));
      memcpy(&roll, &packet[1 + sizeof(yaw) + sizeof(pitch)], sizeof(roll));

      ds->yawRatePid.target = yaw;

      if(ds->pitchPid.enabled)
        ds->pitchPid.target = pitch;
      else
        ds->pitchRatePid.target = pitch;

      if(ds->rollPid.enabled)
        ds->rollPid.target = roll;
      else
        ds->rollRatePid.target = roll;
      // disable hold pos if we have user defined target
      ds->holdPosEnabled = PDL_HOLDPOS_BOTH_XY;
      if(roll != 0.f && pitch != 0.f)
        ds->holdPosEnabled = PDL_HOLDPOS_DISABLED;
      else if(pitch != 0.f)
        ds->holdPosEnabled = PDL_HOLDPOS_Y;
      else if(roll != 0.f)
        ds->holdPosEnabled = PDL_HOLDPOS_X;
      break;
    case PDL_CMD_SET_TELEMETRY_PERIOD:
      memcpy(&ut0, &packet[1], sizeof(ut0));
      pdlSetTelemetryUpdatePeriod(ut0);
      break;
    case PDL_CMD_ENABLE_STABILIZATION:
      ds->stabilizationEnabled = packet[1];
      break;
    case PDL_CMD_RESET_ALTITUDE:
      //vz = 0;
      ds->baro.seaLevelPressure = ds->baro.pressure;
      break;
    case PDL_CMD_SET_SEA_LEVEL:
      memcpy(&ds->baro.seaLevelPressure, &packet[1], sizeof(ds->baro.seaLevelPressure));
      break;
    case PDL_CMD_SET_ALTITUDE:
      memcpy(&ds->altPid.target, &packet[1], sizeof(ds->altPid.target));
      break;
    case PDL_CMD_SET_VELOCITY_Z:
      memcpy(&ds->velocityZPid.target, &packet[1], sizeof(ds->velocityZPid.target));
      break;
  }
}

void parseTripleAxisSensorConfigPacket(pdlTripleAxisSensorState *ps, uint8_t *packet)
{
  int16_t dx,dy,dz;

  memcpy(&dx, &packet[1], sizeof(int16_t));
  memcpy(&dy, &packet[3], sizeof(int16_t));
  memcpy(&dz, &packet[5], sizeof(int16_t));

  ps->offset[PDL_X] = dx;
  ps->offset[PDL_Y] = dy;
  ps->offset[PDL_Z] = dz;
}

void parsePidConfigPacket(pdlPidState *pid, uint8_t *packet)
{
  float kp,ki,kd,maxOut,maxErrSum;
  uint8_t enabled = packet[1];
  memcpy(&kp, &packet[2], sizeof(kp));
  memcpy(&ki, &packet[2 + sizeof(kp)], sizeof(ki));
  memcpy(&kd, &packet[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
  memcpy(&maxOut, &packet[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
  memcpy(&maxErrSum, &packet[2 + sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(maxErrSum)], sizeof(maxErrSum));

  pid->enabled = enabled;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->maxOut = maxOut;
  //pid->maxErrSum = maxErrSum;
}
