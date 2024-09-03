#include "pdl.h"
#include "pdl_tasks.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

char pdlLog[PDL_LOG_BUF_SIZE];
int pdlLogPos = 0;

int pdl_printf(const char *format, ... )
{
  if(pdlLogPos >= PDL_LOG_BUF_SIZE)
    return 0;

  va_list args;
  va_start(args, format);
  int res = vsnprintf(&pdlLog[pdlLogPos],PDL_LOG_BUF_SIZE - pdlLogPos,format,args);
  va_end(args);

  if(res > 0)
    pdlLogPos += res;

  if(pdlLogPos > PDL_LOG_BUF_SIZE)
    pdlLogPos = PDL_LOG_BUF_SIZE;

  return res;
}

char* pdlGetLog()
{
  return &pdlLog[0];
}

int pdlGetLogSize()
{
  return pdlLogPos;
}

void pdlResetLog()
{
  memset(&pdlLog[0],0,PDL_LOG_BUF_SIZE);
  pdlLogPos = 0;
}

void pdlSetup(pdlDroneState *ds)
{
  memset(ds,0,sizeof(pdlDroneState));

  pdlSetupTasks();

  ds->battery.minVoltage = 6.2f;
  ds->battery.maxVoltage = 8.3f;

  ds->kfSettings.navModelNoise = 0.05f;
  ds->kfSettings.accVariance = 10.f;
  ds->kfSettings.baroAltVariance = 50.f;
  ds->kfSettings.lidarVelVariance1 = 0.03f;
  ds->kfSettings.ofVelVariance1 = 0.015f;
  ds->kfSettings.ofVelVariance2 = 0.095f;
  ds->kfSettings.lidarVelVariance2 = 1000000.f;
  ds->kfSettings.poseModelNoise = 20.f;
  ds->kfSettings.yawRateVariance = 0.0035f;
  ds->kfSettings.pitchRollRateVariance = 0.013f;
  ds->kfSettings.magHeadingVariance = 0.1f;
  ds->kfSettings.accPitchRollVariance = 0.0041f;
  ds->kfSettings.gpsHorSpeedVariance = 1.f;
  ds->kfSettings.gpsVerSpeedVariance = 1.f;
  ds->kfSettings.gpsHorPosVariance = 10.f;
  ds->kfSettings.gpsVerPosVariance = 10.f;

  pdlResetKalman(ds);
  pdlStopMotors(ds);
  pdlLoadDefaultCfg(ds);
  // reset errors flag after load
  // because before we saved entire pdl.DroneState with errors flag
  // which is not actual after load config
  ds->errors = 0;
  ds->videoState = 0;
  ds->motorsEnabled = 0;
  ds->stabilizationEnabled = 0;
  ds->version = PDL_VERSION;
  pdlSetupEsc(ds);
  pdlSetupRc(ds);
  pdlSetupTelemetry(ds);
  pdlSetupGyro(ds);
  pdlSetupAccel(ds);
  pdlSetupMagneto(ds);
  pdlSetupBaro(ds);
  pdlSetupLidar(ds);
  pdlSetupOpticalFlow(ds);
  pdlSetupBattery(ds);
  pdlSetupGps(ds);
  pdlSetupLoads(ds);
  pdlSetupCamera(ds);
}

void pdlUpdate(pdlDroneState *ds)
{
  ds->timestamp = pdlMicros();

  pdlPidTask(ds);
  pdlEscTask(ds);
  pdlGyroTask(ds);
  pdlAccelTask(ds);
  pdlMagTask(ds);
  pdlLidarTask(ds);
  pdlOpticalFlowTask(ds);
  pdlBaroTask(ds);
  pdlGpsTask(ds);
  pdlRcTask(ds);
  pdlTelemetryTask(ds);
  pdlBatteryTask(ds);
  pdlLoadTask(ds);

  if(!ds->motorsEnabled)
  {
    pdlStopMotors(ds);
  }
}

//-----------------------------------------------------------------------------
// MOTORS

static int32_t pdlMotorMaxGas = PDL_DEFAULT_MOTOR_GAS_MAX;
static int32_t pdlMotorNullGas = PDL_DEFAULT_MOTOR_GAS_NULL;
static int32_t pdlMotorMinGas = PDL_DEFAULT_MOTOR_GAS_MIN;

void pdlSetMotorMaxGas(int32_t gas)
{
  pdlMotorMaxGas = gas;
}

void pdlSetMotorNullGas(int32_t gas)
{
  pdlMotorNullGas = gas;
}

void pdlSetMotorMinGas(int32_t gas)
{
  pdlMotorMinGas = gas;
}

int32_t pdlGetMotorMaxGas()
{
  return pdlMotorMaxGas;
}

int32_t pdlGetMotorNullGas()
{
  return pdlMotorNullGas;
}

int32_t pdlGetMotorMinGas()
{
  return pdlMotorMinGas;
}

void pdlStopMotors(pdlDroneState *ds)
{
  ds->motorsEnabled = 0;
  ds->baseGas = pdlMotorNullGas;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++) {
    pdlSetMotorGas(ds, i, pdlMotorNullGas);
  }
}

void pdlSetBaseGas(pdlDroneState* ds, int32_t gas)
{
  if(gas < pdlMotorMinGas)
      gas = pdlMotorMinGas;

    if(gas > pdlMotorMaxGas)
      gas = pdlMotorMaxGas;

    ds->baseGas = gas;
}

void pdlSetMotorGas(pdlDroneState *ds, uint8_t m, int32_t gas)
{
  if(m >= PDL_MOTOR_COUNT)
    return;

  if(gas < pdlMotorMinGas)
    gas = pdlMotorMinGas;

  if(gas > pdlMotorMaxGas)
    gas = pdlMotorMaxGas;

  ds->motorGas[m] = gas;
}

void pdlAddMotorGas(pdlDroneState *ds, uint8_t m, int32_t gas)
{
  if(m >= PDL_MOTOR_COUNT)
    return;

  pdlSetMotorGas(ds, m, ds->motorGas[m] + gas);
}

//-----------------------------------------------------------------------------
// OTHER

void pdlPreventFlyAway(pdlDroneState *ds, int32_t rssiMinLevel, float safeAlt, float safeVeloZ)
{
  static int32_t old_rssi = 0;

  if(ds->motorsEnabled && ds->stabilizationEnabled)
  {
    if(ds->rc.rssi < rssiMinLevel && old_rssi >= rssiMinLevel)
    {
      // when drone gets out control zone
      // lets stop it
      ds->posNorthPid.target = ds->posNorthPid.input;
      ds->velocityXPid.target = 0;
      ds->pitchPid.target = 0;
      ds->posEastPid.target = ds->posEastPid.input;
      ds->velocityYPid.target = 0;
      ds->rollPid.target = 0;
      ds->velocityZPid.target = 0;
      ds->altPid.target = ds->altPid.input;
      ds->xRatePid.target = 0;
      ds->yRatePid.target = 0;
      ds->zRatePid.target = 0;
      ds->headingPid.target = ds->headingPid.input;
      if(ds->nav[PDL_UP].pos > safeAlt)
      {
        if(safeVeloZ > 0.f)
          safeVeloZ = -safeVeloZ;

        ds->velocityZPid.target = safeVeloZ;
      }
      else if(ds->nav[PDL_UP].pos < -safeAlt)
      {
        if(safeVeloZ < 0.f)
          safeVeloZ = -safeVeloZ;

        ds->velocityZPid.target = safeVeloZ;
      }
      // turn off motors if we in acro mode because no other way to stop drone
      if(!ds->rollPid.enabled || !ds->pitchPid.enabled)
        ds->motorsEnabled = 0;
    }
  }

  old_rssi = ds->rc.rssi;
}

void pdlSetPitchTarget(pdlDroneState* ds, float target)
{
  ds->pitchPid.target = target;

  if(target == 0)
  {
    pdlSetPidVeloXFlag(ds,1);
    // we restore posXPid flag in pdlUpdateHorVelocityPids
  }
  else
  {
    pdlSetPidVeloXFlag(ds,0);
    pdlSetPidPosNorthFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
  }
}

void pdlSetRollTarget(pdlDroneState* ds, float target)
{
  ds->rollPid.target = target;

  if(target == 0)
  {
    pdlSetPidVeloYFlag(ds,1);
    // we restore posYPid flag in pdlUpdateHorVelocityPids
  }
  else
  {
    pdlSetPidVeloYFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
    pdlSetPidPosNorthFlag(ds,0);
  }
}

void pdlSetYawRateTarget(pdlDroneState* ds, float target)
{
  ds->yawRateTarget = target;

  if(target == 0)
  {
    pdlSetPidHeadingFlag(ds,1);
  }
  else
  {
    pdlSetPidHeadingFlag(ds,0);
  }
}

void pdlSetHeadingTarget(pdlDroneState *ds, float target)
{
  ds->headingPid.target = target;
}

void pdlSetPitchRateTarget(pdlDroneState *ds, float target)
{
  ds->pitchRateTarget = target;

  if(target == 0)
  {
    pdlSetPidPitchFlag(ds,1);
    pdlSetPidVeloXFlag(ds,1);
    // we restore posXPid flag in pdlUpdateHorVelocityPids
  }
  else
  {
    pdlSetPidPitchFlag(ds,0);
    pdlSetPidVeloXFlag(ds,0);
    pdlSetPidPosNorthFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
  }
}

void pdlSetRollRateTarget(pdlDroneState *ds, float target)
{
  ds->rollRateTarget = target;

  if(target == 0)
  {
    pdlSetPidRollFlag(ds,1);
    pdlSetPidVeloYFlag(ds,1);
    // we restore posYPid flag in pdlUpdateHorVelocityPids
  }
  else
  {
    pdlSetPidRollFlag(ds,0);
    pdlSetPidVeloYFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
    pdlSetPidPosNorthFlag(ds,0);
  }
}

void pdlSetZRateTarget(pdlDroneState *ds, float target)
{
  uint8_t flag = 0;

  ds->zRatePid.target = target;

  // we can't control the drone by body angular rates and euler angular rates simultaneously
  // to prevent it we reset targets of euler angular rates
  ds->yawRateTarget = 0;
  ds->pitchRateTarget = 0;
  ds->rollRateTarget = 0;
  // the user could send the request to control the drone by x or y body rate previously
  // if pitchPid/rollPid is not reseted then there was not the control request of the body-y rate
  // thus we can reset angular body rates
  // note: we loose the control by pitch/roll at this time if it is chosen the control by pitch/roll
  // thus it is not recommended to set the control by z-rate and pitch/roll simultaneously
  if(pdlGetPidPitchFlag(ds))
  {
    ds->yRatePid.target = 0;
  }
  if(pdlGetPidRollFlag(ds))
  {
    ds->xRatePid.target = 0;
  }

  // disable up-level PIDs because they disturb to control the drone by the angular rate
  // and enable back when there isn't the control request from user

  if( ds->zRatePid.target == 0 &&
      ds->xRatePid.target == 0 &&
      ds->yRatePid.target == 0 )
  {
    flag = 1;
  }
  else
  {
    pdlSetPidPosNorthFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
  }

  pdlSetPidHeadingFlag(ds,flag);
  pdlSetPidPitchFlag(ds,flag);
  pdlSetPidRollFlag(ds,flag);
  pdlSetPidVeloXFlag(ds,flag);
  pdlSetPidVeloYFlag(ds,flag);
}

void pdlSetXRateTarget(pdlDroneState *ds, float target)
{
  uint8_t flag = 0;

  ds->xRatePid.target = target;

  ds->yawRateTarget = 0;
  ds->pitchRateTarget = 0;
  ds->rollRateTarget = 0;

  if(pdlGetPidPitchFlag(ds))
  {
    ds->yRatePid.target = 0;
  }
  if(pdlGetPidHeadingFlag(ds))
  {
    ds->zRatePid.target = 0;
  }

  if( ds->zRatePid.target == 0 &&
      ds->xRatePid.target == 0 &&
      ds->yRatePid.target == 0 )
  {
    flag = 1;
  }
  else
  {
    pdlSetPidPosNorthFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
  }

  pdlSetPidHeadingFlag(ds,flag);
  pdlSetPidPitchFlag(ds,flag);
  pdlSetPidRollFlag(ds,flag);
  pdlSetPidVeloXFlag(ds,flag);
  pdlSetPidVeloYFlag(ds,flag);
}

void pdlSetYRateTarget(pdlDroneState *ds, float target)
{
  uint8_t flag = 0;

  ds->yRatePid.target = target;

  ds->yawRateTarget = 0;
  ds->pitchRateTarget = 0;
  ds->rollRateTarget = 0;

  if(pdlGetPidRollFlag(ds))
  {
    ds->xRatePid.target = 0;
  }
  if(pdlGetPidHeadingFlag(ds))
  {
    ds->zRatePid.target = 0;
  }

  if( ds->zRatePid.target == 0 &&
      ds->xRatePid.target == 0 &&
      ds->yRatePid.target == 0 )
  {
    flag = 1;
  }
  else
  {
    pdlSetPidPosNorthFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
  }

  pdlSetPidHeadingFlag(ds,flag);
  pdlSetPidPitchFlag(ds,flag);
  pdlSetPidRollFlag(ds,flag);
  pdlSetPidVeloXFlag(ds,flag);
  pdlSetPidVeloYFlag(ds,flag);
}

void pdlSetVelocityXTarget(pdlDroneState *ds, float t)
{
  ds->velocityXPid.target = t;

  if(ds->velocityXPid.target != 0.f)
  {
    pdlSetPidPosNorthFlag(ds,0);
    pdlSetPidPosEastFlag(ds,0);
    // we restore posXPid flag in pdlUpdateHorVelocityPids
  }
}

void pdlSetVelocityYTarget(pdlDroneState *ds, float t)
{
  ds->velocityYPid.target = t;

  if(ds->velocityYPid.target != 0.f)
  {
    pdlSetPidPosEastFlag(ds,0);
    pdlSetPidPosNorthFlag(ds,0);
    // we restore posYPid flag in pdlUpdateHorVelocityPids
  }
}

void pdlSetVelocityZTarget(pdlDroneState *ds, float t)
{
  ds->velocityZPid.target = t;

  if(ds->velocityZPid.target != 0.f)
  {
    pdlSetPidAltFlag(ds,0);
    // we restore altPid flag in pdlUpdateAltPid
  }
}

void pdlEnableMotors(pdlDroneState *ds, uint8_t enable)
{
  pdlStopMotors(ds);
  ds->motorsEnabled = enable;
  if(ds->motorsEnabled)
  {
    ds->zRatePid.target = 0;
    ds->yRatePid.target = 0;
    ds->xRatePid.target = 0;
    ds->yawRateTarget = 0;
    ds->pitchRateTarget = 0;
    ds->rollRateTarget = 0;
    ds->pitchPid.target = 0;
    ds->rollPid.target = 0;
    ds->altPid.target = 0;
    ds->posNorthPid.target = 0;
    ds->posEastPid.target = 0;
    ds->velocityXPid.target = 0;
    ds->velocityYPid.target = 0;
    ds->velocityZPid.target = 0;
    ds->pidFlags = PDL_PID_ALL_FLAG;
    ds->headingPid.target = ds->pose[PDL_YAW].pos;
    ds->opticalFlow.sumX = 0;
    ds->opticalFlow.sumY = 0;
    // to prevent d-reg from first run error
    ds->headingPid.input = ds->headingPid.target;
    // to prevent overflowed err-sum in PIDs if user switch on stabilization before
    ds->stabilizationEnabled = 0;
    // reset Kalman filters
    for(uint8_t i = 0; i < 3; i++)
    {
      // can't use pdlNavKF_Init to prevent reset accBias
      ds->nav[i].pos = 0;
      ds->nav[i].vel = 0;
      ds->nav[i].acc = 0;

      ds->pose[i].pos = 0;
      ds->pose[i].vel = 0;
      ds->pose[i].acc = 0;
    }
    ds->pose[PDL_YAW].pos = ds->magneto.heading;
    ds->pose[PDL_PITCH].pos = ds->accel.pitch;
    ds->pose[PDL_ROLL].pos = ds->accel.roll;
    // store gps position
    ds->gps.startLat = ds->gps.curLat;
    ds->gps.startLon = ds->gps.curLon;
    ds->gps.startAlt = ds->gps.curAlt;
  }
}

void pdlEnableStabilization(pdlDroneState *ds, uint8_t enable)
{
  ds->stabilizationEnabled = enable;
  if(ds->stabilizationEnabled)
  {
    ds->pidFlags = PDL_PID_ALL_FLAG;
    ds->headingPid.target = ds->pose[PDL_YAW].pos;
  }
}

void pdlEnableTrickMode(pdlDroneState *ds, uint8_t enable)
{
  ds->trickModeEnabled = enable;
}

void pdlSetEscMode(pdlDroneState *ds, uint8_t mode)
{
  ds->esc = mode;
  pdlSetupEsc(ds);
}

void pdlSetFrameType(pdlDroneState *ds, uint8_t frame)
{
  ds->frame = frame;
}
