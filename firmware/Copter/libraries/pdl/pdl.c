#include "pdl.h"
#include <stdlib.h>
#include <math.h>

int32_t pdlMinGas;
int32_t pdlNullGas;
int32_t pdlMaxGas;
uint32_t pdlImuReadPeriod;
uint32_t pdlBaroReadPeriod;
uint32_t pdlBatteryReadPeriod;
uint32_t pdlEscaperUpdatePeriod;
uint32_t pdlRcUpdatePeriod;

uint32_t pdlGetDeltaTime(uint32_t cur, uint32_t last);
void pdlCrossFrameApplyPids(pdlDroneState*);
void pdlXFrameApplyPids(pdlDroneState*);

uint32_t pdlGetDeltaTime(uint32_t cur, uint32_t last)
{
  if(cur >= last)
    return cur - last;

  return UINT32_MAX - last + cur;
}

void pdlSetup(pdlDroneState *ds)
{
  pdlMinGas = PDL_DEFAULT_MIN_MOTOR_GAS;
  pdlNullGas = PDL_DEFAULT_NULL_MOTOR_GAS;
  pdlMaxGas = PDL_DEFAULT_MAX_MOTOR_GAS;
  pdlImuReadPeriod = PDL_DEFAULT_IMU_READ_PERIOD;
  pdlBaroReadPeriod = PDL_DEFAULT_BARO_READ_PERIOD;
  pdlBatteryReadPeriod = PDL_DEFAULT_BATTERY_READ_PERIOD;
  pdlEscaperUpdatePeriod = PDL_DEFAULT_ESCAPER_UPDATE_PERIOD;
  pdlRcUpdatePeriod = PDL_DEFAULT_RC_UPDATE_PERIOD;

  ds->version = PDL_DRONE_STATE_VERSION;

  pdlStopMotors(ds);
  pdlSetupEscaper(ds);
  pdlSetupAccel(ds);
  pdlSetupGyro(ds);
  pdlSetupMagneto(ds);
  pdlSetupBaro(ds);
  pdlSetupRc(ds);
}

void pdlUpdate(pdlDroneState *ds)
{
  static uint32_t imuLastReadTime = 0;
  static uint32_t baroLastReadTime = 0;
  static uint32_t batteryLastReadTime = 0;
  static uint32_t escaperLastUpdateTime = 0;
  static uint32_t rcLastUpdateTime = 0;

  ds->timestamp = pdlMicros();

  if(pdlGetDeltaTime(ds->timestamp,rcLastUpdateTime) >= pdlRcUpdatePeriod)
  {
    rcLastUpdateTime = ds->timestamp;
    pdlRemoteControl(ds);
  }

  ds->timestamp = pdlMicros();

  if(pdlGetDeltaTime(ds->timestamp,batteryLastReadTime) >= pdlBatteryReadPeriod)
  {
    batteryLastReadTime = ds->timestamp;
    pdlReadBattery(ds);
  }

  ds->timestamp = pdlMicros();

  if(pdlGetDeltaTime(ds->timestamp,baroLastReadTime) >= pdlBaroReadPeriod)
  {
    baroLastReadTime = ds->timestamp;
    pdlReadBaro(ds);
  }

  ds->timestamp = pdlMicros();

  if(pdlGetDeltaTime(ds->timestamp,imuLastReadTime) >= pdlImuReadPeriod)
  {
    float dt = (float)pdlGetDeltaTime()/1000000.f;

    imuLastReadTime = ds->timestamp;
    pdlReadAccel(ds);
    pdlReadGyro(ds);
    pdlReadMagneto(ds);
    pdlTripleAxisSensorFusion(ds);

    if(ds->stabilizationEnabled && ds->motorsEnabled)
    {
      pdlPidUpdate(&ds->yawRatePid, ds->gyro.pure[PDL_Z], dt);
      pdlPidUpdate(&ds->pitchPid, ds->pitch, dt);
      pdlPidUpdate(&ds->rollPid, ds->roll, dt);

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
    else
    {
      pdlResetPid(&ds->yawRatePid);
      pdlResetPid(&ds->pitchPid);
      pdlResetPid(&ds->rollPid);

    }
  }

  if(!ds->motorsEnabled)
  {
    pdlStopMotors();
  }

  ds->timestamp = pdlMicros();

  if(pdlGetDeltaTime(ds->timestamp, escaperLastUpdateTime) >= pdlEscaperUpdatePeriod)
  {
    escaperLastUpdateTime = ds->timestamp;
    pdlUpdateEscaper(ds);
  }
}

void pdlStopMotors(pdlDroneState *ds)
{
  ds->motorsEnabled = 0;
  ds->baseGas = pdlNullGas;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, pdlNullGas);
}

void pdlCrossFrameApplyPids(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  dg[0] += ds->pitchPid.out - ds->yawRatePid.out;
  dg[1] += ds->rollPid.out + ds->yawRatePid.out;

#ifndef PDL_ASYMMETRIC_STABILIZATION
  dg[2] += -ds->pitchPid.out - ds->yawRatePid.out;
  dg[3] += -ds->rollPid.out + ds->yawRatePid.out;
#endif

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg);
}

void pdlXFrameApplyPids(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  dg[1] += ds->yawRatePid.out + ds->pitchPid.out;
  dg[2] += ds->pitchPid.out + ds->rollPid.out;
  dg[3] += ds->yawRatePid.out + ds->rollPid.out;

#ifndef PDL_ASYMMETRIC_STABILIZATION
  dg[0] += -ds->yawRatePid.out - ds->pitchPid.out - ds->rollPid.out;
  dg[1] += -ds->rollPid.out;
  dg[2] += -ds->yawRatePid.out;
  dg[3] += -ds->pitchPid.out;
#endif

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg);
}

void pdlSetMotorGasLimits(int32_t min, int32_t nul, int32_t max)
{
  pdlMinGas = min;
  pdlNullGas = nul;
  pdlMaxGas = max;
}

void pdlSetImuReadPeriod(uint32_t t)
{
  pdlImuReadPeriod = t;
}

void pdlSetBaroReadPeriod(uint32_t t)
{
  pdlBaroReadPeriod = t;
}

void pdlSetBatteryReadPeriod(uint32_t t)
{
  pdlBatteryReadPeriod = t;
}

void pdlSetEscaperUpdatePeriod(uint32_t t)
{
  pdlEscaperUpdatePeriod = t;
}

void pdlSetRcUpdatePeriod(uint32_t t)
{
  pdlRcUpdatePeriod = t;
}

void pdlSetMotorGas(pdlDroneState *ds, uint8_t m, int32_t gas)
{
  if(m >= PDL_MOTOR_COUNT)
    return;

  if(gas < pdlMinGas)
    gas = pdlMinGas;

  if(gas > pdlMaxGas)
    gas = pdlMaxGas;

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

  //output = Un1 + (kp + kd/dt)*En + (-kp - 2.f*kd/dt)*En1 + kd/dt*En2;
  //output = Un1 + kp*(En - En1) + ki_d*En + kd_d*(En - 2.f*En1 + En2);
  pid->errSum += dt*(err + pid->prevErr)/2.f;
  pid->out = pid->kp*err + pid->ki*pid->errSum + pid->kd*(err - pid->prevErr)/dt;
  pid->prevErr = err;
}

void pdlResetPid(pdlPidState *pid)
{
  pid->errSum = 0;
  pid->prevErr = 0;
  pid->out = 0;
}

float pdlCalcHeading(pdlDroneState* ds)
{
  double fx = ds->magneto.pure[PDL_X];
  double fy = ds->magneto.pure[PDL_Y];
  double fz = ds->magneto.pure[PDL_Z];

  double cor = cos(ds->roll);
  double sir = sin(ds->roll);
  double cop = cos(ds->pitch);
  double sip = sin(ds->pitch);

  double fx1 = fx*cop + fy*sir*sip + fz*cor*sip;
  double fy1 = fy*cor - fz*sir;

  //int16_t result = (180.0*atan2(fy1,fx1)/M_PI);
  double result = M_PI - atan2(fy1,fx1);

  if(result >= 2.0*M_PI)
    result -= 2.0*M_PI;

  if(result < 0)
    result += 2.0*M_PI;

  return (float)result;
}

float pdlComplementaryFilter(float alpha, float rate, float value, float newValue, float dt)
{
  value = alpha*(value + rate*dt) + (1.f - alpha)*newValue;
  return value;
}

void pdlComplFilterTripleAxisFusion(pdlDroneState *ds, float alpha, float dt)
{

}
