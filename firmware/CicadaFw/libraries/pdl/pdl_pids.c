#include "pdl.h"
#include "pdl_pids.h"

#include <math.h>

void pdlSetPidFlag(pdlDroneState *ds, uint8_t flag, uint8_t en)
{
  if(en)
  {
    ds->pidFlags |= flag;
  }
  else if(ds->pidFlags & flag)
  {
    ds->pidFlags ^= flag;
  }
}

uint8_t pdlGetPidVeloXFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode == PDL_TRICK_MODE_DISABLED)?1:0;
  uint8_t veloXFlag = (ds->pidFlags & PDL_PID_VELOX_FLAG)?1:0;
  return (ds->velocityXPid.enabled && veloXFlag && trickFlag)?1:0;
}

void pdlSetPidVeloXFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_VELOX_FLAG, en);
}

uint8_t pdlGetPidVeloYFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode == PDL_TRICK_MODE_DISABLED)?1:0;
  uint8_t veloYFlag = (ds->pidFlags & PDL_PID_VELOY_FLAG)?1:0;
  return (ds->velocityYPid.enabled && veloYFlag && trickFlag)?1:0;
}

void pdlSetPidVeloYFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_VELOY_FLAG, en);
}

uint8_t pdlGetPidAltFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode == PDL_TRICK_MODE_DISABLED)?1:0;
  uint8_t altFlag = (ds->pidFlags & PDL_PID_ALT_FLAG)?1:0;
  return (ds->altPid.enabled && altFlag && trickFlag)?1:0;
}

void pdlSetPidAltFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_ALT_FLAG, en);
}

uint8_t pdlGetPidHeadingFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode <= PDL_TRICK_MODE_ACRO)?1:0;
  uint8_t headingFlag = (ds->pidFlags & PDL_PID_HEADING_FLAG)?1:0;
  return (ds->headingPid.enabled && headingFlag && trickFlag)?1:0;
}

void pdlSetPidHeadingFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_HEADING_FLAG, en);
}

uint8_t pdlGetPidPosNorthFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode == PDL_TRICK_MODE_DISABLED)?1:0;
  uint8_t posNorthFlag = (ds->pidFlags & PDL_PID_POS_NORTH_FLAG)?1:0;
  return (ds->posNorthPid.enabled && posNorthFlag && trickFlag)?1:0;
}

void pdlSetPidPosNorthFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_POS_NORTH_FLAG, en);
}

uint8_t pdlGetPidPosEastFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode == PDL_TRICK_MODE_DISABLED)?1:0;
  uint8_t posEastFlag = (ds->pidFlags & PDL_PID_POS_EAST_FLAG)?1:0;
  return (ds->posEastPid.enabled && posEastFlag && trickFlag)?1:0;
}

void pdlSetPidPosEastFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_POS_EAST_FLAG, en);
}

uint8_t pdlGetPidPitchFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode <= PDL_TRICK_MODE_ACRO)?1:0;
  uint8_t pitchFlag = (ds->pidFlags & PDL_PID_PITCH_FLAG)?1:0;
  return (ds->pitchPid.enabled && pitchFlag && trickFlag)?1:0;
}

void pdlSetPidPitchFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_PITCH_FLAG, en);
}

uint8_t pdlGetPidRollFlag(pdlDroneState *ds)
{
  uint8_t trickFlag = (ds->trickMode <= PDL_TRICK_MODE_ACRO)?1:0;
  uint8_t rollFlag = (ds->pidFlags & PDL_PID_ROLL_FLAG)?1:0;
  return (ds->rollPid.enabled && rollFlag && trickFlag)?1:0;
}

void pdlSetPidRollFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_ROLL_FLAG, en);
}

uint8_t pdlUpdatePid(pdlDroneState *ds, pdlPidState *pid, float input, float dt, uint8_t angVal)
{
  if(!pid->enabled || !ds->stabilizationEnabled)
  {
    pdlResetPid(pid);
    // pid->input should to be updated all times to prevent big Dterm when it gets enabled
    pid->input = input;
    return 0;
  }

  float err = pid->target - input;
  float prevErr = pid->target - pid->input;

  if(angVal)
  {
    if(err < -M_PI)
      err += M_TWOPI;
    else if(err > M_PI)
      err -= M_TWOPI;

    if(prevErr < -M_PI)
      prevErr += M_TWOPI;
    else if(prevErr > M_PI)
      prevErr -= M_TWOPI;
  }

  /*
  if(angVal == 1)
  {
    if(err < -M_PI)
      err += M_PI;
    else if(err > M_PI)
      err -= M_PI;

    if(prevErr < -M_PI)
      prevErr += M_PI;
    else if(prevErr > M_PI)
      prevErr -= M_PI;
  }
  else if(angVal == 2)
  {
    if(err < -M_PI)
      err += M_TWOPI;
    else if(err > M_PI)
      err -= M_TWOPI;

    if(prevErr < -M_PI)
      prevErr += M_TWOPI;
    else if(prevErr > M_PI)
      prevErr -= M_TWOPI;
  }
  */

  //output = Un1 + (kp + kd/dt)*En + (-kp - 2.f*kd/dt)*En1 + kd/dt*En2;
  //output = Un1 + kp*(En - En1) + ki_d*En + kd_d*(En - 2.f*En1 + En2);
  pid->errSum += dt*(err + prevErr)/2.f;
  if(fabsf(pid->errSum) * pid->ki > pid->maxOut)
  {
    if(pid->errSum > 0)
      pid->errSum = pid->maxOut / pid->ki;
    else
      pid->errSum = -pid->maxOut / pid->ki;
  }

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

  return 1;
}

void pdlResetPid(pdlPidState *pid)
{
  pid->errSum = 0;
  pid->out = 0;
}

void pdlResetPid2(pdlPidState *pid, float input)
{
  pdlResetPid(pid);
  pid->target = input;
  pid->input = input;
}

void pdlUpdateAngularRatePids(pdlDroneState *ds, float dt)
{
  /* yaw/pitch/roll rates is not gyro rates
  pdlUpdatePid(ds,&ds->yawRatePid,ds->gyro.pure[PDL_Z],dt,0);
  pdlUpdatePid(ds,&ds->pitchRatePid,ds->gyro.pure[PDL_Y],dt,0);
  pdlUpdatePid(ds,&ds->rollRatePid,ds->gyro.pure[PDL_X],dt,0);
  */

  pdlVector3 target;

  // if we need to control the drone by body angular rates
  // we have to reset targets of euler angular rates and disable up-level PIDs
  // see the code of pdlSetZRateTarget

  target.x = ds->xRatePid.target;
  target.y = ds->yRatePid.target;
  target.z = ds->zRatePid.target;

  if( ds->yawRateTarget != 0   ||     // if we have the control request by body angular rates all these conditions is reseted in pdlSetZRateTarget
      ds->pitchRateTarget != 0 ||
      ds->rollRateTarget != 0  ||
      pdlGetPidHeadingFlag(ds) ||
      pdlGetPidPitchFlag(ds)   ||
      pdlGetPidRollFlag(ds) )
  {
    // if we have the control request by euler angles or euler angular rates
    // we need to convert euler angular rates targets to body angular rates
    target = pdlEulerRateToBodyRate( ds->pose[PDL_YAW].pos,
                                     ds->pose[PDL_PITCH].pos,
                                     ds->pose[PDL_ROLL].pos,
                                     ds->yawRateTarget,
                                     ds->pitchRateTarget,
                                     ds->rollRateTarget );
  }

  ds->zRatePid.target = target.z;
  ds->yRatePid.target = target.y;
  ds->xRatePid.target = target.x;

  // there is noise in gyro data
  // but it can be reduced by lower cut-off frequency of gyro DLPF
  pdlUpdatePid(ds,&ds->zRatePid,ds->gyro.pure[PDL_Z],dt,0);
  pdlUpdatePid(ds,&ds->yRatePid,ds->gyro.pure[PDL_Y],dt,0);
  pdlUpdatePid(ds,&ds->xRatePid,ds->gyro.pure[PDL_X],dt,0);

  /*
  // there is time delay about 50ms between rate from Kalman Filter and rate from gyro
  // it leads to smaller PID koef
  pdlVector3 input = pdlEulerRateToBodyRate(  ds->pose[PDL_YAW].pos,
                                              ds->pose[PDL_PITCH].pos,
                                              ds->pose[PDL_ROLL].pos,
                                              ds->pose[PDL_YAW].vel,
                                              ds->pose[PDL_PITCH].vel,
                                              ds->pose[PDL_ROLL].vel );
  pdlUpdatePid(ds,&ds->zRatePid,input.z,dt,0);
  pdlUpdatePid(ds,&ds->yRatePid,input.y,dt,0);
  pdlUpdatePid(ds,&ds->xRatePid,input.x,dt,0);
  */
}

void pdlUpdateLevelsPid(pdlDroneState *ds, float dt)
{
  float heading = ds->pose[PDL_YAW].pos;
  float pitch = ds->pose[PDL_PITCH].pos;
  float roll = ds->pose[PDL_ROLL].pos;
  // apply level pids to angular rate pids
  if( pdlGetPidHeadingFlag(ds) &&
      pdlUpdatePid(ds,&ds->headingPid,heading,dt,2) )
  {
    ds->yawRateTarget = ds->headingPid.out;
  }
  else
  {
    pdlResetPid2(&ds->headingPid,heading);
  }

  if( pdlGetPidPitchFlag(ds) &&
      pdlUpdatePid(ds,&ds->pitchPid,pitch,dt,1) )
  {
    ds->pitchRateTarget = ds->pitchPid.out;
  }
  else
  {
    pdlResetPid2(&ds->pitchPid,pitch);
  }

  if( pdlGetPidRollFlag(ds) &&
      pdlUpdatePid(ds,&ds->rollPid,roll,dt,1) )
  {
    ds->rollRateTarget = ds->rollPid.out;
  }
  else
  {
    pdlResetPid2(&ds->rollPid,roll);
  }
}

void pdlUpdateAltPid(pdlDroneState *ds, float dt)
{
  //static float old_range = 0.f;
  float velZ = ds->nav[PDL_UP].vel;
  float altitude = ds->nav[PDL_UP].pos;

  if(ds->trickMode != PDL_TRICK_MODE_DISABLED)
  {
    pdlResetPid2(&ds->altPid,altitude);

    ds->velocityZPid.target = 0;
    // don't reset velocity-z because we lose errSum if we reset it
    return;
  }

  // restore altPid flag when drone stops and there is no remote control signal
  // this code prevents drone go back when we release thrust stick
  if( ds->stabilizationEnabled &&
      !pdlGetPidAltFlag(ds)    &&
      ds->velocityZPid.target == 0.f)
  {
    if(fabs(velZ) < 0.1f) // wait for drone stops lifting
    {
      ds->altPid.target = altitude;
      pdlSetPidAltFlag(ds,1);

      /*
      // switch alt target when lidar gets right
      if(ds->lidar.range > 0.f && ds->lidar.range < PDL_FOLLOW_TERRAIN_RANGE)
      {
        ds->altPid.target = ds->lidar.range;
      }*/
    }
  }

  // It works no good and it doesn't need at this time
  /*
  if(pdlGetPidAltFlag(ds))
  {
    // follow terrain function
    if(ds->lidar.range > 0.f && ds->lidar.range < PDL_FOLLOW_TERRAIN_RANGE)
    {
      altitude = ds->lidar.range;
    }
    // correct alt target when  lidar gets right
    if( (ds->lidar.range > 0.f && ds->lidar.range < PDL_FOLLOW_TERRAIN_RANGE) &&
        (old_range < 0.f || old_range > PDL_FOLLOW_TERRAIN_RANGE) )
    {
      // switch alt target from kf to lidar
      ds->altPid.target = ds->lidar.range;
    }
    // correct alt target when lidar gets wrong
    if( (old_range > 0.f && old_range < PDL_FOLLOW_TERRAIN_RANGE) &&
        (ds->lidar.range < 0.f || ds->lidar.range > PDL_FOLLOW_TERRAIN_RANGE) )
    {
      // switch alt target from lidar to kf
      ds->altPid.target = ds->nav[PDL_Z].pos;
    }
  }

  old_range = ds->lidar.range;
  */

  if( pdlGetPidAltFlag(ds) &&
      pdlUpdatePid(ds,&ds->altPid,altitude,dt,0) )
  {
    ds->velocityZPid.target = ds->altPid.out;
  }
  else
  {
    pdlResetPid2(&ds->altPid,altitude);
  }

  if(pdlUpdatePid(ds,&ds->velocityZPid,velZ,dt,0))
  {
    pdlSetBaseGas(ds,ds->velocityZPid.out);
  }
}

void pdlUpdateHorVelocityPids(pdlDroneState *ds, float dt)
{
  float cos_theta = cosf(ds->pose[PDL_YAW].pos);
  float sin_theta = sinf(ds->pose[PDL_YAW].pos);
  float posNorth = ds->nav[PDL_NORTH].pos;
  float posEast = ds->nav[PDL_EAST].pos;
  // Rotate velocities by yaw to body frame
  float velX = ds->nav[PDL_NORTH].vel * cos_theta + ds->nav[PDL_EAST].vel * sin_theta;
  float velY = -ds->nav[PDL_NORTH].vel * sin_theta + ds->nav[PDL_EAST].vel * cos_theta;

  // restore posPid flags when drone stops and there is no remote control signal
  // this code prevents drone go back when we release pitch/roll stick in hold position mode

  if( ds->stabilizationEnabled          &&
      (!pdlGetPidPosNorthFlag(ds) || !pdlGetPidPosEastFlag(ds)) &&
      pdlGetPidVeloXFlag(ds)          &&
      pdlGetPidVeloYFlag(ds)          &&
      ds->velocityXPid.target == 0.f  &&
      ds->velocityYPid.target == 0.f)
  {
    if(fabs(velX) < 0.1f && fabs(velY) < 0.1f) // wait for drone stops
    {
      ds->posNorthPid.target = posNorth;
      ds->posEastPid.target = posEast;
      pdlSetPidPosNorthFlag(ds,1);
      pdlSetPidPosEastFlag(ds,1);
    }
  }

  if( pdlGetPidPosNorthFlag(ds) &&
      pdlUpdatePid(ds,&ds->posNorthPid,posNorth,dt,0) )
  {
    // Rotate output velocity to body frame
    ds->velocityXPid.target = ds->posNorthPid.out * cos_theta + ds->posEastPid.out * sin_theta;
  }
  else
  {
    pdlResetPid2(&ds->posNorthPid,posNorth);
  }

  if( pdlGetPidVeloXFlag(ds) &&
      pdlGetPidVeloYFlag(ds) &&
      pdlUpdatePid(ds,&ds->velocityXPid,velX,dt,0) )
  {
    ds->pitchPid.target = -ds->velocityXPid.out;
  }
  else
  {
    pdlResetPid2(&ds->velocityXPid,0);
  }

  if( pdlGetPidPosEastFlag(ds) &&
      pdlUpdatePid(ds,&ds->posEastPid,posEast,dt,0) )
  {
    // Rotate output velocity to body frame
    ds->velocityYPid.target = -ds->posNorthPid.out * sin_theta + ds->posEastPid.out * cos_theta;
  }
  else
  {
    pdlResetPid2(&ds->posEastPid,posEast);
  }

  if( pdlGetPidVeloXFlag(ds) &&
      pdlGetPidVeloYFlag(ds) &&
      pdlUpdatePid(ds,&ds->velocityYPid,velY,dt,0) )
  {
    ds->rollPid.target = ds->velocityYPid.out;
  }
  else
  {
    pdlResetPid2(&ds->velocityYPid,0);
  }
}

void pdlPidsToCrossFrame(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  dg[0] += -ds->zRatePid.out + ds->yRatePid.out;
  dg[1] += ds->zRatePid.out + ds->xRatePid.out;
  dg[2] += -ds->zRatePid.out - ds->yRatePid.out;
  dg[3] += ds->zRatePid.out - ds->xRatePid.out;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg[i]);
}

void pdlPidsToCrossFrameReversed(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  dg[0] += ds->zRatePid.out + ds->yRatePid.out;
  dg[1] += -ds->zRatePid.out + ds->xRatePid.out;
  dg[2] += ds->zRatePid.out - ds->yRatePid.out;
  dg[3] += -ds->zRatePid.out - ds->xRatePid.out;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg[i]);
}

void pdlPidsToXFrame(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  dg[0] += -ds->zRatePid.out + ds->yRatePid.out + ds->xRatePid.out;
  dg[1] += ds->zRatePid.out - ds->yRatePid.out + ds->xRatePid.out;
  dg[2] += -ds->zRatePid.out - ds->yRatePid.out - ds->xRatePid.out;
  dg[3] += ds->zRatePid.out + ds->yRatePid.out - ds->xRatePid.out;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg[i]);
}

void pdlPidsToXFrameReversed(pdlDroneState *ds)
{
  float dg[4];

  dg[0] = ds->baseGas;
  dg[1] = ds->baseGas;
  dg[2] = ds->baseGas;
  dg[3] = ds->baseGas;

  // reverse motors scheme
  dg[0] += ds->zRatePid.out + ds->yRatePid.out + ds->xRatePid.out;
  dg[1] += -ds->zRatePid.out - ds->yRatePid.out + ds->xRatePid.out;
  dg[2] += ds->zRatePid.out - ds->yRatePid.out - ds->xRatePid.out;
  dg[3] += -ds->zRatePid.out + ds->yRatePid.out - ds->xRatePid.out;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
    pdlSetMotorGas(ds, i, dg[i]);
}
