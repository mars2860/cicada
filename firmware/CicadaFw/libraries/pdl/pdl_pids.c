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
  return (ds->velocityXPid.enabled && (ds->pidFlags & PDL_PID_VELOX_FLAG))?1:0;
}

void pdlSetPidVeloXFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_VELOX_FLAG, en);
}

uint8_t pdlGetPidVeloYFlag(pdlDroneState *ds)
{
  return (ds->velocityYPid.enabled && (ds->pidFlags & PDL_PID_VELOY_FLAG))?1:0;
}

void pdlSetPidVeloYFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_VELOY_FLAG, en);
}

uint8_t pdlGetPidAltFlag(pdlDroneState *ds)
{
  return (ds->altPid.enabled && (ds->pidFlags & PDL_PID_ALT_FLAG))?1:0;
}

void pdlSetPidAltFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_ALT_FLAG, en);
}

uint8_t pdlGetPidHeadingFlag(pdlDroneState *ds)
{
  return (ds->headingPid.enabled && (ds->pidFlags & PDL_PID_HEADING_FLAG))?1:0;
}

void pdlSetPidHeadingFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_HEADING_FLAG, en);
}

uint8_t pdlGetPidPosNorthFlag(pdlDroneState *ds)
{
  return (ds->posNorthPid.enabled && (ds->pidFlags & PDL_PID_POS_NORTH_FLAG))?1:0;
}

void pdlSetPidPosNorthFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_POS_NORTH_FLAG, en);
}

uint8_t pdlGetPidPosEastFlag(pdlDroneState *ds)
{
  return (ds->posEastPid.enabled && (ds->pidFlags & PDL_PID_POS_EAST_FLAG))?1:0;
}

void pdlSetPidPosEastFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_POS_EAST_FLAG, en);
}

uint8_t pdlGetPidPitchFlag(pdlDroneState *ds)
{
  return (ds->pitchPid.enabled && (ds->pidFlags & PDL_PID_PITCH_FLAG))?1:0;
}

void pdlSetPidPitchFlag(pdlDroneState *ds, uint8_t en)
{
  pdlSetPidFlag(ds, PDL_PID_PITCH_FLAG, en);
}

uint8_t pdlGetPidRollFlag(pdlDroneState *ds)
{
  return (ds->rollPid.enabled && (ds->pidFlags & PDL_PID_ROLL_FLAG))?1:0;
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

void pdlUpdateAngularRatePids(pdlDroneState *ds, float dt)
{
  /* yaw/pitch/roll rates != gyro rates
  pdlUpdatePid(ds,&ds->yawRatePid,ds->gyro.pure[PDL_Z],dt,0);
  pdlUpdatePid(ds,&ds->pitchRatePid,ds->gyro.pure[PDL_Y],dt,0);
  pdlUpdatePid(ds,&ds->rollRatePid,ds->gyro.pure[PDL_X],dt,0);
  */

  // convert euler rates to body rates

  pdlVector3 target = pdlEulerRateToBodyRate( ds->pose[PDL_YAW].pos,
                                              ds->pose[PDL_PITCH].pos,
                                              ds->pose[PDL_ROLL].pos,
                                              ds->yawRateTarget,
                                              ds->pitchRateTarget,
                                              ds->rollRateTarget );

  // don't use converted euler rates if trickMode is enabled or we control drone by body rates
  if(ds->trickModeEnabled || !pdlGetPidHeadingFlag(ds))
  {
    ds->zRatePid.target = ds->yawRateTarget;
  }
  else
  {
    ds->zRatePid.target = target.z;
  }

  if(ds->trickModeEnabled || !pdlGetPidPitchFlag(ds))
  {
    ds->yRatePid.target = ds->pitchRateTarget;
  }
  else
  {
    ds->yRatePid.target = target.y;
  }

  if(ds->trickModeEnabled || !pdlGetPidRollFlag(ds))
  {
    ds->xRatePid.target = ds->rollRateTarget;
  }
  else
  {
    ds->xRatePid.target = target.x;
  }

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
  if(ds->trickModeEnabled)
  {
    pdlResetPid(&ds->headingPid);
    ds->headingPid.target = ds->pose[PDL_YAW].pos;

    pdlResetPid(&ds->pitchPid);
    ds->pitchPid.target = 0;

    pdlResetPid(&ds->rollPid);
    ds->rollPid.target = 0;
    return;
  }

  if(pdlUpdatePid(ds,&ds->headingPid,ds->pose[PDL_YAW].pos,dt,2))
  {
    // apply level pids to angular rate pids
    if(pdlGetPidHeadingFlag(ds))
    {
      //ds->yawRatePid.target = ds->headingPid.out;
      ds->yawRateTarget = ds->headingPid.out;
    }
    else
    {
      pdlResetPid(&ds->headingPid);
      ds->headingPid.target = ds->pose[PDL_YAW].pos;
    }
  }

  if(pdlUpdatePid(ds,&ds->pitchPid,ds->pose[PDL_PITCH].pos,dt,1))
  {
    if(pdlGetPidPitchFlag(ds))
    {
      //ds->pitchRatePid.target = ds->pitchPid.out;
      ds->pitchRateTarget = ds->pitchPid.out;
    }
    else
    {
      pdlResetPid(&ds->pitchPid);
      ds->pitchPid.target = 0;
    }
  }

  if(pdlUpdatePid(ds,&ds->rollPid,ds->pose[PDL_ROLL].pos,dt,1))
  {
    if(pdlGetPidRollFlag(ds))
    {
      //ds->rollRatePid.target = ds->rollPid.out;
      ds->rollRateTarget = ds->rollPid.out;
    }
    else
    {
      pdlResetPid(&ds->rollPid);
      ds->rollPid.target = 0;
    }
  }
}

void pdlUpdateAltPid(pdlDroneState *ds, float dt)
{
  //static float old_range = 0.f;
  float velZ = ds->nav[PDL_Z].vel;
  float altitude = ds->nav[PDL_Z].pos;

  if(ds->trickModeEnabled)
  {
    pdlResetPid(&ds->altPid);
    ds->altPid.target = altitude;

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

  if(pdlUpdatePid(ds,&ds->altPid,altitude,dt,0))
  {
    // apply alt pid
    if(pdlGetPidAltFlag(ds))
    {
      ds->velocityZPid.target = ds->altPid.out;
    }
    else
    {
      pdlResetPid(&ds->altPid);
      ds->altPid.target = altitude;
    }
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

  if(ds->trickModeEnabled)
  {
    pdlResetPid(&ds->posNorthPid);
    ds->posNorthPid.target = posNorth;

    pdlResetPid(&ds->posEastPid);
    ds->posEastPid.target = posEast;

    pdlResetPid(&ds->velocityYPid);
    ds->velocityYPid.target = 0;

    pdlResetPid(&ds->velocityXPid);
    ds->velocityXPid.target = 0;
    return;
  }

  // restore posPid flags when drone stops and there is no remote control signal
  // this code prevents drone go back when we release pitch/roll stick in hold position mode
  /*if( ds->stabilizationEnabled &&
      !pdlGetPidPosXFlag(ds)   &&
      pdlGetPidVeloXFlag(ds)   &&
      ds->velocityXPid.target == 0.f)
  {
    if(fabs(ds->velocity[PDL_X]) < 0.01f)
    {
      ds->posXPid.target = ds->posX;
      pdlSetPidPosXFlag(ds,1);
    }
  }

  if( ds->stabilizationEnabled &&
      !pdlGetPidPosYFlag(ds)   &&
      pdlGetPidVeloYFlag(ds)   &&
      ds->velocityYPid.target == 0.f)
  {
    if(fabs(ds->velocity[PDL_Y]) < 0.01f)
    {
      ds->posYPid.target = ds->posY;
      pdlSetPidPosYFlag(ds,1);
    }
  }*/

  if( ds->stabilizationEnabled          &&
      (!pdlGetPidPosNorthFlag(ds) || !pdlGetPidPosEastFlag(ds)) &&
      pdlGetPidVeloXFlag(ds)          &&
      pdlGetPidVeloYFlag(ds)          &&
      ds->velocityXPid.target == 0.f  &&
      ds->velocityYPid.target == 0.f)
  {
    ds->posNorthPid.target = posNorth;
    ds->posEastPid.target = posEast;
    pdlSetPidPosNorthFlag(ds,1);
    pdlSetPidPosEastFlag(ds,1);
  }

  if(pdlUpdatePid(ds,&ds->posNorthPid,posNorth,dt,0))
  {
    if(pdlGetPidPosNorthFlag(ds))
    {
      // Rotate output velocity to body frame
      ds->velocityXPid.target = ds->posNorthPid.out * cos_theta + ds->posEastPid.out * sin_theta;
      //ds->velocityXPid.target = ds->posXPid.out;
    }
    else
    {
      pdlResetPid(&ds->posNorthPid);
      //ds->posXPid.target = ds->posX;
    }
  }

  if(pdlUpdatePid(ds,&ds->velocityXPid,velX,dt,0))
  {
    //if(pdlGetPidVeloXFlag(ds))
    if(pdlGetPidVeloXFlag(ds) && pdlGetPidVeloYFlag(ds))  // don't apply veloPid if we have control from user by pitch/roll
    {
      ds->pitchPid.target = -ds->velocityXPid.out;
    }
    else
    {
      pdlResetPid(&ds->velocityXPid);
      ds->velocityXPid.target = 0;
    }
  }

  if(pdlUpdatePid(ds,&ds->posEastPid,posEast,dt,0))
  {
    if(pdlGetPidPosEastFlag(ds))
    {
      // Rotate output velocity to body frame
      ds->velocityYPid.target = -ds->posNorthPid.out * sin_theta + ds->posEastPid.out * cos_theta;
      //ds->velocityYPid.target = ds->posYPid.out;
    }
    else
    {
      pdlResetPid(&ds->posEastPid);
      //ds->posYPid.target = ds->posY;
    }
  }

  if(pdlUpdatePid(ds,&ds->velocityYPid,velY,dt,0))
  {
    //if(pdlGetPidVeloYFlag(ds))
    if(pdlGetPidVeloXFlag(ds) && pdlGetPidVeloYFlag(ds))  // don't apply veloPid if we have control from user by pitch/roll
    {
      ds->rollPid.target = ds->velocityYPid.out;
    }
    else
    {
      pdlResetPid(&ds->velocityYPid);
      ds->velocityYPid.target = 0;
    }
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
