#include "pdl.h"
#include "pdl_tasks.h"
#include "pdl_pids.h"

#include <string.h>
#include <math.h>

pdlNavKF navKf[3];
pdlNavKF poseKf[3];

pdlTaskState pdlGyroTS;
pdlTaskState pdlAccelTS;
pdlTaskState pdlMagTS;
pdlTaskState pdlRcTS;
pdlTaskState pdlBatteryTS;
pdlTaskState pdlOpticalFlowTS;
pdlTaskState pdlLidarTS;
pdlTaskState pdlBaroTS;
pdlTaskState pdlEscTS;
pdlTaskState pdlTelemetryTS;
pdlTaskState pdlPidTS;
pdlTaskState pdlGpsTS;
pdlTaskState pdlLoadTS;

void pdlSetupTasks()
{
  memset(&pdlGyroTS,0,sizeof(pdlTaskState));
  memset(&pdlAccelTS,0,sizeof(pdlTaskState));
  memset(&pdlMagTS,0,sizeof(pdlTaskState));
  memset(&pdlRcTS,0,sizeof(pdlTaskState));
  memset(&pdlBatteryTS,0,sizeof(pdlTaskState));
  memset(&pdlLidarTS,0,sizeof(pdlTaskState));
  memset(&pdlOpticalFlowTS,0,sizeof(pdlTaskState));
  memset(&pdlBaroTS,0,sizeof(pdlTaskState));
  memset(&pdlEscTS,0,sizeof(pdlTaskState));
  memset(&pdlTelemetryTS,0,sizeof(pdlTaskState));
  memset(&pdlPidTS,0,sizeof(pdlTaskState));
  memset(&pdlGpsTS,0,sizeof(pdlTaskState));
  memset(&pdlLoadTS,0,sizeof(pdlTaskState));

  pdlGyroTS.period = PDL_GYRO_READ_PERIOD;
  pdlAccelTS.period = PDL_ACCEL_READ_PERIOD;
  pdlMagTS.period = PDL_MAG_READ_PERIOD;
  pdlRcTS.period = PDL_RC_UPDATE_PERIOD;
  pdlBatteryTS.period = PDL_BATTERY_READ_PERIOD;
  pdlOpticalFlowTS.period = PDL_OPTICAL_FLOW_READ_PERIOD;
  pdlLidarTS.period = PDL_LIDAR_READ_PERIOD;
  pdlBaroTS.period = PDL_BARO_READ_PERIOD;
  pdlEscTS.period = PDL_ESCAPER_UPDATE_PERIOD;
  pdlTelemetryTS.period = PDL_DEFAULT_TELEMETRY_UPDATE_PERIOD;
  pdlPidTS.period = PDL_PID_UPDATE_PERIOD;
  pdlGpsTS.period = PDL_GPS_READ_PERIOD;
  pdlLoadTS.period = PDL_LOAD_UPDATE_PERIOD;
}

void pdlResetKalman(pdlDroneState *ds)
{
  for(uint8_t i = 0; i < 3; i++)
  {
    pdlNavKF_Init(&navKf[i],&ds->nav[i],ds->kfSettings.navModelNoise);
    pdlNavKF_Init(&poseKf[i],&ds->pose[i],ds->kfSettings.poseModelNoise);
  }
}

void pdlSetTelemetryUpdatePeriod(pdlDroneState *ds, uint32_t t)
{
  ds->telemetryPeriod = t;
  pdlTelemetryTS.period = t;
}

uint32_t pdlGetTelemetryUpdatePeriod(void)
{
  return pdlTelemetryTS.period;
}

uint8_t pdlCanTaskRun(pdlTaskState *ts, uint8_t resetFrame)
{
  static uint32_t frameStartTime = 0;
  uint32_t dt, frameDt;

  dt = pdlGetDeltaTime(pdlMicros(), ts->lastUpdateTime);
  if(dt < ts->period)
    return 0;

  if(resetFrame)
    frameStartTime = pdlMicros();

  dt -= ts->period;
  frameDt = pdlGetDeltaTime(pdlMicros(), frameStartTime);
  frameDt += ts->execTime;

  if(frameDt >= PDL_DESIRE_UPDATE_TIME && dt < PDL_TASK_MAX_WAIT_TIME)
    return 0;

  ts->realPeriod = pdlGetDeltaTime(pdlMicros(), ts->lastUpdateTime);
  ts->lastUpdateTime = pdlMicros();
  return 1;
}

void pdlTaskBegin(pdlTaskState *ts)
{
  ts->startTime = pdlMicros();
}

void pdlTaskEnd(pdlTaskState *ts)
{
  ts->execTime = pdlGetDeltaTime(pdlMicros(), ts->startTime);
}

void pdlPidTask(pdlDroneState *ds)
{
  float dt;

  if(!pdlCanTaskRun(&pdlPidTS,1))
      return;

  pdlTaskBegin(&pdlPidTS);

  dt = pdlPidTS.realPeriod / 1000000.f; // microseconds to seconds

  for(uint8_t i = 0; i < 3; i++)
  {
    //pdlNavKF_Predict_ConstVelo(&poseKf[i], &ds->pose[i], dt); // model with const acc is better for pose estimate
    pdlNavKF_Predict_ConstAcc(&poseKf[i], &ds->pose[i], dt);
    // commented because we update navKf in accelTask when we get new accel data
    //pdlNavKF_Predict_ConstAcc(&navKf[i], &ds->nav[i], dt); // model with control input is better for nav estimate
  }

  pdlUpdateHorVelocityPids(ds,dt);
  pdlUpdateLevelsPid(ds,dt);
  pdlUpdateAltPid(ds,dt);
  pdlUpdateAngularRatePids(ds,dt);

  pdlTaskEnd(&pdlPidTS);

  ds->pidTaskTime = pdlPidTS.execTime;
  ds->loopPeriod = pdlPidTS.realPeriod;
}

void pdlGyroTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlGyroTS,0))
    return;

  pdlTaskBegin(&pdlGyroTS);

  pdlReadGyro(ds);

  pdlTaskEnd(&pdlGyroTS);

  ds->gyroTaskTime = pdlGyroTS.execTime;
}

void pdlMagTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlMagTS,0))
    return;

  pdlTaskBegin(&pdlMagTS);

  pdlReadMagneto(ds);

  pdlTaskEnd(&pdlMagTS);

  ds->magTaskTime = pdlMagTS.execTime;
}

void pdlAccelTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlAccelTS,0))
    return;

  pdlTaskBegin(&pdlAccelTS);

  pdlReadAccel(ds);

  pdlTaskEnd(&pdlAccelTS);

  ds->accelTaskTime = pdlAccelTS.execTime;
}

void pdlOpticalFlowTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlOpticalFlowTS,0))
    return;

  pdlTaskBegin(&pdlOpticalFlowTS);

  pdlReadOpticalFlow(ds);

  pdlTaskEnd(&pdlOpticalFlowTS);

  ds->ofTaskTime = pdlOpticalFlowTS.execTime;
}

void pdlLidarTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlLidarTS,0))
    return;

  pdlTaskBegin(&pdlLidarTS);

  pdlReadLidar(ds);

  pdlTaskEnd(&pdlLidarTS);

  ds->lidarTaskTime = pdlLidarTS.execTime;
}

void pdlBaroTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlBaroTS,0))
    return;

  pdlTaskBegin(&pdlBaroTS);

  pdlReadBaro(ds);

  pdlTaskEnd(&pdlBaroTS);

  ds->baroTaskTime = pdlBaroTS.execTime;
}

void pdlEscTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlEscTS,0))
    return;

  pdlTaskBegin(&pdlEscTS);

  if(ds->stabilizationEnabled && ds->motorsEnabled)
  {
    pdlPidsToMotors(ds);
  }

  pdlUpdateEsc(ds);

  pdlTaskEnd(&pdlEscTS);

  ds->escTaskTime = pdlEscTS.execTime;
}

void pdlRcTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlRcTS,0))
    return;

  pdlTaskBegin(&pdlRcTS);

  pdlRemoteControl(ds);

  pdlTaskEnd(&pdlRcTS);

  ds->rcTaskTime = pdlRcTS.execTime;
}

void pdlTelemetryTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlTelemetryTS,0))
    return;

  pdlTaskBegin(&pdlTelemetryTS);

  pdlUpdateTelemetry(ds);

  pdlTaskEnd(&pdlTelemetryTS);

  ds->telemetryTaskTime = pdlTelemetryTS.execTime;
}

void pdlBatteryTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlBatteryTS,0))
    return;

  pdlTaskBegin(&pdlBatteryTS);

  pdlReadBattery(ds);

  pdlTaskEnd(&pdlBatteryTS);

  ds->batteryTaskTime = pdlBatteryTS.execTime;
}

void pdlGpsTask(pdlDroneState *ds)
{
  if(!pdlCanTaskRun(&pdlGpsTS,0))
    return;

  pdlTaskBegin(&pdlGpsTS);

  pdlReadGps(ds);

  pdlTaskEnd(&pdlGpsTS);

  ds->gpsTaskTime = pdlGpsTS.execTime;
}

void pdlLoadTask(pdlDroneState *ds)
{
  uint8_t i;
  uint32_t period;
  uint32_t dt;
  uint8_t reqState;

  if(!pdlCanTaskRun(&pdlLoadTS,0))
    return;

  pdlTaskBegin(&pdlLoadTS);

  for(i = 0; i < PDL_LOAD_COUNT; i++)
  {
    if(ds->load[i].enabled)
    {
      if(ds->load[i].period > 0)
      {
        period = ds->load[0].period;
        period *= 1000; // convert to us
        dt = pdlGetDeltaTime(pdlMicros(),ds->load[i].timestamp);
        if(dt >= period)
        {
          ds->load[i].timestamp = pdlMicros();
          reqState = !ds->load[i].state;
          if(pdlSwitchLoad(ds,i,reqState))
          {
            ds->load[i].state = reqState;
          }
        }
      }
      else if(!ds->load[0].state)
      {
        if(pdlSwitchLoad(ds,i,1))
        {
          ds->load[i].state = 1;
        }
      }
    }
    else if(ds->load[0].state)
    {
      if(pdlSwitchLoad(ds,i,0))
      {
        ds->load[i].state = 0;
      }
    }
  }

  pdlUpdateCamera(ds);

  pdlTaskEnd(&pdlLoadTS);

  ds->loadTaskTime = pdlLoadTS.execTime;
}

//-----------------------------------------------------------------------------

void pdlNewGyroData(pdlDroneState *ds, int16_t rawX, int16_t rawY, int16_t rawZ, float pureX, float pureY, float pureZ)
{
  pdlVector3 eulerRate;

  ds->gyro.raw[PDL_X] = rawX;
  ds->gyro.pure[PDL_X] = pureX;

  ds->gyro.raw[PDL_Y] = rawY;
  ds->gyro.pure[PDL_Y] = pureY;

  ds->gyro.raw[PDL_Z] = rawZ;
  ds->gyro.pure[PDL_Z] = pureZ;

  eulerRate = pdlBodyRateToEulerRate( ds->pose[PDL_YAW].pos,
                                      ds->pose[PDL_PITCH].pos,
                                      ds->pose[PDL_ROLL].pos,
                                      ds->gyro.pure[PDL_X],
                                      ds->gyro.pure[PDL_Y],
                                      ds->gyro.pure[PDL_Z]);

  ds->gyro.eulerRate[PDL_YAW] = eulerRate.z;
  ds->gyro.eulerRate[PDL_PITCH] = eulerRate.y;
  ds->gyro.eulerRate[PDL_ROLL] = eulerRate.x;

  pdlNavKF_CorrectByVelMeasurement( &poseKf[PDL_YAW],
                                    &ds->pose[PDL_YAW],
                                    eulerRate.z,
                                    ds->kfSettings.yawRateVariance);

  pdlNavKF_CorrectByVelMeasurement( &poseKf[PDL_PITCH],
                                    &ds->pose[PDL_PITCH],
                                    eulerRate.y,
                                    ds->kfSettings.pitchRollRateVariance);

  pdlNavKF_CorrectByVelMeasurement( &poseKf[PDL_ROLL],
                                    &ds->pose[PDL_ROLL],
                                    eulerRate.x,
                                    ds->kfSettings.pitchRollRateVariance);

  pdlConstrain(&ds->pose[PDL_YAW].pos,0.f,M_PI*2.f);
  pdlConstrain(&ds->pose[PDL_PITCH].pos,-M_PI,M_PI);
  pdlConstrain(&ds->pose[PDL_ROLL].pos,-M_PI,M_PI);
}

void pdlNewAccelData(pdlDroneState *ds, int16_t rawX, int16_t rawY, int16_t rawZ, float pureX, float pureY, float pureZ)
{
  #define LINEAR_ACC_COUNTS 100

  static uint8_t oldMotorsEnabled = 0;
  static uint32_t oldMicros = 0;

  static float linearAccOffsetX = 0.f;
  static float linearAccOffsetY = 0.f;
  static float linearAccOffsetZ = 0.f;

  /// to calc offsets for linear accel
  static uint8_t linearStabCounter = 0;

  pdlVector3 wAcc;
  pdlVector3 accEuler;
  float da, dt;

  ds->accel.raw[PDL_X] = rawX;
  ds->accel.pure[PDL_X] = pureX;

  ds->accel.raw[PDL_Y] = rawY;
  ds->accel.pure[PDL_Y] = pureY;

  ds->accel.raw[PDL_Z] = rawZ;
  ds->accel.pure[PDL_Z] = pureZ;

  dt = ((float)pdlGetDeltaTime(pdlMicros(),oldMicros)) / 1000000.f; // microseconds to seconds
  oldMicros = pdlMicros();

  /*
  // There is the problem. When angular rates gets zero the accel pitch/roll gets wrong caused by the linear accels
  // At this moment the estimated pitch/roll downgrades to the wrong accel pitch/roll
  // I tried to remove the linear accels from the accel measuruments but the code below has the problem
  float ax = ds->accel.pure[PDL_X];
  float ay = ds->accel.pure[PDL_Y];
  float az = ds->accel.pure[PDL_Z];

  // convert current linear accels to remove its from pitch/roll calculation
  wAcc = pdlWorldToBody(  ds->pose[PDL_YAW].pos,
                          ds->pose[PDL_PITCH].pos,
                          ds->pose[PDL_ROLL].pos,
                          ds->accel.world[PDL_X],
                          ds->accel.world[PDL_Y],
                          -ds->accel.world[PDL_Z] );

  // There is the problem. When the drone hits the wall the linear accs/pitch/roll gets wrong.
  // These errors is stored and when I try to remove its I get the wrong pitch/roll
  // Also there is the loop back between the world linear accels and the estimated pitch/roll angles

  if(fabsf(wAcc.x) > 1.f)
    ax = ax - wAcc.x;
  if(fabsf(wAcc.y) > 1.f)
    ay = ay - wAcc.y;
  if(fabsf(wAcc.z) > 1.f )
    az = az - wAcc.z;

  accEuler = pdlEulerFromAcc(ax,ay,az);
   */

  accEuler = pdlEulerFromAcc(  ds->accel.pure[PDL_X],
                               ds->accel.pure[PDL_Y],
                               ds->accel.pure[PDL_Z]);


  // before apply new pitch we have to exclude transition between 180deg to -180deg
  da = accEuler.y - ds->pose[PDL_PITCH].pos;
  if(da < -M_PI_2)
    accEuler.y += M_PI * 2.f;
  else if(da > M_PI_2)
    accEuler.y -= M_PI * 2.f;
  // before apply new roll we have to exclude transition between 180deg to -180deg
  da = accEuler.x - ds->pose[PDL_ROLL].pos;
  if(da < -M_PI_2)
    accEuler.x += M_PI * 2.f;
  else if(da > M_PI_2)
    accEuler.x -= M_PI * 2.f;

  ds->accel.pitch = accEuler.y;
  ds->accel.roll = accEuler.x;

  // when vibration is too high or drone is stricken by wall
  // may be a chance that difference between current euler angles
  // and angles from accel is bigger 180 deg
  // in this case accEuler is about 360 deg
  // thus constrain accEuler again
  pdlConstrain(&ds->accel.pitch,-M_PI,M_PI);
  pdlConstrain(&ds->accel.roll,-M_PI,M_PI);
  // RESEET current pose to accel pose if current pose has bigger difference from accel pose
  if(fabsf(ds->accel.pitch - ds->pose[PDL_PITCH].pos) > M_PI_2)
    ds->pose[PDL_PITCH].pos = ds->accel.pitch;
  if(fabsf(ds->accel.roll - ds->pose[PDL_ROLL].pos) > M_PI_2)
      ds->pose[PDL_ROLL].pos = ds->accel.roll;

  pdlNavKF_CorrectByPosMeasurement( &poseKf[PDL_PITCH],
                                    &ds->pose[PDL_PITCH],
                                    accEuler.y,
                                    ds->kfSettings.accPitchRollVariance);

  pdlNavKF_CorrectByPosMeasurement( &poseKf[PDL_ROLL],
                                    &ds->pose[PDL_ROLL],
                                    accEuler.x,
                                    ds->kfSettings.accPitchRollVariance);

  pdlConstrain(&ds->pose[PDL_PITCH].pos,-M_PI,M_PI);
  pdlConstrain(&ds->pose[PDL_ROLL].pos,-M_PI,M_PI);

  wAcc = pdlBodyToWorld( ds->pose[PDL_YAW].pos,
                         ds->pose[PDL_PITCH].pos,
                         ds->pose[PDL_ROLL].pos,
                         ds->accel.pure[PDL_X],
                         ds->accel.pure[PDL_Y],
                         ds->accel.pure[PDL_Z]);

  ds->accel.world[PDL_NORTH] = wAcc.x;
  ds->accel.world[PDL_EAST] = wAcc.y;
  ds->accel.world[PDL_UP] = -wAcc.z - PDL_G;

  // accel has a drift in data depends on temperature
  // linear accels calculated by system are not null after startup
  // thus we calc linear accel offsets each time motors get enabled
  if(linearStabCounter < LINEAR_ACC_COUNTS)
  {
    if( !isnan(ds->accel.world[PDL_X]) &&   // at startup we get NaN values from pose kalman filters
        !isnan(ds->accel.world[PDL_Y]) &&   // use this condition to prevent breaking nav kalman filters
        !isnan(ds->accel.world[PDL_Z]) )
    {
      linearAccOffsetX += ds->accel.world[PDL_X];
      linearAccOffsetY += ds->accel.world[PDL_Y];
      linearAccOffsetZ += ds->accel.world[PDL_Z];

      linearStabCounter++;

      if(linearStabCounter == LINEAR_ACC_COUNTS)
      {
        ds->nav[PDL_X].accBias = linearAccOffsetX / LINEAR_ACC_COUNTS;
        ds->nav[PDL_Y].accBias = linearAccOffsetY / LINEAR_ACC_COUNTS;
        ds->nav[PDL_Z].accBias = linearAccOffsetZ / LINEAR_ACC_COUNTS;
        // if drone gets down or up when velocityZPid is enabled and user control stick is not moved
        // this problem can be caused by bad accelerometer
        // some bad accelerometers give wrong values along Z axis in flight time
        // try to compensate this by adding additional offset
        ds->nav[PDL_Z].accBias += ds->accUpOffset;
      }
    }
  }
  else  // linearStabCounter
  {
    for(uint8_t i = 0; i < 3; i++)
    {
      // commented because it used with const acc model but this model is not good
      //pdlNavKF_CorrectByAccMeasurement(  &navKf[i],
      //                                   &ds->nav[i],
      //                                   ds->accel.world[i],
      //                                   ds->kfSettings.accVariance);
      // model with control input is better
      pdlNavKF_Predict_ControlInput(&navKf[i], &ds->nav[i], ds->accel.world[i], dt);
    }
  }

  // reset linear acc bias
  if(ds->motorsEnabled && !oldMotorsEnabled)
  {
    linearStabCounter = 0;
    linearAccOffsetX = 0.f;
    linearAccOffsetY = 0.f;
    linearAccOffsetZ = 0.f;
  }

  oldMotorsEnabled = ds->motorsEnabled;
}

void pdlNewMagnetoData(pdlDroneState *ds, int16_t rawX, int16_t rawY, int16_t rawZ, float pureX, float pureY, float pureZ)
{
  float heading;
  float da;

  ds->magneto.raw[PDL_X] = rawX;
  ds->magneto.pure[PDL_X] = pureX;

  ds->magneto.raw[PDL_Y] = rawY;
  ds->magneto.pure[PDL_Y] = pureY;

  ds->magneto.raw[PDL_Z] = rawZ;
  ds->magneto.pure[PDL_Z] = pureZ;

  heading = pdlHeadingFromMagneto(  ds->pose[PDL_PITCH].pos,
                                    ds->pose[PDL_ROLL].pos,
                                    ds->magneto.pure[PDL_X],
                                    ds->magneto.pure[PDL_Y],
                                    ds->magneto.pure[PDL_Z]);

  heading += ds->magneto.declination;

  if(abs(ds->baseGas) > abs(pdlGetMotorMaxGas() - pdlGetMotorNullGas()) / 8)
    heading += ds->magneto.inflightCorrection;

  // before apply new heading we have to exclude transition between 0deg to 360deg
  da = heading - ds->pose[PDL_YAW].pos;
  if(da < -M_PI_2)
    heading += M_TWOPI;
  else if(da > M_PI_2)
    heading -= M_TWOPI;

  ds->magneto.heading = heading;

  pdlNavKF_CorrectByPosMeasurement( &poseKf[PDL_YAW],
                                    &ds->pose[PDL_YAW],
                                    heading,
                                    ds->kfSettings.magHeadingVariance);

  pdlConstrain(&ds->pose[PDL_YAW].pos,0.f,M_TWOPI);
}

void pdlNewOpticalFlowData(pdlDroneState *ds, int16_t rawX, int16_t rawY, uint8_t squal, float fov, float npx, float scaler)
{
  static uint32_t old_micros = 0;
  //static float velX[3] = {0,0,0};
  //static float velY[3] = {0,0,0};
  //static uint8_t medianPos = 0;

  const float a1 = 0.3f;  // range to ground for ofVelVariance1
  const float a2 = 0.9f;  // range to ground for ofVelVariance2

  float dt,x,y,delta_pitch,delta_roll;
  float cos_theta;
  float sin_theta;
  float R;
  float alt;
  float k;
  float b;

  ds->opticalFlow.rawX = rawX;
  ds->opticalFlow.rawY = rawY;
  ds->opticalFlow.squal = squal;

  x = ds->opticalFlow.rawX;
  y = ds->opticalFlow.rawY;

  dt = pdlGetDeltaTime(pdlMicros(),old_micros);
  old_micros = pdlMicros();
  dt /= 1000000.f;

  delta_pitch = ds->gyro.pure[PDL_Y]*dt;
  delta_roll = -ds->gyro.pure[PDL_X]*dt;

  ds->opticalFlow.pureX = x - pdlOpticalFlowTiltPixels(delta_pitch,fov,npx,scaler);
  ds->opticalFlow.pureY = y - pdlOpticalFlowTiltPixels(delta_roll,fov,npx,scaler);

  if(ds->lidar.range <= 0.f)
  {
    ds->opticalFlow.velX = 0;
    ds->opticalFlow.velY = 0;
    ds->opticalFlow.velNorth = 0;
    ds->opticalFlow.velEast = 0;

    return; // we can't transform pixels to meters without known distance to ground
  }

  x = pdlOpticalFlowToMeters(x,ds->lidar.range,delta_pitch,fov,npx,scaler);
  y = pdlOpticalFlowToMeters(y,ds->lidar.range,delta_roll,fov,npx,scaler);

  ds->opticalFlow.sumX += x;
  ds->opticalFlow.sumY += y;

  // the code has been before median filter (it worked well)
  ds->opticalFlow.velX = x / dt;
  ds->opticalFlow.velY = y / dt;
  // apply median filter for of.velX
  // commented because not good results was received with median filters
  /*velX[medianPos] = x/dt;
  velY[medianPos] = y/dt;
  medianPos++;
  if(medianPos > 2)
    medianPos = 0;
  ds->opticalFlow.velX = pdlMedian3(velX[0],velX[1],velX[2]);
  ds->opticalFlow.velY = pdlMedian3(velY[0],velY[1],velY[2]);
  */

  // TODO pmw3901 is not restored when returned from dark room to light room, squal is bad
  // Rotate velocities to world frame
  cos_theta = cosf(ds->pose[PDL_YAW].pos);
  sin_theta = sinf(ds->pose[PDL_YAW].pos);

  ds->opticalFlow.velNorth = ds->opticalFlow.velX * cos_theta - ds->opticalFlow.velY * sin_theta;
  ds->opticalFlow.velEast = ds->opticalFlow.velX * sin_theta + ds->opticalFlow.velY * cos_theta;
  // Update KF
  // The variance of optical flow data is not constant and it depends from distance to ground
  // User have to measure the variance at alt1 and alt2
  // Next we use linear model to calculate variance for current distance to ground
  alt = ds->lidar.range;
  R = ds->kfSettings.ofVelVariance1;

  if(alt > a1)
  {
    b = (ds->kfSettings.ofVelVariance2 * a1 - ds->kfSettings.ofVelVariance1 * a2) / (a1 - a2);
    k = (ds->kfSettings.ofVelVariance1 - b) / a1;

    R = k*alt + b;
  }

  // We have bad data if optical flow squal is below 86. It can be caused by darkness
  if(ds->opticalFlow.squal < 86)  // level 86 is determined by experiment
  {
    R *= (float)(86 - ds->opticalFlow.squal); // tell to Kalman filter that we have bad optical flow data
  }

  pdlNavKF_CorrectByVelMeasurement(&navKf[PDL_NORTH],&ds->nav[PDL_NORTH],ds->opticalFlow.velNorth,R);
  pdlNavKF_CorrectByVelMeasurement(&navKf[PDL_EAST],&ds->nav[PDL_EAST],ds->opticalFlow.velEast,R);
}

void pdlNewLidarData(pdlDroneState *ds, float range, float fov)
{
  static float old_range = 0;
  static float old_vz = 0;
  static uint32_t old_micros = 0;
  // change to 3 or more to turn on Median Filter to filter spikes of velZ
  // but Median Filter is not effective solution in this case
  #define VELZ_BUF_SIZE 1
  static float velZ[VELZ_BUF_SIZE];
  static uint8_t bufPos = 0;
  static uint8_t horVelStabCounter = 0;

  float dt;
  float R;
  float vz;

  ds->lidar.range = range;

  if(range < 0)
    return;

  float pitch = ds->pose[PDL_PITCH].pos;
  float roll = ds->pose[PDL_ROLL].pos;

  // tilt compensation
  // this formula from craziflie project
  // (eq.6.53 from Modelling and Control of the Crazyflie Quadrotor for Aggressive and
  // Autonomous Flight by Optical Flow Driven State Estimation by Marcus Greiff
  /*float a = acosf(cosf(pitch) * cosf(roll));
  a = fabsf(a);
  if(a > fov/2.f)
  {
    LOG_INFO("r1=%i,a=%i",(int)(ds->lidar.range*1000.f),(int)(a*180.f/3.14f));
    ds->lidar.range *= cosf(a - fov/2.f);
    LOG_INFO("r2=%i",(int)(ds->lidar.range*1000.f));
  }
  */

  (void)fov;
  // this from https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/sensors.cpp#L34
  ds->lidar.range *= cosf(pitch)*cosf(roll);
  // additional compensation if lidar is displaced from gravity center
  ds->lidar.range -= tanf(pitch)*PDL_LIDAR_OFFSET;

  dt = pdlGetDeltaTime(pdlMicros(),old_micros);
  old_micros = pdlMicros();
  dt /= 1000000.0f;

  // to prevent drone shaking in case when lidar were wrong long time and becomes right
  if(dt > (float)(PDL_LIDAR_READ_PERIOD*3)/1000000.f)
  {
    old_range = ds->lidar.range;
    old_vz = 0;
    return;
  }

  // Median filters adds time delay! It leads to jumping when we try stabilize velocity-Z
  // And in this cause median filters prevent from spikes not good
  vz = velZ[bufPos] = (ds->lidar.range - old_range)/dt;
  old_range = ds->lidar.range;
  bufPos++;
  if(bufPos >= VELZ_BUF_SIZE)
  {
    bufPos = 0;
  }

  if(VELZ_BUF_SIZE == 3)
  {
    vz = pdlMedian3(velZ[0],velZ[1],velZ[2]);
  }
  else if(VELZ_BUF_SIZE > 3)
  {
    vz = pdlMedian(velZ,VELZ_BUF_SIZE);
  }

  // Better way became to simply filter velZ by delta velZ
  // and to reduce uncertainty of velZ in Kalman filter when drone moves
  if(fabsf((vz - old_vz)/dt) < PDL_G)
  {
    ds->lidar.velZ = vz;

    R = ds->kfSettings.lidarVelVariance2;

    if( fabsf(ds->velocityXPid.input) < 0.3f &&
        fabsf(ds->velocityYPid.input) < 0.3f )
    {
      if(horVelStabCounter >= 20)  // drone holds position, horizontal velocity below 0.3 m/s at least 2s
      {
        R = ds->kfSettings.lidarVelVariance1;
      }
      else
      {
        horVelStabCounter++;
      }
    }
    else
    {
      horVelStabCounter = 0;
    }

    pdlNavKF_CorrectByVelMeasurement(  &navKf[PDL_Z],
                                       &ds->nav[PDL_Z],
                                       ds->lidar.velZ,
                                       R);
  }
  old_vz = vz;
}

void pdlNewGpsData( pdlDroneState *ds,
                    uint8_t fixType,
                    uint16_t numSV,
                    float lat,
                    float lon,
                    float alt,
                    float course,
                    float velN,
                    float velE,
                    float velUp,
                    float hAcc,
                    float vAcc,
                    float sAcc)
{
  float distance;
  float bearing;
  float hPosR;
  float vPosR;
  float hVelR;
  float vVelR;

  ds->gps.fixType = fixType;
  ds->gps.numSV = numSV;
  ds->gps.curLat = lat;
  ds->gps.curLon = lon;
  ds->gps.curAlt = alt;
  ds->gps.course = course;
  ds->gps.velN = velN;
  ds->gps.velE = velE;
  ds->gps.velU = velUp;
  ds->gps.hAcc = hAcc;
  ds->gps.vAcc = vAcc;
  ds->gps.sAcc = sAcc;

  if(  ds->gps.startLat != 0 &&
       ds->gps.startLon != 0 &&
       ds->gps.startAlt != 0 &&
       ds->gps.enabled &&
       //ds->gps.fixType == 3 &&
       //ds->gps.numSV >= 6)
       ds->gps.fixType >= 2 &&
       ds->gps.fixType <= 3)
   {
     distance = pdlGpsDistance(ds->gps.startLat, ds->gps.startLon, ds->gps.curLat, ds->gps.curLon);
     bearing = pdlGpsBearing(ds->gps.startLat, ds->gps.startLon, ds->gps.curLat, ds->gps.curLon);

     ds->gps.posNorth = pdlGpsNorthDistance(distance,bearing);
     ds->gps.posEast = pdlGpsEastDistance(distance,bearing);
     ds->gps.posUp = ds->gps.curAlt - ds->gps.startAlt;

     hPosR = ds->gps.hAcc * ds->gps.hAcc * ds->kfSettings.gpsHorPosVariance;
     vPosR = ds->gps.vAcc * ds->gps.vAcc * ds->kfSettings.gpsVerPosVariance;
     hVelR = ds->gps.sAcc * ds->gps.sAcc * ds->kfSettings.gpsHorSpeedVariance;
     vVelR = ds->gps.sAcc * ds->gps.sAcc * ds->kfSettings.gpsVerSpeedVariance;

     /*
     hPosR = ds->kfSettings.gpsHorPosVariance;
     vPosR = ds->kfSettings.gpsVerPosVariance;
     hVelR = ds->kfSettings.gpsHorSpeedVariance;
     vVelR = ds->kfSettings.gpsVerSpeedVariance;
     */

     pdlNavKF_CorrectByPosMeasurement(  &navKf[PDL_Z],
                                        &ds->nav[PDL_Z],
                                        ds->gps.posUp,
                                        vPosR);

     pdlNavKF_CorrectByPosMeasurement(  &navKf[PDL_NORTH],
                                        &ds->nav[PDL_NORTH],
                                        ds->gps.posNorth,
                                        hPosR);

     pdlNavKF_CorrectByPosMeasurement(  &navKf[PDL_EAST],
                                        &ds->nav[PDL_EAST],
                                        ds->gps.posEast,
                                        hPosR);

     pdlNavKF_CorrectByVelMeasurement(  &navKf[PDL_Z],
                                        &ds->nav[PDL_Z],
                                        ds->gps.velU,
                                        vVelR);

     pdlNavKF_CorrectByVelMeasurement(  &navKf[PDL_NORTH],
                                        &ds->nav[PDL_NORTH],
                                        ds->gps.velN,
                                        hVelR);

     pdlNavKF_CorrectByVelMeasurement(  &navKf[PDL_EAST],
                                        &ds->nav[PDL_EAST],
                                        ds->gps.velE,
                                        hVelR);
   }
}

void pdlNewBaroData( pdlDroneState *ds,
                     float pressure,
                     float alt)
{
  ds->baro.pressure = pressure;
  ds->baro.altitude = alt;

  pdlNavKF_CorrectByPosMeasurement(  &navKf[PDL_Z],
                                     &ds->nav[PDL_Z],
                                     ds->baro.altitude,
                                     ds->kfSettings.baroAltVariance);
}

void pdlNewTemperatureData( pdlDroneState *ds, float temp)
{
  ds->temperature = temp;
}
