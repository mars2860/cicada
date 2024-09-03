/**
 *  @mainpage Introduction
 *
 *  Portable Drone Logic (PDL) is the lightweight pure C library implements a common drone logic. The goal of
 *  this project is to be easy to use and easy to port to your hardware platform. PDL supports only quadcopters
 *  at this time
 *
 *  It provides for you
 *  - data model
 *  - application template
 *  - task scheduler
 *  - chain of PIDs
 *  - parser for binary digital commands
 *  - sensors fusing by linear Kalman filters to estimate pose, velocities, coordinates
 *
 *  You need to implement yourself on your platform
 *  - sensors reading
 *  - motor speed regulation (PWM, DSHOT, etc)
 *  - remote commands receiving
 *  - telemetry sending to host or saving to disk
 *  - logging
 *  - firmware update process
 *
 *  System requirements:
 *  - Float data processing
 *  - 1 kB of RAM
 *
 * Pages
 *  - @subpage porting "How to port"
 *  - @subpage task_scheduler "Task scheduler"
 *  - @subpage chain_of_pids "Chain of PIDs"
 *  - @subpage data_model "Data model"
 *  - @subpage commands "Binary commands"
 *
 * Usage
 * - @ref pdlData
 * - @ref pdlInterface
 * - @ref pdlPortableInterface
 * - @ref pdlConfig
 * - @ref pdlCmds
 *
 *  @author anton.sysoev.ru68@gmail.com
 *  @copyright GNU Public License
 */

/**
   * @page task_scheduler Task scheduler
   *
   * - Task needs some cpu time to be executed
   * - One task can't be interrupted by other task
   * - Task runs periodically as it defined by its update period in pdl_config.h
   *
   * The goal of this task scheduler is to prevent a situation when in one moment periods of many
   * task are elapsed simultaneously. In this situation update period will be very long. To prevent
   * this situation each task has priority. We have PDL_DESIRE_UPDATE_TIME. If in one moment cpu has
   * to execute many tasks and these tasks take time more than PDL_DESIRE_UPDATE_TIME thats low
   * priority tasks would be put aside for next pdlUpdate. In other word period of low priority
   * tasks will be increased automatically. If in the next pdlUpdate we have the same situations thats low priority
   * tasks would be put aside again or not. It is regulated by PDL_TASK_MAX_WAIT_TIME. Task scheduler guarantees
   * that task would be executed at some time. There is no situation that task never be executed.
   * Priority of task is order of task handler invoke. High priority task are invoked first
   */

/**
 * @page chain_of_pids Chain of PIDs
 *
 * @image html chain_of_pids.svg "Chain of PIDs" width=100%
 */

#ifndef PDL_H
#define PDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pdl_data.h"
#include "pdl_config.h"
#include "pdl_rc_cmds.h"
#include "pdl_port.h"
#include "pdl_utils.h"
#include "pdl_nav_kf.h"

int pdl_printf(const char *format, ... );
char* pdlGetLog();
int   pdlGetLogSize();
void  pdlResetLog();

#include "macrologger.h"

#define ERR_IMU_DATA_NOT_READY    1
#define ERR_MAG_DATA_NOT_READY    2
#define ERR_MAG_DATA_SKIP         4
#define ERR_LIDAR_DATA_NOT_READY  8
#define ERR_LIDAR_DATA_INVALID   16
#define ERR_PCA9536_NOT_READY    32
#define ERR_OF_DATA_NOT_READY    64
#define ERR_GPS_INIT_FAILED      128
#define ERR_GPS_DATA_INVALID     256
#define ERR_LIDAR_NOT_FOUND      512
#define ERR_OF_NOT_FOUND        1024
#define ERR_BARO_NOT_FOUND      2048

#define pdlResetError(ds,err) if(ds->errors & err)ds->errors^=err;
#define pdlSetError(ds,err) ds->errors |= err;

/// @defgroup pdlInterface Main functions

void pdlSetup(pdlDroneState*);
void pdlUpdate(pdlDroneState*);

void pdlStopMotors(pdlDroneState*);
void pdlSetMotorGas(pdlDroneState*, uint8_t, int32_t);
void pdlAddMotorGas(pdlDroneState*, uint8_t, int32_t);
void pdlSetBaseGas(pdlDroneState*, int32_t);
void pdlSetMotorMaxGas(int32_t);
void pdlSetMotorNullGas(int32_t);
void pdlSetMotorMinGas(int32_t);
int32_t pdlGetMotorMaxGas();
int32_t pdlGetMotorNullGas();
int32_t pdlGetMotorMinGas();

/** @param dt delta time in seconds
 *  @param angVal if set true it will keep error between -pi and +pi rad
 *  @return 1 when pid output has been updated and 0 if pid is disabled
 */
uint8_t pdlUpdatePid(pdlDroneState *ds, pdlPidState *pid, float input, float dt, uint8_t angVal);
void pdlResetPid(pdlPidState *pid);

/// Applies XYZ-Pids output values to motors of a cross-frame quadcopter
void pdlPidsToCrossFrame(pdlDroneState*);
/// Applies XYZ-Pids output values to motors of a reversed cross-frame quadcopter
void pdlPidsToCrossFrameReversed(pdlDroneState*);
/// Applies XYZ-Pids output values to motors of a x-frame quadcopter
void pdlPidsToXFrame(pdlDroneState*);
/// Applies XYZ-Pids output values to motors of a reversed x-frame quadcopter
void pdlPidsToXFrameReversed(pdlDroneState*);

/** Allows for user to change telemetry period or fully disable telemetry if he set this period as long as it possible */
void pdlSetTelemetryUpdatePeriod(pdlDroneState*, uint32_t);
uint32_t pdlGetTelemetryUpdatePeriod(void);

/// Parses binary packet with command from remote control and applies result to drone state
void pdlParseCommand(pdlDroneState*,uint8_t*);

/** Provides to system new data from gyroscope
 * @param rawX raw data from gyroscope around X axis (this value is no matter and intended for telemetry)
 * @param rawY raw data from gyroscope around Y axis (this value is no matter and intended for telemetry)
 * @param rawZ raw data from gyroscope around Z axis (this value is no matter and intended for telemetry)
 * @param pureX rotation speed around PDL_X axis in rad/s. If your gyroscope axis is not equal PDL axis you have to convert it
 * @param pureY rotation speed around PDL_Y axis in rad/s. If your gyroscope axis is not equal PDL axis you have to convert it
 * @param pureZ rotation speed around PDL_Z axis in rad/s. If your gyroscope axis is not equal PDL axis you have to convert it
 */
void pdlNewGyroData(pdlDroneState *ds, int16_t rawX, int16_t rawY, int16_t rawZ, float pureX, float pureY, float pureZ);

/** Provides to system new data from accelerometer
 * @param rawX raw data from accel along X axis (this value is no matter and intended for telemetry)
 * @param rawY raw data from accel along Y axis (this value is no matter and intended for telemetry)
 * @param rawZ raw data from accel along Z axis (this value is no matter and intended for telemetry)
 * @param pureX acceleration along PDL_X axis in m/s^2. If your accel axis is not equal PDL axis you have to convert it
 * @param pureY acceleration along PDL_Y axis in m/s^2. If your accel axis is not equal PDL axis you have to convert it
 * @param pureZ acceleration along PDL_Z axis in m/s^2. If your accel axis is not equal PDL axis you have to convert it
 */
void pdlNewAccelData(pdlDroneState *ds, int16_t rawX, int16_t rawY, int16_t rawZ, float pureX, float pureY, float pureZ);

/** Provides to system new data from magnetometer
 * @param rawX raw data from magneto along X axis (this value is no matter and intended for telemetry)
 * @param rawY raw data from magneto along Y axis (this value is no matter and intended for telemetry)
 * @param rawZ raw data from magneto along Z axis (this value is no matter and intended for telemetry)
 * @param pureX magnetic field along PDL_X axis in mGauss. If your magneto axis is not equal PDL axis you have to convert it
 * @param pureY magnetic field along PDL_Y axis in mGauss. If your magneto axis is not equal PDL axis you have to convert it
 * @param pureZ magnetic field along PDL_Z axis in mGauss. If your magneto axis is not equal PDL axis you have to convert it
 */
void pdlNewMagnetoData(pdlDroneState *ds, int16_t rawX, int16_t rawY, int16_t rawZ, float pureX, float pureY, float pureZ);

/** Provides to system new data from optical flow sensor
 * @param rawX optical flow data along PDL_X
 * @param rawY optical flow data along PDL_Y
 * @param squal surface quality determined by optical flow sensor
 * @param fov Field of view in radians. See in datasheet of sensor
 * @param npx Matrix size in pixels. See in datasheet of sensor
 * @param scaler Magic number. It is determined by experiment.
 *               Rotate drone by roll. OpticalFlow.pureY should to be zero if scaler is right
 */
void pdlNewOpticalFlowData(pdlDroneState *ds, int16_t rawX, int16_t rawY, uint8_t squal, float fov, float npx, float scaler);

/** Provides to system new data from lidar/sonar sensor
 * @param range distance in meters
 * @param fov Field of view in radians. See in datasheet of sensor
 */
void pdlNewLidarData(pdlDroneState *ds, float range, float fov);

/** Provides to system new data from GPS sensor
 *
 * @param ds
 * @param fixType Position fix type. See ublox gps receiver datasheet
 * @param numSV Number satellites in view
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude above sea in meters
 * @param course Course in degrees
 * @param velN Velocity along North in m/s
 * @param velE Velocity along East in m/s
 * @param velUp Velocity along Up in m/s
 * @param hAcc 1-sigma two-dimensional horizontal position accuracy in meters
 * @param vAcc 1-sigma one-dimensional vertical position accuracy in meters
 * @param sAcc 1-sigma one-dimensional speed accuracy in m/s
 */
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
                    float sAcc);

/** Provides to system new data from baro sensor
 *
 * @param ds
 * @param pressure
 * @param alt
 */
void pdlNewBaroData( pdlDroneState *ds,
                     float pressure,
                     float alt);

/**  Provides to system new data from temperature sensor
 *
 * @param ds
 * @param temp
 */
void pdlNewTemperatureData( pdlDroneState *ds, float temp);

/// @}

/**
 *  Resets all PIDs targets to zero and make velocityZPid.target = safeVeloZ until
 *  altitude is higher safeAlt or lower -safeAlt. Call this function in your mainLoop()
 *  if you need this feature
 */
void pdlPreventFlyAway(pdlDroneState *ds, int32_t rssiMinLevel, float safeAlt, float safeVeloZ);

void pdlSetPitchTarget(pdlDroneState *ds, float target);
void pdlSetRollTarget(pdlDroneState *ds, float target);
void pdlSetYawRateTarget(pdlDroneState *ds, float target);
void pdlSetHeadingTarget(pdlDroneState *ds, float target);
void pdlSetPitchRateTarget(pdlDroneState *ds, float target);
void pdlSetRollRateTarget(pdlDroneState *ds, float target);
void pdlSetZRateTarget(pdlDroneState *ds, float target);
void pdlSetXRateTarget(pdlDroneState *ds, float target);
void pdlSetYRateTarget(pdlDroneState *ds, float target);
void pdlSetVelocityXTarget(pdlDroneState *ds, float target);
void pdlSetVelocityYTarget(pdlDroneState *ds, float target);
void pdlSetVelocityZTarget(pdlDroneState *ds, float target);

void pdlEnableMotors(pdlDroneState *ds, uint8_t enable);
void pdlEnableStabilization(pdlDroneState *ds, uint8_t enable);
void pdlEnableTrickMode(pdlDroneState *ds, uint8_t enable);
void pdlSetEscMode(pdlDroneState *ds, uint8_t mode);
void pdlSetFrameType(pdlDroneState *ds, uint8_t frame);

#ifdef __cplusplus
}
#endif

#endif
