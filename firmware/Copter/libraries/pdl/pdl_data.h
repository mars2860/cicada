#ifndef PDL_DATA_H
#define PDL_DATA_H

#include <stdint.h>
#include "pdl_config.h"

/**
 * @page data_model Data model
 *
 * Data model is presented by pdlDroneState. It stores drone state at one moment of time.
 * You need to declare one pdlDroneState instance in global memory space.
 *
 * @ref pdlData
 *
 * @defgroup pdlData Data structures
 * @{
 */

#define PDL_VERSION 1

#define PDL_X     0
#define PDL_Y     1
#define PDL_Z     2

#define PDL_HOLDPOS_DISABLED 0
#define PDL_HOLDPOS_X        1
#define PDL_HOLDPOS_Y        2
#define PDL_HOLDPOS_BOTH_XY  3

/// Your drone frame type. Define it in pdl_config.h
typedef enum
{
  PDL_DRONE_FRAME_CROSS,
  PDL_DRONE_FRAME_X
} pdlDroneFrame;

/// Battery state takes 8 bytes
typedef struct s_pdlBatteryState
{
  float voltage;
  float percent;
} pdlBatteryState;

/// PID state takes 36 bytes
typedef struct s_pdlPidState
{
  float kp;
  float ki;
  float kd;
  float target;
  float out;
  float errSum;
  float input;
  float maxOut;
  //float maxErrSum;  // maxSum is restricted by maxOut
  uint32_t enabled; // not use uint8_t because it needs to align to be divisible by 4 bytes
} pdlPidState;

/// Remote control state takes 4 bytes
typedef struct s_pdlRcState
{
  int32_t rssi;
} pdlRcState;

/// Optical flow state takes 4 bytes
typedef struct s_pdlOpticalFlowSate
{
  /// Sensor raw x data. You have to convert it to velocity[PDL_X] to be used by system
  int16_t rawX;
  /// Sensor raw y data. You have to convert it to velocity[PDL_Y] to be used by system
  int16_t rawY;
} pdlOpticalFlowState;

/// Barometer state takes 12 bytes
typedef struct s_pdlBaroState
{
  /// Atmospheric pressure. The value has to be in hPa
  float pressure;
  /// Altitude above sea level. The value has to be in meters
  float altitude;
  /// Atmospheric pressure at sea level. The value has to be in hPa
  float seaLevelPressure;
} pdlBaroState;

/// Triple axis sensor state takes 24 bytes
typedef struct s_pdlTripleAxisSensorState
{
  /// Values after digital filters and converted to metric units
  float pure[3];
  /// Raw values
  int16_t raw[3];
  /// Calibration offsets (Depends from your sensor)
  int16_t offset[3];
} pdlTripleAxisSensorState;

/// Drone state takes 504 bytes
typedef struct s_pdlDroneState
{
  /// pdl version
  uint8_t version;
  /// Motors enabled
  uint8_t motorsEnabled;
  /// Global flag to enable PIDs
  uint8_t stabilizationEnabled;
  /// Hold position flag (one of macro PDL_HOLDPOS_). Velocity pids are applied to drone if this flag is above 0
  uint8_t holdPosEnabled;
  /// Base gas. This value is output of altitude PID. If altitude PID is disabled you have to control this value yourself
  int32_t baseGas;
  /// Motors gas. These values are intended to motor regulator. Gas means your PWM value, DSHOT value, etc.
  /// When PIDs are enabled these value are baseGas + pids output
  int32_t motorGas[PDL_MOTOR_COUNT];
  /// Current time in microseconds from system has been started
  uint32_t timestamp;
  /// Battery state
  pdlBatteryState battery;
  /// RemoteControl system state
  pdlRcState rc;
  /// Accelerometer state. Pure values have to be in G units
  pdlTripleAxisSensorState accel;
  /// Gyroscope state. Pure values have to be in rad/s units
  pdlTripleAxisSensorState gyro;
  /// Magnetometer state. Pure values have to be in mGauss
  pdlTripleAxisSensorState magneto;
  /// Yaw (see TaitBryan angles). The value has to be in rad
  float yaw;
  /// Pitch (see TaitBryan angles). The value has to be in rad
  float pitch;
  /// Roll ((see TaitBryan angles). The value has to be in rad
  float roll;
  /// Magnetic heading. The value has to be in rad
  float heading;
  /// Altitude above reference you selected. This value is used by altitude PIDs. The value has to be in meters
  float altitude;
  /// Velocities in m/s along drone axis and related to ground. These values are used in velocity PIDs
  float velocity[3];
  /// Averaging time of executing one loop in microseconds
  uint32_t avgLoopTime;
  /// Max time of executing one loop in microseconds
  uint32_t maxLoopTime;
  /// Temperature of air. The value has to be in Celsius degree
  float temperature;
  /// Lidar range in meters. If value is less 0 that means there is error in lidar system
  float lidarRange;
  /// Barometer state
  pdlBaroState baro;
  /// OpticalFlow state
  pdlOpticalFlowState opticalFlow;
  /// Yaw Rate PID
  pdlPidState yawRatePid;
  /// Pitch Rate PID
  pdlPidState pitchRatePid;
  /// Roll Rate PID
  pdlPidState rollRatePid;
  /// Pitch PID
  pdlPidState pitchPid;
  /// Roll PID
  pdlPidState rollPid;
  /// PID to hold altitude
  pdlPidState altPid;
  /// VelocityX PID. This pid is used only to hold position. Change target for yaw/pitch/roll pids to move a drone
  pdlPidState velocityXPid;
  /// VelocityY PID. This pid is used only to hold position. Change target for yaw/pitch/roll pids to move a drone
  pdlPidState velocityYPid;
  /// VelocityZ PID
  pdlPidState velocityZPid;
} pdlDroneState;

/// @}

#endif
