/**
 *  This is Portable Drone Logic. The lightweight pure C library implements common drone logic
 *  and is easy portable to various hardware profiles
 *
 *  @author anton.sysoev.ru68@gmail.com
 */
#ifndef PDL_H_
#define PDL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//-----------------------------------------------------------------------------
// SETTINGS

/// Number of motors, motors are indexed from left-top in counterclockwise direction like pins of chip
#define PDL_MOTOR_COUNT                 4
/// Default maximum gas of motor. Use pdlSetMotorGasLimits in your pdlSetupEscaper
#define PDL_DEFAULT_MAX_MOTOR_GAS       2000
#define PDL_DEFAULT_NULL_MOTOR_GAS      0
#define PDL_DEFAULT_MIN_MOTOR_GAS       0
/// accel+gyro+magneto read period in microseconds. Use pdlSetImuReadPeriod in your pdlSetupGyro/Accel/Magneto
#define PDL_DEFAULT_IMU_READ_PERIOD     5000
/// Barometer read period in microseconds. Use pdlSetBaroReadPeriod in your pdlSetupBaro
#define PDL_DEFAULT_BARO_READ_PERIOD    50000
/// Battery read period in microseconds. Use pdlSetBatteryReadPeriod in your pdlSetupBattery
#define PDL_DEFAULT_BATTERY_READ_PERIOD 100000
/// Escaper update period. Use pdlSetEscaperUpdatePeriod in pdlEscaperSetup
#define PDL_DEFAULT_ESCAPER_UPDATE_PERIOD 0
/// Remote control update period. Use pdlSetRcUpdatePeriod in pdlRcSetup
#define PDL_DEFAULT_RC_UPDATE_PERIOD     0
/// Lidar update period. Use pdlSetLidarReadPeriod in pdlLidarSetup
#define PDL_DEFAULT_LIDAR_READ_PERIOD 50000
/// OpticalFlow read period. Use pdlSetOpticalFlowReadPeriod in pdlLidarSetup
#define PDL_DEFAULT_OPTICAL_FLOW_READ_PERIOD 50000
/// Telemetry update period
#define PDL_DEFAULT_TELEMETRY_PERIOD 50000
/// Desire time for one loop
#define PDL_DESIRE_UPDATE_TIME 5000
/// Max wait time for task to be executed
#define PDL_TASK_MAX_WAIT_TIME 10000
/// Maximum lidar range in meters
#define PDL_LIDAR_MAX_RANGE 3.f
/// Drone frame PDL_DRONE_FRAME_X | PDL_DRONE_FRAME_CROSS
#define PDL_DRONE_FRAME                 PDL_DRONE_FRAME_CROSS
/// comment to enable symmetric motors regulation in stabilization
//#define PDL_ASYMMETRIC_STABILIZATION

//-----------------------------------------------------------------------------

#define PDL_X     0
#define PDL_Y     1
#define PDL_Z     2

#define PDL_HOLDPOS_DISABLED 0
#define PDL_HOLDPOS_X        1
#define PDL_HOLDPOS_Y        2
#define PDL_HOLDPOS_BOTH_XY  3

//---------------------------------------------------------------
// COMMANDS

#define CMD_SWITCH_MOTORS         100
#define CMD_SET_MOTORS_GAS        101
#define CMD_SET_ACCEL             102
#define CMD_SET_GYRO              103
#define CMD_SET_MAGNETO           104
#define CMD_SELF_CALIB_ACCEL      105
#define CMD_SELF_CALIB_GYRO       106
#define CMD_SET_YAW_RATE_PID      107
#define CMD_SET_PITCH_PID         108
#define CMD_SET_ROLL_PID          109
#define CMD_SET_ALT_PID           110
#define CMD_SET_YPR               111
#define CMD_SET_PERIODS           112
#define CMD_ENABLE_STABILIZATION  113
#define CMD_RESET_ALTITUDE        114
#define CMD_SET_SEA_LEVEL         115
#define CMD_SET_ALTITUDE          116
#define CMD_SET_BASE_GAS          117
#define CMD_SET_PITCH_RATE_PID    118
#define CMD_SET_ROLL_RATE_PID     119
#define CMD_SET_VELOCITY_X_PID    120
#define CMD_SET_VELOCITY_Y_PID    121
#define CMD_SET_VELOCITY_Z_PID    122
#define CMD_SET_VELOCITY_Z        123

//-----------------------------------------------------------------------------

typedef enum
{
  PDL_DRONE_FRAME_CROSS,//!< PDL_DRONE_FRAME_CROSS
  PDL_DRONE_FRAME_X     //!< PDL_DRONE_FRAME_X
} pdlDroneFrame;

typedef struct s_pdlBatteryState
{
  float voltage;
  float percent;
} pdlBatteryState;

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
  //float maxErrSum;
  uint32_t enabled; // not use uint8_t because it needs to align to be divisible by 4 bytes
} pdlPidState;  // 36 bytes

typedef struct s_pdlRcState
{
  int32_t rssi;
} pdlRcState;

typedef struct s_pdlOpticalFlowSate
{
  int16_t rawX;
  int16_t rawY;
} pdlOpticalFlowState;

typedef struct s_pdlBaroState
{
  // hPa
  float pressure;
  // meters
  float altitude;
  // hPa
  float seaLevelPressure;
} pdlBaroState;

typedef struct s_pdlDroneState
{
  /// values after digital filters and converted to metric units
  float pure[3];
  /// raw values
  int16_t raw[3];
  /// calibration offsets
  int16_t offset[3];
} pdlTripleAxisSensorState; // 24 bytes

typedef struct
{
  uint32_t timestamp;
  pdlBatteryState battery;
  pdlRcState rc;
  /// accelerometer pure values are in G units
  pdlTripleAxisSensorState accel;
  /// gyro pure values are in rad/s units
  pdlTripleAxisSensorState gyro;
  /// magneto pure values are in mGauss
  pdlTripleAxisSensorState magneto;
  // radians
  float yaw;
  // radians
  float pitch;
  // radians
  float roll;
  // radians
  float heading;
  // meters (can be lidarRange or baroAlt or fused, this value used by altPid)
  float altitude;
  // velocities in m/s along drone axis and related to ground
  float velocity[3];
  // Averaging time of executing one loop in microseconds
  uint32_t avgLoopTime;
  // Max time of executing one loop in microseconds
  uint32_t maxLoopTime;
  // degree
  float temperature;
  // Lidar range in meters (if less 0, that means there is error in lidar)
  float lidarRange;
  // baro
  pdlBaroState baro;
  // OpticalFlow
  pdlOpticalFlowState opticalFlow;
  // Yaw Rate PID
  pdlPidState yawRatePid;
  // Pitch Rate PID
  pdlPidState pitchRatePid;
  // Roll Rate PID
  pdlPidState rollRatePid;
  // Pitch PID
  pdlPidState pitchPid;
  // Roll PID
  pdlPidState rollPid;
  // Alt PID
  pdlPidState altPid;
  // VelocityX PID
  pdlPidState velocityXPid;
  // VelocityY PID
  pdlPidState velocityYPid;
  // VelocityZ PID
  pdlPidState velocityZPid;
  // Motors
  int32_t baseGas;
  int32_t motorGas[PDL_MOTOR_COUNT];
  uint8_t motorsEnabled;
  // Stabilization
  uint8_t stabilizationEnabled;
  // Hold position (one of macro PDL_HOLDPOS_)
  uint8_t holdPosEnabled;
  /// explicit struct align to divisible by 4
  uint8_t gap;
} pdlDroneState;

void pdlSetup(pdlDroneState*);
void pdlUpdate(pdlDroneState*);

uint32_t pdlGetDeltaTime(uint32_t cur, uint32_t last);

void pdlSetMotorGasLimits(int32_t min, int32_t nul, int32_t max);
void pdlSetImuReadPeriod(uint32_t);
void pdlSetBaroReadPeriod(uint32_t);
void pdlSetBatteryReadPeriod(uint32_t);
void pdlSetEscaperUpdatePeriod(uint32_t);
void pdlSetRcUpdatePeriod(uint32_t);
void pdlSetLidarReadPeriod(uint32_t);
void pdlSetOpticalFlowReadPeriod(uint32_t);

void pdlSetTelemetryPeriod(uint32_t);
uint32_t pdlGetTelemetryPeriod();

void pdlStopMotors(pdlDroneState*);
void pdlSetMotorGas(pdlDroneState*, uint8_t, int32_t);
void pdlAddMotorGas(pdlDroneState*, uint8_t, int32_t);

void pdlSetLidarMaxRange(float);
float pdlGetLidarMaxRange();

/** @param dt delta time in microseconds */
void pdlUpdatePid(pdlPidState*, float input, float dt);
void pdlResetPid(pdlPidState*);

/// Parses binary packet with command from remote control and applies result to drone state
void pdlParseCommand(pdlDroneState*,uint8_t*);

/**
 *  Calcs heading with tilt compensation. Accel/Gyro/Magneto axes should be sorted out as in MPU9250
 *
 *  @return heading also save result to pdlDroneState.heading
 */
float pdlCalcHeading(pdlDroneState*);

/**
 * Complementary filter
 *
 * @param a         filter coefficient
 * @param rate      change rate of value in units/seconds
 * @param dt        delta time in seconds
 * @param value     current value
 * @param newValue  new value
 *
 * @return new fusion value
 */
float pdlComplementaryFilter(float alpha, float rate, float value, float newValue, float dt);

/**
 *  Triple axis sensors fusion by complementary filter.
 *
 *  Accel/Gyro/Magneto axes should be sorted out as in MPU9250.
 *
 *  Call it in your pdlTripleAxisSensorFusion if you want this method
 *  to calculate yaw, pitch roll
 *
 *  Yaw - rotate around Z
 *  Pitch - rotate around Y
 *  Roll - rotate around X
 *
 *  @return results are saved in pdlDroneState
 */
void pdlComplFilterTripleAxisFusion(pdlDroneState *ds, float alpha, float dt);

//-----------------------------------------------------------------------------
// IMPLEMENT THESE FUNCTIONS IN YOUR SYSTEM

uint32_t pdlMicros(void);

void pdlSetupRc(pdlDroneState*);
void pdlSetupEscaper(pdlDroneState*);
void pdlSetupAccel(pdlDroneState*);
void pdlSetupGyro(pdlDroneState*);
void pdlSetupMagneto(pdlDroneState*);
void pdlSetupBaro(pdlDroneState*);
void pdlSetupBattery(pdlDroneState*);
void pdlSetupLidar(pdlDroneState*);
void pdlSetupOpticalFlow(pdlDroneState*);
void pdlSetupTelemetry(pdlDroneState*);

void pdlCalibrateAccel(pdlDroneState*);
void pdlCalibrateGyro(pdlDroneState*);

/// @note should to take motorGas from pdlDroneState and send it to escaper
void pdlUpdateEscaper(pdlDroneState*);
/// @note implements your remote control this @return RC state should be save in pdlDroneState.rc
void pdlRemoteControl(pdlDroneState*);
/// @return result should be saved in pdlDroneState.battery
void pdlReadBattery(pdlDroneState*);
/// @return result should be saved in pdlDroneState.accel
void pdlReadAccel(pdlDroneState*);
/// @return result should be saved in pdlDroneState.gyro
void pdlReadGyro(pdlDroneState*);
/// @return result should be saved in pdlDroneState.magneto
void pdlReadMagneto(pdlDroneState*);
/// implements your telemetry in/out
void pdlUpdateTelemetry(pdlDroneState*);
/**
 * @note result should be saved in pdlDroneState.temperature,pressure,altitude
 *
 * @return 1 if to need update altPid
 */
uint8_t pdlReadBaro(pdlDroneState*);
/**
 * @note result should be saved in pdlDroneState.lidarRange
 *
 * @return 1 if to need update altPid
 */
uint8_t pdlReadLidar(pdlDroneState*);
/// @return result should be saved in pdlDroneState.opticalFlow
void pdlReadOpticalFlow(pdlDroneState*);
/// @return result should be saved in pdlDroneState.yaw,pitch,roll
void pdlTripleAxisSensorFusion(pdlDroneState*);

#ifdef __cplusplus
}
#endif

#endif
