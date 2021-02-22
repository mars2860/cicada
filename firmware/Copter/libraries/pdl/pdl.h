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

/// Number of motors, motors are indexed from left-top in counterclockwise direction
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
/// Drone frame PDL_DRONE_FRAME_X | PDL_DRONE_FRAME_CROSS
#define PDL_DRONE_FRAME                 PDL_DRONE_FRAME_CROSS
/// comment to enable symmetric motors regulation in stabilization
//#define PDL_ASYMMETRIC_STABILIZATION

//-----------------------------------------------------------------------------

#define PDL_X     0
#define PDL_Y     1
#define PDL_Z     2

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
  float prevInput;
  uint32_t enabled; // not use uint8_t because it needs to align divisible by 4 bytes
} pdlPidState;  // 32 bytes

typedef struct s_pdlRcState
{
  int32_t rssi;
} pdlRcState;

typedef struct s_pdlDroneState
{
  /// values after digital filters and converted to metric units
  float pure[3];
  /// raw values
  int16_t raw[3];
  /// values after digital filters
  int16_t filtered[3];
  /// calibration offsets
  int16_t offset[3];
  /// explicit struct align to divisible by 4
  int16_t gap;
} pdlTripleAxisSensorState; // 32 bytes

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
  // Time of executing one loop in microseconds
  uint32_t mainLoopTime;
  // degree
  float temperature;
  // hPa
  float pressure;
  // meters
  float altitude;
  // hPa
  float seaLevel;
  // Yaw Rate PID
  pdlPidState yawRatePid;
  // Pitch PID
  pdlPidState pitchPid;
  // Roll PID
  pdlPidState rollPid;
  // Alt PID
  pdlPidState altPid;
  // Motors
  int32_t baseGas;
  int32_t motorGas[PDL_MOTOR_COUNT];
  uint8_t motorsEnabled;
  // Stabilization
  uint8_t stabilizationEnabled;
  /// explicit struct align to divisible by 4
  int16_t gap;
} pdlDroneState;

void pdlSetup(pdlDroneState*);
void pdlUpdate(pdlDroneState*);

void pdlSetMotorGasLimits(int32_t min, int32_t nul, int32_t max);
void pdlSetImuReadPeriod(uint32_t);
void pdlSetBaroReadPeriod(uint32_t);
void pdlSetBatteryReadPeriod(uint32_t);
void pdlSetEscaperUpdatePeriod(uint32_t);
void pdlSetRcUpdatePeriod(uint32_t);

void pdlStopMotors(pdlDroneState*);
void pdlSetMotorGas(pdlDroneState*, uint8_t, int32_t);
void pdlAddMotorGas(pdlDroneState*, uint8_t, int32_t);

void pdlUpdatePid(pdlPidState*, float input, float dt);
void pdlResetPid(pdlPidState*);

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
// IMPLEMENT THESE FUNCTION IN YOUR SYSTEM

uint32_t pdlMicros(void);

void pdlSetupRc(pdlDroneState*);
void pdlSetupEscaper(pdlDroneState*);
void pdlSetupAccel(pdlDroneState*);
void pdlSetupGyro(pdlDroneState*);
void pdlSetupMagneto(pdlDroneState*);
void pdlSetupBaro(pdlDroneState*);
void pdlSetupBattery(pdlDroneState*);

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
/// @return result should be saved in pdlDroneState.temperature,pressure,altitude
void pdlReadBaro(pdlDroneState*);
/// @return result should be saved in pdlDroneState.yaw,pitch,roll
void pdlTripleAxisSensorFusion(pdlDroneState*);

#ifdef __cplusplus
}
#endif

#endif
