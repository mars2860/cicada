#ifndef PDL_RC_CMDS_H
#define PDL_RC_CMDS_H

/**
 * @page commands Binary commands
 *
 * PDL provides binary commands and parser to remote control a drone.
 * The command has easy format = {1 byte (id of command), n bytes (command data)}
 * If you need CRC check you has to implement it yourself
 *
 * @ref pdlCmds
 *
 * @defgroup pdlCmds Commands format
 * @{
 *
 */

/// Switches on/off motors, also resets all pids targets if motors switched off. Format = {100, uint8_t enabled}
#define PDL_CMD_SWITCH_MOTORS         100
/// Sets gas of each motor. Format = {101, int32_t, int32_t, int32_t, int32_t}
#define PDL_CMD_SET_MOTORS_GAS        101
/// Sets accel offsets and invokes pdlSetupAccel. Format = {102, int16_t, int16_t, int16_t}
#define PDL_CMD_SET_ACCEL             102
/// Sets gyro offsets and invokes pdlSetupGyro. Format = {103, int16_t, int16_t, int16_t}
#define PDL_CMD_SET_GYRO              103
/// Sets magneto offsets and invokes pdlSetupMagneto. Format = {104, int16_t, int16_t, int16_t}
#define PDL_CMD_SET_MAGNETO           104
/// Invokes pdlCalibrateAccel. Format = {105}
#define PDL_CMD_SELF_CALIB_ACCEL      105
/// Invokes pdlCalibrateGyro. Format = {106}
#define PDL_CMD_SELF_CALIB_GYRO       106
/// Sets yawRatePid. Format = {107, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_YAW_RATE_PID      107
/// Sets pitchPid. Format = {108, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_PITCH_PID         108
/// Sets rollPid. Format = {109, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_ROLL_PID          109
/// Sets altPid. Format = {110, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_ALT_PID           110
/// Sets targets for yawRatePid,pitchPid,rollPid. Format = {111, float targetYawRate, float targetPitch, float targetRoll}
/// If pitchPid is disabled it sets targetPitchRate. If rollPid is disabled it sets targetRollRate. All values have to be in rad (rad/s)
#define PDL_CMD_SET_YPR               111
/// Sets telemetry update period in microseconds. Format = {112, uint32_t}
#define PDL_CMD_SET_TELEMETRY_PERIOD  112
/// Switches on/off all pids. Format = {113, uint8_t}
#define PDL_CMD_ENABLE_STABILIZATION  113
/// Applies current atmospheric pressure as pressure at sea level. Format = {114}
#define PDL_CMD_RESET_ALTITUDE        114
/// Sets atmospheric pressure at sea level. Format = {115, float}
#define PDL_CMD_SET_SEA_LEVEL         115
/// Sets target altitude in meters. Format = {116, float}
#define PDL_CMD_SET_ALTITUDE          116
/// Sets baseGas value and writes this value for each motor. If altitude pids are enabled it has no effect. Format = {117, int32_t}
#define PDL_CMD_SET_BASE_GAS          117
/// Sets pitchRatePid. Format = {118, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_PITCH_RATE_PID    118
/// Sets rollRatePid. Format = {119, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_ROLL_RATE_PID     119
/// Sets velocityXPid. Format = {120, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_VELOCITY_X_PID    120
/// Sets velocityYPid. Format = {121, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_VELOCITY_Y_PID    121
/// Sets velocityZPid. Format = {122, uint8_t enabled, float kp, float ki, float kd, float maxOut, float maxErrSum}
#define PDL_CMD_SET_VELOCITY_Z_PID    122
/// Sets target vertical velocity. Format = {123, float}
#define PDL_CMD_SET_VELOCITY_Z        123

/// @}

#endif
