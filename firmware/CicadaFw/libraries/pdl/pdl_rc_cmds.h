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
/// Sets accel offsets and invokes pdlSetupAccel. Format = {102, int16_t, int16_t, int16_t, uint8_t}
#define PDL_CMD_SET_ACCEL             102
/// Sets gyro offsets and invokes pdlSetupGyro. Format = {103, int16_t, int16_t, int16_t, uint8_t}
#define PDL_CMD_SET_GYRO              103
/// Sets magneto offsets and invokes pdlSetupMagneto. Format = {104, int16_t, int16_t, int16_t, float, float, float}
#define PDL_CMD_SET_MAGNETO           104
/// Invokes pdlCalibrateAccel. Format = {105}
#define PDL_CMD_SELF_CALIB_ACCEL      105
/// Invokes pdlCalibrateGyro. Format = {106}
#define PDL_CMD_SELF_CALIB_GYRO       106
/// Sets yawRatePid. Format = {107, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_YAW_RATE_PID      107
/// Sets pitchPid. Format = {108, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_PITCH_PID         108
/// Sets rollPid. Format = {109, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_ROLL_PID          109
/// Sets altPid. Format = {110, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_ALT_PID           110
/// @Deprecated Sets targets for yawRatePid,pitchPid,rollPid. Format = {111, float targetYawRate, float targetPitch, float targetRoll}
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
/// Sets pitchRatePid. Format = {118, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_PITCH_RATE_PID    118
/// Sets rollRatePid. Format = {119, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_ROLL_RATE_PID     119
/// Sets velocityXPid. Format = {120, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_VELOCITY_X_PID    120
/// Sets velocityYPid. Format = {121, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_VELOCITY_Y_PID    121
/// Sets velocityZPid. Format = {122, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_VELOCITY_Z_PID    122
/// Sets target vertical velocity. Format = {123, float}
#define PDL_CMD_SET_VELOCITY_Z        123
/// Sets heading PID. {124, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_HEADING_PID       124
/// Sets target velocity-x. Format = {125, float}
#define PDL_CMD_SET_VELOCITY_X        125
/// Sets target velocity-y. Format = {126, float}
#define PDL_CMD_SET_VELOCITY_Y        126
/// Sets target for heading. Format = {127, float}
#define PDL_CMD_SET_HEADING           127
/// Sets target for pitch. Format = {128, float}
#define PDL_CMD_SET_PITCH             128
/// Sets target for roll. Format = {129, float}
#define PDL_CMD_SET_ROLL              129
/// Sets target for yawRate. Format = {130, float}
#define PDL_CMD_SET_YAW_RATE               130
/// Sets user data. Format = {131, float[12]}
#define PDL_CMD_SET_USER_DATA1        131
/// Sets posXPid. Format = {132, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_POS_NORTH_PID          132
/// Sets posXPid. Format = {133, uint8_t enabled, float kp, float ki, float kd, float maxOut}
#define PDL_CMD_SET_POS_EAST_PID          133
/// Sets target for pitchRate. Format = {134, float}
#define PDL_CMD_SET_PITCH_RATE        134
/// Sets target for rollRate. Format = {135, float}
#define PDL_CMD_SET_ROLL_RATE         135
/// Sets Kalman filter covariances. Format = {136, float,float,float,float,float,float}
#define PDL_CMD_SET_KALMAN            136
/// Sets Battery scalers. Format = {137,float,float}
#define PDL_CMD_SET_BATTERY           137
/// Sets GPS. Format = {138,uin8_t}
#define PDL_CMD_SET_GPS               138
/// Enables/Disables trick mode, Format = {139, uint8_t enabled}
#define PDL_CMD_ENABLE_TRICK_MODE     139
/// Set load, Format = {140, uint8_t num, uint8_t enabled, uint16_t freq}
#define PDL_CMD_ENABLE_LOAD           140
/// Set ESC mode, Format = {141, uint8_t mode}
#define PDL_CMD_SET_ESC               141
/// Set Frame type, Format = {142, uint8_t mode}
#define PDL_CMD_SET_FRAME             142
/// Set new Wifi SSID, Format = {143, char*}
#define PDL_CMD_SET_SSID              143
/// Set new Wifi PSK, Format = {144, char*}
#define PDL_CMD_SET_PSK               144
/// Set new Wifi IP, Format = {145, char*}
#define PDL_CMD_SET_IP                145
/// Set new Wifi Gateway, Format = {146, char*}
#define PDL_CMD_SET_GATEWAY           146
/// Set new Wifi Subnet, Format = {147, char*}
#define PDL_CMD_SET_SUBNET            147
/// Enables stabilization and set error sum for VeloZPid, this allows fast takeoff when veloZPid is enabled, Format = {148, float errSum}
#define PDL_CMD_VELOZ_TAKEOFF         148
/// Set motor spin direction, Format = {149, uint8_t dir}
#define PDL_CMD_SET_MOTORS_DIR        149
/// Save current config as default, Format = {150}
#define PDL_CMD_SAVE_DEFAULT_CFG      150
/// Load default config, Format = {151}
#define PDL_CMD_LOAD_DEFAULT_CFG      151
/// Set camera angle, Format = {152,int32_t}
#define PDL_CMD_SET_CAMERA_ANGLE      152
/// Start video/stop video, Format = {153, uint8_t start}
#define PDL_CMD_START_STOP_VIDEO      153
/// Take a photo, Format = {154}
#define PDL_CMD_TAKE_PHOTO            154
/// Enable dynamic ip, Format = {155, uint8_t en}
#define PDL_CMD_ENABLE_DYNAMIC_IP     155
/// Set accUpOffset, Format = {156, float accUpOffset}
#define PDL_CMD_SET_ACCUP_OFFSET      156
/// Sets target for zRate. Format = {157, float}
#define PDL_CMD_SET_Z_RATE            157
/// Sets target for xRate. Format = {158, float}
#define PDL_CMD_SET_X_RATE            158
/// Sets target for yRate. Format = {159, float}
#define PDL_CMD_SET_Y_RATE            159

/// @}

#endif
