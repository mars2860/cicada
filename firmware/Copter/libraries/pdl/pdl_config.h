#ifndef PDL_CONFIG
#define PDL_CONFIG

/// @defgroup pdlConfig Config
/// @{

/// Number of motors. Motors are indexed from left-top in counterclockwise direction like pins of chip.
/// At this time only quadrocopters are supported. So this param have to be always 4
#define PDL_MOTOR_COUNT                 4
/// Maximum gas of motor. Gas means your PWM resolution, DSHOT max value, etc
#define PDL_MOTOR_GAS_MAX               1999
/// Gas of motors when they have to be stopped
#define PDL_MOTOR_GAS_NULL              0
/// Mininmum gas of motor. It represents maximum gas for reverse rotation
#define PDL_MOTOR_GAS_MIN               0
/// Accel+Gyro+Magneto read period in microseconds
#define PDL_IMU_READ_PERIOD             5000
/// Barometer read period in microseconds
#define PDL_BARO_READ_PERIOD            50000
/// Battery state read period in microseconds
#define PDL_BATTERY_READ_PERIOD         100000
/// Escaper update period in microseconds
#define PDL_ESCAPER_UPDATE_PERIOD       0
/// Remote control update period in microseconds
#define PDL_RC_UPDATE_PERIOD            0
/// Lidar read period in microseconds
#define PDL_LIDAR_READ_PERIOD           50000
/// OpticalFlow read period in microseconds
#define PDL_OPTICAL_FLOW_READ_PERIOD    50000
/// Telemetry update period in microseconds
#define PDL_DEFAULT_TELEMETRY_UPDATE_PERIOD     50000
/// Desire time for one loop @ref task_scheduler "See task scheduler"
#define PDL_DESIRE_UPDATE_TIME                  5000
/// Max wait time for task to be executed @ref task_scheduler "See task scheduler"
#define PDL_TASK_MAX_WAIT_TIME                  10000
/// Maximum lidar range in meters
#define PDL_LIDAR_MAX_RANGE 3.f
/// Drone frame type. One of PDL_DRONE_FRAME_X | PDL_DRONE_FRAME_CROSS
#define PDL_DRONE_FRAME                 PDL_DRONE_FRAME_CROSS

/// @}

#endif
