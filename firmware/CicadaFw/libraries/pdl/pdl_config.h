#ifndef PDL_CONFIG
#define PDL_CONFIG

/// @defgroup pdlConfig Config
/// @{

#define LOG_LEVEL INFO_LEVEL

/// Number of motors. Motors are indexed from left-top in counterclockwise direction like pins of chip.
#define PDL_MOTOR_COUNT                 4
/// Number of loads (led, buzzer, drop-machine, etc)
#define PDL_LOAD_COUNT                 2
/// Accel read period in microseconds
#define PDL_ACCEL_READ_PERIOD           10000
/// Gyro read period in microseconds
#define PDL_GYRO_READ_PERIOD            10000
/// Magneto read period in microseconds
#define PDL_MAG_READ_PERIOD             10000
/// Barometer read period in microseconds
#define PDL_BARO_READ_PERIOD            53000
//#define PDL_BARO_READ_PERIOD            20000
/// Battery state read period in microseconds
#define PDL_BATTERY_READ_PERIOD         100000
/// Escaper update period in microseconds
#define PDL_ESCAPER_UPDATE_PERIOD       0
/// Remote control update period in microseconds
#define PDL_RC_UPDATE_PERIOD            0
/// Loads update period in microseconds
#define PDL_LOAD_UPDATE_PERIOD          0
/// Lidar read period in microseconds
//#define PDL_LIDAR_READ_PERIOD           20000
//#define PDL_LIDAR_READ_PERIOD           23000
//#define PDL_LIDAR_READ_PERIOD           53000
#define PDL_LIDAR_READ_PERIOD             100000
//#define PDL_LIDAR_READ_PERIOD           103000
//#define PDL_LIDAR_READ_PERIOD           123000
//#define PDL_LIDAR_READ_PERIOD           203000
/// OpticalFlow read period in microseconds (10 ms in pixhawk)
#define PDL_OPTICAL_FLOW_READ_PERIOD      20000
//#define PDL_OPTICAL_FLOW_READ_PERIOD    100000
/// GPS read period in microseconds
//#define PDL_GPS_READ_PERIOD             100000
#define PDL_GPS_READ_PERIOD               200000
/// Telemetry update period in microseconds
#define PDL_DEFAULT_TELEMETRY_UPDATE_PERIOD     10000
/// PID update period in microseconds
#define PDL_PID_UPDATE_PERIOD           10000
/// Desire time for one loop @ref task_scheduler "See task scheduler"
#define PDL_DESIRE_UPDATE_TIME          10000
/// Max wait time for task to be executed @ref task_scheduler "See task scheduler"
#define PDL_TASK_MAX_WAIT_TIME          20000
/// Maximum gas of motor. Gas means your PWM resolution, DSHOT max value, etc
#define PDL_DEFAULT_MOTOR_GAS_MAX               1999
/// Gas of motors when they have to be stopped
#define PDL_DEFAULT_MOTOR_GAS_NULL              0
/// Mininmum gas of motor. It represents maximum gas for reverse rotation
#define PDL_DEFAULT_MOTOR_GAS_MIN               0

/// The distance to ground at witch drone switch to follow terrain mode
#define PDL_FOLLOW_TERRAIN_RANGE    1.f

/// Size of buffer to store log before send to host
#define PDL_LOG_BUF_SIZE            1024

/// Lidar offset from gravity center along X-axis in meters
#define PDL_LIDAR_OFFSET            0.055f
/// @}

#endif
