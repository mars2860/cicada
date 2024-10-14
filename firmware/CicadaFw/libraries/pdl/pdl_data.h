#ifndef PDL_DATA_H
#define PDL_DATA_H

#include <stdint.h>
#include "pdl_config.h"
#include "pdl_nav_kf.h"

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

#define PDL_VERSION 3

#define PDL_X     0
#define PDL_Y     1
#define PDL_Z     2

#define PDL_ROLL    0
#define PDL_PITCH   1
#define PDL_YAW     2

#define PDL_NORTH     0
#define PDL_EAST      1
#define PDL_UP        2

/// 1G acceleration = 9.81f m/s^2
#define PDL_G       9.81f

#define PDL_PID_HEADING_FLAG     1
#define PDL_PID_VELOX_FLAG       2
#define PDL_PID_VELOY_FLAG       4
#define PDL_PID_ALT_FLAG         8
#define PDL_PID_POS_NORTH_FLAG   16
#define PDL_PID_POS_EAST_FLAG    32
#define PDL_PID_PITCH_FLAG       64
#define PDL_PID_ROLL_FLAG        128
#define PDL_PID_ALL_FLAG (PDL_PID_HEADING_FLAG | PDL_PID_VELOX_FLAG | PDL_PID_VELOY_FLAG | PDL_PID_ALT_FLAG | PDL_PID_POS_NORTH_FLAG | PDL_PID_POS_EAST_FLAG | PDL_PID_PITCH_FLAG | PDL_PID_ROLL_FLAG)

#define PDL_TRICK_MODE_DISABLED   0
#define PDL_TRICK_MODE_ACRO       10
#define PDL_TRICK_MODE_GYRO       20

#define PDL_OSD_BAT_STATE         1
#define PDL_OSD_RSSI              (1<<1)
#define PDL_OSD_FLY_TIME          (1<<2)
#define PDL_OSD_YAW               (1<<3)
#define PDL_OSD_PITCH             (1<<4)
#define PDL_OSD_ROLL              (1<<5)
#define PDL_OSD_ALT               (1<<6)
#define PDL_OSD_HOME              (1<<7)
#define PDL_OSD_LAT               (1<<8)
#define PDL_OSD_LON               (1<<9)
#define PDL_OSD_SATTELS           (1<<10)
#define PDL_OSD_VSPD              (1<<11)
#define PDL_OSD_HSPD              (1<<12)

/// Battery state takes 24 bytes
typedef struct s_pdlBatteryState
{
  float voltage;
  float percent;
  float current;
  float capacity;
  float voltScaler;
  float curnScaler;
  float minVoltage;
  float maxVoltage;
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

/// Optical flow state takes 36 bytes
typedef struct s_pdlOpticalFlowSate
{
  /// Sensor raw X data
  int16_t rawX;
  /// Sensor raw Y data
  int16_t rawY;
  /// Surface quality
  uint32_t squal;
  /// Compensated by tilt X-value
  float pureX;
  /// Compensated by tilt Y-value
  float pureY;
  /// Velocity in m/s along drone course (positive when drone moves ahead).
  float velX;
  /// Velocity in m/s perpendicular of drone course (positive when drone moves right).
  float velY;
  /// Velocity in m/s along north. It is used by NavKalman
  float velNorth;
  /// Velocity in m/s along east. It is used by NavKalman
  float velEast;
  /// Sums of x in meters. It is used just for testing and debuging optical flow. This value is reseted when motors get enabled
  float sumX;
  /// Sums of y in meters. It is used just for testing and debuging optical flow. This value is reseted when motors get enabled
  float sumY;
} pdlOpticalFlowState;

/// Lidar state takes 12 bytes
typedef struct s_pdlLidarState
{
  /// Distance to ground (or obstacle) in meters. If value is less 0 that means there is error in lidar system
  float range;
  /// Vertical velocity in m/s
  float velZ;
} pdlLidarState;

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

/// Triple axis sensor state takes 28 bytes
typedef struct s_pdlGyroState
{
  /// digital low pass filter mode, use this value to setup dlpf inside your pdlSetupGyro
  uint32_t dlpf;
  /// Values converted to radians and converted to PDL body coordinate system
  float pure[3];
  /// Body rotation speed measured by gyro and converted to Euler rotation speed in rad
  float eulerRate[3];
  /// Raw values
  int16_t raw[3];
  /// Calibration offsets (Depends from your sensor)
  int16_t offset[3];
} pdlGyroState;

/// Accel sensor state takes 40 bytes
typedef struct s_pdlAccelState
{
  /// digital low pass filter mode, use this value to setup dlpf inside your pdlSetupAccel
  uint32_t dlpf;
  /// Values converted to m/s^2 and converted to PDL body coordinate system
  float pure[3];
  /// Linear accels in m/s^2 in world frame
  float world[3];
  /// Pitch angle determined by accel in rad
  float pitch;
  /// Roll angle determined by accel in rad
  float roll;
  /// Raw values
  int16_t raw[3];
  /// Calibration offsets (Depends from your sensor)
  int16_t offset[3];
} pdlAccelState;

/// Magneto sensor state takes 40 bytes
typedef struct s_pdlMagnetoState
{
  /// Heading in rad determined by magneto measurements with tilt compensation
  float heading;
  /// Magnetic declination in rad when motors off
  float declination;
  /// Additional course correction when motors on
  float inflightCorrection;
  /// Values converted to mGauss and converted to PDL body coordinate system
  float pure[3];
  /// Soft-iron compensation values
  float scale[3];
  /// Raw values
  int16_t raw[3];
  /// Hard-iron compensation values
  int16_t offset[3];
} pdlMagnetoState;

/// GPS state takes 56 bytes
typedef struct s_pdlGpsState {
  uint8_t enabled;
  /**
     GNSSfix Type, range 0..5
     0x00 = No Fix
     0x01 = Dead Reckoning only
     0x02 = 2D-Fix
     0x03 = 3D-Fix
     0x04 = GNSS + dead reckoning combined
     0x05 = Time only fix
     0x06..0xff: reserved
  */
  uint8_t fixType;
  /// Number of satellites used in Nav Solution
  uint16_t numSV;
  /// Current latitude as DD.DDDDDD
  float curLat;
  /// Current longitude as DD.DDDDDD
  float curLon;
  /// Current altitude above mean sea level in meters
  float curAlt;
  /// start latitude as DD.DDDDDD
  float startLat;
  /// start longitude as DD.DDDDDD
  float startLon;
  /// start altitude above mean sea level in meters
  float startAlt;
  /// true heading
  float course;
  /// North velocity in m/s
  float velN;
  /// East velocity in m/s
  float velE;
  /// Up velocity in m/s
  float velU;
  /// Horizontal accuracy in meters
  float hAcc;
  /// Vertical accuracy in meters
  float vAcc;
  /// Speed accuracy in m/s
  float sAcc;
  /// Distance in meters from start point in North direction (calc by PDL)
  float posNorth;
  /// Distance in meters from start point in East direction (calc by PDL)
  float posEast;
  /// Distance in meters from start point in Up direction (calc by PDL)
  float posUp;

} pdlGpsState;

/// Kalman Filter settings takes 24 bytes
typedef struct s_pdlKalmanSettings
{
  /// Navigation model uncertainty
  float navModelNoise;
  /// The uncertainty of linear accel in world
  float accVariance;
  /// The uncertainty of altitude measured by baro
  float baroAltVariance;
  /// The uncertainty of vertical velocity measured by lidar when horizontal velocity below 0.3 m/s
  float lidarVelVariance1;
  /// The uncertainty of horizontal velocity measured by optical flow at 0.3m above ground
  float ofVelVariance1;
  /// The uncertainty of horizontal velocity measured by optical flow at 0.9m above ground
  float ofVelVariance2;
  /// The uncertainty of vertical velocity measured by lidar when horizontal velocity above 0.3 m/s
  float lidarVelVariance2;
  /// Pose model uncertainty
  float poseModelNoise;
  /// The uncertainty of yaw angular rate measured by gyroscope
  float yawRateVariance;
  /// The uncertainty of pitch/roll angular rate measured by gyroscope
  float pitchRollRateVariance;
  /// The uncertainty of heading measured by magnetometer
  float magHeadingVariance;
  /// The uncertainty of pitch/roll measured by accelerometer
  float accPitchRollVariance;
  /// The uncertainty of horizontal speed from GPS
  float gpsHorSpeedVariance;
  /// The uncertainty of vertical speed from GPS
  float gpsVerSpeedVariance;
  /// The uncertainty of horizontal position from GPS
  float gpsHorPosVariance;
  /// The uncertainty of vertical position from GPS
  float gpsVerPosVariance;
} pdlKalmanSettings;

/// Load state takes 8 bytes
typedef struct s_pdlLoadState
{
  uint8_t enabled;
  /// Current load state
  uint8_t state;
  /// Load switch on/off time in ms
  uint16_t period;
  /// Last system time when load is switched
  uint32_t timestamp;
} pdlLoadState;

/// Drone state takes 1008 bytes
typedef struct s_pdlDroneState
{
  /// pdl version
  uint8_t version;
  /// Motors enabled
  uint8_t motorsEnabled;
  /// Global flag to enable PIDs
  uint8_t stabilizationEnabled;
  /// MASK of macro PDL_PID_FLAG. Controls applying of pid to input of other pid in pid-chain (see pid-chain image)
  uint8_t pidFlags;
  /// ESC mode (DSHOT, PWM, etc)
  uint8_t esc;
  /// Frame type (Cross, X, Dead cat, etc)
  uint8_t frame;
  /// Video record state
  uint8_t videoState;
  /// Trick Mode
  uint8_t trickMode;
  /// System errors. And-mask of error flags. The flags are depended from your platform
  uint32_t errors;
  /// Base gas. This value is output of altitude PID. If altitude PID is disabled you have to control this value yourself
  int32_t baseGas;
  /// Current time in microseconds from system has been started
  uint32_t timestamp;
  /// Battery state
  pdlBatteryState battery;
  /// RemoteControl system state
  pdlRcState rc;
  /// Accelerometer state. Pure values have to be in G units
  pdlAccelState accel;
  /// Gyroscope state. Pure values have to be in rad/s units
  pdlGyroState gyro;
  /// Magnetometer state. Pure values have to be in mGauss
  pdlMagnetoState magneto;
  /// Time between the executing of one loop in microseconds
  uint32_t loopPeriod;
  /// Executing time of imu task
  uint32_t gyroTaskTime;
  /// Executing time of ahrs task
  uint32_t accelTaskTime;
  /// Executing time of mag task
  uint32_t magTaskTime;
  /// Executing time of lidar task
  uint32_t lidarTaskTime;
  /// Executing time of baro task
  uint32_t baroTaskTime;
  /// Executing time of opticalFlow task
  uint32_t ofTaskTime;
  /// Executing time of rc task
  uint32_t rcTaskTime;
  /// Executing time of battery task
  uint32_t batteryTaskTime;
  /// Executing time of telemetry task
  uint32_t telemetryTaskTime;
  /// Executing time of esc task
  uint32_t escTaskTime;
  /// Executing time of pid task
  uint32_t pidTaskTime;
  /// Executing time of gps task
  uint32_t gpsTaskTime;
  /// Executing time of load task
  uint32_t loadTaskTime;
  /// Yaw angular rate target in rad
  /// We need this variable because bodyRatePid works in body frame
  /// but all levels PID output angular rate in world frame
  float yawRateTarget;
  /// Pitch angular rate target in rad
  float pitchRateTarget;
  /// Roll angular rate target in rad
  float rollRateTarget;
  /// Temperature of air
  float temperature;
  /// Barometer state
  pdlBaroState baro;
  /// OpticalFlow state
  pdlOpticalFlowState opticalFlow;
  /// Lidar state
  pdlLidarState lidar;
  /// The linear motion description of drone in the world frame
  pdlNavState nav[3];
  /// The angular motion description of drone
  pdlNavState pose[3];
  /// Angular Rate PID around Z axis in body frame
  pdlPidState zRatePid;
  /// Angular Rate PID around Y axis in body frame
  pdlPidState yRatePid;
  /// Angular Rate PID around X axis in body frame
  pdlPidState xRatePid;
  /// Pitch PID
  pdlPidState pitchPid;
  /// Roll PID
  pdlPidState rollPid;
  /// Altitude PID
  pdlPidState altPid;
  /// VelocityX PID. Controls velocity in world frame rotated by heading
  pdlPidState velocityXPid;
  /// VelocityY PID. Controls velocity in world frame rotated by heading
  pdlPidState velocityYPid;
  /// VelocityZ PID. Controls vertical velocity in world frame
  pdlPidState velocityZPid;
  /// Magnetic heading PID
  pdlPidState headingPid;
  /// Pos along North PID. Controls position in world frame along North
  pdlPidState posNorthPid;
  /// Pos along East PID. Controls position in world frame along East
  pdlPidState posEastPid;
  /// Kalman filter settings
  pdlKalmanSettings kfSettings;
  /// GPS state
  pdlGpsState gps;
  /// Yaw from other AHRS to compare with used AHRS (this value is not used by PDL)
  float refYaw;
  /// Pitch from other AHRS to compare with used AHRS (this value is not used by PDL)
  float refPitch;
  /// Roll from other AHRS to compare with used AHRS (this value is not used by PDL)
  float refRoll;
  /// Additional offset for accel.up. It can be used in systems have bad accelerometer
  float accUpOffset;
  /// Telemetry period
  uint32_t telemetryPeriod;
  /// OSD (on screen display) flags. PDL doesn't implement drawing OSD, but you can use this field in your implementation to know what data to draw on the screen
  uint32_t osd;
  /// Reserved5
  uint32_t reserved5;
  /// Reserved6
  uint32_t reserved6;
  /// Motors gas. These values are intended to motor regulator. Gas means your PWM value, DSHOT value, etc.
  int32_t motorGas[PDL_MOTOR_COUNT];
  /// Loads (led,buzzer,drop-machine,etc)
  pdlLoadState load[PDL_LOAD_COUNT];

} pdlDroneState;

/// @}

#endif
