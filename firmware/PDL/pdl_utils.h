#ifndef PDL_UTILS_H
#define PDL_UTILS_H

#include <stdint.h>

typedef struct s_pdlVector3
{
  float x;
  float y;
  float z;
} pdlVector3;

/// @return (cur - last) with overflow correction
uint32_t pdlGetDeltaTime(uint32_t cur, uint32_t last);

/** Converts pixels from optical flow sensor to meters.
 *  Implementation of <a href="https://ardupilot.org/copter/docs/common-mouse-based-optical-flow-sensor-adns3080.html">this algorithm</a>
 *
 * @param pixels  Pixels returned by optical flow sensors
 * @param range   Distance to ground in meters (You can provide it by sonar or lidar)
 * @param angDiff Angle difference along this axis in radians (in other words it is angular rate * delta time).
 *                For roll put inverted angDiff (in other words for roll put -angDiff)
 * @param fov     Field of view of sensor lens (see in datasheet)
 * @param npx     Optical matrix size in pixels
 * @param scaler  Magic number determined from experiment
 */
float pdlOpticalFlowToMeters(float pixels, float range, float angDiff, float fov, float npx, float scaler);


/** Estimates pixels from rotation of optical flow sensor
 *
 * @param range   Distance to ground in meters (You can provide it by sonar or lidar)
 * @param angDiff Angle difference along this axis in radians (in other words it is angular rate * delta time).
 *                For roll put inverted angDiff (in other words for roll put -angDiff)
 * @param fov     Field of view of sensor lens (see in datasheet)
 * @param npx     Optical matrix size in pixels
 * @param scaler  Magic number determined from experiment
 */
float pdlOpticalFlowTiltPixels(float angDiff, float fov, float npx, float scaler);

/** Calculates distance in meters between two geo-points by haversine formula
 *
 * @param lat1 WGS84 latitude of first point in degrees
 * @param lon1 WGS84 longitude of first point in degrees
 * @param lat2 WGS84 latitude of second point in degrees
 * @param lon2 WGS84 longitude of second point in degrees
 */
float pdlGpsDistance(float lat1, float lon1, float lat2, float lon2);

/** Calculates bearing in radians between two geo-points
 *
 * @param lat1 WGS84 latitude of first point in degrees
 * @param lon1 WGS84 longitude of first point in degrees
 * @param lat2 WGS84 latitude of second point in degrees
 * @param lon2 WGS84 longitude of second point in degrees
 */
float pdlGpsBearing(float lat1, float lon1, float lat2, float lon2);

/** Calculates distance in meters along north by known bearing and distance along bearing
 *
 * @param distance Distance in meters
 * @param bearing True heading in radians
 */
float pdlGpsNorthDistance(float distance, float bearing);

/** Calculates distance in meters along east by known bearing and distance along bearing
 *
 * @param distance Distance in meters
 * @param bearing True heading in radians
 */
float pdlGpsEastDistance(float distance, float bearing);

/** Converts heading from radians to degrees */
float pdlHeadingToDegrees(float radians);

/** Converts coordinates from body frame to world frame
 * @note yaw,pitch,roll have to be in radians
 */
pdlVector3 pdlBodyToWorld(float yaw, float pitch, float roll, float x, float y, float z);

/** Converts coordinates from world frame to body frame
 * @note yaw,pitch,roll have to be in radians
 */
pdlVector3 pdlWorldToBody(float yaw, float pitch, float roll, float x, float y, float z);

/** Converts body angular rate to euler angular rate. In other words, gyroscope to euler
 * @note yaw,pitch,roll have to be in radians
 * @note gx,gy,gz have to be converted to PDL coordinate system
 * @return euler angular rates in radians, x = rollRate, y = pitchRate, z = yawRate
 */
pdlVector3 pdlBodyRateToEulerRate(float yaw, float pitch, float roll, float gx, float gy, float gz);

/** Converts euler angular rate to body angular rate
 * @note all params have to be in radians
 * @return angular rates in radians in body frame
 */
pdlVector3 pdlEulerRateToBodyRate(float yaw, float pitch, float roll, float yawRate, float pitchRate, float rollRate);

/** Converts acceleration in body frame to euler angles
 * @note ax,ay,az have to be converted to PDL coordinate system
 * @return euler angles in radians, range is from -M_PI to M_PI
 */
pdlVector3 pdlEulerFromAcc(float ax, float ay, float az);

/** Constrains value in given range */
void pdlConstrain(float *value, float minVal, float maxVal);

/** Converts magnetometer data to magnetic heading
 * @note pitch, roll have to be in radians
 * @note mx,my,mz have to be converted to PDL coordinate system
 * @return heading in radians, range is from 0 to M_2_PI
 */
float pdlHeadingFromMagneto(float pitch, float roll, float mx, float my, float mz);

/** Converts battery voltage to battery percent using
 * Symmetric sigmoidal approximation
 * https://www.desmos.com/calculator/7m9lu26vpy
 *
 * c - c / (1 + k*x/v)^3
 */
float pdlBatteryPercentSigmoidal(float voltage, float minVoltage, float maxVoltage);

/** Converts battery voltage to battery percent using
 * Asymmetric sigmoidal approximation
 * https://www.desmos.com/calculator/oyhpsu8jnw
 *
 * c - c / [1 + (k*x/v)^4.5]^3
 */
float pdlBatteryPercentAsigmoidal(float voltage, float minVoltage, float maxVoltage);

/** Converts battery voltage to battery percent using
 * Linear mapping
 * https://www.desmos.com/calculator/sowyhttjta
 *
 * x * 100 / v
 */
float pdlBatteryPercentLinear(float voltage, float minVoltage, float maxVoltage);

/** Simple Median filter for buffer has size 3 */
float pdlMedian3(float a, float b, float c);

/** Median filter
 * @param buf data buffer
 * @param size count of elements of data buffer
 */
float pdlMedian(const float *buf, uint8_t size);

/** Complementary filter to estimate Euler angles
 *
 * @param tau         Filter gain. It is recommended to set 0.98
 * @param prevAngle   Estimated before Euler angle in radians
 * @param rate        Euler angle rotation speed in rad/s. @see pdlBodyRateToEuler to convert body rate from gyroscope sensor to Euler rate
 * @param refAngle    Referenced Euler angle in radians. @see pdlEulerFromAcc to convert data from accelerometer sensor to Euler angle
 * @param dt          Delta time in seconds
 * @return
 */
float pdlComplementaryFilter(float tau, float prevAngle, float rate, float refAngle, float dt);

#endif
