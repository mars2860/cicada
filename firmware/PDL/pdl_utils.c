#include <math.h>
#include <string.h>
#include "pdl_utils.h"

uint32_t pdlGetDeltaTime(uint32_t cur, uint32_t last)
{
  if(cur >= last)
    return cur - last;

  return UINT32_MAX - last + cur;
}

float pdlOpticalFlowToMeters(float pixels, float range, float angDiff, float fov, float npx, float scaler)
{
    float K1;
    float K2;
    float K3;

    K1 = npx * scaler;
    K2 = K1/fov;
    K3 = (2.f * tanf(fov / 2.f)) / K1;

    float tilt_compensation = angDiff * K2;
    pixels -= tilt_compensation;

    return pixels * range * K3; // height compensation
}

float pdlOpticalFlowTiltPixels(float angDiff, float fov, float npx, float scaler)
{
  float K1;
  float K2;

  K1 = npx * scaler;
  K2 = K1/fov;

  return angDiff * K2;
}

float pdlGpsDistance(float lat1, float lon1, float lat2, float lon2)
{
    //static const float R = 6371000.f;     // Earth radius in meters
    static const float R = 6378137.f;
    float phi1 = lat1 * M_PI/180.f;
    float phi2 = lat2 * M_PI/180.f;
    float sin_delta_phi = sinf((phi2 - phi1)/2.f);
    float sin_delta_lambda = sinf(((lon2-lon1)/2.f)*M_PI/180.f);

    float a = sin_delta_phi*sin_delta_phi + cosf(phi1)*cosf(phi2)*sin_delta_lambda*sin_delta_lambda;
    float c = 2.f*atan2f(sqrtf(a),sqrtf(1.f-a));
    float d = R * c;
    return d;
}

float pdlGpsBearing(float lat1, float lon1, float lat2, float lon2)
{
    float phi1 = lat1 * M_PI/180.f;
    float phi2 = lat2 * M_PI/180.f;
    float delta_lambda = (lon2-lon1)*M_PI/180.f;

    float y = sinf(delta_lambda)*cosf(phi2);
    float x = cosf(phi1)*sinf(phi2) - sinf(phi1)*cosf(phi2)*cosf(delta_lambda);
    float t = atan2f(y, x);

    return t;
}

float pdlGpsNorthDistance(float distance, float bearing)
{
    return distance * cosf(bearing);
}

float pdlGpsEastDistance(float distance, float bearing)
{
    return distance * sinf(bearing);
}

float pdlHeadingToDegrees(float radians)
{
    float degrees = radians * 180.f / M_PI;

    if(degrees < 0.f)
        degrees += 360.f;
    else if(degrees > 360.f)
        degrees -= 360.0;

    return degrees;
}

pdlVector3 pdlBodyToWorld(float yaw, float pitch, float roll, float i, float j, float k)
{
  /*
  (j*sin(p)*sin(r)*cos(y) - j*cos(r)*sin(y) + k*sin(p)*cos(r)*cos(y) + k*sin(r)*sin(y)) + i*cos(p)*cos(y)
  (j*sin(p)*sin(r)*sin(y) + j*cos(r)*cos(y) + k*sin(p)*cos(r)*sin(y) - k*sin(r)*cos(y)) + i*cos(p)*sin(y)
  (j*cos(p)*sin(r) + k*cos(p)*cos(r)) - i*sin(p)

  a = sin(p)
  b = cos(p)
  c = sin(r)
  d = cos(r)
  e = sin(y)
  f = cos(y)

  (j*c*a*f - j*d*e + k*d*a*f + k*c*e) + i*b*f
  (j*c*a*e + j*d*f + k*d*a*e - k*c*f) + i*b*e
  (j*c*b + k*d*b) - i*a

  u = i*b
  v = k*c
  w = k*d
  m = j*c
  n = j*d

  (m*a*f - n*e + w*a*f + v*e) + u*f
  (m*a*e + n*f + w*a*e - v*f) + u*e
  (m*b + w*b) - i*a

  s = m*a
  t = w*a

  (s*f - n*e + t*f + v*e) + u*f
  (s*e + n*f + t*e - v*f) + u*e
  (m*b + w*b) - i*a
  */
  pdlVector3 res;

  float a = sinf(pitch);
  float b = cosf(pitch);
  float c = sinf(roll);
  float d = cosf(roll);
  float e = sinf(yaw);
  float f = cosf(yaw);

  float u = i*b;
  float v = k*c;
  float w = k*d;
  float m = j*c;
  float n = j*d;

  float s = m*a;
  float t = w*a;

  res.x = (s*f - n*e + t*f + v*e) + u*f;
  res.y = (s*e + n*f + t*e - v*f) + u*e;
  res.z = (m*b + w*b) - i*a;

  return res;
}

pdlVector3 pdlWorldToBody(float yaw, float pitch, float roll, float i, float j, float k)
{
  /*
   j*cos(p)*sin(y) - k*sin(p) + i*cos(p)*cos(y)
   j*cos(y)*cos(r) + j*sin(p)*sin(y)*sin(r) + k*cos(p)*sin(r) + i*(-sin(y)*cos(r) + sin(p)*cos(y)*sin(r))
   j*sin(p)*sin(y)*cos(r) - j*cos(y)*sin(r) + k*cos(p)*cos(r) + i*(sin(y)*sin(r) + sin(p)*cos(y)*cos(r))

   a = sin(p)
   b = cos(p)
   c = sin(r)
   d = cos(r)
   e = sin(y)
   f = cos(y)

   j*b*e - k*a + i*b*f
   j*f*d + j*a*e*c + k*b*c -i*e*d + i*a*f*c
   j*a*e*d - j*f*c + k*b*d + i*e*c + i*a*f*d

   u = j*e
   v = j*f
   w = i*e
   m = i*a*f
   n = k*b

   u*b - k*a + i*b*f
   v*d + u*a*c + n*c - w*d + m*c
   u*a*d - v*c + n*d + w*c + m*d

   s = u*a

   u*b - k*a + i*b*f
   v*d + s*c + n*c - w*d + m*c
   s*d - v*c + n*d + w*c + m*d

   */

   pdlVector3 res;

   float a = sinf(pitch);
   float b = cosf(pitch);
   float c = sinf(roll);
   float d = cosf(roll);
   float e = sinf(yaw);
   float f = cosf(yaw);

   float u = j*e;
   float v = j*f;
   float w = i*e;
   float m = i*a*f;
   float n = k*b;

   float s = u*a;

   res.x = u*b - k*a + i*b*f;
   res.y = v*d + s*c + n*c - w*d + m*c;
   res.z = s*d - v*c + n*d + w*c + m*d;

   return res;
}

pdlVector3 pdlBodyRateToEulerRate(float yaw, float pitch, float roll, float gx, float gy, float gz)
{
    // Explanation https://www.youtube.com/watch?v=nEJ9bA3M3V0

    pdlVector3 result;

    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float tan_pitch = tanf(pitch);
    float cos_pitch = cosf(pitch);

    result.x = gx + gy*sin_roll*tan_pitch + gz*cos_roll*tan_pitch;
    result.y = gy*cos_roll - gz*sin_roll;
    result.z = gy*sin_roll/cos_pitch + gz*cos_roll/cos_pitch;

    (void)yaw;

    return result;
}

pdlVector3 pdlEulerRateToBodyRate(float yaw, float pitch, float roll, float yawRate, float pitchRate, float rollRate)
{
    pdlVector3 result;

    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);

    result.x = rollRate - yawRate*sin_pitch;
    result.y = pitchRate*cos_roll + yawRate*sin_roll*cos_pitch;
    result.z = yawRate*cos_roll*cos_pitch - pitchRate*sin_roll;

    (void)yaw;

    return result;
}

pdlVector3 pdlEulerFromAcc(float ax, float ay, float az)
{
    pdlVector3 result;

    result.x = atan2f(-ay,-az);
    result.y = atan2f(ax,sqrtf(ay*ay + az*az));
    result.z = 0;   // it is impossible to obtain yaw angle from accel data

    //r = sqrt(x*x+y*y+z*z);
    //pitch  = asin(x/r);
    //roll   = -asin(y/r);

    return result;
}

void pdlConstrain(float *value, float minVal, float maxVal)
{
    float range = fabsf(maxVal - minVal);

    while(*value < minVal)
        *value += range;

    while(*value > maxVal)
        *value -= range;
}

float pdlHeadingFromMagneto(float pitch, float roll, float mx, float my, float mz)
{
    // references
    // https://www.infineon.com/dgdl/Infineon-AN2272_PSoC_1_Sensing_Magnetic_Compass_with_Tilt_Compensation-ApplicationNotes-v04_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0731b0d05573
    // https://www.best-microcontroller-projects.com/magnetometer-tilt-compensation.html
    // https://danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part1.html

    // we have to rotate magneto data from body frame to world frame
    // to reverse a rotation (that is, to return a rotated point to its
    // original coordinate in the reference frame), you simply reverse
    // the order of the rotations, and also change the signs of the three
    // rotation angles. So if the forward rotation is yaw(w), pitch(v), roll(u)
    // then the inverse rotation is roll(-u), pitch(-v), yaw(-w).
    float heading;
    float cos_roll = cosf(roll);
    float sin_roll = sinf(roll);
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);

    float x = mx*cos_pitch + my*sin_pitch*sin_roll + mz*sin_pitch*cos_roll;
    float y = my*cos_roll - mz*sin_roll;

    if(x == 0.f && y < 0.f)
      heading = M_PI_2;
    else if(x == 0.f && y > 0.f)
      heading = M_PI_2 * 3.f;
    else if(x > 0.f && y > 0.f)
      heading = M_TWOPI - atanf(y/x);
    else if(x > 0.f && y < 0.f)
      heading = -atanf(y/x);
    else
      heading = M_PI - atanf(y/x);

    return heading;
}

float pdlBatteryPercentSigmoidal(float voltage, float minVoltage, float maxVoltage)
{
  // slow
  // uint8_t result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

  // steep
  // uint8_t result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

  // normal
  float result = 105.f - (105.f / (1.f + powf(1.724f * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5f)));
  return result >= 100.f ? 100.f : result;
}

float pdlBatteryPercentAsigmoidal(float voltage, float minVoltage, float maxVoltage)
{
  float result = 101.f - (101.f / powf(1.f + powf(1.33f * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5f), 3.f));
  return result >= 100.f ? 100.f : result;
}

float pdlBatteryPercentLinear(float voltage, float minVoltage, float maxVoltage)
{
  return (voltage - minVoltage) * 100.f / (maxVoltage - minVoltage);
}


float pdlMedian3(float a, float b, float c)
{
  if ((a <= b) && (a <= c))
    return (b <= c) ? b : c;
  if ((b <= a) && (b <= c))
    return (a <= c) ? a : c;
  return (a <= b) ? a : b;
}

float pdlMedian(const float *buf, uint8_t size)
{
  if(size < 3 || size > 16)
    return 0;

  uint8_t i,j;
  float data[16];
  float val;

  memcpy(data,buf,size*sizeof(float));

  for(i = 0; i < size; i++)
  {
    val = data[i];
    for(j = i; j < size; j++)
    {
      if(data[j] < val)
      {
        val = data[i];
        data[i] = data[j];
        data[j] = val;
        val = data[i];
      }
    }
  }

  if(size % 2 == 0)
  {
    val = (data[size/2] + data[size/2 - 1])/2.f;
  }
  else
  {
    val = data[size/2];
  }

  return val;
}

float pdlComplementaryFilter(float tau, float prevAngle, float rate, float refAngle, float dt)
{
  return tau*(prevAngle + rate*dt) + (1.f-tau)*refAngle;
}
