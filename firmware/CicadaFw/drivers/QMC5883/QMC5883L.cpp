#include <Wire.h>
#include <math.h>
#include "QMC5883L.h"

/*
 * QMC5883L
 * http://wiki.epalsite.com/images/7/72/QMC5883L-Datasheet-1.0.pdf
 */

/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

static void write_register(int addr, int reg, int value)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static int read_register(int addr, int reg, int count)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(addr, count);
  int n = Wire.available();
  if(n != count)
    return 0;

  return n;
}

void QMC5883L::reconfig()
{
  write_register(addr, QMC5883L_CONFIG, oversampling | range | rate | mode);
}

void QMC5883L::reset()
{
  write_register(addr, QMC5883L_RESET, 0x01);
  reconfig();
}

void QMC5883L::setOversampling(int x)
{
  switch(x)
  {
  case 512:
    oversampling = QMC5883L_CONFIG_OS512;
    break;
  case 256:
    oversampling = QMC5883L_CONFIG_OS256;
    break;
  case 128:
    oversampling = QMC5883L_CONFIG_OS128;
    break;
  case 64:
    oversampling = QMC5883L_CONFIG_OS64;
    break;
  }
  reconfig();
}

void QMC5883L::setRange(int x)
{
  switch(x)
  {
  case 2:
    range = QMC5883L_CONFIG_2GAUSS;
    break;
  case 8:
    range = QMC5883L_CONFIG_8GAUSS;
    break;
  }
  reconfig();
}

void QMC5883L::setSamplingRate(int x)
{
  switch(x)
  {
  case 10:
    rate = QMC5883L_CONFIG_10HZ;
    break;
  case 50:
    rate = QMC5883L_CONFIG_50HZ;
    break;
  case 100:
    rate = QMC5883L_CONFIG_100HZ;
    break;
  case 200:
    rate = QMC5883L_CONFIG_200HZ;
    break;
  }
  reconfig();
}

bool QMC5883L::init(  uint8_t rt,
                      uint8_t rng,
                      uint8_t ovr)
{
  /* This assumes the wire library has been initialized. */
  addr = QMC5883L_ADDR;
  mode = QMC5883L_CONFIG_CONT;
  oversampling = ovr;
  range = rng;
  rate = rt;

  Wire.beginTransmission(addr);
  uint8_t err = Wire.endTransmission();

  if(err)
    return false;

  if(read_register(addr, QMC5883L_CHIP_ID, 1))
  {
    uint8_t chipId = Wire.read();

    if(chipId != 0xFF)
      return false;
  }
  else
  {
    return false;
  }

  reset();
  return true;
}

uint8_t QMC5883L::ready()
{
  if(!read_register(addr, QMC5883L_STATUS, 1))
    return 0;
  uint8_t status = Wire.read();
  return status & QMC5883L_STATUS_DRDY;
}

uint8_t QMC5883L::readRaw(int16_t *x, int16_t *y, int16_t *z, int16_t *t)
{
  while(!ready())
  {
  }

  if(!read_register(addr, QMC5883L_X_LSB, 6))
    return 0;

  *x = Wire.read() | (Wire.read() << 8);
  *y = Wire.read() | (Wire.read() << 8);
  *z = Wire.read() | (Wire.read() << 8);

  return 1;
}

uint8_t QMC5883L::readRaw(int16_t *x, int16_t *y, int16_t *z)
{
  if(ready())
  {
    Wire.beginTransmission(0x0d);
    Wire.write(0);
    Wire.endTransmission();

    Wire.requestFrom(0x0d, 6);
    int n = Wire.available();
    if(n != 6)
      return 0;

    *x = Wire.read() | (Wire.read() << 8);
    *y = Wire.read() | (Wire.read() << 8);
    *z = Wire.read() | (Wire.read() << 8);

    return 1;
  }

  return 0;
}

void QMC5883L::resetCalibration()
{
  xhigh = yhigh = 0;
  xlow = ylow = 0;
}

int32_t QMC5883L::readHeading()
{
  int16_t x, y, z;

  if(!readRaw(&x, &y, &z))
    return 0;

  /* Update the observed boundaries of the measurements */

  if(x < xlow)
    xlow = x;
  if(x > xhigh)
    xhigh = x;
  if(y < ylow)
    ylow = y;
  if(y > yhigh)
    yhigh = y;

  /* Bail out if not enough data is available. */

  if(xlow == xhigh || ylow == yhigh)
    return 0;

  /* Recenter the measurement by subtracting the average */

  x -= (xhigh + xlow) / 2;
  y -= (yhigh + ylow) / 2;

  /* Rescale the measurement to the range observed. */

  float fx = (float) x / (xhigh - xlow);
  float fy = (float) y / (yhigh - ylow);

  int32_t heading = 180.0 * atan2(fy, fx) / M_PI;
  if(heading <= 0)
    heading += 360;

  return heading;
}
