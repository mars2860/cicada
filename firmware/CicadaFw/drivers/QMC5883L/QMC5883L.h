#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdint.h>

class QMC5883L
{
public:
  void init();
  void reset();
  uint8_t ready();
  void reconfig();

  int32_t readHeading();
  ///< Blocks cpu until gets answer from the sensor
  uint8_t readRaw(int16_t *x, int16_t *y, int16_t *z, int16_t *t);
  ///< Don't blocks cpu, returns 0 if data is not ready
  uint8_t readRaw(int16_t *x, int16_t *y, int16_t *z);

  void resetCalibration();

  void setSamplingRate(int rate);
  void setRange(int range);
  void setOversampling(int ovl);

private:
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;
};

#endif
