#ifndef DRIVERS_BARO_BARO_H_
#define DRIVERS_BARO_BARO_H_

#include "pdl.h"

class BaroDriver
{
public:
  virtual ~BaroDriver() {};
  virtual bool setup() = 0;
  virtual bool read(float seaLevel) = 0;
  virtual float getPressure() = 0;
  virtual float getAltitude() = 0;
  virtual float getTemperature() = 0;
};

#endif /* DRIVERS_BARO_BARO_H_ */
