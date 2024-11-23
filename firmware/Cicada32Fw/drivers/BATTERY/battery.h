#ifndef DRIVERS_BATTERY_BATTERY_H_
#define DRIVERS_BATTERY_BATTERY_H_

#include "pdl.h"

class BatteryDriver
{
public:
  virtual ~BatteryDriver() {};
  virtual bool setup() = 0;
  virtual bool read(pdlDroneState *ds) = 0;
  virtual float getVoltage() = 0;
  virtual float getCurrent() = 0;
  virtual float getPercent() = 0;
  virtual float getCapacity() = 0;
};

#endif /* DRIVERS_BATTERY_BATTERY_H_ */
