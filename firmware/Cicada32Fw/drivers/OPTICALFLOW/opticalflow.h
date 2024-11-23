#ifndef DRIVERS_OPTICALFLOW_OPTICALFLOW_H_
#define DRIVERS_OPTICALFLOW_OPTICALFLOW_H_

#include "pdl.h"

class OpticalFlowDriver
{
public:
  virtual ~OpticalFlowDriver() {};
  virtual bool setup() = 0;
  virtual bool read() = 0;
  virtual int16_t getRawX() = 0;
  virtual int16_t getRawY() = 0;
  virtual uint8_t getSqual() = 0;
  virtual float getFOV() = 0;
  virtual float getNpx() = 0;
  virtual float getScaler() = 0;
};

#endif /* DRIVERS_OPTICALFLOW_OPTICALFLOW_H_ */
