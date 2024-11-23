#ifndef DRIVERS_IMU_IMU_H_
#define DRIVERS_IMU_IMU_H_

#include "pdl.h"

class ImuDriver
{
public:
  virtual ~ImuDriver() {};
  virtual bool setupAccel(pdlDroneState *ds) = 0;
  virtual bool setupGyro(pdlDroneState *ds) = 0;
  virtual bool setupMagneto(pdlDroneState *ds) = 0;
  virtual void calibrateAccel(pdlDroneState *ds) = 0;
  virtual void calibrateGyro(pdlDroneState *ds) = 0;
  virtual void calibrateMagneto(pdlDroneState *ds) = 0;
  virtual bool readAccel() = 0;
  virtual bool readGyro() = 0;
  virtual bool readMagneto() = 0;
  virtual int16_t getAccelRaw(uint8_t i) = 0;
  virtual float getAccelPure(uint8_t i) = 0;
  virtual int16_t getGyroRaw(uint8_t i) = 0;
  virtual float getGyroPure(uint8_t i) = 0;
  virtual int16_t getMagnetoRaw(uint8_t i) = 0;
  virtual float getMagnetoPure(uint8_t i) = 0;
};

#endif /* DRIVERS_IMU_IMU_H_ */
