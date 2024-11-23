#ifndef DRIVERS_OPTICALFLOW_OPTICALFLOW_PMW3901_H_
#define DRIVERS_OPTICALFLOW_OPTICALFLOW_PMW3901_H_

#include "opticalflow.h"
#include "pmw3901.h"
#include "pca9536.h"

class PMW3901ex : public PMW3901
{
private:
  PCA9536 exGpio;
public:
  boolean begin()
  {
    exGpio.setMode(IO_OUTPUT);
    return PMW3901::begin();
  }

  void cs_high()
  {
    exGpio.setState(IO1,IO_HIGH);
  }

  void cs_low()
  {
    exGpio.setState(IO1,IO_LOW);
  }
};

class CicadaGy91PMW3901Driver: public OpticalFlowDriver
{
private:
  PMW3901ex flow;
  int16_t rawX;
  int16_t rawY;
  uint8_t squal;

  // new lens L214-ZSZ
  // old lens in datasheet LN03-ZSZ
  // pixel size = 30 um ? from some internet resource

  // field of view in radians from datasheet
  const float FOV = (42.f * PI / 180.f);
  // (determined by experiment1)
  //#define SCALER 10.5f
  // (determined by experiment2)
  const float SCALER = 9.f;
  // sensor resolution from bitcraze driver
  const float NPX = 35.f;
public:
  bool setup()
  {
    uint8_t prodId = 0;
    uint8_t revId = 0;

    if(flow.begin() == false)
    {
      return false;
    }

    prodId = flow.readProdId();
    revId = flow.readRevId();

    LOG_INFO("PMW3901 prod=%i,rev=%i",prodId,revId);
    return true;
  }

  bool read()
  {
    flow.readMotionCount(&rawY, &rawX, &squal);
    rawX = -rawX;
    return true;
  }

  int16_t getRawX()
  {
    return rawX;
  }

  int16_t getRawY()
  {
    return rawY;
  }

  uint8_t getSqual()
  {
    return squal;
  }

  float getFOV()
  {
    return FOV;
  }

  float getNpx()
  {
    return NPX;
  }

  float getScaler()
  {
    return SCALER;
  }
};

#endif /* DRIVERS_OPTICALFLOW_OPTICALFLOW_PMW3901_H_ */
