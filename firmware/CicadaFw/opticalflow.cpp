#include "pdl.h"
#include "main.h"
#include <math.h>
#include <PMW3901.h>
#include <PCA9536.h>

// Warn: for SPI-CS used TXD pin! Sold out this wire from OF-module if you going to program esp8266 by UART
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

class PMW3901Driver: public OpticalFlowDriver
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

static OpticalFlowDriver *pOpticalFlow = 0;
static uint8_t stabCounter = 0;

bool isPmw3901Available()
{
  if(pOpticalFlow)
    return true;
  return false;
}

//-----------------------------------------------------------------------------
// PDL OPTICAL FLOW FUNCTION IMPLEMENTATION

void pdlSetupOpticalFlow(pdlDroneState *ds)
{
  if(pOpticalFlow)  // reinit when load default config command is received
  {
    pOpticalFlow->setup();
    return;
  }
  // scan for pmw3901
  pOpticalFlow = new PMW3901Driver();
  if(pOpticalFlow->setup())
    return;
  // optical flow not found
  delete pOpticalFlow;
  pOpticalFlow = 0;

  ds->opticalFlow.rawX = 0;
  ds->opticalFlow.rawY = 0;
  ds->opticalFlow.pureX = 0;
  ds->opticalFlow.pureY = 0;
  ds->opticalFlow.velX = 0;
  ds->opticalFlow.velY = 0;

  pdlSetError(ds,ERR_OF_NOT_FOUND);
  LOG_INFO("OpticalFlow is not found");
}

uint8_t pdlReadOpticalFlow(pdlDroneState *ds)
{
  if(!pOpticalFlow)
    return 0;

  if( hostIsSet() == false ||                 // if host is not set, there are invalid data
      ds->kfSettings.poseModelNoise == 0.f    // also if Kalman is not set there are invalid pitch/roll
    )
    return 0;

  if(stabCounter < 50)
  {
    stabCounter++;
    return 0;
  }

  // TODO add fov,npx,scaler,rotation x, rotation y to settings
  if(pOpticalFlow->read())
  {
    pdlResetError(ds,ERR_OF_DATA_NOT_READY);
    pdlNewOpticalFlowData(  ds,
                            pOpticalFlow->getRawX(),
                            pOpticalFlow->getRawY(),
                            pOpticalFlow->getSqual(),
                            pOpticalFlow->getFOV(),
                            pOpticalFlow->getNpx(),
                            pOpticalFlow->getScaler());
  }
  else
  {
    pdlSetError(ds,ERR_OF_DATA_NOT_READY);
  }

  return 1;
}
