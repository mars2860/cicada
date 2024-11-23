#include "pdl.h"
#include "main.h"
#include "opticalflow_pmw3901.h"

static OpticalFlowDriver *pOpticalFlow = NULL;
static uint8_t stabCounter = 0;

//-----------------------------------------------------------------------------
// PDL OPTICAL FLOW FUNCTION IMPLEMENTATION

void pdlSetupOpticalFlow(pdlDroneState *ds)
{
  if(pOpticalFlow)  // reinit when load default config command is received
  {
    pOpticalFlow->setup();
    return;
  }

#ifdef CICADA_GY91
  // scan for pmw3901
  pOpticalFlow = new CicadaGy91PMW3901Driver();
  if(pOpticalFlow->setup())
    return;
#endif
  // optical flow not found
  if(pOpticalFlow)
    delete pOpticalFlow;
  pOpticalFlow = NULL;

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
