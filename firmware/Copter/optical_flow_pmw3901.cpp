#include "pdl.h"
#include "Bitcraze_PMW3901.h"

// Warn: for CS used TXD pin!
Bitcraze_PMW3901 flow(1);
boolean sensorValid;

void pdlSetupOpticalFlow(pdlDroneState*)
{
  delay(500);

  sensorValid = flow.begin();
  pdlSetOpticalFlowReadPeriod(50000);
}

void pdlReadOpticalFlow(pdlDroneState *ds)
{
  int16_t deltaX, deltaY;
  if(sensorValid)
  {
    flow.readMotionCount(&deltaX, &deltaY);
    ds->opticalFlowX = deltaX;
    ds->opticalFlowY = -deltaY;
  }
}
