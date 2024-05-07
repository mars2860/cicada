#include "pdl.h"

#include <ESP8266WiFi.h>
#include <Wire.h>
#include "movingAvg.h"

//-----------------------------------------------------------------------------
// INSTALL
// 1. https://github.com/enjoyneering/ESP8266-I2C-Driver

//---------------------------------------------------------------
// VARIABLES

pdlDroneState droneState;

movingAvg avgLoopTime(10);

uint32_t pdlMicros()
{
  return micros();
}

//--------------------------------------------------------------
// INTERRUPTS

//--------------------------------------------------------------

void setup()
{
  memset(&droneState, 0, sizeof(droneState));

  Wire.begin();
  Wire.setClock(200000UL);  // line capacity is too high, can't run at higher frequency

  pdlSetup(&droneState);

  avgLoopTime.begin();
}

//-----------------------------------------------------------------------------

void loop()
{
  static uint32_t loopStartTime = 0;
  uint32_t loopTime = 0;

  loopStartTime = micros();

  pdlUpdate(&droneState);

  loopTime = micros() - loopStartTime;

  if(loopTime > droneState.maxLoopTime)
    droneState.maxLoopTime = loopTime;

  droneState.avgLoopTime = avgLoopTime.reading(loopTime);
}
