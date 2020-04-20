#include "pdl.h"

#include <ESP8266WiFi.h>
#include <Wire.h>

//-----------------------------------------------------------------------------
// INSTALL
// 1. https://github.com/enjoyneering/ESP8266-I2C-Driver


//---------------------------------------------------------------
// VARIABLES

pdlDroneState droneState;

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

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(200000UL);  // line capacity is too high, can't run at higher frequency

  pdlSetup(&droneState);
}

//-----------------------------------------------------------------------------

void loop()
{
  static uint32_t loopStartTime = 0;

  loopStartTime = micros();

  pdlUpdate(&droneState);

  droneState.mainLoopTime = micros() - loopStartTime;
}
