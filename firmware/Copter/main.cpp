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

  /* debug
  pdlDroneState *ds = &droneState;
  Serial.print("timestamp addr = ");Serial.println((uint32_t)&ds->timestamp,HEX);
  Serial.print("battery addr = ");Serial.println((uint32_t)&ds->battery,HEX);
  Serial.print("rc addr = ");Serial.println((uint32_t)&ds->rc,HEX);
  Serial.print("accel addr = ");Serial.println((uint32_t)&ds->accel,HEX);
  Serial.print("gyro addr = ");Serial.println((uint32_t)&ds->gyro,HEX);
  Serial.print("magneto addr = ");Serial.println((uint32_t)&ds->magneto,HEX);
  Serial.print("yaw addr = ");Serial.println((uint32_t)&ds->yaw,HEX);
  Serial.print("pitch addr = ");Serial.println((uint32_t)&ds->pitch,HEX);
  Serial.print("roll addr = ");Serial.println((uint32_t)&ds->roll,HEX);
  Serial.print("heading addr = ");Serial.println((uint32_t)&ds->heading,HEX);
  Serial.print("mainLoop addr = ");Serial.println((uint32_t)&ds->mainLoopTime,HEX);
  Serial.print("temperature addr = ");Serial.println((uint32_t)&ds->temperature,HEX);
  Serial.print("pressure addr = ");Serial.println((uint32_t)&ds->pressure,HEX);
  Serial.print("altitude addr = ");Serial.println((uint32_t)&ds->altitude,HEX);
  Serial.print("seaLevel addr = ");Serial.println((uint32_t)&ds->seaLevel,HEX);
  Serial.print("yawRatePid addr = ");Serial.println((uint32_t)&ds->yawRatePid,HEX);
  Serial.print("pitchPid addr = ");Serial.println((uint32_t)&ds->pitchPid,HEX);
  Serial.print("rollPid addr = ");Serial.println((uint32_t)&ds->rollPid,HEX);
  Serial.print("altPid addr = ");Serial.println((uint32_t)&ds->altPid,HEX);
  Serial.print("baseGas addr = ");Serial.println((uint32_t)&ds->baseGas,HEX);
  Serial.print("motorGas addr = ");Serial.println((uint32_t)&ds->motorGas[0],HEX);
  Serial.print("motorsEnabled addr = ");Serial.println((uint32_t)&ds->motorsEnabled,HEX);
  Serial.print("stabEn addr = ");Serial.println((uint32_t)&ds->stabilizationEnabled,HEX);
*/

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
