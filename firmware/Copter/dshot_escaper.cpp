#include "pdl.h"

#include <ESP8266WiFi.h>
#include "dshot.h"

#define M1_PIN              16
#define M2_PIN              0
#define M3_PIN              15
#define M4_PIN              2

void pdlSetupEscaper(pdlDroneState*)
{
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);

  digitalWrite(M1_PIN, LOW);  // digitalWrite takes 175 cpu cycles to execute!
  digitalWrite(M2_PIN, LOW);
  digitalWrite(M3_PIN, LOW);
  digitalWrite(M4_PIN, LOW);

  dshotSetup(M1_PIN,M2_PIN,M3_PIN,M4_PIN,1000);
  pdlSetMotorGasLimits(0,0,2000);
  /*
    dshotEnable(0);

    dshotSet(9,9,9,9);

    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);

    dshotSet(12,12,12,12);

    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);
    dshotWrite300();
    delayMicroseconds(100);

    dshotSet(48,48,48,48);

    delay(50);

    dshotEnable(1);
   */
}

void pdlUpdateEscaper(pdlDroneState* ds)
{
  static int32_t oldMotorGas[PDL_MOTOR_COUNT] = {0,0,0,0};

  uint8_t modified = 0;

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT ; i++)
  {
    if(ds->motorGas[i] != oldMotorGas[i])
    {
      modified = 1;
      oldMotorGas[i] = ds->motorGas[i];
    }
  }

  if(modified)
  {
    dshotSet( ds->motorGas[0] + 48,
              ds->motorGas[1] + 48,
              ds->motorGas[2] + 48,
              ds->motorGas[3] + 48);
  }
}
