#include "pdl.h"
#include <arduino.h>

void pdlSetupBattery(pdlDroneState*)
{
  pdlSetBatteryReadPeriod(100000);
}

void pdlReadBattery(pdlDroneState *ds)
{
  float voltage = analogRead(A0);
  voltage *= 0.0055572f; // 4.2f/ 734.65f;
  ds->battery.voltage = voltage;

  if(voltage > 4.15f)
    ds->battery.percent = 100.0f;
  else if(voltage > 4.11f)
    ds->battery.percent = 95.0f;
  else if(voltage > 4.08f)
    ds->battery.percent = 90.0f;
  else if(voltage > 4.02f)
    ds->battery.percent = 85.0f;
  else if(voltage > 3.98f)
    ds->battery.percent = 80.0f;
  else if(voltage > 3.95f)
    ds->battery.percent = 75.0f;
  else if(voltage > 3.91f)
    ds->battery.percent = 70.0f;
  else if(voltage > 3.87f)
    ds->battery.percent = 65.0f;
  else if(voltage > 3.85f)
    ds->battery.percent = 60.0f;
  else if(voltage > 3.84f)
    ds->battery.percent = 55.0f;
  else if(voltage > 3.82f)
    ds->battery.percent = 50.0f;
  else if(voltage > 3.80f)
    ds->battery.percent = 45.0f;
  else if(voltage > 3.79f)
    ds->battery.percent = 40.0f;
  else if(voltage > 3.77f)
    ds->battery.percent = 35.0f;
  else if(voltage > 3.75f)
    ds->battery.percent = 30.0f;
  else if(voltage > 3.73f)
    ds->battery.percent = 25.0f;
  else if(voltage > 3.71f)
    ds->battery.percent = 20.0f;
  else if(voltage > 3.69f)
    ds->battery.percent = 15.0f;
  else if(voltage > 3.61f)
    ds->battery.percent = 10.0f;
  else if(voltage > 3.27f)
    ds->battery.percent = 5.0f;
  else
    ds->battery.percent = 0.0f;
}
