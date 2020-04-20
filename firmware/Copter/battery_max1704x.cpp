#include "pdl.h"

#include "MAX1704X.h"

MAX1704X max17043(0.00125f);

void pdlSetupBattery(pdlDroneState*)
{
  max17043.quickstart();
  pdlSetBatteryReadPeriod(100000);
}

void pdlReadBattery(pdlDroneState *ds)
{
  ds->battery.voltage = max17043.voltage();
  ds->battery.percent = max17043.percent();
}
