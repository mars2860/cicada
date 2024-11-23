#include "pdl.h"
#include "main.h"
#include "battery.h"

#ifdef CICADA_GY91
  #include "cicada_gy91_battery.h"
#elif CICADA_MICRO
  #include "adc_battery.h"
#elif ARDUINO_XIAO_ESP32S3
  #include "adc_battery.h"
#endif

static BatteryDriver *pBattery = NULL;

//-----------------------------------------------------------------------------
// PDL BATTERY FUNCTION IMPLEMENTATION

void pdlSetupBattery(pdlDroneState*)
{
  if(pBattery)  // reinit when load default config command is received
  {
    pBattery->setup();
    return;
  }

#ifdef CICADA_GY91
  // scan for two chl adc battery indicator
  pBattery = new CicadaGy91BatteryDriver(A0);
  if(pBattery->setup())
    return;
#elif CICADA_MICRO
  // create one chl adc battery indicator
  pBattery = new OneChlAdcBatteryDriver(A0);
  if(pBattery->setup())
    return;
#elif ARDUINO_XIAO_ESP32S3
  // create one chl adc battery indicator
  pBattery = new OneChlAdcBatteryDriver(A8);
  if(pBattery->setup())
    return;
#endif

  // battery is not found
  if(pBattery)
    delete pBattery;
  pBattery = NULL;
  LOG_INFO("Battery indicator is not found");
}

void pdlReadBattery(pdlDroneState *ds)
{
  if(!pBattery)
    return;

  if(pBattery->read(ds))
  {
    ds->battery.voltage = pBattery->getVoltage();
    ds->battery.current = pBattery->getCurrent();
    ds->battery.percent = pBattery->getPercent();
    ds->battery.capacity = pBattery->getCapacity();
  }
}
