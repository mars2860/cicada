#include "pdl.h"
#include "main.h"

#include "baro_bmp280.h"

static BaroDriver *pBaro = NULL;
static uint8_t old_motorsEnabled = 0;
static uint8_t stabCounter = 0;
static float qnePressure = 0;

//-----------------------------------------------------------------------------
// PDL BARO FUNCTIONS IMPLEMENTATION

void pdlSetupBaro(pdlDroneState *ds)
{
  if(pBaro) // reinit when load default config command is received
  {
    pBaro->setup();
    return;
  }
  // scan for bmp280
  pBaro = new Bmp280DevDriver();
  if(pBaro->setup())
    return;
  // baro is not found
  if(pBaro)
    delete pBaro;
  pBaro = NULL;
  LOG_INFO("Barometer is not found");
  pdlSetError(ds,ERR_BARO_NOT_FOUND);
}

uint8_t pdlReadBaro(pdlDroneState *ds)
{
  // if host is not set, there are invalid data
  if(hostIsSet() == false || !pBaro)
  {
    stabCounter = 0;
    return 0;
  }

  if(pBaro->read(ds->baro.seaLevelPressure) == false)
    return 0;

  pdlNewTemperatureData(ds,pBaro->getTemperature());

  if(stabCounter < 10)
  {
    // calc reference pressure for zero altitude
    stabCounter++;
    qnePressure += pBaro->getPressure();
    if(stabCounter == 10)
    {
      ds->baro.seaLevelPressure = qnePressure / 10.f;
    }

    return 0;
  }

  // when motors get armed we starts to calc reference pressure
  if(ds->motorsEnabled && !old_motorsEnabled)
  {
    stabCounter = 0;
    qnePressure = 0;
  }

  old_motorsEnabled = ds->motorsEnabled;

  pdlNewBaroData(ds,pBaro->getPressure(),pBaro->getAltitude());

  return 1;
}
