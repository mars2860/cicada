#include "pdl.h"
#include "main.h"

#include "lidar_tfmini_plus.h"
#include "lidar_vl53l1x.h"
#include "sonar_hcsr04.h"

static LidarDriver *pLidar = NULL;
static uint8_t stabCounter = 0;

//-----------------------------------------------------------------------------
// PDL LIDAR FUNCTIONS IMPLEMENTATION

void pdlSetupLidar(pdlDroneState *ds)
{
  ds->lidar.range = -1.f;
  if(pLidar)  // reinit when load default config command is received
  {
    pLidar->setup();
    return;
  }
  // scan for tfmini-plus
  pLidar = new TFminiPlusDriver();
  if(pLidar->setup())
    return;
  // tfmini not found, delete its object
  if(pLidar)
    delete pLidar;
  // scan for vl53l1x
  pLidar = new VL53L1xDriver();
  if(pLidar->setup())
    return;
  // vl53l1x not found, delete its object
  if(pLidar)
    delete pLidar;
  // scan for hcsr04
  pLidar = new HCSR04Driver();
  if(pLidar->setup())
    return;
  // no any lidar found
  if(pLidar)
    delete pLidar;
  pLidar = NULL;
  pdlSetError(ds,ERR_LIDAR_NOT_FOUND);
  LOG_INFO("Lidar is not found");
}

uint8_t pdlReadLidar(pdlDroneState *ds)
{
  // if host is not set, there are invalid data
  if(hostIsSet() == false || !pLidar)
  {
    return 0;
  }

  if(stabCounter < 20)  // wait 2s for data stabilization
  {
    stabCounter++;
    return 0;
  }

  bool dataReady = pLidar->isDataReady();

  if(dataReady == false)
  {
    pdlNewLidarData(ds,-1.f,pLidar->getFOV());
    pdlSetError(ds,ERR_LIDAR_DATA_NOT_READY);
    return 0;
  }

  pdlResetError(ds,ERR_LIDAR_DATA_NOT_READY);

  float range = pLidar->getRange();
  pdlNewLidarData(ds,range,pLidar->getFOV());

  if(range > 0)
  {
    pdlResetError(ds,ERR_LIDAR_DATA_INVALID);
  }
  else
  {
    pdlSetError(ds,ERR_LIDAR_DATA_INVALID);
  }

  if(range < 0)
  {
    stabCounter = 0;
  }

  return 1;
}
