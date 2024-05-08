#include "pdl.h"
#include "tfmini_plus.h"
#include "vl53l1x_class.h"
#include "hcsr04_i2c.h"
#include "main.h"

#include <math.h>

class LidarDriver
{
public:
  virtual ~LidarDriver() {};
  virtual bool setup() = 0;
  virtual bool isDataReady() = 0;
  virtual float getRange() = 0;
  virtual float getFOV() = 0;
};

class TFminiPlusDriver: public LidarDriver
{
private:
  const float TFMINI_FOV = 3.6f * PI/180.f;   // field of view for tfmini in rad
  TFminiPlus tf;
  tfminiplus_data_t data;
public:
  bool setup()
  {
    tf.begin();

    tfminiplus_version_t version = tf.get_version();

    if(version.major == 0)
    {
      return false;
    }

    LOG_INFO("TFmini v%i.%i.%i",version.major,version.minor,version.revision);

    return true;
  }

  bool isDataReady()
  {
    return tf.read_data(data, true);
  }

  float getRange()
  {
    float range = -1.f;
    if(data.strength > 100 && data.strength != 0XFFFF)
    {
      range = data.distance;
      range /= 1000.f;
    }
    return range;
  }

  float getFOV()
  {
    return TFMINI_FOV;
  }
};

class VL53L1xDriver: public LidarDriver
{
private:
  const float VL53L1X_FOV = 27.f * PI/180.f;   // field of view for vl53l1x in rad
  VL53L1X vl;
public:
  VL53L1xDriver(): vl(&Wire,-1) {};

  bool setup()
  {
    if(vl.Init() != 0)
      return false;

    uint16_t sensorId;
    vl.VL53L1X_GetSensorId(&sensorId);

    LOG_INFO("VL53L1X id=%i",sensorId);

    /*  Timing budget (TB)
        The VL53L1X timing budget can be set from 20 ms up to 1000 ms.
          • 20 ms is the minimum timing budget and can be used only in Short distance mode.
          • 33 ms is the minimum timing budget which can work for all distance modes.
          • 140 ms is the timing budget which allows the maximum distance of 4 m (in the dark on
            a white chart) to be reached under Long distance mode
        Increasing the timing budget increases the maximum distance the device can range and improves
        the repeatability error. However, average power consumption augments accordingly.
    */
    /* Ranging is continuous, with a programmable delay between two ranging operations
       (called an inter-measurement period). Ranging duration (timing budget) is also
       programmable.
    */

    // Long mode 1
    vl.VL53L1X_SetDistanceMode(2);
    vl.VL53L1X_SetTimingBudgetInMs(100);
    vl.VL53L1X_SetInterMeasurementInMs(100);

    // Short mode 1
    //vl.VL53L1X_SetDistanceMode(1);
    //vl.VL53L1X_SetTimingBudgetInMs(100);
    //vl.VL53L1X_SetInterMeasurementInMs(100);

    /*
    // Long mode 2
    vl.VL53L1X_SetDistanceMode(2);
    vl.VL53L1X_SetTimingBudgetInMs(200);
    vl.VL53L1X_SetInterMeasurementInMs(200);
    */

    /*
    // Short mode 2
    vl.VL53L1X_SetDistanceMode(1);
    vl.VL53L1X_SetTimingBudgetInMs(50);
    vl.VL53L1X_SetInterMeasurementInMs(50);
    */
    // work in single shot mode, because update period is not constant
    vl.VL53L1X_ClearInterrupt();
    vl.VL53L1X_StartSingle();

    return true;
  }

  bool isDataReady()
  {
    uint8_t dataReady = 0;
    uint32_t startTimeout = micros();

    while(micros() - startTimeout < 2000)
    {
      vl.VL53L1X_CheckForDataReady(&dataReady);
      if(dataReady)
      {
        return true;
      }
    }

    return false;
  }

  float getRange()
  {
    float range = -1.f;
    uint8_t rangeStatus = 0;
    uint16_t distance = 0;

    vl.VL53L1X_GetDistance(&distance);
    vl.VL53L1X_GetRangeStatus(&rangeStatus);

    if(rangeStatus == 0)
    {
      range = distance;
      range /= 1000.f;
    }

    vl.VL53L1X_ClearInterrupt();
    vl.VL53L1X_StartSingle();

    return range;
  }

  float getFOV()
  {
    return VL53L1X_FOV;
  }
};

class HCSR04Driver: public LidarDriver
{
private:
  const float HCSR04_FOV = 15.f * PI/180.f;   // field of view for tfmini in rad
  HCSR04_I2C sonar;
public:
  bool setup()
  {
    bool result = false;
    for(uint8_t i = 0; i < 5; i++)
    {
      if(getRange() > 0)
      {
        result = true;
        break;
      }
    }
    if(result)
    {
      LOG_INFO("HCSR04");
    }
    return result;
  }

  bool isDataReady()
  {
    return true;
  }

  float getRange()
  {
    Wire.flush();
    Wire.setClock(100000UL);

    float range = sonar.getRange_m();
    sonar.measure();

    Wire.setClock(400000UL);
    Wire.flush();

    return range;
  }

  float getFOV()
  {
    return HCSR04_FOV;
  }
};

static LidarDriver *pLidar = 0;
static uint8_t stabCounter = 0;

//-----------------------------------------------------------------------------
// PDL LIDAR FUNCTIONS IMPLEMENTATION

void pdlSetupLidar(pdlDroneState *ds)
{
  ds->lidar.range = -1.f;
  // scan for tfmini-plus
  pLidar = new TFminiPlusDriver();
  if(pLidar->setup())
    return;
  // tfmini not found, delete its object
  delete pLidar;
  // scan for vl53l1x
  pLidar = new VL53L1xDriver();
  if(pLidar->setup())
    return;
  // vl53l1x not found, delete its object
  delete pLidar;
  // scan for hcsr04
  pLidar = new HCSR04Driver();
  if(pLidar->setup())
    return;
  // no any lidar found
  delete pLidar;
  pLidar = 0;
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
