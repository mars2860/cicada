#include "pdl.h"
#include "main.h"
#include "Adafruit_BMP280.h"
#include "BMP280_DEV.h"
#include "RunningMedian.h"

class BaroDriver
{
public:
  virtual ~BaroDriver() {};
  virtual bool setup() = 0;
  virtual bool read(float seaLevel) = 0;
  virtual float getPressure() = 0;
  virtual float getAltitude() = 0;
  virtual float getTemperature() = 0;
};

/// Deprecated (TODO: it needs to remove adafruit driver)
class Bmp280Driver: public BaroDriver
{
private:
  Adafruit_BMP280 baro;
  // median filter is needed because sometimes there are spikes in baro data
  // I guess it caused by tfmini-plus on i2c line? or direct sun light?
  RunningMedian pf;
  RunningMedian af;
  RunningMedian tf;
  float pressure;
  float temperature;
  float altitude;
public:
  Bmp280Driver(): pf(7), af(7), tf(3)
  {
    pressure = 0;
    temperature = 0;
    altitude = 0;
  };

  bool setup()
  {
    /* Default settings from datasheet. */
    uint8_t chipId = baro.begin(BMP280_ADDRESS_ALT);

    if(chipId != BMP280_CHIPID)
    {
      LOG_INFO("Unknown baro,id=0x%x",chipId);
      return false;
    }

    LOG_INFO("BMP280 is ok,id=0x%x",chipId);

    // DLPF adds time delay. It is very bad for Kalman Filter and leads to estimation error
    // Thus to turn off DLPF inside BMP280

    // Indoor navigation mode, ODR=26.3Hz
    /*
    baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                     Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                     Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                     Adafruit_BMP280::FILTER_X16,      // Filtering.
                     Adafruit_BMP280::STANDBY_MS_1);   // Standby time.
    */

    // Drop detection mode, ODR=125Hz
    baro.setSampling( Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                      Adafruit_BMP280::SAMPLING_X1,     // Temp. oversampling
                      Adafruit_BMP280::SAMPLING_X2,    // Pressure oversampling
                      Adafruit_BMP280::FILTER_OFF,      // Filtering.
                      Adafruit_BMP280::STANDBY_MS_1);   // Standby time.

    /*
    // handheld device (dynamic) ODR=83.3 Hz
    baro.setSampling( Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                      Adafruit_BMP280::SAMPLING_X1,     // Temp. oversampling
                      Adafruit_BMP280::SAMPLING_X4,     // Pressure oversampling
                      Adafruit_BMP280::FILTER_X16,      // Filtering.
                      Adafruit_BMP280::STANDBY_MS_1);   // Standby time.
    */

    /*
    baro.setSampling( Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                      Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                      Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                      Adafruit_BMP280::FILTER_OFF,      // Filtering.
                      Adafruit_BMP280::STANDBY_MS_1);   // Standby time.
    */

    baro.readCoefficients();
    return true;
  }

  bool read(float seaLevel)
  {
    baro.update(seaLevel);

    pf.add(baro.pressure);
    pressure = pf.getMedian();

    af.add(baro.altitude);
    altitude = af.getMedian();

    tf.add(baro.temperature);
    temperature = tf.getMedian();

    return true;
  }

  float getPressure()
  {
    return pressure;
  }

  float getAltitude()
  {
    return altitude;
  }

  float getTemperature()
  {
    return temperature;
  }
};

class Bmp280DevDriver: public BaroDriver
{
private:
  BMP280_DEV baro;
  // median filter is needed because sometimes there are spikes in baro data
  // I guess it caused by tfmini-plus on i2c line? or direct sun light?
  RunningMedian pf;
  RunningMedian af;
  RunningMedian tf;
  float pressure;
  float temperature;
  float altitude;
public:
  Bmp280DevDriver(): pf(3), af(3), tf(3)
  {
    pressure = 0;
    temperature = 0;
    altitude = 0;
  };

  bool setup()
  {
    if(!baro.begin(FORCED_MODE, BMP280_I2C_ALT_ADDR))
      return false;

    // It's already default settings
    //baro.setTimeStandby(TIME_STANDBY_05MS);
    //baro.setIIRFilter(IIR_FILTER_OFF);
    //baro.setPresOversampling(OVERSAMPLING_X16);
    //baro.setTempOversampling(OVERSAMPLING_X2);

    LOG_INFO("BMP280 is ok");

    return true;
  }

  bool read(float seaLevel)
  {
    float temp, press, alt;

    baro.setSeaLevelPressure(seaLevel);

    baro.getCurrentMeasurements(temp, press, alt);

    pf.add(press);
    pressure = pf.getMedian();

    af.add(alt);
    altitude = af.getMedian();

    tf.add(temp);
    temperature = tf.getMedian();

    baro.startForcedConversion();

    return true;
  }

  float getPressure()
  {
    return pressure;
  }

  float getAltitude()
  {
    return altitude;
  }

  float getTemperature()
  {
    return temperature;
  }
};

static BaroDriver *pBaro = 0;
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
  delete pBaro;
  pBaro = 0;
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
