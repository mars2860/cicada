#ifndef DRIVERS_BMP280_BARO_ADAFRUIT_BMP280_H_
#define DRIVERS_BMP280_BARO_ADAFRUIT_BMP280_H_

#include "baro.h"
#include "Adafruit_BMP280.h"
#include "RunningMedian.h"

/// Deprecated
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

#endif /* DRIVERS_BMP280_BARO_ADAFRUIT_BMP280_H_ */
