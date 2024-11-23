#ifndef DRIVERS_BMP280_DEV_BARO_BMP280_H_
#define DRIVERS_BMP280_DEV_BARO_BMP280_H_

#include "baro.h"
#include "BMP280_DEV.h"
#include "RunningMedian.h"

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

#endif /* DRIVERS_BMP280_DEV_BARO_BMP280_H_ */
