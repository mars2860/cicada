#include "pdl.h"

#include "Adafruit_BMP280.h"

Adafruit_BMP280 baro;

//Altitude_KF alt_estimator(0.1f, 0.1f);
/*AltitudeEstimator altest(0.0005,  // sigma Accel
                         0.0005,  // sigma Gyro
                         0.018,   // sigma Baro
                         0.5,   // ca
                         0.1);  // accelThreshold*/

void pdlSetupBaro(pdlDroneState*)
{
  /* Default settings from datasheet. */
  baro.init(BMP280_ADDRESS_ALT);
  baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  baro.readCoefficients();
}

uint8_t pdlReadBaro(pdlDroneState *ds)
{
  baro.update(ds->baro.seaLevelPressure);
  //temperature = (float)(mpu.getTemperature()) / 340.f;  // require temperature offset
  ds->baro.pressure = baro.pressure;
  ds->baro.altitude = baro.altitude;
  ds->temperature = baro.temperature;

  //altitude = altest.getAltitude();
  //alt_estimator.update(baro.altitude);
  //altitude = alt_estimator.h;
  //temperature = altest.getVerticalVelocity();//alt_estimator.v;//vz;////baro.temperature;

  if(ds->lidarRange < 0 || ds->lidarRange > PDL_LIDAR_MAX_RANGE)
  {
    ds->altitude = ds->baro.altitude;
    //return 1;
  }

  return 0;
}
