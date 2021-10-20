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
  pdlSetBaroReadPeriod(50000);
}

void pdlReadBaro(pdlDroneState *ds)
{
  baro.update(ds->seaLevel);
  //temperature = (float)(mpu.getTemperature()) / 340.f;  // require temperature offset
  ds->pressure = baro.pressure;
  ds->baroAlt = baro.altitude;
  ds->temperature = baro.temperature;
  //altitude = altest.getAltitude();
  //alt_estimator.update(baro.altitude);
  //altitude = alt_estimator.h;
  //temperature = altest.getVerticalVelocity();//alt_estimator.v;//vz;////baro.temperature;
}
