#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdint.h>

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001

class QMC5883L
{
public:
  bool init(  uint8_t rate = QMC5883L_CONFIG_50HZ,
              uint8_t range = QMC5883L_CONFIG_2GAUSS,
              uint8_t ovr = QMC5883L_CONFIG_OS512);
  void reset();
  uint8_t ready();
  void reconfig();

  int32_t readHeading();
  ///< Blocks cpu until gets answer from the sensor
  uint8_t readRaw(int16_t *x, int16_t *y, int16_t *z, int16_t *t);
  ///< Don't blocks cpu, returns 0 if data is not ready
  uint8_t readRaw(int16_t *x, int16_t *y, int16_t *z);

  void resetCalibration();

  void setSamplingRate(int rate);
  void setRange(int range);
  void setOversampling(int ovl);

private:
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;
};

#endif
