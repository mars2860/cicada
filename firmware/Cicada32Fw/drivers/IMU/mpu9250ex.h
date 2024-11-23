#ifndef MPU9250EX_H
#define MPU9250EX_H

#include "mpu9250.h"

class MPU9250Ex: public MPU9250
{
private:
  int16_t raw_acc_gyro_data[7];
  int16_t mag_count[3];
public:
  void setMagMode(uint8_t mode)
  {
    MAG_MODE = mode;
    write_byte(AK8963_ADDRESS, AK8963_CNTL, (uint8_t)setting.mag_output_bits << 4 | mode);  // Set magnetometer data resolution and sample ODR
  }

  bool isAccGyroDataReady()
  {
    return MPU9250::available();
  }

  bool isMagnetoDataReady()
  {
    uint8_t st1 = read_byte(AK8963_ADDRESS, AK8963_ST1);
    if(st1 & 0x01)
      return true;
    return false;
  }

public:
  byte whoAmI()
  {
    return read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
  }

  ///! @return false if data is not ready
  uint8_t readAccGyroData()
  {
    uint8_t raw_data[15];
    //read_accel_gyro(raw_acc_gyro_data);  // INT cleared on any read // takes 826 - 890 us on ESP8266 with 200 kHz I2C
    //read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 14, &raw_data[0]);
    read_bytes(mpu_i2c_addr, INT_STATUS, 15, &raw_data[0]);
    raw_acc_gyro_data[0] = ((int16_t)raw_data[1] << 8) | (int16_t)raw_data[2];
    raw_acc_gyro_data[1] = ((int16_t)raw_data[3] << 8) | (int16_t)raw_data[4];
    raw_acc_gyro_data[2] = ((int16_t)raw_data[5] << 8) | (int16_t)raw_data[6];
    raw_acc_gyro_data[3] = ((int16_t)raw_data[7] << 8) | (int16_t)raw_data[8];
    raw_acc_gyro_data[4] = ((int16_t)raw_data[9] << 8) | (int16_t)raw_data[10];
    raw_acc_gyro_data[5] = ((int16_t)raw_data[11] << 8) | (int16_t)raw_data[12];
    raw_acc_gyro_data[6] = ((int16_t)raw_data[13] << 8) | (int16_t)raw_data[14];

    // Now we'll calculate the accleration value into actual g's
    a[0] = (float)raw_acc_gyro_data[0] * acc_resolution;  // get actual g value, this depends on scale being set
    a[1] = (float)raw_acc_gyro_data[1] * acc_resolution;
    a[2] = (float)raw_acc_gyro_data[2] * acc_resolution;

    //temperature_count = raw_acc_gyro_data[3];                  // Read the adc values
    //temperature = ((float)temperature_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade

    // Calculate the gyro value into actual degrees per second
    g[0] = (float)raw_acc_gyro_data[4] * gyro_resolution;  // get actual gyro value, this depends on scale being set
    g[1] = (float)raw_acc_gyro_data[5] * gyro_resolution;
    g[2] = (float)raw_acc_gyro_data[6] * gyro_resolution;

    return raw_data[0];
  }

  bool readMagnetoData()
  {
    // this function takes 718 - 786 us on ESP8266 80 MHz, I2C 200 kHz, Timer0 enabled
    // Read the x/y/z adc values
    if (read_mag(mag_count)) {
      // Calculate the magnetometer values in milliGauss

      m[0] = (float)(mag_count[0] - mag_bias[0]) * mag_resolution * mag_bias_factory[0] * mag_scale[0];  // get actual magnetometer value, this depends on scale being set
      m[1] = (float)(mag_count[1] - mag_bias[1]) * mag_resolution * mag_bias_factory[1] * mag_scale[1];
      m[2] = (float)(mag_count[2] - mag_bias[2]) * mag_resolution * mag_bias_factory[2] * mag_scale[2];

      return true;
    }

    return false;
  }

  int16_t getAccRawX() const {return raw_acc_gyro_data[0];}
  int16_t getAccRawY() const {return raw_acc_gyro_data[1];}
  int16_t getAccRawZ() const {return raw_acc_gyro_data[2];}

  int16_t getGyroRawX() const {return raw_acc_gyro_data[4];}
  int16_t getGyroRawY() const {return raw_acc_gyro_data[5];}
  int16_t getGyroRawZ() const {return raw_acc_gyro_data[6];}

  int16_t getMagRawX() const {return mag_count[0];}
  int16_t getMagRawY() const {return mag_count[1];}
  int16_t getMagRawZ() const {return mag_count[2];}

  int16_t getAccRaw(const uint8_t i) const { return (i < 3) ? raw_acc_gyro_data[i] : 0; }
  int16_t getGyroRaw(const uint8_t i) const { return (i < 3) ? raw_acc_gyro_data[i + 4] : 0; }
  int16_t getMagRaw(const uint8_t i) const { return (i < 3) ? mag_count[i] : 0; }

  void writeAccelOffset(int16_t x, int16_t y, int16_t z)
  {
    int16_t acc_bias_reg[3] = {x,y,z};
    uint8_t write_data[6] = {0};

    write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    write_data[1] = (acc_bias_reg[0]) & 0xFF;
    write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    write_data[3] = (acc_bias_reg[1]) & 0xFF;
    write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    write_data[5] = (acc_bias_reg[2]) & 0xFF;

    // Push accelerometer biases to hardware registers
    write_byte(mpu_i2c_addr, XA_OFFSET_H, write_data[0]);
    write_byte(mpu_i2c_addr, XA_OFFSET_L, write_data[1]);
    write_byte(mpu_i2c_addr, YA_OFFSET_H, write_data[2]);
    write_byte(mpu_i2c_addr, YA_OFFSET_L, write_data[3]);
    write_byte(mpu_i2c_addr, ZA_OFFSET_H, write_data[4]);
    write_byte(mpu_i2c_addr, ZA_OFFSET_L, write_data[5]);
  }

  void readAccelOffset(int16_t *px, int16_t *py, int16_t *pz)
  {
    int16_t h, l;

    h = read_byte(mpu_i2c_addr, XA_OFFSET_H);
    l = read_byte(mpu_i2c_addr, XA_OFFSET_L);

    *px = ((h & 0x00FF) << 8) | (l & 0x00FF);

    h = read_byte(mpu_i2c_addr, YA_OFFSET_H);
    l = read_byte(mpu_i2c_addr, YA_OFFSET_L);

    *py = ((h & 0x00FF) << 8) | (l & 0x00FF);

    h = read_byte(mpu_i2c_addr, ZA_OFFSET_H);
    l = read_byte(mpu_i2c_addr, ZA_OFFSET_L);

    *pz = ((h & 0x00FF) << 8) | (l & 0x00FF);
  }

  void writeGyroOffset(int16_t x, int16_t y, int16_t z)
  {
    int16_t gyro_bias_reg[3] = {x,y,z};
    uint8_t write_data[6] = {0};

    write_data[0] = (gyro_bias_reg[0] >> 8) & 0xFF;
    write_data[1] = (gyro_bias_reg[0]) & 0xFF;
    write_data[2] = (gyro_bias_reg[1] >> 8) & 0xFF;
    write_data[3] = (gyro_bias_reg[1]) & 0xFF;
    write_data[4] = (gyro_bias_reg[2] >> 8) & 0xFF;
    write_data[5] = (gyro_bias_reg[2]) & 0xFF;

    // Push gyro biases to hardware registers
    write_byte(mpu_i2c_addr, XG_OFFSET_H, write_data[0]);
    write_byte(mpu_i2c_addr, XG_OFFSET_L, write_data[1]);
    write_byte(mpu_i2c_addr, YG_OFFSET_H, write_data[2]);
    write_byte(mpu_i2c_addr, YG_OFFSET_L, write_data[3]);
    write_byte(mpu_i2c_addr, ZG_OFFSET_H, write_data[4]);
    write_byte(mpu_i2c_addr, ZG_OFFSET_L, write_data[5]);
  }

  void readGyroOffset(int16_t *px, int16_t *py, int16_t *pz)
  {
    int16_t h, l;

    h = read_byte(mpu_i2c_addr, XG_OFFSET_H);
    l = read_byte(mpu_i2c_addr, XG_OFFSET_L);

    *px = ((h & 0x00FF) << 8) | (l & 0x00FF);

    h = read_byte(mpu_i2c_addr, YG_OFFSET_H);
    l = read_byte(mpu_i2c_addr, YG_OFFSET_L);

    *py = ((h & 0x00FF) << 8) | (l & 0x00FF);

    h = read_byte(mpu_i2c_addr, ZG_OFFSET_H);
    l = read_byte(mpu_i2c_addr, ZG_OFFSET_L);

    *pz = ((h & 0x00FF) << 8) | (l & 0x00FF);
  }

  void writeMagOffset(int16_t x, int16_t y, int16_t z)
  {
    mag_bias[0] = (float) x;
    mag_bias[1] = (float) y;
    mag_bias[2] = (float) z;
  }

  bool update(float an, float ae, float ad, float gn, float ge, float gd, float mn, float me, float md)
  {

    // Madgwick function needs to be fed North, East, and Down direction like
    // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
    // Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
    // Magneto direction is Right-Hand, Y-Forward, Z-Down
    // So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
    // we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
    // but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
    // because gravity is by convention positive down, we need to ivnert the accel data

    // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
    // acc[mg], gyro[deg/s], mag [mG]
    // gyro will be convert from [deg/s] to [rad/s] inside of this function
    // quat_filter.update(-a[0], a[1], a[2], g[0] * DEG_TO_RAD, -g[1] * DEG_TO_RAD, -g[2] * DEG_TO_RAD, m[1], -m[0], m[2], q);

    /*
    float an = -a[0];
    float ae = +a[1];
    float ad = +a[2];
    float gn = +g[0] * DEG_TO_RAD;
    float ge = -g[1] * DEG_TO_RAD;
    float gd = -g[2] * DEG_TO_RAD;
    float mn = +m[1];
    float me = -m[0];
    float md = +m[2];
    */

    for (size_t i = 0; i < n_filter_iter; ++i) {
      quat_filter.update(an, ae, ad, gn, ge, gd, mn, me, md, q);
    }

    update_rpy(q[0], q[1], q[2], q[3]);

    return true;
  }
};

#endif
