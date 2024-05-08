#ifndef HCSR04_I2C_H
#define HCSR04_I2C_H

#define HC_SR04_I2C_ADDR 0x57

// this sonar works only on 100 kHz of I2C
class HCSR04_I2C
{
  public:
    HCSR04_I2C();
    void measure();
    float getRange_m();
    int getRange_mm();
};

#endif
    
