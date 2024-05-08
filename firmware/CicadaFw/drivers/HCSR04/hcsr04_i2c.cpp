/*
  This is a library for the HC-SR04 I2C Ultrasonic sensor.
  This version of HC-SR04 uses I2C for communication. Two pins (SCL / SDA)
  are required to interface to the device. This library does not work with 
  HC-SR04 with GPIO interface.
  Written by KC, SGBotic.com 2020.
*/

#include <Wire.h>
#include "hcsr04_i2c.h"

// Define milliseconds delay for ESP8266 platform
#if defined(ESP8266)

  #include <pgmspace.h>
  #define _delay_ms(ms) delayMicroseconds((ms) * 1000)

// Use _delay_ms from utils for AVR-based platforms
#elif defined(__avr__)
  #include <util/delay.h>

// Use Wiring's delay for compability with another platforms
#else
  #define _delay_ms(ms) delay(ms)
#endif


HCSR04_I2C::HCSR04_I2C()
{
}

void HCSR04_I2C::measure()
{
  Wire.beginTransmission(HC_SR04_I2C_ADDR);
  Wire.write(0x1);
  Wire.endTransmission();
}

int HCSR04_I2C::getRange_mm()
{
  int distance = 0;
  int ds[3];

  Wire.requestFrom(HC_SR04_I2C_ADDR,3);  //read distance

  if(Wire.available() < 3)
    return -1.f;

  ds[0] = Wire.read();
  ds[1] = Wire.read();
  ds[2] = Wire.read();

  distance = (ds[0] & 0xFF);
  distance <<= 8;
  distance |= (ds[1] & 0xFF);
  distance <<= 8;
  distance |= (ds[2] & 0xFF);

  distance /= 1000;

  if(distance > 4000)
    return -1;

  return distance;
}

float HCSR04_I2C::getRange_m()
{
  int range = getRange_mm();

  if(range < 0)
    return -1.f;

  return (float)range/1000.f;
}
