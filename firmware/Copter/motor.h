#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor
{
public:
  static int32_t maxGas;
public:
  int32_t gas;
  uint8_t chl;
  uint8_t pin;
  uint8_t modified;
public:
  Motor()
  {
    gas = 0;
    chl = 0;
    pin = 0;
    modified = 0;
  }

  void setGas(int32_t value)
  {
    if(value > maxGas)
      value = maxGas;

    if(value < -maxGas)
      value = -maxGas;

    gas = value;
    modified = 1;
  }

  void addGas(int32_t dx)
  {
    int32_t value = gas;
    value += dx;

    setGas(value);
  }
};

#endif
