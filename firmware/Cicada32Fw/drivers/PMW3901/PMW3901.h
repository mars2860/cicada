#ifndef __PMW3901_H__
#define __PMW3901_H__

#include "Arduino.h"

#include <stdint.h>

class PMW3901
{
public:
  virtual ~PMW3901() {};

  virtual boolean begin(void);

  void readMotionCount(int16_t *deltaX, int16_t *deltaY, uint8_t *squal);
  void enableFrameBuffer();
  void readFrameBuffer(char *FBuffer);

  uint8_t readProdId();
  uint8_t readRevId();

  void setLed(bool ledOn);

protected:
  virtual void cs_high() = 0;
  virtual void cs_low() = 0;
  void registerWrite(uint8_t reg, uint8_t value);
  uint8_t registerRead(uint8_t reg);
  uint8_t registerFastRead(uint8_t reg);
  void initRegisters(void);
};

#endif
