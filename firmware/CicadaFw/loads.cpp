#include "pdl.h"

#include "pca9536.h"

PCA9536 exGpio;

void load1(uint8_t state)
{
  if(state)
  {
    exGpio.setState(IO2,IO_HIGH);
  }
  else
  {
    exGpio.setState(IO2,IO_LOW);
  }
}

void load2(uint8_t state)
{
  if(state)
  {
    exGpio.setState(IO3,IO_HIGH);
  }
  else
  {
    exGpio.setState(IO3,IO_LOW);
  }
}

void pdlSetupLoads(pdlDroneState*)
{
  exGpio.setMode(IO_OUTPUT);

  load1(0);
  load2(0);
}

uint8_t pdlSwitchLoad(pdlDroneState*, uint8_t num, uint8_t state)
{
  switch(num)
  {
  case 0:
    load1(state);
    break;
  case 1:
    load2(state);
    break;
  }

  return 1;
}
