#ifndef DRIVERS_ESC_ESC_CICADA_GY91_H_
#define DRIVERS_ESC_ESC_CICADA_GY91_H_

#include "esc_cicada_dshot.h"

bool pdlSetupPwmEsc(pdlDroneState*, int32_t maxGas, int32_t nullGas, int32_t minGas)
{
  pdlSetMotorMaxGas(maxGas);
  pdlSetMotorNullGas(nullGas);
  pdlSetMotorMinGas(minGas);

  // pwm esc is not supported by this board

  return false;
}

void pdlUpdatePwmEsc(pdlDroneState *ds)
{
  // pwm esc is not supported by this board
}

#endif /* DRIVERS_ESC_ESC_CICADA_GY91_H_ */
