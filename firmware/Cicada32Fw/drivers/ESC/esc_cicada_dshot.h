#ifndef DRIVERS_ESC_ESC_CICADA_DSHOT_H_
#define DRIVERS_ESC_ESC_CICADA_DSHOT_H_

#include "pdl.h"
#include "dshot.h"

// Teeny pro 5A Esc
#define M1_CICADA_0703_PIN              15
#define M2_CICADA_0703_PIN              2
#define M3_CICADA_0703_PIN              16
#define M4_CICADA_0703_PIN              0

// Skystars KO25
#define M1_KO25_PIN              0
#define M2_KO25_PIN              16
#define M3_KO25_PIN              2
#define M4_KO25_PIN              15

void pdlUpdateDefaultDshotEsc(pdlDroneState *ds)
{
  uint8_t i;
  uint8_t modified = 0;
  static int32_t oldMotorGas[PDL_MOTOR_COUNT] = {0,0,0,0};

  for(i = 0; i < PDL_MOTOR_COUNT ; i++)
  {
    if(ds->motorGas[i] != oldMotorGas[i])
    {
      modified = 1;
      oldMotorGas[i] = ds->motorGas[i];
    }
  }

  if(modified)
  {
    dshotSet( ds->motorGas[0] + 48,
              ds->motorGas[1] + 48,
              ds->motorGas[2] + 48,
              ds->motorGas[3] + 48);
  }

  if(ds->motorsEnabled != dshotIsEnabled())
  {
    delay(2);   // wait for motors stop
    dshotEnable(ds->motorsEnabled);
  }
}

#endif /* DRIVERS_ESC_ESC_CICADA_DSHOT_H_ */
