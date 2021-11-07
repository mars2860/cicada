#include "pdl.h"
#include "QMC5883L.h"
#include <string.h>

QMC5883L mag;

void pdlSetupMagneto(pdlDroneState*)
{
  mag.init();
  mag.setSamplingRate(200);
  mag.setOversampling(512);
}

void pdlReadMagneto(pdlDroneState *ds)
{
  uint8_t result;

  result = mag.readRaw(&ds->magneto.raw[PDL_X], &ds->magneto.raw[PDL_Y], &ds->magneto.raw[PDL_Z]);

  if(result)
  {
    for(uint8_t i = 0; i < 3; i++)
    {
      //x *= magnetScale[0];
      //y *= magnetScale[1];
      //z *= magnetScale[2];
      ds->magneto.filtered[i] = ds->magneto.raw[i] - ds->magneto.offset[i];;
      ds->magneto.pure[i] = ((float)ds->magneto.filtered[i]) / 12.f;
    }
  }
}
