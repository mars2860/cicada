#include "pdl.h"
#include "main.h"

#ifdef CICADA_GY91
  #include "load_cicada_gy91.h"
#else

void pdlSetupLoads(pdlDroneState*)
{
  // no
}

uint8_t pdlSwitchLoad(pdlDroneState*, uint8_t num, uint8_t state)
{
  return 0;
}

#endif
