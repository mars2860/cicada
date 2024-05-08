#ifndef PDL_PDL_PIDS_H
#define PDL_PDL_PIDS_H

#include "pdl_data.h"

void pdlUpdateAltPid(pdlDroneState*,float);
void pdlUpdateLevelsPid(pdlDroneState*,float);
void pdlUpdateHorVelocityPids(pdlDroneState*,float);
void pdlUpdateAngularRatePids(pdlDroneState*,float);

void pdlSetPidFlag(pdlDroneState*,uint8_t,uint8_t);

uint8_t pdlGetPidVeloXFlag(pdlDroneState*);
void pdlSetPidVeloXFlag(pdlDroneState*,uint8_t);

uint8_t pdlGetPidVeloYFlag(pdlDroneState*);
void pdlSetPidVeloYFlag(pdlDroneState*,uint8_t);

uint8_t pdlGetPidAltFlag(pdlDroneState*);
void pdlSetPidAltFlag(pdlDroneState*,uint8_t);

uint8_t pdlGetPidHeadingFlag(pdlDroneState*);
void pdlSetPidHeadingFlag(pdlDroneState*,uint8_t);

uint8_t pdlGetPidPosNorthFlag(pdlDroneState*);
void pdlSetPidPosNorthFlag(pdlDroneState*,uint8_t);

uint8_t pdlGetPidPosEastFlag(pdlDroneState*);
void pdlSetPidPosEastFlag(pdlDroneState*,uint8_t);

uint8_t pdlGetPidPitchFlag(pdlDroneState*);
void pdlSetPidPitchFlag(pdlDroneState*,uint8_t);

uint8_t pdlGetPidRollFlag(pdlDroneState*);
void pdlSetPidRollFlag(pdlDroneState*,uint8_t);

#endif
