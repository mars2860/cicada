#ifndef PDL_PDL_TASKS_H
#define PDL_PDL_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct s_pdlTask
{
  uint32_t lastUpdateTime;
  uint32_t period;
  /// real time between executing of task
  uint32_t realPeriod;
  /// task time execution in us
  uint32_t execTime;
  /// system time at task begins
  uint32_t startTime;
} pdlTaskState;

uint8_t pdlCanTaskRun(pdlTaskState*,uint8_t);
void pdlTaskBegin(pdlTaskState*);
void pdlTaskEnd(pdlTaskState*);

void pdlSetupTasks();

void pdlGyroTask(pdlDroneState*);
void pdlAccelTask(pdlDroneState*);
void pdlMagTask(pdlDroneState*);
void pdlOpticalFlowTask(pdlDroneState*);
void pdlLidarTask(pdlDroneState*);
void pdlBaroTask(pdlDroneState*);
void pdlPidTask(pdlDroneState*);
void pdlEscTask(pdlDroneState*);
void pdlRcTask(pdlDroneState*);
void pdlTelemetryTask(pdlDroneState*);
void pdlBatteryTask(pdlDroneState*);
void pdlGpsTask(pdlDroneState*);
void pdlLoadTask(pdlDroneState*);

void pdlResetKalman(pdlDroneState *ds);

#ifdef __cplusplus
}
#endif

#endif
