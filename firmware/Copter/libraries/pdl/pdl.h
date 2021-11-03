/**
 *  @mainpage Introduction
 *
 *  Portable Drone Logic (PDL) is the lightweight pure C library implements a common drone logic. The goal of
 *  this project is to be easy to use and easy to port to your hardware platform. PDL supports only quadrocopters
 *  at this time
 *
 *  It provides for you
 *  - data model
 *  - application template
 *  - task scheduler
 *  - chain of PIDs
 *  - parser for binary digital commands
 *
 *  You need to implement yourself on your platform
 *  - sensors reading
 *  - sensors fusing
 *  - motor speed regulation (PWM, DSHOT, etc)
 *  - remote commands receiving
 *  - telemetry sending to host or saving to disk
 *  - logging
 *  - firmware update process
 *
 *  System requirements:
 *  - Float data processing
 *  - 1 kB of RAM
 *
 * Pages
 *  - @subpage porting "How to port"
 *  - @subpage task_scheduler "Task scheduler"
 *  - @subpage chain_of_pids "Chain of PIDs"
 *  - @subpage data_model "Data model"
 *  - @subpage commands "Binary commands"
 *
 * Usage
 * - @ref pdlData
 * - @ref pdlInterface
 * - @ref pdlPortableInterface
 * - @ref pdlConfig
 * - @ref pdlCmds
 *
 *  @author anton.sysoev.ru68@gmail.com
 *  @copyright GNU Public License
 */

/**
   * @page task_scheduler Task scheduler
   *
   * - Task needs some cpu time to be executed
   * - One task can't be interrupted by other task
   * - Task runs periodically as it defined by its update period in pdl_config.h
   *
   * The goal of this task scheduler is to prevent a situation when in one moment periods of many
   * task are elapsed simultaneously. In this situation update period will be very long. To prevent
   * this situation each task has priority. We have PDL_DESIRE_UPDATE_TIME. If in one moment cpu has
   * to execute many tasks and these tasks take time more than PDL_DESIRE_UPDATE_TIME thats low
   * priority tasks would be put aside for next pdlUpdate. In other word period of low priority
   * tasks will be increased automatically. If in the next pdlUpdate we have the same situations thats low priority
   * tasks would be put aside again or not. It is regulated by PDL_TASK_MAX_WAIT_TIME. Task scheduler guarantees
   * that task would be executed at some time. There is no situation that task never be executed.
   * Priority of task is order of task handler invoke. High priority task are invoked first
   */

/**
 * @page chain_of_pids Chain of PIDs
 *
 * @image html chain_of_pids.svg "Chain of PIDs" width=100%
 */

#ifndef PDL_H
#define PDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pdl_data.h"
#include "pdl_config.h"
#include "pdl_rc_cmds.h"
#include "pdl_port.h"

/// @defgroup pdlInterface Main functions

void pdlSetup(pdlDroneState*);
void pdlUpdate(pdlDroneState*);

/// @return cur - last with overflow correction
uint32_t pdlGetDeltaTime(uint32_t cur, uint32_t last);

void pdlStopMotors(pdlDroneState*);
void pdlSetMotorGas(pdlDroneState*, uint8_t, int32_t);
void pdlAddMotorGas(pdlDroneState*, uint8_t, int32_t);

/** @param dt delta time in seconds */
void pdlUpdatePid(pdlPidState*, float input, float dt);
void pdlResetPid(pdlPidState*);

/** Allow to user to change telemetry period or fully disable telemetry if he set this period as long as it possible */
void pdlSetTelemetryUpdatePeriod(uint32_t);
uint32_t pdlGetTelemetryUpdatePeriod(void);

/// Parses binary packet with command from remote control and applies result to drone state
void pdlParseCommand(pdlDroneState*,uint8_t*);

/// @}

#ifdef __cplusplus
}
#endif

#endif
