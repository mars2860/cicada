#ifndef PDL_PORT_H
#define PDL_PORT_H

#include "pdl_data.h"

/**
 * @page porting How to port
 *
 * First of all you need to declare one instance of pdlDroneState in global memory space. Next you have
 * to implement all functions defined in this file. After these steps in start of your code invoke pdlSetup
 * with your global instance of pdlDroneState. Invoke pdlUpdate inside the loop statement
 *
 * For example
 *
 *     #include "pdl.h"
 *
 *     pdlDroneState ds;
 *
 *     void main(void)
 *     {
 *       pdlSetup(&ds);
 *
 *       while(1)
 *       {
 *         pdlUpdate(&ds);
 *       }
 *     }
 *
 *     pdlSetupAccel(pdlDroneState *pds) { ... }
 *     pdlReadAccel(pdlDroneState *pds) { ... }
 *
 * @ref pdlPortableInterface
 *
 * PDL uses right-handed coordinate system with pozitive Z-axis is down toward Earth. You have to be according to this coordinate system when store in pdlDroneState
 * results from your sensors.
 *
 * @image html pdl_coordinate_system.jpg "PDL coordinate system" width=100%
 *
 * @defgroup pdlPortableInterface Portable functions
 * @{
 *
 */

/// @return microseconds from system has been started
uint32_t pdlMicros(void);

void pdlSetupRc(pdlDroneState*);
void pdlSetupEscaper(pdlDroneState*);
void pdlSetupAccel(pdlDroneState*);
void pdlSetupGyro(pdlDroneState*);
void pdlSetupMagneto(pdlDroneState*);
void pdlSetupBaro(pdlDroneState*);
void pdlSetupBattery(pdlDroneState*);
void pdlSetupLidar(pdlDroneState*);
void pdlSetupOpticalFlow(pdlDroneState*);
void pdlSetupTelemetry(pdlDroneState*);

void pdlCalibrateAccel(pdlDroneState*);
void pdlCalibrateGyro(pdlDroneState*);

/// It has to take motorGas from pdlDroneState and send it to escaper
void pdlUpdateEscaper(pdlDroneState*);
/// Implements your remote control. RC state has to be saved in pdlDroneState.rc
void pdlRemoteControl(pdlDroneState*);
/// Result has to be saved in pdlDroneState.battery
void pdlReadBattery(pdlDroneState*);
/// Result has to be saved in pdlDroneState.accel
void pdlReadAccel(pdlDroneState*);
/// Result has to be saved in pdlDroneState.gyro
void pdlReadGyro(pdlDroneState*);
/// Result has to be saved in pdlDroneState.magneto
void pdlReadMagneto(pdlDroneState*);
/// Implements your telemetry in/out
void pdlUpdateTelemetry(pdlDroneState*);
/**
 * Result has to be saved in pdlDroneState.baro
 *
 * If baro.altitude valid and you want to use it as altitude to hold vertical position that you have
 * to copy baro.altitude to pdlDroneState.altitude and return 1
 */
uint8_t pdlReadBaro(pdlDroneState*);
/**
 * Result has to be saved in pdlDroneState.lidarRange

 * If lidarRange is valid and you want to use it as altitude to hold vertical position that you have
 * to copy lidarRange to pdlDroneState.altitude and return 1
 */
uint8_t pdlReadLidar(pdlDroneState*);
/// Result has to be saved in pdlDroneState.opticalFlow
void pdlReadOpticalFlow(pdlDroneState*);
/// Result has to be saved in pdlDroneState.yaw,pitch,roll
void pdlTripleAxisSensorFusion(pdlDroneState*);

/// @}

#endif
