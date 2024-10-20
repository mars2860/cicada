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
 * results from your sensors. The drone is a zero point of coordinate system. In this case the drone don't moves but
 * ground moves around drone. Remember it when you will provide data from your sensors to set right sign of position,
 * velocity or acceleration. <a href="https://khandronesorigin.com/2016/11/28/u-a-v-coordinate-system-transformation/">Read</a>
 * this good article to understand drone's coordinate systems.
 *
 * Body fixed frame
 *
 * @image html pdl_coordinate_system.jpg "PDL body coordinate system" width=100%
 *
 * World inertial frame
 *
 * @image html enu_frame.svg "PDL world coordinate system" width=100%
 *
 * @defgroup pdlPortableInterface Portable functions
 * @{
 *
 */

/// @return microseconds from system has been started
uint32_t pdlMicros(void);

void pdlSetupRc(pdlDroneState*);
void pdlSetupEsc(pdlDroneState*);
void pdlSetupAccel(pdlDroneState*);
void pdlSetupGyro(pdlDroneState*);
void pdlSetupMagneto(pdlDroneState*);
void pdlSetupBaro(pdlDroneState*);
void pdlSetupBattery(pdlDroneState*);
void pdlSetupLidar(pdlDroneState*);
void pdlSetupOpticalFlow(pdlDroneState*);
void pdlSetupTelemetry(pdlDroneState*);
void pdlSetupGps(pdlDroneState*);
void pdlSetupLoads(pdlDroneState*);

void pdlCalibrateAccel(pdlDroneState*);
void pdlCalibrateGyro(pdlDroneState*);

void pdlLoadDefaultCfg(pdlDroneState *ds);
void pdlSaveDefaultCfg(pdlDroneState *ds);

void pdlSetupCamera(pdlDroneState *ds);
void pdlUpdateCamera(pdlDroneState *ds);
void pdlSetCameraAngle(pdlDroneState *ds, int32_t angle);
void pdlStartVideo(pdlDroneState *ds);
void pdlStopVideo(pdlDroneState *ds);
void pdlTakePhoto(pdlDroneState *ds);

/// Implements your telemetry in/out
void pdlUpdateTelemetry(pdlDroneState*);
/// It has to take motorGas from pdlDroneState and send it to ESC
void pdlUpdateEsc(pdlDroneState*);
/// Implements your remote control. RC state has to be saved in pdlDroneState.rc
void pdlRemoteControl(pdlDroneState*);
/// Result has to be saved in pdlDroneState.battery
void pdlReadBattery(pdlDroneState*);

/** Read accel sensor inside this function. To provide the system new data
 *  call pdlNewAccelData inside this function
 *
 *  @return 1 if it is success
 */
uint8_t pdlReadAccel(pdlDroneState*);

/** Read gyro sensor inside this function. To provide the system new data
 *  call pdlNewGyroData inside this function
 *
 *  @return 1 if it is success
 */
uint8_t pdlReadGyro(pdlDroneState*);

/** Read magneto sensor inside this function. To provide the system new data
 *  call pdlNewMagnetoData inside this function
 *
 *  @return 1 if it is success
 */
uint8_t pdlReadMagneto(pdlDroneState*);

/** Read baro sensor inside this function. To provide the system new data
 *  call pdlNewBaroData inside this function
 *
 *  @return 1 if it is success
 */
uint8_t pdlReadBaro(pdlDroneState*);

/** Read lidar sensor inside this function. To provide the system new data
 *  call pdlNewLidarData inside this function
 *
 *  @return 1 if it is success
 */
uint8_t pdlReadLidar(pdlDroneState*);

/** Read optical flow sensor inside this function. To provide the system new data
 *  call pdlNewOpticalFlowData inside this function
 *
 *  @return 1 if it is success
 */
uint8_t pdlReadOpticalFlow(pdlDroneState*);

/** Read GPS sensor inside this function. To provide the system new data
 *  call pdlNewGPSData inside this function
 *
 *  @return 1 if it is success
 */
uint8_t pdlReadGps(pdlDroneState*);

/** Implement switch on/off your loads (led,buzzer,drop-machine, etc) inside this function
 * @param num     Load num
 * @param state   Load state
 *
 *  @return 1 if it is success
 */
uint8_t pdlSwitchLoad(pdlDroneState *ds, uint8_t num, uint8_t state);

/// Callback function. It is invoked when new user data packet has been received
//void pdlCallbackUserDataReceived(pdlDroneState*);

/** Inside this function you have to convert XYZ-Pids output to motors pwm, dshot, etc
 *  You may use one of implemented function for quadcopter:
 *    - pdlPidsToCrossFrame
 *    - pdlPidsToXFrame
 *  for example:
 *  [code]
 *  void pdlPidsToMotors(pdlDroneState *ds)
 *  {
 *    pdlPidsToXFrame(ds);
 *  }
 *  [/code]
 */
void pdlPidsToMotors(pdlDroneState*);

/** Sets motors spin direction
 *
 * @param ds Drone state
 * @param dir 0 - for normal direction, 1 - for reversed direction
 */
void pdlSetMotorsDir(pdlDroneState *ds, uint8_t dir);

/** Sets new WiFi SSID
 * @param ds Drone state
 * @param packet Received from host packet
 */
void pdlCmdSetSsid(pdlDroneState *ds,const uint8_t* packet);

/** Sets new WiFi PSK
 * @param ds Drone state
 * @param packet Received from host packet
 */
void pdlCmdSetPsk(pdlDroneState *ds,const uint8_t* packet);

/** Sets new static IP of drone
 * @param ds Drone state
 * @param packet Received from host packet
 */
void pdlCmdSetIp(pdlDroneState *ds, const uint8_t* packet);

/** Sets new gateway
 * @param ds Drone state
 * @param packet Received from host packet
 */
void pdlCmdSetGateway(pdlDroneState *ds, const uint8_t* packet);

/** Sets new mask of subnet
 * @param ds Drone state
 * @param packet Received from host packet
 */
void pdlCmdSetSubnet(pdlDroneState*ds, const uint8_t* packet);

/** Setups WiFi channel, mode, tx power, tx rate
 *  @param ds Drone state
 *  @param packet Received from host packet
 */
void pdlCmdSetupWifi(pdlDroneState* ds, const uint8_t* packet);

/** Enables WiFi broadcast mode */
void pdlCmdEnableWifiBroadcast(pdlDroneState* ds, const uint8_t* packet);

/** Set drone id */
void pdlCmdSetDroneId(pdlDroneState* ds, const uint8_t* packet);

/// @}

#endif
