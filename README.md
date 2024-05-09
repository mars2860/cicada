### About

The Cicada Project is the flight controller based on ESP8266 Wifi module. You can make it yourself to come in the world of drone developers. Cicada doesn't have a hardware pult with sticks. To control a drone there is desktop application developed on Java and mobile application for Android v7 and higher.

<img src="/photo/cicada-main-v4-top.png" width="50%" alt="cicada-main-v4"/>
<img src="/photo/cicada-micro-v2-top.png" width="50%" alt="cicada-micro-v2"/>
<img src="/photo/desktop.jpg" width="100%" alt="DesktopApp"/>
<img src="/photo/mobile.jpg" width="100%" alt="MobileApp"/>

### Features

|     |     |     |     |
| --- | --- | --- | --- |
|     | **Min** | **Typ** | **Max** |
| **1\. Supply voltage** | 3,4V |     | 5V  |
| **2\. Update frequency** |     | 100 Hz |     |
| **3\. Control from notebook or smartphone through WiFi net.** |     | 2.4 GHz |     |
| **4\. Instant telemetry through WiFi net.** Charts of all parameters. BlackBox is stored on host. Telemetry packets is sent every 10ms. |     |     |     |
| **5\. Control devices:**<br><br>* Gamepad (supported by desktop and mobile applications)<br>* Keyboard (supported by desktop application only)<br>* Virtual keyboard (supported by desktop application only)<br>* Virtual gamepad (supported by mobile application only) |     |     |     |
| **6\. Control range** |     |     | 100 meters |
| **7\. Stabilization modes:**<br><br>* Stabilization of body angular rates<br>* Stabilization of yaw,pitch,roll angles<br>* Stabilization of vertical velocity (only if barometer or lidar are available in the system)<br>* Stabilization of horizontal velocity (only if optical flow sensor or GPS are available in the system) |     |     |     |
| **8\. Supported sensors and devices:**<br><br>* MPU9250 - imu chip<br>* MPU6500 - imu chip<br>* MPU6050 - imu chip<br>* QMC5883L - magnetometer<br>* BMP280 - barometer<br>* VL53L1X - lidar<br>* TFmini Plus (in I2C mode) - lidar<br>* HCSR04 (in I2C mode) - sonar<br>* BN180 and other GPS modules with UBX protocol<br>* Analog 0-3V sensor of battery current<br>* Servo 500-2400us to control camera pitch angle<br>* Camera Runcam Split4 (only supported start/stop video commands) |     |     |     |
| **9\. Electronic speed controllers:**<br><br>* DSHOT-300 to control BLDC motors<br>* All listed below PWM-modesе to direct control coreless dc motors<br>* PWM 2.5 kHz, resolution 2000<br>* PWM 5 kHz, resolution 1000<br>* PWM 10 kHz, resolution 500<br>* PWM 20 kHz, resolution 250<br>* PWM 40 kHz, resolution 125<br>* PWM 50 kHz, resolution 100 |     |     |     |
| **10\. Dimensions**<br><br>* cicada-main-v4<br>* cicada-micro-v2 |     | 58,67mm х 41,65mm  <br>26,67mm x 30,98mm |     |
| **11\. Installation dimensions**<br><br>* cicada-main-v4<br>* cicada-micro-v2 |     | 48,51mm х 31,49mm  <br>21,84mm x 26,16mm |     |
| **12\. Weight**<br><br>* cicada-main-v4 with cicada-gy91-v3<br>* cicada-micro-v2 |     | 16g  <br>6g |     |
	
### Drones

Examples of drones you can find in drones folder of this project.

### Firmware

The firmware is based on PDL. Portable Drone Logic (PDL) is the lightweight pure C library implements a common drone logic. The goal of PDL is to be easy to use and easy to port to your hardware platform. PDL supports only quadcopters at this time.

It provides for you
- data model
- application template
- task scheduler
- chain of PIDs
- parser for binary digital commands

PDL is a part of this project. But you can use it to create firmware for your own hardware platform. Documentation is available in /firmware/CicadaFw/libraries/pdl

To build firmware you need
- Install Sloeber IDE https://github.com/Sloeber/arduino-eclipse-plugin/releases/
- Run Sloeber IDE. Go to Arduino->Preferences->Platforms and Boards. Download latest version of ESP8266 platform
- Install I2C driver https://github.com/enjoyneering/ESP8266-I2C-Driver
- Create ssid_config.h file with your wifi-net settings. For example

Firmware supports update by the air. It is based on ESP8266HTTPUpdateServer.

### Chain of PIDs

<img src="/firmware/CicadaFw/libraries/pdl/chain_of_pids.svg" width="100%" alt="Chain of PIDs"/>

### Design tools

- Sloeber IDE
- FreeCAD
- KiCAD
