### About

Cicada project is a quadcopter based on ESP8266 Wifi module. You can make it yourself to come in the world of drone developers. Cicada doesn't have a hardware pult with sticks. To control a drone there is desktop application developed on Java  

### Hardware

The hardware is designed to be easy assembled by hands. Parts you need
- ESP8266 wifi module
- MT3608 step-up converter
- 1117-3.3 low drop out regulator 3v3
- GY-91 module (10DOF sensor based on MPU9250 and BMP280)
- Lidar module VL53L1X
- Optical Flow module PMW3901
- SMD-0805 resistors and capacitors
- SMD coil SPM4020T-4R7
- Motors and escaper. It is supported DSHOT150, DSHOT300, PWM protocols by firmware

### Firmware

The firmware is based on PDL. Portable Drone Logic (PDL) is the lightweight pure C library implements a common drone logic. The goal of PDL is to be easy to use and easy to port to your hardware platform. PDL supports only quadcopters at this time

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

ssid_config.h

    #ifndef SSID_CONFIG_H
    #define SSID_CONFIG_H

    #define STASSID                         "MyHomeWifi"
    #define STAPSK                          "MyHomeWifiPassword"
    #define DEFAULT_IP_ADDRESS              "192.168.1.33"
    #define DEFAULT_GATEWAY_ADDRESS         "192.168.1.1"
    #define DEFAULT_SUBNET                  "255.255.255.0"

    #endif

Firmware supports update by the air. It is based on ESP8266HTTPUpdateServer

### Design tools
- Sloeber IDE
- FreeCAD
- KiCAD
