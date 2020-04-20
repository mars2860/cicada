#include "pdl.h"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>

#define STASSID                         "liftelectronica"
#define STAPSK                          "cosmos327"
#define DEFAULT_IP_ADDRESS              "192.168.1.33"
#define DEFAULT_GATEWAY_ADDRESS         "192.168.1.1"
#define DEFAULT_SUBNET                  "255.255.255.0"

#define DEFAULT_CMD_PORT                4210
#define DEFAULT_TELEMETRY_PORT          4211

#define DEFAULT_TELEMETRY_PERIOD        100
#define TELEMETRY_MAX_PACKET_SIZE       255

IPAddress ip;
IPAddress gateway;
IPAddress subnet;
IPAddress host;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiUDP udp;
uint8_t udpPacket[TELEMETRY_MAX_PACKET_SIZE];

uint32_t cmdPort;
uint32_t telemetryPort;

//---------------------------------------------------------------
// COMMANDS

#define CMD_SWITCH_MOTORS         100
#define CMD_SET_MOTORS_GAS        101
#define CMD_SET_ACCEL_OFFSET      102
#define CMD_SET_GYRO_OFFSET       103
#define CMD_SET_MAGNET_CALIB      104
#define CMD_SELF_CALIB_ACCEL      105
#define CMD_SELF_CALIB_GYRO       106
#define CMD_SET_YAW_PID           107
#define CMD_SET_PITCH_PID         108
#define CMD_SET_ROLL_PID          109
#define CMD_SET_ALT_PID           110
#define CMD_SET_YPR               111
#define CMD_SET_PERIODS           112
#define CMD_ENABLE_STABILIZATION  113
#define CMD_RESET_ALTITUDE        114
#define CMD_SET_SEA_LEVEL         115
#define CMD_SET_ALTITUDE          116

/// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);
void processCommand(pdlDroneState*);
void processTelemetry(pdlDroneState*);

void pdlSetupRc(pdlDroneState*)
{
  telemetryPort = DEFAULT_TELEMETRY_PORT;
  cmdPort = DEFAULT_CMD_PORT;

  ip.fromString(DEFAULT_IP_ADDRESS);
  gateway.fromString(DEFAULT_GATEWAY_ADDRESS);
  subnet.fromString(DEFAULT_SUBNET);

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  WiFi.config(ip, gateway, subnet);

  while(WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  udp.begin(cmdPort);

  httpUpdater.setup(&httpServer);
  httpServer.begin();
}

void pdlRemoteControl(pdlDroneState *ds)
{
  static uint32_t telemetryLastUpdateTime = 0;

  processCommand(ds);

  uint32_t timestamp = micros();

  if(timestamp - telemetryLastUpdateTime >= telemetryPeriod)
  {
    telemetryLastUpdateTime = timestamp;
    processTelemetry(ds);
  }

  httpServer.handleClient();
}

uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize)
{
  if(pos + valueSize >= TELEMETRY_MAX_PACKET_SIZE)
    return pos;

  memcpy(&packet[pos], value, valueSize);
  pos += valueSize;
  return pos;
}

void processCommand(pdlDroneState *ds)
{
  if(udp.parsePacket())
  {
    udp.read(udpPacket, sizeof(udpPacket));
    host = udp.remoteIP();
    uint8_t cmd = udpPacket[0];
    int16_t dx,dy,dz;
    int32_t t0,t1,t2,t3;
    float kp,ki,kd;
    uint8_t enabled;
    switch(cmd)
    {
      case CMD_SWITCH_MOTORS:
        copterState.motorsEnabled = udpPacket[1];
        if(copterState.motorsEnabled)
        {
          yawRatePid.setTarget(0);
          pitchPid.setTarget(0);
          rollPid.setTarget(0);
        }
        copterState.baseGas = 0;
        motor[0].setGas(0);
        motor[1].setGas(0);
        motor[2].setGas(0);
        motor[3].setGas(0);
        break;
      case CMD_SET_MOTORS_GAS:
        memcpy(&t0, &udpPacket[1], sizeof(int32_t));
        memcpy(&t1, &udpPacket[5], sizeof(int32_t));
        memcpy(&t2, &udpPacket[9], sizeof(int32_t));
        memcpy(&t3, &udpPacket[13], sizeof(int32_t));
        copterState.baseGas = (t0 + t1 + t2 + t3)/4;
        if(!copterState.enStabilization)
        {
          motor[0].setGas(t0);
          motor[1].setGas(t1);
          motor[2].setGas(t2);
          motor[3].setGas(t3);
        }
        /*else
        {
          int32_t dx = (motor[0].gas + motor[1].gas + motor[2].gas + motor[3].gas)/4;
          motor[0].addGas(baseGas - dx);
          motor[1].addGas(baseGas - dx);
          motor[2].addGas(baseGas - dx);
          motor[3].addGas(baseGas - dx);
        }*/
        break;
      case CMD_SET_ACCEL_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));
        //mpu.setDMPEnabled(false);
        mpu.setXAccelOffset(dx);
        mpu.setYAccelOffset(dy);
        mpu.setZAccelOffset(dz);
        //mpu.setDMPEnabled(true);
        readMpuOffsets();
        break;
      case CMD_SET_GYRO_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));
        //mpu.setDMPEnabled(false);
        mpu.setXGyroOffset(dx);
        mpu.setYGyroOffset(dy);
        mpu.setZGyroOffset(dz);
        //mpu.setDMPEnabled(true);
        readMpuOffsets();
        break;
      case CMD_SET_MAGNET_CALIB:
        memcpy(&copterState.magnet.offset[0], &udpPacket[1], 6);
        //memcpy(&magnetScale[0], &udpPacket[7], 12);
        break;
      case CMD_SELF_CALIB_ACCEL:
        mpu.CalibrateAccel(10);
        readMpuOffsets();
        break;
      case CMD_SELF_CALIB_GYRO:
        mpu.CalibrateGyro(10);
        readMpuOffsets();
        break;
      case CMD_SET_YAW_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        yawRatePid.enabled = enabled;
        yawRatePid.setGains(kp,ki,kd);
        break;
      case CMD_SET_PITCH_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        pitchPid.enabled = enabled;
        pitchPid.setGains(kp,ki,kd);
        break;
      case CMD_SET_ROLL_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        rollPid.enabled = enabled;
        rollPid.setGains(kp,ki,kd);
        break;
      case CMD_SET_ALT_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        altPid.enabled = enabled;
        altPid.setGains(kp,ki,kd);
        break;
      case CMD_SET_YPR:
        memcpy(&kp, &udpPacket[1], sizeof(kp));
        memcpy(&ki, &udpPacket[1 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[1 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        yawRatePid.setTarget(kp);
        pitchPid.setTarget(ki);
        rollPid.setTarget(kd);
        break;
      case CMD_SET_PERIODS:
        memcpy(&copterState.telemetryPeriod, &udpPacket[1], sizeof(copterState.telemetryPeriod));
        //memcpy(&pidPeriod, &udpPacket[1 + sizeof(telemetryPeriod)], sizeof(pidPeriod));
        if(copterState.telemetryPeriod < 1)
          copterState.telemetryPeriod = 1;
        //if(pidPeriod < 1)
        //  pidPeriod = 1;
        //yawRatePid.setTimeStep(pidPeriod);
        //pitchPid.setTimeStep(pidPeriod);
        //rollPid.setTimeStep(pidPeriod);
        //altPid.setTimeStep(pidPeriod);
        break;
      case CMD_ENABLE_STABILIZATION:
        copterState.enStabilization = udpPacket[1];
        break;
      case CMD_RESET_ALTITUDE:
        //vz = 0;
        copterState.seaLevelhPa = baro.pressure;
        break;
      case CMD_SET_SEA_LEVEL:
        memcpy(&copterState.seaLevelhPa, &udpPacket[1], sizeof(copterState.seaLevelhPa));
        break;
      case CMD_SET_ALTITUDE:
        memcpy(&altPid.target, &udpPacket[1], sizeof(altPid.target));
        break;
    }
  }
}

void processTelemetry(pdlDroneState *ds)
{
  size_t packetSize = sizeof(copterState);

  if(packetSize > TELEMETRY_MAX_PACKET_SIZE)
    packetSize = TELEMETRY_MAX_PACKET_SIZE;

  copterState.rssi = WiFi.RSSI();

  memcpy(&udpPacket[0], &copterState, packetSize);

  udp.beginPacket(host, telemetryPort);
  udp.write(udpPacket, packetSize);
  udp.endPacket();
}
