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

#define DEFAULT_TELEMETRY_PERIOD        50000
// max packet size may be 512, this size is resctricted by esp8266 platform
#define TELEMETRY_MAX_PACKET_SIZE       512

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

uint32_t telemetryPeriod;

//---------------------------------------------------------------
// COMMANDS

#define CMD_SWITCH_MOTORS         100
#define CMD_SET_MOTORS_GAS        101
#define CMD_SET_ACCEL_OFFSET      102
#define CMD_SET_GYRO_OFFSET       103
#define CMD_SET_MAGNET_OFFSET     104
#define CMD_SELF_CALIB_ACCEL      105
#define CMD_SELF_CALIB_GYRO       106
#define CMD_SET_YAW_RATE_PID      107
#define CMD_SET_PITCH_PID         108
#define CMD_SET_ROLL_PID          109
#define CMD_SET_ALT_PID           110
#define CMD_SET_YPR               111
#define CMD_SET_PERIODS           112
#define CMD_ENABLE_STABILIZATION  113
#define CMD_RESET_ALTITUDE        114
#define CMD_SET_SEA_LEVEL         115
#define CMD_SET_ALTITUDE          116
#define CMD_SET_BASE_GAS          117
#define CMD_SET_PITCH_RATE_PID    118
#define CMD_SET_ROLL_RATE_PID     119

/// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);
void processCommand(pdlDroneState*);
void processTelemetry(pdlDroneState*);

extern void imuCalibrateAccel(pdlDroneState *ds);
extern void imuCalibrateGyro(pdlDroneState *ds);

void pdlSetupRc(pdlDroneState*)
{
  telemetryPort = DEFAULT_TELEMETRY_PORT;
  cmdPort = DEFAULT_CMD_PORT;
  telemetryPeriod = DEFAULT_TELEMETRY_PERIOD;

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
    float kp,ki,kd,maxOut;
    uint8_t enabled;
    switch(cmd)
    {
      case CMD_SWITCH_MOTORS:
        pdlStopMotors(ds);
        ds->motorsEnabled = udpPacket[1];
        if(ds->motorsEnabled)
        {
          ds->yawRatePid.target = 0;
          ds->pitchRatePid.target = 0;
          ds->rollRatePid.target = 0;
          ds->pitchPid.target = 0;
          ds->rollPid.target = 0;
        }
        break;
      case CMD_SET_BASE_GAS:
        memcpy(&t0, &udpPacket[1], sizeof(int32_t));
        ds->baseGas = t0;
        if(!ds->stabilizationEnabled)
        {
          pdlSetMotorGas(ds,0,ds->baseGas);
          pdlSetMotorGas(ds,1,ds->baseGas);
          pdlSetMotorGas(ds,2,ds->baseGas);
          pdlSetMotorGas(ds,3,ds->baseGas);
        }
        break;
      case CMD_SET_MOTORS_GAS:
        memcpy(&t0, &udpPacket[1], sizeof(int32_t));
        memcpy(&t1, &udpPacket[5], sizeof(int32_t));
        memcpy(&t2, &udpPacket[9], sizeof(int32_t));
        memcpy(&t3, &udpPacket[13], sizeof(int32_t));

        pdlSetMotorGas(ds,0,t0);
        pdlSetMotorGas(ds,1,t1);
        pdlSetMotorGas(ds,2,t2);
        pdlSetMotorGas(ds,3,t3);

        break;
      case CMD_SET_ACCEL_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));

        ds->accel.offset[PDL_X] = dx;
        ds->accel.offset[PDL_Y] = dy;
        ds->accel.offset[PDL_Z] = dz;

        pdlSetupAccel(ds);
        break;
      case CMD_SET_GYRO_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));

        ds->gyro.offset[PDL_X] = dx;
        ds->gyro.offset[PDL_Y] = dy;
        ds->gyro.offset[PDL_Z] = dz;

        pdlSetupGyro(ds);
        break;
      case CMD_SET_MAGNET_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));

        ds->magneto.offset[PDL_X] = dx;
        ds->magneto.offset[PDL_Y] = dy;
        ds->magneto.offset[PDL_Z] = dz;

        pdlSetupMagneto(ds);
        break;
      case CMD_SELF_CALIB_ACCEL:
        imuCalibrateAccel(ds);
        break;
      case CMD_SELF_CALIB_GYRO:
        imuCalibrateGyro(ds);
        break;
      case CMD_SET_YAW_RATE_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        memcpy(&maxOut, &udpPacket[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
        ds->yawRatePid.enabled = enabled;
        ds->yawRatePid.kp = kp;
        ds->yawRatePid.ki = ki;
        ds->yawRatePid.kd = kd;
        ds->yawRatePid.maxOut = maxOut;
        break;
      case CMD_SET_PITCH_RATE_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        memcpy(&maxOut, &udpPacket[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
        ds->pitchRatePid.enabled = enabled;
        ds->pitchRatePid.kp = kp;
        ds->pitchRatePid.ki = ki;
        ds->pitchRatePid.kd = kd;
        ds->pitchRatePid.maxOut = maxOut;
        break;
      case CMD_SET_ROLL_RATE_PID:
         enabled = udpPacket[1];
         memcpy(&kp, &udpPacket[2], sizeof(kp));
         memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
         memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
         memcpy(&maxOut, &udpPacket[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
         ds->rollRatePid.enabled = enabled;
         ds->rollRatePid.kp = kp;
         ds->rollRatePid.ki = ki;
         ds->rollRatePid.kd = kd;
         ds->rollRatePid.maxOut = maxOut;
         break;
      case CMD_SET_PITCH_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        memcpy(&maxOut, &udpPacket[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
        ds->pitchPid.enabled = enabled;
        ds->pitchPid.kp = kp;
        ds->pitchPid.ki = ki;
        ds->pitchPid.kd = kd;
        ds->pitchPid.maxOut = maxOut;
        break;
      case CMD_SET_ROLL_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        memcpy(&maxOut, &udpPacket[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
        ds->rollPid.enabled = enabled;
        ds->rollPid.kp = kp;
        ds->rollPid.ki = ki;
        ds->rollPid.kd = kd;
        ds->rollPid.maxOut = maxOut;
        break;
      case CMD_SET_ALT_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        memcpy(&maxOut, &udpPacket[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
        ds->altPid.enabled = enabled;
        ds->altPid.kp = kp;
        ds->altPid.ki = ki;
        ds->altPid.kd = kd;
        ds->altPid.maxOut = maxOut;
        break;
      case CMD_SET_YPR:
        memcpy(&kp, &udpPacket[1], sizeof(kp));
        memcpy(&ki, &udpPacket[1 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[1 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        ds->yawRatePid.target = kp;

        if(ds->pitchPid.enabled)
          ds->pitchPid.target = ki;
        else
          ds->pitchRatePid.target = ki;

        if(ds->rollPid.enabled)
          ds->rollPid.target = kd;
        else
          ds->rollRatePid.target = kd;
        break;
      case CMD_SET_PERIODS:
        memcpy(&telemetryPeriod, &udpPacket[1], sizeof(telemetryPeriod));
        //memcpy(&pidPeriod, &udpPacket[1 + sizeof(telemetryPeriod)], sizeof(pidPeriod));
        if(telemetryPeriod < 1)
          telemetryPeriod = 1;
        //if(pidPeriod < 1)
        //  pidPeriod = 1;
        //yawRatePid.setTimeStep(pidPeriod);
        //pitchPid.setTimeStep(pidPeriod);
        //rollPid.setTimeStep(pidPeriod);
        //altPid.setTimeStep(pidPeriod);
        break;
      case CMD_ENABLE_STABILIZATION:
        ds->stabilizationEnabled = udpPacket[1];
        break;
      case CMD_RESET_ALTITUDE:
        //vz = 0;
        ds->seaLevel = ds->pressure;
        break;
      case CMD_SET_SEA_LEVEL:
        memcpy(&ds->seaLevel, &udpPacket[1], sizeof(ds->seaLevel));
        break;
      case CMD_SET_ALTITUDE:
        memcpy(&ds->altPid.target, &udpPacket[1], sizeof(ds->altPid.target));
        break;
    }
  }
}

void processTelemetry(pdlDroneState *ds)
{
  if(!host.isSet())
    return;

  uint16_t pos = 0;
  size_t sz = sizeof(pdlDroneState);

  ds->timestamp = pdlMicros();
  ds->rc.rssi = WiFi.RSSI();

  // DroneState size
  pos = writeTelemetryPacket(pos, udpPacket, &sz, sizeof(sz));
  // DroneState
  pos = writeTelemetryPacket(pos, udpPacket, ds, sz);
  // Telemetry Period
  pos = writeTelemetryPacket(pos, udpPacket, &telemetryPeriod, sizeof(telemetryPeriod));

  udp.beginPacket(host, telemetryPort);
  udp.write(udpPacket, pos);
  udp.endPacket();
}
