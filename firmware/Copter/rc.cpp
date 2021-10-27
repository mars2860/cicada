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
#define CMD_SET_VELOCITY_X_PID    120
#define CMD_SET_VELOCITY_Y_PID    121
#define CMD_SET_VELOCITY_Z_PID    122
#define CMD_SET_VELOCITY_Z        123

/// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);
void processCommand(pdlDroneState*);
void processTelemetry(pdlDroneState*);
void parsePidConfigPacket(pdlPidState*, byte*);
void parseTripleAxisSensorConfigPacket(pdlTripleAxisSensorState*, byte*);

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
    int32_t t0,t1,t2,t3;
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
          ds->velocityXPid.target = 0;
          ds->velocityYPid.target = 0;
          ds->velocityZPid.target = 0;
          ds->holdPosEnabled = PDL_HOLDPOS_BOTH_XY;
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
        parseTripleAxisSensorConfigPacket(&ds->accel, udpPacket);
        pdlSetupAccel(ds);
        break;
      case CMD_SET_GYRO_OFFSET:
        parseTripleAxisSensorConfigPacket(&ds->gyro, udpPacket);
        pdlSetupGyro(ds);
        break;
      case CMD_SET_MAGNET_OFFSET:
        parseTripleAxisSensorConfigPacket(&ds->magneto, udpPacket);
        pdlSetupMagneto(ds);
        break;
      case CMD_SELF_CALIB_ACCEL:
        imuCalibrateAccel(ds);
        break;
      case CMD_SELF_CALIB_GYRO:
        imuCalibrateGyro(ds);
        break;
      case CMD_SET_YAW_RATE_PID:
        parsePidConfigPacket(&ds->yawRatePid, udpPacket);
        break;
      case CMD_SET_PITCH_RATE_PID:
        parsePidConfigPacket(&ds->pitchRatePid, udpPacket);
        break;
      case CMD_SET_ROLL_RATE_PID:
        parsePidConfigPacket(&ds->rollRatePid, udpPacket);
        break;
      case CMD_SET_PITCH_PID:
        parsePidConfigPacket(&ds->pitchPid, udpPacket);
        break;
      case CMD_SET_ROLL_PID:
        parsePidConfigPacket(&ds->rollPid, udpPacket);
        break;
      case CMD_SET_ALT_PID:
        parsePidConfigPacket(&ds->altPid, udpPacket);
        break;
      case CMD_SET_VELOCITY_X_PID:
        parsePidConfigPacket(&ds->velocityXPid, udpPacket);
        break;
      case CMD_SET_VELOCITY_Y_PID:
        parsePidConfigPacket(&ds->velocityYPid, udpPacket);
      break;
      case CMD_SET_VELOCITY_Z_PID:
        parsePidConfigPacket(&ds->velocityZPid, udpPacket);
        break;
      case CMD_SET_YPR:
        float yaw,pitch,roll;
        memcpy(&yaw, &udpPacket[1], sizeof(yaw));
        memcpy(&pitch, &udpPacket[1 + sizeof(pitch)], sizeof(pitch));
        memcpy(&roll, &udpPacket[1 + sizeof(yaw) + sizeof(pitch)], sizeof(roll));

        ds->yawRatePid.target = yaw;

        if(ds->pitchPid.enabled)
          ds->pitchPid.target = pitch;
        else
          ds->pitchRatePid.target = pitch;

        if(ds->rollPid.enabled)
          ds->rollPid.target = roll;
        else
          ds->rollRatePid.target = roll;
        // disable hold pos if we have user defined target
        ds->holdPosEnabled = PDL_HOLDPOS_BOTH_XY;
        if(roll != 0.f && pitch != 0.f)
          ds->holdPosEnabled = PDL_HOLDPOS_DISABLED;
        else if(pitch != 0.f)
          ds->holdPosEnabled = PDL_HOLDPOS_X;
        else if(roll != 0.f)
          ds->holdPosEnabled = PDL_HOLDPOS_Y;
        /*if(pitch != 0.f || roll != 0.f)
          ds->holdPosEnabled = PDL_HOLDPOS_DISABLED;
        else
          ds->holdPosEnabled = PDL_HOLDPOS_BOTH_XY;
          */
        break;
      case CMD_SET_PERIODS:
        memcpy(&telemetryPeriod, &udpPacket[1], sizeof(telemetryPeriod));
        if(telemetryPeriod < 1)
          telemetryPeriod = 1;
        break;
      case CMD_ENABLE_STABILIZATION:
        ds->stabilizationEnabled = udpPacket[1];
        break;
      case CMD_RESET_ALTITUDE:
        //vz = 0;
        ds->baro.seaLevelPressure = ds->baro.pressure;
        break;
      case CMD_SET_SEA_LEVEL:
        memcpy(&ds->baro.seaLevelPressure, &udpPacket[1], sizeof(ds->baro.seaLevelPressure));
        break;
      case CMD_SET_ALTITUDE:
        memcpy(&ds->altPid.target, &udpPacket[1], sizeof(ds->altPid.target));
        break;
      case CMD_SET_VELOCITY_Z:
        memcpy(&ds->velocityZPid.target, &udpPacket[1], sizeof(ds->velocityZPid.target));
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

void parseTripleAxisSensorConfigPacket(pdlTripleAxisSensorState *ps, byte *packet)
{
  int16_t dx,dy,dz;

  memcpy(&dx, &packet[1], sizeof(int16_t));
  memcpy(&dy, &packet[3], sizeof(int16_t));
  memcpy(&dz, &packet[5], sizeof(int16_t));

  ps->offset[PDL_X] = dx;
  ps->offset[PDL_Y] = dy;
  ps->offset[PDL_Z] = dz;
}

void parsePidConfigPacket(pdlPidState *pid, byte *packet)
{
  float kp,ki,kd,maxOut,maxErrSum;
  uint8_t enabled = packet[1];
  memcpy(&kp, &packet[2], sizeof(kp));
  memcpy(&ki, &packet[2 + sizeof(kp)], sizeof(ki));
  memcpy(&kd, &packet[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
  memcpy(&maxOut, &packet[2 + sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(maxOut));
  memcpy(&maxErrSum, &packet[2 + sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(maxErrSum)], sizeof(maxErrSum));

  pid->enabled = enabled;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->maxOut = maxOut;
  //pid->maxErrSum = maxErrSum;
}
