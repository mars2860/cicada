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

/// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);

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

void pdlSetupTelemetry(pdlDroneState*) {}

void pdlRemoteControl(pdlDroneState *ds)
{
  if(udp.parsePacket())
  {
    if(udp.read(udpPacket, sizeof(udpPacket)) > 0)
    {
      if(!host.isSet())
        host = udp.remoteIP();
      pdlParseCommand(ds,udpPacket);
    }
  }

  /* tcp were losing packets (how it can be?! but after some cmd has been sent, telemetry says no changes)
   * so, tcp has been rejected
  if(tcp.hasClient())
  {
    WiFiClient client = tcp.available();

    if(client.available() > 0)
    {
      if(client.read(udpPacket,sizeof(udpPacket)) > 0)
      {
        if(!host.isSet())
          host = client.remoteIP();
        pdlParseCommand(ds,udpPacket);
        client.stop();
      }
    }
  }
  */

  httpServer.handleClient();
}

void pdlUpdateTelemetry(pdlDroneState *ds)
{
  if(!host.isSet())
    return;

  uint16_t pos = 0;
  uint32_t telemetryPeriod = pdlGetTelemetryPeriod();
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

uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize)
{
  if(pos + valueSize >= TELEMETRY_MAX_PACKET_SIZE)
    return pos;

  memcpy(&packet[pos], value, valueSize);
  pos += valueSize;
  return pos;
}
