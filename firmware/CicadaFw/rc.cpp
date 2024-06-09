#include "pdl.h"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <string.h>
#include "ssid_config.h"

#define DEFAULT_CMD_PORT                4210
#define DEFAULT_TELEMETRY_PORT          4211
#define DEFAULT_LOG_PORT                4212

// UDP packets cannot go over the link MTU (Max Transmission Unit)
// which is total of 1500 bytes for Ethernet, therefore the maximum
// data packet being 1472 (after subtracting the overhead).

#define TELEMETRY_MAX_PACKET_SIZE       1096

#define PARAM_SSID "ssid"
#define PARAM_PSK  "psk"
#define PARAM_IP   "ip"
#define PARAM_GATEWAY "gateway"
#define PARAM_SUBNET "subnet"
#define PARAM_USE_DHCP   "dhcp"

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
uint32_t logPort;

/// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);

void printParam(const char *param, const char *value)
{
  LOG_INFO("Loaded %s=%s",param,value);
}

/// @note: this function have to be used between SPIFFS.begin() and SPIFFS.end()
void getParamFromFile(const char *param, char *buf, uint8_t buf_size, const char *defValue)
{
  char filename[32];
  snprintf(filename,sizeof(filename),"/%s.txt",param);
  File file = SPIFFS.open(filename,"r");
  if(!file)
  {
    strncpy(buf,defValue,buf_size);
  }
  else
  {
    uint8_t pos = 0;
    while(file.available() && pos < buf_size)
    {
      buf[pos] = file.read();
      pos++;
    }
    if(pos < buf_size)
      buf[pos] = 0;
    else
      buf[buf_size-1] = 0;
  }
  file.close();
}

void saveParamToFile(const char* param, const char* buf)
{
  char filename[32];
  char oldValue[24];
  snprintf(filename,sizeof(filename),"/%s.txt",param);
  if(strlen(buf) >= 24 || strlen(buf) == 0)
  {
    LOG_ERROR("New %s is invalid: has to be less 24 symbols",param);
    return;
  }

  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
    return;
  }
  getParamFromFile(param,oldValue,sizeof(oldValue),"");
  if(strcmp(oldValue,buf) != 0)
  {
    size_t written = 0;
    File file = SPIFFS.open(filename,"w+");
    if(!file)
    {
      LOG_ERROR("Can't open %s",filename);
    }
    else
    {
      written = file.print(buf);
      file.flush();
    }
    file.close();
    // checks saved params
    // ESP-07 doesn't save anything but return OK and written > 0 !!!
    getParamFromFile(param,oldValue,sizeof(oldValue),"");
    if(strcmp(oldValue,buf) == 0)
    {
      LOG_INFO("New %s=%s stored, %i bytes",param,buf,written);
    }
    else
    {
      LOG_ERROR("Can't store new %s=%s, %i bytes",param,buf,strlen(buf));
    }
  }

  SPIFFS.end();
}

void pdlSetupRc(pdlDroneState*)
{
  char strSsid[24];
  char strPsk[24];
  char strIp[24];
  char strGateway[24];
  char strSubnet[24];
  char strUseDhcp[4];
  uint8_t connectionResult;

  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
  }

  getParamFromFile(PARAM_SSID,strSsid,sizeof(strSsid),DEFAULT_STASSID);
  getParamFromFile(PARAM_PSK,strPsk,sizeof(strPsk),DEFAULT_STAPSK);
  getParamFromFile(PARAM_IP,strIp,sizeof(strIp),DEFAULT_IP_ADDRESS);
  getParamFromFile(PARAM_GATEWAY,strGateway,sizeof(strGateway),DEFAULT_GATEWAY_ADDRESS);
  getParamFromFile(PARAM_SUBNET,strSubnet,sizeof(strSubnet),DEFAULT_SUBNET);
  getParamFromFile(PARAM_USE_DHCP,strUseDhcp,sizeof(strUseDhcp),DEFAULT_USE_DHCP);

  printParam(PARAM_SSID,strSsid);
  printParam(PARAM_PSK,strPsk);
  printParam(PARAM_IP,strIp);
  printParam(PARAM_GATEWAY,strGateway);
  printParam(PARAM_SUBNET,strSubnet);
  printParam(PARAM_USE_DHCP,strUseDhcp);

  SPIFFS.end();

  telemetryPort = DEFAULT_TELEMETRY_PORT;
  cmdPort = DEFAULT_CMD_PORT;
  logPort = DEFAULT_LOG_PORT;

  ip.fromString(strIp);
  gateway.fromString(strGateway);
  subnet.fromString(strSubnet);

  WiFi.mode(WIFI_STA);

  // Try to connect USER wifi
  for(uint8_t i = 0; i < 5; i++)
  {
    if(strcmp(strUseDhcp,"n") == 0)
    {
      WiFi.config(ip, gateway, subnet);
    }
    WiFi.begin(strSsid, strPsk);
    connectionResult = WiFi.waitForConnectResult(5000);
    if(connectionResult == WL_CONNECTED)
    {
      break;
    }
    LOG_ERROR("Connection to %s is failed", strSsid);
  }
  // Try to connect default wifi
  if(connectionResult != WL_CONNECTED)
  {
    ip.fromString(DEFAULT_IP_ADDRESS);
    gateway.fromString(DEFAULT_GATEWAY_ADDRESS);
    subnet.fromString(DEFAULT_SUBNET);

    for(uint8_t i = 0; i < 5; i++)
    {
      WiFi.config(ip, gateway, subnet);
      WiFi.begin(DEFAULT_STASSID, DEFAULT_STAPSK);
      connectionResult = WiFi.waitForConnectResult(5000);
      if(connectionResult == WL_CONNECTED)
      {
        break;
      }
      LOG_ERROR("Connection to %s is failed", DEFAULT_STASSID);
    }
  }
  // Check connection
  if(connectionResult != WL_CONNECTED)
  {
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
      // immediately send back telemetry packet to confirm command execution
      pdlUpdateTelemetry(ds);
    }
  }

  if(pdlGetLogSize() > 0 && host.isSet())
  {
    udp.beginPacket(host, logPort);
    udp.write(pdlGetLog(),pdlGetLogSize());
    udp.endPacket();

    pdlResetLog();
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

bool hostIsSet()
{
  return host.isSet();
}

void pdlUpdateTelemetry(pdlDroneState *ds)
{
  if(!host.isSet())
    return;

  uint16_t pos = 0;
  uint32_t telemetryPeriod = pdlGetTelemetryUpdatePeriod();
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

void pdlCmdSetSsid(pdlDroneState *ds,const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_SSID,(const char*)&packet[1]);
}

void pdlCmdSetPsk(pdlDroneState *ds,const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_PSK,(const char*)&packet[1]);
}

void pdlCmdSetIp(pdlDroneState *ds, const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_IP,(const char*)&packet[1]);
}

void pdlCmdSetGateway(pdlDroneState *ds, const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_GATEWAY,(const char*)&packet[1]);
}

void pdlCmdSetSubnet(pdlDroneState*ds, const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_SUBNET,(const char*)&packet[1]);
}

void pdlCmdEnableDynamicIp(pdlDroneState*ds, const uint8_t* packet)
{
  (void)ds;
  char value[2] = {'n',0};
  if(packet[1])
    value[0] = 'y';
  saveParamToFile(PARAM_USE_DHCP,value);
}
