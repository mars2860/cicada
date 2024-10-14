#include "pdl.h"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <string.h>
#include "ssid_config.h"
#include "main.h"

#include "WifiBroadcastModem.h"

//-------------------------------------------
// drone wlan packet structure
// {
//   WLAN_IEEE_HEADER = 24 bytes,
//   SSID = 0 to 24 bytes
//   '\0' = 1 byte string termination symbol
//   packetType = 1 byte (0 - Command, 1 - Telemetry, 2 - LOG)
//   various (Command data, DroneState size (4 bytes) + DroneState bytes, LOG text)
// }

#define WLAN_COMMAND_PACKET     0
#define WLAN_TELEMETRY_PACKET   1
#define WLAN_LOG_PACKET         2

#define WLAN_RX_BUF_SIZE        128
#define WLAN_TX_BUF_SIZE        WLAN_MAX_PAYLOAD_SIZE

#define DEFAULT_UDP_PORT                  4210
//#define DEFAULT_CMD_PORT                4210
//#define DEFAULT_TELEMETRY_PORT          4211
//#define DEFAULT_LOG_PORT                4212

// UDP packets cannot go over the link MTU (Max Transmission Unit)
// which is total of 1500 bytes for Ethernet, therefore the maximum
// data packet being 1472 (after subtracting the overhead).

// We save each net setting to a separate file
// PARAM name = file name
// a param file stores only string
// to store 1 byte integer param we convert its to ASCII char and store as text
#define PARAM_SSID                      "ssid"
#define PARAM_PSK                       "psk"
#define PARAM_IP                        "ip"
#define PARAM_GATEWAY                   "gateway"
#define PARAM_SUBNET                    "subnet"
#define PARAM_USE_DHCP                  "dhcp"
#define PARAM_WIFI_STA_MODE             "sta"
#define PARAM_WIFI_CHANNEL              "chl"
#define PARAM_WIFI_TX_POWER_LEVEL       "tpw"
#define PARAM_WIFI_PHY_MODE             "phy"
#define PARAM_WIFI_RATE                 "rate"

char strSsid[24];

IPAddress ip;
IPAddress gateway;
IPAddress subnet;
IPAddress host;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiUDP udp;
uint8_t wlanTxBuf[WLAN_TX_BUF_SIZE];
uint8_t wlanRxBuf[WLAN_RX_BUF_SIZE];
uint16_t wlanRxBufLen = 0;
uint8_t wlanRxBufBusy = 0;  // onWifiBroadcastRx will skip received packet if this flag is set

uint32_t cmdPort;
uint32_t telemetryPort;
uint32_t logPort;

static uint8_t wifiChl;
static uint8_t wifiTpw;
static uint8_t wifiPhy;
static uint8_t wifiRate;

/// Open Project Settings->Arduino->Compile Options append to link -Wl,-wrap,ppEnqueueRxq
uint8_t wifiBroadcastEnabled = 0;

/// Writes data to wlan packet at given position. Returns new position to write
uint16_t writeWlanPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);

void printParam(const char *param, const char *value, bool printAsInt)
{
  if(printAsInt)
  {
    LOG_INFO("Loaded %s=%i",param,value[0]);
  }
  else
  {
    LOG_INFO("Loaded %s=%s",param,value);
  }
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

void saveParamToFile(const char* param, const char* buf, bool isIntParam)
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

    if(isIntParam)
    {
      if(oldValue[0] == buf[0])
      {
        LOG_INFO("New %s=%i stored, %i bytes",param,buf[0],written);
      }
      else
      {
        LOG_ERROR("Can't store new %s=%i, %i bytes",param,buf[0],strlen(buf));
      }
    }
    else
    {
      if(strcmp(oldValue,buf) == 0)
      {
        LOG_INFO("New %s=%s stored, %i bytes",param,buf,written);
      }
      else
      {
        LOG_ERROR("Can't store new %s=%s, %i bytes",param,buf,strlen(buf));
      }
    }
  }

  SPIFFS.end();
}

void pdlSetupRc(pdlDroneState*)
{
  char strPsk[24];
  char strIp[24];
  char strGateway[24];
  char strSubnet[24];
  char strUseDhcp[4];
  char strSta[4];
  char strChl[4];
  char strTpw[4];
  char strPhy[4];
  char strRate[4];

  WiFiPhyMode phy;
  uint8_t staMode = 0;
  uint8_t connectionResult;

  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
  }

  getParamFromFile(PARAM_SSID,strSsid,sizeof(strSsid),DEFAULT_SSID);
  getParamFromFile(PARAM_PSK,strPsk,sizeof(strPsk),DEFAULT_PSK);
  getParamFromFile(PARAM_IP,strIp,sizeof(strIp),DEFAULT_IP_ADDRESS);
  getParamFromFile(PARAM_GATEWAY,strGateway,sizeof(strGateway),DEFAULT_GATEWAY_ADDRESS);
  getParamFromFile(PARAM_SUBNET,strSubnet,sizeof(strSubnet),DEFAULT_SUBNET);
  getParamFromFile(PARAM_USE_DHCP,strUseDhcp,sizeof(strUseDhcp),DEFAULT_USE_DHCP);
  getParamFromFile(PARAM_WIFI_STA_MODE,strSta,sizeof(strSta),DEFAULT_WIFI_STA_MODE);
  getParamFromFile(PARAM_WIFI_CHANNEL,strChl,sizeof(strChl),DEFAULT_WIFI_CHANNEL);
  getParamFromFile(PARAM_WIFI_TX_POWER_LEVEL,strTpw,sizeof(strTpw),DEFAULT_WIFI_TX_POWER_LEVEL);
  getParamFromFile(PARAM_WIFI_PHY_MODE,strPhy,sizeof(strPhy),DEFAULT_WIFI_PHY_MODE);
  getParamFromFile(PARAM_WIFI_RATE,strRate,sizeof(strRate),DEFAULT_WIFI_RATE);

  printParam(PARAM_SSID,strSsid,false);
  printParam(PARAM_PSK,strPsk,false);
  printParam(PARAM_IP,strIp,false);
  printParam(PARAM_GATEWAY,strGateway,false);
  printParam(PARAM_SUBNET,strSubnet,false);
  printParam(PARAM_USE_DHCP,strUseDhcp,false);
  printParam(PARAM_WIFI_STA_MODE,strSta,false);
  printParam(PARAM_WIFI_CHANNEL,strChl,true);
  printParam(PARAM_WIFI_TX_POWER_LEVEL,strTpw,true);
  printParam(PARAM_WIFI_PHY_MODE,strPhy,true);
  printParam(PARAM_WIFI_RATE,strRate,true);

  SPIFFS.end();

  telemetryPort = DEFAULT_UDP_PORT;
  cmdPort = DEFAULT_UDP_PORT;
  logPort = DEFAULT_UDP_PORT;

  ip.fromString(strIp);
  gateway.fromString(strGateway);
  subnet.fromString(strSubnet);

  switch(strPhy[0])
  {
  case WIFI_PHY_MODE_11B:
    phy = WIFI_PHY_MODE_11B;
    break;
  case WIFI_PHY_MODE_11G:
    phy = WIFI_PHY_MODE_11G;
    break;
  case WIFI_PHY_MODE_11N:
    phy = WIFI_PHY_MODE_11N;
    break;
  default:
    phy = WIFI_PHY_MODE_11G;
    break;
  }

  wifiChl = strChl[0];
  wifiTpw = strTpw[0];
  wifiPhy = strPhy[0];
  wifiRate = strRate[0];

  if(strcmp(strSta,"y") == 0)
  {
    staMode = 1;

    LOG_INFO("Start WIFI_STA");

    if(!WiFi.mode(WIFI_STA))
    {
      LOG_ERROR("Can't set WIFI_STA mode");
    }

    // Try to connect USER wifi
    for(uint8_t i = 0; i < 5; i++)
    {
      if(strcmp(strUseDhcp,"n") == 0)
      {
        if(!WiFi.config(ip, gateway, subnet))
        {
          LOG_ERROR("Can't config static IP addr for wifi station");
        }
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
        WiFi.begin(DEFAULT_SSID, DEFAULT_PSK);
        connectionResult = WiFi.waitForConnectResult(5000);
        if(connectionResult == WL_CONNECTED)
        {
          break;
        }
        LOG_ERROR("Connection to %s is failed", DEFAULT_SSID);
      }
    }

    // Check connection
    if(connectionResult != WL_CONNECTED)
    {
      staMode = 0;
    }
  }

  if(!staMode)
  {
    LOG_INFO("Start WIFI_AP");

    if(!WiFi.mode(WIFI_AP))
    {
      LOG_ERROR("Can't set WIFI_AP mode");
    }

    if(!WiFi.softAPConfig(ip, gateway, subnet))
    {
      LOG_ERROR("Can't set ip,gateway,subnet of wifi access point");
    }

    if(!WiFi.softAP(strSsid, strPsk, wifiChl, false, 1))
    {
      LOG_ERROR("Can't start wifi access point");
    }
  }

  wbmSetMaxTpw(wifiTpw);

  if(!WiFi.setPhyMode(phy))
  {
    LOG_ERROR("Can't set wifi phy mode=%i",phy);
  }

  if(wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL,wifiRate) != 0)
  {
    LOG_ERROR("Can't set wifi rate=%i\n",wifiRate);
  }

  LOG_INFO("Wifi chl=%i",wbmGetChannel());
  LOG_INFO("Wifi tpw=%i",wbmGetMaxTpw());
  LOG_INFO("Wifi phy=%i",wbmGetPhyMode());
  LOG_INFO("Wifi rate=%i",wbmGetRate());

  udp.begin(cmdPort);

  httpUpdater.setup(&httpServer);
  httpServer.begin();
}

void pdlSetupTelemetry(pdlDroneState*) {}

void pdlRemoteControl(pdlDroneState *ds)
{
  // send log to the host
  if(pdlGetLogSize() > 0 && pdlIsHostConnected(ds))
  {
    uint32_t logSize = pdlGetLogSize();
    if(logSize >= WLAN_TX_BUF_SIZE)
    {
      pdlResetLog();
      LOG_ERROR("Can't send log, its size too big=%i",logSize);
    }
    logSize = pdlGetLogSize();
    if(logSize < WLAN_TX_BUF_SIZE)
    {
      uint16_t pos = 0;
      // prepare wifi packet
      strcpy((char*)wlanTxBuf,strSsid);
      pos = strlen(strSsid);
      wlanTxBuf[pos++] = 0; // string termination symbol
      wlanTxBuf[pos++] = WLAN_LOG_PACKET; // log packet type
      pos = writeWlanPacket(pos, wlanTxBuf, pdlGetLog(), logSize);
      if(pos < WLAN_TX_BUF_SIZE)
        wlanTxBuf[pos++] = 0;   // string termination symbol

      if(wifiBroadcastEnabled)
      {
        if( !wbmIsTxBusy() &&
            wbmGetRssi() != 0 ) // that we determine that host is connected after switching to the broadcast mode
        {
          if(wbmSendPktFreedom(wlanTxBuf,pos))
          {
            pdlResetLog();
          }
        }
      }
      else
      {
        pdlResetLog();
        udp.beginPacket(host, logPort);
        udp.write(wlanTxBuf,pos);
        udp.endPacket();
      }
    }
  }
  // receive udp packets in normal wifi mode
  if(!wifiBroadcastEnabled)
  {
    if(udp.parsePacket())
    {
      wlanRxBufLen = udp.read(wlanRxBuf, sizeof(wlanRxBuf));

      if(wlanRxBufLen > 0)
      {
        if(WiFi.getMode() == WIFI_STA && !hostIsSet()) // In WIFI_STA mode we trust only who first sent a packet to the drone
        {
          host = udp.remoteIP();
        }
        else  // In WIFI_AP mode we trust to anyone because we allow the only one connection to our AP
              // if host is reconnected to our AP it will have new IP but it will not cause the problem
        {
          host = udp.remoteIP();
        }
      }
    }
  }
  // parse command
  wlanRxBufBusy = 1;  // to prevent data corruption by onWifiBroadcastRx
  if(wlanRxBufLen > 0)
  {
     // we use ssid for data collision avoidance if many drones work at the same channel
     // ssid we put at the head of the packet
     if(strcmp((const char*)wlanRxBuf,strSsid) == 0)
     {
       // skip ssid
       uint16_t pos = strlen((const char*)wlanRxBuf);
       // skip '\0' which is terminated ssid
       pos++;
       // get packet type
       uint8_t packetType = wlanRxBuf[pos++];
       if(packetType == WLAN_COMMAND_PACKET)
       {
         pdlParseCommand(ds,&wlanRxBuf[pos]);
         // immediately send back telemetry packet to confirm command execution
         pdlUpdateTelemetry(ds);
       }
       else
       {
         LOG_ERROR("Received unknown packet type=%i",packetType);
       }
     }
  }
  wlanRxBufLen = 0;
  wlanRxBufBusy = 0;

  // firmware update http server
  if(!wifiBroadcastEnabled)
  {
    httpServer.handleClient();
  }
}

bool hostIsSet()
{
  return host.isSet();
}

uint8_t pdlIsHostConnected(pdlDroneState*)
{
  if(wifiBroadcastEnabled)  // TODO I don't know how to determine that the host is connected in wifi broadcast mode
  {
    return 1;
  }

  if( WiFi.getMode() == WIFI_STA &&
      WiFi.status() == WL_CONNECTED &&
      hostIsSet() )
  {
    return 1;
  }

  if( WiFi.getMode() == WIFI_AP &&
      WiFi.softAPgetStationNum() > 0 &&
      hostIsSet() )
  {
    return 1;
  }

  return 0;
}

int32_t getRssi()
{
  if(wifiBroadcastEnabled)
  {
    return wbmGetRssi();
  }
  else if(WiFi.getMode() == WIFI_STA)
  {
    return WiFi.RSSI();
  }
  return -1;  // There is no way to determine RSSI in SoftAp mode
}

void pdlUpdateTelemetry(pdlDroneState *ds)
{
  if(!hostIsSet())
  {
    return;
  }

  uint16_t pos = 0;
  size_t sz = sizeof(pdlDroneState);

  ds->timestamp = pdlMicros();
  ds->rc.rssi = getRssi();

  // prepare wifi broadcast packet
  strcpy((char*)wlanTxBuf,strSsid);
  pos = strlen(strSsid);
  wlanTxBuf[pos++] = 0; // string termination symbol
  wlanTxBuf[pos++] = WLAN_TELEMETRY_PACKET; // telemetry packet type
  // DroneState size
  pos = writeWlanPacket(pos, wlanTxBuf, &sz, sizeof(sz));
  // DroneState
  pos = writeWlanPacket(pos, wlanTxBuf, ds, sz);
  // Send packet
  if(wifiBroadcastEnabled)
  {
    if(!wbmIsTxBusy())
    {
      wbmSendPktFreedom(wlanTxBuf,pos);
    }
  }
  else
  {
    udp.beginPacket(host, telemetryPort);
    udp.write(wlanTxBuf, pos);
    udp.endPacket();
  }
}

uint16_t writeWlanPacket(uint16_t pos, byte *packet, void *value, size_t valueSize)
{
  if(pos + valueSize >= WLAN_MAX_PAYLOAD_SIZE)
    return pos;

  memcpy(&packet[pos], value, valueSize);
  pos += valueSize;
  return pos;
}

void onWifiBroadcastRx(uint8_t *data, uint16_t len)
{
  if(!wifiBroadcastEnabled)
  {
    LOG_ERROR("Wifi broadcast is disabled but rx func was invoked");
    return;
  }
  // copy received data to buffer
  // onWifiBroadcastRx is invoked from interrupt, it can be invoked when we parse udpPacket or send them
  // this clause will destroy our data
  // to prevent this we check udpPacketBusy flag and we lose received packet if udpPacketBusy is set
  if(!wlanRxBufBusy)
  {
    wlanRxBufLen = len;
    if(wlanRxBufLen >= WLAN_RX_BUF_SIZE)
    {
      wlanRxBufLen = 0;
      LOG_ERROR("Received wifi broadcast packet is too big, len=%i",len);
      return;
    }
    memcpy(wlanRxBuf,data,wlanRxBufLen);
  }
  // FIXME why I lose packets here if I send its every 50ms? This time have to be enough to process packets
  //else // it gives too many messages
  //{
  //  LOG_ERROR("Wifi broadcast packet is lost because wlanRxBuf is busy");
  //}
}

void pdlCmdSetSsid(pdlDroneState *ds,const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_SSID,(const char*)&packet[1],false);
}

void pdlCmdSetPsk(pdlDroneState *ds,const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_PSK,(const char*)&packet[1],false);
}

void pdlCmdSetIp(pdlDroneState *ds, const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_IP,(const char*)&packet[1],false);
}

void pdlCmdSetGateway(pdlDroneState *ds, const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_GATEWAY,(const char*)&packet[1],false);
}

void pdlCmdSetSubnet(pdlDroneState*ds, const uint8_t* packet)
{
  (void)ds;
  saveParamToFile(PARAM_SUBNET,(const char*)&packet[1],false);
}

void pdlCmdEnableDynamicIp(pdlDroneState*ds, const uint8_t* packet)
{
  (void)ds;
  char value[] = {'n',0};
  if(packet[1])
  {
    value[0] = 'y';
  }
  saveParamToFile(PARAM_USE_DHCP,value,false);
}

void pdlCmdSetupWifi(pdlDroneState *ds, const uint8_t* packet)
{
  (void)ds;

  char sta[] = {'n',0};
  char chl[] = {packet[2], 0};
  char tpw[] = {packet[3], 0};
  char phy[] = {packet[4], 0};
  char rate[] = {packet[5], 0};

  if(packet[1])
  {
    sta[0] = 'y';
  }

  if(chl[0] < 1 || chl[0] > 14)
  {
    chl[0] = 7;
  }

  if(tpw[0] > 82)
  {
    tpw[0] = 82;
  }

  if(phy[0] < 1 || phy[0] > 3)
  {
    phy[0] = 2;
  }

  if(rate[0] > 0x0F)
  {
    rate[0] = 0x0B;
  }

  // don't allow to change wifi param on the fly because it is not safe
  /*
  wifiChl = chl[0];
  wifiTpw = tpw[0];
  wifiPhy = phy[0];
  wifiRate = rate[0];

  if(wifiBroadcastEnabled)
  {
    wbmSetChannel(wifiChl);
    wbmSetMaxTpw(wifiTpw);
    wbmSetPhyMode(wifiPhy);
    wbmSetRate(wifiRate);
  }
  */

  saveParamToFile(PARAM_WIFI_STA_MODE,sta,false);
  saveParamToFile(PARAM_WIFI_CHANNEL,chl,true);
  saveParamToFile(PARAM_WIFI_TX_POWER_LEVEL,tpw,true);
  saveParamToFile(PARAM_WIFI_PHY_MODE,phy,true);
  saveParamToFile(PARAM_WIFI_RATE,rate,true);
}

void pdlCmdEnableWifiBroadcast(pdlDroneState* ds, const uint8_t* packet)
{
  (void)ds;
  uint8_t oldWifiBroadcastEnabled = wifiBroadcastEnabled;
  wifiBroadcastEnabled = packet[1];

  if(wifiBroadcastEnabled)
  {
    if(!oldWifiBroadcastEnabled)
    {
      udp.stop();
      httpServer.stop();
      httpServer.close();
    }

    wbmSetRxFunc(onWifiBroadcastRx);
    wbmStartNormal();
    wbmSetChannel(wifiChl);
    wbmSetMaxTpw(wifiTpw);
    wbmSetPhyMode(wifiPhy);
    wbmSetRate(wifiRate);

    LOG_INFO("Wifi broadcast is enabled");
  }
  else
  {
    wbmStop();

    pdlSetupRc(ds);
    pdlSetupTelemetry(ds);

    LOG_INFO("Wifi broadcast is disabled");
  }
}
