#include "pdl.h"
#include "main.h"

#ifdef ARDUINO_ARCH_ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266HTTPUpdateServer.h>
  #include <FS.h>
  #include "WifiBroadcastModem.h"
#elif ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <WebServer.h>
  #include <HTTPUpdateServer.h>
  #include <SPIFFS.h>
  #include <esp_wifi.h>

  // Difference from ESP8266
  // 1. Can't use wifi broadcast because wifibroadcastmodem code is intended for ESP8266
  // 2. Can't setup PHY mode
  // 3. Setting wifi rate is not needed

#endif

#include <WiFiUdp.h>
#include <string.h>
#include "ssid_config.h"

//-------------------------------------------
// drone wlan packet structure
// {
//   ***HEADER*** (Only for wifi broadcast mode)
//   WLAN_IEEE_HEADER = 24 bytes,
//   ***PACKET HEADER***
//   droneId = 4 bytes
//   packetNum = 4 bytes
//   ***DATA***
//   packetType = 1 byte (0 - Command, 1 - Telemetry, 2 - LOG, 3 - JPEG Chunk)
//   various:
//   0 - Command = {1 byte of cmd code, various size command data}
//   1 - Telemetry = {4 bytes of pdlDroneState size, whole struct pdlDroneState}
//   2 - LOG = {4 bytes of log size, various size log data}
//   3 - Picture First Chunk = {uint16_t width, uint16_t height, uint8_t pixformat, uint8_t quality, uint64_t timestamp, size_t dataLen, uint8_t *data}
//   4 - Picture Data Chunk = {uint16_t chunkNum,data bytes}
//   5 - Picture Last Data Chunk = {uint16_t chunkNum,data bytes}
// }

#define WLAN_COMMAND_PACKET             0
#define WLAN_TELEMETRY_PACKET           1
#define WLAN_LOG_PACKET                 2
#define WLAN_PICTURE_START_PACKET       3
#define WLAN_PICTURE_DATA_PACKET        4
#define WLAN_PICTURE_LAST_DATA_PACKET   5

#define WLAN_MAX_PAYLOAD_SIZE   1400

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
#define PARAM_DRONE_ID                  "droneid"

char strSsid[24];

IPAddress ip;
IPAddress gateway;
IPAddress subnet;
IPAddress host;

#ifdef ARDUINO_ARCH_ESP8266
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
#elif ARDUINO_ARCH_ESP32
WebServer httpServer(80);
HTTPUpdateServer httpUpdater;
SemaphoreHandle_t semWlan = NULL;
#endif

WiFiUDP udp;

uint8_t wlanTxBuf[WLAN_TX_BUF_SIZE];
uint8_t wlanRxBuf[WLAN_RX_BUF_SIZE];
uint16_t wlanRxBufLen = 0;
uint8_t wlanRxBufBusy = 0;  // onWifiBroadcastRx will skip received packet if this flag is set
uint32_t wlanRxPacketNum = 0;
uint32_t wlanTxPacketNum = 0;

uint32_t droneId; // we use droneId to avoid collisions when many drones work at the same channel

static uint8_t wifiChl;
static uint8_t wifiTpw;
static uint8_t wifiPhy;
static uint8_t wifiRate;

static bool hostPinned = false;

uint8_t wifiBroadcastEnabled = 0;	// inherited from Cicada for ESP8266. WiFiBroadcast is not supported on ESP32S3 platform

/// Writes data to wlan packet at given position. Returns new position to write
uint16_t writeWlanPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);

bool sendWlanPacket(uint8_t *data, uint16_t data_len);
bool sendWlanLogPacket(pdlDroneState *ds);
bool sendWlanTelemetryPacket(pdlDroneState *ds);

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

void setWifiParams(uint8_t chl, uint8_t tpw, uint8_t phy, uint8_t rate)
{
  if(chl > 14 || chl < 1)
    chl = atoi(DEFAULT_WIFI_CHANNEL_STR);
  if(tpw > 82)
    tpw = atoi(DEFAULT_WIFI_TX_POWER_LEVEL_STR);
  if(phy > 3 || phy < 1)
    phy = atoi(DEFAULT_WIFI_PHY_MODE_STR);
  if(rate > 0x0F || rate < 0x01)
    rate = atoi(DEFAULT_WIFI_RATE_STR);

  wifiChl = chl;
  wifiTpw = tpw;
  wifiPhy = phy;
  wifiRate = rate;
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
  char strDroneId[8];

  uint8_t staMode = 0;
  uint8_t connectionResult;

#ifdef ARDUINO_ARCH_ESP32
  if(!semWlan)
  {
    semWlan = xSemaphoreCreateMutex();
  }
#endif

  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
  }

  getParamFromFile(PARAM_SSID,strSsid,sizeof(strSsid),DEFAULT_SSID_STR);
  getParamFromFile(PARAM_PSK,strPsk,sizeof(strPsk),DEFAULT_PSK_STR);
  getParamFromFile(PARAM_IP,strIp,sizeof(strIp),DEFAULT_IP_ADDRESS_STR);
  getParamFromFile(PARAM_GATEWAY,strGateway,sizeof(strGateway),DEFAULT_GATEWAY_ADDRESS_STR);
  getParamFromFile(PARAM_SUBNET,strSubnet,sizeof(strSubnet),DEFAULT_SUBNET_STR);
  getParamFromFile(PARAM_USE_DHCP,strUseDhcp,sizeof(strUseDhcp),DEFAULT_USE_DHCP_STR);
  getParamFromFile(PARAM_WIFI_STA_MODE,strSta,sizeof(strSta),DEFAULT_WIFI_STA_MODE_STR);
  getParamFromFile(PARAM_WIFI_CHANNEL,strChl,sizeof(strChl),DEFAULT_WIFI_CHANNEL_STR);
  getParamFromFile(PARAM_WIFI_TX_POWER_LEVEL,strTpw,sizeof(strTpw),DEFAULT_WIFI_TX_POWER_LEVEL_STR);
  getParamFromFile(PARAM_WIFI_PHY_MODE,strPhy,sizeof(strPhy),DEFAULT_WIFI_PHY_MODE_STR);
  getParamFromFile(PARAM_WIFI_RATE,strRate,sizeof(strRate),DEFAULT_WIFI_RATE_STR);
  getParamFromFile(PARAM_DRONE_ID,strDroneId,sizeof(strDroneId),DEFAULT_DRONE_ID_STR);

  printParam(PARAM_SSID,strSsid);
  printParam(PARAM_PSK,strPsk);
  printParam(PARAM_IP,strIp);
  printParam(PARAM_GATEWAY,strGateway);
  printParam(PARAM_SUBNET,strSubnet);
  printParam(PARAM_USE_DHCP,strUseDhcp);
  printParam(PARAM_WIFI_STA_MODE,strSta);
  printParam(PARAM_WIFI_CHANNEL,strChl);
  printParam(PARAM_WIFI_TX_POWER_LEVEL,strTpw);
  printParam(PARAM_WIFI_PHY_MODE,strPhy);
  printParam(PARAM_WIFI_RATE,strRate);
  printParam(PARAM_DRONE_ID,strDroneId);

  SPIFFS.end();

  droneId = atoi(strDroneId);

  setWifiParams( atoi(strChl),
                 atoi(strTpw),
                 atoi(strPhy),
                 atoi(strRate));

  ip.fromString(strIp);
  gateway.fromString(strGateway);
  subnet.fromString(strSubnet);

  if( strcmp(strSta,"y") == 0 ||
      strSta[0] == 0 ) // if we have bad or empty sta.txt file on SPIFFS partition
  {
    staMode = 1;
  }

  WiFi.persistent(false);

  if(staMode)
  {
    /*
    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);

    delay(100);

    connectionResult = WiFi.waitForConnectResult(5000);
    if(connectionResult != WL_DISCONNECTED)
    {
      LOG_ERROR("Old WiFi is not disconnected,stat=%i",connectionResult);
    }
    */

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
      LOG_INFO("Try to connect to default ssid=%s as wifi_sta", DEFAULT_SSID_STR);
      ip.fromString(DEFAULT_IP_ADDRESS_STR);
      gateway.fromString(DEFAULT_GATEWAY_ADDRESS_STR);
      subnet.fromString(DEFAULT_SUBNET_STR);

      for(uint8_t i = 0; i < 5; i++)
      {
        WiFi.config(ip, gateway, subnet);
        WiFi.begin(DEFAULT_SSID_STR, DEFAULT_PSK_STR);
        connectionResult = WiFi.waitForConnectResult(5000);
        if(connectionResult == WL_CONNECTED)
        {
          break;
        }
        LOG_ERROR("Connection to %s is failed", DEFAULT_SSID_STR);
      }
    }

    // failed to connect to user defined AP
    // failed to connect to default AP
    if(connectionResult != WL_CONNECTED)
    {
      // switch to WIFI_AP mode with default net settings
      staMode = 0;

      strcpy(strSsid,DEFAULT_SSID_STR);
      strcpy(strPsk,DEFAULT_PSK_STR);
      ip.fromString(DEFAULT_IP_ADDRESS_STR);
      gateway.fromString(DEFAULT_GATEWAY_ADDRESS_STR);
      subnet.fromString(DEFAULT_SUBNET_STR);
      strcpy(strChl,DEFAULT_WIFI_CHANNEL_STR);
      strcpy(strTpw,DEFAULT_WIFI_TX_POWER_LEVEL_STR);
      strcpy(strPhy,DEFAULT_WIFI_PHY_MODE_STR);
      strcpy(strRate,DEFAULT_WIFI_RATE_STR);

      setWifiParams( atoi(strChl),
                     atoi(strTpw),
                     atoi(strPhy),
                     atoi(strRate));
    }
  }

  if(!staMode)
  {
    /*
    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);

    delay(100);

    connectionResult = WiFi.waitForConnectResult(5000);
    if(connectionResult != WL_DISCONNECTED)
    {
      LOG_ERROR("Old WiFi is not disconnected,stat=%i",connectionResult);
    }
    */

    LOG_INFO("Start WIFI_AP");

    if(!WiFi.mode(WIFI_AP))
    {
      LOG_ERROR("Can't set WIFI_AP mode");
    }

    delay(100);

    // I revealed the bug inside the esp core. There are devices if it is disconnected ESP softAP still considers that it is connected
    // Even If I power down this device ESP softAP still considers that it is connected. For example, the some device is RTL8812AU
    // Due to this I set max connections to 4 instead 1
    // Also I guess that this bug related to the bug where some devices can't communicate with ESP softAP
    if(!WiFi.softAP(strSsid, strPsk, wifiChl, false, 4))
    {
      LOG_ERROR("Can't start wifi access point");
    }

    // for esp32 we have to call softApConfig after softAp and after AP_START event
    // this delay waits AP_START event
    delay(200);

    IPAddress dhcpStart = ip;
    dhcpStart[3] = dhcpStart[3] + 1;

    // NOTE: if you connect to softAP from PC having internet connection
    // the gateway has to be the same as your in inet connection
    // else you lose inet connection and can't access softAp network!
    if(!WiFi.softAPConfig(ip, gateway, subnet, dhcpStart))
    {
      LOG_ERROR("Can't set ip,gateway,subnet of wifi access point");
    }

    IPAddress myIP = WiFi.softAPIP();
    LOG_INFO("WIFI_AP IP address=%s",myIP.toString().c_str());
  }

#ifdef ARDUINO_ARCH_ESP8266

  WiFiPhyMode phy;

  switch(wifiPhy)
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
#elif ARDUINO_ARCH_ESP32

  uint8_t curWifiChl;
  int8_t curTxPwr;
  wifi_second_chan_t curSecWifiChl;

  // reducing tx power improves throughput in low range
  esp_wifi_set_max_tx_power(wifiTpw);

  esp_wifi_get_channel(&curWifiChl,&curSecWifiChl);
  esp_wifi_get_max_tx_power(&curTxPwr);

  LOG_INFO("Wifi chl=%i",curWifiChl);
  LOG_INFO("Wifi tpw=%i",curTxPwr);

#endif

  udp.begin(DEFAULT_UDP_PORT);

  httpUpdater.setup(&httpServer);
  httpServer.begin();

#ifdef SERIAL_DEBUG_ENABLED
  //WiFi.printDiag(Serial);
#endif
}

void pdlSetupTelemetry(pdlDroneState*) {}

bool sendWlanLogPacket(pdlDroneState *ds)
{
  uint16_t pos = 0;
  bool result = false;

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreTake(semWlan,portMAX_DELAY);
#endif

  uint32_t logSize = pdlGetLogSize();
  // prepare wifi packet
  pos = writeWlanPacket(pos, wlanTxBuf, &droneId, sizeof(droneId));
  pos = writeWlanPacket(pos, wlanTxBuf, &wlanTxPacketNum, sizeof(wlanTxPacketNum));
  wlanTxBuf[pos++] = WLAN_LOG_PACKET; // log packet type
  pos = writeWlanPacket(pos, wlanTxBuf, pdlGetLog(), logSize);
  if(pos < WLAN_TX_BUF_SIZE)
    wlanTxBuf[pos++] = 0;   // string termination symbol

  if(sendWlanPacket(wlanTxBuf,pos))
  {
    pdlResetLog();
    result = true;
  }

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreGive(semWlan);
#endif

  return result;
}

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
      sendWlanLogPacket(ds);
    }
  }
  // receive udp packets in normal wifi mode
  if(!wifiBroadcastEnabled)
  {
    if(udp.parsePacket())
    {
      wlanRxBufLen = udp.read(wlanRxBuf, sizeof(wlanRxBuf));
    }
  }
  // parse command
  wlanRxBufBusy = 1;  // to prevent data corruption by onWifiBroadcastRx
  if(wlanRxBufLen > 0)
  {
    // we use droneId for data collision avoidance if many drones work at the same channel
    // droneId we put at the head of the packet
    uint32_t pktDroneId = 0;
    memcpy(&pktDroneId,&wlanRxBuf[0],sizeof(pktDroneId));
    if(droneId == pktDroneId)
    {
      if(!wifiBroadcastEnabled)
      {
        if(WiFi.getMode() == WIFI_STA && !hostIsSet()) // In WIFI_STA mode we trust only who first sent a packet to the drone
        {
          host = udp.remoteIP();
          hostPinned = true;
        }
        else  // In WIFI_AP mode we trust to anyone because we allow the only one connection to our AP
              // if host is reconnected to our AP it will have new IP but it will not cause the problem
        {
          host = udp.remoteIP();
          hostPinned = true;
        }
      }

      // skip droneId
      uint16_t pos = 4;
      // skip packetNum
      pos += 4;
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
  //return host.isSet(); // not available for ESP32
  return hostPinned;
}

int32_t getRssi()
{
#ifdef ARDUINO_ARCH_ESP8266
  if(wifiBroadcastEnabled)
  {
    return wbmGetRssi();
  }
  else if(WiFi.getMode() == WIFI_STA)
  {
    return WiFi.RSSI();
  }
  return -1;  // There is no way to determine RSSI in SoftAp mode
#elif ARDUINO_ARCH_ESP32
  return WiFi.RSSI();
#endif
}

bool sendWlanTelemetryPacket(pdlDroneState *ds)
{
  uint16_t pos = 0;
  size_t sz = sizeof(pdlDroneState);
  bool result = false;

  pdlUpdateTime(ds);
  ds->rc.rssi = getRssi();

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreTake(semWlan,portMAX_DELAY);
#endif

  // prepare wifi broadcast packet
  pos = writeWlanPacket(pos, wlanTxBuf, &droneId, sizeof(droneId));
  pos = writeWlanPacket(pos, wlanTxBuf, &wlanTxPacketNum, sizeof(wlanTxPacketNum));
  wlanTxBuf[pos++] = WLAN_TELEMETRY_PACKET; // telemetry packet type
  // DroneState size
  pos = writeWlanPacket(pos, wlanTxBuf, &sz, sizeof(sz));
  // DroneState
  pos = writeWlanPacket(pos, wlanTxBuf, ds, sz);
  // Send packet
  result = sendWlanPacket(wlanTxBuf,pos);

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreGive(semWlan);
#endif

  return result;
}

void pdlUpdateTelemetry(pdlDroneState *ds)
{
  if(!hostIsSet())
  {
    return;
  }

  sendWlanTelemetryPacket(ds);
}

uint16_t writeWlanPacket(uint16_t pos, byte *packet, void *value, size_t valueSize)
{
  if(pos + valueSize > WLAN_MAX_PAYLOAD_SIZE)
    return pos;

  memcpy(&packet[pos], value, valueSize);
  pos += valueSize;
  return pos;
}

#ifdef ARDUINO_ARCH_ESP8266

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

#endif

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

void pdlCmdSetDroneId(pdlDroneState* ds, const uint8_t* packet)
{
  (void)ds;
  char buf[24];
  uint32_t drone_id = 0;
  memcpy(&drone_id,&packet[1],sizeof(drone_id));
  saveParamToFile(PARAM_DRONE_ID,itoa(drone_id,buf,10));
}

void pdlCmdSetupWifi(pdlDroneState *ds, const uint8_t* packet)
{
  (void)ds;

  char buf[24];
  char sta[] = {'n',0};
  char dhcp[] = {'n',0};

  if(packet[1])
  {
    sta[0] = 'y';
  }

  if(packet[2])
  {
    dhcp[0] = 'y';
  }

  uint8_t chl = packet[3];
  uint8_t tpw = packet[4];
  uint8_t phy = packet[5];
  uint8_t rate = packet[6];

  setWifiParams(chl,tpw,phy,rate);

  // don't allow to change wifi param on the fly because it is not safe
  /*
  if(wifiBroadcastEnabled)
  {
    wbmSetChannel(wifiChl);
    wbmSetMaxTpw(wifiTpw);
    wbmSetPhyMode(wifiPhy);
    wbmSetRate(wifiRate);
  }
  */

  saveParamToFile(PARAM_WIFI_STA_MODE,sta);
  saveParamToFile(PARAM_USE_DHCP,dhcp);
  saveParamToFile(PARAM_WIFI_CHANNEL,itoa(chl,buf,10));
  saveParamToFile(PARAM_WIFI_TX_POWER_LEVEL,itoa(tpw,buf,10));
  saveParamToFile(PARAM_WIFI_PHY_MODE,itoa(phy,buf,10));
  saveParamToFile(PARAM_WIFI_RATE,itoa(rate,buf,10));
}

void pdlCmdEnableWifiBroadcast(pdlDroneState* ds, const uint8_t* packet)
{
  (void)ds;
  uint8_t oldWifiBroadcastEnabled = wifiBroadcastEnabled;
  wifiBroadcastEnabled = packet[1];
  // FIXME sometimes the drone resets if we send settings in wifi broadcast mode with delay between commands less than 100ms or if we try to switch back to normal wifi mode
#ifdef ARDUINO_ARCH_ESP8266
  if(wifiBroadcastEnabled && !oldWifiBroadcastEnabled)
  {
    udp.stop();
    picUdp.stop();
    httpServer.stop();
    httpServer.close();

    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);

    delay(1000);

    wbmSetRxFunc(onWifiBroadcastRx);
    wbmStartNormal();
    wbmSetChannel(wifiChl);
    wbmSetMaxTpw(wifiTpw);
    wbmSetPhyMode(wifiPhy);
    wbmSetRate(wifiRate);

    LOG_INFO("Wifi broadcast is enabled");
  }

  if(!wifiBroadcastEnabled && oldWifiBroadcastEnabled)
  {
    wbmStop();

    delay(1000);

    pdlSetupRc(ds);
    pdlSetupTelemetry(ds);

    LOG_INFO("Wifi broadcast is disabled");
  }
#else
  wifiBroadcastEnabled = 0;
  LOG_INFO("WiFi broadcast is not supported");
#endif
}

bool sendUdpPacket(uint8_t *data, uint16_t data_len)
{
  udp.beginPacket(host,DEFAULT_UDP_PORT);
  udp.write(data,data_len);

  return (udp.endPacket())?true:false;
}

bool sendWlanPacket(uint8_t *data, uint16_t data_len)
{
  bool result = false;

  if(hostIsSet() == false)
    return false;

#ifdef ARDUINO_ARCH_ESP32
  result = sendUdpPacket(data,data_len);
#elif ARDUINO_ARCH_ESP8266

  if(wifiBroadcastEnabled)
  {
    if( !wbmIsTxBusy() &&
         wbmGetRssi() != 0 ) // that we determine that host is connected after switching to the broadcast mode
    {
      if(wbmSendPktFreedom(data,data_len))
      {
        result = true;
      }
    }
  }
  else
  {
    result = sendUdpPacket(port,data,data_len);
  }
#endif

  if(result)
  {
    wlanTxPacketNum++;
  }

  return result;
}

size_t sendWlanPictureStartPacket(  uint16_t width,
                                    uint16_t height,
                                    uint8_t pixformat,
                                    uint8_t quality,
                                    uint64_t timestamp,
                                    size_t dataLen,
                                    uint8_t *data)
{
#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreTake(semWlan,portMAX_DELAY);
#endif

  size_t pos = 0;
  // prepare packet
  pos = writeWlanPacket(pos, wlanTxBuf, &droneId, sizeof(droneId));
  pos = writeWlanPacket(pos, wlanTxBuf, &wlanTxPacketNum, sizeof(wlanTxPacketNum));
  wlanTxBuf[pos++] = WLAN_PICTURE_START_PACKET;
  pos = writeWlanPacket(pos, wlanTxBuf, &width, sizeof(width));
  pos = writeWlanPacket(pos, wlanTxBuf, &height, sizeof(height));
  pos = writeWlanPacket(pos, wlanTxBuf, &pixformat, sizeof(pixformat));
  pos = writeWlanPacket(pos, wlanTxBuf, &quality, sizeof(quality));
  pos = writeWlanPacket(pos, wlanTxBuf, &timestamp, sizeof(timestamp));
  pos = writeWlanPacket(pos, wlanTxBuf, &dataLen, sizeof(dataLen));
  size_t len = WLAN_TX_BUF_SIZE - pos;
  if(dataLen < len)
    len = dataLen;
  pos = writeWlanPacket(pos, wlanTxBuf, data, len);
  if(sendWlanPacket(wlanTxBuf, pos) == false)
  {
    len = 0;
  }

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreGive(semWlan);
#endif

  return len;
}

size_t sendWlanPictureDataPacket(size_t dataOffset, size_t dataLen, uint8_t *data, uint16_t chunkNum)
{
#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreTake(semWlan,portMAX_DELAY);
#endif
  size_t pos = 0;
  // prepare packet
  pos = writeWlanPacket(pos, wlanTxBuf, &droneId, sizeof(droneId));
  pos = writeWlanPacket(pos, wlanTxBuf, &wlanTxPacketNum, sizeof(wlanTxPacketNum));
  wlanTxBuf[pos++] = WLAN_PICTURE_DATA_PACKET;
  pos = writeWlanPacket(pos, wlanTxBuf, &chunkNum, sizeof(chunkNum));
  size_t len = WLAN_TX_BUF_SIZE - pos;
  if( (dataLen - dataOffset) < len)
    len = dataLen - dataOffset;
  pos = writeWlanPacket(pos, wlanTxBuf, &data[dataOffset], len);
  if(sendWlanPacket(wlanTxBuf, pos) == false)
  {
    len = 0;
  }

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreGive(semWlan);
#endif

  return len;
}

size_t sendWlanPictureStartPacketV2(  uint16_t width,
                                      uint16_t height,
                                      uint8_t pixformat,
                                      uint8_t quality,
                                      uint64_t timestamp,
                                      size_t dataLen,
                                      uint8_t *data,
                                      size_t chunkSize)
{
#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreTake(semWlan,portMAX_DELAY);
#endif

  size_t pos = 0;
  // prepare packet
  pos = writeWlanPacket(pos, wlanTxBuf, &droneId, sizeof(droneId));
  pos = writeWlanPacket(pos, wlanTxBuf, &wlanTxPacketNum, sizeof(wlanTxPacketNum));
  wlanTxBuf[pos++] = WLAN_PICTURE_START_PACKET;
  pos = writeWlanPacket(pos, wlanTxBuf, &width, sizeof(width));
  pos = writeWlanPacket(pos, wlanTxBuf, &height, sizeof(height));
  pos = writeWlanPacket(pos, wlanTxBuf, &pixformat, sizeof(pixformat));
  pos = writeWlanPacket(pos, wlanTxBuf, &quality, sizeof(quality));
  pos = writeWlanPacket(pos, wlanTxBuf, &timestamp, sizeof(timestamp));
  pos = writeWlanPacket(pos, wlanTxBuf, &dataLen, sizeof(dataLen));
  size_t len = WLAN_TX_BUF_SIZE - pos;
  if(chunkSize < len)
    len = chunkSize;
  pos = writeWlanPacket(pos, wlanTxBuf, data, len);
  if(sendWlanPacket(wlanTxBuf, pos) == false)
  {
    len = 0;
  }

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreGive(semWlan);
#endif

  return len;
}

size_t sendWlanPictureDataPacketV2( uint8_t *data,
                                    size_t chunkSize,
                                    uint16_t chunkNum )
{
#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreTake(semWlan,portMAX_DELAY);
#endif
  size_t pos = 0;
  // prepare packet
  pos = writeWlanPacket(pos, wlanTxBuf, &droneId, sizeof(droneId));
  pos = writeWlanPacket(pos, wlanTxBuf, &wlanTxPacketNum, sizeof(wlanTxPacketNum));
  wlanTxBuf[pos++] = WLAN_PICTURE_DATA_PACKET;
  pos = writeWlanPacket(pos, wlanTxBuf, &chunkNum, sizeof(chunkNum));
  size_t len = WLAN_TX_BUF_SIZE - pos;
  if(chunkSize < len)
    len = chunkSize;
  pos = writeWlanPacket(pos, wlanTxBuf, &data[0], len);
  if(sendWlanPacket(wlanTxBuf, pos) == false)
  {
    len = 0;
  }

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreGive(semWlan);
#endif

  return len;
}

size_t sendWlanPictureLastDataPacketV2( uint8_t *data,
                                        size_t chunkSize,
                                        uint16_t chunkNum )
{
#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreTake(semWlan,portMAX_DELAY);
#endif
  size_t pos = 0;
  // prepare packet
  pos = writeWlanPacket(pos, wlanTxBuf, &droneId, sizeof(droneId));
  pos = writeWlanPacket(pos, wlanTxBuf, &wlanTxPacketNum, sizeof(wlanTxPacketNum));
  wlanTxBuf[pos++] = WLAN_PICTURE_LAST_DATA_PACKET;
  pos = writeWlanPacket(pos, wlanTxBuf, &chunkNum, sizeof(chunkNum));
  size_t len = WLAN_TX_BUF_SIZE - pos;
  if(chunkSize < len)
    len = chunkSize;
  pos = writeWlanPacket(pos, wlanTxBuf, &data[0], len);
  if(sendWlanPacket(wlanTxBuf, pos) == false)
  {
    len = 0;
  }

#ifdef ARDUINO_ARCH_ESP32
  xSemaphoreGive(semWlan);
#endif

  return len;
}
