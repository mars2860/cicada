#include <ESP8266Wifi.h>
#include <string.h>
#include "WifiBroadcastModem.h"

extern "C"
{
  #include "wifi_raw.h"
}

#ifdef MODEM_SERIAL_ENABLED

static uint8_t uartRxBuf[1536];
static uint8_t uartTxBuf[1536];
static uint8_t uartChar = 0;
static uint8_t uartPrevChar = 0;
static uint16_t uartRxPos = 0;
static uint16_t uartTxPos = 0;
static uint8_t uartMsgDetected = 0;
static uint8_t uartFunc;
static uint16_t uartDataFrameLen;

static const unsigned short wCRCTable[] =
{
  0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
  0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
  0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
  0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
  0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
  0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
  0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
  0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
  0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
  0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
  0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
  0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
  0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
  0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
  0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
  0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
  0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
  0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
  0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
  0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
  0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
  0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
  0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
  0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
  0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
  0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
  0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
  0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
  0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
  0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
  0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
  0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};

uint16_t crc16(const uint8_t *nData, uint16_t wLength)
{
  unsigned char nTemp;
  unsigned short wCRCWord = 0xFFFF;
  while (wLength--)
  {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}

uint16_t wbmSendDataToHost(uint8_t *buf, uint16_t len, uint8_t isLog)
{
  if(len >= sizeof(uartTxBuf) - 7)
  {
    LOG_ERROR("Can't send data to host, dataLen=%i is too big",len);
    return 0;
  }

  if(uartTxPos != 0)  // test if we have sent previous data
  {
    return 0;
  }

  uartTxBuf[uartTxPos++] = MSG_FIRST_BYTE;
  uartTxBuf[uartTxPos++] = MSG_SECOND_BYTE;
  uartTxBuf[uartTxPos++] = (isLog) ? MSG_FUNC_LOG : MSG_FUNC_DATA;
  uartTxBuf[uartTxPos++] = (len & 0xFF);
  uartTxBuf[uartTxPos++] = ((len >> 8) & 0xFF);
  memcpy(&uartTxBuf[uartTxPos],buf,len);
  uartTxPos += len;
  uint16_t crc = crc16(uartTxBuf,uartTxPos);

  uint8_t crc_low = crc & 0xFF;
  uint8_t crc_high = (crc >> 8) & 0xFF;

  uartTxBuf[uartTxPos++] = crc_low;
  uartTxBuf[uartTxPos++] = crc_high;

  uint16_t written = uartTxPos;

  // if data size is lower than UART TX FIFO we can send it immediately
  if(Serial.availableForWrite() >= uartTxPos)
  {
    Serial.write(uartTxBuf,uartTxPos);
    uartTxPos = 0;
  }

  return written;
}

#endif

static uint8_t wbmWifiTxBuf[1536];
static uint8_t wbmWifiChannel = WIFI_DEFAULT_CHANNEL;
static uint8_t wbmWifiMaxTpw = WIFI_DEFAULT_TX_POWER;
static uint8_t wbmWifiPhy = WIFI_DEFAULT_PHY;
static uint8_t wbmWifiRate = WIFI_DEFAULT_RATE;
static uint8_t wbmWifiTxBusy = 0;
static int wbmWifiRssi = 0;

static uint8_t wbmState = STATE_STOPPED;

mdm_recv_cb_fn mdmRxFunc = NULL;

void packet_sent_cb(uint8_t status)
{
  (void)status;
  wbmWifiTxBusy = 0;
}

uint8_t wbmSendPktFreedom(uint8_t *data, uint16_t len)
{
  if(wbmWifiTxBusy)
  {
    LOG_ERROR("Wifi tx is busy");
    return 0;
  }

  if(len >= WLAN_MAX_PAYLOAD_SIZE)
  {
    LOG_ERROR("Can't send wifi data, len=%i is too big",len);
    return 0;
  }
  /*
  Packet has to be the whole 802.11 packet, excluding the FCS. The length of
  the packet has to be longer than the minimum length of the header of 802.11
  packet, which is 24 bytes, and less than 1400 bytes.
  • Duration area is invalid for user, it will be filled in SDK.
  • The rate of sending packet is same as the management packet which is the
  same as the system rate of sending packets.
  • Can send: unencrypted data packet, unencrypted beacon/probe req/probe
  resp.
  • Can NOT send: all encrypted packets (the encrypt bit in the packet has to be
  0, otherwise it is not supported), control packet, other management packet
  except unencrypted beacon/probe req/probe resp.
  • Only after the previous packet was sent, and the sent callback is entered, the
  next packet is allowed to send. Otherwise, wifi_send_pkt_freedom will return
  “fail”.
  */
  uint16_t pos = 0;
  memcpy(&wbmWifiTxBuf[pos], WLAN_IEEE_HEADER, WLAN_IEEE_HEADER_SIZE);
  pos += WLAN_IEEE_HEADER_SIZE;
  // TODO FEC ENCODING
  memcpy(&wbmWifiTxBuf[pos], data, len);
  pos += len;
  wbmWifiTxBusy = 1;
  int result = wifi_send_pkt_freedom(wbmWifiTxBuf, pos, true);
  if(result != 0)
  {
    LOG_ERROR("Can't send packet, something is wrong, len=%i",len);
  }
  else
  {
    LOG_DEBUG("Sent packet, data_len=%i\n, packet_len=%i", len, pos);
  }

  return (result)?0:1;
}

// Promiscuous RX callback function
void rx_cb_sniffer(uint8_t *buf, uint16_t len)
{
  if(mdmRxFunc != NULL)
  {
    mdmRxFunc(buf,len);
  }
#ifdef MODEM_SERIAL_ENABLED
  uint16_t written = wbmSendDataToHost(buf,len,false);
  if(written == 0 && uartTxPos != 0)
  {
    LOG_ERROR("Uart tx is busy. Received packet is lost, len=%i",len);
  }
#endif
}

// Raw RX callback function
void packet_received_cb(struct RxPacket *pkt)
{
  uint16_t len = pkt->rx_ctl.legacy_length;
  if (len <= WLAN_IEEE_HEADER_SIZE)
  {
    LOG_ERROR("Received WLAN packet is to small, len=%i",len);
    return;
  }

  //Serial.printf("Recv callback #%d: %d bytes\n", counter++, len);
  //Serial.printf("Channel: %d PHY: %d\n", pkt->rx_ctl.channel, wifi_get_phy_mode());

  //uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  //Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  uint8_t* data = pkt->data;
  // filter incoming packets by MAC
  if(memcmp(data + 10, &WLAN_IEEE_HEADER[10], 6) != 0)
  {
    return;
  }

  LOG_DEBUG("Received packet,len=%i",len);

  data += WLAN_IEEE_HEADER_SIZE;
  len -= WLAN_IEEE_HEADER_SIZE; //skip the 802.11 header

  len -= 4;//the received length has 4 more bytes at the end for some reason.

  wbmWifiRssi = pkt->rx_ctl.rssi;
  if(wbmWifiRssi > 0) // It is strange but in line above I get positive values
  {
    wbmWifiRssi = -wbmWifiRssi;
  }
  // TODO FEC DECODING
  if(mdmRxFunc != NULL)
  {
    mdmRxFunc(data,len);
  }

#ifdef MODEM_SERIAL_ENABLED
  uint16_t written = wbmSendDataToHost(data,len,false);
  if(written == 0 && uartTxPos != 0)
  {
    LOG_ERROR("Uart tx is busy. Received packet is lost, len=%i",len);
  }
#endif
}

#ifdef MODEM_SERIAL_ENABLED

void cmdModemState()
{
  uint8_t msg[16];
  uint8_t pos = 0;

  msg[pos++] = MSG_FIRST_BYTE;
  msg[pos++] = MSG_SECOND_BYTE;
  msg[pos++] = MSG_FUNC_MODEM_STATE;
  msg[pos++] = MODEM_FW_VERSION;
  msg[pos++] = wbmState;
  msg[pos++] = wbmGetChannel();
  msg[pos++] = wbmGetMaxTpw();
  msg[pos++] = wbmGetRate();
  msg[pos++] = wbmGetPhyMode();

  uint16_t crc = crc16(msg,pos);

  uint8_t crc_low = crc & 0xFF;
  uint8_t crc_high = (crc >> 8) & 0xFF;

  msg[pos++] = crc_low;
  msg[pos++] = crc_high;

  // don't care, serial is free when cmdModemState is invoked
  Serial.write(msg,pos);
}
#else
void cmdModemState() {}
#endif

void wbmSetChannel(uint8_t chl)
{
  struct softap_config config;

  if(chl < 1 || chl > 14)
  {
    chl = 7;
  }

  if(wifi_get_opmode() == SOFTAP_MODE)
  {
    wifi_set_channel(chl); // actually we can't set channel by this method in softap mode

    if(!wifi_softap_get_config(&config))
    {
      LOG_ERROR("Can't get softap config");
      return;
    }

    config.channel = chl; // in the documentation it is said that we can't set chl 14 for SOFTAP_MODE
    config.ssid_hidden = 1;
    config.beacon_interval = 1000;

    if(!wifi_softap_set_config_current(&config))
    {
      LOG_ERROR("Can't set softap config");
    }
  }
  else
  {
    if(!wifi_set_channel(chl))
    {
      LOG_ERROR("Can't set wifi chl=%i",chl);
    }
    else
    {
      LOG_DEBUG("Set wifi chl=%i",chl);
    }
  }

  // store channel to set it again when we switch on normal/sniffer mode
  wbmWifiChannel = chl;
}

uint8_t wbmGetChannel()
{
  uint8_t chl;
  struct softap_config config;

  if(wifi_get_opmode() == SOFTAP_MODE)
  {
    if(!wifi_softap_get_config(&config))
    {
      LOG_ERROR("Can't get softap config");
    }
    chl = config.channel;
  }
  else
  {
    chl = wifi_get_channel();
  }

  return chl;
}

void wbmSetMaxTpw(uint8_t tpw)
{
  if(tpw > 82)
  {
    tpw = 82;
  }
  wbmWifiMaxTpw = tpw;
  system_phy_set_max_tpw(tpw);
  LOG_DEBUG("Set wifi maxTpw=%i",tpw);
}

uint8_t wbmGetMaxTpw()
{
  return wbmWifiMaxTpw;
}

void wbmSetRate(uint8_t rate)
{
  if(wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL,rate) != 0)
  {
    LOG_ERROR("Can't set wifi rate=%i",rate);
  }
  else
  {
    LOG_DEBUG("Set wifi rate=%i",rate);
  }

  wbmWifiRate = rate;
}

uint8_t wbmGetRate()
{
  uint8_t enable_mask;
  uint8_t rate;

  if(wifi_get_user_fixed_rate(&enable_mask,&rate) != 0)
  {
    LOG_ERROR("wifi_get_user_fixed_rate returns failure");
  }

  return rate;
}

void wbmSetPhyMode(uint8_t mode)
{
  if(mode < 1 || mode > 3)
  {
    mode = 1;
  }

  if(!wifi_set_phy_mode((phy_mode_t)mode))
  {
    LOG_ERROR("Can't set wifi phy=%i",mode);
  }
  else
  {
    LOG_DEBUG("Set wifi phy=%i",mode);
  }

  wbmWifiPhy = mode;
}

uint8_t wbmGetPhyMode()
{
  return wifi_get_phy_mode();
}

void wbmStartSniffer()
{
  wifi_station_disconnect();
  wifi_set_opmode_current(STATION_MODE);
  wifi_promiscuous_enable(false);
  wifi_raw_set_recv_cb(NULL);
  wifi_set_promiscuous_rx_cb(rx_cb_sniffer);
  wifi_promiscuous_enable(true);
  wbmWifiTxBusy = 0;
  wifi_register_send_pkt_freedom_cb(packet_sent_cb);

  wbmSetChannel(wbmWifiChannel);

  wbmState = STATE_SNIFFER;

  LOG_DEBUG("Start sniffer");
}

void wbmStop()
{
  wifi_station_disconnect();
  wifi_promiscuous_enable(false);
  wifi_set_promiscuous_rx_cb(NULL);
  wifi_raw_set_recv_cb(NULL);
  wbmWifiTxBusy = 0;
  wbmWifiRssi = 0;

  wbmState = STATE_STOPPED;

  LOG_DEBUG("Stop");
}

void wbmStartNormal()
{
  wifi_station_disconnect();
  wifi_promiscuous_enable(false);
  wifi_set_promiscuous_rx_cb(NULL);
  wbmWifiTxBusy = 0;
  wifi_register_send_pkt_freedom_cb(packet_sent_cb);

  if(!wifi_set_opmode_current(SOFTAP_MODE))
  {
    LOG_ERROR("Can't set SOFTAP_MODE");
  }

  wbmSetChannel(wbmWifiChannel);

  wifi_raw_set_recv_cb(packet_received_cb); // in wifi_raw it is said that this hook works only in softap_mode but I tested it successfully in station_mode

  wbmState = STATE_NORMAL;
  LOG_DEBUG("Start normal");
}

#ifdef MODEM_SERIAL_ENABLED

void resetParser()
{
  uartRxPos = 0;
  uartPrevChar = 0;
  uartMsgDetected = 0;
  uartFunc = 0;
  uartChar = 0;
  uartDataFrameLen = 0;
}

void readSerial()
{
  while(Serial.available() > 0)
  {
    uartPrevChar = uartChar;
    uartChar = Serial.read();

    LOG_DEBUG("%X ",uartChar);

    if( uartChar == MSG_SECOND_BYTE &&
        uartPrevChar == MSG_FIRST_BYTE &&
        !uartMsgDetected )
    {
      uartRxBuf[0] = uartPrevChar;
      uartRxBuf[1] = uartChar;
      uartRxPos = 2;
      uartMsgDetected = 1;
      uartDataFrameLen = 0;
      uartFunc = 0;

      LOG_DEBUG("New packet");

      continue;
    }

    if(!uartMsgDetected)
      continue;

    uartRxBuf[uartRxPos++] = uartChar;

    if(uartRxPos >= sizeof(uartRxBuf))
    {
      resetParser();
      LOG_ERROR("Uart rx buf is overflowed, func=%i", uartFunc);
      continue;
    }

    uartFunc = uartRxBuf[2];

    if(uartFunc < MSG_FUNC_FIRST || uartFunc > MSG_FUNC_LAST)
    {
      resetParser();
      LOG_ERROR("Received uart cmd has wrong FUNC");
      continue;
    }

    switch(uartFunc)
    {
    case MSG_FUNC_START_NORMAL:
    case MSG_FUNC_START_SNIFFER:
    case MSG_FUNC_STOP:
    case MSG_FUNC_MODEM_STATE:
      if(uartRxPos < 5)
      {
        continue;
      }
      break;
    case MSG_FUNC_SET_CHANNEL:
    case MSG_FUNC_SET_TX_POWER:
    case MSG_FUNC_SET_TX_RATE:
    case MSG_FUNC_SET_PHY_MODE:
      if(uartRxPos < 6)
      {
        continue;
      }
      break;
    case MSG_FUNC_DATA:
      if(uartRxPos < 5)
      {
        continue;
      }

      if(uartDataFrameLen == 0)
      {
        uartDataFrameLen = (((uint16_t)uartRxBuf[4]) << 8) & 0xFF00;
        uartDataFrameLen |= (((uint16_t)uartRxBuf[3]) & 0xFF);

        if((size_t)(uartRxPos + uartDataFrameLen + 2) >= sizeof(uartRxBuf))
        {
          resetParser();
          LOG_ERROR("Received uart data is too big, len=%i", uartDataFrameLen);
          continue;
        }
      }

      if(uartRxPos < uartDataFrameLen + 7)
      {
        continue;
      }
      break;
    default:
      break;
    }

    uint16_t crc = crc16(uartRxBuf,uartRxPos - 2);
    uint8_t crc_low = crc & 0xFF;
    uint8_t crc_high = (crc >> 8) & 0xFF;

    LOG_DEBUG("Process command");

    if( crc_low != uartRxBuf[uartRxPos-2] ||
        crc_high != uartRxBuf[uartRxPos-1] )
    {
      resetParser();
      LOG_ERROR("Received uart cmd has wrong CRC=%X %X",crc_high,crc_low);
      continue;
    }

    switch(uartFunc)
    {
    case MSG_FUNC_MODEM_STATE:
      cmdModemState();
      break;
    case MSG_FUNC_SET_CHANNEL:
      wbmSetChannel(uartRxBuf[3]);
      break;
    case MSG_FUNC_STOP:
      wbmStop();
      break;
    case MSG_FUNC_START_NORMAL:
      wbmStartNormal();
      break;
    case MSG_FUNC_START_SNIFFER:
      wbmStartSniffer();
      break;
    case MSG_FUNC_SET_TX_POWER:
      wbmSetMaxTpw(uartRxBuf[3]);
      break;
    case MSG_FUNC_SET_TX_RATE:
      wbmSetRate(uartRxBuf[3]);
      break;
    case MSG_FUNC_SET_PHY_MODE:
      wbmSetPhyMode(uartRxBuf[3]);
      break;
    case MSG_FUNC_DATA:
      wbmSendPktFreedom(&uartRxBuf[5],uartDataFrameLen);
      break;
    default:
      LOG_ERROR("Unknown function");
      break;
    }

    resetParser();
  }
}

void wbmUpdateSerial()
{
  readSerial();

  if(uartTxPos != 0)
  {
    Serial.write(uartTxBuf,uartTxPos);
    uartTxPos = 0;
  }
}

#endif

void wbmSetup()
{
  wbmSetPhyMode(wbmWifiPhy);
  wbmSetChannel(wbmWifiChannel);
  wbmSetRate(wbmWifiRate);
  wbmSetMaxTpw(wbmWifiMaxTpw);
}

void wbmSetRxFunc(mdm_recv_cb_fn func)
{
  mdmRxFunc = func;
}

int wbmGetRssi()
{
  return wbmWifiRssi;
}

uint8_t wbmIsTxBusy()
{
  return wbmWifiTxBusy;
}
