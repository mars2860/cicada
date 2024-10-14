#ifndef WIFIBROADCASTMODEM_H_
#define WIFIBROADCASTMODEM_H_

#include <Arduino.h>
// you have to implement your own modem_config.h
#include "modem_config.h"

//-----------------------------------------------------------------------------
// MODEM UART PROTOCOL
// 0x3D 0x62 <uint8_t function code> <various data> <crc16>
// MESSAGES AND COMMANDS
// modem state:           0x3d 0x62 0x01 <uint8_t version> <uint8_t state> <uint8_t chl> <uint8_t txLevel> <uint8_t rate> < uint8_t phyMode> <crc16>
// start normal mode:     0x3d 0x62 0x02 <crc16>
// start sniffer mode:    0x3d 0x62 0x03 <crc16>
// stop:                  0x3d 0x62 0x04 <crc16>
// data:                  0x3d 0x62 0x05 <uint8_t data_len_low uint8_t data_len_high> <data> <crc16>
// set channel:           0x3d 0x62 0x06 <uint8_t chl 1 to 14> <crc16>
// set tx power:          0x3d 0x62 0x07 <uint8_t txPowerLevel 0 to 82> <crc16>
// set tx rate:           0x3d 0x62 0x08 <uint8_t txRate 0 to 82> <crc16>
// set phy mode:          0x3d 0x62 0x09 <uint8_t phyMode 1 to 3> <crc16>

// Rate values
// 0x0, //0 - B 1M   CCK
// 0x1, //1 - B 2M   CCK
// 0x5, //2 - G 2M   CCK Short Preamble
// 0x2, //3 - B 5.5M CCK
// 0x6, //4 - G 5.5M CCK Short Preamble
// 0x3, //5 - B 11M  CCK
// 0x7, //6 - G 11M  CCK Short Preamble
// 0xB, //7 - G 6M   ODFM
// 0xF, //8 - G 9M   ODFM
// 0xA, //9 - G 12M  ODFM
// 0xE, //A - G 18M  ODFM
// 0x9, //B - G 24M  ODFM
// 0xD, //C - G 36M  ODFM
// 0x8, //D - G 48M  ODFM
// 0xC, //E - G 54M  ODFM

// PHY mode values
// PHY_MODE_11B    = 1
// PHY_MODE_11G    = 2
// PHY_MODE_11N    = 3

//-----------------------------------------------------------------------------

#define MODEM_FW_VERSION   1

#define MSG_FIRST_BYTE   0x3D
#define MSG_SECOND_BYTE  0x62

#define MSG_FUNC_MODEM_STATE        0x01
#define MSG_FUNC_START_NORMAL       0x02
#define MSG_FUNC_START_SNIFFER      0x03
#define MSG_FUNC_STOP               0x04
#define MSG_FUNC_DATA               0x05
#define MSG_FUNC_SET_CHANNEL        0x06
#define MSG_FUNC_SET_TX_POWER       0x07
#define MSG_FUNC_SET_TX_RATE        0x08
#define MSG_FUNC_SET_PHY_MODE       0x09
#define MSG_FUNC_LOG                0x0A

#define MSG_FUNC_FIRST              0x01
#define MSG_FUNC_LAST               0x0A

//first byte - frame control
//https://www.geeksforgeeks.org/ieee-802-11-mac-frame/
//https://en.wikipedia.org/wiki/802.11_Frame_Types
//https://www.oreilly.com/library/view/80211-wireless-networks/0596100523/ch04.html
//each byte shifted from lower bits
//08 = {00 version, 01 frame type (DATA), 0000 subtype}, 01 = {toDS = 1, fromDS = 0}

constexpr uint8_t WLAN_IEEE_HEADER[] =
{
    0x08, 0x01, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
    0x10, 0x86
};

constexpr size_t WLAN_IEEE_HEADER_SIZE = sizeof(WLAN_IEEE_HEADER);
constexpr size_t WLAN_MAX_PACKET_SIZE = 1400;
constexpr size_t WLAN_MAX_PAYLOAD_SIZE = WLAN_MAX_PACKET_SIZE - WLAN_IEEE_HEADER_SIZE - 4;

static_assert(WLAN_IEEE_HEADER_SIZE == 24, "");

#define WIFI_DEFAULT_CHANNEL    7
#define WIFI_DEFAULT_TX_POWER   82
#define WIFI_DEFAULT_RATE       0xB
#define WIFI_DEFAULT_PHY        PHY_MODE_11G

#define STATE_DISCONNECTED    0
#define STATE_STOPPED         1
#define STATE_NORMAL          2
#define STATE_SNIFFER         3

#ifdef MODEM_SERIAL_ENABLED
  uint16_t wbmSendDataToHost(uint8_t *buf, uint16_t len, uint8_t isLog);
  void wbmUpdateSerial();
#endif

typedef void (*mdm_recv_cb_fn)(uint8_t *data, uint16_t len);

/** @return 1 if it is successed and 0 otherwise */
uint8_t wbmSendPktFreedom(uint8_t *data, uint16_t len);
void wbmSetChannel(uint8_t chl);
uint8_t wbmGetChannel();
void wbmSetMaxTpw(uint8_t tpw);
uint8_t wbmGetMaxTpw();
void wbmSetRate(uint8_t rate);
uint8_t wbmGetRate();
void wbmSetPhyMode(uint8_t mode);
uint8_t wbmGetPhyMode();
void wbmStartSniffer();
void wbmStop();
void wbmStartNormal();
void wbmSetup();
void wbmSetRxFunc(mdm_recv_cb_fn func);
int wbmGetRssi();
uint8_t wbmIsTxBusy();

#endif
