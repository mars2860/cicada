#ifndef DRIVERS_CAMERA_RUNCAM_H_
#define DRIVERS_CAMERA_RUNCAM_H_

#include <arduino.h>
#include <SoftwareSerial.h>

#include "pdl.h"

#define RUNCAM_BAUDRATE       115200

// Only Runcam Split4 is supported

class RuncamBase
{
public:
  static const uint8_t PACKET_HEADER = 0xCC;
  static const uint8_t CMD_GET_DEVICE_INFO = 0x00;
  static const uint8_t CMD_CAMERA_CONTROL = 0x01;

  static const uint8_t ACTION_SIMULATE_PWR_BTN = 0x01;
  static const uint8_t ACTION_START_VIDEO = 0x03;
  static const uint8_t ACTION_STOP_VIDEO = 0x04;
public:
  uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
  {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii)
    {
      if(crc & 0x80)
      {
        crc = (crc << 1) ^ 0xD5;
      }
      else
      {
        crc = crc << 1;
      }
    }

    return crc;
  }

  uint8_t prepareDeviceInfoPacket(uint8_t *packet, uint8_t len)
  {
    if(len < 3)
      return 0;

    packet[0] = PACKET_HEADER;
    packet[1] = CMD_GET_DEVICE_INFO;

    uint8_t crc = 0;
    for(uint8_t i = 0; i < 2; i++)
    {
      crc = crc8_dvb_s2(crc, packet[i]);
    }
    packet[2] = crc;

    return 3;
  }

  uint8_t prepareCameraControlPacket(uint8_t *packet, uint8_t len, uint8_t action)
  {
    if(len < 4)
      return 0;

    packet[0] = PACKET_HEADER;
    packet[1] = CMD_CAMERA_CONTROL;
    packet[2] = action;

    uint8_t crc = 0;
    for(uint8_t i = 0; i < 3; i++)
    {
      crc = crc8_dvb_s2(crc, packet[i]);
    }
    packet[3] = crc;

    return 4;
  }

  uint8_t prepareDrawStringHor(uint8_t *packet, uint8_t len, const char* text, uint8_t textLen, uint8_t x, uint8_t y)
  {
    // THIS DOESN'T WORK ON RUNCAM SPLIT 4 V2.0.4
    // When I send this command the runcam is dead
    if(len < textLen + 6 || !textLen)
      return 0;

    packet[0] = PACKET_HEADER;
    packet[1] = 0x22;
    packet[2] = textLen;
    packet[3] = x;
    packet[4] = y;

    memcpy(&packet[5],text,textLen);

    uint8_t crc = 0;
    for(uint8_t i = 0; i < textLen + 5; i++)
    {
      crc = crc8_dvb_s2(crc, packet[i]);
    }
    packet[textLen+5] = crc;

    return textLen+6;

    /* all code below also kills the runcam
    uint8_t p = 0;
    uint8_t crc = 0;

    packet[p++] = PACKET_HEADER;
    packet[p++] = 0x22;
    packet[p++] = 4;
    packet[p++] = 10;
    packet[p++] = 10;
    packet[p++] = 'T';
    packet[p++] = 'E';
    packet[p++] = 'S';
    packet[p++] = 'T';

    for(uint8_t i = 0; i < p; i++)
    {
      crc = crc8_dvb_s2(crc, packet[i]);
    }

    packet[p++] = crc;
    */

    /*
    packet[p++] = PACKET_HEADER;
    packet[p++] = 0x20;
    packet[p++] = 0;
    packet[p++] = 0;
    packet[p++] = 10;
    packet[p++] = 10;
    packet[p++] = 'X';

    for(uint8_t i = 0; i < p; i++)
    {
      crc = crc8_dvb_s2(crc, packet[i]);
    }

    packet[p++] = crc;

    return p;
    */
  }

  virtual ~RuncamBase() {}
  virtual bool init() = 0;
  virtual uint8_t request(uint8_t *packet, uint8_t len) = 0;
  virtual uint8_t readAnswer() = 0;
  virtual void printAnswer(uint8_t *answer, uint8_t len);

  void reqDeviceInfo()
  {
    uint8_t packet[4];
    uint8_t len = prepareDeviceInfoPacket(packet,sizeof(packet));
    request(packet,len);
  }

  void reqStartVideo()
  {
    // IT IS NOT SUPPORTED BY RUNCAM 4 SPLIT

    uint8_t packet[5];
    uint8_t len = prepareCameraControlPacket(packet,sizeof(packet),ACTION_START_VIDEO);
    request(packet,len);
  }

  void reqStopVideo()
  {
    // IT IS NOT SUPPORTED BY RUNCAM 4 SPLIT

    uint8_t packet[5];
    uint8_t len = prepareCameraControlPacket(packet,sizeof(packet),ACTION_STOP_VIDEO);
    request(packet,len);
  }

  void reqSimulatePwrBtn()
  {
    uint8_t packet[5];
    uint8_t len = prepareCameraControlPacket(packet,sizeof(packet),ACTION_SIMULATE_PWR_BTN);
    request(packet,len);
  }

  void drawStringHor(const char* text, uint8_t textLen, uint8_t x, uint8_t y)
  {
    // IT IS NOT SUPPORTED BY RUNCAM 4 SPLIT
    uint8_t packet[128];
    uint8_t len = prepareDrawStringHor(packet,sizeof(packet),text,textLen,x,y);
    request(packet,len);
  }
};

class Runcam: public RuncamBase
{
private:
  EspSoftwareSerial::UART rcSerial;
  uint8_t rxBuf[16];
  uint8_t rxPin;
  uint8_t txPin;
  bool available;
public:
  Runcam(uint8_t rx_pin, uint8_t tx_pin)
  {
    rxPin = rx_pin;
    txPin = tx_pin;
    available = false;
  }

  ~Runcam() {}
  uint8_t request(uint8_t *packet, uint8_t len)
  {
    return rcSerial.write(packet,len);
  }

  uint8_t readAnswer()
  {
    uint8_t pos = 0;
    while(rcSerial.available() > 0 && pos < sizeof(rxBuf))
    {
      rxBuf[pos] = rcSerial.read();
      pos++;
    }

    return pos;
  }

  void printAnswer(uint8_t *answer, uint8_t len)
  {
    for(uint8_t i = 0; i < len; i++)
    {
      if(i != 0)
        pdl_printf(",");
      pdl_printf("0x%x",answer[i]);
    }
    pdl_printf("\n");
  }

  bool init()
  {
    rcSerial.begin( RUNCAM_BAUDRATE,
                    EspSoftwareSerial::SWSERIAL_8N1,
                    rxPin,
                    txPin,
                    false);
    for(uint8_t i = 0; i < 3; i++)
    {
      reqDeviceInfo();
      delay(100); // wait for answer
      uint8_t len = readAnswer();
      if(len > 0 && rxBuf[0] == PACKET_HEADER)
      {
        pdl_printf("Runcam Info=");
        printAnswer(rxBuf,len);
        available = true;
        break;
      }
      else
      {
        available = false;
      }
    }

    return available;
  }

  bool isAvailable()
  {
    return available;
  }

};

#endif /* DRIVERS_CAMERA_RUNCAM_H_ */
