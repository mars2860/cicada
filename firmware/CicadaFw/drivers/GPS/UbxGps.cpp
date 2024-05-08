#include <Arduino.h>
#include "UbxGps.h"
#include "pdl.h"

#define UBX_HEADER1 0xB5
#define UBX_HEADER2 0x62

#define UBX_PAYLOAD_OFFSET      6
#define UBX_EMPTY_PACKET_SIZE   8

const uint8_t UBX_HEADER[] = {UBX_HEADER1,UBX_HEADER2};

uint8_t ubxRxBuf[UBX_RX_SIZE];

int32_t ubxRead()
{
  return Serial.read();
}

int32_t ubxWrite(uint8_t c)
{
  return Serial.write(c);
}

int32_t ubxAvailable()
{
  return Serial.available();
}

int32_t ubxWaitTxComplete()
{
  Serial.flush();
  return UBX_ERR_OK;
}

uint16_t ubxGetMsgId(uint8_t *packet)
{
  uint16_t id = packet[3];
  id <<= 8;
  id |= packet[2];
  return id;
}

int32_t ubxPreparePacket( uint8_t* buf,
                          uint32_t bufSize,
                          uint16_t clsMsgId,
                          void* payload,
                          uint32_t payloadSize)
{
  uint32_t packSize = payloadSize + UBX_EMPTY_PACKET_SIZE;

  if(bufSize < packSize)
    return UBX_ERR_SIZE;

  buf[0] = UBX_HEADER1;
  buf[1] = UBX_HEADER2;
  buf[2] = clsMsgId & 0xFF;
  buf[3] = (clsMsgId >> 8) & 0xFF;
  buf[4] = payloadSize & 0xFF;
  buf[5] = (payloadSize >> 8) & 0xFF;

  memcpy(&buf[6],payload,payloadSize);

  uint16_t chk = ubxChkPacket(buf,packSize);

  buf[packSize - 2] = chk & 0xFF;
  buf[packSize - 1] = (chk >> 8) & 0xFF;

  return UBX_ERR_OK;
}

uint16_t ubxChkPacket(uint8_t *buf, uint32_t size)
{
  uint8_t checksum[2] = {0,0};
  uint16_t result = 0;

  if(size < UBX_EMPTY_PACKET_SIZE)
    return 0;

  for(uint32_t i = 2; i < size - 2; i++)
  {
    checksum[0] += buf[i];
    checksum[1] += checksum[0];
  }

  result = checksum[1];
  result <<= 8;
  result |= checksum[0];

  return result;
}

int32_t ubxReadPacket()
{
  static uint32_t savedPos = 0;
  static uint32_t payloadLength = 0;

  uint32_t p = savedPos;

  //pdl_printf("r=%i\n",p);

  while(ubxAvailable() > 0)
  {
    ubxRxBuf[p] = ubxRead();

    //pdl_printf("%x ",ubxRxBuf[p]);

    // Carriage is at the first or the second sync byte, should be equals.
    if(p < 2)
    {
      if(ubxRxBuf[p] != UBX_HEADER[p])
      {
        p = 0;
        savedPos = 0;
        payloadLength = 0;
        continue;
      }
    }

    if(p == 5) // get payload length
    {
      payloadLength = ubxRxBuf[5];
      payloadLength <<= 8;
      payloadLength |= ubxRxBuf[4];

      //pdl_printf("\nlen=%i\n",payloadLength);
    }

    if(p == payloadLength + UBX_EMPTY_PACKET_SIZE - 1)
    {
      uint16_t packetChk = ubxRxBuf[p];
      packetChk <<= 8;
      packetChk |= ubxRxBuf[p-1];
      uint16_t chk = ubxChkPacket(&ubxRxBuf[0],p+1);

      p = 0;
      savedPos = 0;
      payloadLength = 0;

      //pdl_printf("pchk=%i,chk=%i",packetChk,chk);

      if(packetChk == chk)
        return UBX_ERR_OK;

      return UBX_ERR_BAD_CHK;
    }

    p++;
    savedPos = p;

    if(p >= UBX_RX_SIZE)
    {
      p = 0;
      savedPos = 0;
      payloadLength = 0;
      //pdl_printf("low buf");
      return UBX_ERR_LOW_RX_BUF;
    }
  }

  return UBX_ERR_EMPTY;
}

int32_t ubxSendPacket(uint8_t *buf, uint32_t size)
{
  UbxAckNack answer;
  uint8_t stat = 0;

  memset(&answer,0,sizeof(answer));

  if(!buf)
    return UBX_ERR_PTR;

  // clear rx buf from other messages that can overflow buffer and we don't see answer
  while(ubxAvailable())
  {
    ubxRead();
  }

  //pdl_printf("s\n");

  for(uint32_t i = 0; i < size; i++)
  {
    ubxWrite(buf[i]);
    //pdl_printf("%x ",buf[i]);
  }

  //pdl_printf("\n");

  ubxWaitTxComplete();

  for(int i = 0; i < 15; i++)
  {
    while(ubxAvailable() > 0)
    {
      stat = ubxReadAckNack(&answer);

      if(answer.clsId == buf[2] && answer.msgId == buf[3])
      {
        //pdl_printf("ack=%i,clsId=%i,msgId=%i\n",stat,answer.clsId,answer.msgId);
        return stat;
      }
    }

    delay(10);
  }

  return UBX_ERR_EMPTY;
}

int32_t ubxSendCfgCfg(UbxCfgCfg *pMsg)
{
  if(!pMsg)
    return UBX_ERR_PTR;

  int32_t result;
  uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_CFG_SIZE];

  result = ubxPreparePacket( packet,
                             sizeof(packet),
                             UBX_CFG_CFG,
                             pMsg,
                             UBX_CFG_CFG_SIZE);

  if(result != UBX_ERR_OK)
    return result;

  return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxSendCfgMsg(UbxCfgMsg *pMsg)
{
  if(!pMsg)
    return UBX_ERR_PTR;

  int32_t result;
  uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_MSG_SIZE];

  result = ubxPreparePacket( packet,
                             sizeof(packet),
                             UBX_CFG_MSG,
                             pMsg,
                             UBX_CFG_MSG_SIZE);

  if(result != UBX_ERR_OK)
    return result;

  return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxSendCfgInf(UbxCfgInf *pMsg)
{
  if(!pMsg)
     return UBX_ERR_PTR;

   int32_t result;
   uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_INF_SIZE];

   result = ubxPreparePacket( packet,
                              sizeof(packet),
                              UBX_CFG_INF,
                              pMsg,
                              UBX_CFG_INF_SIZE);

   if(result != UBX_ERR_OK)
     return result;

   return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxSendCfgNav5(UbxCfgNav5 *pMsg)
{
  if(!pMsg)
    return UBX_ERR_PTR;

  int32_t result;
  uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_NAV5_SIZE];

  result = ubxPreparePacket(  packet,
                              sizeof(packet),
                              UBX_CFG_NAV5,
                              pMsg,
                              UBX_CFG_NAV5_SIZE);

  if(result != UBX_ERR_OK)
    return result;

  return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxSendCfgRate(UbxCfgRate *pMsg)
{
  if(!pMsg)
    return UBX_ERR_PTR;

  int32_t result;
  uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_RATE_SIZE];

  pMsg->navRate = 1; // Navigation Rate, in number of measurement
                     // cycles. This parameter cannot be changed, and
                     // must be set to 1.

  result = ubxPreparePacket( packet,
                             sizeof(packet),
                             UBX_CFG_RATE,
                             pMsg,
                             UBX_CFG_RATE_SIZE);

  if(result != UBX_ERR_OK)
    return result;

  return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxSendCfgPrt(UbxCfgPrt *pMsg)
{
  if(!pMsg)
    return UBX_ERR_PTR;

  int32_t result;
  uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_PRT_SIZE];

  result = ubxPreparePacket(  packet,
                              sizeof(packet),
                              UBX_CFG_PRT,
                              pMsg,
                              UBX_CFG_PRT_SIZE);

  /*pdl_printf("s\n");
  for(uint32_t i = 0; i < sizeof(packet); i++)
  {
    pdl_printf("%x ",packet[i]);
  }*/

  if(result != UBX_ERR_OK)
    return result;

  return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxSendCfgGnss(UbxCfgGnss *pMsg)
{
  if(!pMsg)
      return UBX_ERR_PTR;

  int32_t result;
  uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_GNSS_SIZE];

  result = ubxPreparePacket(  packet,
                              sizeof(packet),
                              UBX_CFG_GNSS,
                              pMsg,
                              UBX_CFG_GNSS_SIZE);

  if(result != UBX_ERR_OK)
    return result;

  /*
  pdl_printf("CFG_GNSS\n");
  for(uint8_t i = 0; i < sizeof(packet); i++)
  {
    pdl_printf("%x ", packet[i]);
  }
  */

  return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxSendCfgRst(UbxCfgRst *pMsg)
{
  if(!pMsg)
     return UBX_ERR_PTR;

   int32_t result;
   uint8_t packet[UBX_EMPTY_PACKET_SIZE + UBX_CFG_RST_SIZE];

   result = ubxPreparePacket( packet,
                              sizeof(packet),
                              UBX_CFG_RST,
                              pMsg,
                              UBX_CFG_RST_SIZE);

   if(result != UBX_ERR_OK)
     return result;

   return ubxSendPacket(packet,sizeof(packet));
}

int32_t ubxReadAckNack(UbxAckNack *pMsg)
{
  if(!pMsg)
    return UBX_ERR_PTR;

  int32_t result = ubxReadPacket();
  if(result == UBX_ERR_OK)
  {
    uint16_t id = ubxGetMsgId(ubxRxBuf);
    if(id == UBX_ACK_NACK || id == UBX_ACK_ACK)
    {
      memcpy(pMsg,&ubxRxBuf[UBX_PAYLOAD_OFFSET],sizeof(UbxAckNack));

      if(id == UBX_ACK_ACK)
      {
        return UBX_ERR_ACK;
      }

      return UBX_ERR_NACK;
    }
  }

  return result;
}

int32_t ubxReadNavPvt(UbxNavPvt *pMsg)
{
  if(!pMsg)
    return UBX_ERR_PTR;

  int32_t result = ubxReadPacket();
  if(result == UBX_ERR_OK)
  {
    if(ubxGetMsgId(ubxRxBuf) == UBX_NAV_PVT)
    {
      memcpy(pMsg,&ubxRxBuf[UBX_PAYLOAD_OFFSET],sizeof(UbxNavPvt));
      return UBX_ERR_OK;
    }
  }

  return result;
}

int32_t ubxRestoreDefaults()
{
  /*
    // CFG-CFG packet.
    byte packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x09, // id
        0x0D, // length
        0x00, // length
        0xFF, // clearMask
        0xFF, // clearMask
        0x00, // clearMask
        0x00, // clearMask
        0x00, // saveMask
        0x00, // saveMask
        0x00, // saveMask
        0x00, // saveMask
        0xFF, // loadMask
        0xFF, // loadMask
        0x00, // loadMask
        0x00, // loadMask
        0x17, // deviceMask
        0x2F, // CK_A
        0xAE, // CK_B
    };
  */
  UbxCfgCfg msg;
  memset(&msg,0,sizeof(msg));
  msg.clearMask = 0xFFFF;
  msg.loadMask = 0xFFFF;
  msg.deviceMask = 0x17;
  return ubxSendCfgCfg(&msg);
}

int32_t ubxReset(uint8_t resetMode, uint16_t navBbrMask)
{
  UbxCfgRst msg;
  memset(&msg,0,sizeof(msg));
  msg.resetMode = resetMode;
  msg.navBbrMask = navBbrMask;
  return ubxSendCfgRst(&msg);
}

int32_t ubxDisableNmea()
{
  int32_t result = UBX_ERR_OK;
  int32_t t;
  UbxCfgMsg msg;
  UbxCfgInf msgInf;
  memset(&msg,0,sizeof(msg));
  memset(&msgInf,0,sizeof(msgInf));
  uint8_t msgId[][2] =
  {
    {0xF0, 0x00},   // GGA
    {0xF0, 0x01},   // GLL
    {0xF0, 0x02},   // GSA
    {0xF0, 0x03},   // GSV
    {0xF0, 0x04},   // RMC
    {0xF0, 0x05},   // VTG
    //{0xF0, 0x06},   // GRS
    //{0xF0, 0x07},   // GST
    //{0xF0, 0x08},   // ZDA
    //{0xF0, 0x09},   // GBS
    //{0xF0, 0x0A},   // DTM
    //{0xF0, 0x0D},   // GNS
    //{0xF0, 0x0E},   // THS
    //{0xF0, 0x0F},   // VLW

    //{0xF1, 0x00},   // PUBX00
    //{0xF1, 0x01},   // PUBX01
    //{0xF1, 0x03},   // PUBX02
    //{0xF1, 0x04},   // PUBX03
    //{0xF1, 0x05},   // PUBX04
  };

  for(int i = 0; i < 6; i++)
  {
    msg.msgCls = msgId[i][0];
    msg.msgId = msgId[i][1];

    delay(10);
    t = ubxSendCfgMsg(&msg);

    if(t != UBX_ERR_OK)
    {
      //pdl_printf("can't disable %i\n,err=%i",i,t);
      result = t;
    }

    //if(result == UBX_ERR_OK)
    //  pdlResetLog();
  }

  // disable GNTXT messages
  msgInf.protocolId = 1; // NMEA
  t = ubxSendCfgInf(&msgInf);
  if(t != UBX_ERR_OK)
  {
    //pdl_printf("Can't disable GNTXT,err=%i",t);
    result = t;
  }

  return result;
}

int32_t ubxEnableNavPvt()
{
  /*
    // CFG-MSG packet.
    byte packet[] = {
        0xB5,          // sync char 1
        0x62,          // sync char 2
        0x06,          // class
        0x01,          // id
        0x03,          // length
        0x00,          // length
        0x01,          // payload
        0x07,          // payload
        0x01,          // payload
        0x13, // CK_A
        0x51, // CK_B
    };
  */
  UbxCfgMsg msg;
  memset(&msg,0,sizeof(msg));

  msg.msgCls = 0x01;
  msg.msgId = 0x07;
  msg.uart1Rate = 1;

  return ubxSendCfgMsg(&msg);
}

int32_t ubxSetUartBaudrate(uint32_t baudrate)
{
  /*
       // CFG-PRT packet.
    byte packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x00, // id
        0x14, // length
        0x00, // length
        0x01, // portId
        0x00, // reserved0
        0x00, // txReady
        0x00, // txReady
        0xD0, // mode
        0x08, // mode
        0x00, // mode
        0x00, // mode
        0x00, // baudrate
        0xC2, // baudrate
        0x01, // baudrate
        0x00, // baudrate
        0x07, // inProtoMask
        0x00, // inProtoMask
        0x03, // outProtoMask
        0x00, // outProtoMask
        0x00, // flags
        0x00, // flags
        0x00, // reserved
        0x00, // reserved
        0xC0, // CK_A
        0x7E, // CK_B
    };
  */
  UbxCfgPrt msg;
  memset(&msg,0,sizeof(msg));

  msg.portId = 1;
  msg.mode = 0x08D0;
  msg.baudRate = baudrate;
  msg.inProtoMask = 0x07;
  msg.outProtoMask = 0x03;

  return ubxSendCfgPrt(&msg);
}

int32_t ubxSetMeasRate(uint32_t measRate)
{
  /*
  // CFG-RATE packet.
  byte packet[] = {
      0xB5, // sync char 1
      0x62, // sync char 2
      0x06, // class
      0x08, // id
      0x06, // length
      0x00, // length
      0x64, // payload
      0x00, // payload
      0x01, // payload
      0x00, // payload
      0x01, // payload
      0x00, // payload
      0x7A, // CK_A
      0x12, // CK_B
  };
  */
  UbxCfgRate msg;
  memset(&msg,0,sizeof(msg));

  msg.measRate = measRate;
  msg.timeRef = 1;  // GPS Time;

  return ubxSendCfgRate(&msg);
}

int32_t ubxSetPedestrianDynModel()
{
  UbxCfgNav5 msg;
  memset(&msg,0,sizeof(msg));

  msg.mask = 0x1 | 0x4;     // apply dynamic model & fix mode
  msg.dynModel = 0x3;       // pedestrian
  msg.fixMode = 2;          // 3D only

  return ubxSendCfgNav5(&msg);
}

int32_t ubxSetAirborn2gDynModel()
{
  UbxCfgNav5 msg;
  memset(&msg,0,sizeof(msg));

  msg.mask = 0x1 | 0x4;     // apply dynamic model & fix mode
  msg.dynModel = 0x7;       // airborn 2g
  msg.fixMode = 3;          // Auto 2D/3D

  return ubxSendCfgNav5(&msg);
}

int32_t ubxSetGlonassEnabled(uint8_t enabled)
{
  UbxCfgGnss msg;
  memset(&msg,0,sizeof(msg));

  msg.msgVer = 0;
  msg.numTrkChHw = 0;
  msg.numTrkChUse = 32; // number of channels available
  msg.numConfigBlocks = 1;
  msg.gnssId = 6;     // GLONASS
  msg.resTrkCh = 8;   // min channels
  msg.maxTrkCh = 14;  // max channels
  msg.reserved = 0;
  msg.flags = 0x01010000 | (enabled & 0x01); // u-center sends additional 01 01 in last two high bytes, also low byte goes first

  return ubxSendCfgGnss(&msg);
}

int32_t ubxSetGalileoEnabled(uint8_t enabled)
{
  UbxCfgGnss msg;
  memset(&msg,0,sizeof(msg));

  msg.msgVer = 0;
  msg.numTrkChHw = 0;
  msg.numTrkChUse = 32; // number of channels available
  msg.numConfigBlocks = 1;
  msg.gnssId = 2;     // Galileo
  msg.resTrkCh = 4;   // min channels
  msg.maxTrkCh = 8;   // max channels
  msg.reserved = 0;
  msg.flags = 0x01010000 | (enabled & 0x01); // u-center sends additional 01 01 in last two high bytes, also low byte goes first

  return ubxSendCfgGnss(&msg);
}
