#ifndef UBXGPS_H_INCLUDED
#define UBXGPS_H_INCLUDED

#include <stdint.h>

#define UBX_RX_SIZE      128

#define UBX_ERR_OK           1
#define UBX_ERR_ACK          1
#define UBX_ERR_EMPTY        0
#define UBX_ERR_PTR         -1
#define UBX_ERR_SIZE        -2
#define UBX_ERR_LOW_RX_BUF  -3
#define UBX_ERR_BAD_CHK     -4
#define UBX_ERR_NACK        -5


#define UBX_ACK_ACK         0x0105
#define UBX_ACK_NACK        0x0005
#define UBX_ACK_NACK_SIZE   2

struct UbxAckNack
{
  uint8_t clsId;
  uint8_t msgId;
};

#define UBX_NAV_PVT           0x0701
#define UBX_NAV_PVT_SIZE      92

struct UbxNavPvt
{
  // Type   Name           Unit   Description (Scaling)
  uint32_t  iTOW;       //  ms    GPS time of week of the navigation epoch. See the description of iTOW for
                        //        details.
  uint16_t  year;       //  y     Year (UTC)
  uint8_t   month;      //  month Month, range 1..12 (UTC)
  uint8_t   day;        //  d     Day of month, range 1..31 (UTC)
  uint8_t   hour;       //  h     Hour of day, range 0..23 (UTC)
  uint8_t   min;        //  min   Minute of hour, range 0..59 (UTC)
  uint8_t   sec;        //  s     Seconds of minute, range 0..60 (UTC)
  char      valid;      //        Validity Flags (see graphic below)
  uint32_t  tAcc;       //  ns    Time accuracy estimate (UTC)
  int32_t   nano;       //  ns    Fraction of second, range -1e9 .. 1e9 (UTC)
  uint8_t   fixType;    //        GNSSfix Type, range 0..5
                        //        0x00 = No Fix
                        //        0x01 = Dead Reckoning only
                        //        0x02 = 2D-Fix
                        //        0x03 = 3D-Fix
                        //        0x04 = GNSS + dead reckoning combined
                        //        0x05 = Time only fix
                        //        0x06..0xff: reserved
  char      flags;      //        Fix Status Flags (see graphic below)
  uint8_t   reserved1;  //        Reserved
  uint8_t   numSV;      //        Number of satellites used in Nav Solution
  int32_t   lon;        //  deg   Longitude (1e-7)
  int32_t   lat;        //  deg   Latitude (1e-7)
  int32_t   height;     //  mm    Height above Ellipsoid
  int32_t   hMSL;       //  mm    Height above mean sea level
  uint32_t  hAcc;       //  mm    Horizontal Accuracy Estimate
  uint32_t  vAcc;       //  mm    Vertical Accuracy Estimate
  int32_t   velN;       //  mm/s  NED north velocity
  int32_t   velE;       //  mm/s  NED east velocity
  int32_t   velD;       //  mm/s  NED down velocity
  int32_t   gSpeed;     //  mm/s  Ground Speed (2-D)
  int32_t   heading;    //  deg   Heading of motion 2-D (1e-5)
  uint32_t  sAcc;       //  mm/s  Speed Accuracy Estimate
  uint32_t  headingAcc; //  deg   Heading Accuracy Estimate (1e-5)
  uint16_t  pDOP;       //        Position DOP (0.01)
  int16_t   reserved2;  //        Reserved
  uint32_t  reserved3;  //        Reserved
  int32_t   headVeh;    //  deg Heading (1e-5) of vehicle (2-D)
  uint32_t  reserved4;  //        Reserved
};

#define UBX_CFG_NAV5        0x2406
#define UBX_CFG_NAV5_SIZE   36

struct UbxCfgNav5
{
  // Type     Name           Unit   Description (Scaling)
  uint16_t    mask;     //          mask - Parameters Bitmask. Only the masked parameters will be applied
  uint8_t     dynModel; //          Dynamic Platform model:
                        //            0 Portable
                        //            2 Stationary
                        //            3 Pedestrian
                        //            4 Automotive
                        //            5 Sea
                        //            6 Airborne with <1g Acceleration
                        //            7 Airborne with <2g Acceleration
                        //            8 Airborne with <4g Acceleration
  uint8_t     fixMode;  //          Position Fixing Mode.
                        //            1: 2D only
                        //            2: 3D only
                        //            3: Auto 2D/3D
  int32_t     fixedAlt; //     m    Fixed altitude (mean sea level) for 2D fix mode.
  uint32_t    fixedAltVar;//  m^2   Fixed altitude variance for 2D mode.
  char        minElev;  //    deg   Minimum Elevation for a GNSS satellite to be used in NAV
  uint8_t     drLimit;  //     s    Reserved
  uint16_t    pDop;     //          Position DOP Mask to use
  uint16_t    tDop;     //          Time DOP Mask to use
  uint16_t    pAcc;     //     m    Position Accuracy Mask
  uint16_t    tAcc;     //     m    Time Accuracy Mask
  uint8_t     staticHoldThr;//cm/s  Static hold threshold
  uint8_t     dgpsTimeOut;//   s    DGPS timeout.
  uint8_t     cnoThreshNumS;// Vs   Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted
  uint8_t     cnoThresh;//    dBHz  C/N0 threshold for deciding whether to attempt a fix
  uint16_t    reserved;//           Reserved
  uint16_t    staticHoldMax;//  m   Static hold distance threshold (before quitting static hold)
  uint8_t     utcStandard;//        UTC standard to be used
                          //          0 Not specified - Receiver may choose freely
                          //          3 UTC as operated by the U.S. Naval Observatory (USNO). Derived from GPS time.
                          //          6 UTC as operated by the former Soviet Union. Derived from GLONASS time.
                          //          7 UTC as operated by the National Time Service Center, China. Derived from BeiDou time. (not supported in protocol versions less than 16).
  uint8_t     reserved3;  //        Always set to zero
  uint32_t    reserved4;  //        Always set to zero
};

#define UBX_CFG_MSG         0x0106
#define UBX_CFG_MSG_SIZE    8

struct UbxCfgMsg
{
  uint8_t msgCls;
  uint8_t msgId;
  uint8_t i2cRate;
  uint8_t uart1Rate;
  uint8_t uart2Rate;
  uint8_t usbRate;
  uint8_t spiRate;
  uint8_t reserved;
};

#define UBX_CFG_RATE        0x0806
#define UBX_CFG_RATE_SIZE   6

struct UbxCfgRate
{
  uint16_t measRate;
  uint16_t navRate;
  uint16_t timeRef;
};

#define UBX_CFG_PRT        0x0006
#define UBX_CFG_PRT_SIZE   20

struct UbxCfgPrt
{
  uint8_t portId;
  uint8_t reserved;
  uint16_t txReady;
  uint32_t mode;
  uint32_t baudRate;
  uint16_t inProtoMask;
  uint16_t outProtoMask;
  uint16_t flags;
  uint16_t reserved5;
};

#define UBX_CFG_CFG         0x0906
#define UBX_CFG_CFG_SIZE    13

struct UbxCfgCfg
{
  uint32_t clearMask;
  uint32_t saveMask;
  uint32_t loadMask;
  uint8_t deviceMask;
};

#define UBX_CFG_RST         0x0406
#define UBX_CFG_RST_SIZE    4

struct UbxCfgRst
{
  uint16_t navBbrMask;
  uint8_t resetMode;
  uint8_t reserved1;
};

#define UBX_CFG_GNSS        0x3E06
#define UBX_CFG_GNSS_SIZE   12

struct UbxCfgGnss
{
  uint8_t msgVer;
  uint8_t numTrkChHw;
  uint8_t numTrkChUse;
  uint8_t numConfigBlocks;
  uint8_t gnssId;
  uint8_t resTrkCh;
  uint8_t maxTrkCh;
  uint8_t reserved;
  int32_t flags;
};

#define UBX_CFG_INF         0x0206
#define UBX_CFG_INF_SIZE    10

struct UbxCfgInf
{
  uint8_t protocolId;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t infMsgMask[6];
};

int32_t ubxRead();
int32_t ubxWrite(uint8_t c);
int32_t ubxAvailable();
int32_t ubxWaitTxComplete();

/// Calcs checksum for packet. @param buf whole packet with header and checksum
uint16_t ubxChkPacket(uint8_t *buf, uint32_t size);
/// Reads available bytes from stream. @return 1 if end of ubx is detected and checksum is correct
int32_t ubxReadPacket();
/// Sends UBX packet. @return 1 if success
int32_t ubxSendPacket(uint8_t *buf, uint32_t size);

/// Sends UBX-CFG-CFG
int32_t ubxSendCfgCfg(UbxCfgCfg *pMsg);
/// Sends UBX-CFG-MSG. Returns 1 if AckMsg is received
int32_t ubxSendCfgMsg(UbxCfgMsg *pMsg);
/// Sends UBX-CFG-INF. Returns 1 if AckMsg is received
int32_t ubxSendCfgInf(UbxCfgInf *pMsg);
/// Sends UBX-CFG-NAV5. Returns 1 if AckMsg is received
int32_t ubxSendCfgNav5(UbxCfgNav5 *pMsg);
/// Sends UBX-CFG-RATE. Return 1 if AckMsg is received
int32_t ubxSendCfgRate(UbxCfgRate *pMsg);
/// Sends UBX-CFG-PRT. Return 1 if AckMsg is received
int32_t ubxSendCfgPrt(UbxCfgPrt *pMsg);
/// Sends UBX-CFG-GNSS. Return 1 if AckMsg is received
int32_t ubxSendCfgGnss(UbxCfgGnss *pMsg);
/// Sends UBX-CFG-RST. Return 1
int32_t ubxSendCfgRst(UbxCfgRst *pMsg);

/// Reads ACK-NACK. @return 1 if ack-nack is received
int32_t ubxReadAckNack(UbxAckNack *pMsg);
/// Reads NAV-PVT. @return 1 if nav-pvt is received
int32_t ubxReadNavPvt(UbxNavPvt *pMsg);

int32_t ubxReset(uint8_t resetMode, uint16_t navBbrMask);
int32_t ubxRestoreDefaults();
int32_t ubxDisableNmea();
int32_t ubxEnableNavPvt();
int32_t ubxSetUartBaudrate(uint32_t baudrate);
int32_t ubxSetMeasRate(uint32_t measRate);
int32_t ubxSetPedestrianDynModel();
int32_t ubxSetAirborn2gDynModel();
int32_t ubxSetGlonassEnabled(uint8_t enabled);
int32_t ubxSetGalileoEnabled(uint8_t enabled);

// An array of possible baudrates that can be used by the GPS receiver, sorted descending to prevent excess Serial
// flush/begin after restoring defaults. You can uncomment values that can be used by your GPS receiver before the
// auto-configuration.
const uint32_t gpsPossibleBaudrates[] =
{
    // 921600,
    // 460800,
    // 230400,
    115200,
    // 57600,
    38400,
    // 19200,
    9600,
    // 4800,
};

#endif
