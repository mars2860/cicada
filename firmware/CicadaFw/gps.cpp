#include "pdl.h"
#include "UbxGps.h"
#include <Arduino.h>
#include "main.h"

#ifdef SERIAL_DEBUG_ENABLED

void pdlSetupGps(pdlDroneState* ds)
{
  (void)ds;
}

uint8_t pdlReadGps(pdlDroneState *ds)
{
  (void)ds;
  return 0;
}

#else

void clearSerialRx()
{
  while(Serial.available())
  {
    Serial.read();
  }
}

void pdlSetupGps(pdlDroneState* ds)
{
  int32_t res;
  uint32_t defBaudrate = 9600;
  clearSerialRx();

  // Restore the default GPS receiver configuration.
  for(uint32_t i = 0; i < sizeof(gpsPossibleBaudrates) / sizeof(*gpsPossibleBaudrates); i++)
  {
    Serial.begin(gpsPossibleBaudrates[i]);
    delay(30);
    res = ubxRestoreDefaults();
    if(res == UBX_ERR_OK)
    {
      LOG_INFO("UbxGps - Restored to default at %i",gpsPossibleBaudrates[i]);
      defBaudrate = gpsPossibleBaudrates[i];
      break;
    }
  }

  if(res != UBX_ERR_OK)
  {
    LOG_ERROR("UbxGps - Can't restore defaults,err=%i",res);
  }

  // Switch to the default baudrate.
  Serial.begin(defBaudrate);
  // wait no more 1s until GPS start to work
  uint32_t timeoutStart = pdlMicros();
  while(!Serial.available())
  {
    if(pdlGetDeltaTime(pdlMicros(),timeoutStart) > 1000000)
    {
      LOG_ERROR("GPS is not found");
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
      return;
    }
  }
  // Clear RX buffer
  clearSerialRx();
  // Disable NMEA messages by sending appropriate packets.
  res = ubxDisableNmea();
  if(res != UBX_ERR_OK)
  {
    res = ubxDisableNmea();
    if(res != UBX_ERR_OK)
    {
      LOG_ERROR("UbxGps - Can't disable NMEA,err=%i",res);
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
    }
  }

  // Clear RX buffer from rest of nmea messages
  clearSerialRx();

  // GPS,GLONASS,SBAS,QZSS are enabled by default

  //From Datasheet M8030 page 5, 3.2 Configuration

  //The combinations of systems, which can be configured simultaneously depends on the receivers capability to
  //receive several carrier frequencies. Please check the data sheet of your receiver. Usually GPS, SBAS (e.g. WAAS,
  //EGNOS, MSAS), QZSS L1 and Galileo can be enabled together, because they all use the 1575.42MHz L1
  //frequency. GLONASS and BeiDou both operate on different frequencies, therefore the receiver must be able to
  //receive a second or even third carrier frequency in order to process these systems together with GPS.
  // Disable GLONASS

  /*res = ubxSetGlonassEnabled(0);
  if(res != UBX_ERR_OK)
  {
    res = ubxSetGlonassEnabled(0);
    if(res != UBX_ERR_OK)
    {
      LOG_ERROR("UbxGps - Can't disable GLONASS,err=%i",res);
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
    }
  }

  // Enable Galileo
  res = ubxSetGalileoEnabled(1);
  if(res != UBX_ERR_OK)
  {
    res = ubxSetGalileoEnabled(1);
    if(res != UBX_ERR_OK)
    {
      LOG_ERROR("UbxGps - Can't enable Galileo,err=%i",res);
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
    }
  }*/

  // Airborn2g/4g in Pixhawk/ArduPilot
  // Enable DynModel - airborne with <2g acceleration
  res = ubxSetAirborn2gDynModel();
  if(res != UBX_ERR_OK)
  {
    // Don't enable pedestrian mode
    // in pedestrian mode velE,velN are very smooth but there is time delay about 1s between real speed and gps output
    //res = ubxSetPedestrianDynModel();
    res = ubxSetAirborn2gDynModel();
    if(res != UBX_ERR_OK)
    {
      LOG_ERROR("UbxGps - Can't set airborn2g mode,err=%i",res);
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
    }
  }

  // Change measurement frequency to 200 ms. (200ms in ArduPilot and 125ms in Pixhawk) (Max 100ms)
  res = ubxSetMeasRate(200);
  if(res != UBX_ERR_OK)
  {
    res = ubxSetMeasRate(200);
    if(res != UBX_ERR_OK)
    {
      LOG_ERROR("UbxGps - Can't set update rate 5Hz,err=%i",res);
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
    }
  }

  // Enable NAV-PVT messages.
  res = ubxEnableNavPvt();
  if(res != UBX_ERR_OK)
  {
    res = ubxEnableNavPvt();
    if(res != UBX_ERR_OK)
    {
      LOG_ERROR("UbxGps - Can't enable NAVPVT,err=%i",res);
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
    }
  }

  // Switch the GPS receiver serial configuration to the target baudrate.
  res = ubxSetUartBaudrate(115200);
  if(res != UBX_ERR_EMPTY)  // We don't receive answer if baudRate changed successfully
  {
    res = ubxSetUartBaudrate(115200);
    if(res != UBX_ERR_EMPTY)
    {
      LOG_ERROR("UbxGps - Can't set baudrate 115200,err=%i",res);
      pdlSetError(ds,ERR_GPS_INIT_FAILED);
    }
  }

  Serial.begin(115200);
}

uint8_t pdlReadGps(pdlDroneState *ds)
{
  UbxNavPvt msg;
  memset(&msg,0,sizeof(msg));

  while(ubxReadNavPvt(&msg))
  {
    /*
    ds->gps.fixType = msg.fixType;
    ds->gps.numSV = msg.numSV;
    ds->gps.curLat = (float)msg.lat / 10000000.0f;
    ds->gps.curLon = (float)msg.lon / 10000000.0f;
    ds->gps.curAlt = (float)msg.hMSL / 1000.0f;
    ds->gps.course = (float)msg.heading / 100000.0f;
    ds->gps.velN = (float)msg.velN / 1000.f;
    ds->gps.velE = (float)msg.velE / 1000.f;
    ds->gps.velU = -(float)msg.velD / 1000.f;
    ds->gps.hAcc = (float)msg.hAcc / 1000.f;
    ds->gps.vAcc = (float)msg.vAcc / 1000.f;
    ds->gps.sAcc = (float)msg.sAcc / 1000.f;
    */

    pdlNewGpsData( ds,
                   msg.fixType,
                   msg.numSV,
                   (float)msg.lat / 10000000.0f,
                   (float)msg.lon / 10000000.0f,
                   (float)msg.hMSL / 1000.0f,
                   (float)msg.heading / 100000.0f,
                   (float)msg.velN / 1000.f,
                   (float)msg.velE / 1000.f,
                   -(float)msg.velD / 1000.f,
                   (float)msg.hAcc / 1000.f,
                   (float)msg.vAcc / 1000.f,
                   (float)msg.sAcc / 1000.f);
  }

  //if(ds->gps.fixType == 3 && ds->gps.enabled)
  if( ds->gps.fixType >= 2 &&
      ds->gps.fixType <= 3 &&
      ds->gps.enabled)
  {
    pdlResetError(ds,ERR_GPS_DATA_INVALID);
    return 1;
  }

  pdlSetError(ds,ERR_GPS_DATA_INVALID);
  return 0;
}

#endif
