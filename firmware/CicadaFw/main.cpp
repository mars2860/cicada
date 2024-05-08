#include "pdl.h"
#include "main.h"

#include <ESP8266WiFi.h>
#include <Wire.h>
#include <FS.h>

//-----------------------------------------------------------------------------
// INSTALL
// 1. https://github.com/enjoyneering/ESP8266-I2C-Driver

//---------------------------------------------------------------
// VARIABLES

pdlDroneState droneState;

uint32_t pdlMicros()
{
  return micros();
}

//--------------------------------------------------------------
// INTERRUPTS

//--------------------------------------------------------------

#define FW_VERSION "2"

void setup()
{
  pdlResetLog();

  LOG_INFO("System started, fw v%s", FW_VERSION);

  Wire.begin();
  Wire.setClock(400000UL);  // In reality I observe 200 kHz by oscilloscope

  Serial.begin(115200);

  scanI2C();

  pdlSetup(&droneState);

  LOG_INFO("System initialized. DroneState = %i bytes",sizeof(droneState));
}

void scanI2C()
{
  byte error, address;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if(error == 0)
    {
      LOG_INFO("I2Cdev=0x%x",address);
    }
    else if(error==4)
    {
      LOG_ERROR("I2C error at address 0x%x",address);
    }
  }
}

void pdlSaveDefaultCfg(pdlDroneState *ds)
{
  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
    return;
  }

  File file = SPIFFS.open("default.cfg","w+");
  if(!file)
  {
    LOG_ERROR("Can't open %s","default.cfg");
  }
  else
  {
    size_t written = file.write((unsigned char*)ds,sizeof(pdlDroneState));
    file.flush();

    if(written != sizeof(pdlDroneState))
    {
      LOG_ERROR("Can't save default.cfg");
    }
    else
    {
      LOG_INFO("New default.cfg has been saved");
    }
  }
  file.close();

  SPIFFS.end();
}

void pdlLoadDefaultCfg(pdlDroneState *ds)
{
  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
    return;
  }

  File file = SPIFFS.open("default.cfg","r");
  if(!file)
  {
    LOG_ERROR("Can't open %s","default.cfg");
  }
  else
  {
    size_t read = file.read((unsigned char*)ds,sizeof(pdlDroneState));

    if(read != sizeof(pdlDroneState))
    {
      LOG_ERROR("Can't load default.cfg");
    }
    else
    {
      LOG_INFO("default.cfg has been loaded");
    }

    if(ds->version != PDL_VERSION)
    {
      LOG_ERROR("invalid firmware version of default.cfg");
    }
  }
  file.close();

  SPIFFS.end();
}

void loop()
{
  pdlUpdate(&droneState);
  pdlPreventFlyAway(  &droneState,
                      -85,  // min RSSI
                      3.f,  // safe alt
                      0.4f); // safe veloZ
}
