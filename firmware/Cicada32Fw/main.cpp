#include "pdl.h"
#include "main.h"

#include <Wire.h>
#ifdef ARDUINO_ARCH_ESP8266
#include <FS.h>
#elif ARDUINO_ARCH_ESP32
#include <SPIFFS.h>
#endif


//---------------------------------------------------------------
// VARIABLES

pdlDroneState droneState;

// for all esp32 based boards
#ifdef ARDUINO_ARCH_ESP32

// How to setup debuger in Sloeber for ESP32 https://evertdekker.com/?p=1191
// How to setup debuger in Eclipse for ESP32 https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/using-debugger.html

// This sets Arduino Stack Size - comment this line to use default 8K stack size
// SET_LOOP_TASK_STACK_SIZE(16 * 1024);  // 16KB

// change settings on Arduino
// https://github.com/espressif/arduino-esp32/issues/4529
// https://esp32.com/viewtopic.php?t=18432

// TASKS
// ARDUINO - CORE1
// EVENTS - CORE0
// ESC - CORE1
// CAMERA - CORE1

SemaphoreHandle_t semLog = NULL;
SemaphoreHandle_t semTime = NULL;

void pdlLockLog()
{
  xSemaphoreTake(semLog,portMAX_DELAY);
}

void pdlUnlockLog()
{
  xSemaphoreGive(semLog);
}

void pdlLockTime()
{
  xSemaphoreTake(semTime,portMAX_DELAY);
}

void pdlUnlockTime()
{
  xSemaphoreGive(semTime);
}

#endif

#ifdef ARDUINO_XIAO_ESP32S3
static uint32_t ledTimestamp = 0;
static uint8_t ledState = 0;
#endif

uint32_t pdlMicros()
{
  return micros();
}

#ifdef SERIAL_DEBUG_ENABLED
void printLog()
{
  if(pdlGetLogSize() > 0)
  {
    Serial.write(pdlGetLog(),pdlGetLogSize());
    Serial.flush();
    pdlResetLog();
  }
}
#endif

//--------------------------------------------------------------
// INTERRUPTS

//--------------------------------------------------------------

/** Firmware version is not the same as PDL version.
 *  You can add for example supporting new sensor.
 *  This will be new firmware version and
 *  old desktop and mobile software will work with this version.
 *  But if you add new command or changed pdlDroneState
 *  this will be new PDL version. In this case old desktop and
 *  mobile software will not work with new firmware
 */
#define FW_VERSION "4"

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
    //else if(error==4) // this floods the log when no I2C devices
    //{
    //  LOG_ERROR("I2C error at address 0x%x",address);
    //}
  }
}

void setup()
{
#ifdef ARDUINO_XIAO_ESP32S3
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
#endif

  Serial.begin(115200);

#ifdef SERIAL_DEBUG_ENABLED
  Serial.setDebugOutput(true);
  delay(5000);
#endif

#ifdef ARDUINO_ARCH_ESP32
  semLog =  xSemaphoreCreateMutex();
  semTime = xSemaphoreCreateMutex();
#endif

  pdlResetLog();

  LOG_INFO("System started, fw v%s", FW_VERSION);

#ifdef ARDUINO_ARCH_ESP8266
  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
    SPIFFS.format();
    LOG_INFO("Format SPIFFS");
  }
  else
  {
    SPIFFS.end();
  }
#elif ARDUINO_ARCH_ESP32
  // Format SPIFFS for first run
  if(SPIFFS.begin(true) == false)
  {
    LOG_ERROR("Can't format SPIFFS");
  }
  else
  {
    SPIFFS.end();
  }
#endif

  Wire.begin();
  Wire.setClock(400000UL);  // In reality I observe 200 kHz by oscilloscope for ESP8266 and 363kHZ for ESP32S3

#ifdef SERIAL_DEBUG_ENABLED
  printLog();
#endif

  scanI2C();

#ifdef SERIAL_DEBUG_ENABLED
  printLog();
#endif

  pdlSetup(&droneState);

  LOG_INFO("System initialized. DroneState = %i bytes",sizeof(droneState));
}

void pdlSaveDefaultCfg(pdlDroneState *ds)
{
  if(SPIFFS.begin() == false)
  {
    LOG_ERROR("Can't mount SPIFFS");
    return;
  }

  File file = SPIFFS.open("/default.cfg","w+");
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

  File file = SPIFFS.open("/default.cfg","r");
  if(!file)
  {
    LOG_ERROR("Can't open %s","default.cfg");
  }
  else
  {
    pdlDroneState cfg;
    size_t read = file.read((unsigned char*)&cfg,sizeof(pdlDroneState));

    if(read != sizeof(pdlDroneState))
    {
      LOG_ERROR("Can't load default.cfg");
    }
    else
    {
      memcpy(ds,&cfg,sizeof(pdlDroneState));
      LOG_INFO("default.cfg has been loaded");

      if(ds->version != PDL_VERSION)
      {
        LOG_ERROR("invalid firmware version of default.cfg");
      }
    }
  }
  file.close();

  SPIFFS.end();
}

uint8_t oldHostConnected = 0;

void loop()
{
  //-----------------------------------------
  #ifdef SERIAL_DEBUG_ENABLED
    printLog();
  #endif
  //-----------------------------------------

#ifdef ARDUINO_XIAO_ESP32S3
  if(pdlGetDeltaTime(pdlMicros(),ledTimestamp) >= 1000000)
  {
    ledTimestamp = pdlMicros();
    ledState = !ledState;
    if(ledState)
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
#endif

  pdlUpdate(&droneState);

  if(!pdlIsHostConnected(&droneState))
  {
    pdlSafeStop(&droneState, 3.f);

    if(oldHostConnected)
    {
      LOG_INFO("Host is disconnected. Try to safe stop");
    }
  }

  oldHostConnected = pdlIsHostConnected(&droneState);
}
