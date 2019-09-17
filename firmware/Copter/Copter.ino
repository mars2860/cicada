#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Math.h>

#include "MAX1704X.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "QMC5883L.h"

// Install:
// 1. https://github.com/enjoyneering/ESP8266-I2C-Driver
// 2. https://github.com/porrey/MAX1704X
// 3. https://github.com/jrowberg/i2cdevlib
// 4. https://github.com/dthain/QMC5883L

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

//----------------------------------------------------------------
// HARDWARE

#define ESP8266_LED     2
#define HC595_DATA      0
#define HC595_LATCH     2
#define HC595_CLOCK     16
#define CAM_SD          15

//---------------------------------------------------------------
// SETTINGS

#ifndef STASSID
  #define STASSID "liftelectronica"
  #define STAPSK  "cosmos327"
#endif

IPAddress ip(192,168,1,33);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
IPAddress host;

unsigned int cmdPort = 4210;          // local port to receive commands
unsigned int telemetryPort = 4211;    // local port to send telemetry data

#define LED_FLASH_TIME 1000
#define TELEMETRY_UPDATE_TIME 250

#define MOTORS_PWM_FREQ           1024UL
#define MOTORS_PWM_RESOLUTION     256UL

//---------------------------------------------------------------
// COMMANDS

#define CMD_SWITCH_MOTORS         100
#define CMD_SET_MOTORS_GAS        101
#define CMD_SET_ACCEL_OFFSET      102
#define CMD_SET_GYRO_OFFSET       103
#define CMD_SET_MAGNET_CALIB      104
#define CMD_SELF_CALIB_ACCEL      105
#define CMD_SELF_CALIB_GYRO       106

//---------------------------------------------------------------
// VARIABLES

const char* ssid = STASSID;
const char* password = STAPSK;

#define TELEMETRY_MAX_PACKET_SIZE 255
WiFiUDP udp;
uint8_t udpPacket[TELEMETRY_MAX_PACKET_SIZE];             // buffer for udp packets

uint32_t ledTimer;

uint8_t escOutputs = 0;             // state of outputs of escaper, one bit per channel

uint8_t motorsEnabled = 0;

volatile uint32_t pwmStepTicks;              // ticks of timer to next pwm step
volatile uint32_t pwmPeriodTicks;            // ticks of timer per one pwm period
volatile uint32_t pwmNext = 0;               // ticks to next pwm timer interrupt
volatile uint32_t pwmTimer = 0;              // count ticks from pwm period start

volatile uint32_t telemetryTimer = 0;

MPU6050 mpu;      // 6-axis sensor

int16_t magnet[3];          // Magnetometer calibrated data
int16_t magnetOffset[3] = {0,0,0};    // Magnetometer hard-iron calibration offset
float magnetScale[3] = {1.f,1.f,1.f}; // Magnetometer soft-iron calibration scales
int16_t heading;            // Heading to magnetic north
int16_t accel[3];           // Accel raw data
int16_t accelOffset[3];     // Accel calibration offsets
int16_t gyro[3];            // Gyro raw data
int16_t gyroOffset[3];      // Gyro calibration offset
float ypr[3];               // Yaw, Pitch, Roll
Quaternion q;               // [w, x, y, z] quaternion container
VectorFloat gravity;        // [x, y, z] gravity vector

class Motor
{
  private:
    uint32_t gas;                     // count of pwm steps (use setMotorGas function)
  
  public:
    uint32_t gasTicks;                // count of ticks of pwm timer
    uint8_t setMask;                  // bit mask to update escOutput
    uint8_t clearMask;                // bit mask to update escOutput

  public:
    Motor()
    {
      gas = 0;
      gasTicks = 0;
      setMask = 0;
      clearMask = 0xFF;
    }
    
    void setNum(uint8_t num)
    {
      setMask = 1 << num;
      clearMask = ~setMask;
      
    }

    void setGas(uint32_t value)
    {
      if(value > MOTORS_PWM_RESOLUTION)
        value = MOTORS_PWM_RESOLUTION;

      gas = value;
      gasTicks = gas * pwmStepTicks;
    }
} motor[4];

class MAX17040: public MAX1704X
{
  public:
  MAX17040(): MAX1704X(1.25) {}
  uint16_t readRegister16(uint8_t registerId)
  {
    return MAX1704X::readRegister16(registerId);
  }
} FuelGauge;

class MyQMC5883L: public QMC5883L
{
  public:
  
  uint8_t readRaw(int16_t *x, int16_t *y, int16_t *z)
  {
    if(ready())
    {
      Wire.beginTransmission(0x0d);
      Wire.write(0);
      Wire.endTransmission();
  
      Wire.requestFrom(0x0d,6);
      int n = Wire.available();
      if(n != 6)
        return 0;

      *x = Wire.read() | (Wire.read()<<8);
      *y = Wire.read() | (Wire.read()<<8);
      *z = Wire.read() | (Wire.read()<<8);

      return 1;
    }

    return 0;
  }
} mag;

//--------------------------------------------------------------
// FUNCTIONS

// Updates escaper outputs. Each bit of state represents one channel state
void ICACHE_RAM_ATTR escaperUpdate(uint8_t state);
// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);
// Reads magnetometer and applies calibration to raw data
void readMagnet(int16_t *mx, int16_t *my, int16_t *mz);
// Calcs heading to magnetic north
int16_t calcHeading(int16_t mx, int16_t my);
// Update stored MPU6050 calibration
void readMpuOffsets();

//void processEspLed();
void processSensors();

//--------------------------------------------------------------
// INTERRUPTS

volatile uint32_t tmp32;
volatile uint8_t tmp8;

void ICACHE_RAM_ATTR onTimerISR()
{  
  pwmTimer += pwmNext;

   // New period
  if(pwmTimer >= pwmPeriodTicks)
    pwmTimer = 0;

  // Calc ticks to next interrupt
  pwmNext = pwmPeriodTicks - pwmTimer;
  for(tmp8 = 0; tmp8 < 4; tmp8++)
  {
    if(motor[tmp8].gasTicks > pwmTimer)
    {
      escOutputs |= motor[tmp8].setMask;
      tmp32 = motor[tmp8].gasTicks - pwmTimer;
      if(tmp32 < pwmNext && tmp32 > 0)
        pwmNext = tmp32;
    }
    else
    {
      escOutputs &= motor[tmp8].clearMask;
    }
  }

  // Update timer
  timer1_write(pwmNext);

  // Update outputs
  escaperUpdate(escOutputs);
}

void setup()
{
  // GPIO config
  pinMode(HC595_LATCH, OUTPUT);
  digitalWrite(HC595_LATCH, HIGH);  // Disable OE pin of HC595
  pinMode(HC595_DATA, OUTPUT);
  pinMode(HC595_CLOCK, OUTPUT);
  pinMode(CAM_SD, OUTPUT);

  // GPIO reset
  digitalWrite(HC595_DATA, LOW);
  digitalWrite(HC595_LATCH, HIGH);  // Disable OE pin of HC595
  digitalWrite(HC595_CLOCK, LOW);
  digitalWrite(CAM_SD, LOW);

  Serial.begin(115200);
 
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.config(ip, gateway, subnet);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    String type;
    if(ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]()
  {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      Serial.println("Auth Failed");
    }
    else if(error == OTA_BEGIN_ERROR)
    {
      Serial.println("Begin Failed");
    }
    else if(error == OTA_CONNECT_ERROR)
    {
      Serial.println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      Serial.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup motors
  for(uint8_t i = 0; i < 4; i++)
    motor[i].setNum(i);

  // Setup PWM
  pwmStepTicks = (10000000UL / (2*MOTORS_PWM_FREQ*MOTORS_PWM_RESOLUTION)); // (1 MHz / freq*res) / 0.2 us (timer step)
  
  if(pwmStepTicks == 0)
    pwmStepTicks = 1;

  pwmPeriodTicks = (10000000UL / (2*MOTORS_PWM_FREQ));   // (1 MHz / freq) / 0.2 us (timer step)

  if(pwmPeriodTicks == 0)
    pwmPeriodTicks = 1;
    
  timer1_isr_init();
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(pwmPeriodTicks);

  // Process timers

  ledTimer = millis();
  telemetryTimer = millis();
  
  /*Serial.println("pwmPeriodTicks = ");
  Serial.print(pwmPeriodTicks);
  Serial.print(", pwmStepTicks = ");
  Serial.println(pwmStepTicks);*/

  Wire.begin();
  Wire.setClock(200000UL);    // there are bad SCL fronts if set greater

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  
  mag.init();
  mag.setSamplingRate(200);
  mag.setOversampling(512);

  readMpuOffsets();

  udp.begin(cmdPort);
}

void loop()
{
  ArduinoOTA.handle();

  if(udp.parsePacket())
  {
    udp.read(udpPacket, sizeof(udpPacket));
    host = udp.remoteIP();
    uint8_t cmd = udpPacket[0];
    int16_t dx,dy,dz;
    switch(cmd)
    {
      case CMD_SWITCH_MOTORS:
        motorsEnabled = udpPacket[1];
        break;
      case CMD_SET_MOTORS_GAS:
        for(uint8_t i = 0; i < 4; i++)
          motor[i].setGas(udpPacket[1 + i]);
        break;
      case CMD_SET_ACCEL_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));
        mpu.setXAccelOffset(dx);
        mpu.setYAccelOffset(dy);
        mpu.setZAccelOffset(dz);
        readMpuOffsets();
        break;
      case CMD_SET_GYRO_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));
        mpu.setXGyroOffset(dx);
        mpu.setYGyroOffset(dy);
        mpu.setZGyroOffset(dz);
        readMpuOffsets();
        break;
      case CMD_SET_MAGNET_CALIB:
        memcpy(&magnetOffset[0], &udpPacket[1], 6);
        memcpy(&magnetScale[0], &udpPacket[7], 12);
        break;
      case CMD_SELF_CALIB_ACCEL:
        mpu.CalibrateAccel(6);
        readMpuOffsets();
        break;
      case CMD_SELF_CALIB_GYRO:
        mpu.CalibrateGyro(6);
        readMpuOffsets();
        break;
    }
  }

  if(millis() - telemetryTimer >= TELEMETRY_UPDATE_TIME && host.isSet())
  {
    telemetryTimer = millis();

    uint16_t pos = 0;
    // Read battery voltage
    uint16_t val16 = FuelGauge.adc();
    pos = writeTelemetryPacket(pos, udpPacket, &val16, sizeof(val16));
    // Read battery percent
    val16 = FuelGauge.readRegister16(REGISTER_SOC);
    pos = writeTelemetryPacket(pos, udpPacket, &val16, sizeof(val16));
    // Read Wifi Level
    int32_t val32 = WiFi.RSSI();
    pos = writeTelemetryPacket(pos, udpPacket, &val32, sizeof(val32));
    // IMU data
    pos = writeTelemetryPacket(pos, udpPacket, &accel[0], sizeof(accel[0]));
    pos = writeTelemetryPacket(pos, udpPacket, &accel[1], sizeof(accel[1]));
    pos = writeTelemetryPacket(pos, udpPacket, &accel[2], sizeof(accel[2]));
    pos = writeTelemetryPacket(pos, udpPacket, &gyro[0], sizeof(gyro[0]));
    pos = writeTelemetryPacket(pos, udpPacket, &gyro[1], sizeof(gyro[1]));
    pos = writeTelemetryPacket(pos, udpPacket, &gyro[2], sizeof(gyro[2]));
    // Magnetometer data
    pos = writeTelemetryPacket(pos, udpPacket, &magnet[0], sizeof(magnet[0]));
    pos = writeTelemetryPacket(pos, udpPacket, &magnet[1], sizeof(magnet[1]));
    pos = writeTelemetryPacket(pos, udpPacket, &magnet[2], sizeof(magnet[2]));
    // Get IMU calibration settings
    pos = writeTelemetryPacket(pos, udpPacket, &accelOffset[0], sizeof(accelOffset[0]));
    pos = writeTelemetryPacket(pos, udpPacket, &accelOffset[1], sizeof(accelOffset[1]));
    pos = writeTelemetryPacket(pos, udpPacket, &accelOffset[2], sizeof(accelOffset[2]));
    pos = writeTelemetryPacket(pos, udpPacket, &gyroOffset[0], sizeof(gyroOffset[0]));
    pos = writeTelemetryPacket(pos, udpPacket, &gyroOffset[1], sizeof(gyroOffset[1]));
    pos = writeTelemetryPacket(pos, udpPacket, &gyroOffset[2], sizeof(gyroOffset[2]));
    // Magnetometer calibration
    pos = writeTelemetryPacket(pos, udpPacket, &magnetOffset[0], sizeof(magnetOffset[0]));
    pos = writeTelemetryPacket(pos, udpPacket, &magnetOffset[1], sizeof(magnetOffset[1]));
    pos = writeTelemetryPacket(pos, udpPacket, &magnetOffset[2], sizeof(magnetOffset[2]));
    pos = writeTelemetryPacket(pos, udpPacket, &magnetScale[0], sizeof(magnetScale[0]));
    pos = writeTelemetryPacket(pos, udpPacket, &magnetScale[1], sizeof(magnetScale[1]));
    pos = writeTelemetryPacket(pos, udpPacket, &magnetScale[2], sizeof(magnetScale[2]));
    // Yaw Pitch Roll
    pos = writeTelemetryPacket(pos, udpPacket, &ypr[0], sizeof(float));
    pos = writeTelemetryPacket(pos, udpPacket, &ypr[1], sizeof(float));
    pos = writeTelemetryPacket(pos, udpPacket, &ypr[2], sizeof(float));
    // Heading
    pos = writeTelemetryPacket(pos, udpPacket, &heading, sizeof(heading));

    udp.beginPacket(host, telemetryPort);
    udp.write(udpPacket, pos);
    udp.endPacket();

    // About 1 millis to sent telemetry
    //uint32_t tt = millis() - telemetryTimer;
    //Serial.print("telemetry time = ");
    //Serial.print(tt);
  }

  processSensors();   // Max 5 millis to process sensors
  
  //Serial.print("pwmNext = "); Serial.println(pwmNext);

  //processEspLed(); // just for debug
}

void ICACHE_RAM_ATTR escaperUpdate(uint8_t state)
{
  for(uint8_t i = 8; i > 0; i--)
  {
    // Set data
    if(state & (1 << (i - 1)))
      digitalWrite(HC595_DATA, HIGH);
    else
      digitalWrite(HC595_DATA, LOW);
    // Clock out data
    digitalWrite(HC595_CLOCK, LOW);
    digitalWrite(HC595_CLOCK, HIGH);
  }

  if(motorsEnabled)
  {
    // Apply data to HC595 output
    digitalWrite(HC595_LATCH, HIGH);
    digitalWrite(HC595_LATCH, LOW);
  }
  else
  {
    digitalWrite(HC595_LATCH, HIGH);
  }
}

uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize)
{
  if(pos + valueSize >= TELEMETRY_MAX_PACKET_SIZE)
    return pos;

  memcpy(&packet[pos], value, valueSize);
  pos += valueSize;
  return pos;
}

void readMagnet(int16_t *mx, int16_t *my, int16_t *mz)
{
  uint8_t result;
  int16_t rx,ry,rz;
  
  result = mag.readRaw(&rx, &ry, &rz);
  
  if(result)
  {
    float x = rx - magnetOffset[0];
    float y = ry - magnetOffset[1];
    float z = rz - magnetOffset[2];

    x *= magnetScale[0];
    y *= magnetScale[1];
    z *= magnetScale[2];

    *mx = x;
    *my = y;
    *mz = z;
  }
}

void processSensors()
{
  uint8_t mpuIntStatus = mpu.getIntStatus();        // holds actual interrupt status byte from MPU
  uint8_t fifoBuffer[64];                           // FIFO storage buffer
  uint16_t packetSize = mpu.dmpGetFIFOPacketSize();

  if(mpu.dmpPacketAvailable())
  {
    // read a packet from FIFO (2-3 millis to execute)
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // calc ypr (< 1 millis to execute)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(accel, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // reset fifo to get fresh data in next cycle
    mpu.resetFIFO();
  }
  // read magnetometer (1 millis to execute)
  readMagnet(&magnet[0],&magnet[1],&magnet[2]);
  heading = calcHeading(-magnet[0],magnet[1]);
}

int16_t calcHeading(int16_t mx, int16_t my)
{
  double fx = mx;
  double fy = my;
  int16_t result = (180.0*atan2(fy,fx)/M_PI);
  if(result < 0 )
    result += 360;

  return result;
}

void readMpuOffsets()
{
    accelOffset[0] = mpu.getXAccelOffset();
    accelOffset[1] = mpu.getYAccelOffset();
    accelOffset[2] = mpu.getZAccelOffset();
    gyroOffset[0] = mpu.getXGyroOffset();
    gyroOffset[1] = mpu.getYGyroOffset();
    gyroOffset[2] = mpu.getZGyroOffset();  
}

/*
void processEspLed()
{
  if(espLedEnabled)
  {
    if(millis() - ledTimer >= LED_FLASH_TIME)
    {
      digitalWrite(ESP8266_LED, !digitalRead(ESP8266_LED));
      ledTimer = millis();
    }
  }
  else
  {
    digitalWrite(ESP8266_LED, LOW);
  }
}
*/
