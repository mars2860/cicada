#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Math.h>

#include "MAX1704X.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "QMC5883L.h"
#include "Adafruit_BMP280.h"
#include "altitude_kf.h"
#include "altitude.h"

#include "RTFusion.h"
#include "RTFusionRTQF.h"
#include "RTFusionKalman4.h"
#include "motor.h"
#include "pid.h"

//-----------------------------------------------------------------------------
// INSTALL
// 1. https://github.com/enjoyneering/ESP8266-I2C-Driver

//---------------------------------------------------------------
// SETTINGS

#define STASSID                         "liftelectronica"
#define STAPSK                          "cosmos327"
#define DEFAULT_IP_ADDRESS              "192.168.1.33"
#define DEFAULT_GATEWAY_ADDRESS         "192.168.1.1"
#define DEFAULT_SUBNET                  "255.255.255.0"

#define DEFAULT_CMD_PORT                4210

#define DEFAULT_TELEMETRY_PORT          4211
#define DEFAULT_TELEMETRY_PERIOD        100
#define TELEMETRY_MAX_PACKET_SIZE       255

#define DEFAULT_PID_PERIOD              10

#define DEFAULT_SEA_LEVEL               1013.25

// good video about pwm freq https://www.youtube.com/watch?v=paXW_8Fn69Y
// and thread https://community.micro-motor-warehouse.com/t/brushed-pwm-rate-and-hidden-thrust/3182/10
// PWM freq = 500Hz, max duty = 2000
#define MOTOR_MAX_GAS                   2000UL
#define MOTOR_COUNT                     4
#define DEFAULT_MOTORS_ENABLED          0
#define DEFAULT_STABILIZATION_ENABLED   0

// milliseconds between barometer reading
#define BARO_READ_PERIOD  50

#define MPU_1G 8192.f

//#define USE_MPU6050_DMP
#define ESCAPER_TEENYPRO

//----------------------------------------------------------------
// HARDWARE

#define ESP8266_LED     2

#ifdef ESCAPER_TEENYPRO
  #define M1              16
  #define M2              0
  #define M3              15
  #define M4              2

  #include "dshot.h"
#elif ESCAPER_PWM
  #define M1              15
  #define M2              2
  #define M3              0
  #define M4              16

  extern "C"
  {
    #include "pwm.h"
    #include "user_interface.h"
  }
#endif

//---------------------------------------------------------------
// COMMANDS

#define CMD_SWITCH_MOTORS         100
#define CMD_SET_MOTORS_GAS        101
#define CMD_SET_ACCEL_OFFSET      102
#define CMD_SET_GYRO_OFFSET       103
#define CMD_SET_MAGNET_CALIB      104
#define CMD_SELF_CALIB_ACCEL      105
#define CMD_SELF_CALIB_GYRO       106
#define CMD_SET_YAW_PID           107
#define CMD_SET_PITCH_PID         108
#define CMD_SET_ROLL_PID          109
#define CMD_SET_ALT_PID           110
#define CMD_SET_YPR               111
#define CMD_SET_PERIODS           112
#define CMD_ENABLE_STABILIZATION  113
#define CMD_RESET_ALTITUDE        114
#define CMD_SET_SEA_LEVEL         115
#define CMD_SET_ALTITUDE          116

//---------------------------------------------------------------
// VARIABLES

IPAddress ip;
IPAddress gateway;
IPAddress subnet;
IPAddress host;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiUDP udp;
uint8_t udpPacket[TELEMETRY_MAX_PACKET_SIZE];

uint32_t cmdPort = DEFAULT_CMD_PORT;
uint32_t telemetryPort = DEFAULT_TELEMETRY_PORT;

uint32_t telemetryTimer;
uint32_t telemetryPeriod = DEFAULT_TELEMETRY_PERIOD;
uint32_t pidPeriod = DEFAULT_PID_PERIOD;

uint32_t loopTime;      ///< time of one main loop (in micros)
uint32_t loopTimer;     ///< timer to measure main loop time (in micros)

uint32_t mpuTime;       ///< time beetween mpu6050 readings
uint32_t mpuTimer;

uint32_t baroTimer;     ///< timer to read barometer

uint8_t motorsEnabled = DEFAULT_MOTORS_ENABLED;
Motor *motor;
int32_t Motor::maxGas = MOTOR_MAX_GAS;

MAX1704X FuelGauge(0.00125f);
QMC5883L mag;
Adafruit_BMP280 baro;
MPU6050 mpu;
#ifndef USE_MPU6050_DMP
RTFusion* imuFusion;
// micros
#define MPU6050_READ_PERIOD 5000
#else
Quaternion q;               // [w, x, y, z] quaternion container
VectorInt16 aa;             // [x, y, z] accel sensor measurements
VectorInt16 aaReal;         // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld;        // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity;        // [x, y, z] gravity vector
#endif

float batVoltage;
float batPercent;
int16_t magnet[3];          // Magnetometer calibrated data
int16_t magnetOffset[3] = {0,0,0};    // Magnetometer hard-iron calibration offset
float magnetScale[3] = {1.f,1.f,1.f}; // Magnetometer soft-iron calibration scales
float heading;              // Heading to magnetic north
int16_t accel[3];           // Accel raw data
int16_t accelOffset[3];     // Accel calibration offsets
int16_t gyro[3];            // Gyro raw data
int16_t gyroOffset[3];      // Gyro calibration offset
float ypr[3];               // Yaw, Pitch, Roll (radians)
uint8_t enStabilization = DEFAULT_STABILIZATION_ENABLED;  // Stabilization is enabled
int32_t baseGas = 0;        // Base gas level for motor (doesn't affect if alt pid is on)
float temperature;
float pressure;
float seaLevelhPa = DEFAULT_SEA_LEVEL;
float altitude;
float vz = 0;
//float fusedYaw;

//Altitude_KF alt_estimator(0.1f, 0.1f);
AltitudeEstimator altest(0.0005,  // sigma Accel
                         0.0005,  // sigma Gyro
                         0.018,   // sigma Baro
                         0.5,   // ca
                         0.1);  // accelThreshold
Pid yawPid, pitchPid, rollPid, altPid;

//--------------------------------------------------------------
// FUNCTIONS

/// switch GPIO to HIGH state
//static inline ICACHE_RAM_ATTR void setPin(uint8_t pin);
/// switch GPIO to LOW state
//static inline ICACHE_RAM_ATTR void clearPin(uint8_t pin);

/// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);
/// Update stored MPU6050 calibration
void readMpuOffsets();
/// Reads magnetometer and applies calibration to raw data
uint8_t readMagnet(int16_t *mx, int16_t *my, int16_t *mz);
/// Calcs heading to magnetic north
float calcHeading(int16_t mx, int16_t my, int16_t mz, double pitch, double roll);
/**
 *
 * @param a     coef a
 * @param b     coef b
 * @param dt    delta time in seconds
 * @param sx    x speed
 * @param prevX previous x
 * @param y     y
 * @return
 */
float complementaryFilter(float a, float b, float dt, float sx, float prevX, float y);

//void processEspLed();
void processCommand();
void processTelemetry();
uint8_t processSensors();
void processPids();

//--------------------------------------------------------------
// INTERRUPTS

//-----------------------------------------------------------------------------
// SETUP

void setupGpio()
{
  // GPIO config
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  digitalWrite(M1, LOW);  // digitalWrite needs 175 cpu cycles to execute!
  digitalWrite(M2, LOW);
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);
}

void setupWifi()
{
  ip.fromString(DEFAULT_IP_ADDRESS);
  gateway.fromString(DEFAULT_GATEWAY_ADDRESS);
  subnet.fromString(DEFAULT_SUBNET);

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  WiFi.config(ip, gateway, subnet);

  while(WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
}

void setupFirmwareUpdater()
{
  httpUpdater.setup(&httpServer);
  httpServer.begin();
}

void setupMotors()
{
  motor = new Motor[MOTOR_COUNT];

  motor[0].chl = 2;
  motor[0].pin = M1;

  motor[1].chl = 1;
  motor[1].pin = M2;

  motor[2].chl = 0;
  motor[2].pin = M3;

  motor[3].chl = 3;
  motor[3].pin = M4;

#ifdef ESCAPER_PWM
  ///< PWM setup (choice all pins that you use PWM)
  uint32 io_info[MOTOR_COUNT][3] =
  {
      // MUX, FUNC, PIN
      //{PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5, 5},    // D1
      //{PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4, 4},    // D2
      {PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0, 0},      // D3
      {PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2, 2},      // D4
      //{PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14, 14},   // D5
      //{PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12, 12},   // D6
      //{PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13, 13},   // D7
      {PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15, 15},     // D8
      {0, 0, 16},                                   // D0
  };

  pwm_init(MOTOR_MAX_GAS, 0, MOTOR_COUNT, io_info);
#elif defined ESCAPER_TEENYPRO
  dshotSetup( motor[0].pin,
              motor[1].pin,
              motor[2].pin,
              motor[3].pin,
              1000);
/*
  dshotEnable(0);

  dshotSet(9,9,9,9);

  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);

  dshotSet(12,12,12,12);

  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);
  dshotWrite300();
  delayMicroseconds(100);

  dshotSet(48,48,48,48);

  delay(50);

  dshotEnable(1);
 */
#endif
}

void setupSensors()
{
  Wire.begin();
  Wire.setClock(200000UL);  // line capacity is too high, can't run at higher frequency
  // Setup IMU
  mpu.initialize();

 #ifdef USE_MPU6050_DMP
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
 #else
  mpu.setRate(4);   // 200 Hz
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  //imuFusion = new RTFusionRTQF();
  imuFusion = new RTFusionKalman4();
  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  imuFusion->setSlerpPower(0.02f);
  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  imuFusion->setGyroEnable(true);
  imuFusion->setAccelEnable(true);
  imuFusion->setCompassEnable(false);
#endif

  // Setup Magnetometer
  mag.init();
  mag.setSamplingRate(200);
  mag.setOversampling(512);

  readMpuOffsets();
  // Setup barometer
  /* Default settings from datasheet. */
  baro.init(BMP280_ADDRESS_ALT);
  baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  baro.readCoefficients();

  FuelGauge.quickstart();
}

void setup()
{
  setupGpio();
  Serial.begin(115200);
  setupWifi();
  setupFirmwareUpdater();
  setupMotors();
  setupSensors();

  udp.begin(cmdPort);
}

//-----------------------------------------------------------------------------

void loop()
{
  loopTime = micros() - loopTimer;
  loopTimer = micros();

  if(processSensors())   // Max 5 millis to process sensors
    processPids();

  //Serial.print("pwmNext = "); Serial.println(pwmNext);
  //Serial.print("PWM ISR time = "); Serial.println(escIsrTimer);
  //Serial.print("PWM calc next period = "); Serial.println(tCalcNextPeriod);
  //Serial.print("PWM escUpdate = "); Serial.println(tEscUpdate);
  //Serial.print("DigWrite = "); Serial.println(tDigWrite);

  if(!motorsEnabled)
  {
    baseGas = 0;
    motor[0].setGas(0);
    motor[1].setGas(0);
    motor[2].setGas(0);
    motor[3].setGas(0);
  }

#ifdef ESCAPER_PWM
  bool pwm_up = false;
  uint32_t pwm_duty;

  for(uint8_t i = 0; i < 4; i++)
  {
    if(pwm_get_duty(motor[i].chl) != (uint32_t)motor[i].gas)
    {
      pwm_duty = 0;
      if(motor[i].gas > 0)
        pwm_duty = (uint32_t)motor[i].gas;
      pwm_set_duty(pwm_duty, motor[i].chl);
      pwm_up = true;
    }
  }

  if(pwm_up)
    pwm_start();
#elif defined ESCAPER_TEENYPRO
  //if(!motorsEnabled)
  //{
  //  setDshotValues(0,0,0,0);
  //}
  //else
  {
    uint8_t escUp = 0;
    for(uint8_t i = 0; i < 4; i++)
    {
      escUp |= motor[i].modified;
      motor[i].modified = 0;
    }
    if(escUp)
      dshotSet(motor[0].gas + 48, motor[1].gas + 48, motor[2].gas + 48, motor[3].gas + 48);

    // reset saved 3d mode
    //dshotSet(12,12,12,12);
    //else
    //  dshotSet(9,9,9,9);
  }
#endif

  httpServer.handleClient();
  processCommand();
  processTelemetry();
}

/*
static inline ICACHE_RAM_ATTR void setPin(uint8_t pin)
{
  if(pin < 16)
    GPOS = 1 << pin;
  else if(pin == 16)
     GP16O |= 1;
}

static inline ICACHE_RAM_ATTR void clearPin(uint8_t pin)
{
  if(pin < 16)
    GPOC = 1 << pin;
  else if(pin == 16)
     GP16O &= ~1;
}
*/

uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize)
{
  if(pos + valueSize >= TELEMETRY_MAX_PACKET_SIZE)
    return pos;

  memcpy(&packet[pos], value, valueSize);
  pos += valueSize;
  return pos;
}

uint8_t readMagnet(int16_t *mx, int16_t *my, int16_t *mz)
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

  return result;
}

void processCommand()
{
  if(udp.parsePacket())
  {
    udp.read(udpPacket, sizeof(udpPacket));
    host = udp.remoteIP();
    uint8_t cmd = udpPacket[0];
    int16_t dx,dy,dz;
    int32_t t0,t1,t2,t3;
    float kp,ki,kd;
    uint8_t enabled;
    switch(cmd)
    {
      case CMD_SWITCH_MOTORS:
        motorsEnabled = udpPacket[1];
        if(motorsEnabled)
        {
          yawPid.setTarget(ypr[0]);
          pitchPid.setTarget(0);
          rollPid.setTarget(0);
        }
        baseGas = 0;
        motor[0].setGas(0);
        motor[1].setGas(0);
        motor[2].setGas(0);
        motor[3].setGas(0);
        break;
      case CMD_SET_MOTORS_GAS:
        memcpy(&t0, &udpPacket[1], sizeof(int32_t));
        memcpy(&t1, &udpPacket[5], sizeof(int32_t));
        memcpy(&t2, &udpPacket[9], sizeof(int32_t));
        memcpy(&t3, &udpPacket[13], sizeof(int32_t));
        baseGas = (t0 + t1 + t2 + t3)/4;
        if(!enStabilization)
        {
          motor[0].setGas(t0);
          motor[1].setGas(t1);
          motor[2].setGas(t2);
          motor[3].setGas(t3);
        }
        /*else
        {
          int32_t dx = (motor[0].gas + motor[1].gas + motor[2].gas + motor[3].gas)/4;
          motor[0].addGas(baseGas - dx);
          motor[1].addGas(baseGas - dx);
          motor[2].addGas(baseGas - dx);
          motor[3].addGas(baseGas - dx);
        }*/
        break;
      case CMD_SET_ACCEL_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));
        //mpu.setDMPEnabled(false);
        mpu.setXAccelOffset(dx);
        mpu.setYAccelOffset(dy);
        mpu.setZAccelOffset(dz);
        //mpu.setDMPEnabled(true);
        readMpuOffsets();
        break;
      case CMD_SET_GYRO_OFFSET:
        memcpy(&dx, &udpPacket[1], sizeof(int16_t));
        memcpy(&dy, &udpPacket[3], sizeof(int16_t));
        memcpy(&dz, &udpPacket[5], sizeof(int16_t));
        //mpu.setDMPEnabled(false);
        mpu.setXGyroOffset(dx);
        mpu.setYGyroOffset(dy);
        mpu.setZGyroOffset(dz);
        //mpu.setDMPEnabled(true);
        readMpuOffsets();
        break;
      case CMD_SET_MAGNET_CALIB:
        memcpy(&magnetOffset[0], &udpPacket[1], 6);
        memcpy(&magnetScale[0], &udpPacket[7], 12);
        break;
      case CMD_SELF_CALIB_ACCEL:
        mpu.CalibrateAccel(10);
        readMpuOffsets();
        break;
      case CMD_SELF_CALIB_GYRO:
        mpu.CalibrateGyro(10);
        readMpuOffsets();
        break;
      case CMD_SET_YAW_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        yawPid.enabled = enabled;
        yawPid.setGains(kp,ki,kd);
        break;
      case CMD_SET_PITCH_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        pitchPid.enabled = enabled;
        pitchPid.setGains(kp,ki,kd);
        break;
      case CMD_SET_ROLL_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        rollPid.enabled = enabled;
        rollPid.setGains(kp,ki,kd);
        break;
      case CMD_SET_ALT_PID:
        enabled = udpPacket[1];
        memcpy(&kp, &udpPacket[2], sizeof(kp));
        memcpy(&ki, &udpPacket[2 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[2 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        altPid.enabled = enabled;
        altPid.setGains(kp,ki,kd);
        break;
      case CMD_SET_YPR:
        memcpy(&kp, &udpPacket[1], sizeof(kp));
        memcpy(&ki, &udpPacket[1 + sizeof(kp)], sizeof(ki));
        memcpy(&kd, &udpPacket[1 + sizeof(kp) + sizeof(ki)], sizeof(kd));
        yawPid.setTarget(kp);
        pitchPid.setTarget(ki);
        rollPid.setTarget(kd);
        break;
      case CMD_SET_PERIODS:
        memcpy(&telemetryPeriod, &udpPacket[1], sizeof(telemetryPeriod));
        memcpy(&pidPeriod, &udpPacket[1 + sizeof(telemetryPeriod)], sizeof(pidPeriod));
        if(telemetryPeriod < 1)
          telemetryPeriod = 1;
        if(pidPeriod < 1)
          pidPeriod = 1;
        yawPid.setTimeStep(pidPeriod);
        pitchPid.setTimeStep(pidPeriod);
        rollPid.setTimeStep(pidPeriod);
        altPid.setTimeStep(pidPeriod);
        break;
      case CMD_ENABLE_STABILIZATION:
        enStabilization = udpPacket[1];
        break;
      case CMD_RESET_ALTITUDE:
        vz = 0;
        seaLevelhPa = baro.pressure;
        break;
      case CMD_SET_SEA_LEVEL:
        memcpy(&seaLevelhPa, &udpPacket[1], sizeof(seaLevelhPa));
        break;
      case CMD_SET_ALTITUDE:
        memcpy(&altPid.target, &udpPacket[1], sizeof(altPid.target));
        break;
    }
  }
}

void processTelemetry()
{
  if(millis() - telemetryTimer < telemetryPeriod || !host.isSet())
    return;

  telemetryTimer = millis();

  uint16_t pos = 0;
  // Read battery state
  batVoltage = FuelGauge.voltage();
  batPercent = FuelGauge.percent();
  // Battery voltage
  pos = writeTelemetryPacket(pos, udpPacket, &batVoltage, sizeof(batVoltage));
  // Battery percent
  pos = writeTelemetryPacket(pos, udpPacket, &batPercent, sizeof(batPercent));
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
  pos = writeTelemetryPacket(pos, udpPacket, &ypr[0], sizeof(ypr[0]));
  pos = writeTelemetryPacket(pos, udpPacket, &ypr[1], sizeof(ypr[1]));
  pos = writeTelemetryPacket(pos, udpPacket, &ypr[2], sizeof(ypr[2]));
  // Heading
  pos = writeTelemetryPacket(pos, udpPacket, &heading, sizeof(heading));
  // Yaw PID
  pos = writeTelemetryPacket(pos, udpPacket, &yawPid.enabled, sizeof(yawPid.enabled));
  pos = writeTelemetryPacket(pos, udpPacket, &yawPid.kp, sizeof(yawPid.kp));
  pos = writeTelemetryPacket(pos, udpPacket, &yawPid.ki, sizeof(yawPid.ki));
  pos = writeTelemetryPacket(pos, udpPacket, &yawPid.kd, sizeof(yawPid.kd));
  pos = writeTelemetryPacket(pos, udpPacket, &yawPid.target, sizeof(yawPid.target));
  // Pitch PID
  pos = writeTelemetryPacket(pos, udpPacket, &pitchPid.enabled, sizeof(pitchPid.enabled));
  pos = writeTelemetryPacket(pos, udpPacket, &pitchPid.kp, sizeof(pitchPid.kp));
  pos = writeTelemetryPacket(pos, udpPacket, &pitchPid.ki, sizeof(pitchPid.ki));
  pos = writeTelemetryPacket(pos, udpPacket, &pitchPid.kd, sizeof(pitchPid.kd));
  pos = writeTelemetryPacket(pos, udpPacket, &pitchPid.target, sizeof(pitchPid.target));
  // Roll PID
  pos = writeTelemetryPacket(pos, udpPacket, &rollPid.enabled, sizeof(rollPid.enabled));
  pos = writeTelemetryPacket(pos, udpPacket, &rollPid.kp, sizeof(rollPid.kp));
  pos = writeTelemetryPacket(pos, udpPacket, &rollPid.ki, sizeof(rollPid.ki));
  pos = writeTelemetryPacket(pos, udpPacket, &rollPid.kd, sizeof(rollPid.kd));
  pos = writeTelemetryPacket(pos, udpPacket, &rollPid.target, sizeof(rollPid.target));
  // Alt PID
  pos = writeTelemetryPacket(pos, udpPacket, &altPid.enabled, sizeof(altPid.enabled));
  pos = writeTelemetryPacket(pos, udpPacket, &altPid.kp, sizeof(altPid.kp));
  pos = writeTelemetryPacket(pos, udpPacket, &altPid.ki, sizeof(altPid.ki));
  pos = writeTelemetryPacket(pos, udpPacket, &altPid.kd, sizeof(altPid.kd));
  pos = writeTelemetryPacket(pos, udpPacket, &altPid.target, sizeof(altPid.target));
  // Motors
  pos = writeTelemetryPacket(pos, udpPacket, &motorsEnabled, sizeof(motorsEnabled));
  pos = writeTelemetryPacket(pos, udpPacket, &motor[0].gas, sizeof(motor[0].gas));
  pos = writeTelemetryPacket(pos, udpPacket, &motor[1].gas, sizeof(motor[1].gas));
  pos = writeTelemetryPacket(pos, udpPacket, &motor[2].gas, sizeof(motor[2].gas));
  pos = writeTelemetryPacket(pos, udpPacket, &motor[3].gas, sizeof(motor[3].gas));
  // Periods
  pos = writeTelemetryPacket(pos, udpPacket, &telemetryPeriod, sizeof(telemetryPeriod));
  pos = writeTelemetryPacket(pos, udpPacket, &pidPeriod, sizeof(pidPeriod));
  // Stabilization
  pos = writeTelemetryPacket(pos, udpPacket, &enStabilization, sizeof(enStabilization));
  // PIDs output
  pos = writeTelemetryPacket(pos, udpPacket, &yawPid.output, sizeof(yawPid.output));
  pos = writeTelemetryPacket(pos, udpPacket, &pitchPid.output, sizeof(pitchPid.output));
  pos = writeTelemetryPacket(pos, udpPacket, &rollPid.output, sizeof(rollPid.output));
  pos = writeTelemetryPacket(pos, udpPacket, &altPid.output, sizeof(altPid.output));
  // Loop Time
  //pos = writeTelemetryPacket(pos, udpPacket, &loopTime, sizeof(loopTime));
  pos = writeTelemetryPacket(pos, udpPacket, &mpuTime, sizeof(mpuTime));
  // Baro
  pos = writeTelemetryPacket(pos, udpPacket, &temperature, sizeof(temperature));
  pos = writeTelemetryPacket(pos, udpPacket, &pressure, sizeof(pressure));
  pos = writeTelemetryPacket(pos, udpPacket, &altitude, sizeof(altitude));
  pos = writeTelemetryPacket(pos, udpPacket, &seaLevelhPa, sizeof(seaLevelhPa));

  udp.beginPacket(host, telemetryPort);
  udp.write(udpPacket, pos);
  udp.endPacket();

  // About 1 millis to sent telemetry
  //uint32_t tt = millis() - telemetryTimer;
  //Serial.print("telemetry time = ");
  //Serial.print(tt);
}

void processPids()
{
  if(!enStabilization | !motorsEnabled)
  {
    yawPid.stop();
    pitchPid.stop();
    rollPid.stop();
    altPid.stop();
    return;
  }

  float dg = 0;
  float mg[4] = {0,0,0,0};  // Motors gas
  uint8_t i;

  if(yawPid.enabled)
  {
    dg = yawPid.run(ypr[0],true);

    mg[1] += dg;
    mg[3] += dg;
    mg[0] -= dg;
    mg[2] -= dg;

    /*
    if(dg > 0)
    {
      mg[1] += dg;
      mg[3] += dg;
    }
    else
    {
      mg[0] -= dg;
      mg[2] -= dg;
    }
    */

    /*
    mg[1] += dg;
    mg[3] += dg;
    mg[0] -= dg;
    mg[2] -= dg;

    for(i = 0; i < 4; i++)
    {
      if(mg[i] < 0)
        mg[i] = 0;
    }
    */
    /*
    if(gyro[2] > 0)
    {
      if(gyro[2] > maxRotateSpeed && dg > 0)
        dg = -dg;
    }
    else
    {
      if(gyro[2] < -maxRotateSpeed && dg < 0)
        dg = -dg;
    }

    motor[1].addGas(int32_t(dg));
    motor[3].addGas(int32_t(dg));
    motor[0].addGas(int32_t(-dg));
    motor[2].addGas(int32_t(-dg));
    */
  }
  else
  {
    yawPid.stop();
  }

  if(pitchPid.enabled)
  {
    dg = pitchPid.run(ypr[1],true);

    //mg[1] += dg;
    //mg[2] += dg;

    /*if(dg > 0)
    {
      mg[1] += dg;
      mg[2] += dg;
    }
    else
    {
      mg[0] -= dg;
      mg[3] -= dg;
    }*/

    mg[1] += dg;
    mg[2] += dg;
    mg[0] -= dg;
    mg[3] -= dg;

    /*
    for(i = 0; i < 4; i++)
    {
      if(mg[i] < 0)
        mg[i] = 0;
    }
    */
    /*
    if(gyro[1] > 0)
    {
      if(gyro[1] > maxRotateSpeed && dg < 0)
        dg = -dg;
    }
    else
    {
      if(gyro[1] < -maxRotateSpeed && dg > 0)
        dg = -dg;
    }

    motor[1].addGas(int32_t(dg));
    motor[2].addGas(int32_t(dg));
    motor[0].addGas(int32_t(-dg));
    motor[3].addGas(int32_t(-dg));
    */
  }
  else
  {
    pitchPid.stop();
  }

  if(rollPid.enabled)
  {
    dg = rollPid.run(ypr[2],true);

    //mg[2] += dg;
    //mg[3] += dg;

    /*if(dg > 0)
    {
      mg[2] += dg;
      mg[3] += dg;
    }
    else
    {
      mg[0] -= dg;
      mg[1] -= dg;
    }*/

    mg[0] -= dg;
    mg[1] -= dg;
    mg[2] += dg;
    mg[3] += dg;

    /*
    for(i = 0; i < 4; i++)
    {
      if(mg[i] < 0)
        mg[i] = 0;
    }
    */
    /*
    if(gyro[0] > 0)
    {
      if(gyro[0] > maxRotateSpeed && dg > 0)
        dg = -dg;
    }
    else
    {
      if(gyro[0] < -maxRotateSpeed && dg < 0)
        dg = -dg;
    }

    motor[0].addGas(int32_t(-dg));
    motor[1].addGas(int32_t(-dg));
    motor[2].addGas(int32_t(dg));
    motor[3].addGas(int32_t(dg));
    */
  }
  else
  {
    rollPid.stop();
  }

  if(altPid.enabled)
  {
    dg = altPid.run(altitude);

    for(i = 0; i < 4; i++)
    {
      mg[i] += dg;
      //if(mg[i] < 0)
      //  mg[i] = 0;
      //motor[i].addGas(int32_t(dg));
    }
  }
  else
  {
    altPid.stop();

    for(i = 0; i < 4; i++)
      mg[i] += float(baseGas);
  }

  for(i = 0; i < 4; i++)
    motor[i].setGas(mg[i]);
}

float complementaryFilter(float a, float b, float dt, float sx, float prevX, float y)
{
  float result = a*(prevX + sx*dt) + b*y;
  return result;
}

uint8_t processSensors()
{
  uint8_t result = 0;

#ifdef USE_MPU6050_DMP
  uint8_t updateHeading;
  // this approach takes 10-12 ms to read sensors and update pids state
  // it is slow because we have slow I2C. Reading 42 bytes of fifo takes to much time
  //uint8_t mpuIntStatus = mpu.getIntStatus();        // holds actual interrupt status byte from MPU
  uint8_t fifoBuffer[64];                           // FIFO storage buffer
  uint16_t packetSize = mpu.dmpGetFIFOPacketSize();

#ifdef ESCAPER_TEENYPRO
  dshotEnable(0);
#endif
  //if((mpuIntStatus & MPU6050_INTERRUPT_DMP_INT_BIT) && mpu.dmpPacketAvailable())
  if(mpu.dmpPacketAvailable())
  {
    //uint32_t profDt = micros();
    result = 1;
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // reset fifo to get fresh data in next cycle
    mpu.resetFIFO();

#ifdef ESCAPER_TEENYPRO
    dshotEnable(1);
#endif

    // calc ypr (< 1 millis to execute)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(accel, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //profDt = micros() - profDt;
    //Serial.print("DMP processing = ");
    //Serial.println(profDt); // 2852 - 3210 is time to get YPR from DMP
    //mpu.dmpGetAccel(&aa, fifoBuffer);
    //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpuTime = micros() - mpuTimer;
    mpuTimer = micros();
    //float dt = float(mpuTime) / 1000000.f; // convert micros to seconds

    //fusedYaw = complementaryFilter(0.98f, 0.02f, dt, float(-gyro[2])/16.4f, fusedYaw, heading);
    //if(fusedYaw > 360.f)
    //  fusedYaw -= 360.f;
    //if(fusedYaw < 0)
    //  fusedYaw += 360.f;
    //ypr[0] = fusedYaw;

    //float az = (float(aaWorld.z) / MPU_1G)*9.80665f;

    // Derive altitude and get velocity in cm/s
    /*static float last_alt;
    float alt_baro_vel = (baro.altitude - last_alt) / dt;
    last_alt = baro.altitude;

    // Velocity = initial velocity + (acceleration * time)
    float vel_temp = vz + az*dt;
    // Calculate velocity using complimentary filter
    static float vel_coeff = 0.999f;
    vz = vel_coeff * vel_temp + (1 - vel_coeff) * alt_baro_vel;
    // Position = initial position + (initial velocity * time) + (1/2 * acceleration * (time^2))
    float alt_temp = altitude + (vel_temp*dt) + (0.5f *az*dt*dt);
    // Calculate altitude with complimentary filter
    static float alt_coeff = 0.998f;
    altitude = alt_coeff * alt_temp + (1 - alt_coeff) * baro.altitude;
     */
    /*float alt = altitude + vz*dt + az*dt*dt/2.f;
    vz += az*dt;
    altitude = 0.9f*alt + 0.1f*baro.altitude;*/

    /*
    float ac[3];
    float gy[3];

    for(uint8_t i = 0; i < 3; i++)
    {
      ac[i] = ((float)accel[i])/MPU_1G;
      gy[i] = ((float)gyro[i])*M_PI/2952.f;//(16.4f*180.f);
    }
    altest.estimate(ac,gy,baro.altitude,micros());
    altitude = altest.getAltitude();
    */
    //alt_estimator.propagate(az, dt);
  }

  /*if((mpuIntStatus & MPU6050_INTERRUPT_FIFO_OFLOW_BIT) || mpu.getFIFOCount() == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("MPU FIFO overflow!"));
  }*/

#ifdef ESCAPER_TEENYPRO
  dshotEnable(0);
#endif

  // read magnetometer (1 millis to execute)
  updateHeading = readMagnet(&magnet[0],&magnet[1],&magnet[2]);

#ifdef ESCAPER_TEENYPRO
  dshotEnable(1);
#endif

  // heading
  if(updateHeading)
    heading = calcHeading(magnet[0], magnet[1], magnet[2], ypr[1], ypr[2]);

#else

  mpuTime = micros() - mpuTimer;
  if(mpuTime >= MPU6050_READ_PERIOD) // 200 Hz
  {
    // disadvantage of this method is no temperature compensation for gyro and accel
    // and result is not so clean as in DMP, there are more fluctuations
    // values are invalid when motors run high speed. it is very sensetive to vibrations
    // advantage of this method is fast execute. it takes 5ms to execute

    result = 1;

    mpuTimer = micros();

#ifdef ESCAPER_TEENYPRO
    dshotEnable(0);
#endif
    readMagnet(&magnet[0],&magnet[1],&magnet[2]);
    //uint32_t profDt = micros();
    mpu.getMotion6(&accel[0],&accel[1],&accel[2],&gyro[0],&gyro[1],&gyro[2]);
#ifdef ESCAPER_TEENYPRO
    dshotEnable(1);
#endif

    unsigned long timestamp = micros();
    float gx = float(gyro[0])/16.4f;
    float gy = float(gyro[1])/16.4f;
    float gz = float(gyro[2])/16.4f;
    //float df = 1000000.f / float(mpuTime);
    //imu.begin(df);
    //imu.update(gx,gy,gz,accel[0],accel[1],accel[2],magnet[0],-magnet[1],-magnet[2]);
    //ypr[0] = imu.getYawRadians();
    //ypr[1] = -imu.getPitchRadians();
    //ypr[2] = imu.getRollRadians();
    // convert LSB to m/s^2 (no need convert because accel vector is normalized in computaion)
    float ax = float(accel[0]);///16384.f;
    float ay = float(accel[1]);///16384.f;
    float az = float(accel[2]);///16384.f;
    // convert LSB to mgauss/10 (no need convert because magnet vector is normalized in computaion)
    float mx = float(magnet[0]);///120.f;
    float my = float(magnet[1]);///120.f;
    float mz = float(magnet[2]);///120.f;
    // convert deg to rad
    gx *= RTMATH_DEGREE_TO_RAD;
    gy *= RTMATH_DEGREE_TO_RAD;
    gz *= RTMATH_DEGREE_TO_RAD;
    // sort out axes as in RTIMUMPU9250
    RTIMU_DATA fusionData;
    fusionData.fusionPoseValid = false;
    fusionData.fusionQPoseValid = false;
    fusionData.timestamp = timestamp;
    fusionData.gyroValid = true;
    fusionData.gyro = RTVector3(gx,-gy,-gz);
    fusionData.accelValid = true;
    fusionData.accel = RTVector3(-ax,ay,az);
    fusionData.compassValid = true;
    fusionData.compass = RTVector3(mx,my,-mz);
    fusionData.pressureValid = false;
    fusionData.temperatureValid = false;
    fusionData.humidityValid = false;
    imuFusion->newIMUData(fusionData, 0);
    const RTVector3 &pose = fusionData.fusionPose;
    heading = ypr[0] = pose.z() + M_PI;
    ypr[1] = pose.y();
    ypr[2] = pose.x();

    /*float aa[3];
    float gg[3];

    for(uint8_t i = 0; i < 3; i++)
    {
      aa[i] = ((float)accel[i])/16384.f;
      gg[i] = ((float)gyro[i])*M_PI/2952.f;//(16.4f*180.f);
    }
    altest.estimate(aa,gg,baro.altitude,micros());
    altitude = altest.getAltitude();*/

    //readMpuOffsets();
    //profDt = micros() - profDt;
    //Serial.print("Magwick processing = ");
    //Serial.println(profDt); // 1414 - 1822 is time to get YPR from Magwick (with reading MPU6050) Magwick very slow updates yaw angle and has big noise when motors run
                            // 2280 - 2341 is time to get YPR from RTIMUlib (with reading MPU6050)
  }
#endif

  // baro sensor
  if(millis() - baroTimer >= BARO_READ_PERIOD)
  {
    baroTimer = millis();

#ifdef ESCAPER_TEENYPRO
  dshotEnable(0);
#endif

  baro.update(seaLevelhPa);

#ifdef ESCAPER_TEENYPRO
  dshotEnable(1);
#endif
    //temperature = (float)(mpu.getTemperature()) / 340.f;  // require temperature offset
    pressure = baro.pressure;
    altitude = baro.altitude;
    temperature = baro.temperature;
    //altitude = altest.getAltitude();
    //alt_estimator.update(baro.altitude);
    //altitude = alt_estimator.h;
    //temperature = altest.getVerticalVelocity();//alt_estimator.v;//vz;////baro.temperature;
  }

  return result;
}

float calcHeading(int16_t mx, int16_t my, int16_t mz, double pitch, double roll)
{
  double fx = mx;
  double fy = my;
  double fz = mz;

  double cor = cos(roll);
  double sir = sin(roll);
  double cop = cos(pitch);
  double sip = sin(pitch);

  double fx1 = fx*cop + fy*sir*sip + fz*cor*sip;
  double fy1 = fy*cor - fz*sir;

  //int16_t result = (180.0*atan2(fy1,fx1)/M_PI);
  double result = M_PI - atan2(fy1,fx1);

  if(result >= 2.0*M_PI)
    result -= 2.0*M_PI;
  if(result < 0)
    result += 2.f*M_PI;

  return (float)result;
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
