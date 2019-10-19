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
#include "PID_v1.h"
#include "Adafruit_BMP280.h"
//#include "MadgwickAHRS.h"
#include "RTFusionRTQF.h"
#include "altitude_kf.h"

extern "C"
{
  #include "pwm.h"
  #include "user_interface.h"
}

// Install:
// 1. https://github.com/enjoyneering/ESP8266-I2C-Driver
// 2. https://github.com/porrey/MAX1704X
// 3. https://github.com/jrowberg/i2cdevlib
// 4. https://github.com/dthain/QMC5883L
// 5. https://github.com/br3ttb/Arduino-PID-Library/
// 6. https://github.com/adafruit/Adafruit_BMP280_Library
// 7. https://github.com/rblilja/AltitudeKF
// 8. https://github.com/arduino-libraries/MadgwickAHRS
// 9. https://github.com/RTIMULib/RTIMULib-Arduino

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

//----------------------------------------------------------------
// HARDWARE

#define ESP8266_LED     2
#define M1              15
#define M2              2
#define M3              0
#define M4              16

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
#define DEFAULT_TELEMETRY_PERIOD  100
#define DEFAULT_PID_PERIOD        10
#define DEFAULT_SEA_LEVEL         1013.25

// good video about pwm freq https://www.youtube.com/watch?v=paXW_8Fn69Y
// and thread https://community.micro-motor-warehouse.com/t/brushed-pwm-rate-and-hidden-thrust/3182/10
//#define MOTORS_PWM_FREQ           1000UL
#define MOTORS_PWM_RESOLUTION     2000UL

#define TELEMETRY_MAX_PACKET_SIZE 255

#define MPU_1G 8192

// COMPILE SETTINGS

//#define USE_MPU6050_DMP

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

const char* ssid = STASSID;
const char* password = STAPSK;

WiFiUDP udp;
uint8_t udpPacket[TELEMETRY_MAX_PACKET_SIZE];             // buffer for udp packets

uint32_t telemetryTimer;
uint32_t telemetryPeriod = DEFAULT_TELEMETRY_PERIOD;
uint32_t pidPeriod = DEFAULT_PID_PERIOD;

uint32_t loopTime;      // time of one main loop (in micros)
uint32_t loopTimer;     // timer to measure main loop time (in micros)

uint32_t mpuTime;       // time beetween mpu6050 readings
uint32_t mpuTimer;

uint32_t ledTimer;

uint8_t motorsEnabled = 0;

//--------------------------------------------------------------------
// PWM

#define PWM_CHANNELS 4

// PWM setup (choice all pins that you use PWM)

uint32 io_info[PWM_CHANNELS][3] =
{
  // MUX, FUNC, PIN
//  {PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5,   5}, // D1
//  {PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4,   4}, // D2
  {PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0,   0}, // D3
  {PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2,   2}, // D4
//  {PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}, // D5
//  {PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12, 12}, // D6
//  {PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13}, // D7
  {PERIPHS_IO_MUX_MTDO_U,  FUNC_GPIO15 ,15}, // D8
  {0,0 ,16}, // D0
};

// PWM initial duty: all off

uint32 pwm_duty_init[PWM_CHANNELS];


/*
volatile uint32_t pwmStepTicks;                               // ticks of timer to next pwm step
volatile uint32_t pwmPeriodTicks;                             // ticks of timer per one pwm period
volatile uint32_t pwmNext = 0;                                // ticks to next pwm timer interrupt
volatile uint32_t pwmSetMask = 0;                             // bit mask to set GPIO0-15 and GPIO16 when pwm period starts
volatile uint32_t pwmClearMask[5] = {0,0,0,0,0};              // bit mask to clear GPIO0-15 when pwm timer interrupt is
volatile uint32_t pwmTicks[5] = {1000,1000,1000,1000,1000};   // sorted precalculated ticks to next pwm timer interrupt
volatile uint8_t pwmLoop = 0;                                 // index
volatile uint8_t pwmMaxLoop = 0;                              // max index

// Updates pwm variables
void pwmUpdate();
*/
//-------------------------------------------------------------------

MPU6050 mpu;      // 6-axis sensor

int16_t magnet[3];          // Magnetometer calibrated data
int16_t magnetOffset[3] = {0,0,0};    // Magnetometer hard-iron calibration offset
float magnetScale[3] = {1.f,1.f,1.f}; // Magnetometer soft-iron calibration scales
float heading;              // Heading to magnetic north
int16_t accel[3];           // Accel raw data
int16_t accelOffset[3];     // Accel calibration offsets
int16_t gyro[3];            // Gyro raw data
int16_t gyroOffset[3];      // Gyro calibration offset
float ypr[3];               // Yaw, Pitch, Roll (radians)
Quaternion q;               // [w, x, y, z] quaternion container
VectorInt16 aa;             // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;         // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;        // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;        // [x, y, z] gravity vector
uint8_t enStabilization = 0;  // Stabilization is enabled
int32_t baseGas = 0;        // Base gas level for motor (doesn't affect if alt pid is on)

Adafruit_BMP280 baro;
float temperature;
float pressure;
float seaLevelhPa = DEFAULT_SEA_LEVEL;
float altitude;

Altitude_KF alt_estimator(0.1f, 0.1f);

#ifndef USE_MPU6050_DMP
//Madgwick imu;               // 9DOF IMU processor
RTFusionRTQF rtFusion;      // Another one 9DOF IMU processor
#endif

class Motor
{
  public:
    static uint32_t pwmTicks[4];
    static uint8_t pwmLoop;
  public:
    int32_t gas;                     // count of pwm steps (use setGas function to change value)
    uint32_t gasTicks;               // count of ticks of pwm timer
    uint8_t pin;
  public:
    Motor(uint8_t pin)
    {
      gas = 0;
      gasTicks = 0;
      this->pin = pin;
    }

    void setGas(int32_t value)
    {
      int32_t limit = int32_t(MOTORS_PWM_RESOLUTION);
      if(value > limit)
        value = limit;
      if(value < -limit)
        value = -limit;

      gas = value;

      /*
      if(gas > 0)
        gasTicks = gas * pwmStepTicks;
      else
        gasTicks = 0;
      */
    }

    void addGas(int32_t dx)
    {
      int32_t value = gas;
      value += dx;

      setGas(value);
    }
} motor[4] = {(2),(1),(0),(3)};//{(M1),(M2),(M3),(M4)};

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

class MyPID: public PID
{
  protected:
    double apInput;
    double apSetpoint;
    double apOutput;

    /*void run()
    {
      return AutoPID::run();
    }*/
    float Un1;
    float En;
    float En1;
    float En2;
    float errSum;
    float dt;
    uint32_t timer;
  public:
    float kp;
    float ki;
    float kd;
    float target;
    float output;
    uint8_t enabled;
    float ki_d;
    float kd_d;
  public:
    MyPID(  float outMin=-float(MOTORS_PWM_RESOLUTION),
            float outMax=float(MOTORS_PWM_RESOLUTION),
            float kp=0,
            float ki=0,
            float kd=0):
            //AutoPID(&this->apInput, &this->apSetpoint, &this->apOutput,outMin,outMax,kp,ki,kd)
            PID(&this->apInput, &this->apOutput, &this->apSetpoint, kp, ki, kd, DIRECT)
    {
      enabled = 0;
      this->setTarget(0);
      this->setTimeStep(pidPeriod);
      this->setOutputRange(outMin, outMax);
      this->SetMode(AUTOMATIC);
      this->setGains(kp,ki,kd);
    }

    void setTarget(float t)
    {
      target = t;
      Un1 = 0;
      En = 0;
      En1 = 0;
      En2 = 0;
      errSum = 0;
    }

    void setOutputRange(double outMin, double outMax)
    {
      PID::SetOutputLimits(outMin,outMax);
    }

    void setTimeStep(unsigned long timeStep)
    {
      dt = float(pidPeriod) / 1000.f;
      PID::SetSampleTime(timeStep);
    }

    void setGains(double kp, double ki, double kd)
    {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;

      this->ki_d = kp*ki*dt;
      this->kd_d = (kp*kd)/dt;
      
      PID::SetTunings(kp,ki,kd);
      //return AutoPID::setGains(kp,ki,kd);
    }

    float run(float input, bool correctAngle = false)
    {
      if(correctAngle)
      {
        float error = target - input;
        if(error > M_PI)
          input += 2.f*M_PI;
        else if(error < -M_PI)
          input -= 2.f*M_PI;
      }

      dt = float(micros() - timer)/1000000.f;
      timer = micros();
      En = target - input;
      //output = Un1 + (kp + kd/dt)*En + (-kp - 2.f*kd/dt)*En1 + kd/dt*En2;
      //output = Un1 + kp*(En - En1) + ki_d*En + kd_d*(En - 2.f*En1 + En2);
      errSum += dt*(En + En1)/2.f;
      output = kp*En + ki*errSum + kd*(En - En1)/dt;
      Un1 = output;
      En2 = En1;
      En1 = En;
      
      /*apSetpoint = target;
      apInput = input;
      
      //AutoPID::run();
      PID::Compute();

      output = (float)apOutput;*/

    
      
      return output;
    }

    void stop()
    {
      apInput = target;
      //PID::Compute();
      output = 0;
      errSum = 0;
      En = 0;
      En1 = 0;
      En2 = 0;
      Un1 = 0;
      timer = micros();
    }
} yawPid, pitchPid, rollPid, altPid;

//--------------------------------------------------------------
// FUNCTIONS

// switch GPIO to HIGH state
static inline ICACHE_RAM_ATTR void setPin(uint8_t pin);
// switch GPIO to LOW state
static inline ICACHE_RAM_ATTR void clearPin(uint8_t pin);

// Writes data to telemetry packet at given position. Returns new position to write
uint16_t writeTelemetryPacket(uint16_t pos, byte *packet, void *value, size_t valueSize);
// Reads magnetometer and applies calibration to raw data
void readMagnet(int16_t *mx, int16_t *my, int16_t *mz);
// Calcs heading to magnetic north
double calcHeading(int16_t mx, int16_t my);
// Update stored MPU6050 calibration
void readMpuOffsets();

//void processEspLed();
void processCommand();
void processTelemetry();
void processSensors();
void processPids();

//--------------------------------------------------------------
// INTERRUPTS

/*
bool mti = false;
uint32_t escIsrTimer = 0;
uint32_t tEscUpdate;
uint32_t tCalcNextPeriod;
uint32_t tDigWrite;
uint32_t ccount;

//I don't understand. My interrupt handler takes 190 cycles. How PWM from sdk can work with resolution 45 ns ???

// Speed critical bits
#pragma GCC optimize ("O2")
// Normally would not want two copies like this, but due to different
// optimization levels the inline attribute gets lost if we try the
// other version.

static inline ICACHE_RAM_ATTR uint32_t GetCycleCountIRQ() {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount":"=a"(ccount));
  return ccount;
}

void ICACHE_RAM_ATTR onTimerISR()
{
  //if(!mti)
  //{
    motorsEnabled = 1;
    escIsrTimer = GetCycleCountIRQ();
  //}
    
  if(motorsEnabled)
  {
    if(pwmLoop == 0)
    {
      // it takes 107 instructions
      GPOS = pwmSetMask;
      GP16O = pwmSetMask >> 16;
    }
    
    pwmNext = pwmTicks[pwmLoop];
    GPOC = pwmClearMask[pwmLoop];
    if(pwmClearMask[pwmLoop] >> 16);
      GP16O = 0;
      
    pwmLoop++;
    
    if(pwmLoop > pwmMaxLoop)
      pwmLoop = 0;
  }
  else
  {
    clearPin(M1);
    clearPin(M2);
    clearPin(M3);
    clearPin(M4);
    pwmNext = pwmPeriodTicks;
  }

  if(pwmNext < 4) // there is a bug inside core, if we set period < 4 next interrupt will be after max counter value
    pwmNext = 4;
  GPOS = 1;
  GP16O = 1;
  pwmNext = pwmPeriodTicks;
  // update timer
  timer1_write(pwmNext);

  //if(!mti)
  //{
    
    escIsrTimer = GetCycleCountIRQ() - escIsrTimer;
  //  mti = true;
  //}
}
*/

void setup()
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

  // Setup PWM

  /*
  pwmStepTicks = (10000000UL / (2*MOTORS_PWM_FREQ*MOTORS_PWM_RESOLUTION)); // (1 / freq*res) / 0.2 us (timer step)
  
  if(pwmStepTicks == 0)
    pwmStepTicks = 1;

  pwmPeriodTicks = (10000000UL / (2*MOTORS_PWM_FREQ));   // (1 / freq) / 0.2 us (timer step)

  if(pwmPeriodTicks == 0)
    pwmPeriodTicks = 1;
    
  pwmUpdate();
    
  timer1_isr_init();
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(pwmPeriodTicks);
  */

  //analogWriteFreq(MOTORS_PWM_FREQ);
  //analogWriteRange(MOTORS_PWM_RESOLUTION);
  // Initial duty -> all off

  for (uint8_t channel = 0; channel < PWM_CHANNELS; channel++) {
    pwm_duty_init[channel] = 0;
  }

  // Period

  uint32_t period = MOTORS_PWM_RESOLUTION;

  // Initialize

  pwm_init(period, pwm_duty_init, PWM_CHANNELS, io_info);
  
  // Setup program timers
  ledTimer = millis();
  telemetryTimer = millis();
  
  // Setup I2C
  Wire.begin();
  Wire.setClock(200000UL);    // there are bad SCL fronts if set greater
  // Setup IMU
  mpu.initialize();

#ifdef USE_MPU6050_DMP
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
#else
    mpu.setRate(4);   // 200 Hz
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

     // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    rtFusion.setSlerpPower(0.02);
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    rtFusion.setGyroEnable(true);
    rtFusion.setAccelEnable(true);
    rtFusion.setCompassEnable(true);
#endif

  // Setup Magnetometer
  mag.init();
  mag.setSamplingRate(200);
  mag.setOversampling(512);

  readMpuOffsets();
  // Setup barometer
  /* Default settings from datasheet. */
  baro.begin(BMP280_ADDRESS_ALT);
  baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  
  //Setup clock again because it could be reset by some sensor constructor
  Wire.setClock(200000UL);    // there are bad SCL fronts if set greater 200000, if set 400000 the system hugs
  // Setup PIDs

  FuelGauge.quickstart();
  // Start WiFi
  udp.begin(cmdPort);
}

void loop()
{
  loopTime = micros() - loopTimer;
  loopTimer = micros();
  
  ArduinoOTA.handle();

  processTelemetry();
  processPids();
  processSensors();   // Max 5 millis to process sensors
  processCommand();
  
  
  //Serial.print("pwmNext = "); Serial.println(pwmNext);
  //Serial.print("PWM ISR time = "); Serial.println(escIsrTimer);
  //Serial.print("PWM calc next period = "); Serial.println(tCalcNextPeriod);
  //Serial.print("PWM escUpdate = "); Serial.println(tEscUpdate);
  //Serial.print("DigWrite = "); Serial.println(tDigWrite);

  //processEspLed(); // just for debug
  //pwmUpdate();

  if(!motorsEnabled)
  {
    baseGas = 0;
    motor[0].setGas(0);
    motor[1].setGas(0);
    motor[2].setGas(0);
    motor[3].setGas(0);
  }

  bool pwm_up = false;
  
  for(uint8_t i = 0; i < 4; i++)
  {
    if(pwm_get_duty(motor[i].pin) != motor[i].gas)
    {
      pwm_set_duty(motor[i].gas,motor[i].pin);
      pwm_up = true;
    }
  }

  if(pwm_up)
    pwm_start();
}

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

/*
void pwmUpdate()
{
  uint32_t pwmNewSetMask = 0;
  uint32_t pwmNewClearMask[5] = {0,0,0,0,0};
  uint32_t pwmNewTicks[5] = {pwmPeriodTicks,pwmPeriodTicks,pwmPeriodTicks,pwmPeriodTicks,pwmPeriodTicks};
  uint32_t pwmGasTicks[4] = {pwmPeriodTicks,pwmPeriodTicks,pwmPeriodTicks,pwmPeriodTicks};
  uint32_t p = pwmPeriodTicks;
  uint8_t i = 0, j = 0, k = 0;
  uint8_t pwmNewMaxLoop = 0;
  uint8_t oldMotorsEnabled = motorsEnabled;

  // sort gasTicks
  for(i = 0; i < 4; i++)
  {
    if(motor[i].gasTicks > 0)
      pwmNewSetMask |= (1 << motor[i].pin);

    for(j = 0; j < 4; j++)
    {
      if(motor[i].gasTicks == pwmGasTicks[j])
      {
        pwmNewClearMask[j] |= (1 << motor[i].pin);
        break;
      }
      
      if(motor[i].gasTicks < pwmGasTicks[j] && motor[i].gasTicks > 0)
      {
        for(k = 3; j > i; j--)
        {
          pwmGasTicks[k] = pwmGasTicks[k-1];
          pwmNewClearMask[k] = pwmNewClearMask[k-1];
        }
        
        pwmGasTicks[j] = motor[i].gasTicks;
        pwmNewClearMask[j] = (1 << motor[i].pin);
        pwmNewMaxLoop++;
      }
    }
  }

  for(i = 0; i < 4; i++)
  {
    Serial.print("pwmGasTicks = ");Serial.println(pwmGasTicks[i]);
    Serial.print("pwmNewClearMask = ");Serial.println(pwmNewClearMask[i]);
  }

  for(i = 0; i < pwmNewMaxLoop; i++)
  {
    if(i > 0)
      pwmNewTicks[i] = pwmGasTicks[i] - pwmGasTicks[i - 1];
    else
      pwmNewTicks[i] = pwmGasTicks[i];
  }

  if(pwmNewMaxLoop > 0 && pwmNewMaxLoop < 5)
    pwmNewTicks[pwmNewMaxLoop] = p - pwmGasTicks[pwmNewMaxLoop - 1];
  else
    pwmNewTicks[0] = pwmPeriodTicks;

  for(i = pwmNewMaxLoop; i > 1; i--)
    pwmNewClearMask[i] = pwmNewClearMask[i-1];

  for(i = 0; i < 4; i++)
  {
    if(!(pwmNewSetMask & (1 << motor[i].pin)))
      pwmNewClearMask[0] |= (1 << motor[i].pin);
  }

  // copy new pwm values
  motorsEnabled = 0;
  pwmMaxLoop = pwmNewMaxLoop;
  pwmSetMask = pwmNewSetMask;
  Serial.print("pwmPeriodTicks = ");Serial.println(pwmPeriodTicks);
  Serial.print("pwmStepTicks = ");Serial.println(pwmStepTicks);
  Serial.print("pwmLoop = ");Serial.println(pwmLoop);
  Serial.print("pwmNext = ");Serial.println(pwmNext);
  Serial.print("pwmMaxLoop = ");Serial.println(pwmMaxLoop);
  Serial.print("pwmSetMask = ");Serial.println(pwmSetMask);
  for(i = 0; i <= pwmNewMaxLoop && i < 5; i++)
  {
    pwmClearMask[i] = pwmNewClearMask[i];
    pwmTicks[i] = pwmNewTicks[i];

    Serial.print("pwmClearMask = ");Serial.println(pwmClearMask[i]);
    Serial.print("pwmTicks = ");Serial.println(pwmTicks[i]);
  }
  motorsEnabled = oldMotorsEnabled;
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
        seaLevelhPa = baro.readPressure() / 100.f;
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
  pos = writeTelemetryPacket(pos, udpPacket, &loopTime, sizeof(loopTime));
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
  const int32_t maxRotateSpeed = 40;
  
  if(yawPid.enabled)
  {  
    dg = yawPid.run(ypr[0],true);

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

    if(dg > 0)
    {
      mg[1] += dg;
      mg[2] += dg;
    }
    else
    {
      mg[0] -= dg;
      mg[3] -= dg;
    }
    /*mg[1] += dg;
    mg[2] += dg;
    mg[0] -= dg;
    mg[3] -= dg;

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

    if(dg > 0)
    {
      mg[2] += dg;
      mg[3] += dg;  
    }
    else
    {
      mg[0] -= dg;
      mg[1] -= dg;  
    }

    /*
    mg[0] -= dg;
    mg[1] -= dg;
    mg[2] += dg;
    mg[3] += dg;

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

void processSensors()
{
#ifdef USE_MPU6050_DMP
  uint8_t mpuIntStatus = mpu.getIntStatus();        // holds actual interrupt status byte from MPU
  uint8_t fifoBuffer[64];                           // FIFO storage buffer
  uint16_t packetSize = mpu.dmpGetFIFOPacketSize();

  //if((mpuIntStatus & MPU6050_INTERRUPT_DMP_INT_BIT) && mpu.dmpPacketAvailable())
  if(mpu.dmpPacketAvailable())
  {
    //uint32_t profDt = micros();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // calc ypr (< 1 millis to execute)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(accel, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    
    //profDt = micros() - profDt;
    //Serial.print("DMP processing = ");
    //Serial.println(profDt); // 2852 - 3210 is time to get YPR from DMP
    
    // reset fifo to get fresh data in next cycle
    //mpu.resetFIFO();
    //mpuTime = micros() - mpuTimer;
    //mpuTimer = micros();
    //float dt = float(mpuTime) / 1000000.f; // convert micros to seconds
    //float az = (float(aaWorld.z) / MPU_1G)*9.80665f;
    //alt_estimator.propagate(az, dt);
  }

  if((mpuIntStatus & MPU6050_INTERRUPT_FIFO_OFLOW_BIT) || mpu.getFIFOCount() == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("MPU FIFO overflow!"));
  }

  // read magnetometer (1 millis to execute)
  readMagnet(&magnet[0],&magnet[1],&magnet[2]);
#else
  
  mpuTime = micros() - mpuTimer;
  if(mpuTime >= 5000) // 200 Hz
  {
    mpuTimer = micros();
    readMagnet(&magnet[0],&magnet[1],&magnet[2]);
    
    //uint32_t profDt = micros();
    mpu.getMotion6(&accel[0],&accel[1],&accel[2],&gyro[0],&gyro[1],&gyro[2]);
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
    // convert LSB to m/s^2
    float ax = float(accel[0]);///16384.f;
    float ay = float(accel[1]);///16384.f;
    float az = float(accel[2]);///16384.f;
    // convert LSB to mgauss/10
    float mx = float(magnet[0]);///120.f;
    float my = float(magnet[1]);///120.f;
    float mz = float(magnet[2]);///120.f;
    // convert deg to rad
    gx *= RTMATH_DEGREE_TO_RAD;
    gy *= RTMATH_DEGREE_TO_RAD;
    gz *= RTMATH_DEGREE_TO_RAD;
    // sort out axes as in RTIMUMPU9250
    RTVector3 rtGyro(gx,-gy,-gz);
    RTVector3 rtAccel(-ax,ay,az);
    RTVector3 rtMagnet(mx,my,mz);
    rtFusion.newIMUData(rtGyro, rtAccel, rtMagnet, timestamp);
    const RTVector3 &pose = rtFusion.getFusionPose();
    ypr[0] = pose.z();
    ypr[1] = pose.y();
    ypr[2] = pose.x();
    //readMpuOffsets();
    //profDt = micros() - profDt;
    //Serial.print("Magwick processing = ");
    //Serial.println(profDt); // 1414 - 1822 is time to get YPR from Magwick (with reading MPU6050) Magwick very slow updates yaw angle and has big noise when motors run
                            // 2280 - 2341 is time to get YPR from RTIMUlib (with reading MPU6050)
  }
#endif  
  // correct result according to copter x axis
  heading = calcHeading(magnet[0],magnet[1],magnet[2], ypr[1], ypr[2]);
  // baro sensor
  temperature = baro.readTemperature();
  pressure = baro.readPressure() / 100.f;
  altitude = baro.readAltitude(seaLevelhPa);
  //alt_estimator.update(altitude);
  //altitude = alt_estimator.h;
}

double calcHeading(int16_t mx, int16_t my, int16_t mz, double pitch, double roll)
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
