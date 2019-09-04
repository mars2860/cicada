#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

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

#define LED_FLASH_TIME 1000

#define MOTORS_PWM_FREQ           1024UL
#define MOTORS_PWM_RESOLUTION     256UL

//---------------------------------------------------------------
// COMMANDS

#define CMD_SWITCH_MOTORS         100
#define CMD_SET_MOTORS_GAS        101

//---------------------------------------------------------------
// VARIABLES

const char* ssid = STASSID;
const char* password = STAPSK;

WiFiUDP udp;
unsigned int localUdpPort = 4210;   // local port to listen on
uint8_t udpPacket[255];             // buffer for udp packets

uint32_t ledTimer;

uint8_t escOutputs = 0;             // state of outputs of escaper, one bit per channel

uint8_t motorsEnabled = 0;

volatile uint32_t pwmStepTicks;              // ticks of timer to next pwm step
volatile uint32_t pwmPeriodTicks;            // ticks of timer per one pwm period
volatile uint32_t pwmNext = 0;               // ticks to next pwm timer interrupt
volatile uint32_t pwmTimer = 0;              // count ticks from pwm period start

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

//--------------------------------------------------------------
// FUNCTIONS

// Updates escaper outputs. Each bit of state represents one channel state
void ICACHE_RAM_ATTR escaperUpdate(uint8_t state);

//void processEspLed();

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
  // GPIO reset
  digitalWrite(HC595_DATA, LOW);
  digitalWrite(HC595_LATCH, HIGH);  // Disable OE pin of HC595
  digitalWrite(HC595_CLOCK, LOW);
  digitalWrite(CAM_SD, LOW);
  
  // GPIO config
  pinMode(HC595_DATA, OUTPUT);
  pinMode(HC595_LATCH, OUTPUT);
  pinMode(HC595_CLOCK, OUTPUT);
  pinMode(CAM_SD, OUTPUT);

  // Clear HC595
  escaperUpdate(0xAA);

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

  ledTimer = millis();
  
  /*Serial.println("pwmPeriodTicks = ");
  Serial.print(pwmPeriodTicks);
  Serial.print(", pwmStepTicks = ");
  Serial.println(pwmStepTicks);*/

  udp.begin(localUdpPort);
}

void loop()
{
  ArduinoOTA.handle();

  if(udp.parsePacket())
  {
    udp.read(udpPacket, sizeof(udpPacket));
    uint8_t cmd = udpPacket[0];
    switch(cmd)
    {
      case CMD_SWITCH_MOTORS:
        motorsEnabled = udpPacket[1];
        break;
      case CMD_SET_MOTORS_GAS:
        for(uint8_t i = 0; i < 4; i++)
        {
          motor[i].setGas(udpPacket[1 + i]);

          /*Serial.print("m");
          Serial.print(i);
          Serial.print(" = ");
          Serial.println(motor[i].gas);*/
        }
         
        break;
    }
  }

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
