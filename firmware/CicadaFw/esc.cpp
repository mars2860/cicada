#include "pdl.h"

#include <ESP8266WiFi.h>
#include <SPI.h>
#include "dshot.h"

extern "C"
{
  #include "pwm.h"
  #include "user_interface.h"
}

#define DSHOT_MAX_GAS       1999
#define DSHOT_NULL_GAS      0
#define DSHOT_MIN_GAS       0

#define PWM_2K5HZ_MAX_GAS   2000
#define PWM_5KHZ_MAX_GAS    1000
#define PWM_10KHZ_MAX_GAS   500
#define PWM_20KHZ_MAX_GAS   250
#define PWM_40KHZ_MAX_GAS   125
#define PWM_50KHZ_MAX_GAS   100

#define CICADA_DEFAULT_DSHOT_ESC        0
#define CICADA_0703_TEENY_PRO_ESC       10
#define CICADA_PWM_2K5HZ_ESC            20
#define CICADA_PWM_5KHZ_ESC             21
#define CICADA_PWM_10KHZ_ESC            22
#define CICADA_PWM_20KHZ_ESC            23
#define CICADA_PWM_40KHZ_ESC            24
#define CICADA_PWM_50KHZ_ESC            25

// Teeny pro 5A Esc
#define M1_CICADA_0703_PIN              15
#define M2_CICADA_0703_PIN              2
#define M3_CICADA_0703_PIN              16
#define M4_CICADA_0703_PIN              0

// Skystars KO25
#define M1_KO25_PIN              0
#define M2_KO25_PIN              16
#define M3_KO25_PIN              2
#define M4_KO25_PIN              15

// PWM Esc
#define M1_PWM_PIN              15
#define M2_PWM_PIN              2
#define M3_PWM_PIN              0
#define M4_PWM_PIN              16
#define PWM_OE_PIN              12

#define CICADA_X_FRAME_REVERSED   0
#define CICADA_X_FRAME            1
#define CICADA_CROSS_FRAME        2

/// to convert motor index to pwm channel
uint8_t pwmMotorToChannel[PDL_MOTOR_COUNT];

void pdlSetupTeenyProEsc(pdlDroneState*)
{
  pdlSetMotorMaxGas(DSHOT_MAX_GAS);
  pdlSetMotorNullGas(0);
  pdlSetMotorMinGas(0);

  pinMode(M1_CICADA_0703_PIN, OUTPUT);
  pinMode(M2_CICADA_0703_PIN, OUTPUT);
  pinMode(M3_CICADA_0703_PIN, OUTPUT);
  pinMode(M4_CICADA_0703_PIN, OUTPUT);

  digitalWrite(M1_CICADA_0703_PIN, LOW);  // digitalWrite takes 175 cpu cycles to execute!
  digitalWrite(M2_CICADA_0703_PIN, LOW);
  digitalWrite(M3_CICADA_0703_PIN, LOW);
  digitalWrite(M4_CICADA_0703_PIN, LOW);

  dshotSetup( M1_CICADA_0703_PIN,
              M2_CICADA_0703_PIN,
              M3_CICADA_0703_PIN,
              M4_CICADA_0703_PIN,
              1000);
}

void pdlSetupDefaultDshotEsc(pdlDroneState*)
{
  pdlSetMotorMaxGas(DSHOT_MAX_GAS);
  pdlSetMotorNullGas(0);
  pdlSetMotorMinGas(0);

  pinMode(M1_KO25_PIN, OUTPUT);
  pinMode(M2_KO25_PIN, OUTPUT);
  pinMode(M3_KO25_PIN, OUTPUT);
  pinMode(M4_KO25_PIN, OUTPUT);

  digitalWrite(M1_KO25_PIN, LOW);  // digitalWrite takes 175 cpu cycles to execute!
  digitalWrite(M2_KO25_PIN, LOW);
  digitalWrite(M3_KO25_PIN, LOW);
  digitalWrite(M4_KO25_PIN, LOW);

  dshotSetup(M1_KO25_PIN,M2_KO25_PIN,M3_KO25_PIN,M4_KO25_PIN,1000);
}

void pdlSetupPwmEsc(pdlDroneState*, int32_t maxGas, int32_t nullGas, int32_t minGas)
{
  pdlSetMotorMaxGas(maxGas);
  pdlSetMotorNullGas(nullGas);
  pdlSetMotorMinGas(minGas);

  // GPIO config
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M3_PWM_PIN, OUTPUT);
  pinMode(M4_PWM_PIN, OUTPUT);

  digitalWrite(M1_PWM_PIN, LOW);  // digitalWrite takes 175 cpu cycles to execute!
  digitalWrite(M2_PWM_PIN, LOW);
  digitalWrite(M3_PWM_PIN, LOW);
  digitalWrite(M4_PWM_PIN, LOW);

  //output enable pin
  SPI.end(); // close SPI because we use its pin. SPI is not available For PWM Esc
  pinMode(PWM_OE_PIN, OUTPUT);
  digitalWrite(PWM_OE_PIN, HIGH);

  uint32 io_info[PDL_MOTOR_COUNT][3] =
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

  pwmMotorToChannel[0] = 0;
  pwmMotorToChannel[1] = 3;
  pwmMotorToChannel[2] = 2;
  pwmMotorToChannel[3] = 1;

  pwm_init(maxGas, 0, PDL_MOTOR_COUNT, io_info);
}

void pdlSetupPwm2K5hzEsc(pdlDroneState *ds)
{
  pdlSetupPwmEsc(ds, PWM_2K5HZ_MAX_GAS, 0, 0);
}

void pdlSetupPwm5KhzEsc(pdlDroneState *ds)
{
  pdlSetupPwmEsc(ds, PWM_5KHZ_MAX_GAS, 0, 0);
}

void pdlSetupPwm10KhzEsc(pdlDroneState *ds)
{
  pdlSetupPwmEsc(ds, PWM_10KHZ_MAX_GAS, 0, 0);
}

void pdlSetupPwm20KhzEsc(pdlDroneState *ds)
{
  pdlSetupPwmEsc(ds, PWM_20KHZ_MAX_GAS, 0, 0);
}

void pdlSetupPwm40KhzEsc(pdlDroneState *ds)
{
  pdlSetupPwmEsc(ds, PWM_40KHZ_MAX_GAS, 0, 0);
}

void pdlSetupPwm50KhzEsc(pdlDroneState *ds)
{
  pdlSetupPwmEsc(ds, PWM_50KHZ_MAX_GAS, 0, 0);
}

void pdlUpdateDefaultDshotEsc(pdlDroneState *ds)
{
  uint8_t i;
  uint8_t modified = 0;
  static int32_t oldMotorGas[PDL_MOTOR_COUNT] = {0,0,0,0};

  for(i = 0; i < PDL_MOTOR_COUNT ; i++)
  {
    if(ds->motorGas[i] != oldMotorGas[i])
    {
      modified = 1;
      oldMotorGas[i] = ds->motorGas[i];
    }
  }

  if(modified)
  {
    dshotSet( ds->motorGas[0] + 48,
              ds->motorGas[1] + 48,
              ds->motorGas[2] + 48,
              ds->motorGas[3] + 48);
  }

  if(ds->motorsEnabled != dshotIsEnabled())
  {
    delay(2);   // wait for motors stop
    dshotEnable(ds->motorsEnabled);
  }
}

void pdlUpdatePwmEsc(pdlDroneState *ds)
{
  uint8_t modified = 0;

  digitalWrite(PWM_OE_PIN,!ds->motorsEnabled); // we use 74hc125 which enables outputs by LOW signal

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
  {
    if( (int32_t)pwm_get_duty(pwmMotorToChannel[i]) != ds->motorGas[i])
    {
      pwm_set_duty(ds->motorGas[i], pwmMotorToChannel[i]);
      modified = 1;
    }
  }

  if(modified)
    pwm_start();
}

void pdlSetupEsc(pdlDroneState *ds)
{
  switch(ds->esc)
  {
  case CICADA_0703_TEENY_PRO_ESC:
    pdlSetupTeenyProEsc(ds);
    break;
  case CICADA_PWM_2K5HZ_ESC:
      pdlSetupPwm2K5hzEsc(ds);
      break;
  case CICADA_PWM_5KHZ_ESC:
      pdlSetupPwm5KhzEsc(ds);
      break;
  case CICADA_PWM_10KHZ_ESC:
      pdlSetupPwm10KhzEsc(ds);
      break;
  case CICADA_PWM_20KHZ_ESC:
    pdlSetupPwm20KhzEsc(ds);
    break;
  case CICADA_PWM_40KHZ_ESC:
      pdlSetupPwm40KhzEsc(ds);
      break;
  case CICADA_PWM_50KHZ_ESC:
      pdlSetupPwm50KhzEsc(ds);
      break;
  default:
    pdlSetupDefaultDshotEsc(ds);
    break;
  }
}

void pdlPidsToMotors(pdlDroneState* ds)
{
  switch(ds->frame)
  {
  case CICADA_CROSS_FRAME:
    pdlPidsToCrossFrame(ds);
    break;
  case CICADA_X_FRAME:
    pdlPidsToXFrame(ds);
    break;
  case CICADA_X_FRAME_REVERSED:
    pdlPidsToXFrameReversed(ds);
    break;
  default:
    pdlPidsToXFrameReversed(ds);
  }
}

void pdlUpdateEsc(pdlDroneState* ds)
{
  switch(ds->esc)
  {
  case CICADA_PWM_2K5HZ_ESC:
  case CICADA_PWM_5KHZ_ESC:
  case CICADA_PWM_10KHZ_ESC:
  case CICADA_PWM_20KHZ_ESC:
  case CICADA_PWM_40KHZ_ESC:
  case CICADA_PWM_50KHZ_ESC:
    pdlUpdatePwmEsc(ds);
    break;
  default:
    pdlUpdateDefaultDshotEsc(ds);
  }
}

void pdlSetMotorsDir(pdlDroneState *ds, uint8_t dir)
{
  uint8_t enabled;
  uint8_t cmd;

  switch(ds->esc)
  {
  case CICADA_DEFAULT_DSHOT_ESC:
  case CICADA_0703_TEENY_PRO_ESC:

    enabled = dshotIsEnabled();
    // 20 DSHOT_CMD_SPIN_DIRECTION_NORMAL, // Need 6x, no wait required
    // 21 DSHOT_CMD_SPIN_DIRECTION_REVERSED, // Need 6x, no wait required
    cmd = 20;
    if(dir)
      cmd = 21;

    dshotEnable(0);
    delay(5);

    dshotSet(cmd,cmd,cmd,cmd);

    dshotWrite300();
    delay(1);
    dshotWrite300();
    delay(1);
    dshotWrite300();
    delay(1);
    dshotWrite300();
    delay(1);
    dshotWrite300();
    delay(1);
    dshotWrite300();
    delay(1);
    dshotWrite300();
    delay(1);
    dshotWrite300();

    delay(5);

    dshotEnable(enabled);

    break;
  }
}
