#include "pdl.h"

#include <ESP8266WiFi.h>
extern "C"
{
  #include "pwm.h"
  #include "user_interface.h"
}

#define M1_PIN              15
#define M2_PIN              2
#define M3_PIN              0
#define M4_PIN              16

/// to convert motor index to pwm channel
uint8_t pwmMotorToChannel[PDL_MOTOR_COUNT];

void pdlSetupEscaper(pdlDroneState*)
{
  // GPIO config
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);

  digitalWrite(M1_PIN, LOW);  // digitalWrite takes 175 cpu cycles to execute!
  digitalWrite(M2_PIN, LOW);
  digitalWrite(M3_PIN, LOW);
  digitalWrite(M4_PIN, LOW);

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

  pdlSetMotorGasLimits(0,0,2000);
  pdlSetEscaperUpdatePeriod(0);

  pwmMotorToChannel[0] = 2;
  pwmMotorToChannel[1] = 1;
  pwmMotorToChannel[2] = 0;
  pwmMotorToChannel[3] = 3;

  pwm_init(2000, 0, PDL_MOTOR_COUNT, io_info);
}

void pdlUpdateEscaper(pdlDroneState *ds)
{
  uint8_t modified = 0;

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
