#ifndef DRIVERS_ESC_ESC_CICADA_MICRO_H_
#define DRIVERS_ESC_ESC_CICADA_MICRO_H_

#include "esc_cicada_dshot.h"

extern "C"
{
  #include "pwm.h"
  #include "user_interface.h"
}

// PWM Esc
#define M1_PWM_PIN              15
#define M2_PWM_PIN              2
#define M3_PWM_PIN              0
#define M4_PWM_PIN              16
#define PWM_OE_PIN              14

/// to convert motor index to pwm channel
uint8_t pwmMotorToChannel[PDL_MOTOR_COUNT];

bool pdlSetupPwmEsc(pdlDroneState*, int32_t maxGas, int32_t nullGas, int32_t minGas)
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

  return true;
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

#endif /* DRIVERS_ESC_ESC_CICADA_MICRO_H_ */
