#include "pdl.h"
#include "main.h"

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

#define CICADA_X_FRAME_REVERSED       0
#define CICADA_X_FRAME                1
#define CICADA_CROSS_FRAME            2
#define CICADA_CROSS_FRAME_REVERSED   3

#ifdef CICADA_MICRO
  #include "esc_cicada_micro.h"
#elif CICADA_GY91
  #include "esc_cicada_gy91.h"
#elif ARDUINO_XIAO_ESP32S3
  #include "esc_xiao_esp32s3.h"
#endif

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

  LOG_INFO("TeenyPro5A ESC is ok");
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

  LOG_INFO("DShot300 ESC is ok");
}

void pdlSetupPwm2K5hzEsc(pdlDroneState *ds)
{
  if(pdlSetupPwmEsc(ds, PWM_2K5HZ_MAX_GAS, 0, 0))
  {
    LOG_INFO("PWM-2.5KHz ESC is ok");
  }
  else
  {
    LOG_ERROR("PWM-2.5KHz ESC is not available");
  }
}

void pdlSetupPwm5KhzEsc(pdlDroneState *ds)
{
  if(pdlSetupPwmEsc(ds, PWM_5KHZ_MAX_GAS, 0, 0))
  {
    LOG_INFO("PWM-5KHz ESC is ok");
  }
  else
  {
    LOG_ERROR("PWM-5KHz ESC is not available");
  }
}

void pdlSetupPwm10KhzEsc(pdlDroneState *ds)
{
  if(pdlSetupPwmEsc(ds, PWM_10KHZ_MAX_GAS, 0, 0))
  {
    LOG_INFO("PWM-10KHz ESC is ok");
  }
  else
  {
    LOG_ERROR("PWM-10KHz ESC is not available");
  }
}

void pdlSetupPwm20KhzEsc(pdlDroneState *ds)
{
  if(pdlSetupPwmEsc(ds, PWM_20KHZ_MAX_GAS, 0, 0))
  {
    LOG_INFO("PWM-20KHz ESC is ok");
  }
  else
  {
    LOG_ERROR("PWM-20KHz ESC is not available");
  }
}

void pdlSetupPwm40KhzEsc(pdlDroneState *ds)
{
  if(pdlSetupPwmEsc(ds, PWM_40KHZ_MAX_GAS, 0, 0))
  {
    LOG_INFO("PWM-40KHz ESC is ok");
  }
  else
  {
    LOG_ERROR("PWM-40KHz ESC is not available");
  }
}

void pdlSetupPwm50KhzEsc(pdlDroneState *ds)
{
  if(pdlSetupPwmEsc(ds, PWM_50KHZ_MAX_GAS, 0, 0))
  {
    LOG_INFO("PWM-50KHz ESC is ok");
  }
  else
  {
    LOG_ERROR("PWM-50KHz ESC is not available");
  }
}

void pdlSetupEsc(pdlDroneState *ds)
{
#ifdef ARDUINO_XIAO_ESP32S3
  if(!semEsc)
  {
    semEsc = xSemaphoreCreateMutex();
  }

  xSemaphoreTake(semEsc,portMAX_DELAY);
#endif

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

#ifdef ARDUINO_XIAO_ESP32S3
  xSemaphoreGive(semEsc);

  if(!hEscTask)
  {
    xTaskCreatePinnedToCore(escTask,"escTask",2048,ds,5,&hEscTask,1);
  }
#endif
}

void pdlPidsToMotors(pdlDroneState* ds)
{
  switch(ds->frame)
  {
  case CICADA_CROSS_FRAME:
    pdlPidsToCrossFrame(ds);
    break;
  case CICADA_CROSS_FRAME_REVERSED:
      pdlPidsToCrossFrameReversed(ds);
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

#if defined CICADA_GY91 || defined CICADA_MICRO

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

#elif ARDUINO_XIAO_ESP32S3

void pdlUpdateEsc(pdlDroneState* ds)
{
  // We update ESC in separate task because we need to update dshot esc every 1ms
  // but PDL invokes pdlUpdateEsc every 1-10 ms
}

void escTask(void* pvParameters)
{
  pdlDroneState *ds = (pdlDroneState*)pvParameters;

  if(!ds)
  {
    LOG_ERROR("pdlDroneState is NULL in escTask");
    return;
  }

  for(;;)
  {
    xSemaphoreTake(semEsc,portMAX_DELAY);

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

    xSemaphoreGive(semEsc);

    delay(1);
  }
}

void pdlSetMotorsDir(pdlDroneState *ds, uint8_t dir)
{
  uint8_t cmd;

  xSemaphoreTake(semEsc, portMAX_DELAY);

  switch(ds->esc)
  {
  case CICADA_DEFAULT_DSHOT_ESC:
  case CICADA_0703_TEENY_PRO_ESC:
    if(!pDShotM0 || !pDShotM1 || !pDShotM2 || !pDShotM3)
    {
      return;
    }
    // 20 DSHOT_CMD_SPIN_DIRECTION_NORMAL, // Need 6x, no wait required
    // 21 DSHOT_CMD_SPIN_DIRECTION_REVERSED, // Need 6x, no wait required
    cmd = 20;
    if(dir)
    {
      cmd = 21;
    }

    for(uint8_t i = 0; i < 10; i++)
    {
      pDShotM0->sendCmdValue(cmd);
      pDShotM1->sendCmdValue(cmd);
      pDShotM2->sendCmdValue(cmd);
      pDShotM3->sendCmdValue(cmd);
      delay(1);
    }

    break;
  }

  xSemaphoreGive(semEsc);
}

#endif
