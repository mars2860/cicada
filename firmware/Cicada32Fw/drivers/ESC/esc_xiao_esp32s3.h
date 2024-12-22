#ifndef DRIVERS_ESC_ESC_XIAO_ESP32S3_H_
#define DRIVERS_ESC_ESC_XIAO_ESP32S3_H_

#include "pdl.h"
#include "DShotRMT.h"

#ifdef CICADA32_0802
#define DSHOT_M0_PIN        D7
#define DSHOT_M1_PIN        D10
#define DSHOT_M2_PIN        D0
#define DSHOT_M3_PIN        D6
#else
#define DSHOT_M0_PIN        D0
#define DSHOT_M1_PIN        D1
#define DSHOT_M2_PIN        D2
#define DSHOT_M3_PIN        D3
#endif

// Teeny pro 5A Esc
#define M1_CICADA_0703_PIN              DSHOT_M0_PIN
#define M2_CICADA_0703_PIN              DSHOT_M1_PIN
#define M3_CICADA_0703_PIN              DSHOT_M2_PIN
#define M4_CICADA_0703_PIN              DSHOT_M3_PIN

// Skystars KO25
#define M1_KO25_PIN              DSHOT_M0_PIN
#define M2_KO25_PIN              DSHOT_M1_PIN
#define M3_KO25_PIN              DSHOT_M2_PIN
#define M4_KO25_PIN              DSHOT_M3_PIN

// PWM Esc
#define M1_PWM_PIN              D0
#define M2_PWM_PIN              D1
#define M3_PWM_PIN              D2
#define M4_PWM_PIN              D3
#define PWM_OE_PIN              D10

#define DSHOT_MODE DSHOT300

DShotRMT *pDShotM0 = NULL;
DShotRMT *pDShotM1 = NULL;
DShotRMT *pDShotM2 = NULL;
DShotRMT *pDShotM3 = NULL;

TaskHandle_t hEscTask = NULL;
SemaphoreHandle_t semEsc = NULL;
void escTask(void* pvParameters);

void dshotSetup(uint8_t m0, uint8_t m1, uint8_t m2, uint8_t m3, uint32_t updatePeriod)
{
  (void)updatePeriod;

  if(pDShotM0)
  {
    delete pDShotM0;
    pDShotM0 = NULL;
  }

  if(pDShotM1)
  {
    delete pDShotM1;
    pDShotM1 = NULL;
  }

  if(pDShotM2)
  {
    delete pDShotM2;
    pDShotM2 = NULL;
  }

  if(pDShotM3)
  {
    delete pDShotM3;
    pDShotM3 = NULL;
  }

  pDShotM0 = new DShotRMT(m0,0);
  pDShotM1 = new DShotRMT(m1,1);
  pDShotM2 = new DShotRMT(m2,2);
  pDShotM3 = new DShotRMT(m3,3);

  pDShotM0->begin(DSHOT_MODE);
  pDShotM1->begin(DSHOT_MODE);
  pDShotM2->begin(DSHOT_MODE);
  pDShotM3->begin(DSHOT_MODE);
}

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

  // TODO Setup PWM ESC
  return false;
}

void pdlUpdateDefaultDshotEsc(pdlDroneState *ds)
{
  static uint8_t oldMotorsEnabled = 0;

  if(!ds->motorsEnabled && oldMotorsEnabled)
  {
    // STOP MOTORS
    for(uint8_t i = 0; i < 5; i++)
    {
      pDShotM0->sendThrottleValue(48);
      pDShotM1->sendThrottleValue(48);
      pDShotM2->sendThrottleValue(48);
      pDShotM3->sendThrottleValue(48);
      delay(1);
    }
  }
  else if(ds->motorsEnabled)
  {
    pDShotM0->sendThrottleValue(ds->motorGas[0] + 48);
    pDShotM1->sendThrottleValue(ds->motorGas[1] + 48);
    pDShotM2->sendThrottleValue(ds->motorGas[2] + 48);
    pDShotM3->sendThrottleValue(ds->motorGas[3] + 48);
  }
  else
  {
    digitalWrite(DSHOT_M0_PIN, LOW);
    digitalWrite(DSHOT_M1_PIN, LOW);
    digitalWrite(DSHOT_M2_PIN, LOW);
    digitalWrite(DSHOT_M3_PIN, LOW);
  }

  oldMotorsEnabled = ds->motorsEnabled;
}

void pdlUpdatePwmEsc(pdlDroneState *ds)
{
  /* TODO Update PWM ESC
  uint8_t modified = 0;

  digitalWrite(PWM_OE_PIN,!ds->motorsEnabled); // we use 74hc125 which enables outputs by LOW signal

  for(uint8_t i = 0; i < PDL_MOTOR_COUNT; i++)
  {

  }
  */
}

#endif /* DRIVERS_ESC_ESC_XIAO_ESP32S3_H_ */
