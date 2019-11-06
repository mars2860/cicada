#ifndef PID_H
#define PID_H

#include <Arduino.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

class Pid
{
  protected:
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
    Pid()
    {
      enabled = 0;
      kp = 0;
      ki = 0;
      kd = 0;
      this->setTarget(0);
    }

    void setTarget(float t)
    {
      target = t;
      //Un1 = 0;
      //En = 0;
      //En1 = 0;
      //En2 = 0;
      //errSum = 0;
    }

    void setTimeStep(uint32_t dt)
    {

    }

    void setGains(float kp, float ki, float kd)
    {
      this->kp = kp;
      this->kd = kd;
      if(this->ki != ki)
      {
        this->ki = ki;
        errSum = 0;
      }
    }

    float run(float input, bool correctAngle = false)
    {
      En = target - input;

      if(correctAngle)
      {
        if(En > M_PI)
          input += 2.f*M_PI;
        else if(En < -M_PI)
          input -= 2.f*M_PI;
      }

      En = target - input;
      dt = float(micros() - timer)/1000000.f;
      timer = micros();
      //output = Un1 + (kp + kd/dt)*En + (-kp - 2.f*kd/dt)*En1 + kd/dt*En2;
      //output = Un1 + kp*(En - En1) + ki_d*En + kd_d*(En - 2.f*En1 + En2);
      errSum += dt*(En + En1)/2.f;
      output = kp*En + ki*errSum + kd*(En - En1)/dt;
      Un1 = output;
      En2 = En1;
      En1 = En;

      return output;
    }

    void stop()
    {
      output = 0;
      errSum = 0;
      En = 0;
      En1 = 0;
      En2 = 0;
      Un1 = 0;
      timer = micros();
    }
};

#endif
