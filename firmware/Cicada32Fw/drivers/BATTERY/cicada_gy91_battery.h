#ifndef DRIVERS_BATTERY_CICADA_GY91_BATTERY_H_
#define DRIVERS_BATTERY_CICADA_GY91_BATTERY_H_

#include "battery.h"
#include "pca9536.h"
#include "RunningMedian.h"

class CicadaGy91BatteryDriver: public BatteryDriver
{
private:
  PCA9536 exGpio;
  uint8_t adcPin;
  uint8_t adcStabilizeDelay;
  uint32_t timestamp;
  float voltage;
  float current;
  float percent;
  float capacity;
  RunningMedian fv;
  RunningMedian fa;
public:
  CicadaGy91BatteryDriver(uint8_t pin): fv(10), fa(10)
  {
    adcPin = pin;
    adcStabilizeDelay = 0;
    timestamp = 0;
    voltage = 0;
    current = 0;
    percent = 100.f;
    capacity = 0;
  }

  bool setup()
  {
    if(exGpio.ping() != 0)
      return false;
    else
      LOG_INFO("PCA9536 is ok");

    exGpio.setMode(IO_OUTPUT);
    exGpio.setState(IO0,IO_LOW);

    LOG_INFO("Cicada-gy91 battery indicator is ok");
    return true;
  }

  bool read(pdlDroneState *ds)
  {
    voltage = analogRead(adcPin);
    exGpio.setState(IO0,IO_HIGH);
    delay(1);
    float dt = pdlGetDeltaTime(pdlMicros(),timestamp);
    timestamp = pdlMicros();
    current = analogRead(adcPin);
    exGpio.setState(IO0,IO_LOW);

    voltage *= ds->battery.voltScaler;
    current *= ds->battery.curnScaler;

    fv.add(voltage);
    voltage = fv.getAverage();

    fa.add(current);
    current = fa.getAverage();

    if(adcStabilizeDelay < 20 && ds->battery.voltScaler > 0)  // also wait for voltScaler setup from host
    {
      adcStabilizeDelay++;
      voltage = 0;
      current = 0;
      percent = 100.f;  // we can't set lower percent if we set 0% here
      capacity = 0;
      return false;
    }

    capacity += (current * dt) / 3600000.0f; // convert to mA*h

    if(voltage >= ds->battery.minVoltage && ds->battery.minVoltage >= 0.f)
    {
      float curPercent = 0.f;

      // for Li-Ion minVoltage = 5.9V, maxVoltage = 6.95V at load

      /*if(ds->motorGas[0] > PDL_MOTOR_GAS_MAX / 4)
        // Кривая разряда при включенных двигателях
        // Чтобы определить уровни напряжений необходимо выполнить полет до полного разряда батареи и посмотреть кривую разряда
        curPercent = pdlBatteryPercentLinear(ds->battery.voltage, 5.9f, 6.95f);
      else
      {
        // Кривая разряда при выключенных двигателях
        curPercent = pdlBatteryPercentSigmoidal(ds->battery.voltage, 5.9f, 8.4f);
      }*/

      curPercent = pdlBatteryPercentSigmoidal(voltage, ds->battery.minVoltage, ds->battery.maxVoltage);

      if(curPercent < percent)
        percent = curPercent;
    }

    return true;
  }

  float getVoltage()
  {
    return voltage;
  }

  float getCurrent()
  {
    return current;
  }

  float getPercent()
  {
    return percent;
  }

  float getCapacity()
  {
    return capacity;
  }
};

#endif /* DRIVERS_BATTERY_CICADA_GY91_BATTERY_H_ */
