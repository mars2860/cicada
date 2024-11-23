#ifndef DRIVERS_BATTERY_ADC_BATTERY_H_
#define DRIVERS_BATTERY_ADC_BATTERY_H_

#include "battery.h"
#include "RunningMedian.h"

class OneChlAdcBatteryDriver: public BatteryDriver
{
private:
  uint8_t adcStabilizeDelay;
  uint8_t adcPin;
  float voltage;
  float percent;
  RunningMedian fv;
public:
  OneChlAdcBatteryDriver(uint8_t pin): fv(10)
  {
    adcPin = pin;
    adcStabilizeDelay = 0;
    percent = 100.f;
    voltage = 0;
  }

  bool setup()
  {
    LOG_INFO("Simple battery indicator is ok");
    return true;
  }

  bool read(pdlDroneState *ds)
  {
    float adcValue = analogRead(adcPin);

    voltage = adcValue * ds->battery.voltScaler;

    fv.add(voltage);
    voltage = fv.getAverage();

    if(adcStabilizeDelay < 20 && ds->battery.voltScaler > 0)  // also wait for voltScaler setup from host
    {
      adcStabilizeDelay++;
      percent = 100.f;  // we can't set lower percent next if we set 0% here at this time
      voltage = 0;
      return false;
    }

    if(voltage >= ds->battery.minVoltage && ds->battery.minVoltage >= 0.f)
    {
      float curPercent = 0.f;

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
    return 0.f;
  }

  float getPercent()
  {
    return percent;
  }

  float getCapacity()
  {
    return 0.f;
  }
};

#endif /* DRIVERS_BATTERY_ADC_BATTERY_H_ */
