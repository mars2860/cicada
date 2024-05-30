#include "pdl.h"
#include "pca9536.h"
#include "RunningMedian.h"

class BatteryDriver
{
public:
  virtual ~BatteryDriver() {};
  virtual bool setup() = 0;
  virtual bool read(pdlDroneState *ds) = 0;
  virtual float getVoltage() = 0;
  virtual float getCurrent() = 0;
  virtual float getPercent() = 0;
  virtual float getCapacity() = 0;
};

class OneChlAdcBatteryDriver: public BatteryDriver
{
private:
  uint8_t adcStabilizeDelay;
  float voltage;
  float percent;
  RunningMedian fv;
public:
  OneChlAdcBatteryDriver(): fv(10)
  {
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
    float adcValue = analogRead(A0);

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

class TwoChlAdcBatteryDriver: public BatteryDriver
{
private:
  PCA9536 exGpio;
  uint8_t adcStabilizeDelay;
  uint32_t timestamp;
  float voltage;
  float current;
  float percent;
  float capacity;
  RunningMedian fv;
  RunningMedian fa;
public:
  TwoChlAdcBatteryDriver(): fv(10), fa(10)
  {
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

    exGpio.setMode(IO_OUTPUT);
    exGpio.setState(IO0,IO_LOW);
    LOG_INFO("PCA9536 is ok");
    return true;
  }

  bool read(pdlDroneState *ds)
  {
    voltage = analogRead(A0);
    exGpio.setState(IO0,IO_HIGH);
    delay(1);
    float dt = pdlGetDeltaTime(pdlMicros(),timestamp);
    timestamp = pdlMicros();
    current = analogRead(A0);
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

BatteryDriver *pBattery = 0;

//-----------------------------------------------------------------------------
// PDL BATTERY FUNCTION IMPLEMENTATION

void pdlSetupBattery(pdlDroneState*)
{
  if(pBattery)  // reinit when load default config command is received
  {
    pBattery->setup();
    return;
  }
  // scan for two chl adc battery indicator
  pBattery = new TwoChlAdcBatteryDriver();
  if(pBattery->setup())
    return;
  // create one chl adc battery indicator
  delete pBattery;  // remove old instance
  pBattery = new OneChlAdcBatteryDriver();
  if(pBattery->setup())
    return;
  // battery is not found
  delete pBattery;
  pBattery = 0;
  LOG_INFO("Battery indicator is not found");
}

void pdlReadBattery(pdlDroneState *ds)
{
  if(!pBattery)
    return;

  if(pBattery->read(ds))
  {
    ds->battery.voltage = pBattery->getVoltage();
    ds->battery.current = pBattery->getCurrent();
    ds->battery.percent = pBattery->getPercent();
    ds->battery.capacity = pBattery->getCapacity();
  }
}
