#include "pdl.h"
#include "Bitcraze_PMW3901.h"
#include "movingAvg.h"

// Warn: for SPI-CS used TXD pin! Sold out this wire from OF-module if you going to programm esp8266 by UART
Bitcraze_PMW3901 flow(1);

boolean sensorValid;

const float fov = 42.f * 3.14f / 180.f; // field of view in radians
const float scaler = 12.5f; // pixels per mm (determined by experiment)
const float Npx = 30.f; // sensor resolution
const float k1 = Npx*scaler;
const float k2 = k1/fov;
const float k3 =  2.f * tanf(fov / 2.f);

void pdlSetupOpticalFlow(pdlDroneState*)
{
  delay(500);

  sensorValid = flow.begin();
  pdlSetOpticalFlowReadPeriod(100000);  // as in ArduCopter
}

void pdlReadOpticalFlow(pdlDroneState *ds)
{
  static uint32_t lastUpdate = 0;
  static float old_roll = 0;
  static float old_pitch = 0;

  if(sensorValid)
  {
    flow.readMotionCount(&ds->opticalFlow.rawX, &ds->opticalFlow.rawY);

    // LP filter measurements (this code snippet is from ESP-drone project
    /*
     #define LP_CONSTANT 0.8f
     flowData.dpixelx = LP_CONSTANT * dpixelx_previous + (1.0f - LP_CONSTANT) * (float)rawx;
     flowData.dpixely = LP_CONSTANT * dpixely_previous + (1.0f - LP_CONSTANT) * (float)rawy;
     dpixelx_previous = flowData.dpixelx;
     dpixely_previous = flowData.dpixely;
    */

    /*
     * Optical flow raw data should be compensated corresponding pitch-roll angles and height
     * read https://ardupilot.org/copter/docs/common-mouse-based-optical-flow-sensor-adns3080.html
     */

    float h = ds->lidarRange;

    if(h > 0.08 && h < pdlGetLidarMaxRange()) // pmw3901 works above 80mm
    {
      float dt = (float)pdlGetDeltaTime(pdlMicros(),lastUpdate) / 1000000.f;
      lastUpdate = pdlMicros();
      // tilt compensation
      float ex = (ds->roll - old_roll) * k2;
      float ey = (ds->pitch - old_pitch) * k2;
      old_roll = ds->roll;
      old_pitch = ds->pitch;

      float x = ds->opticalFlow.rawX;
      float y = ds->opticalFlow.rawY;

      x = ( ((x + ex) * h) / k1 ) * k3; // +ex caused by roll sign
      y = ( ((y - ey) * h) / k1 ) * k3;

      ds->velocity[PDL_X] = x / dt;
      ds->velocity[PDL_Y] = -y / dt;   //-y caused by pitch sign
    }
    else
    {
      ds->velocity[PDL_X] = 0;
      ds->velocity[PDL_Y] = 0;
    }
  }
  else
  {
    ds->opticalFlow.rawX = 0;
    ds->opticalFlow.rawY = 0;
    ds->velocity[PDL_X] = 0;
    ds->velocity[PDL_Y] = 0;
  }
}
