#include "pdl.h"
#include "Bitcraze_PMW3901.h"
#include "movingAvg.h"

// Warn: for CS used TXD pin!
Bitcraze_PMW3901 flow(1);
boolean sensorValid;

/*float mini_flow_x_i, mini_flow_y_i;
float flow_fix_x_i, flow_fix_y_i;
float flow_ang_x, flow_ang_y;
float out_x, out_y, old_out_x, old_out_y;
float pix_flow_fix_x, pix_flow_fix_y;
uint32_t lastTime;*/

void pdlSetupOpticalFlow(pdlDroneState*)
{
  delay(500);

  sensorValid = flow.begin();
  pdlSetOpticalFlowReadPeriod(100000);  // � ArduCopter ������ ������ ����� 100��

  /*mini_flow_x_i = 0;
  mini_flow_y_i = 0;
  flow_fix_x_i = 0;
  flow_fix_y_i = 0;
  flow_ang_x = 0;
  flow_ang_y = 0;
  out_x = 0;
  out_y = 0;
  old_out_x = 0;
  old_out_y = 0;
  lastTime = 0;*/
}

#define LP_CONSTANT 0.8f

void pdlReadOpticalFlow(pdlDroneState *ds)
{
  static uint32_t lastUpdate = 0;
  static float old_roll = 0;
  static float old_pitch = 0;
  const float fov = 42.f * 3.14f / 180.f; // field of view
  const float scaler = 12.5f; // pixels per mm (experimental determined)
  const float Npx = 30.f; // sensor resolution

  if(sensorValid)
  {
    flow.readMotionCount(&ds->opticalFlow.rawX, &ds->opticalFlow.rawY);

    // Use LP filter measurements
    /*
     flowData.dpixelx = LP_CONSTANT * dpixelx_previous + (1.0f - LP_CONSTANT) * (float)rawx;
     flowData.dpixely = LP_CONSTANT * dpixely_previous + (1.0f - LP_CONSTANT) * (float)rawy;
     dpixelx_previous = flowData.dpixelx;
     dpixely_previous = flowData.dpixely;
     */

    /*
     * � ���� ������������� ����������, ��� ��������� ����������� ������� �������� ������� �� ������
     * � ��� ����������� ���������. ����� �� ������ ������������� ���� �������� ��������� ������ ����
     * ����������� (���������). ����� ��������� ��� ������, �� �������� ���������� ������ �� ������� � ������, �������
     * ������ ���� ���������������� �� ������ 1,5�. ���� �� ����� ���������� ������ ��� ���������, ��
     * ��� ����������� ����� ������ �������� ������ �� ����� ������
     */

    /*
     * ������ �������� � ���, ��� ��� ������� ����� ��� �������, optical flow ����� ����� ����������
     * ��������, ����� � ��� ����� ������ ����� ������ �� ����� �����
     */

    float h = ds->lidarRange;

    // ��� �� crazyflie - ��� ������� ������� ����� ��������� � ������������
    /*if(h > 0.08f)
    {
      float dt = (float)pdlGetDeltaTime(pdlMicros(),lastUpdate) / 1000000.f;
      lastUpdate = pdlMicros();
      // ������ ������� � ��������
      float Npix = 30.f; // [pixels] (same in x and y)
      //float thetapix = DEG_TO_RAD * 4.0f;     // [rad]    (same in x and y)
      // �������� ��������� ������ (�������)
      float thetapix = 42.f * 3.14f / 180.f;
      float nx = ds->opticalFlow.rawX;
      float ny = ds->opticalFlow.rawY;

      ds->opticalFlow.vx = ((h * thetapix * nx) / (dt * Npix)) + (h*ds->gyro.pure[PDL_X]); // ���� + � ������������ � ������������ X
      ds->opticalFlow.vy = ((h * thetapix * ny) / (dt * Npix)) - (h*ds->gyro.pure[PDL_Y]);
    }
    else
    {
      ds->opticalFlow.vx = 0;
      ds->opticalFlow.vy = 0;
    }
    */

    //if(h > 0.08f)
    if(h > 0)
    {
      float dt = (float)pdlGetDeltaTime(pdlMicros(),lastUpdate) / 1000000.f;
      lastUpdate = pdlMicros();
      // tilt compensation
      float ex = (ds->roll - old_roll) * Npx * scaler / fov;
      float ey = (ds->pitch - old_pitch) * Npx * scaler / fov;
      old_roll = ds->roll;
      old_pitch = ds->pitch;

      float x = ds->opticalFlow.rawX;
      float y = ds->opticalFlow.rawY;

      x = ( ((x + ex) * h) / (Npx * scaler) ) * 2.f * tanf(fov / 2.f);
      y = ( ((y - ey) * h) / (Npx * scaler) ) * 2.f * tanf(fov / 2.f);

      ds->opticalFlow.vx = x / dt;
      ds->opticalFlow.vy = -y / dt;
    }
    else
    {
      ds->opticalFlow.vx = 0;
      ds->opticalFlow.vy = 0;
    }

    // ��� � ���-��������. ��� � �������� � ������� Cheerson CX-OF
    /* �����
    // �����������
    mini_flow_x_i += (float)ds->opticalFlow.rawX;
    mini_flow_y_i += (float)ds->opticalFlow.rawY;
    // ����� low-pass filter
    flow_fix_x_i += (mini_flow_x_i - flow_fix_x_i)*0.2f;
    flow_fix_y_i += (mini_flow_y_i - flow_fix_y_i)*0.2f;
    // ������������ �� ����� � �������
    flow_ang_x += (600.f*tanf(ds->roll) - flow_ang_x)*0.2f;
    flow_ang_y += (600.f*tanf(ds->pitch) - flow_ang_y)*0.2f;
    // � �������� ���������
    out_x = flow_fix_x_i - flow_ang_x;
    out_y = flow_fix_y_i - flow_ang_y;

    float dt = pdlMicros() - lastTime;
    lastTime = pdlMicros();
    dt /= 1000000.f;

    pix_flow_fix_x = (out_x - old_out_x)/dt;
    pix_flow_fix_y = (out_y - old_out_y)/dt;

    old_out_x = out_x;
    old_out_y = out_y;

    ds->opticalFlow.vx += (pix_flow_fix_x - ds->opticalFlow.vx)*0.1f;
    ds->opticalFlow.vx += (pix_flow_fix_y - ds->opticalFlow.vy)*0.1f;
*/
    //float high = ds->lidarRange;
    //if(ds->lidarRange < 0.004f)

    //float cpi = (high/11.914f)*2.54f;
    /*
    float alt = ds->baroAlt;

    if(ds->lidarRange > -0.1f && ds->lidarRange <= pdlGetLidarMaxRange())
      alt = ds->lidarRange;

    if(alt < 0.004f)
      alt = 0.004f;

    // ������� ����� ���������, �� ���� ���� ���-�� ��������������� ������
    // ����� ������� �������������, ��� ������� � ��������� �������
    // ��� ��� ����� ����������� ������ ������� �� ������,
    // � ��������� ����� ����������� ������ � ������� ������� �� ��������
    // �����������
    // ����� ������� �������������� �������� ������ ��� ������� ��� ����� ���������
    // � ���� 40.f ������� �� ������� ��� ����������� ����� ���������� �� ����������� � �������
    // ������� ����� ����������� C � ����� ������, ������� ������ ���� �� ������ 1.5�
    float a = 40.f*powf(alt,-0.728144332f);
    if(a > 0)
    {
      float c = 40.f * ds->opticalFlowX / a;
      ds->opticalFlowX = c*powf(1.5f,-0.728144332f);
      c = 40.f * ds->opticalFlowY / a;
      ds->opticalFlowY = c*powf(1.5f,-0.728144332f);
    }
    */
  }
}
