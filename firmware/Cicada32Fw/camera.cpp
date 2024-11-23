#include "pdl.h"
#include "main.h"

#ifdef CICADA_MICRO
  #include "cam_cicada_micro.h"
#elif ARDUINO_XIAO_ESP32S3
  #include "cam_xiao_esp32s3.h"
#endif

// Any servo with PWM 500-2400us is supported and 0-180 degree position

bool servOk = false;
int32_t angleInMicroSecs;
uint32_t oldMicros;
int32_t targetAngle = 0;
const int32_t firstAngle = 30;

void pdlSetupCamera(pdlDroneState *ds)
{
  angleInMicroSecs = 0;
  oldMicros = pdlMicros();

#ifdef CICADA_MICRO
  if(!runcam.init())
  {
    LOG_INFO("Runcam is not available");
  }

  if(ds->esc >= 20 && ds->esc <= 30)
  {
    servOk = false;
    LOG_INFO("Camera servo is not available");
    return;
  }

  pinMode(CAM_SERVO_PIN,OUTPUT);
  digitalWrite(CAM_SERVO_PIN,LOW);

  servOk = true;

  LOG_INFO("Camera servo is ok");
#elif ARDUINO_XIAO_ESP32S3
  // ESP camera
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
#ifdef SENSOR_OV5640
  config.xclk_freq_hz = 20000000;
#else
  config.xclk_freq_hz = 12000000; //real frequency will be 80Mhz/6 = 13,333Mhz and we use clk2x
#endif
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 36;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_DRAM;

  //config.data_available_callback = camera_data_available;

  // camera init
  esp_err_t err = esp_camera_init(&config);

  if(err != ESP_OK)
  {
    LOG_ERROR("ESP32S3 camera init failed with error 0x%x", err);
  }
  else
  {
    LOG_INFO("ESP32S3 camera is ok");

    if(!hCamTask)
    {
      xTaskCreatePinnedToCore(camTask,"camTask",2048,ds,5,&hCamTask,0);
    }
  }

  pinMode(CAM_SERVO_PIN, OUTPUT);
  digitalWrite(CAM_SERVO_PIN, LOW);

  servOk = true;

  LOG_INFO("Camera servo is ok");
#endif
}

void pdlSetCameraAngle(pdlDroneState *ds, int32_t angle)
{
  (void)ds;
  targetAngle = angle;
  //angle = constrain(angle,0,90);
  //angleInMicroSecs = map(angle,0,90,900,2100);
}

void pdlStartVideo(pdlDroneState *ds)
{
#ifdef CICADA_MICRO
  if(runcam.isAvailable() == false)
    return;

  LOG_INFO("Start video");

  ds->videoState = 1;

  //runcam.reqStartVideo(); //this command has no effect for runcam split4
  runcam.reqSimulatePwrBtn();
#else
  LOG_INFO("Start video");
  ds->videoState = 1;
#endif
}

void pdlStopVideo(pdlDroneState *ds)
{
#ifdef CICADA_MICRO
  if(runcam.isAvailable() == false)
    return;

  LOG_INFO("Stop video");

  ds->videoState = 0;

  //runcam.reqStopVideo();
  runcam.reqSimulatePwrBtn();
#else
  LOG_INFO("Stop video");
  ds->videoState = 0;
#endif
}

void pdlTakePhoto(pdlDroneState *ds)
{
  (void)ds;
  LOG_INFO("Take photo");
}

void updateServo(pdlDroneState *ds)
{
  if(!hostIsSet() || !servOk)
    return;

  // Simulate PWM-50Hz
  uint32_t period = pdlGetDeltaTime(pdlMicros(), oldMicros);
  if(period < 20000)
    return;

  oldMicros = pdlMicros();

  int32_t angle = firstAngle + targetAngle + (int32_t)(ds->pose[PDL_PITCH].pos * RAD_TO_DEG);
  angle = constrain(angle, 0, 140);
  angleInMicroSecs = map(angle, 0, 180, 500, 2400);

  if(angleInMicroSecs > 0)
  {
    digitalWrite(CAM_SERVO_PIN, HIGH);
    delayMicroseconds(angleInMicroSecs);
    digitalWrite(CAM_SERVO_PIN, LOW);
  }
}

void pdlUpdateCamera(pdlDroneState *ds)
{
#if defined CICADA_MICRO || defined ARDUINO_XIAO_ESP32S3
  updateServo(ds);
#endif
}

#ifdef ARDUINO_XIAO_ESP32S3

void camTask(void* pvParameters)
{
  pdlDroneState *ds = (pdlDroneState*)pvParameters;
  for(;;)
  {
    delay(1000);
    // get camera buffer
    // check frame timestamp
    // send jpeg in loop
  }
}

#endif
