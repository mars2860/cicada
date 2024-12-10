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

#ifdef ARDUINO_XIAO_ESP32S3
uint8_t jpegQuality;
#endif

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

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG;

  jpegQuality = 8; // if I set start quality to 0 I get "Stack canary watchpoint triggered (camTask)"
  // jpegQuality = 20;

  // FPS=25 for OV2640
  //config.frame_size = FRAMESIZE_SVGA;
  //config.frame_size = FRAMESIZE_VGA;
  // FPS=50 for OV2640
  config.frame_size = FRAMESIZE_CIF;

  config.jpeg_quality = jpegQuality;
  config.fb_count = 3;
  config.grab_mode = CAMERA_GRAB_LATEST;
  //config.fb_location = CAMERA_FB_IN_DRAM;
  config.fb_location = CAMERA_FB_IN_PSRAM; // My investigation says that is no difference in frame delay if I change it to DRAM

  //config.data_available_callback = camera_data_available;

  // camera init
  esp_err_t err = esp_camera_init(&config);

  if(err != ESP_OK)
  {
    LOG_ERROR("esp32-camera init failed with error 0x%x", err);
  }
  else
  {
    sensor_t *pCamSens = esp_camera_sensor_get();

    if(pCamSens)
    {
      camera_sensor_info_t *pCamInfo = esp_camera_sensor_get_info(&pCamSens->id);
      if(pCamInfo)
      {
        LOG_INFO("esp32-camera:%s is ok",pCamInfo->name);
      }
      else
      {
        LOG_ERROR("Unknown esp32-camera");
      }
    }
    else
    {
      LOG_INFO("esp32-camera is not found");
    }

    if(!hCamTask)
    {
      xTaskCreatePinnedToCore(camTask,"camTask",4096,ds,1,&hCamTask,0);
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

  bool updStat = false;
  uint32_t fbTimestamp;
  uint64_t frameTime;
  uint32_t frameTimestamp;
  size_t maxFrameSize;
  uint32_t fps;
  uint32_t minFps;
  uint32_t maxFps;
  uint32_t avgFps;
  uint32_t fpsTimestamp;
  uint32_t frameDelay;
  size_t sent;
  size_t dataOffset;
  uint16_t chunkNum;

  maxFrameSize = 0;
  minFps = 100;
  maxFps = 0;
  avgFps = 0;
  fps = 0;
  frameDelay = 0;
  frameTimestamp = pdlMicros();
  fpsTimestamp = pdlMicros();

  sensor_t *pCamSens = esp_camera_sensor_get();
  camera_sensor_info_t *pCamInfo = NULL;

  if(pCamSens)
  {
     pCamInfo = esp_camera_sensor_get_info(&pCamSens->id);
  }

  for(;;)
  {
    // get camera buffer
    camera_fb_t* fb = esp_camera_fb_get();

    if(fb)
    {
      chunkNum = 0;
      fbTimestamp = fb->timestamp.tv_sec*1000000UL + fb->timestamp.tv_usec;
    }

    if(fb && fbTimestamp != frameTimestamp)
    {
      frameTimestamp = fbTimestamp;
      frameDelay = pdlGetDeltaTime(pdlMicros(),frameTimestamp);
      // convert frame timestamp to host time
      frameTime = pdlSystemTimeToHostTime(frameTimestamp);
      // log frame size
      if(fb->len > maxFrameSize)
      {
        maxFrameSize = fb->len;
        updStat = true;
      }
      // send jpeg to host
      if(hostIsSet())
      {
        sent = sendWlanPictureStartPacket( fb->width,
                                           fb->height,
                                           fb->format,
                                           jpegQuality,
                                           frameTime,
                                           fb->len,
                                           fb->buf);

        dataOffset = sent;

        while(dataOffset < fb->len && sent > 0)
        {
          // on ESP32 platforme endPacket says err 12 (ENOMEM) if I send packets very often
          // forums say increase CONFIG_ESP_WIFI_STATIC_TX_BUFFER_NUM
          // but increasing this param in sdkconfig.h in esp-idf placed in Arduino folder has no effects
          // also see parameters of esp-idf for max throughput https://github.com/espressif/esp-idf/tree/v5.2.3/examples/wifi/iperf
          // to change CONFIG_ESP_WIFI_STATIC_TX_BUFFER_NUM it is need to recompile the whole arduino-esp32-lib
          // to do this see https://github.com/espressif/esp32-arduino-lib-builder
          // upd: rebuilding of arduino-lib gave nothing, bitrate is low
          for(uint8_t i = 0; i < 5; i++)
          {
            sent = sendWlanPictureDataPacket(dataOffset,fb->len,fb->buf,chunkNum);
            if(sent > 0)
            {
              chunkNum++;
              break;
            }
            else
            {
              // the only recommended method to fight with err 12 (ENOMEM) it is delay and send again
              // https://esp32.com/viewtopic.php?t=5340
              delay(3);
            }
          }
          dataOffset += sent;
        }

        //if(!sent)
        //{
        //  LOG_ERROR("Can't sent a picture chunk");
        //}

        if(dataOffset >= fb->len) // count fps if picture has been sent successfully
        {
          fps++;
        }
      }
      else
      {
        // count offline fps
        fps++;
      }
    }
    // return buffer
    if(fb)
    {
      esp_camera_fb_return(fb);
    }

    // statistics
    if(pdlGetDeltaTime(pdlMicros(),fpsTimestamp) >= 1000000)
    {
      fpsTimestamp = pdlMicros();
      if(fps < minFps)
      {
        minFps = fps;
        updStat = true;
      }
      if(fps > maxFps)
      {
        maxFps = fps;
        updStat = true;
      }

      uint32_t f = (avgFps + fps) / 2;

      if(avgFps != f)
      {
        avgFps = f;
        updStat = true;
      }

      //avgFps = (avgFps + fps) / 2;

      // apply adaptive quality for OV2640
      if(hostIsSet() && pCamSens && pCamInfo)
      {
        if(pCamInfo->model == CAMERA_OV2640)
        {
          if(fb->height <= 296)
          {
            if(avgFps < 30 && jpegQuality < 63)
            {
              jpegQuality++;
              pCamSens->set_quality(pCamSens,jpegQuality);
            }
            else if(avgFps >= 45 && jpegQuality > 8)
            {
              jpegQuality--;
              pCamSens->set_quality(pCamSens,jpegQuality);
            }
          }
          else if(fb->height <= 600)
          {
            if(avgFps < 22 && jpegQuality < 63)
            {
              jpegQuality++;
              pCamSens->set_quality(pCamSens,jpegQuality);
            }
            else if(avgFps >= 24 && jpegQuality > 8)
            {
              jpegQuality--;
              pCamSens->set_quality(pCamSens,jpegQuality);
            }
          }
        }
      }

      //LOG_INFO("esp_cam: maxFrameSize=%i,minFps=%i,fps=%i,maxFps=%i,delay=%i",maxFrameSize,minFpv,fpv,maxFpv,frameDelay);

      fps = 0;
    }
    if(updStat)
    {
      // about FPS of esp32-camera https://github.com/espressif/esp32-camera/issues/201
      LOG_INFO("esp_cam: maxFrameSize=%i,minFps=%i,avgFps=%i,maxFps=%i,delay=%i",maxFrameSize,minFps,avgFps,maxFps,frameDelay);
      updStat = false;
    }
  }
}

#endif
