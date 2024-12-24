#include "pdl.h"
#include "main.h"

#ifdef CICADA_MICRO
  #include "cam_cicada_micro.h"
#elif ARDUINO_XIAO_ESP32S3
  #include "cam_xiao_esp32s3.h"
#endif

#if defined ARDUINO_XIAO_ESP32S3 && defined FAST_ESP32_CAMERA
  #include "ll_cam.h" // cam_obj_t defination, used in camera_data_available
  IRAM_ATTR size_t camera_data_available(void * cam_obj,const uint8_t* data, size_t count, bool last);
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
#endif

#ifdef ARDUINO_XIAO_ESP32S3
  // Normal esp32-camera driver https://github.com/espressif/esp32-camera/tree/master
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
  config.fb_location = CAMERA_FB_IN_PSRAM; // My investigation says there is no difference in frame delay if I change it to DRAM
#endif

#if defined ARDUINO_XIAO_ESP32S3 && defined FAST_ESP32_CAMERA
  // Modified esp32-camera driver from https://github.com/RomanLut/hx-esp32-cam-fpv/tree/master/components/esp32-camera
#ifdef SENSOR_OV5640
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_P_HD; // changed to 800x456 inside driver core
    //config.frame_size = FRAMESIZE_P_FHD; // changed to 1024x576 inside driver core
#else
    config.xclk_freq_hz = 12000000;  //real frequency will be 80Mhz/6 = 13,333Mhz and we use clk2x
    config.frame_size = FRAMESIZE_SVGA;
#endif
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.data_available_callback = camera_data_available;
#endif

#ifdef ARDUINO_XIAO_ESP32S3
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

#if defined FAST_ESP32_CAMERA && defined SENSOR_OV5640
      if(config.frame_size == FRAMESIZE_P_HD) // set FPS 50 for 800x456 of OV5640
      {
        pCamSens->set_colorbar(pCamSens,1);
      }
#endif

    }
    else
    {
      LOG_INFO("esp32-camera is not found");
    }

    if(err == ESP_OK && !hCamTask)
    {
      xTaskCreatePinnedToCore(camTask,"camTask",4096,ds,2,&hCamTask,0);
    }
  }
#endif

#ifdef CICADA_MICRO
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

void updateCamPitchServo(pdlDroneState *ds)
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
#if defined CICADA_MICRO
  updateCamPitchServo(ds);
#endif
}

#if defined ARDUINO_XIAO_ESP32S3 && !defined FAST_ESP32_CAMERA

void camTask(void* pvParameters)
{
  pdlDroneState *ds = (pdlDroneState*)pvParameters;

  bool updStat = false;
  uint32_t fbTimestamp;
  uint64_t frameTime;
  uint32_t frameTimestamp;
  uint16_t frameHeight;
  uint32_t frameDelay;
  size_t maxFrameSize;
  uint32_t fps;
  uint32_t minFps;
  uint32_t maxFps;
  uint32_t avgFps;
  uint32_t fpsTimestamp;

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
      frameHeight = fb->height;
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
        chunkNum++;

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
          if(frameHeight <= 296)
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
          else if(frameHeight <= 600)
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
    // give other processes some cpu time
    delay(1);
  }
}

#endif


#if defined ARDUINO_XIAO_ESP32S3 && defined FAST_ESP32_CAMERA
/*
'bool last' does not mark last JPEG block reliably.

ov2640: if VSYNC interrupt occurs near the end of the last block, we receive another 1Kb block of garbage.
Note that ov2460 sends up zero bytes after FF D9 till 16 byte boundary, then a garbage to the end of the block,
so if we receive extra block, the end of proper 'last' block is filled by zeros after FF D9

ov5640: we always receive up to 3 more zero 1kb blocks (uncertain: can we receive +7 blocks if VSYNK interrupt occurs at the end of 4th block?)
The end of proper 'last' block is filled by zeros after FF D9 (up to 1 kb)

There are two problems:
1) As we may receive garbage block, searching for the end marker in garbage may lead to incorrect JPEG size calculation and inclusion of garbage bytes
  (possibly with misleading JPEG markers) in the stream.
2) with 0v5640 we waste bandwidth for 1..3 1kb blocks, it's 10% video bandwidth wasted!

Proper solution - jpeg chunks parsing - is possible but waste of CPU resources, because jpeg has to be processed byte-by-byte.

We apply very fast workaround instead.

We always zero first 16 bytes in the DMA buffer after processing.

ov2640: instead of receiving garbage block, we receive block with 16 zeros at start. This way we know that the whole buffer has to be skipped;
jpeg is terminated already in the previous block. We waste only few bytes at the end of the block, which are filled by zeros.

ov5640: we are able to finish frame at the first completely zero block,
and even on the block wich has 16 zeros at end
*/

#define MAX_VIDEO_DATA_PAYLOAD_SIZE   1300

static bool s_video_frame_started = false;
static int64_t s_video_last_sent_tp = esp_timer_get_time();
static int64_t s_video_target_frame_dt = 0; // s_video_target_frame_dt = 1000000 / src.camera.fps_limit;
static uint8_t s_video_part_index = 0;
static size_t s_video_full_frame_size = 0;
static bool s_lastByte_ff = false;
static size_t s_maxFrameSize = 0;

static xQueueHandle qVideoChunks = NULL;

struct VideoFrameChunk
{
  uint16_t chunkNum;
  uint16_t width;
  uint16_t height;
  size_t length;
  uint64_t frameTime;
  uint8_t format;
  uint8_t quality;
  uint16_t chunkDataSize;
  void* chunkData;
  bool last;
};

IRAM_ATTR size_t camera_data_available(void * cam_obj,const uint8_t* data, size_t count, bool last)
{
  cam_obj_t *pCamObj = ((cam_obj_t *)cam_obj);
  size_t stride = pCamObj->dma_bytes_per_item;

  /*if(getOVFFlagAndReset())
  {
    s_quality_framesize_K3 = 0.05;
    cam_ovf_count++;
    s_stats.video_frames_expected++;
    applyAdaptiveQuality();
  }*/

  if(data == nullptr) //start frame
  {
    if(!s_video_frame_started)
    {
      //limit fps
      int64_t n = esp_timer_get_time();
      int64_t send_dt = n - s_video_last_sent_tp;
      if(send_dt >= s_video_target_frame_dt)
      {
        s_video_last_sent_tp = n;
        s_video_frame_started = true;
        s_video_part_index = 0;
        s_video_full_frame_size = 0;
        s_lastByte_ff = false;
      }
    }
  }
  else
  {
    //ESP32S3 - sample offset: 0, stride: 1
    const uint8_t* src = (const uint8_t*)data;
    uint8_t* clrSrc = (uint8_t*)src;

    if(s_video_frame_started && (s_video_full_frame_size == 0))
    {
      if(src[0] != 0xff || src[stride] != 0xd8)
      {
        //broken frame data, no start marker
        //we probably missed the start of the frame. Have to skip it.
        s_video_frame_started = false;
      }
    }

    if(s_video_frame_started)
    {
      count /= stride;
      // check if we missed last block due to VSYNK near the end of prev (last) block
      // if block starts with 16 zero bytes - it is either not filled at all (ov2640)
      // or is fully filled with zeros(ov5640)
      const uint8_t* src1 = src;
      int i;
      for(i = 0; i < 16; i++)
      {
        if(*src1 != 0)
        {
          break;
        }
        src1 += stride;
      }

      if(i == 16)
      {
        // we missed last block, this is next block after last
        // the whole block should be skipped
        last = true;
        count = 0;
      }
      else
      {
        //ov5640: check if 16 bytes at the end of the block are zero
        const uint32_t* pTail = (const uint32_t*)(&(src[count - 16]));
        if((pTail[0] == 0 ) && (pTail[1] == 0) && (pTail[2] == 0) && (pTail[3] == 0 ))
        {
          //zeros at the end - block has to contain end marker
          last = true;
        }

        if(last) //find the end marker for JPEG. Data after that can be discarded
        {
          //edge case - 0xFF at the end of prev block, 0xD9 on the start of this
          if(s_lastByte_ff && (*src == 0xD9))
          {
            count = 1;
          }
          else
          {
            //search from the start of the block
            //tail of the block can contain end markers in garbage
            const uint8_t* dptr = src;
            const uint8_t* dptrEnd = src + (count - 2) * stride;
            while(dptr <= dptrEnd)
            {
              if(dptr[0] == 0xFF && dptr[stride] == 0xD9)
              {
                count = (dptr - src) / stride + 2; //to include the 0xFFD9
                if((count & 0x1FF) == 0)
                {
                  count += 1;
                }
                if((count % 100) == 0)
                {
                  count += 1;
                }
                break;
              }
              dptr += stride;
            }
          }
        }
      }

      s_lastByte_ff = count > 0 ? src[count-1] == 0xFF : false;

      while(count > 0)
      {
        size_t c = std::min((size_t)MAX_VIDEO_DATA_PAYLOAD_SIZE,count);
        count -= c;
        s_video_full_frame_size += c;

        if(c > 0)
        {
          VideoFrameChunk chunk;

          if(last && count == 0)
          {
            chunk.last = true;
          }
          else
          {
            chunk.last = false;
          }

          if(s_video_part_index == 0)
          {
            chunk.frameTime = pdlSystemTimeToHostTime(s_video_last_sent_tp);
          }

          chunk.format = PIXFORMAT_JPEG;
          chunk.width = pCamObj->width;
          chunk.height = pCamObj->height;
          chunk.quality = jpegQuality;
          chunk.length = 0; // it's not possible to determine the size of video frame here
          chunk.chunkDataSize = c;
          chunk.chunkData = heap_caps_malloc(c,MALLOC_CAP_8BIT);
          chunk.chunkNum = s_video_part_index;
          if(chunk.chunkData)
          {
            memcpy(chunk.chunkData,src,c);
            // Can't send data from here function because it throws stack overflow
            // Put data to FreeRTOS Queue and send from camTask thread
            if(qVideoChunks)
            {
              if(xQueueSend(qVideoChunks,(void*)&chunk,1) != pdTRUE)
              {
                heap_caps_free(chunk.chunkData);
                LOG_DEBUG("The video queue is full");
              }
            }
          }
          else
          {
            LOG_ERROR("Can't allocate memory for video chunk, free heap=%i", heap_caps_get_free_size(MALLOC_CAP_8BIT));
          }

          src += c;
          s_video_part_index++;
        }
      }  //while count>0
    }  //s_frame_started

    //////////////////

    if(last)  //note: can occur multiple times during frame
    {
      //end of frame - send leftover
      if(s_video_frame_started)
      {
        s_video_frame_started = false;
        //end of frame - stats, camera settings, osd, mavlink
        if(s_video_full_frame_size > 0)
        {
          if(s_video_full_frame_size > s_maxFrameSize)
          {
            s_maxFrameSize = s_video_full_frame_size;
          }

        }
      }
    }

    //zero start of the DMA block
    for(int i = 0; i < 16; i++)
    {
      *clrSrc = 0;
      clrSrc += stride;
    }
  }

  return count;
}

void camTask(void* pvParameters)
{
  pdlDroneState *ds = (pdlDroneState*)pvParameters;
  qVideoChunks = xQueueCreate(128, sizeof(VideoFrameChunk));
  VideoFrameChunk chunk;
  bool updStat = false;
  sensor_t *pCamSens = esp_camera_sensor_get();
  camera_sensor_info_t *pCamInfo = NULL;
  float fpsDt = 0;
  uint16_t fps;
  uint16_t fpsCounter = 0;
  uint16_t avgFps = 0;
  uint16_t minFps = 100;
  uint16_t maxFps = 0;
  uint32_t fpsTimestamp = 0;
  size_t sent;
  bool sentOk = true;

  if(pCamSens)
  {
    pCamInfo = esp_camera_sensor_get_info(&pCamSens->id);
  }

  for(;;)
  {
    xQueueReceive(qVideoChunks, &chunk, portMAX_DELAY);
    if(chunk.chunkNum == 0)
    {
      sentOk = true;
      sent = sendWlanPictureStartPacketV2(   chunk.width,
                                      chunk.height,
                                      chunk.format,
                                      chunk.quality,
                                      chunk.frameTime,
                                      chunk.length,
                                      (uint8_t*)chunk.chunkData,
                                      chunk.chunkDataSize);
      if(!sent)
      {
        sentOk = false;
      }
    }
    else if(chunk.last == false)
    {
      sent = sendWlanPictureDataPacketV2((uint8_t*)chunk.chunkData,chunk.chunkDataSize,chunk.chunkNum);
      if(!sent)
      {
        sentOk = false;
      }
    }
    else
    {
      sent = sendWlanPictureLastDataPacketV2((uint8_t*)chunk.chunkData,chunk.chunkDataSize,chunk.chunkNum);
      if(!sent)
      {
        sentOk = false;
      }

      if(sentOk)
      {
        fpsCounter++;
      }
    }

    if(chunk.chunkData)
    {
      heap_caps_free(chunk.chunkData);
      chunk.chunkData = NULL;
    }

    // statistic
    if(pdlGetDeltaTime(pdlMicros(),fpsTimestamp) >= 1000000)
    {
      fpsDt = pdlGetDeltaTime(pdlMicros(),fpsTimestamp);
      fpsTimestamp = pdlMicros();
      fps = (uint16_t)(((float)fpsCounter)*1000000.f/fpsDt);
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

      /*
      // apply adaptive quality
      if(hostIsSet() && pCamSens && pCamInfo)
      {
        if(avgFps < 25 && jpegQuality < 63)
        {
          jpegQuality++;
          pCamSens->set_quality(pCamSens,jpegQuality);
        }
        else if(avgFps >= 28 && jpegQuality > 8)
        {
          jpegQuality--;
          pCamSens->set_quality(pCamSens,jpegQuality);
        }
      }
      */

      //LOG_INFO("esp_cam: maxFrameSize=%i,minFps=%i,fps=%i,maxFps=%i,delay=%i",maxFrameSize,minFpv,fpv,maxFpv,frameDelay);

      fpsCounter = 0;

      if(updStat)
      {
        LOG_INFO("esp_cam: maxFrameSize=%i,minFps=%i,avgFps=%i,maxFps=%i",s_maxFrameSize,minFps,avgFps,maxFps);
        updStat = false;
      }
    }
  }
}

#endif
