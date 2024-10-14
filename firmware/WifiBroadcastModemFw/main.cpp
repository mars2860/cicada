#include "WifiBroadcastModem.h"
#include <string.h>

#define LOG_BUF_SIZE  1024

char logBuf[LOG_BUF_SIZE];
int logPos = 0;

int log_printf(const char *format, ... )
{
  if(logPos >= LOG_BUF_SIZE)
    return 0;

  va_list args;
  va_start(args, format);
  int res = vsnprintf(&logBuf[logPos],LOG_BUF_SIZE - logPos,format,args);
  va_end(args);

  if(res > 0)
    logPos += res;

  if(logPos > LOG_BUF_SIZE)
    logPos = LOG_BUF_SIZE;

  return res;
}

char* getLog()
{
  return &logBuf[0];
}

int getLogSize()
{
  return logPos;
}

void resetLog()
{
  memset(&logBuf[0],0,LOG_BUF_SIZE);
  logPos = 0;
}

void setup()
{
  Serial.begin(921600);
  wbmSetup();
  log_printf("Started, fw.v1");
}

void loop()
{
  wbmUpdateSerial();

  if(getLogSize() > 0)
  {
    uint16_t written = wbmSendDataToHost((uint8_t*)getLog(),getLogSize(),1);
    if(written > 0)
    {
      resetLog();
    }
  }
}
