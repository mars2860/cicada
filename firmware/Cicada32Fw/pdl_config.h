#ifndef PDL_CONFIG_H_
#define PDL_CONFIG_H_

#ifdef ARDUINO_ARCH_ESP8266
  #include "pdl_config_esp8266.h"
#elif ARDUINO_ARCH_ESP32
  #include "pdl_config_esp32.h"
#endif

#endif /* PDL_CONFIG_H_ */
