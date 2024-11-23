#ifndef MAIN_H_
#define MAIN_H_

#include <arduino.h>

bool hostIsSet();
bool sendWlanPacket(uint8_t *data, uint16_t data_len);

#endif
