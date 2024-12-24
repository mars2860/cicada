#ifndef MAIN_H_
#define MAIN_H_

#include <arduino.h>

bool hostIsSet();

/// @return number of sent picture data bytes
size_t sendWlanPictureStartPacket(  uint16_t width,
                                    uint16_t height,
                                    uint8_t pixformat,
                                    uint8_t quality,
                                    uint64_t timestamp,
                                    size_t dataLen,
                                    uint8_t *data);

/// @return number of sent picture data bytes
size_t sendWlanPictureDataPacket( size_t dataOffset,
                                  size_t dataLen,
                                  uint8_t *data,
                                  uint16_t chunkNum);

size_t sendWlanPictureStartPacketV2(  uint16_t width,
                                      uint16_t height,
                                      uint8_t pixformat,
                                      uint8_t quality,
                                      uint64_t timestamp,
                                      size_t dataLen,
                                      uint8_t *data,
                                      size_t chunkSize);

size_t sendWlanPictureDataPacketV2( uint8_t *data,
                                    size_t chunkSize,
                                    uint16_t chunkNum );

size_t sendWlanPictureLastDataPacketV2( uint8_t *data,
                                        size_t chunkSize,
                                        uint16_t chunkNum );

#endif
