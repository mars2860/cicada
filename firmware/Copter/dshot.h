#ifndef DSHOT_H
#define DSHOT_H

#include <arduino.h>
#include <stdint.h>

/**
 * @param pin1
 * @param pin2
 * @param pin3
 * @param pin4
 * @param timeGap time gap between frames in microseconds. This function automatically setups timer1 if this value greater 0.
 * If you use timer1 somewhere else you need to set timeGap as 0 in this function and call qWriteDshot300 in your
 * timer1 interrupt routine with needed timeGap
 */
void dshotSetup(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint32_t timeGap);

/** Sets new values will be transmitted each loop
 *
 * Values 0-47 are reserved
 *
 * BLHeli_S codes
 *
 * 0  DSHOT_CMD_MOTOR_STOP,   // Currently not implemented
 * 1  DSHOT_CMD_BEEP1,     // Wait at least length of beep (380ms) before next command
 * 2  DSHOT_CMD_BEEP2,     // Wait at least length of beep (380ms) before next command
 * 3  DSHOT_CMD_BEEP3,     // Wait at least length of beep (400ms) before next command
 * 4  DSHOT_CMD_BEEP4,     // Wait at least length of beep (400ms) before next command
 * 5  DSHOT_CMD_BEEP5,     // Wait at least length of beep (400ms) before next command
 * 6  DSHOT_CMD_ESC_INFO,      // Currently not implemented
 * 7  DSHOT_CMD_SPIN_DIRECTION_1,    // Need 6x, no wait required
 * 8  DSHOT_CMD_SPIN_DIRECTION_2,    // Need 6x, no wait required
 * 9  DSHOT_CMD_3D_MODE_OFF,   // Need 6x, no wait required
 * 10 DSHOT_CMD_3D_MODE_ON,    // Need 6x, no wait required
 * 11 DSHOT_CMD_SETTINGS_REQUEST,    // Currently not implemented
 * 12 DSHOT_CMD_SAVE_SETTINGS,     // Need 6x, wait at least 12ms before next command
 * 20 DSHOT_CMD_SPIN_DIRECTION_NORMAL, // Need 6x, no wait required
 * 21 DSHOT_CMD_SPIN_DIRECTION_REVERSED, // Need 6x, no wait required
 * 22 DSHOT_CMD_LED0_ON,     // Currently not implemented
 * 23 DSHOT_CMD_LED1_ON,     // Currently not implemented
 * 24 DSHOT_CMD_LED2_ON,     // Currently not implemented
 * 25 DSHOT_CMD_LED3_ON,     // Currently not implemented
 * 26 DSHOT_CMD_LED0_OFF,      // Currently not implemented
 * 27 DSHOT_CMD_LED1_OFF,      // Currently not implemented
 * 28 DSHOT_CMD_LED2_OFF,      // Currently not implemented
 * 29 DSHOT_CMD_LED3_OFF,      // Currently not implemented
 *
 * @param value1
 * @param value2
 * @param value3
 * @param value4
 *
 */
void dshotSet(uint16_t value1, uint16_t value2, uint16_t value3, uint16_t value4);

/** Writes 4 new values to escaper using DSHOT300 protocol. Takes 108 us to send all data */
void ICACHE_RAM_ATTR dshotWrite150();
/** Takes 54 us to send all data */
void ICACHE_RAM_ATTR dshotWrite300();
/** Takes 27 us to send all data */
//void ICACHE_RAM_ATTR dshotWrite600();

void dshotEnable(uint8_t enable);

#endif
