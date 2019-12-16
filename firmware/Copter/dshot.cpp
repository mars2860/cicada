#include "dshot.h"

#define NOP asm volatile("nop")

struct gpio_regs
{
  uint32_t out;         /* 0x60000300 */
  uint32_t out_w1ts;    /* 0x60000304 */
  uint32_t out_w1tc;    /* 0x60000308 */
  uint32_t enable;      /* 0x6000030C */
  uint32_t enable_w1ts; /* 0x60000310 */
  uint32_t enable_w1tc; /* 0x60000314 */
  uint32_t in;          /* 0x60000318 */
  uint32_t status;      /* 0x6000031C */
  uint32_t status_w1ts; /* 0x60000320 */
  uint32_t status_w1tc; /* 0x60000324 */
};

static struct gpio_regs* gpio = (struct gpio_regs*)(0x60000300);

struct gpio16_regs
{
  uint32_t out;     /* 0x60000768 */
};

static struct gpio16_regs* gpio16 = (struct gpio16_regs*)(0x60000768);

static volatile uint8_t pins[4];
static volatile uint8_t enabled;
static volatile uint32_t timeGapTicks;
static volatile uint32_t gpioSetMask;
static volatile uint32_t gpioClearMask0[16];
static volatile uint32_t gpioClearMask1[16];
static volatile uint8_t gpio16Set;
static volatile uint8_t gpio16State0[16];
static volatile uint8_t gpio16State1[16];

void ICACHE_RAM_ATTR onTimerISR()
{
  dshotWrite300();
  timer1_write(timeGapTicks);
}

void dshotEnable(uint8_t enable)
{
  enabled = enable;

  if(enabled && timeGapTicks > 0)
  {
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE); // ticks every 0.2 us
    timer1_write(300);
  }
  else if(!enabled)
  {
    timer1_disable();
  }
}

void dshotSetup(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint32_t timeGap)
{
  pins[0] = pin1;
  pins[1] = pin2;
  pins[2] = pin3;
  pins[3] = pin4;

  gpioSetMask = (1 << pin1) | (1 << pin2) | (1 << pin3) | (1 << pin4);

  if(gpioSetMask & 0x10000)
    gpio16Set = 1;
  else
    gpio16Set = 0;

  dshotSet(48,48,48,48);

  if(timeGap > 0)
  {
    timeGapTicks = (timeGap*10)/2;
    timer1_attachInterrupt(onTimerISR);
    //timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE); // ticks every 0.2 us
    //timer1_write(timeGapTicks);
  }
  else
  {
    timeGapTicks = 0;
  }

  dshotEnable(1);
}

uint16_t createDshotPacket(uint16_t throttle)
{
  uint16_t csum = 0;
  uint16_t csum_data = 0;
  uint16_t packet = throttle << 1;

  // Indicate as command if less than 48
  if(throttle < 48 && throttle > 0)
    packet |= 1;

  csum = 0;
  csum_data = packet;

  for(uint8_t j = 0; j < 3; j++)
  {
    csum ^= csum_data;
    csum_data >>= 4;
  }

  csum &= 0x000F;

  packet = (packet << 4) | csum;

  return packet;
}

void dshotSet(uint16_t value1, uint16_t value2, uint16_t value3, uint16_t value4)
{
  uint8_t i, j;
  uint16_t packet[4];
  uint8_t oldEnabled = enabled;

  packet[0] = createDshotPacket(value1);
  packet[1] = createDshotPacket(value2);
  packet[2] = createDshotPacket(value3);
  packet[3] = createDshotPacket(value4);

  dshotEnable(0);

  for(i = 0; i < 16; i++)
  {
    gpioClearMask0[i] = 0;
    gpioClearMask1[i] = 0;
    for(j = 0; j < 4; j++)
    {
      if(packet[j] & (1 << i))
        gpioClearMask1[i] |= 1 << pins[j];
      else
        gpioClearMask0[i] |= 1 << pins[j];
    }

    if(gpioClearMask0[i] & 0x10000)
    {
      gpio16State0[i] = 0;
      gpio16State1[i] = 0;
    }
    else if(gpioClearMask1[i] & 0x10000)
    {
      gpio16State0[i] = 1;
      gpio16State1[i] = 0;
    }
  }

  dshotEnable(oldEnabled);
}

void ICACHE_RAM_ATTR dshotWrite150()
{
  volatile uint8_t i;
  volatile uint8_t j;

  for(i = 16; i > 0; i--)
  {
    // force write to GPIO registers on each loop
    asm volatile ("" : : : "memory");

    gpio->out_w1ts |= gpioSetMask;
    //if(gpioSetMask & 0x10000)
    //  gpio16->out = 1;
    gpio16->out = gpio16Set;

    // delay 2500 ns to reset pins to send 0 bit
    j = 11;
    do
    {
      j -= 1;
      // stop compiler from optimizing delay loop to noop
      NOP;
    }
    while(j > 0);

    // end 0f T0H
    gpio->out_w1tc |= gpioClearMask0[i - 1];
    //if(gpioClearMask0[i - 1] & 0x10000)
    //  gpio16->out = 0;
    gpio16->out = gpio16State0[i - 1];

    // delay 2500 ns to reset pins to send 1 bit
    j = 11;
    do
    {
      j -= 1;
      // stop compiler from optimizing delay loop to noop
      NOP;
    }
    while(j > 0);

    // end of T1H
    gpio->out_w1tc |= gpioClearMask1[i - 1];
    //if(gpioClearMask1[i - 1] & 0x10000)
    //  gpio16->out = 0;
    gpio16->out = gpio16State1[i - 1];

    // delay to make a period 3333 ns
    j = 7;
    do
    {
      j -= 1;
      // stop compiler from optimizing delay loop to noop
      NOP;
    }
    while(j > 0);
  }
}

void ICACHE_RAM_ATTR dshotWrite300()
{
  volatile uint8_t i;
  volatile uint8_t j;

  for(i = 16; i > 0; i--)
  {
    // force write to GPIO registers on each loop
    asm volatile ("" : : : "memory");

    gpio->out_w1ts |= gpioSetMask;
    //if(gpioSetMask & 0x10000)
    //  gpio16->out = 1;
    gpio16->out = gpio16Set;

    // delay 1250 ns to reset pins to send 0 bit
    j = 4;
    do
    {
      j -= 1;
      // stop compiler from optimizing delay loop to noop
      NOP;
    }
    while(j > 0);

    NOP;
    NOP;
    NOP;
    NOP;
    NOP;

    // end 0f T0H
    gpio->out_w1tc |= gpioClearMask0[i - 1];
    gpio16->out = gpio16State0[i - 1];

    // delay 1250 ns to reset pins to send 1 bit
    j = 4;
    do
    {
      j -= 1;
      // stop compiler from optimizing delay loop to noop
      NOP;
    }
    while(j > 0);

    // end of T1H
    gpio->out_w1tc |= gpioClearMask1[i - 1];
    gpio16->out = gpio16State1[i - 1];

    // delay to make a period 3333 ns
    j = 2;
    do
    {
      j -= 1;
      // stop compiler from optimizing delay loop to noop
      NOP;
    }
    while(j > 0);
  }
}

/* can't implement because can't make T0H length on GPIO16 less than 666 ns. it needs 625 ns and low
void ICACHE_RAM_ATTR dshotWrite600()
{
  volatile uint8_t i;

  if(!enabled)
    return;

  for(i = 16; i > 0; i--)
  {
    // force write to GPIO registers on each loop
    asm volatile ("" : : : "memory");

    gpio16->out = gpio16Set;
    gpio->out_w1ts |= gpioSetMask;

    // end 0f T0H
    gpio16->out = gpio16State0[i - 1];
    gpio->out_w1tc |= gpioClearMask0[i - 1];

    // delay 625 ns to reset pins to send 1 bit
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;

    // end of T1H
    gpio->out_w1tc |= gpioClearMask1[i - 1];
    //if(gpioClearMask1[i - 1] & 0x10000)
    //  gpio16->out = 0;
    gpio16->out = gpio16State1[i - 1];
  }
}
*/
