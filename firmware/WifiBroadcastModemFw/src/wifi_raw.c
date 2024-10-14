#include "wifi_raw.h"

#include <lwip/netif.h>
#include <lwip/pbuf.h>

#include "mem.h"
#include "osapi.h"

static wifi_raw_recv_cb_fn rx_func = NULL;

IRAM_ATTR void __wrap_ppEnqueueRxq(void *a) {
  // 4 is the only spot that contained the packets
  // Discovered by trial and error printing the data
  if (rx_func) rx_func((struct RxPacket *)(((void **)a)[4]));

  // If you get an "undefined reference" error from this, make sure
  // you're linking with "-Wl,-wrap=ppEnqueueRxq" in your
  // platforms.txt, most likely in your "compiler.c.elf.flags"
  // setting.
  __real_ppEnqueueRxq(a);
}

void wifi_raw_set_recv_cb(wifi_raw_recv_cb_fn rx_fn) { rx_func = rx_fn; }

static void *transmit_buf = 0;

static void send_callback(uint8_t status) {
  (void)status;
  transmit_buf = 0;
}

int wifi_send_raw_packet(void *data, uint16_t len) {
  static uint8_t initted = 0;
  if (!initted) {
    wifi_register_send_pkt_freedom_cb(send_callback);
    // Make sure the ppEnqueueRxq wrap doesn't get skipped, for
    // instance if the wrapper linker option is missing.
    initted = 1 | (intptr_t)__wrap_ppEnqueueRxq;
  }

  if (transmit_buf) {
    return -1;
  }
  transmit_buf = data;

  return wifi_send_pkt_freedom(transmit_buf, len, true);
}
