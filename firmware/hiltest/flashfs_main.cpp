// HIL isolation gate (target-only): exercise flashfs over the REAL flash diskio
// (diskio_flash), independent of USB-MSC. Observation via the SDK's USB-CDC.
// Diagnoses whether flash write+read-back works on-device (the diskio_flash path
// was previously compile-verified only). Reports over CDC then returns to BOOTSEL.
#include "hal/flashfs.hpp"
#include "pico/bootrom.h"
#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>

int main() {
  stdio_init_all(); // USB-CDC (SDK stdio)
  const uint LED = PICO_DEFAULT_LED_PIN;
  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);

  static const char content[] = "rpm,btdc\n6000,25.00\nFLASHOK\n";
  const std::size_t clen = sizeof(content) - 1;

  bool fmt = hal::fs_format();
  bool mnt = hal::fs_mount();
  bool app = hal::fs_append("T.CSV", content);
  char buf[128];
  std::size_t len = 0;
  bool rd = hal::fs_read("T.CSV", buf, sizeof buf, len);
  bool match = rd && (len == clen) && (std::memcmp(buf, content, clen) == 0);

  for (int rep = 0; rep < 18; ++rep) {
    gpio_put(LED, rep & 1);
    printf("FLASHFS fmt=%d mnt=%d app=%d rd=%d len=%u clen=%u match=%d\n", fmt, mnt, app, rd,
           (unsigned)len, (unsigned)clen, match);
    printf("FLASHFS bytes=[");
    for (std::size_t i = 0; i < len && i < 40; ++i) {
      char c = buf[i];
      putchar((c >= 32 && c < 127) ? c : '.');
    }
    printf("]\n");
    printf("FLASHFS RESULT %s\n", match ? "PASS" : "FAIL");
    sleep_ms(1000);
  }
  printf("HIL-DONE reset_to_bootsel\n");
  sleep_ms(50);
  reset_usb_boot(0, 0);
  while (true) {
  }
}
