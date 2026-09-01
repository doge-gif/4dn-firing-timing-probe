// HIL gate for the TinyUSB CDC+MSC composite (built with -DPROBER_HILTEST=ON,
// target `hil_msc`). This firmware formats the on-board FAT volume, writes a known
// file, then brings up the composite USB device so a PC can:
//   1. mount the Mass Storage drive (driverless) and read MAP_TEST.CSV, and
//   2. see a CDC ACM debug port (/dev/ttyACM*) enumerate.
//
// It does NOT use pico_enable_stdio_usb / stdio_init_all -- this build ships its
// own TinyUSB descriptors (hal/usb_descriptors.c), which would collide with the
// stdio_usb shim's descriptors. The primary gate signal is the MSC-readable file.
//
// The volume is exposed READ-ONLY (writer lock engaged): the firmware
// owns the FS, so the host mounts it read-only. Reads are unaffected.
//
// After ~60 s the device returns to BOOTSEL via reset_usb_boot() so the controller
// can reflash without a physical button press.

#include "hal/flash_msc.hpp"
#include "hal/flashfs.hpp"
#include "hal/pins.hpp"
#include "hal/usb_msc.hpp"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include <cstdint>
#include <cstdio>

namespace {

// Known content the controller verifies after mounting the MSC drive. Keep this
// EXACTLY in sync with the controller's expectation. The trailing HILMSC_OK line
// is the pass sentinel.
constexpr const char* kMapFile = "MAP_TEST.CSV";
constexpr const char* kMapContent = "rpm_cmd,cyl,mean_btdc_deg\n"
                                    "6000,0,25.00\n"
                                    "HILMSC_OK\n";

// Total uptime before dropping back to BOOTSEL (ample time to enumerate + mount +
// read on a slow host).
constexpr std::uint32_t kUptimeMs = 60000;

std::uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

} // namespace

int main() {
  // 1. Prepare the FAT volume BEFORE USB comes up so MSC READ10 sees the file.
  const bool fs_ok = hal::fs_format() && hal::fs_mount() && hal::fs_append(kMapFile, kMapContent);

  // 2. Engage the writer lock: the host mounts the volume read-only.
  hal::msc_set_readonly(true);

  // Diagnostic: read the boot sector straight from flash (same path MSC serves)
  // to confirm a valid FAT (0x55AA at [510..511]) is present before USB comes up.
  std::uint8_t bs[512];
  hal::flash_fs_read_sector(0, bs);
  const unsigned boot_sig = (unsigned(bs[510]) << 8) | unsigned(bs[511]);
  const unsigned cap_sectors = hal::flash_fs_sector_count();
  const unsigned cap_ss = hal::flash_fs_sector_size();

  // LED as a liveness indicator (no stdio here).
  const uint led = PICO_DEFAULT_LED_PIN;
  gpio_init(led);
  gpio_set_dir(led, GPIO_OUT);
  gpio_put(led, 1);

  // 3. Bring up the TinyUSB device stack (RP2040 clocks are already configured by
  // the pico-sdk runtime; no board_init needed).
  tusb_rhport_init_t dev_init = {};
  dev_init.role = TUSB_ROLE_DEVICE;
  dev_init.speed = TUSB_SPEED_AUTO;
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  const std::uint32_t start = now_ms();
  std::uint32_t last_blink = start;
  std::uint32_t last_banner = 0;
  bool led_state = true;
  bool banner_kicked = false;

  while (now_ms() - start < kUptimeMs) {
    tud_task(); // service USB (enumeration, MSC, CDC)

    const std::uint32_t t = now_ms();

    // Blink ~5 Hz while alive.
    if (t - last_blink >= 100) {
      last_blink = t;
      led_state = !led_state;
      gpio_put(led, led_state);
    }

    // CDC banner once mounted (secondary signal): confirms the ttyACM path works
    // and reports the gate status. Emitted roughly once a second.
    if (tud_cdc_connected() && (!banner_kicked || t - last_banner >= 1000)) {
      last_banner = t;
      banner_kicked = true;
      char line[160];
      std::snprintf(line, sizeof line,
                    "HILMSC fs_ok=%d bootsig=%04X cap=%ux%uB read10=%lu last_lba=%lu\r\n",
                    fs_ok ? 1 : 0, boot_sig, cap_sectors, cap_ss,
                    (unsigned long)hal::msc_read10_calls(), (unsigned long)hal::msc_last_lba());
      tud_cdc_write_str(line);
      tud_cdc_write_flush();
    }
  }

  // 4. Return to BOOTSEL for the next flash.
  reset_usb_boot(0, 0);
  while (true) {
  }
}
