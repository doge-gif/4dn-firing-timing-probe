#include "hal/gpio.hpp"

#include "hal/pins.hpp"
#include "hardware/gpio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"

namespace hal {

void init_io() {
  // Pickup pattern output (GP18): plain SIO output. SoftPattern drives it as SIO
  // (a CPU timer-alarm), so it stays an SIO output for the whole run. Boot/ready
  // = LOW so no teeth are emitted until emission starts.
  gpio_init(kPickup);
  gpio_set_dir(kPickup, GPIO_OUT);
  gpio_put(kPickup, 0);

  // Ign-sense inputs (per cylinder, kIgnPins): high-Z. Each coil-drive line has a
  // strong ~470 Ohm pull-up to +12V, then a 2.7k/1k divider scales the node to just
  // under 3.3V at the MCU pin (see schematics/), so the pin already sits at a
  // low-impedance idle-high divider tap (~730 Ohm Thevenin). Disable BOTH internal
  // pulls: either one would fight the divider and shift its logic threshold.
  for (std::size_t i = 0; i < kCylCount; ++i) {
    gpio_init(kIgnPins[i]);
    gpio_set_dir(kIgnPins[i], GPIO_IN);
    gpio_disable_pulls(kIgnPins[i]);
  }

  // Tach input (GP19, optional): plain input, high-Z.
  gpio_init(kTach);
  gpio_set_dir(kTach, GPIO_IN);
  gpio_disable_pulls(kTach);

  // Onboard LED (GP25): output, off.
  gpio_init(kLed);
  gpio_set_dir(kLed, GPIO_OUT);
  gpio_put(kLed, 0);
}

// Read the BOOTSEL button. On a plain Pico BOOTSEL is wired to the flash chip's
// CS line (QSPI SS); there is no dedicated GPIO. This is the canonical
// pico-examples get_bootsel_button() technique: momentarily float the QSPI SS
// output, sample the pad, then restore. Must run from RAM (flash is unusable
// while CS is floated) and with interrupts disabled.
static bool __no_inline_not_in_flash_func(read_bootsel)() {
  const uint kCsPinIndex = 1; // QSPI SS is io_qspi index 1

  uint32_t flags = save_and_disable_interrupts();

  // Set CS pad OEOVER to disable output (float it) so we can read the button.
  hw_write_masked(&ioqspi_hw->io[kCsPinIndex].ctrl,
                  GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                  IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

  // Give the pull-up / button time to drive the line before sampling.
  for (volatile int i = 0; i < 1000; ++i) {
    // busy-wait; flash is unavailable so no timer helpers here
  }

  // BOOTSEL grounds the line when pressed -> logic 0 means "pressed".
  bool pressed = (sio_hw->gpio_hi_in & (1u << kCsPinIndex)) == 0;

  // Restore normal CS output drive.
  hw_write_masked(&ioqspi_hw->io[kCsPinIndex].ctrl,
                  GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                  IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

  restore_interrupts(flags);
  return pressed;
}

bool bootsel_pressed() { return read_bootsel(); }

void led_set(bool on) { gpio_put(kLed, on ? 1 : 0); }

} // namespace hal
