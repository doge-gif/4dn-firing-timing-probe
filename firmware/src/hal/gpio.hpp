#pragma once
// hal GPIO bring-up: boot pin states + small helpers.
namespace hal {

// Configure all board GPIO to their documented boot/ready states:
//   GP18 pickup output driven LOW (deasserted; teeth only while running)
//   GP16/GP17 ign-sense inputs, high-Z (external 1k pull-up, see .cpp rationale)
//   GP19 tach input
//   GP25 LED output, LOW (off)
void init_io();

// True while the BOOTSEL button is held (wraps pico bootrom get_bootsel_button).
bool bootsel_pressed();

// Drive the onboard LED (GP25).
void led_set(bool on);

} // namespace hal
