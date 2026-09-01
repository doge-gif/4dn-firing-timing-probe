#ifndef PROBER_HAL_TUSB_CONFIG_H
#define PROBER_HAL_TUSB_CONFIG_H

// TinyUSB device configuration for the 4dn Timing Probe composite (CDC + MSC).
// Compiled into any target that links `prober_usb` (usb_descriptors.c, usb_msc.cpp)
// and the pico-sdk TinyUSB sources. NOTE: this MUST NOT be combined with
// pico_enable_stdio_usb in the same executable -- that shim ships its own
// tusb_config.h + descriptors and would collide. hil_selftest keeps stdio_usb;
// hil_msc uses this.

#ifdef __cplusplus
extern "C" {
#endif

// CFG_TUSB_MCU and CFG_TUSB_OS are supplied by the pico-sdk TinyUSB CMake
// (tinyusb_common_base defines CFG_TUSB_MCU=OPT_MCU_RP2040, CFG_TUSB_OS=OPT_OS_PICO
// via -D). We restate the expected values behind guards so this header is
// self-describing without redefining the command-line macros.
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_PICO
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

// RHPort 0 device, default full speed on the RP2040 on-chip PHY.
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT 0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED OPT_MODE_DEFAULT_SPEED
#endif

// Enable device stack.
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED BOARD_TUD_MAX_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 1
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// CDC FIFO size of TX and RX (full speed).
#define CFG_TUD_CDC_RX_BUFSIZE 64
#define CFG_TUD_CDC_TX_BUFSIZE 64

// CDC endpoint transfer buffer size.
#define CFG_TUD_CDC_EP_BUFSIZE 64

// MSC endpoint buffer: one 512 B logical sector per transfer, matching the flash
// FAT volume's logical sector size (see hal/flash_msc.hpp / diskio_flash.cpp).
#define CFG_TUD_MSC_EP_BUFSIZE 512

#ifdef __cplusplus
}
#endif

#endif /* PROBER_HAL_TUSB_CONFIG_H */
