// TinyUSB composite descriptors for the 4dn Timing Probe: one configuration with
// a CDC ACM debug port (2 interfaces) and a Mass Storage interface (1 interface)
// exposing the on-board FAT volume. Full speed only (RP2040 on-chip PHY), so no
// high-speed / device-qualifier / other-speed descriptors are needed.
//
// Enumeration identity:
//   VID 0x2E8A  (Raspberry Pi), PID 0x4D53 ("MS") -- a distinct app PID so the
//   host does not confuse this with the pico-sdk stdio_usb CDC (PID 0x000A) or
//   the RP2 BOOTSEL device (PID 0x0003).
//   Product string: "4dn Timing Probe".
// CDC and MSC are standard USB classes, so Linux binds cdc_acm (ttyACM*) and
// usb-storage drivers by class -- no host-side VID/PID driver match is required.

#include "pico/bootrom.h"
#include "tusb.h"

#define USB_VID 0x2E8Au
#define USB_PID 0x4D53u
#define USB_BCD 0x0200u

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,

    // Interface Association Descriptor (IAD) is required for CDC in a composite,
    // so the device-level class must be MISC / COMMON / IAD.
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01,
};

// Invoked on GET DEVICE DESCRIPTOR.
uint8_t const* tud_descriptor_device_cb(void) { return (uint8_t const*)&desc_device; }

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_MSC,
  ITF_NUM_TOTAL,
};

// RP2040 endpoint assignment (matches the pico-sdk cdc_msc example default).
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82
#define EPNUM_MSC_OUT 0x03
#define EPNUM_MSC_IN 0x83

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

uint8_t const desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power (mA).
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // CDC: interface number, string index, EP notify addr+size, EP data (out,in) addr+size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    // MSC: interface number, string index, EP out & in addr, EP size.
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

// Invoked on GET CONFIGURATION DESCRIPTOR. Full speed only.
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_fs_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC,
  STRID_MSC,
};

// Index-ordered string table. Serial is a fixed literal (no board bsp dependency).
char const* string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // 0: supported language = English (0x0409)
    "4dn",                      // 1: Manufacturer
    "4dn Timing Probe",         // 2: Product
    "4DN-0001",                 // 3: Serial number
    "4dn CDC Debug",            // 4: CDC interface
    "4dn FAT Volume",           // 5: MSC interface
};

static uint16_t _desc_str[32 + 1];

// Invoked on GET STRING DESCRIPTOR. Converts the ASCII table entry to UTF-16LE.
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  size_t chr_count;

  if (index == STRID_LANGID) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
      return NULL;

    const char* str = string_desc_arr[index];

    chr_count = strlen(str);
    size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for header
    if (chr_count > max_count)
      chr_count = max_count;

    for (size_t i = 0; i < chr_count; i++)
      _desc_str[1 + i] = str[i];
  }

  // First 16-bit word: length (bytes, incl. header) in low byte, TUSB_DESC_STRING in high.
  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

  return _desc_str;
}

//--------------------------------------------------------------------+
// Host-triggered force-reset to BOOTSEL: the "1200 bps touch".
//--------------------------------------------------------------------+
// TinyUSB calls this on every CDC SET_LINE_CODING. Opening the CDC port at 1200
// baud (e.g. `stty -F /dev/ttyACMx 1200`) lands here and reboots straight into the
// USB bootloader. This runs in core0's tud_task, INDEPENDENT of core1 -- so even a
// stuck sweep can be recovered from the host with no physical BOOTSEL. 1200 baud
// is the universal touch-to-reset convention, so it will not collide with real
// serial use.
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* coding) {
  (void)itf;
  if (coding->bit_rate == 1200) {
    reset_usb_boot(0, 0);
  }
}
