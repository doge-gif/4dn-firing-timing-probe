#pragma once

#include <cstdint>

// Control surface for the USB Mass Storage backend (usb_msc.cpp), which exposes
// the on-board FAT volume (same flash region + geometry as diskio_flash.cpp) over
// TinyUSB MSC. TARGET-ONLY.
namespace hal {

// Writer lock: when set, the MSC medium reports read-only to the host.
// tud_msc_is_writable_cb returns false and MSC WRITE10 is rejected with a
// write-protected sense, so a firmware run holding the FS keeps the PC from
// mutating the volume. Reads (READ10) are unaffected.
void msc_set_readonly(bool readonly);

// Signal the host that the medium changed (call after the firmware writes the FAT
// itself, e.g. an end-of-run flush) so the OS re-reads the volume and shows new
// files WITHOUT a physical replug. Implemented as a UNIT ATTENTION ("medium may
// have changed") reported on the next few Test Unit Ready polls -- the host
// re-reads the volume in place, NOT a medium-not-present eject (which the OS treats
// as a real removal needing a manual remount). Call from core0.
void msc_signal_media_change();

// Current writer-lock state.
bool msc_readonly();

// HIL diagnostics: how many READ10 calls the host has made, and the last LBA read.
// Used by the MSC bring-up gate to confirm the host is actually reading sectors.
std::uint32_t msc_read10_calls();
std::uint32_t msc_last_lba();

} // namespace hal
