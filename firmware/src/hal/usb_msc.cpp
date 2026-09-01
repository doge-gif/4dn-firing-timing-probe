// TinyUSB Mass Storage callbacks backing the on-board FAT volume. READ10/WRITE10
// map each LBA to a 512 B logical sector of the SAME flash region FatFs uses
// (hal::flash_fs_* in diskio_flash.cpp): identical base offset, sector size, and
// sector count. Coherency: a sector committed by FatFs (f_write -> disk_write ->
// flash program) is read straight back by READ10 because both resolve to the same
// XIP-mapped flash bytes -- so a file written via flashfs BEFORE tusb_init() is
// visible to the host over MSC. WRITE10 does the same 4 KiB read-modify-write as
// disk_write.
//
// The MSC callbacks have C linkage (TinyUSB calls them by name), so they live in
// an extern "C" block. The writer-lock flag (hal::msc_set_readonly) makes the
// medium report read-only to the host without affecting host reads.

#include "hal/usb_msc.hpp"

#include "hal/flash_msc.hpp"
#include "pico/multicore.h"
#include "tusb.h"

#include <cstdint>
#include <cstring>
#include <string_view>

namespace hal {
namespace {

// Writer lock. Volatile: toggled from application context, read in the USB task.
volatile bool g_readonly = false;

// Media-change signal (usb_msc.hpp msc_signal_media_change). After the firmware
// writes the FAT itself (an end-of-run flush), the host's cached view is stale and
// new files do not appear until it re-reads. Report a UNIT ATTENTION ("medium may
// have changed") on the next few Test Unit Ready polls so the host RE-READS THE
// VOLUME IN PLACE -- crucially NOT a MEDIUM-NOT-PRESENT eject, which the OS treats
// as a real removal and then requires a manual remount. UNIT ATTENTION is a
// one-shot condition; latch it for a couple of polls to ensure the host receives
// it. Written + read only in core0's USB-task context, so a plain volatile is safe.
constexpr std::uint32_t kMediaChangePolls = 2;
volatile std::uint32_t g_media_change_polls = 0;

// Scratch for one logical sector (sub-sector reads/writes patch through it).
std::uint8_t g_sector[512];

// Program one flash sector with core1 (the lockout victim) held in a RAM spin, so
// it is never executing from XIP flash while we erase/program it (RP2040 multicore
// XIP hazard -> otherwise a host write that lands while core1 runs would hardfault
// core1). Bounded timeout so a not-yet-launched / wedged core1 can never hang the
// USB task; on failure the write is reported as an error and the host retries.
// Reads are XIP-safe and need no lockout -- only the program step does.
bool locked_write_sector(std::uint32_t lba, const std::uint8_t* data) {
  if (!multicore_lockout_start_timeout_us(100000))
    return false;
  hal::flash_fs_write_sector(lba, data);
  multicore_lockout_end_blocking();
  return true;
}

// HIL diagnostics.
volatile std::uint32_t g_read10_calls = 0;
volatile std::uint32_t g_last_lba = 0xFFFFFFFFu;

} // namespace

void msc_set_readonly(bool readonly) { g_readonly = readonly; }

bool msc_readonly() { return g_readonly; }

void msc_signal_media_change() { g_media_change_polls = kMediaChangePolls; }

std::uint32_t msc_read10_calls() { return g_read10_calls; }

std::uint32_t msc_last_lba() { return g_last_lba; }

} // namespace hal

// Fill a fixed-width SCSI ASCII field: copy up to n bytes of src, then space-pad the
// rest. SCSI INQUIRY fields are fixed-width, space-padded, left-justified -- NOT C
// strings, so no NUL terminator is written. A byte loop (not memcpy-of-strlen) is
// correct regardless of the caller's buffer state and needs no lint suppression.
static void fill_ascii_field(std::uint8_t* dst, std::size_t n, std::string_view src) {
  std::size_t i = 0;
  for (; i < n && i < src.size(); ++i) {
    dst[i] = static_cast<std::uint8_t>(src[i]);
  }
  for (; i < n; ++i) {
    dst[i] = static_cast<std::uint8_t>(' ');
  }
}

extern "C" {

// SCSI INQUIRY: fill vendor (8), product (16), revision (4) fixed-width fields.
// The three field out-params are consecutive uint8_t* by the TinyUSB MSC callback
// signature -- fixed upstream, cannot be reordered.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16],
                        uint8_t product_rev[4]) {
  (void)lun;
  fill_ascii_field(vendor_id, 8, "4dn");
  fill_ascii_field(product_id, 16, "Timing Probe");
  fill_ascii_field(product_rev, 4, "1.0");
}

// Test Unit Ready: medium normally always present (read-only state is reported via
// tud_msc_is_writable_cb, not here, so the host can still read when locked). During
// a media-change pulse (msc_signal_media_change, after a flush) report UNIT ATTENTION
// so the host re-reads the volume in place -- see the g_media_change_polls note above.
bool tud_msc_test_unit_ready_cb(uint8_t lun) {
  if (hal::g_media_change_polls > 0) {
    --hal::g_media_change_polls;
    // UNIT ATTENTION / "NOT READY TO READY CHANGE, MEDIUM MAY HAVE CHANGED": the
    // host re-reads the volume in place (vs a MEDIUM-NOT-PRESENT eject, which
    // unmounts and needs a manual remount).
    tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
    return false;
  }
  return true;
}

// READ CAPACITY: report the flash FAT volume geometry (shared with FatFs).
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size) {
  (void)lun;
  *block_count = hal::flash_fs_sector_count();
  *block_size = static_cast<uint16_t>(hal::flash_fs_sector_size());
}

// Writer lock: false => host mounts the volume read-only.
bool tud_msc_is_writable_cb(uint8_t lun) {
  (void)lun;
  return !hal::msc_readonly();
}

// Start Stop Unit (spin up / eject). Nothing to do for a flash-backed medium.
// (lun, power_condition) are both uint8_t by the TinyUSB MSC callback signature.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
  (void)lun;
  (void)power_condition;
  (void)start;
  (void)load_eject;
  return true;
}

// READ10: copy [offset, offset+bufsize) of logical sector `lba` into `buffer`.
// (lba, offset) are consecutive uint32 by the TinyUSB MSC callback signature.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer,
                          uint32_t bufsize) {
  (void)lun;
  const uint32_t ss = hal::flash_fs_sector_size();
  hal::g_read10_calls++;
  hal::g_last_lba = lba;
  if (lba >= hal::flash_fs_sector_count() || offset + bufsize > ss)
    return -1;

  if (offset == 0 && bufsize == ss) {
    hal::flash_fs_read_sector(lba, static_cast<std::uint8_t*>(buffer));
  } else {
    hal::flash_fs_read_sector(lba, hal::g_sector);
    std::memcpy(buffer, hal::g_sector + offset, bufsize);
  }
  return static_cast<int32_t>(bufsize);
}

// WRITE10: commit [offset, offset+bufsize) of logical sector `lba`. Rejected when
// the writer lock is engaged. (lba, offset): consecutive uint32 fixed by TinyUSB.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer,
                           uint32_t bufsize) {
  const uint32_t ss = hal::flash_fs_sector_size();
  if (lba >= hal::flash_fs_sector_count() || offset + bufsize > ss)
    return -1;

  if (hal::msc_readonly()) {
    // Sense: DATA PROTECT / WRITE PROTECTED.
    tud_msc_set_sense(lun, SCSI_SENSE_DATA_PROTECT, 0x27, 0x00);
    return -1;
  }

  bool ok;
  if (offset == 0 && bufsize == ss) {
    ok = hal::locked_write_sector(lba, buffer);
  } else {
    // Sub-sector write: read-modify the logical sector, then commit.
    hal::flash_fs_read_sector(lba, hal::g_sector);
    std::memcpy(hal::g_sector + offset, buffer, bufsize);
    ok = hal::locked_write_sector(lba, hal::g_sector);
  }
  if (!ok) {
    // Could not fence core1 off XIP in time -> fail cleanly (host retries).
    tud_msc_set_sense(lun, SCSI_SENSE_HARDWARE_ERROR, 0x00, 0x00);
    return -1;
  }
  return static_cast<int32_t>(bufsize);
}

// Other SCSI commands (READ10/WRITE10 have dedicated callbacks and must not be
// handled here). Reject the rest with ILLEGAL REQUEST.
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize) {
  (void)buffer;
  (void)bufsize;
  switch (scsi_cmd[0]) {
  default:
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
  }
}

} // extern "C"
