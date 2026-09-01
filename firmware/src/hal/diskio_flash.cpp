// FatFs diskio backend over on-board RP2040 flash. TARGET-ONLY: linked into the
// pico build; the host build links diskio_ram.cpp instead. This task only needs
// it to COMPILE for the Pico (not run on hardware).
//
// Layout: a reserved region at the TOP of flash holds the FAT volume, well above
// the firmware image. FatFs sees 512-byte logical sectors; the physical flash
// erase unit is 4096 B, so writes do a read-modify-write of the containing 4 KiB
// flash sector. GET_BLOCK_SIZE reports 8 logical sectors so f_mkfs aligns
// structures to erase-block boundaries.

// clang-format off
extern "C" {
#include "ff.h" // defines BYTE / LBA_t / UINT; must precede diskio.h
#include "diskio.h"
}
// clang-format on

#include "constants.hpp"
#include "hal/flash_msc.hpp"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include <cstdint>
#include <cstring>

namespace {

constexpr std::uint32_t kLogicalSs = cfg::kLogicalSectorSizeBytes; // 512 B logical sector
static_assert(cfg::kLogicalSectorSizeBytes == FF_MAX_SS,
              "FatFs logical sector size must match cfg");
constexpr std::uint32_t kFlashSector = FLASH_SECTOR_SIZE;             // 4096 B erase unit
constexpr std::uint32_t kSectorsPerBlock = kFlashSector / kLogicalSs; // 8

// Reserve the top 512 KiB of flash for the FAT volume.
constexpr std::uint32_t kFsBytes = cfg::kFsBytes;
constexpr std::uint32_t kFsSectors = kFsBytes / kLogicalSs; // 1024 logical sectors

// Byte offset (into flash, from the start of the device) of the FS region.
constexpr std::uint32_t kFsFlashOffset = PICO_FLASH_SIZE_BYTES - kFsBytes;

// Scratch for the read-modify-write of a full 4 KiB flash sector. Static so the
// backend stays heap-free; flash writes are single-threaded here.
std::uint8_t g_rmw[kFlashSector];

const std::uint8_t* xip_ptr(std::uint32_t byte_off) {
  // XIP_BASE is a memory-mapped flash window; addressing it is inherently int->ptr.
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<const std::uint8_t*>(XIP_BASE + kFsFlashOffset + byte_off);
}

// Program one 512 B logical sector via read-modify-write of its 4 KiB erase unit.
// Shared by disk_write (FatFs) and flash_fs_write_sector (MSC) so both paths use
// identical geometry and commit semantics.
void program_sector(std::uint32_t lba, const std::uint8_t* src) {
  const std::uint32_t block = lba / kSectorsPerBlock;                 // 4 KiB block index
  const std::uint32_t within = (lba % kSectorsPerBlock) * kLogicalSs; // offset in block
  const std::uint32_t block_off = kFsFlashOffset + block * kFlashSector;

  // Read-modify: pull the current 4 KiB block, patch the one logical sector.
  // XIP_BASE memory-mapped flash window (int->ptr is inherent; see xip_ptr).
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  std::memcpy(g_rmw, reinterpret_cast<const std::uint8_t*>(XIP_BASE + block_off), kFlashSector);
  std::memcpy(g_rmw + within, src, kLogicalSs);

  // Write: erase + reprogram the 4 KiB block with interrupts masked (flash ops
  // must not be preempted by XIP-executing code on RP2040).
  const std::uint32_t irq = save_and_disable_interrupts();
  flash_range_erase(block_off, kFlashSector);
  flash_range_program(block_off, g_rmw, kFlashSector);
  restore_interrupts(irq);
}

} // namespace

namespace hal {

std::uint32_t flash_fs_sector_size() { return kLogicalSs; }

std::uint32_t flash_fs_sector_count() { return kFsSectors; }

void flash_fs_read_sector(std::uint32_t lba, std::uint8_t* dst) {
  if (lba >= kFsSectors)
    return;
  std::memcpy(dst, xip_ptr(lba * kLogicalSs), kLogicalSs);
}

void flash_fs_write_sector(std::uint32_t lba, const std::uint8_t* src) {
  if (lba >= kFsSectors)
    return;
  program_sector(lba, src);
}

} // namespace hal

extern "C" {

DSTATUS disk_status(BYTE pdrv) { return (pdrv == 0) ? 0 : STA_NOINIT; }

DSTATUS disk_initialize(BYTE pdrv) { return (pdrv == 0) ? 0 : STA_NOINIT; }

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
  if (pdrv != 0 || sector + count > kFsSectors)
    return RES_PARERR;
  // XIP: the flash region is memory-mapped, so a plain copy suffices.
  std::memcpy(buff, xip_ptr(static_cast<std::uint32_t>(sector) * kLogicalSs),
              static_cast<std::size_t>(count) * kLogicalSs);
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
  if (pdrv != 0 || sector + count > kFsSectors)
    return RES_PARERR;

  for (UINT i = 0; i < count; ++i) {
    const std::uint32_t lba = static_cast<std::uint32_t>(sector) + i;
    program_sector(lba, buff + static_cast<std::size_t>(i) * kLogicalSs);
  }
  return RES_OK;
}

// disk_ioctl's (pdrv, cmd) are both BYTE by the FatFs diskio API contract -- the
// signature is fixed upstream and cannot be reordered or retyped here.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  if (pdrv != 0)
    return RES_PARERR;
  switch (cmd) {
  case CTRL_SYNC:
    return RES_OK; // writes are synchronous (committed in disk_write)
  case GET_SECTOR_COUNT:
    *static_cast<LBA_t*>(buff) = static_cast<LBA_t>(kFsSectors);
    return RES_OK;
  case GET_SECTOR_SIZE:
    *static_cast<WORD*>(buff) = static_cast<WORD>(kLogicalSs);
    return RES_OK;
  case GET_BLOCK_SIZE:
    *static_cast<DWORD*>(buff) = kSectorsPerBlock; // erase block size in sectors
    return RES_OK;
  default:
    return RES_PARERR;
  }
}

} // extern "C"
