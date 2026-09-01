// FatFs diskio backend backed by a static in-RAM sector store. HOST-ONLY: this
// is what the doctest RAM-disk test formats and round-trips. The pico build
// links diskio_flash.cpp instead.

// clang-format off
extern "C" {
#include "ff.h" // defines BYTE / LBA_t / UINT; must precede diskio.h
#include "diskio.h"
}
// clang-format on

#include <array>
#include <cstdint>
#include <cstring>

namespace {

constexpr std::size_t kSectorSize = FF_MAX_SS; // 512
constexpr std::size_t kSectorCount = 256;      // 256 * 512 B = 128 KiB volume

std::array<std::uint8_t, kSectorSize * kSectorCount> g_disk;

} // namespace

extern "C" {

DSTATUS disk_status(BYTE pdrv) { return (pdrv == 0) ? 0 : STA_NOINIT; }

DSTATUS disk_initialize(BYTE pdrv) { return (pdrv == 0) ? 0 : STA_NOINIT; }

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
  if (pdrv != 0 || sector + count > kSectorCount)
    return RES_PARERR;
  std::memcpy(buff, &g_disk[sector * kSectorSize], count * kSectorSize);
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
  if (pdrv != 0 || sector + count > kSectorCount)
    return RES_PARERR;
  std::memcpy(&g_disk[sector * kSectorSize], buff, count * kSectorSize);
  return RES_OK;
}

// (pdrv, cmd) are both BYTE by the FatFs diskio API contract -- fixed upstream.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  if (pdrv != 0)
    return RES_PARERR;
  switch (cmd) {
  case CTRL_SYNC:
    return RES_OK;
  case GET_SECTOR_COUNT:
    *static_cast<LBA_t*>(buff) = static_cast<LBA_t>(kSectorCount);
    return RES_OK;
  case GET_SECTOR_SIZE:
    *static_cast<WORD*>(buff) = static_cast<WORD>(kSectorSize);
    return RES_OK;
  case GET_BLOCK_SIZE:
    *static_cast<DWORD*>(buff) = 1; // erase block size in sectors
    return RES_OK;
  default:
    return RES_PARERR;
  }
}

} // extern "C"
