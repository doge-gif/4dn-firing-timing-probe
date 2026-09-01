#pragma once

#include <cstdint>

// TARGET-ONLY accessors for the on-board FAT volume's raw sectors, sharing the
// EXACT geometry of the FatFs flash diskio backend (diskio_flash.cpp): same base
// offset in flash, same 512 B logical sector, same sector count, same 4 KiB
// read-modify-write erase unit. The USB Mass Storage callbacks (usb_msc.cpp) use
// these so MSC and FatFs see one coherent medium: a sector written via FatFs
// (f_write -> disk_write) is immediately visible to a MSC READ10 of the same LBA,
// because both resolve to the same XIP-mapped flash bytes.
namespace hal {

// Logical sector size in bytes (512).
std::uint32_t flash_fs_sector_size();

// Number of 512 B logical sectors in the FAT volume.
std::uint32_t flash_fs_sector_count();

// Copy one 512 B logical sector `lba` into `dst` (must hold >= sector_size bytes).
void flash_fs_read_sector(std::uint32_t lba, std::uint8_t* dst);

// Overwrite one 512 B logical sector `lba` from `src` (read-modify-write of the
// containing 4 KiB flash erase unit). Interrupts are masked during the flash op.
void flash_fs_write_sector(std::uint32_t lba, const std::uint8_t* src);

} // namespace hal
