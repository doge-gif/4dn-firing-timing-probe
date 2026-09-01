#include "hal/flashfs.hpp"

extern "C" {
#include "ff.h"
}

#include <cstdio>

// Portable FatFs facade. All storage specifics live in a diskio_*.cpp backend
// (RAM disk on host, on-board flash on pico); this file only drives the FatFs C
// API and is therefore SDK-free and shared by both builds. Heap-free: the
// mount object and work buffers are static / on the stack.
namespace hal {
namespace {

// Default logical drive. With FF_VOLUMES == 1 the empty path selects it for
// f_mkfs / f_mount / f_open.
constexpr const TCHAR* kDrive = _T("");

// Persistent mount object handed to f_mount (must outlive the mount).
FATFS g_fs;

// Single open handle for the streaming-write API (fs_open_write/stream/close).
FIL g_stream_fp;
bool g_stream_open = false;

} // namespace

bool fs_format() {
  BYTE work[FF_MAX_SS * 2];
  // nullptr opt -> default parameters; FatFs picks FAT12/16/32 by volume size.
  return f_mkfs(kDrive, nullptr, work, sizeof work) == FR_OK;
}

bool fs_mount() { return f_mount(&g_fs, kDrive, 1) == FR_OK; }

bool fs_append(const char* name, std::string_view data) {
  FIL fp;
  if (f_open(&fp, name, FA_WRITE | FA_OPEN_APPEND) != FR_OK)
    return false;
  UINT written = 0;
  FRESULT fr = f_write(&fp, data.data(), static_cast<UINT>(data.size()), &written);
  bool ok = (fr == FR_OK) && (written == data.size());
  ok = (f_close(&fp) == FR_OK) && ok;
  return ok;
}

bool fs_open_write(const char* name) {
  if (g_stream_open) { // defensive: never leak a prior stream
    f_close(&g_stream_fp);
    g_stream_open = false;
  }
  if (f_open(&g_stream_fp, name, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    return false;
  g_stream_open = true;
  return true;
}

bool fs_stream_write(std::string_view data) {
  if (!g_stream_open)
    return false;
  UINT written = 0;
  FRESULT fr = f_write(&g_stream_fp, data.data(), static_cast<UINT>(data.size()), &written);
  return (fr == FR_OK) && (written == data.size());
}

bool fs_close_write() {
  if (!g_stream_open)
    return true;
  const bool ok = (f_close(&g_stream_fp) == FR_OK);
  g_stream_open = false;
  return ok;
}

bool fs_read(const char* name, char* buf, std::size_t cap, std::size_t& len) {
  len = 0;
  FIL fp;
  if (f_open(&fp, name, FA_READ) != FR_OK)
    return false;
  UINT got = 0;
  FRESULT fr = f_read(&fp, buf, static_cast<UINT>(cap), &got);
  bool ok = (fr == FR_OK);
  ok = (f_close(&fp) == FR_OK) && ok;
  if (ok)
    len = got;
  return ok;
}

bool fs_exists(const char* name) {
  FIL fp;
  if (f_open(&fp, name, FA_READ) != FR_OK)
    return false; // FR_NO_FILE (or any open error) -> treat as absent
  f_close(&fp);
  return true;
}

const char* fs_next_free_name(char* out, std::size_t cap, const char* fmt) {
  for (unsigned n = 1; n <= 999; ++n) {
    std::snprintf(out, cap, fmt, n);
    if (!fs_exists(out))
      return out;
  }
  std::snprintf(out, cap, fmt, 999u); // all 999 taken (pathological) -> reuse last
  return out;
}

} // namespace hal
