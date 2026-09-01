#pragma once
// FlashFsBackend -- a core::Backend implementation over the on-board FAT volume
// (hal/flashfs). It bridges the SDK-free core/fileio layer (config load + CSV
// result flush) to the concrete flash FatFs storage. TARGET-ONLY (pulls in
// hal/flashfs, whose pico build talks to hardware_flash); the host unit tests use
// their own in-memory backend, so this file is never compiled for the host.
//
// Usage (core0):
//   * read_config() reads CONFIG.INI into the caller's buffer (config load path).
//   * set_output_file() picks the target CSV (MAP_xxx.CSV) for the run flush.
//   * append() appends to that target file (result flush path).
#include "core/fileio.hpp"

#include <cstddef>
#include <string_view>

namespace hal {

class FlashFsBackend : public core::Backend {
public:
  // Read the whole CONFIG.INI file into buf (up to cap). Sets len to bytes read.
  bool read_config(char* buf, std::size_t cap, std::size_t& len) override;

  // Append s to the current output CSV (see set_output_file). fs_append creates
  // the file if it does not yet exist, so the header write also creates it.
  bool append(std::string_view s) override;

  // Choose the target CSV for subsequent append() calls. 8.3 names only (LFN is
  // disabled in ffconf.h), e.g. "MAP_001.CSV". Copied into a small fixed buffer.
  void set_output_file(const char* name);

  // Bulk-flush streaming: open_output() opens the current output file ONCE (fresh)
  // so subsequent append() calls stream to the open handle instead of re-opening
  // per call; close_output() syncs + closes it. Wrap a result flush in
  // open_output()/close_output() to avoid the O(n^2) per-row fs_append cost that
  // makes a large flush (HOLD's 2048-row window) slow enough to starve USB.
  bool open_output();
  void close_output();

private:
  // True between open_output() and close_output(): append() streams to the open
  // handle. False otherwise: append() falls back to per-call fs_append.
  bool out_open_ = false;
  // Config source file (fixed name). 8.3 name.
  static constexpr const char* kConfigFile = "CONFIG.INI";

  // 8.3 name buffer: 8 + '.' + 3 + NUL. Defaults to MAP_001.CSV until set.
  char out_name_[13] = {'M', 'A', 'P', '_', '0', '0', '1', '.', 'C', 'S', 'V', '\0'};
};

} // namespace hal
