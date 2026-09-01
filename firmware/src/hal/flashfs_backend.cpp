#include "hal/flashfs_backend.hpp"

#include "hal/flashfs.hpp"

#include <cstddef>

namespace hal {

bool FlashFsBackend::read_config(char* buf, std::size_t cap, std::size_t& len) {
  return fs_read(kConfigFile, buf, cap, len);
}

bool FlashFsBackend::append(std::string_view s) {
  // Stream to the open handle during a bulk flush; otherwise fall back to the
  // per-call open/append/close path (fine for one-off writes like CONFIG.INI).
  return out_open_ ? fs_stream_write(s) : fs_append(out_name_, s);
}

bool FlashFsBackend::open_output() {
  out_open_ = fs_open_write(out_name_);
  return out_open_;
}

void FlashFsBackend::close_output() {
  if (out_open_) {
    fs_close_write();
    out_open_ = false;
  }
}

void FlashFsBackend::set_output_file(const char* name) {
  if (name == nullptr)
    return;
  // Copy into the fixed 8.3 buffer, always NUL-terminated. Names longer than the
  // buffer are truncated (callers pass short "MAP_xxx.CSV" 8.3 names).
  std::size_t i = 0;
  for (; name[i] != '\0' && i + 1 < sizeof out_name_; ++i)
    out_name_[i] = name[i];
  out_name_[i] = '\0';
}

} // namespace hal
