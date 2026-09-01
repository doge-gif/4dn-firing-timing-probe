#include "core/fileio.hpp"

#include "constants.hpp" // cfg::kConfigTextBufBytes

namespace core {

LoadResult load_config_via_backend(Backend& b) {
  // Config files are small (a couple dozen short INI lines plus comments); this
  // fixed stack buffer must hold the ENTIRE CONFIG.INI (FatFs f_read truncates
  // to the buffer, dropping trailing keys). Sized by cfg::kConfigTextBufBytes,
  // which a static_assert ties to the baked default's size.
  char buf[cfg::kConfigTextBufBytes];
  std::size_t len = 0;
  if (!b.read_config(buf, sizeof buf, len)) {
    // Read failed (e.g. file too large for the buffer, or backend I/O error).
    // There is no dedicated "read failure" variant, so reuse IniFormat: from
    // the caller's perspective the config text could not be parsed.
    return LoadResult{false, ConfigError::IniFormat, Config{}};
  }
  return load_config(std::string_view(buf, len));
}

bool write_stepped_row(Backend& b, const SteppedRow& r) {
  char line[192]; // row is ~66 chars; sized for margin alongside the ~140-char header
  std::size_t n = csv_stepped_row(line, sizeof line, r);
  if (n == 0)
    return false;
  return b.append(std::string_view(line, n));
}

bool write_ramp_row(Backend& b, const RampRow& r) {
  char line[96];
  std::size_t n = csv_ramp_row(line, sizeof line, r);
  if (n == 0)
    return false;
  return b.append(std::string_view(line, n));
}

} // namespace core
