#pragma once
#include "core/config.hpp"
#include "core/csv.hpp"

#include <cstddef>
#include <string_view>

namespace core {

// Abstract byte backend for config read + result append. NOT on the timing hot
// path (config load at boot / CSV flush at run end), so a virtual interface is
// acceptable here (core/ timing code stays non-virtual).
struct Backend {
  virtual bool read_config(char* buf, std::size_t cap, std::size_t& len) = 0;
  virtual bool append(std::string_view s) = 0;
  virtual ~Backend() = default;
};

// Read the whole config file via the backend into a fixed buffer, then validate.
LoadResult load_config_via_backend(Backend& b);

// Format one row via csv_* into a stack buffer and append it via the backend.
bool write_stepped_row(Backend& b, const SteppedRow& r);
bool write_ramp_row(Backend& b, const RampRow& r);

} // namespace core
