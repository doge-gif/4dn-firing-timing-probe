#pragma once
#include "constants.hpp"

#include <array>
#include <cstddef>
#include <cstdint> // std::uint8_t (fixed-width types are a portability invariant)
#include <string_view>

namespace core {

enum class IniError : std::uint8_t { None, MalformedLine, DuplicateKey, TooManyKeys };

inline constexpr std::size_t kMaxIniKeys = cfg::kMaxIniKeys;

struct IniPair {
  std::string_view key;
  std::string_view value;
};

// All string_view members borrow from the text passed to parse_ini; this object
// must not outlive that buffer.
struct IniResult {
  bool ok = false;
  IniError error = IniError::None;
  std::size_t count = 0;
  std::array<IniPair, kMaxIniKeys> pairs{};
  // Case-insensitive lookup; returns {} if absent. To distinguish "absent" from
  // "present with empty value", call has() first.
  std::string_view value_of(std::string_view key) const;
  bool has(std::string_view key) const;
};

// Parses INI text (borrowed; returned views point into `text` and must not
// outlive it). No NUL terminator required — operates purely on the view length.
IniResult parse_ini(std::string_view text);

} // namespace core
