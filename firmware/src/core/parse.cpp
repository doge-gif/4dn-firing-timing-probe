#include "core/parse.hpp"

#include <cstdlib>
#include <cstring>

namespace core {
namespace {
std::string_view trim(std::string_view s) {
  auto sp = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
  while (!s.empty() && sp(s.front()))
    s.remove_prefix(1);
  while (!s.empty() && sp(s.back()))
    s.remove_suffix(1);
  return s;
}
} // namespace

bool parse_u32(std::string_view s, std::uint32_t& out) {
  s = trim(s);
  if (s.empty())
    return false;
  std::uint64_t v = 0;
  for (char c : s) {
    if (c < '0' || c > '9')
      return false;
    v = v * 10 + std::uint32_t(c - '0');
    if (v > 0xFFFFFFFFull)
      return false;
  }
  out = std::uint32_t(v);
  return true;
}

bool parse_f32(std::string_view s, float& out) {
  s = trim(s);
  if (s.empty())
    return false;
  // Heap-free (no std::string): copy into a fixed stack buffer for strtof,
  // which needs a NUL-terminated C string. Longer-than-buffer tokens are rejected.
  char buf[32];
  if (s.size() >= sizeof(buf))
    return false;
  std::memcpy(buf, s.data(), s.size());
  buf[s.size()] = '\0';
  char* end = nullptr;
  float v = std::strtof(buf, &end);
  if (end != buf + s.size())
    return false;
  out = v;
  return true;
}

} // namespace core
