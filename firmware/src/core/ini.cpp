#include "core/ini.hpp"

namespace core {
namespace {
bool iequal(std::string_view a, std::string_view b) {
  if (a.size() != b.size())
    return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    char ca = a[i], cb = b[i];
    if (ca >= 'A' && ca <= 'Z')
      ca = char(ca - 'A' + 'a');
    if (cb >= 'A' && cb <= 'Z')
      cb = char(cb - 'A' + 'a');
    if (ca != cb)
      return false;
  }
  return true;
}
std::string_view trim(std::string_view s) {
  auto issp = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\f' || c == '\v'; };
  while (!s.empty() && issp(s.front()))
    s.remove_prefix(1);
  while (!s.empty() && issp(s.back()))
    s.remove_suffix(1);
  return s;
}
} // namespace

std::string_view IniResult::value_of(std::string_view key) const {
  for (std::size_t i = 0; i < count; ++i)
    if (iequal(pairs[i].key, key))
      return pairs[i].value;
  return {};
}
bool IniResult::has(std::string_view key) const {
  for (std::size_t i = 0; i < count; ++i)
    if (iequal(pairs[i].key, key))
      return true;
  return false;
}

IniResult parse_ini(std::string_view text) {
  IniResult r{};
  std::size_t pos = 0;
  while (pos < text.size()) {
    std::size_t nl = text.find('\n', pos);
    std::string_view line =
        text.substr(pos, (nl == std::string_view::npos ? text.size() : nl) - pos);
    pos = (nl == std::string_view::npos) ? text.size() : nl + 1;

    // A ';' begins a comment ANYWHERE on the line (full-line or trailing), the
    // convention most editors assume for INI. Strip from the first ';' to
    // end-of-line before parsing. The list separator is ':' (parse.cpp), so ';'
    // never occurs inside a value -- nothing legitimate is lost.
    std::size_t cm = line.find(';');
    if (cm != std::string_view::npos)
      line = line.substr(0, cm);
    std::string_view t = trim(line);
    if (t.empty())
      continue; // blank line, or a line that was entirely a comment

    std::size_t eq = t.find('=');
    if (eq == std::string_view::npos) {
      r.error = IniError::MalformedLine;
      return r;
    }
    std::string_view key = trim(t.substr(0, eq));
    std::string_view val = trim(t.substr(eq + 1));
    if (key.empty()) {
      r.error = IniError::MalformedLine;
      return r;
    }
    if (r.has(key)) {
      r.error = IniError::DuplicateKey;
      return r;
    }
    if (r.count >= kMaxIniKeys) {
      r.error = IniError::TooManyKeys;
      return r;
    }
    r.pairs[r.count++] = IniPair{key, val};
  }
  r.ok = true;
  return r;
}

} // namespace core
