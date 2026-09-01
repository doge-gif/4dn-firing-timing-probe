#include "core/ini.hpp"
#include "doctest/doctest.h"

#include <cstdio>
#include <string_view>

using core::IniResult;

TEST_CASE("ini: parses key=value, trims, ignores full-line and indented comments") {
  static constexpr char text[] = "; a comment\n"
                                 "   ; an indented comment (also stripped)\n"
                                 "SWEEP_START_RPM = 100\n"
                                 "\n"
                                 "  SWEEP_END_RPM =10000  \n"
                                 "TOOTH = 0,10: 60,70 ; trailing comment\n"; // ':' separates items
  IniResult r = core::parse_ini(text);
  CHECK(r.ok);
  CHECK(r.count == 3);
  CHECK(r.value_of("sweep_start_rpm") == "100"); // keys case-insensitive
  CHECK(r.value_of("SWEEP_END_RPM") == "10000");
  CHECK(r.value_of("tooth") == "0,10: 60,70"); // trailing ';' comment stripped
}

TEST_CASE("ini: ';' starts an inline comment anywhere on the line") {
  static constexpr char text[] = "KEY = value ; this part is a comment\n";
  IniResult r = core::parse_ini(text);
  CHECK(r.ok);
  CHECK(r.value_of("key") == "value"); // everything from ';' onward is dropped
}

TEST_CASE("ini: duplicate key is an error") {
  static constexpr char text[] = "A=1\nA=2\n";
  IniResult r = core::parse_ini(text);
  CHECK_FALSE(r.ok);
  CHECK(r.error == core::IniError::DuplicateKey);
}

TEST_CASE("ini: line without '=' is an error") {
  static constexpr char text[] = "not_a_pair\n";
  IniResult r = core::parse_ini(text);
  CHECK_FALSE(r.ok);
  CHECK(r.error == core::IniError::MalformedLine);
}

TEST_CASE("ini: too many keys is an error") {
  char buf[512] = {};
  int pos = 0;
  for (std::size_t i = 0; i <= core::kMaxIniKeys; ++i)
    pos += std::snprintf(buf + pos, sizeof(buf) - std::size_t(pos), "K%zu=%zu\n", i, i);
  IniResult r = core::parse_ini(std::string_view{buf, std::size_t(pos)});
  CHECK_FALSE(r.ok);
  CHECK(r.error == core::IniError::TooManyKeys);
}
