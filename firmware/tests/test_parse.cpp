#include "core/parse.hpp"
#include "doctest/doctest.h"

#include <cstdint>

TEST_CASE("parse: uint and float") {
  std::uint32_t u{};
  CHECK(core::parse_u32("10000", u));
  CHECK(u == 10000u);
  float f{};
  CHECK(core::parse_f32("190.5", f));
  CHECK(f == doctest::Approx(190.5f));
  CHECK_FALSE(core::parse_u32("12x", u));
  CHECK_FALSE(core::parse_f32("", f));
}
