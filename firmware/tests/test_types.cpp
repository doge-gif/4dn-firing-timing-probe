#include "core/types.hpp"
#include "doctest/doctest.h"

TEST_CASE("types: Config default-constructs empty and within capacity") {
  core::Config c{};
  CHECK(c.tooth_count == 0);
  CHECK(c.cyl_count == 0);
  CHECK(core::kMaxTeeth >= 4);
  CHECK(core::kMaxCylinders >= 2);
}
