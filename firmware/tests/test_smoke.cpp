#include "doctest/doctest.h"

// Proves the native toolchain + ctest wiring works before any real core/ code.
TEST_CASE("smoke: host test harness runs") {
    CHECK(1 + 1 == 2);
}
