#include "app/spsc.hpp"
#include "doctest/doctest.h"

TEST_CASE("spsc: push/pop preserves order and reports full/empty") {
  app::Spsc<int, 4> q;
  CHECK(q.empty());
  CHECK(q.push(1));
  CHECK(q.push(2));
  CHECK(q.push(3));
  CHECK_FALSE(q.push(4)); // capacity N-1 usable
  int v = 0;
  CHECK(q.pop(v));
  CHECK(v == 1);
  CHECK(q.pop(v));
  CHECK(v == 2);
  CHECK(q.push(4));
  CHECK(q.pop(v));
  CHECK(v == 3);
  CHECK(q.pop(v));
  CHECK(v == 4);
  CHECK(q.empty());
}
