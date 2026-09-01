#include "core/hold_ring.hpp"
#include "doctest/doctest.h"

#include <vector>

TEST_CASE("RecentRing: fills up to N in order") {
  core::RecentRing<int, 4> r;
  CHECK(r.size() == 0);
  r.push(1);
  r.push(2);
  r.push(3);
  CHECK(r.size() == 3);
  std::vector<int> got;
  r.for_each_chronological([&](const int& v) { got.push_back(v); });
  CHECK(got == std::vector<int>{1, 2, 3});
}

TEST_CASE("RecentRing: overwrites oldest after wrap, chronological order preserved") {
  core::RecentRing<int, 4> r;
  for (int i = 1; i <= 6; ++i)
    r.push(i); // 1,2 overwritten
  CHECK(r.size() == 4);
  std::vector<int> got;
  r.for_each_chronological([&](const int& v) { got.push_back(v); });
  CHECK(got == std::vector<int>{3, 4, 5, 6}); // oldest -> newest
}

TEST_CASE("RecentRing: exactly-full N pushes yields insertion order") {
  core::RecentRing<int, 4> r;
  for (int i = 1; i <= 4; ++i)
    r.push(i); // fills exactly, head_ wraps to 0, count_==N
  CHECK(r.size() == 4);
  std::vector<int> got;
  r.for_each_chronological([&](const int& v) { got.push_back(v); });
  CHECK(got == std::vector<int>{1, 2, 3, 4}); // oldest -> newest, no overwrite yet
}

TEST_CASE("RecentRing: N=1 keeps only the last pushed") {
  core::RecentRing<int, 1> r;
  r.push(7);
  r.push(8);
  r.push(9);
  CHECK(r.size() == 1);
  std::vector<int> got;
  r.for_each_chronological([&](const int& v) { got.push_back(v); });
  CHECK(got == std::vector<int>{9});
}

TEST_CASE("RecentRing: clear resets to empty and reuse works") {
  core::RecentRing<int, 4> r;
  r.push(1);
  r.push(2);
  r.push(3);
  r.clear();
  CHECK(r.size() == 0);
  std::vector<int> got;
  r.for_each_chronological([&](const int& v) { got.push_back(v); });
  CHECK(got.empty());
  r.push(9);
  r.push(8); // reuse after clear
  got.clear();
  r.for_each_chronological([&](const int& v) { got.push_back(v); });
  CHECK(got == std::vector<int>{9, 8});
}
