#include "core/stats.hpp"
#include "doctest/doctest.h"

#include <initializer_list>

TEST_CASE("welford: mean and sample stddev match reference") {
  core::Welford w;
  for (float x : {2.0f, 4.0f, 4.0f, 4.0f, 5.0f, 5.0f, 7.0f, 9.0f})
    w.add(x);
  CHECK(w.count() == 8);
  CHECK(w.mean() == doctest::Approx(5.0f));
  CHECK(w.stddev_sample() == doctest::Approx(2.138089935f).epsilon(1e-5)); // sqrt(32/7)
}

TEST_CASE("welford: single sample has zero stddev") {
  core::Welford w;
  w.add(3.3f);
  CHECK(w.mean() == doctest::Approx(3.3f));
  CHECK(w.stddev_sample() == doctest::Approx(0.0f));
}

TEST_CASE("median_minmax: odd and even counts") {
  float odd[] = {5, 1, 3};
  core::MedianResult a = core::median_minmax(odd, 3);
  CHECK(a.median == doctest::Approx(3.0f));
  CHECK(a.min == 1.0f);
  CHECK(a.max == 5.0f);
  float even[] = {5, 1, 3, 9};
  core::MedianResult b = core::median_minmax(even, 4);
  CHECK(b.median == doctest::Approx(4.0f)); // (3+5)/2
  CHECK(b.min == 1.0f);
  CHECK(b.max == 9.0f);
}
