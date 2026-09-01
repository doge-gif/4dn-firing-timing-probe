#pragma once
// Fixed-capacity overwrite-oldest ring for the HOLD run mode's recent-rows
// window. A forever-running measurement can't buffer the whole stream, so core0
// keeps only the most recent N rows; the oldest is overwritten once full. Pure +
// SDK-free (host-testable); the buffer is a plain array so a `static` instance is
// SWD-readable at a stable symbol. Single-writer (core0 drain) — not thread-safe.
#include <cstddef>

namespace core {

template <typename T, std::size_t N>
struct RecentRing {
  static_assert(N > 0, "ring needs capacity");

  T buf_[N]{};
  std::size_t head_ = 0;  // next write index (mod N)
  std::size_t count_ = 0; // valid entries, saturates at N

  void push(const T& v) {
    buf_[head_] = v;
    head_ = (head_ + 1) % N;
    if (count_ < N)
      ++count_;
  }

  std::size_t size() const { return count_; }

  void clear() {
    head_ = 0;
    count_ = 0;
  }

  // Visit entries oldest -> newest. When full, the oldest is at head_ (the slot
  // about to be overwritten next); when not yet full, the oldest is index 0.
  template <typename Fn>
  void for_each_chronological(Fn&& fn) const {
    const std::size_t start = (count_ == N) ? head_ : 0;
    for (std::size_t k = 0; k < count_; ++k)
      fn(buf_[(start + k) % N]);
  }
};

} // namespace core
