#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace app {

// Lock-free single-producer / single-consumer ring buffer.
//
// Contract: exactly one producer thread/core calls push(), and exactly one
// (possibly different) consumer thread/core calls pop()/empty(). No other
// concurrency pattern is safe with this implementation.
//
// Storage is a fixed-size std::array<T, N> (no heap allocation), so this is
// suitable for use across RP2040 cores (core 1 producer, core 0 consumer)
// as well as for host-side unit testing.
//
// Capacity: N slots are allocated but only N-1 are usable. One slot is
// deliberately kept empty so that head == tail unambiguously means "empty"
// and (tail + 1) % N == head unambiguously means "full" — the classic
// single-slot-sacrifice ring buffer design that needs no extra "count"
// variable (which would itself be a second point of contention between
// producer and consumer).
//
// Memory ordering: head_ is only ever written by the consumer and tail_ is
// only ever written by the producer. Each side does a relaxed load of its
// own index (only it writes it, so no synchronization is needed there) and
// an acquire load of the other side's index (to see the other side's most
// recent writes to the storage array and its index). Each side publishes
// its own new index with a release store, which pairs with the other
// side's acquire load. This is sufficient for correctness on RP2040's
// Cortex-M0+ cores, which support aligned 32-bit atomic loads/stores.
template <typename T, std::size_t N>
class Spsc {
public:
  Spsc() : head_(0), tail_(0) {}

  // Producer side. Returns false (and leaves the queue unchanged) if
  // the queue is full.
  bool push(const T& v) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t next = advance(tail);
    if (next == head_.load(std::memory_order_acquire)) {
      return false; // full
    }
    buf_[tail] = v;
    tail_.store(next, std::memory_order_release);
    return true;
  }

  // Consumer side. Returns false (and leaves 'out' unchanged) if the
  // queue is empty.
  bool pop(T& out) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) {
      return false; // empty
    }
    out = buf_[head];
    head_.store(advance(head), std::memory_order_release);
    return true;
  }

  // Safe to call from either side; uses acquire loads of both indices.
  bool empty() const {
    return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
  }

private:
  static constexpr std::size_t advance(std::size_t idx) { return (idx + 1) % N; }

  std::array<T, N> buf_{};
  std::atomic<std::size_t> head_;
  std::atomic<std::size_t> tail_;
};

} // namespace app
