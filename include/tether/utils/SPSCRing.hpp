#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace Tether {
namespace Utils {

/**
 * Bounded lock-free single-producer / single-consumer ring buffer for
 * trivially-copyable records.
 *
 * The producer writes a slot in place and publishes it by advancing an
 * atomic head counter; the consumer reads published slots and releases
 * them by advancing an atomic tail counter.  No locks, no allocation,
 * no priority inversion — suitable for a realtime producer thread.
 *
 * When the ring is full, produce()/push() return false and increment a
 * dropped counter instead of blocking: the producer is never stalled by
 * a slow or starved consumer.
 *
 * Requirements:
 * - T must be trivially copyable.
 * - Exactly one producer thread calls produce()/push().
 * - Exactly one consumer thread calls drain()/consume().
 * - reset() may only be called while neither side is active.
 */
template <typename T, size_t Capacity>
class SPSCRing {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SPSCRing requires trivially copyable T");
    static_assert(Capacity > 0, "SPSCRing requires Capacity > 0");

public:
    static constexpr size_t capacity() { return Capacity; }

    /**
     * Producer side: write a record in place and publish it.
     * @param fill callable invoked as fill(T&) to populate the slot.
     * @return false (record dropped) if the ring is full.
     * Not re-entrant; single producer thread only.
     */
    template <typename F>
    bool produce(F&& fill) {
        const uint64_t head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) >= Capacity) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        fill(buf_[head % Capacity]);
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    /// Producer side: copy a record into the ring.  false if full.
    bool push(const T& value) {
        return produce([&](T& slot) { slot = value; });
    }

    /**
     * Consumer side: invoke consume(const T&) for up to maxRecords published
     * records, oldest first, then release the consumed slots.
     * @return number of records consumed.
     * Single consumer thread only.
     */
    template <typename F>
    size_t drain(F&& consume, size_t maxRecords) {
        const uint64_t head = head_.load(std::memory_order_acquire);
        uint64_t tail = tail_.load(std::memory_order_relaxed);
        const uint64_t limit =
            (head - tail < maxRecords) ? head : tail + maxRecords;
        size_t n = 0;
        for (; tail < limit; ++tail, ++n)
            consume(buf_[tail % Capacity]);
        tail_.store(tail, std::memory_order_release);
        return n;
    }

    /**
     * Consumer side: invoke consume(const T&) for every published record,
     * oldest first, then release the consumed slots.
     * @return number of records consumed.
     * Single consumer thread only.
     */
    template <typename F>
    size_t drain(F&& consume) {
        return drain(std::forward<F>(consume), UINT64_MAX);
    }

    /// Records dropped because the ring was full (producer side).
    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

    /// Records currently visible to the consumer.
    size_t size() const {
        const uint64_t head = head_.load(std::memory_order_acquire);
        const uint64_t tail = tail_.load(std::memory_order_acquire);
        return static_cast<size_t>(head - tail);
    }

    bool empty() const { return size() == 0; }

    /// Clear the ring.  Only safe while producer and consumer are idle.
    void reset() {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        dropped_.store(0, std::memory_order_relaxed);
    }

private:
    std::array<T, Capacity> buf_{};
    alignas(64) std::atomic<uint64_t> head_{0};  // next write (producer)
    alignas(64) std::atomic<uint64_t> tail_{0};  // next read (consumer)
    std::atomic<uint64_t> dropped_{0};
};

} // namespace Utils
} // namespace Tether
