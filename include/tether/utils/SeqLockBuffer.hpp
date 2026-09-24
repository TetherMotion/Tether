#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace EtherCAT {
namespace Utils {

/**
 * Lock-free single-writer / multi-reader buffer for trivially-copyable data.
 *
 * Uses a sequence lock (seqlock): the writer increments the sequence to an
 * odd value before writing, then to an even value after.  Readers retry if
 * they observe an odd sequence or if the sequence changed during the read.
 *
 * The payload itself is copied through relaxed word-sized atomics so the
 * writer/reader copy overlap is not a formal data race — the sequence
 * counter remains the only ordering required.  Suitable for RT threads.
 *
 * Requirements:
 * - T must be trivially copyable.
 * - Exactly one writer thread.
 * - Any number of reader threads.
 * - The writer must not call write() re-entrantly.
 */
template <typename T>
class SeqLockBuffer {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SeqLockBuffer requires trivially copyable T");
    static_assert(std::atomic<uint64_t>::is_always_lock_free,
                  "SeqLockBuffer requires lock-free 64-bit atomics");

    static constexpr size_t kWords = (sizeof(T) + sizeof(uint64_t) - 1) /
                                     sizeof(uint64_t);

public:
    SeqLockBuffer() = default;

    /// Writer side: publish a new value.  Not re-entrant.
    void write(const T& value) {
        const uint64_t seq = seq_.load(std::memory_order_relaxed);
        seq_.store(seq + 1, std::memory_order_release);  // odd → writing
        uint64_t tmp[kWords]{};
        std::memcpy(tmp, &value, sizeof(T));
        for (size_t i = 0; i < kWords; ++i)
            words_[i].store(tmp[i], std::memory_order_relaxed);
        seq_.store(seq + 2, std::memory_order_release);  // even → stable
    }

    /**
     * Reader side: try to read the current value.
     * @param out destination; written only on success.
     * @return true if a consistent snapshot was read, false if a write
     *         was in progress (caller may retry or use stale data).
     */
    bool read(T& out) const {
        const uint64_t s1 = seq_.load(std::memory_order_acquire);
        if (s1 & 1u) return false;  // write in progress
        uint64_t tmp[kWords];
        for (size_t i = 0; i < kWords; ++i)
            tmp[i] = words_[i].load(std::memory_order_relaxed);
        const uint64_t s2 = seq_.load(std::memory_order_acquire);
        if (s1 != s2) return false; // changed during the copy — torn
        std::memcpy(&out, tmp, sizeof(T));
        return true;
    }

    /**
     * Reader side: read the current value, retrying a bounded number of
     * times if a concurrent write is detected.  Returns the last good
     * value (or the default-constructed value if no write has occurred
     * yet).
     */
    T read_retry(unsigned max_retries = 4) const {
        T tmp{};
        for (unsigned i = 0; i <= max_retries; ++i) {
            if (read(tmp)) return tmp;
        }
        return tmp;  // best effort — may be stale
    }

    /// Current sequence number (even = stable, odd = writing).
    uint64_t sequence() const {
        return seq_.load(std::memory_order_acquire);
    }

private:
    std::atomic<uint64_t> seq_{0};
    std::atomic<uint64_t> words_[kWords]{};
};

} // namespace Utils
} // namespace EtherCAT
