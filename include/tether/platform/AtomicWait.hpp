/**
 * @file AtomicWait.hpp
 * @brief Timed wait/wake on a 32-bit atomic word, per platform.
 *
 * std::atomic<T>::wait() has no timed variant, so timed waits need the OS
 * primitive directly — the same pattern ProcessImage already uses for its
 * futex seam, factored out for reuse:
 *
 *   - Linux:   FUTEX_WAIT_PRIVATE / FUTEX_WAKE_PRIVATE on the word's
 *              address (in-process only — shared/shm words must use the
 *              non-private ops, see ProcessImage::futexWaitOn).
 *   - Windows: WaitOnAddress / WakeByAddressAll (identical semantics).
 *   - Other:   bounded yield-poll fallback (correct, coarser latency).
 *
 * Contract mirrors futex: atomicWait() returns true when the word was
 * observed != expected (or spuriously), false on timeout.  Callers must
 * re-check their own predicate; the word is a wake key, not the condition.
 *
 * NOTE: do NOT mix these waits with std::atomic::wait/notify on the same
 * word — libstdc++ keeps a separate waiter pool (see ProcessImage.cpp).
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <climits>
#include <ctime>

#if defined(__linux__)
#include <sys/syscall.h>
#include <unistd.h>
#include <linux/futex.h>
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <synchapi.h>
#pragma comment(lib, "synchronization.lib")
#else
#include <chrono>
#include <thread>
#endif

namespace Tether {
namespace Platform {

/// Wake every thread blocked in atomicWait() on @p word.
inline void atomicWakeAll(std::atomic<uint32_t>* word) {
#if defined(__linux__)
    ::syscall(SYS_futex, reinterpret_cast<uint32_t*>(word),
              FUTEX_WAKE_PRIVATE, INT32_MAX, nullptr, nullptr, 0);
#elif defined(_WIN32)
    ::WakeByAddressAll(word);
#else
    (void)word;
#endif
}

/// Wake at most one waiter on @p word.
inline void atomicWakeOne(std::atomic<uint32_t>* word) {
#if defined(__linux__)
    ::syscall(SYS_futex, reinterpret_cast<uint32_t*>(word),
              FUTEX_WAKE_PRIVATE, 1, nullptr, nullptr, 0);
#elif defined(_WIN32)
    ::WakeByAddressSingle(word);
#else
    (void)word;
#endif
}

/**
 * @brief Block while `*word == expected`, up to @p timeout_ns (<0 = forever).
 * @return true if the wait ended because the word may have changed
 *         (or spuriously — futex semantics), false on timeout.
 */
inline bool atomicWait(std::atomic<uint32_t>* word, uint32_t expected,
                       int64_t timeout_ns) {
#if defined(__linux__)
    timespec ts{};
    timespec* tsp = nullptr;
    if (timeout_ns >= 0) {
        ts.tv_sec  = timeout_ns / 1'000'000'000;
        ts.tv_nsec = timeout_ns % 1'000'000'000;
        tsp = &ts;
    }
    const long r = ::syscall(SYS_futex,
                             reinterpret_cast<uint32_t*>(word),
                             FUTEX_WAIT_PRIVATE,
                             static_cast<int>(expected), tsp, nullptr, 0);
    return r == 0;
#elif defined(_WIN32)
    DWORD ms = INFINITE;
    if (timeout_ns >= 0) {
        ms = static_cast<DWORD>((timeout_ns + 999'999) / 1'000'000);
    }
    const uint32_t cmp = expected;
    return ::WaitOnAddress(word, const_cast<uint32_t*>(&cmp),
                           sizeof(uint32_t), ms) != FALSE;
#else
    if (word->load(std::memory_order_acquire) != expected) return true;
    if (timeout_ns == 0) return false;
    const auto deadline =
        timeout_ns < 0
            ? std::chrono::steady_clock::time_point::max()
            : std::chrono::steady_clock::now() +
                  std::chrono::nanoseconds(timeout_ns);
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
        if (word->load(std::memory_order_acquire) != expected) return true;
    }
    return word->load(std::memory_order_acquire) != expected;
#endif
}

} // namespace Platform
} // namespace Tether
