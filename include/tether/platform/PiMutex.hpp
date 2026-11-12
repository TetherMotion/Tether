#pragma once

/**
 * @file PiMutex.hpp
 * @brief Priority-inheritance mutexes for realtime-adjacent critical sections.
 *
 * A plain std::mutex offers no priority inheritance: a realtime thread
 * blocked on it is at the mercy of whatever preempts the (lower-priority)
 * holder — unbounded priority inversion.  On Linux these wrappers use
 * pthread_mutex with PTHREAD_PRIO_INHERIT so the kernel boosts the holder
 * to the highest blocked priority while it holds the lock, bounding the
 * inversion to the holder's actual critical-section time.
 *
 * try_lock_for_ns() gives a bounded acquire for callers (like the cyclic
 * exchange path) that must never stall beyond a deadline: it uses
 * pthread_mutex_timedlock, which honours the same PI semantics while
 * sleeping.
 *
 * Non-Linux builds fall back to std::timed_mutex / std::recursive_timed_mutex
 * (same API, no PI) — correct, just without the inversion bound.
 *
 * Both types satisfy BasicLockable/Lockable — std::lock_guard and
 * std::unique_lock work unchanged.
 */

#include <cstdint>

#ifdef __linux__
#include <pthread.h>
#else
#include <mutex>
#endif

namespace Tether {
namespace Platform {

class PiMutex {
public:
    PiMutex();
    ~PiMutex();

    PiMutex(const PiMutex&) = delete;
    PiMutex& operator=(const PiMutex&) = delete;

    void lock();
    bool try_lock();
    /**
     * @brief Acquire the mutex, waiting at most @p timeout_ns ns.
     * @return true when the mutex is held on return.
     */
    bool try_lock_for_ns(uint64_t timeout_ns);
    void unlock();

#ifdef __linux__
    pthread_mutex_t* native_handle() { return &m_; }
#endif

private:
#ifdef __linux__
    pthread_mutex_t m_;
#else
    std::timed_mutex m_;
#endif
};

class PiRecursiveMutex {
public:
    PiRecursiveMutex();
    ~PiRecursiveMutex();

    PiRecursiveMutex(const PiRecursiveMutex&) = delete;
    PiRecursiveMutex& operator=(const PiRecursiveMutex&) = delete;

    void lock();
    bool try_lock();
    /** @copydoc PiMutex::try_lock_for_ns */
    bool try_lock_for_ns(uint64_t timeout_ns);
    void unlock();

#ifdef __linux__
    pthread_mutex_t* native_handle() { return &m_; }
#endif

private:
#ifdef __linux__
    pthread_mutex_t m_;
#else
    std::recursive_timed_mutex m_;
#endif
};

} // namespace Platform
} // namespace Tether
