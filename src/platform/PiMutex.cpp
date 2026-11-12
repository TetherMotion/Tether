/**
 * @file PiMutex.cpp
 * @brief Priority-inheritance mutexes (Linux pthread; std fallback elsewhere).
 */

#include "tether/platform/PiMutex.hpp"

#include <chrono>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <ctime>
#endif

#include "logging/Logger.hpp"

namespace Tether {
namespace Platform {

static const char* TAG = "pi_mutex";

#ifdef __linux__
namespace {

/**
 * pthread_mutexattr with PTHREAD_PRIO_INHERIT.  ENOTSUP/EINVAL degrades
 * to the plain attributes — the mutex still works, it just loses the
 * inversion bound.  Logged once per process.
 */
void initMutexAttr(pthread_mutexattr_t* attr, bool recursive) {
    pthread_mutexattr_init(attr);
    if (recursive) {
        pthread_mutexattr_settype(attr, PTHREAD_MUTEX_RECURSIVE);
    }
    static bool warned = false;
    if (pthread_mutexattr_setprotocol(attr, PTHREAD_PRIO_INHERIT) != 0 &&
        !warned) {
        warned = true;
        TETHER_LOGW(TAG, "PTHREAD_PRIO_INHERIT unsupported — mutexes fall "
                         "back to non-PI semantics (priority inversion "
                         "possible)");
    }
}

/** CLOCK_REALTIME absolute deadline (pthread_mutex_timedlock's clock). */
timespec deadlineFromNow(uint64_t timeout_ns) {
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    const uint64_t ns = static_cast<uint64_t>(ts.tv_nsec) + timeout_ns;
    ts.tv_sec  += static_cast<time_t>(ns / 1'000'000'000ULL);
    ts.tv_nsec  = static_cast<long>(ns % 1'000'000'000ULL);
    return ts;
}

} // namespace
#endif // __linux__

// ============================================================================
// PiMutex
// ============================================================================

PiMutex::PiMutex() {
#ifdef __linux__
    pthread_mutexattr_t attr;
    initMutexAttr(&attr, /*recursive=*/false);
    pthread_mutex_init(&m_, &attr);
    pthread_mutexattr_destroy(&attr);
#endif
}

PiMutex::~PiMutex() {
#ifdef __linux__
    pthread_mutex_destroy(&m_);
#endif
}

void PiMutex::lock() {
#ifdef __linux__
    const int r = pthread_mutex_lock(&m_);
    if (r != 0) {
        TETHER_LOGW(TAG, "PiMutex::lock failed ({})", r);
    }
#else
    m_.lock();
#endif
}

bool PiMutex::try_lock() {
#ifdef __linux__
    return pthread_mutex_trylock(&m_) == 0;
#else
    return m_.try_lock();
#endif
}

bool PiMutex::try_lock_for_ns(uint64_t timeout_ns) {
#ifdef __linux__
    const timespec ts = deadlineFromNow(timeout_ns);
    const int r = pthread_mutex_timedlock(&m_, &ts);
    return r == 0;
#else
    return m_.try_lock_for(std::chrono::nanoseconds(timeout_ns));
#endif
}

void PiMutex::unlock() {
#ifdef __linux__
    pthread_mutex_unlock(&m_);
#else
    m_.unlock();
#endif
}

// ============================================================================
// PiRecursiveMutex
// ============================================================================

PiRecursiveMutex::PiRecursiveMutex() {
#ifdef __linux__
    pthread_mutexattr_t attr;
    initMutexAttr(&attr, /*recursive=*/true);
    pthread_mutex_init(&m_, &attr);
    pthread_mutexattr_destroy(&attr);
#endif
}

PiRecursiveMutex::~PiRecursiveMutex() {
#ifdef __linux__
    pthread_mutex_destroy(&m_);
#endif
}

void PiRecursiveMutex::lock() {
#ifdef __linux__
    const int r = pthread_mutex_lock(&m_);
    if (r != 0) {
        TETHER_LOGW(TAG, "PiRecursiveMutex::lock failed ({})", r);
    }
#else
    m_.lock();
#endif
}

bool PiRecursiveMutex::try_lock() {
#ifdef __linux__
    return pthread_mutex_trylock(&m_) == 0;
#else
    return m_.try_lock();
#endif
}

bool PiRecursiveMutex::try_lock_for_ns(uint64_t timeout_ns) {
#ifdef __linux__
    const timespec ts = deadlineFromNow(timeout_ns);
    const int r = pthread_mutex_timedlock(&m_, &ts);
    return r == 0;
#else
    return m_.try_lock_for(std::chrono::nanoseconds(timeout_ns));
#endif
}

void PiRecursiveMutex::unlock() {
#ifdef __linux__
    pthread_mutex_unlock(&m_);
#else
    m_.unlock();
#endif
}

} // namespace Platform
} // namespace Tether
