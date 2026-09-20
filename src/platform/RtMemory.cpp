/**
 * @file RtMemory.cpp
 * @brief Realtime memory locking / pre-faulting (Linux; stubs elsewhere).
 */

#include "tether/platform/RtMemory.hpp"

#ifdef __linux__
#include <alloca.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <unistd.h>
#endif

#include <cstring>

#include "logging/Logger.hpp"

namespace Tether {
namespace Platform {

static const char* TAG = "rt_mem";

namespace {
constexpr size_t kPage = 4096;
}

bool lockAllMemory() {
#ifdef __linux__
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
        TETHER_LOGI(TAG, "mlockall(MCL_CURRENT|MCL_FUTURE) active");
        return true;
    }
    TETHER_LOGW(TAG,
        "mlockall failed ({}) — cyclic memory can page-fault mid-cycle "
        "(grant CAP_IPC_LOCK or raise RLIMIT_MEMLOCK to fix)",
        strerror(errno));
    return false;
#else
    return false;
#endif
}

bool lockMemory(const void* addr, size_t len) {
#ifdef __linux__
    if (!addr || len == 0) return true;
    // mlock requires page-aligned start — widen to cover the range.
    const uintptr_t a    = reinterpret_cast<uintptr_t>(addr);
    const uintptr_t base = a & ~(static_cast<uintptr_t>(kPage) - 1);
    const size_t    span = static_cast<size_t>(a - base) + len;
    if (mlock(reinterpret_cast<const void*>(base), span) == 0) return true;
    TETHER_LOGW(TAG, "mlock({} bytes @ {:p}) failed: {}", len, addr,
                strerror(errno));
    return false;
#else
    (void)addr; (void)len;
    return false;
#endif
}

bool unlockMemory(const void* addr, size_t len) {
#ifdef __linux__
    if (!addr || len == 0) return true;
    const uintptr_t a    = reinterpret_cast<uintptr_t>(addr);
    const uintptr_t base = a & ~(static_cast<uintptr_t>(kPage) - 1);
    const size_t    span = static_cast<size_t>(a - base) + len;
    return munlock(reinterpret_cast<const void*>(base), span) == 0;
#else
    (void)addr; (void)len;
    return false;
#endif
}

void prefaultMemory(void* addr, size_t len) {
    // One volatile write per page — forces the minor fault now.
    auto* p = static_cast<volatile uint8_t*>(addr);
    for (size_t off = 0; off < len; off += kPage) p[off] = p[off];
    if (len > 0) p[len - 1] = p[len - 1];
}

void prefaultCurrentStack(uint32_t bytes) {
#ifdef __linux__
    if (bytes == 0) bytes = 128 * 1024;   // conservative default
    // Descend the stack touching one byte per page.  alloca() inside the
    // loop keeps extending the frame downward so every page is touched.
    volatile uint8_t sink = 0;
    for (uint32_t off = 0; off < bytes; off += kPage) {
        auto* chunk = static_cast<volatile uint8_t*>(alloca(kPage));
        chunk[0] = sink;
        sink = chunk[0];
    }
    (void)sink;
#else
    (void)bytes;
#endif
}

bool setCurrentThreadTimerSlack(uint64_t slack_ns) {
#ifdef __linux__
    return prctl(PR_SET_TIMERSLACK,
                 static_cast<unsigned long>(slack_ns)) == 0;
#else
    (void)slack_ns;
    return false;
#endif
}

} // namespace Platform
} // namespace Tether
