#pragma once

/**
 * @file RtMemory.hpp
 * @brief Realtime memory locking and pre-faulting helpers.
 *
 * Demand-paged memory is a jitter source: a minor fault mid-cycle costs
 * ~1-10 µs and — without mlock — pages can be evicted under memory
 * pressure, so faults can recur *forever*, not just at startup.
 *
 * Two granularities are provided:
 *
 *   - lockAll(): mlockall(MCL_CURRENT | MCL_FUTURE) — process-wide, the
 *     strongest guarantee but the least selective (locks every future
 *     allocation too; cannot exclude regions).
 *   - lockRegion()/prefaultRegion()/prefaultStack(): per-section locking so
 *     callers can opt individual areas in or out — the cyclic executive
 *     maps CyclicLoopConfig::MemoryLockConfig sections onto these.
 *
 * All functions are best-effort: they log and return false on failure
 * (missing CAP_IPC_LOCK / RLIMIT_MEMLOCK) rather than failing the caller —
 * running unlocked is a performance degradation, never a correctness issue.
 * Non-Linux platforms are stubs that return false.
 */

#include <cstddef>
#include <cstdint>

namespace Tether {
namespace Platform {

/// Per-section lock configuration — each flag is an independent opt-out.
struct MemoryLockSections {
    bool lock_all_process = false;   ///< mlockall(MCL_CURRENT|MCL_FUTURE)
    bool lock_image       = true;    ///< process-image buffers
    bool lock_slots       = true;    ///< cyclic slot bank + staging buffers
    bool prefault_stack   = true;    ///< touch cyclic thread stack pages
    /// Bytes of the worker thread's own stack to pre-touch at thread start
    /// (0 → a conservative default of 128 KiB).
    uint32_t stack_prefault_bytes = 0;
};

/**
 * @brief mlockall(MCL_CURRENT | MCL_FUTURE) — lock all current and future
 *        mappings of this process.  Cannot exclude regions; use the
 *        per-section flags in MemoryLockSections for granular control.
 * @return true on success, false (logged) on insufficient privilege.
 */
bool lockAllMemory();

/**
 * @brief mlock() a region — pins the pages so they can neither fault nor
 *        be evicted.  Best-effort: logs + returns false on failure.
 */
bool lockMemory(const void* addr, size_t len);

/**
 * @brief munlock() a previously locked region (cleanup counterpart).
 */
bool unlockMemory(const void* addr, size_t len);

/**
 * @brief Pre-fault a region by writing one byte per page — forces demand
 *        paging NOW (at configuration time) instead of mid-cycle.  Works
 *        without any privilege; combine with lockMemory() for permanence.
 */
void prefaultMemory(void* addr, size_t len);

/**
 * @brief Pre-fault the calling thread's stack by descending ~bytes deep.
 *        Uses alloca() to extend the frame — each touched page becomes
 *        resident.  Call at the top of an RT thread's main function.
 */
void prefaultCurrentStack(uint32_t bytes);

/**
 * @brief prctl(PR_SET_TIMERSLACK, ns) — kill the ~50 µs default timer
 *        slack applied to nanosleep/poll timeouts for non-RT threads.
 *        No-op under SCHED_FIFO (kernel skips slack for RT tasks) but
 *        critical on the degraded non-RT fallback path.
 * @return true on success / unsupported platform.
 */
bool setCurrentThreadTimerSlack(uint64_t slack_ns);

} // namespace Platform
} // namespace Tether
