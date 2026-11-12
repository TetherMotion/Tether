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

/**Per-section lock configuration — each flag is an independent opt-out. */
struct MemoryLockSections {
    bool lock_all_process = false; /**< mlockall(MCL_CURRENT|MCL_FUTURE) */
    bool lock_image       = true; /**< process-image buffers */
    bool lock_slots       = true; /**< cyclic slot bank + staging buffers */
    bool prefault_stack   = true; /**< touch cyclic thread stack pages */
    /**
     * Bytes of the worker thread's own stack to pre-touch at thread start
     * (0 → a conservative default of 128 KiB).
     */
    uint32_t stack_prefault_bytes = 0;
};

/**
 * @brief Configure the glibc allocator so it cannot hand locked pages
 *        back to the kernel and refault them later (Linux only):
 *
 *          - M_MMAP_MAX = 0      — never service chunks via mmap, so a
 *                                  free() can never munmap() a page the
 *                                  process locked.
 *          - M_TRIM_THRESHOLD=-1 — never return heap top to the kernel.
 *          - M_ARENA_MAX = 1     — one arena: bounds allocator lock
 *                                  scope and arena fragmentation.
 *
 *        Called automatically by lockAllMemory(); harmless to call
 *        standalone before realtime setup.
 */
void configureRtAllocator();

/**
 * @brief mlockall(MCL_CURRENT | MCL_FUTURE) — lock all current and future
 *        mappings of this process.  Faults in every current page at lock
 *        time (code, libraries, stack, heap — nothing can refault from
 *        storage mid-cycle).  Cannot exclude regions; use the
 *        per-section flags in MemoryLockSections for granular control.
 *
 *        configureRtAllocator() is applied first so the allocator does
 *        not defeat the lock afterwards.
 * @param on_fault  Add MCL_ONFAULT (Linux ≥ 4.4): pin against eviction
 *        but skip the up-front population — first touches still fault
 *        (cheap, never goes to storage) instead of paying the whole
 *        fault storm at startup.
 * @return true on success, false (logged) on insufficient privilege.
 */
bool lockAllMemory(bool on_fault);
bool lockAllMemory();   ///< Same as lockAllMemory(false).

/**
 * @brief mlock() a region — pins the pages so they can neither fault nor
 *        be evicted.  Best-effort: logs + returns false on failure.
 *
 * @param addr  Region start (page-aligned recommended).
 * @param len   Region length in bytes.
 * @return true on success; false on failure (logged).
 */
bool lockMemory(const void* addr, size_t len);

/**
 * @brief munlock() a previously locked region (cleanup counterpart).
 * @param addr  Region start.
 * @param len   Region length in bytes.
 * @return true on success.
 */
bool unlockMemory(const void* addr, size_t len);

/**
 * @brief Pre-fault a region by writing one byte per page — forces demand
 *        paging NOW (at configuration time) instead of mid-cycle.  Works
 *        without any privilege; combine with lockMemory() for permanence.
 *
 * @param addr  Region start.
 * @param len   Region length in bytes.
 */
void prefaultMemory(void* addr, size_t len);

/**
 * @brief Pre-fault the calling thread's stack by descending ~bytes deep.
 *        Uses alloca() to extend the frame — each touched page becomes
 *        resident.  Call at the top of an RT thread's main function.
 *
 * @param bytes  Approximate stack depth to pre-touch.
 */
void prefaultCurrentStack(uint32_t bytes);

/**
 * @brief prctl(PR_SET_TIMERSLACK, ns) — kill the ~50 µs default timer
 *        slack applied to nanosleep/poll timeouts for non-RT threads.
 *        No-op under SCHED_FIFO (kernel skips slack for RT tasks) but
 *        critical on the degraded non-RT fallback path.
 * @param slack_ns  Requested timer slack in nanoseconds.
 * @return true on success / unsupported platform.
 */
bool setCurrentThreadTimerSlack(uint64_t slack_ns);

} // namespace Platform
} // namespace Tether
