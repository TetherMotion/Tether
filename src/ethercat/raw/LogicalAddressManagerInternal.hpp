/**
 * @file LogicalAddressManagerInternal.hpp
 * @brief Shared helpers for the LogicalAddressManager translation units.
 *
 * @internal Internal header — not installed, not part of the public API.
 * Clock helper plus the packed per-slice health accessors, shared by the core
 * LAM TU and the slice / health TUs after the split.
 */

#pragma once

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Types.hpp"

#include <ctime>

namespace EtherCAT {

inline uint64_t monoNowNs() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull
         + static_cast<uint64_t>(ts.tv_nsec);
}

// ---- Packed per-slice health ---------------------------------------------
// Slice health is a single 64-bit word (CyclicSliceHealth::pack) written
// by the cyclic thread and read lock-free by diagnostic callers — no
// torn fields, no seqlock retry.  Single writer, so a plain
// load-modify-store is sufficient.

inline CyclicSliceHealth healthLoad(const uint64_t& p) {
    return CyclicSliceHealth::unpack(
        __atomic_load_n(&p, __ATOMIC_ACQUIRE));
}
inline void healthStore(uint64_t& p, const CyclicSliceHealth& h) {
    __atomic_store_n(&p, h.pack(), __ATOMIC_RELEASE);
}
/// Change last_status only (Stale marking keeps wkc/consec for the
/// following timeout classification).
inline void healthMarkStatus(uint64_t& p, CyclicSliceStatus st) {
    auto h = healthLoad(p);
    h.last_status = st;
    healthStore(p, h);
}
/// Unconditional failure: set status and bump the failure streak.
inline void healthFail(uint64_t& p, CyclicSliceStatus st) {
    auto h = healthLoad(p);
    h.last_status = st;
    ++h.consecutive_failures;
    healthStore(p, h);
}
/// Timeout: keeps a Stale classification (the late echo IS the story)
/// but still bumps the consecutive-failure streak.  `expected` refreshes
/// the published expectation so diagnostics stay current.
inline void healthTimeout(uint64_t& p, uint16_t expected) {
    auto h = healthLoad(p);
    if (h.last_status != CyclicSliceStatus::Stale)
        h.last_status = CyclicSliceStatus::Timeout;
    h.expected_wkc = expected;
    ++h.consecutive_failures;
    healthStore(p, h);
}
/// Full outcome write: status + WKC pair + failure-streak bookkeeping.
inline void markRunStatus(uint64_t& p, CyclicSliceStatus st,
                          uint16_t wkc = 0, uint16_t expected = 0xFFFF) {
    auto h = healthLoad(p);
    h.last_status  = st;
    h.last_wkc     = wkc;
    h.expected_wkc = expected;
    h.consecutive_failures = (st == CyclicSliceStatus::Ok)
                                 ? 0 : h.consecutive_failures + 1;
    healthStore(p, h);
}

} // namespace EtherCAT

