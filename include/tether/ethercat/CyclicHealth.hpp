/**
 * @file CyclicHealth.hpp
 * @brief Protocol-level cyclic health snapshot.
 *
 * One authoritative view of whether cyclic traffic is actually making it
 * around the bus — deliberately NOT derived from kernel packet-drop counters,
 * which cannot distinguish intentional BPF filtering or correctly discarded
 * stray replies from real wire loss:
 *
 *   - wire_loss      — datagrams that did not return before their cycle
 *                      deadline (packet didn't circulate)
 *   - wkc_errors     — replies that returned with a bad working counter
 *                      (slave dropped out of OP, FMMU hole)
 *   - rx_bank_drops  — frames that PASSED the socket filter but found no
 *                      free receive bank (real local exhaustion)
 *   - unrouted_datagrams / rx_queue_overflow — stray replies with no waiter,
 *                      correctly discarded: informational, never a fault by
 *                      themselves
 *
 * image_slices[] carries per-slice expected-vs-last WKC and the last collect
 * outcome; pdo_slice_health does the same for user-defined PDO slices.
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

struct CyclicHealth {
    // --- executor level ---
    uint64_t cycles             = 0;  ///< cyclic_loop_ cycle_count
    uint64_t exchange_errors    = 0;  ///< cycles whose exchange returned false
    uint64_t missed_deadlines   = 0;
    // --- protocol level (LogicalAddressManager) ---
    uint32_t exchanges_ok       = 0;
    uint32_t wire_loss          = 0;  ///< replies that never came back
    uint32_t wkc_errors         = 0;
    uint32_t stale_responses    = 0;
    uint32_t send_errors        = 0;
    /// Wire RTT of in-generation replies (emit -> kernel RX stamp),
    /// in microseconds.  Splits "late" from "lost": stale_responses
    /// counts deposits arriving after their deadline (late echo of a
    /// previous send), so wire_loss - stale approximates true
    /// non-return.
    uint32_t rtt_us_min         = 0;
    uint32_t rtt_us_avg         = 0;
    uint32_t rtt_us_max         = 0;
    uint32_t rtt_samples        = 0;
    // --- channel level (real local loss) ---
    uint64_t rx_bank_drops      = 0;  ///< rxPoll couldn't emit (banks held)
    /// Kernel-side drops on the cyclic socket (tp_drops share seen by
    /// this caller) — frames that passed the filter but found no free
    /// ring slot.  Rising while rx_bank_drops stays 0 = the ring
    /// wasn't drained; rising together with wire_loss = real loss.
    uint64_t kernel_rx_drops    = 0;
    // --- transport level (informational — correctly discarded strays) ---
    uint64_t unrouted_datagrams = 0;
    uint64_t rx_queue_overflow  = 0;
    // --- datapath liveness (diagnose "is the collect running?") ---
    uint64_t collect_calls      = 0;  ///< split-collect task invocations
    uint64_t wait_calls         = 0;  ///< datapath wait invocations
    uint64_t dispatch_frames    = 0;  ///< frames consumed from the ring
    uint64_t dispatch_unrouted  = 0;  ///< consumed frames with no slot
    // --- per-slice detail ---
    /// Slices whose last outcome was Stale for many consecutive
    /// cycles — the reply RTT exceeds one whole period (each cycle
    /// only ever sees the previous cycle's echo).  Distinct from a
    /// transient stale, which the gen-guard retry absorbs.
    uint8_t slices_stuck_stale  = 0;
    uint8_t image_slice_count   = 0;
    std::array<CyclicSliceHealth, kNumCyclicSlots> image_slices{};
    std::vector<CyclicSliceHealth> pdo_slice_health;

    /// Multi-line human-readable dump for failure logging: one
    /// counters line, a stuck-stale warning line when any slice only
    /// ever sees the previous cycle's echo, then one line per slice
    /// whose last outcome was not Ok.  Allocation — non-RT callers.
    std::string describe() const;
};

} // namespace EtherCAT
