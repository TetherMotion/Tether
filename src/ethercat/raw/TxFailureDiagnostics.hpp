/**
 * @file TxFailureDiagnostics.hpp
 * @brief Non-RT exchange-health monitor + wire-level TX failure probe.
 *
 * @internal Internal header — not installed, not part of the public API.
 *
 * `probe()` explains why AF_PACKET sends are failing: interface admin/link
 * state, carrier, a pending socket error, and the tx/rx error counters, with
 * a plain-language DIAGNOSIS appended.  Linux only — returns an empty string
 * elsewhere or when the fd is not a diagnosable AF_PACKET socket.
 *
 * The worker is the ONLY place exchange errors are logged: the real-time
 * exchange loops never log — they bump atomic counters via noteSendFailure()
 * / noteTimeout() / noteStall() / noteNoResponseSlot().  The monitor thread
 * is started by LogicalAddressManager::init() (executor startup), polls the
 * counters every 10 ms, and emits each error category rate-limited to 4 Hz
 * with a "(N suppressed)" tail.  Once the consecutive-send-failure streak
 * reaches 10 it additionally invokes the wire probe — first emit per streak,
 * then on verdict change or once per second — and a consecutive-timeout
 * streak crossing a multiple of the escalation threshold fires the ring
 * probe callback.  All RT-side notifiers are allocation- and lock-free.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace EtherCAT {

class TxFailureDiagnostics {
public:
    /// Format a diagnostic report for an AF_PACKET `fd`.  Returns an empty
    /// string when the fd cannot be diagnosed (non-Linux, not a bound
    /// AF_PACKET socket, or no fd).
    static std::string probe(int fd);

    using ProbeFn       = std::function<std::string()>;  ///< wire probe → report
    using ErrnoFn       = std::function<int()>;          ///< last sendto errno
    /// Ring probe on consecutive timeouts — returns a short verdict like
    /// "ALIVE (AL_STATUS=0x0002)", or "" to emit nothing.  Runs a transport
    /// read from the monitor thread (async transport ops are legal there).
    using EscalationFn  = std::function<std::string()>;

    /// @param probe       wire probe → full report (may be null)
    /// @param errno_fn    errno of the last failed sendto (may be null)
    /// @param escalation  ring probe for timeout streaks (may be null)
    /// @param log_tag     logging tag used when emitting
    TxFailureDiagnostics(ProbeFn probe, ErrnoFn errno_fn,
                         EscalationFn escalation,
                         const char* log_tag = "ethercat");
    ~TxFailureDiagnostics();

    TxFailureDiagnostics(const TxFailureDiagnostics&) = delete;
    TxFailureDiagnostics& operator=(const TxFailureDiagnostics&) = delete;

    /// Spawn the persistent monitor thread.  Idempotent.  Called by
    /// LogicalAddressManager::init().
    void start();
    /// Stop and join the worker (the destructor does this too).
    void stop();

    // ----- RT-thread notifiers: atomic bumps only, never log -----

    /// A cyclic sendSingleDatagram failed.  `mask` is the slave mask for
    /// exchangeLRWForSlaves failures (0 = whole-image exchangeLRW).
    void noteSendFailure(uint32_t mask = 0);
    /// No pre-registered response slot was free for the exchange.
    void noteNoResponseSlot();
    /// An exchange timed out waiting for its response.  `what` must point
    /// to a string literal (or other immortal storage).
    void noteTimeout(const char* what);
    /// An exchange response carried WKC=0 — slaves aren't processing.
    void noteWkcZero(const char* what);
    /// exchangeLRW call gap exceeded the stall threshold.
    void noteStall(int64_t gap_ns);
    /// A sendSingleDatagram got on the wire — resets the send-fail streak.
    /// (A successful send can still be followed by a response timeout.)
    void noteSendOk();
    /// A full exchange succeeded — resets both streaks.
    void noteSuccess();

    /// Consecutive-timeout count that fires the ring probe (default 64;
    /// 0 disables).
    void setEscalateAfterTimeouts(uint32_t n) {
        escalate_after_.store(n ? n : 0, std::memory_order_relaxed);
    }

    /// Worker thread spawns so far — one per start().
    uint32_t spawns() const { return spawns_.load(std::memory_order_relaxed); }

private:
    void workerMain();

    ProbeFn       probe_;
    ErrnoFn       errno_fn_;
    EscalationFn  escalation_fn_;
    const char*   log_tag_;

    // Pending counts — incremented by the RT thread, drained by the worker.
    std::atomic<uint32_t>    send_fails_{0};
    std::atomic<uint32_t>    no_slot_{0};
    std::atomic<uint32_t>    timeouts_{0};
    std::atomic<uint32_t>    wkc_zero_{0};
    std::atomic<uint32_t>    stalls_{0};
    std::atomic<int64_t>     last_stall_gap_ns_{0};
    std::atomic<const char*> last_timeout_op_{"exchangeLRW"};
    std::atomic<const char*> last_wkc_op_{"exchangeLRW"};
    std::atomic<uint32_t>    last_send_fail_mask_{0};

    // Streaks — for probe/escalation gating.
    std::atomic<uint32_t> consec_send_fails_{0};
    std::atomic<uint32_t> consec_timeouts_{0};

    std::atomic<uint32_t> escalate_after_{64};
    std::atomic<bool>     running_{false};
    std::atomic<bool>     stop_{false};
    std::atomic<uint32_t> spawns_{0};
    std::thread           thread_;
};

} // namespace EtherCAT
