/**
 * @file TxFailureDiagnostics.hpp
 * @brief Wire-level TX failure probe (sysfs/ioctl) + non-RT escalation worker.
 *
 * @internal Internal header — not installed, not part of the public API.
 *
 * `probe()` explains why AF_PACKET sends are failing: interface admin/link
 * state, carrier, a pending socket error, and the tx/rx error counters, with
 * a plain-language DIAGNOSIS appended.  Linux only — returns an empty string
 * elsewhere or when the fd is not a diagnosable AF_PACKET socket.
 *
 * The worker is the escalation path for the real-time exchange loops: the
 * cyclic thread calls noteSendFailure() on every failed send; every 10th
 * consecutive failure re-arms a background thread that invokes the probe
 * callback.  The worker exits after 1 s without a re-arm and is respawned by
 * the next streak.  The RT-thread side is allocation- and lock-free.
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

    using ProbeFn = std::function<std::string()>;

    /// @param probe     invoked by the worker to produce the report
    /// @param log_tag   logging tag used when emitting the report
    explicit TxFailureDiagnostics(ProbeFn probe, const char* log_tag = "ethercat");
    ~TxFailureDiagnostics();

    TxFailureDiagnostics(const TxFailureDiagnostics&) = delete;
    TxFailureDiagnostics& operator=(const TxFailureDiagnostics&) = delete;

    /// Count a failed send; every 10th consecutive failure re-arms the worker
    /// (spawning it if it had self-exited).
    void noteSendFailure();
    /// Reset the consecutive-failure streak after a successful send.
    void noteSuccess() { send_fail_streak_.store(0, std::memory_order_relaxed); }
    /// Stop and join the worker (the destructor does this too).
    void stop();
    /// Worker thread spawns so far — one per send-failure streak episode.
    uint32_t spawns() const { return spawns_.load(std::memory_order_relaxed); }

private:
    void workerMain();

    ProbeFn               probe_;
    const char*           log_tag_;
    std::atomic<uint32_t> send_fail_streak_{0};
    std::atomic<bool>     requested_{false};
    std::atomic<bool>     running_{false};
    std::atomic<bool>     stop_{false};
    std::atomic<uint32_t> spawns_{0};
    std::thread           thread_;
};

} // namespace EtherCAT
