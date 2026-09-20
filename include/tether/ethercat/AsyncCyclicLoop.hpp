#pragma once

/**
 * @file AsyncCyclicLoop.hpp
 * @brief Event-driven realtime loop — RxPDO send-on-change, TxPDO collect
 *        either per-send or on an independent clock.
 *
 * For producers that emit RxPDO data asynchronously (steered by an
 * external clock or event), unlike CyclicExecutive's free-running
 * deadline.  The producer calls ProcessImage::commitOutputs() +
 * triggerSend(); this loop blocks on the send-request word via C++20
 * atomic wait (shared futex when the image is shm-backed — a
 * process-external producer wakes the master's loop directly).
 *
 * TxPDO collection policy (CollectMode):
 *
 *   OnSend   — every triggered send is immediately followed by a collect:
 *              one wire exchange per producer edge.  Use when inputs are
 *              only interesting in response to outputs.
 *   Periodic — collects run on their own deadline (e.g. 1 kHz),
 *              independent of sends: inputs stream at a fixed rate while
 *              outputs fire on change.  Both actions serialize on the
 *              single loop thread — the channel stays lock-free.
 *
 * Optional max_idle_ns keeps the wire alive in OnSend mode: when no
 * producer edge arrives for that long, a collect-only exchange ticks so
 * slaves' SyncManager/DC watchdogs don't starve.  Drives needing strict
 * periodic DC belong on CyclicExecutive — async + DC is a mismatch by
 * construction.
 */

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

#include "tether/hal/IThreading.hpp"
#include "tether/ethercat/CyclicExecutive.hpp"   // SchedClass

namespace EtherCAT {

class ProcessImage;

class AsyncCyclicLoop {
public:
    using TaskFn     = std::function<bool()>;
    using TimeFunc   = std::function<uint64_t()>;
    using SchedClass = CyclicExecutive::SchedClass;

    /// How TxPDOs are collected relative to triggered RxPDO sends.
    enum class CollectMode : uint8_t {
        OnSend,    ///< collect with every send (TxPDO-synchronous)
        Periodic,  ///< collect on collect_period_us, independent of sends
    };

    struct Config {
        CollectMode collect_mode      = CollectMode::OnSend;
        uint32_t    collect_period_us = 1000;   ///< Periodic tick (µs)

        /// Send rate limiter: triggers closer than this coalesce into one
        /// trailing send carrying the latest committed image.  0 = a send
        /// per trigger.
        uint32_t min_send_interval_ns = 0;

        /// OnSend only: collect at least this often while no triggers
        /// arrive — a wire keep-alive so slaves' watchdogs stay fed.
        /// 0 = no keep-alive.
        uint32_t max_idle_ns = 0;

        // ---- RT plumbing (same knobs as CyclicExecutive) ----
        int        priority       = 80;
        SchedClass sched_class    = SchedClass::Fifo;
        /// SCHED_DEADLINE budget (ns); 0 → derive from collect period.
        uint64_t   dl_runtime_ns  = 0;
        uint64_t   dl_deadline_ns = 0;
        int        cpu_affinity   = -1;
        size_t     stack_size     = 262144;
        uint32_t   stack_prefault_bytes = 128 * 1024;
        bool       low_timer_slack = true;
        bool       continue_on_error = true;
    };

    struct Stats {
        uint64_t wakes            = 0;   ///< trigger wakes observed
        uint64_t sends            = 0;
        uint64_t send_errors      = 0;
        uint64_t sends_coalesced  = 0;   ///< triggers merged by rate limiter
        uint64_t collects         = 0;
        uint64_t collect_errors   = 0;
        uint64_t idle_collects    = 0;   ///< max_idle keep-alive collects
        uint32_t max_send_work_ns    = 0;///< longest send burst
        uint32_t max_collect_work_ns = 0;///< longest collect burst
    };

    /**
     * @param image        Process image whose send-request word drives the
     *                     loop (producer calls its triggerSend()).
     * @param send         One RxPDO send (e.g. PDOManager::cyclicSend).
     *                     May be empty → trigger wakes are counted only.
     * @param collect      One TxPDO collect (e.g. PDOManager::cyclicCollect).
     * @param time_source  Monotonic ns (may be null → platform clock).
     */
    AsyncCyclicLoop(ProcessImage& image, TaskFn send, TaskFn collect,
                    TimeFunc time_source, const Config& config);
    ~AsyncCyclicLoop();

    AsyncCyclicLoop(const AsyncCyclicLoop&) = delete;
    AsyncCyclicLoop& operator=(const AsyncCyclicLoop&) = delete;

    bool start();
    void stop();
    bool isRunning() const { return running_.load(std::memory_order_acquire); }

    Stats getStats() const;

private:
    void main();
    bool doSend();
    bool doCollect();
    uint64_t nowNs() const { return time_source_ ? time_source_()
                                                 : platformNowNs(); }
    static uint64_t platformNowNs();

    ProcessImage& image_;
    TaskFn        send_;
    TaskFn        collect_;
    TimeFunc      time_source_;
    Config        config_;

    std::unique_ptr<HAL::IThread> thread_;
    std::atomic<bool>             running_{false};
    /// Send-request seq sampled in start() — the loop's first waitSend
    /// compares against it so a trigger fired after start() returns is
    /// never absorbed into the initial snapshot (lost wakeup).
    uint32_t                      start_seq_ = 0;

    // Stats — written by the loop thread, read via getStats()
    std::atomic<uint64_t> wakes_{0};
    std::atomic<uint64_t> sends_{0};
    std::atomic<uint64_t> send_errors_{0};
    std::atomic<uint64_t> sends_coalesced_{0};
    std::atomic<uint64_t> collects_{0};
    std::atomic<uint64_t> collect_errors_{0};
    std::atomic<uint64_t> idle_collects_{0};
    std::atomic<uint32_t> max_send_work_ns_{0};
    std::atomic<uint32_t> max_collect_work_ns_{0};
};

} // namespace EtherCAT
