/**
 * @file AsyncCyclicLoop.cpp
 * @brief Event-driven realtime loop — send on producer trigger, collect
 *        per-send or on an independent tick.
 *
 * Hot path per send: one waitSend() wake (atomic wait / shared futex) +
 * the send and optional collect calls — no polling, no free-running
 * deadline.  Single-threaded, so channel TX/RX stay serialized without
 * locking even in Periodic mode.
 */

#include "tether/ethercat/AsyncCyclicLoop.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/platform/CpuIsolation.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"
#include "logging/Logger.hpp"

#include <algorithm>
#include <ctime>

namespace EtherCAT {

static const char* TAG = "async_loop";

AsyncCyclicLoop::AsyncCyclicLoop(ProcessImage& image, TaskFn send,
                                 TaskFn collect, TaskFn dc_sync,
                                 TimeFunc time_source,
                                 const Config& config)
    : image_(image)
    , send_(std::move(send))
    , collect_(std::move(collect))
    , dc_sync_(std::move(dc_sync))
    , time_source_(std::move(time_source))
    , config_(config)
{
    if (config_.collect_period_us == 0) config_.collect_period_us = 1;
}

AsyncCyclicLoop::~AsyncCyclicLoop() {
    stop();
}

uint64_t AsyncCyclicLoop::platformNowNs() {
    return static_cast<uint64_t>(
        Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
}

uint64_t AsyncCyclicLoop::monoNowNs() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull
         + static_cast<uint64_t>(ts.tv_nsec);
}

// ============================================================================
// Lifecycle
// ============================================================================

bool AsyncCyclicLoop::start() {
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        TETHER_LOGW(TAG, "AsyncCyclicLoop already running");
        return false;
    }
    // Sample before spawning the thread: the loop's first waitSend()
    // compares against this, so any triggerSend() after start() returns
    // observes a changed seq and fires — never swallowed by the snapshot.
    start_seq_ = image_.sendSeq();

    HAL::ThreadConfig cfg;
    cfg.name = "rt_async";
    cfg.stackSize = config_.stack_size;
    cfg.priority = HAL::ThreadPriority::Realtime;
    cfg.useRealtimeScheduling = true;
    cfg.cpuAffinity = config_.cpu_affinity;

    thread_ = HAL::getThreadingFactory().createThread(cfg);
    if (!thread_ || thread_->start([this]() { main(); }) != HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to start async loop thread");
        thread_.reset();
        running_.store(false, std::memory_order_release);
        return false;
    }

    // Q23: DC sync runs as its own deadline-driven RT thread — periodic
    // by nature, independent of the trigger/send loop.
    if (config_.dc_interval_us > 0) {
        if (!dc_sync_) {
            TETHER_LOGW(TAG, "dc_interval_us set but no dc_sync task — "
                             "DC thread disabled");
        } else {
            dc_timer_ = Platform::createDeadlineTimer();
            HAL::ThreadConfig dcfg;
            dcfg.name = "rt_async_dc";
            dcfg.stackSize = config_.stack_size;
            dcfg.priority = HAL::ThreadPriority::Realtime;
            dcfg.useRealtimeScheduling = true;
            dcfg.cpuAffinity = config_.dc_cpu_affinity;
            dc_thread_ = HAL::getThreadingFactory().createThread(dcfg);
            if (!dc_timer_ || !dc_thread_ ||
                dc_thread_->start([this]() { dcMain(); }) != HAL::Error::OK) {
                TETHER_LOGE(TAG, "Failed to start async DC thread");
                stop();
                return false;
            }
        }
    }

    TETHER_LOGI(TAG, "AsyncCyclicLoop started: collect={}, min_send={}ns, "
                "idle={}ns, dc={}us",
                config_.collect_mode == CollectMode::OnSend ? "on-send"
                    : "periodic",
                config_.min_send_interval_ns, config_.max_idle_ns,
                config_.dc_interval_us);
    return true;
}

void AsyncCyclicLoop::stop() {
    running_.store(false, std::memory_order_release);
    // Wake the loop out of waitSend — a bump that the (stopping) loop
    // abandons on its running_ re-check, no send fires.
    image_.triggerSend();
    if (thread_) { thread_->requestStop(); thread_->join(); thread_.reset(); }
    if (dc_timer_)  dc_timer_->requestStop();
    if (dc_thread_) { dc_thread_->requestStop(); dc_thread_->join();
                      dc_thread_.reset(); }
    dc_timer_.reset();
}

// ============================================================================
// Loop thread
// ============================================================================

bool AsyncCyclicLoop::doSend() {
    if (!send_) { sends_.fetch_add(1, std::memory_order_relaxed); return true; }
    const uint64_t t0 = platformNowNs();
    const bool ok = send_();
    const uint64_t dt = platformNowNs() - t0;
    uint32_t cur = max_send_work_ns_.load(std::memory_order_relaxed);
    while (dt > cur &&
           !max_send_work_ns_.compare_exchange_weak(
               cur, static_cast<uint32_t>(dt), std::memory_order_relaxed)) {}
    if (ok) sends_.fetch_add(1, std::memory_order_relaxed);
    else    send_errors_.fetch_add(1, std::memory_order_relaxed);
    return ok;
}

bool AsyncCyclicLoop::doCollect() {
    if (!collect_) { collects_.fetch_add(1, std::memory_order_relaxed); return true; }
    const uint64_t t0 = platformNowNs();
    const bool ok = collect_();
    const uint64_t dt = platformNowNs() - t0;
    uint32_t cur = max_collect_work_ns_.load(std::memory_order_relaxed);
    while (dt > cur &&
           !max_collect_work_ns_.compare_exchange_weak(
               cur, static_cast<uint32_t>(dt), std::memory_order_relaxed)) {}
    if (ok) collects_.fetch_add(1, std::memory_order_relaxed);
    else    collect_errors_.fetch_add(1, std::memory_order_relaxed);
    return ok;
}

void AsyncCyclicLoop::main() {
    // Deferred logging: log calls on this thread only enqueue bounded
    // events — no heap, no mutex, no printf on the send/collect path.
    Tether::Platform::Logger::setThreadDeferred(true);
    if (config_.cpu_affinity >= 0) {
        // Join the exclusive cpuset for the claimed CPU first (member-
        // ship grants the exclusivity), then re-apply the pin — the
        // spawn-time pthread_setaffinity_np may have raced/failed.
        Tether::Platform::CpuIsolation::instance().adoptCurrentThread(
            config_.cpu_affinity);
        if (!Tether::Platform::setCurrentThreadAffinity(config_.cpu_affinity)) {
            TETHER_LOGW(TAG, "cyclic thread: CPU {} pin failed — unpinned",
                        config_.cpu_affinity);
        }
    }
    if (config_.low_timer_slack) {
        Tether::Platform::setCurrentThreadTimerSlack(1);
    }
    if (config_.stack_prefault_bytes > 0) {
        Tether::Platform::prefaultCurrentStack(
            std::min<uint32_t>(config_.stack_prefault_bytes,
                               static_cast<uint32_t>(config_.stack_size)
                                   - 16 * 1024));
    }

    // Auto: FIFO only on a hard-isolated CPU — on a shared CPU the global
    // RT-throttle budget can dequeue the loop for ~50 ms (see exec).
    const bool isolated =
        config_.cpu_affinity >= 0
            ? Tether::Platform::cpuIsHardIsolated(config_.cpu_affinity)
            : Tether::Platform::currentThreadRunsIsolatedOnly();
    SchedClass cls = config_.sched_class;
    if (cls == SchedClass::Auto) {
        cls = isolated ? SchedClass::Fifo : SchedClass::Deadline;
        TETHER_LOGI(TAG, "sched_class=Auto → {} (cyclic CPU {} {} isolated)",
                    isolated ? "SCHED_FIFO" : "SCHED_DEADLINE",
                    config_.cpu_affinity,
                    isolated ? "is" : "is NOT");
    }

    if (config_.disable_rt_throttle) {
        if (Tether::Platform::disableRtThrottle()) {
            TETHER_LOGI(TAG, "sched_rt_runtime_us=-1 — global RT throttle "
                             "disabled");
            if (!isolated)
                TETHER_LOGW(TAG, "RT throttle disabled on a NON-isolated "
                                 "CPU — a runaway RT task can starve the "
                                 "core");
        } else {
            TETHER_LOGW(TAG, "disable_rt_throttle: write to "
                             "sched_rt_runtime_us failed (needs root)");
        }
    }
    if (cls == SchedClass::Fifo && !isolated &&
        Tether::Platform::rtThrottleRuntimeUs() >= 0) {
        TETHER_LOGW(TAG, "SCHED_FIFO on a non-isolated CPU with the kernel "
                         "RT throttle armed — RT-group overruns can dequeue "
                         "the loop for ~50 ms; prefer Deadline/Auto or "
                         "CPU isolation");
    }
    if (cls == SchedClass::Deadline && config_.in_cycle_spin_ns > 0) {
        TETHER_LOGW(TAG, "SCHED_DEADLINE with {} ns/cycle in-cycle spin "
                         "(rx_spin_ns/slot_spin_ns): every spin ns is "
                         "charged against the CBS runtime — prefer 0 on "
                         "shared CPUs", config_.in_cycle_spin_ns);
    }

    bool rt = false;
    if (cls == SchedClass::Deadline) {
        const uint64_t period_ns =
            static_cast<uint64_t>(config_.collect_period_us) * 1000ULL;
        const uint64_t runtime = config_.dl_runtime_ns
            ? config_.dl_runtime_ns : period_ns / 2;
        const uint64_t deadline = config_.dl_deadline_ns
            ? config_.dl_deadline_ns : period_ns;
        rt = Tether::Platform::setCurrentThreadDeadline(runtime, deadline,
                                                        period_ns);
        if (!rt) {
            TETHER_LOGW(TAG, "SCHED_DEADLINE unavailable — falling back "
                             "to SCHED_FIFO");
        }
    }
    if (!rt) {
        rt = Tether::Platform::setCurrentThreadRealtime(config_.priority);
    }
    if (!rt) {
        TETHER_LOGW(TAG, "Async loop could not acquire realtime "
                         "scheduling; running with normal scheduling");
    }

    const bool periodic = config_.collect_mode == CollectMode::Periodic;
    const uint64_t collect_ns =
        static_cast<uint64_t>(config_.collect_period_us) * 1000ULL;
    const uint64_t idle_ns = config_.max_idle_ns;
    const uint64_t min_send_ns = config_.min_send_interval_ns;

    uint64_t collect_due  = periodic ? nowNs() + collect_ns : UINT64_MAX;
    // Idle keep-alive only matters when there's no periodic collect
    // guaranteeing wire activity.
    uint64_t idle_due     = (!periodic && idle_ns) ? nowNs() + idle_ns
                                                   : UINT64_MAX;
    uint64_t send_allowed = 0;      // rate limiter: earliest next send
    bool     pending_send = false;  // coalesced trigger awaiting its window
    uint32_t last         = start_seq_;   // sampled in start(), see there

    while (running_.load(std::memory_order_acquire)) {
        const uint64_t now = nowNs();
        // Wake at whichever scheduled action is earliest — collect tick,
        // pending-send window, or idle keep-alive — or on a new trigger.
        uint64_t next = collect_due;
        if (pending_send && send_allowed < next) next = send_allowed;
        if (idle_due < next)                       next = idle_due;

        // waitSend takes a *relative* timeout — a duration is agnostic to
        // this loop's clock domain (platform clock vs DC time source).
        const uint64_t budget = (next == UINT64_MAX) ? UINT64_MAX
                              : (next > now ? next - now : 0);
        const bool trig = image_.waitSend(last, budget);
        if (!running_.load(std::memory_order_acquire)) break;
        const uint64_t t = nowNs();
        bool activity = false;

        if (trig) {
            wakes_.fetch_add(1, std::memory_order_relaxed);
            const uint32_t seq = image_.sendSeq();
            // Back-to-back triggers consumed by one wake still merge into a
            // single send — count them as coalesced alongside rate-limited
            // merges.
            const uint32_t merged = seq - last;
            last = seq;
            // Q24: trigger→send latency.  The stamp is CLOCK_MONOTONIC —
            // the newest coalesced trigger's queueing delay.
            const uint64_t stamp = image_.sendStampNs();
            if (stamp) {
                const uint64_t lat = monoNowNs() - stamp;
                uint32_t cur =
                    max_send_latency_ns_.load(std::memory_order_relaxed);
                while (lat > cur &&
                       !max_send_latency_ns_.compare_exchange_weak(
                           cur, static_cast<uint32_t>(
                                    lat > UINT32_MAX ? UINT32_MAX : lat),
                           std::memory_order_relaxed)) {}
            }
            if (t >= send_allowed) {
                // `merged` triggers collapsed into this one send.
                if (merged > 1)
                    sends_coalesced_.fetch_add(merged - 1,
                                               std::memory_order_relaxed);
                send_allowed = t + min_send_ns;
                activity |= doSend();
                if (config_.collect_mode == CollectMode::OnSend)
                    activity |= doCollect();
            } else {
                // Rate-limited: this wake's triggers produce no send of their
                // own — they merge into the trailing send.  When a pending
                // send already exists, all `merged` fold into it; otherwise
                // one leads the pending send and merged-1 are coalesced.
                sends_coalesced_.fetch_add(pending_send ? merged : merged - 1,
                                           std::memory_order_relaxed);
                pending_send = true;
            }
        }
        if (pending_send && t >= send_allowed) {
            pending_send = false;
            send_allowed = t + min_send_ns;
            activity |= doSend();
            if (config_.collect_mode == CollectMode::OnSend)
                activity |= doCollect();
        }
        if (periodic && t >= collect_due) {
            // Collect lateness = this loop's wake overshoot (Nanosleep +
            // reschedule delay past the deadline).
            const uint64_t late = t - collect_due;
            uint32_t cur_late =
                max_collect_late_ns_.load(std::memory_order_relaxed);
            while (late > cur_late &&
                   !max_collect_late_ns_.compare_exchange_weak(
                       cur_late,
                       static_cast<uint32_t>(
                           late > UINT32_MAX ? UINT32_MAX : late),
                       std::memory_order_relaxed)) {}
            activity |= doCollect();
            // Stay phase-aligned: advance by whole periods, never drift.
            collect_due += collect_ns;
            if (collect_due <= t) collect_due = t + collect_ns;
        }
        if (!periodic && idle_ns && t >= idle_due) {
            activity |= doCollect();
            idle_collects_.fetch_add(1, std::memory_order_relaxed);
        }
        // Any wire activity resets the idle keep-alive deadline.
        if (activity && idle_ns) idle_due = t + idle_ns;
    }

    running_.store(false, std::memory_order_release);
}

// ============================================================================
// DC thread — independent periodic sync emitter (Q23)
// ============================================================================

void AsyncCyclicLoop::dcMain() {
    Tether::Platform::Logger::setThreadDeferred(true);
    if (config_.dc_cpu_affinity >= 0) {
        Tether::Platform::CpuIsolation::instance().adoptCurrentThread(
            config_.dc_cpu_affinity);
        if (!Tether::Platform::setCurrentThreadAffinity(
                config_.dc_cpu_affinity)) {
            TETHER_LOGW(TAG, "DC thread: CPU {} pin failed — unpinned",
                        config_.dc_cpu_affinity);
        }
    }
    const bool rt =
        Tether::Platform::setCurrentThreadRealtime(config_.dc_priority);
    if (!rt) {
        TETHER_LOGW(TAG, "Async DC thread could not acquire realtime "
                         "scheduling; running with normal scheduling");
    }
    const uint64_t period_ns =
        static_cast<uint64_t>(config_.dc_interval_us) * 1000ULL;
    if (!dc_timer_->start(period_ns)) {
        TETHER_LOGE(TAG, "Async DC deadline timer start failed");
        return;
    }
    Platform::IDeadlineTimer::Tick tick{};
    while (running_.load(std::memory_order_acquire)) {
        if (!dc_timer_->waitNext(tick)) break;
        if (dc_sync_ && dc_sync_()) {
            dc_sync_count_.fetch_add(1, std::memory_order_relaxed);
        } else {
            dc_sync_errors_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

AsyncCyclicLoop::Stats AsyncCyclicLoop::getStats() const {
    Stats s;
    s.wakes              = wakes_.load(std::memory_order_relaxed);
    s.sends              = sends_.load(std::memory_order_relaxed);
    s.send_errors        = send_errors_.load(std::memory_order_relaxed);
    s.sends_coalesced    = sends_coalesced_.load(std::memory_order_relaxed);
    s.collects           = collects_.load(std::memory_order_relaxed);
    s.collect_errors     = collect_errors_.load(std::memory_order_relaxed);
    s.idle_collects      = idle_collects_.load(std::memory_order_relaxed);
    s.max_send_work_ns    = max_send_work_ns_.load(std::memory_order_relaxed);
    s.max_collect_work_ns = max_collect_work_ns_.load(std::memory_order_relaxed);
    s.max_send_latency_ns = max_send_latency_ns_.load(std::memory_order_relaxed);
    s.max_collect_late_ns =
        max_collect_late_ns_.load(std::memory_order_relaxed);
    s.dc_sync_count      = dc_sync_count_.load(std::memory_order_relaxed);
    s.dc_sync_errors     = dc_sync_errors_.load(std::memory_order_relaxed);
    return s;
}

} // namespace EtherCAT
