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
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"
#include "logging/Logger.hpp"

#include <algorithm>

namespace EtherCAT {

static const char* TAG = "async_loop";

AsyncCyclicLoop::AsyncCyclicLoop(ProcessImage& image, TaskFn send,
                                 TaskFn collect, TimeFunc time_source,
                                 const Config& config)
    : image_(image)
    , send_(std::move(send))
    , collect_(std::move(collect))
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

    TETHER_LOGI(TAG, "AsyncCyclicLoop started: collect={}, min_send={}ns, "
                "idle={}ns",
                config_.collect_mode == CollectMode::OnSend ? "on-send"
                    : "periodic",
                config_.min_send_interval_ns, config_.max_idle_ns);
    return true;
}

void AsyncCyclicLoop::stop() {
    running_.store(false, std::memory_order_release);
    // Wake the loop out of waitSend — a bump that the (stopping) loop
    // abandons on its running_ re-check, no send fires.
    image_.triggerSend();
    if (thread_) { thread_->requestStop(); thread_->join(); thread_.reset(); }
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
    if (config_.low_timer_slack) {
        Tether::Platform::setCurrentThreadTimerSlack(1);
    }
    if (config_.stack_prefault_bytes > 0) {
        Tether::Platform::prefaultCurrentStack(
            std::min<uint32_t>(config_.stack_prefault_bytes,
                               static_cast<uint32_t>(config_.stack_size)
                                   - 16 * 1024));
    }

    bool rt = false;
    if (config_.sched_class == SchedClass::Deadline) {
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
            if (merged > 1)
                sends_coalesced_.fetch_add(merged - 1,
                                           std::memory_order_relaxed);
            if (t >= send_allowed) {
                send_allowed = t + min_send_ns;
                activity |= doSend();
                if (config_.collect_mode == CollectMode::OnSend)
                    activity |= doCollect();
            } else {
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
    return s;
}

} // namespace EtherCAT
