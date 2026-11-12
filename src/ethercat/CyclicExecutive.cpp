/**
 * @file CyclicExecutive.cpp
 * @brief Single-thread, deadline-driven cyclic executive.
 *
 * Hot path per cycle: one virtual waitNext() + one clock_nanosleep — no
 * producer timer thread, no event, no mutex, no allocation.
 */

#include "tether/ethercat/CyclicExecutive.hpp"
#include "tether/platform/CpuIsolation.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"
#include "logging/Logger.hpp"

#include <algorithm>
#include <vector>

namespace EtherCAT {

static const char* TAG = "cyclic_exec";

namespace {
constexpr size_t kIdxPre      = 0;
constexpr size_t kIdxExchange = 1;
constexpr size_t kIdxPost     = 2;
constexpr size_t kIdxMotion   = 3;
constexpr size_t kIdxDiag     = 4;
} // namespace

size_t CyclicExecutive::phaseIndex(TaskPhase p) {
    switch (p) {
        case TaskPhase::PreExchange:   return kIdxPre;
        case TaskPhase::Exchange:      return kIdxExchange;
        case TaskPhase::PostExchange:  return kIdxPost;
        case TaskPhase::MotionControl: return kIdxMotion;
        case TaskPhase::Diagnostics:   return kIdxDiag;
    }
    return kIdxDiag;
}

uint64_t CyclicExecutive::platformNowNs() {
    return static_cast<uint64_t>(
        Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
}

// ============================================================================
// Construction
// ============================================================================

CyclicExecutive::CyclicExecutive(TaskFn exchange, TaskFn dc_sync,
                                 TimeFunc time_source, const Config& config)
    : exchange_(std::move(exchange))
    , dc_sync_(std::move(dc_sync))
    , time_source_(std::move(time_source))
    , config_(config)
{
    if (config_.dc_interval_cycles == 0) config_.dc_interval_cycles = 1;
}

CyclicExecutive::~CyclicExecutive() {
    stop();
}

bool CyclicExecutive::addTask(TaskPhase phase, TaskFn fn) {
    if (!fn || phase == TaskPhase::Exchange ||
        running_.load(std::memory_order_acquire)) {
        return false;
    }
    phase_tasks_[phaseIndex(phase)].push_back(std::move(fn));
    return true;
}

std::unique_ptr<Platform::IDeadlineTimer> CyclicExecutive::makeTimer() const {
    if (config_.sleep_mode == SleepMode::HybridSpin) {
        return Platform::createHybridDeadlineTimer(
            static_cast<uint64_t>(config_.spin_window_us) * 1000ULL);
    }
    return Platform::createDeadlineTimer();
}

// ============================================================================
// Lifecycle
// ============================================================================

bool CyclicExecutive::start() {
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        TETHER_LOGW(TAG, "CyclicExecutive already running");
        return false;
    }

    jitter_monitor_ = std::make_unique<RealtimeJitterMonitor>(config_.jitter, "cyc");

    timer_ = makeTimer();
    if (!timer_) {
        TETHER_LOGE(TAG, "Failed to create deadline timer");
        running_.store(false, std::memory_order_release);
        return false;
    }

    HAL::ThreadConfig cfg;
    cfg.name = "rt_cyclic";
    cfg.stackSize = config_.stack_size;
    cfg.priority = HAL::ThreadPriority::Realtime;
    cfg.useRealtimeScheduling = true;
    cfg.cpuAffinity = config_.cpu_affinity;

    thread_ = HAL::getThreadingFactory().createThread(cfg);
    if (!thread_ || thread_->start([this]() { cyclicMain(); }) != HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to start cyclic thread");
        thread_.reset();
        timer_.reset();
        running_.store(false, std::memory_order_release);
        return false;
    }

    if (config_.dc_placement == DCPlacement::DedicatedThread && dc_sync_) {
        dc_jitter_monitor_ =
            std::make_unique<RealtimeJitterMonitor>(config_.dc_jitter, "dc");
        dc_timer_ = makeTimer();
        if (!dc_timer_) {
            TETHER_LOGE(TAG, "Failed to create DC deadline timer");
            stop();
            return false;
        }

        HAL::ThreadConfig dcfg;
        dcfg.name = "rt_dc_sync";
        dcfg.stackSize = config_.stack_size;
        dcfg.priority = HAL::ThreadPriority::Realtime;
        dcfg.useRealtimeScheduling = true;
        dcfg.cpuAffinity = config_.dc_cpu_affinity;

        dc_thread_ = HAL::getThreadingFactory().createThread(dcfg);
        if (!dc_thread_ ||
            dc_thread_->start([this]() { dcMain(); }) != HAL::Error::OK) {
            TETHER_LOGE(TAG, "Failed to start DC sync thread");
            stop();
            return false;
        }
    }

    TETHER_LOGI(TAG, "CyclicExecutive started: {} us period, dc={}",
                config_.cycle_period_us,
                config_.dc_placement == DCPlacement::DedicatedThread ? "thread" :
                config_.dc_placement == DCPlacement::Inline ? "inline" : "off");
    return true;
}

void CyclicExecutive::stop() {
    // Not guarded by was_running: the cyclic thread may have self-exited on
    // error (clearing running_) while the thread object still needs joining.
    running_.store(false, std::memory_order_release);

    if (timer_)    timer_->requestStop();
    if (dc_timer_) dc_timer_->requestStop();

    if (thread_)    { thread_->requestStop(); thread_->join();    thread_.reset(); }
    if (dc_thread_) { dc_thread_->requestStop(); dc_thread_->join(); dc_thread_.reset(); }

    timer_.reset();
    dc_timer_.reset();
}

// ============================================================================
// Cyclic thread
// ============================================================================

void CyclicExecutive::cyclicMain() {
    // Deferred logging: log calls on this thread only enqueue bounded
    // events — no heap, no mutex, no printf inside the cycle.
    Tether::Platform::Logger::setThreadDeferred(true);
    if (config_.cpu_affinity >= 0) {
        // Move into the exclusive cpuset first (create_cpuset mode):
        // membership is what actually grants the claimed CPU's
        // exclusivity to this thread.  Then re-apply the pin — the
        // spawner's pthread_setaffinity_np raced this and may have
        // failed.
        Tether::Platform::CpuIsolation::instance().adoptCurrentThread(
            config_.cpu_affinity);
        if (!Tether::Platform::setCurrentThreadAffinity(config_.cpu_affinity)) {
            TETHER_LOGW(TAG, "cyclic thread: CPU {} pin failed — unpinned",
                        config_.cpu_affinity);
        }
    }
    if (config_.low_timer_slack) {
        // ~50 µs default timer slack applies to every sleep/ppoll on
        // non-RT threads; under SCHED_FIFO the kernel skips it anyway.
        Tether::Platform::setCurrentThreadTimerSlack(1);
    }
    if (config_.stack_prefault_bytes > 0) {
        Tether::Platform::prefaultCurrentStack(
            std::min<uint32_t>(config_.stack_prefault_bytes,
                               static_cast<uint32_t>(config_.stack_size)
                                   - 16 * 1024));
    }

    const uint64_t period_ns_cfg =
        static_cast<uint64_t>(config_.cycle_period_us) * 1000ULL;

    // Auto: FIFO only on a hard-isolated CPU — anywhere else the global
    // RT-throttle budget (sched_rt_runtime_us, ~950 ms/1 s shared by ALL
    // RT tasks on the CPU) can dequeue the cyclic thread for ~50 ms.
    // Resolved on the *configured* CPU: the pin is applied by the spawner
    // just after thread start, so measured affinity is not yet reliable.
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

    // Root opt-in: kill the global RT-group throttle.  Only sane on an
    // isolated CPU — on a shared one a runaway RT task gets the whole
    // core with no recovery window.
    if (config_.disable_rt_throttle) {
        if (Tether::Platform::disableRtThrottle()) {
            TETHER_LOGI(TAG, "sched_rt_runtime_us=-1 — global RT throttle "
                             "disabled");
            if (!isolated)
                TETHER_LOGW(TAG, "RT throttle disabled on a NON-isolated "
                                 "cyclic CPU — a runaway RT task can now "
                                 "starve the core");
        } else {
            TETHER_LOGW(TAG, "disable_rt_throttle: write to "
                             "sched_rt_runtime_us failed (needs root)");
        }
    }

    // FIFO on a shared CPU + armed RT throttle = the ~50 ms dequeue
    // signature.  Deadline tasks are exempt (per-task CBS).
    if (cls == SchedClass::Fifo && !isolated &&
        Tether::Platform::rtThrottleRuntimeUs() >= 0) {
        TETHER_LOGW(TAG, "SCHED_FIFO on a non-isolated CPU with the kernel "
                         "RT throttle armed (sched_rt_runtime_us ≥ 0): any "
                         "RT-group overrun on this CPU dequeues the cyclic "
                         "thread for the rest of the ~1 s window.  Prefer "
                         "sched_class=Deadline/Auto, or isolate the CPU.");
    }

    // SCHED_DEADLINE charges every busy-wait nanosecond against the
    // task's own CBS runtime: a sleep-mode spin tail is folded into the
    // auto-measured budget below, but in-cycle spins (rx_spin_ns,
    // slot_spin_ns, lock waits) inflate the measured work itself.
    const uint64_t spin_tail_ns =
        config_.sleep_mode == SleepMode::HybridSpin
            ? static_cast<uint64_t>(config_.spin_window_us) * 1000ULL : 0;
    if (cls == SchedClass::Deadline &&
        (spin_tail_ns > 0 || config_.in_cycle_spin_ns > 0)) {
        TETHER_LOGW(TAG, "SCHED_DEADLINE with busy-waiting: {} ns spin "
                         "tail + {} ns in-cycle spin per cycle is charged "
                         "against the CBS runtime — prefer Nanosleep and "
                         "rx_spin_ns=slot_spin_ns=0 on shared CPUs",
                    spin_tail_ns, config_.in_cycle_spin_ns);
    }

    // Q10: SCHED_DEADLINE auto-measure — with no explicit dl_runtime_ns,
    // the thread first runs on SCHED_FIFO for dl_measure_cycles while the
    // loop records per-cycle work, then switches to CBS with
    // runtime = P99 × (100 + dl_margin_pct)/100 + spin tail.
    const bool dl_auto = cls == SchedClass::Deadline &&
                         config_.dl_runtime_ns == 0 &&
                         config_.dl_auto_measure;

    bool rt = false;
    if (cls == SchedClass::Deadline && !dl_auto) {
        // An explicit dl_runtime_ns is taken verbatim (the warning above
        // tells the user to include their spins); the computed default
        // gets the spin tail folded in like the auto-measured budget.
        const uint64_t runtime = config_.dl_runtime_ns
            ? config_.dl_runtime_ns
            : period_ns_cfg / 2 + spin_tail_ns;
        const uint64_t deadline = config_.dl_deadline_ns
            ? config_.dl_deadline_ns : period_ns_cfg;
        dl_runtime_ns_.store(runtime, std::memory_order_relaxed);
        rt = Tether::Platform::setCurrentThreadDeadline(runtime, deadline,
                                                        period_ns_cfg);
        if (rt) {
            deadline_engaged_.store(true, std::memory_order_relaxed);
        } else {
            TETHER_LOGW(TAG, "SCHED_DEADLINE unavailable — falling back "
                             "to SCHED_FIFO");
        }
    }
    if (!rt) {
        rt = Tether::Platform::setCurrentThreadRealtime(config_.priority);
    }
    if (!rt) {
        TETHER_LOGW(TAG, "Cyclic thread could not acquire realtime "
                         "scheduling; running with normal scheduling");
    }

    // Measurement reservoir — allocated once at thread start, never in
    // the steady-state hot path.
    std::vector<uint32_t> dl_samples;
    if (dl_auto) {
        dl_samples.reserve(std::max<uint32_t>(config_.dl_measure_cycles, 64));
        TETHER_LOGI(TAG, "SCHED_DEADLINE auto-measure: sampling {} cycles "
                         "on SCHED_FIFO", config_.dl_measure_cycles);
    }
    bool dl_measuring = dl_auto;

    if (!timer_->start(period_ns_cfg)) {
        TETHER_LOGE(TAG, "Deadline timer start failed");
        running_.store(false, std::memory_order_release);
        return;
    }

    uint64_t cycle = 0;
    Platform::IDeadlineTimer::Tick tick{};

    while (running_.load(std::memory_order_acquire)) {
        if (!timer_->waitNext(tick)) break;
        ++cycle;

        if (tick.missed) {
            missed_deadlines_.fetch_add(tick.missed, std::memory_order_relaxed);
        }
        // Wake overshoot: how late this wake landed vs its deadline.
        // Separates "the sleep path is jittery" (Nanosleep on a shared
        // CPU) from "the work overran" — the two need different fixes.
        if (tick.woke_ns > tick.deadline_ns && tick.deadline_ns != 0) {
            const uint64_t over_ns = tick.woke_ns - tick.deadline_ns;
            const uint32_t over =
                static_cast<uint32_t>(over_ns > UINT32_MAX
                                          ? UINT32_MAX : over_ns);
            const uint32_t cur =
                max_wake_overshoot_ns_.load(std::memory_order_relaxed);
            if (over > cur) {
                max_wake_overshoot_ns_.store(over,
                                             std::memory_order_relaxed);
            }
            avg_wake_overshoot_ns_.store(
                (avg_wake_overshoot_ns_.load(std::memory_order_relaxed) * 7 +
                 over) / 8,
                std::memory_order_relaxed);
        }
        jitter_monitor_->recordCycle(tick.woke_ns);

        // ── Inline DC sync: first action on decimated cycles so the SYNC
        //    frame leaves as early as possible in the cycle window. ──
        if (config_.dc_placement == DCPlacement::Inline && dc_sync_ &&
            cycle % config_.dc_interval_cycles == 0) {
            if (dc_sync_()) {
                dc_sync_count_.fetch_add(1, std::memory_order_relaxed);
            } else {
                dc_sync_errors_.fetch_add(1, std::memory_order_relaxed);
            }
        }

        if (!runPhase(kIdxPre)) break;

        if (exchange_enabled_.load(std::memory_order_acquire) && exchange_) {
            if (!exchange_()) {
                exchange_errors_.fetch_add(1, std::memory_order_relaxed);
                if (!config_.continue_on_error) break;
            }
        }

        if (!runPhase(kIdxPost))   break;
        if (!runPhase(kIdxMotion)) break;
        if (!runPhase(kIdxDiag))   break;

        // In-cycle work burst duration (wake -> end of work), not the period.
        // platformNowNs() is used deliberately: it shares the deadline timer's
        // clock domain, whereas time_source_ may be an embedder-overridden
        // clock (e.g. PTP-disciplined DC time) with a different epoch.
        const uint64_t work_us = (platformNowNs() - tick.woke_ns) / 1000ULL;
        const uint32_t cur = max_cycle_work_us_.load(std::memory_order_relaxed);
        if (work_us > cur) {
            max_cycle_work_us_.store(static_cast<uint32_t>(work_us),
                                     std::memory_order_relaxed);
        }

        // Q10: collect work samples, then switch to CBS once measured.
        if (dl_measuring) {
            dl_samples.push_back(static_cast<uint32_t>(work_us));
            if (dl_samples.size() >= config_.dl_measure_cycles) {
                dl_measuring = false;
                std::ranges::sort(dl_samples);
                const size_t p99_idx =
                    std::min(dl_samples.size() - 1,
                             dl_samples.size() * 99 / 100);
                const uint64_t p99_us = dl_samples[p99_idx];
                // The spin tail sits inside waitNext() — outside the
                // measured woke→work-end bracket — but CBS still charges
                // it, so it must be budgeted explicitly.
                uint64_t runtime = p99_us * 1000ULL *
                                   (100ULL + config_.dl_margin_pct) / 100ULL
                                   + spin_tail_ns;
                const uint64_t deadline = config_.dl_deadline_ns
                    ? config_.dl_deadline_ns : period_ns_cfg;
                runtime = std::clamp<uint64_t>(runtime, 1000ULL,
                                               deadline * 95 / 100);
                dl_runtime_ns_.store(runtime, std::memory_order_relaxed);
                if (Tether::Platform::setCurrentThreadDeadline(
                        runtime, deadline, period_ns_cfg)) {
                    deadline_engaged_.store(true, std::memory_order_relaxed);
                    TETHER_LOGI(TAG, "SCHED_DEADLINE engaged: runtime={} ns "
                                     "(P99 work {} µs × (100+{})/100)",
                                runtime, p99_us, config_.dl_margin_pct);
                } else {
                    TETHER_LOGW(TAG, "SCHED_DEADLINE auto-apply failed — "
                                     "staying on SCHED_FIFO");
                }
            }
        }

        cycle_count_.store(cycle, std::memory_order_relaxed);
    }

    running_.store(false, std::memory_order_release);
}

bool CyclicExecutive::runPhase(size_t phase_idx) {
    for (const auto& fn : phase_tasks_[phase_idx]) {
        if (fn && !fn()) {
            task_errors_.fetch_add(1, std::memory_order_relaxed);
            if (!config_.continue_on_error) return false;
        }
    }
    return true;
}

// ============================================================================
// Dedicated DC thread — fault-isolated from the cyclic path
// ============================================================================

void CyclicExecutive::dcMain() {
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
    if (config_.low_timer_slack) {
        Tether::Platform::setCurrentThreadTimerSlack(1);
    }
    if (!Tether::Platform::setCurrentThreadRealtime(config_.dc_priority)) {
        TETHER_LOGW(TAG, "DC sync thread could not acquire SCHED_FIFO; "
                         "running with normal scheduling");
    }

    const uint64_t period_ns =
        static_cast<uint64_t>(config_.cycle_period_us) *
        config_.dc_interval_cycles * 1000ULL;

    if (!dc_timer_->start(period_ns)) {
        TETHER_LOGE(TAG, "DC deadline timer start failed");
        return;
    }

    Platform::IDeadlineTimer::Tick tick{};
    while (running_.load(std::memory_order_acquire)) {
        if (!dc_timer_->waitNext(tick)) break;

        dc_jitter_monitor_->recordCycle(tick.woke_ns);

        if (dc_sync_ && dc_sync_()) {
            dc_sync_count_.fetch_add(1, std::memory_order_relaxed);
        } else {
            dc_sync_errors_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

// ============================================================================
// Stats
// ============================================================================

CyclicExecutive::Stats CyclicExecutive::getStats() const {
    Stats s;
    s.cycle_count       = cycle_count_.load(std::memory_order_relaxed);
    s.exchange_errors   = exchange_errors_.load(std::memory_order_relaxed);
    s.task_errors       = task_errors_.load(std::memory_order_relaxed);
    s.missed_deadlines  = missed_deadlines_.load(std::memory_order_relaxed);
    s.max_cycle_work_us = max_cycle_work_us_.load(std::memory_order_relaxed);
    s.max_wake_overshoot_ns =
        max_wake_overshoot_ns_.load(std::memory_order_relaxed);
    s.avg_wake_overshoot_ns =
        avg_wake_overshoot_ns_.load(std::memory_order_relaxed);
    s.dc_sync_count     = dc_sync_count_.load(std::memory_order_relaxed);
    s.dc_sync_errors    = dc_sync_errors_.load(std::memory_order_relaxed);
    s.dl_runtime_ns     = dl_runtime_ns_.load(std::memory_order_relaxed);
    s.deadline_active   = deadline_engaged_.load(std::memory_order_relaxed);
    if (jitter_monitor_)    s.jitter    = jitter_monitor_->getStats();
    if (dc_jitter_monitor_) s.dc_jitter = dc_jitter_monitor_->getStats();
    return s;
}

} // namespace EtherCAT
