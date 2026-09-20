/**
 * @file CyclicExecutive.cpp
 * @brief Single-thread, deadline-driven cyclic executive.
 *
 * Hot path per cycle: one virtual waitNext() + one clock_nanosleep — no
 * producer timer thread, no event, no mutex, no allocation.
 */

#include "tether/ethercat/CyclicExecutive.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"

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

    bool rt = false;
    if (config_.sched_class == SchedClass::Deadline) {
        const uint64_t period_ns =
            static_cast<uint64_t>(config_.cycle_period_us) * 1000ULL;
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
        TETHER_LOGW(TAG, "Cyclic thread could not acquire realtime "
                         "scheduling; running with normal scheduling");
    }

    const uint64_t period_ns =
        static_cast<uint64_t>(config_.cycle_period_us) * 1000ULL;

    if (!timer_->start(period_ns)) {
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
    s.dc_sync_count     = dc_sync_count_.load(std::memory_order_relaxed);
    s.dc_sync_errors    = dc_sync_errors_.load(std::memory_order_relaxed);
    if (jitter_monitor_)    s.jitter    = jitter_monitor_->getStats();
    if (dc_jitter_monitor_) s.dc_jitter = dc_jitter_monitor_->getStats();
    return s;
}

} // namespace EtherCAT
