/**
 * @file RealtimeLoop.cpp
 * @brief Two-thread realtime loop: independent PDO exchange and DC sync threads
 *
 * Each thread has its own timer, event, HAL thread, and jitter monitor.
 * The PDO thread runs at `cycle_period_us`, the DC thread runs at
 * `cycle_period_us * sync_interval_cycles`.
 */

#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/platform/Platform.hpp"

#include <algorithm>

namespace EtherCAT {

static const char* TAG = "ethercat_rt_loop";

// ============================================================================
// Constructor / Destructor
// ============================================================================

RealtimeLoop::RealtimeLoop(ExchangeFunc pdo_exchange,
                                            SyncFunc     dc_sync,
                                            TimeFunc     time_source,
                                            const Config& config)
    : pdo_exchange_(std::move(pdo_exchange))
    , dc_sync_(std::move(dc_sync))
    , time_source_(std::move(time_source))
    , config_(config)
{
}

RealtimeLoop::~RealtimeLoop() {
    stop();
}

// ============================================================================
// Public API
// ============================================================================

bool RealtimeLoop::start() {
    if (running_.load(std::memory_order_acquire)) {
        TETHER_LOGW(TAG, "Realtime loop already running");
        return false;
    }

    // Set running BEFORE creating resources to prevent a race where a thread
    // starts, sees running_==false, and exits immediately.
    running_.store(true, std::memory_order_release);

    // Reset counters (relaxed — only the RT threads write, and start() runs
    // before they do).
    pdo_cycle_count_.store(0, std::memory_order_relaxed);
    pdo_error_count_.store(0, std::memory_order_relaxed);
    dc_sync_count_.store(0, std::memory_order_relaxed);

    // Start both threads.  If one fails, tear down the other.
    if (!startPDOThread()) {
        running_.store(false, std::memory_order_release);
        return false;
    }

    if (!startDCThread()) {
        running_.store(false, std::memory_order_release);
        stopPDOThread();
        return false;
    }

    const uint32_t dc_period_us = config_.cycle_period_us * config_.sync_interval_cycles;
    TETHER_LOGI(TAG, "Realtime loop started: PDO thread @ %u us, DC thread @ %u us",
                config_.cycle_period_us, dc_period_us);
    return true;
}

void RealtimeLoop::stop() {
    running_.store(false, std::memory_order_release);

    stopPDOThread();
    stopDCThread();

    TETHER_LOGI(TAG, "Realtime loop stopped");
}

RealtimeLoop::Stats RealtimeLoop::getStats() const {
    Stats s{};

    // Atomic snapshots (relaxed — independent monotonic counters).
    s.cycle_count     = pdo_cycle_count_.load(std::memory_order_relaxed);
    s.pdo_error_count = pdo_error_count_.load(std::memory_order_relaxed);
    s.sync_count      = dc_sync_count_.load(std::memory_order_relaxed);

    // Jitter (from PDO monitor — primary realtime thread)
    if (pdo_jitter_monitor_) {
        auto js = pdo_jitter_monitor_->getStats();
        s.max_jitter_us = js.max_jitter_us;
        s.avg_jitter_us = js.avg_jitter_us;
    }

    return s;
}

RealtimeLoop::ThreadDiagnostics RealtimeLoop::getDiagnostics() const {
    ThreadDiagnostics d{};
    if (pdo_jitter_monitor_) d.pdo_jitter = pdo_jitter_monitor_->getStats();
    if (dc_jitter_monitor_)  d.dc_jitter  = dc_jitter_monitor_->getStats();
    return d;
}

// ============================================================================
// PDO thread
// ============================================================================

bool RealtimeLoop::startPDOThread() {
    // Create jitter monitor
    pdo_jitter_monitor_ = std::make_unique<RealtimeJitterMonitor>(config_.pdo_jitter, "pdo");

    // Create event for timer→thread signaling
    pdo_event_ = HAL::getThreadingFactory().createEvent(false, false);
    if (!pdo_event_) {
        TETHER_LOGE(TAG, "Failed to create PDO thread event");
        return false;
    }

    // Create realtime thread
    HAL::ThreadConfig cfg;
    cfg.name = "rt_pdo";
    cfg.stackSize = 4096;
    cfg.priority = HAL::ThreadPriority::Realtime;
    cfg.useRealtimeScheduling = true;

    pdo_thread_ = HAL::getThreadingFactory().createThread(cfg);
    if (!pdo_thread_) {
        TETHER_LOGE(TAG, "Failed to create PDO realtime thread");
        pdo_event_.reset();
        return false;
    }

    auto start_err = pdo_thread_->start([this]() { pdoTaskEntry(this); });
    if (start_err != HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to start PDO realtime thread");
        pdo_thread_.reset();
        pdo_event_.reset();
        return false;
    }

    // Create and configure platform timer
    pdo_timer_ = Platform::createPlatformTimer();
    if (!pdo_timer_) {
        TETHER_LOGE(TAG, "Failed to create PDO platform timer");
        running_.store(false, std::memory_order_release);
        if (pdo_event_) pdo_event_->signal();
        pdo_thread_->requestStop();
        pdo_thread_->join();
        pdo_thread_.reset();
        pdo_event_.reset();
        return false;
    }

    Platform::TimerConfig timer_cfg;
    timer_cfg.period_us   = config_.cycle_period_us;
    timer_cfg.callback    = pdoTimerCallback;
    timer_cfg.user_data   = this;
    timer_cfg.auto_reload = true;
    timer_cfg.priority    = 0;

    if (!pdo_timer_->configure(timer_cfg) || !pdo_timer_->start()) {
        TETHER_LOGE(TAG, "Failed to configure/start PDO timer");
        running_.store(false, std::memory_order_release);
        pdo_timer_.reset();
        if (pdo_event_) pdo_event_->signal();
        pdo_thread_->requestStop();
        pdo_thread_->join();
        pdo_thread_.reset();
        pdo_event_.reset();
        return false;
    }

    return true;
}

void RealtimeLoop::stopPDOThread() {
    if (pdo_timer_) {
        pdo_timer_->stop();
        pdo_timer_.reset();
    }

    if (pdo_thread_) {
        if (pdo_event_) pdo_event_->signal();
        Tether::Platform::Clock::instance().delayMilliseconds(50);
        pdo_thread_->requestStop();
        pdo_thread_->join();
        pdo_thread_.reset();
    }

    pdo_event_.reset();
}

bool RealtimeLoop::pdoTimerCallback(void* user_data) {
    auto* loop = static_cast<RealtimeLoop*>(user_data);
    if (loop && loop->pdo_event_) {
        loop->pdo_event_->signal();
    }
    return false;
}

void RealtimeLoop::pdoTaskEntry(void* param) {
    auto* loop = static_cast<RealtimeLoop*>(param);
    if (!loop) return;

    TETHER_LOGI(TAG, "PDO realtime task started");

    if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
        TETHER_LOGW(TAG, "PDO thread could not acquire SCHED_FIFO priority; running with normal scheduling");
    }

    while (loop->running_.load(std::memory_order_acquire)) {
        // Wait for timer event
        if (loop->pdo_event_) {
            if (loop->pdo_event_->wait() != HAL::Error::OK) {
                continue;
            }
        }

        if (!loop->running_.load(std::memory_order_acquire)) break;

        // Jitter self-diagnosis
        const uint64_t now = loop->time_source_ ? loop->time_source_() : 0;
        if (loop->pdo_jitter_monitor_) {
            loop->pdo_jitter_monitor_->recordCycle(now);
        }

        // ── PDO exchange (every cycle if enabled) ──
        if (loop->pdo_enabled_.load(std::memory_order_acquire) && loop->pdo_exchange_) {
            if (!loop->pdo_exchange_()) {
                loop->pdo_error_count_.fetch_add(1, std::memory_order_relaxed);
            }
        }

        // ── Update cycle count ──
        loop->pdo_cycle_count_.fetch_add(1, std::memory_order_relaxed);
    }

    TETHER_LOGI(TAG, "PDO realtime task exiting");
}

// ============================================================================
// DC thread
// ============================================================================

bool RealtimeLoop::startDCThread() {
    const uint32_t dc_period_us = config_.cycle_period_us * config_.sync_interval_cycles;

    // Create jitter monitor
    dc_jitter_monitor_ = std::make_unique<RealtimeJitterMonitor>(config_.dc_jitter, "dc");

    // Create event for timer→thread signaling
    dc_event_ = HAL::getThreadingFactory().createEvent(false, false);
    if (!dc_event_) {
        TETHER_LOGE(TAG, "Failed to create DC thread event");
        return false;
    }

    // Create realtime thread
    HAL::ThreadConfig cfg;
    cfg.name = "rt_dc";
    cfg.stackSize = 4096;
    cfg.priority = HAL::ThreadPriority::Realtime;
    cfg.useRealtimeScheduling = true;

    dc_thread_ = HAL::getThreadingFactory().createThread(cfg);
    if (!dc_thread_) {
        TETHER_LOGE(TAG, "Failed to create DC realtime thread");
        dc_event_.reset();
        return false;
    }

    auto start_err = dc_thread_->start([this]() { dcTaskEntry(this); });
    if (start_err != HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to start DC realtime thread");
        dc_thread_.reset();
        dc_event_.reset();
        return false;
    }

    // Create and configure platform timer
    dc_timer_ = Platform::createPlatformTimer();
    if (!dc_timer_) {
        TETHER_LOGE(TAG, "Failed to create DC platform timer");
        running_.store(false, std::memory_order_release);
        if (dc_event_) dc_event_->signal();
        dc_thread_->requestStop();
        dc_thread_->join();
        dc_thread_.reset();
        dc_event_.reset();
        return false;
    }

    Platform::TimerConfig timer_cfg;
    timer_cfg.period_us   = dc_period_us;
    timer_cfg.callback    = dcTimerCallback;
    timer_cfg.user_data   = this;
    timer_cfg.auto_reload = true;
    timer_cfg.priority    = 0;

    if (!dc_timer_->configure(timer_cfg) || !dc_timer_->start()) {
        TETHER_LOGE(TAG, "Failed to configure/start DC timer");
        running_.store(false, std::memory_order_release);
        dc_timer_.reset();
        if (dc_event_) dc_event_->signal();
        dc_thread_->requestStop();
        dc_thread_->join();
        dc_thread_.reset();
        dc_event_.reset();
        return false;
    }

    return true;
}

void RealtimeLoop::stopDCThread() {
    if (dc_timer_) {
        dc_timer_->stop();
        dc_timer_.reset();
    }

    if (dc_thread_) {
        if (dc_event_) dc_event_->signal();
        Tether::Platform::Clock::instance().delayMilliseconds(50);
        dc_thread_->requestStop();
        dc_thread_->join();
        dc_thread_.reset();
    }

    dc_event_.reset();
}

bool RealtimeLoop::dcTimerCallback(void* user_data) {
    auto* loop = static_cast<RealtimeLoop*>(user_data);
    if (loop && loop->dc_event_) {
        loop->dc_event_->signal();
    }
    return false;
}

void RealtimeLoop::dcTaskEntry(void* param) {
    auto* loop = static_cast<RealtimeLoop*>(param);
    if (!loop) return;

    TETHER_LOGI(TAG, "DC realtime task started");

    if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
        TETHER_LOGW(TAG, "DC thread could not acquire SCHED_FIFO priority; running with normal scheduling");
    }

    while (loop->running_.load(std::memory_order_acquire)) {
        // Wait for timer event
        if (loop->dc_event_) {
            if (loop->dc_event_->wait() != HAL::Error::OK) {
                continue;
            }
        }

        if (!loop->running_.load(std::memory_order_acquire)) break;

        // Jitter self-diagnosis
        const uint64_t now = loop->time_source_ ? loop->time_source_() : 0;
        if (loop->dc_jitter_monitor_) {
            loop->dc_jitter_monitor_->recordCycle(now);
        }

        // ── DC sync ──
        if (loop->dc_sync_ && loop->dc_sync_()) {
            loop->dc_sync_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    TETHER_LOGI(TAG, "DC realtime task exiting");
}

} // namespace EtherCAT
