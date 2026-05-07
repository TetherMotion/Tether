/**
 * @file ESP32Clock.cpp
 * @brief ESP32 clock and timer HAL implementation
 */

#if defined(ESP_PLATFORM) || defined(ESP32)

#include "hal/IClock.hpp"
#include "hal/IPeriodicTimer.hpp"
#include "hal/HALTypes.hpp"

#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#include <atomic>
#include <ctime>

static const char* TAG = "ESP32Clock";

namespace EtherCAT {
namespace HAL {

// ============================================================================
// ESP32 Clock Implementation
// ============================================================================

class ESP32Clock : public IClock {
public:
    Timestamp nowMicros() override {
        return esp_timer_get_time();
    }

    Timestamp nowNanos() override {
        // ESP32 doesn't have nanosecond precision, estimate from microseconds
        return esp_timer_get_time() * 1000;
    }

    Timestamp systemTimeMillis() override {
        time_t now;
        time(&now);
        return static_cast<Timestamp>(now) * 1000;
    }

    Nanoseconds resolution() override {
        // ESP32 timer resolution is 1 microsecond
        return 1000;
    }

    void delayMicros(Microseconds us) override {
        if (us <= 0) return;
        
        if (us < 10) {
            // Very short delay - use ROM busy-wait
            esp_rom_delay_us(static_cast<uint32_t>(us));
        } else {
            // Longer delay - use esp_timer for accuracy
            Timestamp start = esp_timer_get_time();
            while ((esp_timer_get_time() - start) < static_cast<Timestamp>(us)) {
                // Busy wait
            }
        }
    }

    void delayMillis(Milliseconds ms) override {
        if (ms <= 0) return;
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
};

// ============================================================================
// ESP32 Periodic Timer Implementation
// ============================================================================

class ESP32PeriodicTimerNew : public IPeriodicTimer {
public:
    ESP32PeriodicTimerNew() = default;

    ~ESP32PeriodicTimerNew() override {
        stop();
        if (m_timerHandle) {
            esp_timer_delete(m_timerHandle);
        }
    }

    bool init(uint32_t frequencyHz) override {
        if (frequencyHz == 0) return false;

        m_periodUs = 1000000 / frequencyHz;

        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &ESP32PeriodicTimerNew::timerCallback;
        timer_args.arg = this;
        timer_args.name = "hal_periodic";
        timer_args.dispatch_method = ESP_TIMER_TASK;

        esp_err_t err = esp_timer_create(&timer_args, &m_timerHandle);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
            return false;
        }

        TETHER_LOGI(TAG, "Timer initialized: %lu Hz (%lu us period)", 
                 (unsigned long)frequencyHz, (unsigned long)m_periodUs);
        return true;
    }

    void start() override {
        if (!m_timerHandle || m_running) return;

        esp_err_t err = esp_timer_start_periodic(m_timerHandle, m_periodUs);
        if (err == ESP_OK) {
            m_running = true;
            m_stats.tickCount = 0;
            TETHER_LOGI(TAG, "Timer started");
        }
    }

    void stop() override {
        if (!m_running || !m_timerHandle) return;

        esp_timer_stop(m_timerHandle);
        m_running = false;
        TETHER_LOGI(TAG, "Timer stopped");
    }

    bool isRunning() const override {
        return m_running;
    }

    void waitForCycle() override {
        if (!m_running) return;

        // Capture task handle on first call
        if (m_taskHandle == nullptr) {
            m_taskHandle = xTaskGetCurrentTaskHandle();
            TETHER_LOGI(TAG, "Task registered: %s", pcTaskGetName(m_taskHandle));
        }

        // Wait for notification from timer callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Update statistics
        Timestamp now = esp_timer_get_time();
        if (m_lastTick != 0) {
            Microseconds jitter = std::abs(static_cast<int64_t>(now - m_lastTick) - m_periodUs);
            if (jitter > m_stats.maxJitter) {
                m_stats.maxJitter = jitter;
            }
            m_stats.avgJitter = (m_stats.avgJitter * m_stats.tickCount + jitter) / 
                                (m_stats.tickCount + 1);
        }
        m_lastTick = now;
        m_stats.tickCount++;
    }

    void setCallback(TimerCallback callback) override {
        m_callback = callback;
    }

    Microseconds getPeriodMicros() const override {
        return m_periodUs;
    }

    Stats getStats() const override {
        return m_stats;
    }

    void resetStats() override {
        m_stats = {};
        m_lastTick = 0;
    }

private:
    esp_timer_handle_t m_timerHandle = nullptr;
    TaskHandle_t m_taskHandle = nullptr;
    Microseconds m_periodUs = 1000;
    std::atomic<bool> m_running{false};
    TimerCallback m_callback;
    Stats m_stats;
    Timestamp m_lastTick = 0;

    static void timerCallback(void* arg) {
        auto* self = static_cast<ESP32PeriodicTimerNew*>(arg);
        
        // Notify waiting task
        if (self->m_taskHandle) {
            xTaskNotifyGive(self->m_taskHandle);
        }
        
        // Call user callback
        if (self->m_callback) {
            self->m_callback();
        }
    }
};

// ============================================================================
// ESP32 One-Shot Timer Implementation
// ============================================================================

class ESP32OneShotTimer : public IOneShotTimer {
public:
    ESP32OneShotTimer() {
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &ESP32OneShotTimer::timerCallback;
        timer_args.arg = this;
        timer_args.name = "hal_oneshot";
        timer_args.dispatch_method = ESP_TIMER_TASK;

        esp_timer_create(&timer_args, &m_timerHandle);
    }

    ~ESP32OneShotTimer() override {
        cancel();
        if (m_timerHandle) {
            esp_timer_delete(m_timerHandle);
        }
    }

    bool start(Microseconds delayUs, TimerCallback callback) override {
        if (!m_timerHandle || delayUs <= 0) return false;

        m_callback = callback;
        m_expiryTime = esp_timer_get_time() + delayUs;

        esp_err_t err = esp_timer_start_once(m_timerHandle, delayUs);
        if (err != ESP_OK) {
            return false;
        }

        m_pending = true;
        return true;
    }

    bool cancel() override {
        if (!m_pending) return false;

        esp_timer_stop(m_timerHandle);
        m_pending = false;
        return true;
    }

    bool isPending() const override {
        return m_pending;
    }

    Microseconds remaining() const override {
        if (!m_pending) return 0;

        Timestamp now = esp_timer_get_time();
        if (now >= m_expiryTime) return 0;
        return m_expiryTime - now;
    }

private:
    esp_timer_handle_t m_timerHandle = nullptr;
    TimerCallback m_callback;
    Timestamp m_expiryTime = 0;
    std::atomic<bool> m_pending{false};

    static void timerCallback(void* arg) {
        auto* self = static_cast<ESP32OneShotTimer*>(arg);
        self->m_pending = false;
        if (self->m_callback) {
            self->m_callback();
        }
    }
};

// ============================================================================
// ESP32 Clock Factory
// ============================================================================

class ESP32ClockFactory : public IClockFactory {
public:
    IClock& getSystemClock() override {
        static ESP32Clock clock;
        return clock;
    }

    std::unique_ptr<IPeriodicTimer> createPeriodicTimer() override {
        return std::make_unique<ESP32PeriodicTimerNew>();
    }

    std::unique_ptr<IOneShotTimer> createOneShotTimer() override {
        return std::make_unique<ESP32OneShotTimer>();
    }
};

// Clock factory singleton with explicit lifecycle.
// Use resetClockFactory() in tests to ensure clean state.
static std::unique_ptr<IClockFactory> g_clockFactory;

IClockFactory& getClockFactory() {
    if (!g_clockFactory) {
        g_clockFactory = std::make_unique<ESP32ClockFactory>();
    }
    return *g_clockFactory;
}

void setClockFactory(std::unique_ptr<IClockFactory> factory) {
    g_clockFactory = std::move(factory);
}

void resetClockFactory() {
    g_clockFactory.reset();
}

} // namespace HAL
} // namespace EtherCAT

#endif // ESP_PLATFORM || ESP32
