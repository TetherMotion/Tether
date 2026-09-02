/**
 * @file ESP32Timer.cpp
 * @brief ESP32 gptimer-based implementation of IPlatformTimer
 */

#include "tether/platform/IPlatformTimer.hpp"
#include "tether/platform/EspCompat.hpp"

#include "driver/gptimer.h"

namespace EtherCAT {
namespace Platform {

static const char* TAG = "esp32_timer";

/**
 * @brief ESP32 timer implementation using gptimer driver
 */
class ESP32Timer : public IPlatformTimer {
public:
    ESP32Timer() = default;
    
    ~ESP32Timer() override {
        stop();
        if (timer_) {
            gptimer_del_timer(timer_);
            timer_ = nullptr;
        }
    }
    
    bool configure(const TimerConfig& config) override {
        if (isRunning()) {
            TETHER_LOGE(TAG, "Cannot configure while running");
            return false;
        }
        
        // Clean up existing timer if reconfiguring
        if (timer_) {
            gptimer_del_timer(timer_);
            timer_ = nullptr;
        }
        
        config_ = config;
        
        // Create timer with 1MHz resolution (1μs ticks)
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,  // 1MHz = 1μs resolution
            .intr_priority = config.priority,
            .flags = {
                .intr_shared = false,
                .allow_pd = false,
                .backup_before_sleep = false,
            }
        };
        
        esp_err_t err = gptimer_new_timer(&timer_config, &timer_);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "gptimer_new_timer failed: {}", esp_err_to_name(err));
            return false;
        }
        
        // Configure alarm for periodic trigger
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = config.period_us,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = config.auto_reload,
            }
        };
        
        err = gptimer_set_alarm_action(timer_, &alarm_config);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "gptimer_set_alarm_action failed: {}", esp_err_to_name(err));
            gptimer_del_timer(timer_);
            timer_ = nullptr;
            return false;
        }
        
        // Register ISR callback
        gptimer_event_callbacks_t cbs = {
            .on_alarm = timerISRCallback,
        };
        
        err = gptimer_register_event_callbacks(timer_, &cbs, this);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "gptimer_register_event_callbacks failed: {}", esp_err_to_name(err));
            gptimer_del_timer(timer_);
            timer_ = nullptr;
            return false;
        }
        
        configured_ = true;
        return true;
    }
    
    bool start() override {
        if (!configured_ || !timer_) {
            TETHER_LOGE(TAG, "Timer not configured");
            return false;
        }
        
        if (running_) {
            return true;  // Already running
        }
        
        esp_err_t err = gptimer_enable(timer_);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "gptimer_enable failed: {}", esp_err_to_name(err));
            return false;
        }
        
        err = gptimer_start(timer_);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "gptimer_start failed: {}", esp_err_to_name(err));
            gptimer_disable(timer_);
            return false;
        }
        
        running_ = true;
        start_time_us_ = esp_timer_get_time();
        cycle_count_ = 0;
        
        TETHER_LOGI(TAG, "Timer started at {} Hz", 1000000UL / config_.period_us);
        return true;
    }
    
    void stop() override {
        if (!running_ || !timer_) {
            return;
        }
        
        gptimer_stop(timer_);
        gptimer_disable(timer_);
        running_ = false;
        
        TETHER_LOGI(TAG, "Timer stopped after {} cycles", (unsigned long long)cycle_count_);
    }
    
    bool isRunning() const override {
        return running_;
    }
    
    uint32_t getActualPeriodUs() const override {
        return config_.period_us;  // gptimer provides exact period
    }
    
    bool getJitterStats(uint32_t& max_jitter_us, uint32_t& avg_jitter_us) const override {
        max_jitter_us = max_jitter_us_;
        avg_jitter_us = avg_jitter_us_;
        return cycle_count_ > 0;
    }
    
private:
    /**
     * @brief Timer ISR callback (runs in interrupt context)
     */
    static bool IRAM_ATTR timerISRCallback(
        gptimer_handle_t timer,
        const gptimer_alarm_event_data_t* edata,
        void* user_ctx)
    {
        (void)timer;
        (void)edata;
        
        ESP32Timer* self = static_cast<ESP32Timer*>(user_ctx);
        if (!self || !self->running_) {
            return false;
        }
        
        // Measure jitter
        const int64_t now_us = esp_timer_get_time();
        const int64_t expected_time = self->start_time_us_ + 
                                     (self->cycle_count_ * self->config_.period_us);
        const int64_t jitter = now_us - expected_time;
        const uint32_t abs_jitter = static_cast<uint32_t>(jitter > 0 ? jitter : -jitter);
        
        if (abs_jitter > self->max_jitter_us_) {
            self->max_jitter_us_ = abs_jitter;
        }
        
        // Simple moving average
        self->avg_jitter_us_ = (self->avg_jitter_us_ * 7 + abs_jitter) / 8;
        self->cycle_count_++;
        
        // Invoke user callback
        bool high_task_woken = false;
        if (self->config_.callback) {
            high_task_woken = self->config_.callback(self->config_.user_data);
        }
        
        return high_task_woken;
    }
    
    gptimer_handle_t timer_ = nullptr;
    TimerConfig config_;
    bool configured_ = false;
    bool running_ = false;
    
    int64_t start_time_us_ = 0;
    uint64_t cycle_count_ = 0;
    uint32_t max_jitter_us_ = 0;
    uint32_t avg_jitter_us_ = 0;
};

std::unique_ptr<IPlatformTimer> createPlatformTimer()
{
    return std::make_unique<ESP32Timer>();
}

} // namespace Platform
} // namespace EtherCAT
