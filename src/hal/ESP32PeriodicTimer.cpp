#include "hal/ESP32PeriodicTimer.hpp"
#include "esp_log.h"

static const char* TAG = "ESP32Timer";

namespace HAL {

ESP32PeriodicTimer::ESP32PeriodicTimer() {}

ESP32PeriodicTimer::~ESP32PeriodicTimer() {
    stop();
    if (timer_handle_) {
        esp_timer_delete(timer_handle_);
    }
}

bool ESP32PeriodicTimer::init(uint32_t frequencyHz) {
    if (frequencyHz == 0) return false;
    
    // Calculate period in microseconds
    period_us_ = 1000000 / frequencyHz;

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &ESP32PeriodicTimer::timerCallback;
    timer_args.arg = this;
    timer_args.name = "cyclic_timer";
    // Using default dispatch method (ESP_TIMER_TASK).
    // This runs the callback in the high-priority esp_timer task.

    esp_err_t err = esp_timer_create(&timer_args, &timer_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG, "Timer initialized: %lu Hz (%lu us period)", (unsigned long)frequencyHz, (unsigned long)period_us_);
    return true;
}

void ESP32PeriodicTimer::start() {
    if (!timer_handle_) return;
    if (running_) return;
    
    esp_timer_start_periodic(timer_handle_, period_us_);
    running_ = true;
    ESP_LOGI(TAG, "Timer started");
}

void ESP32PeriodicTimer::stop() {
    if (running_ && timer_handle_) {
        esp_timer_stop(timer_handle_);
        running_ = false;
        ESP_LOGI(TAG, "Timer stopped");
    }
}

void ESP32PeriodicTimer::waitForCycle() {
    // Capture task handle of the caller on first call if not set
    if (task_handle_ == nullptr) {
        task_handle_ = xTaskGetCurrentTaskHandle();
        ESP_LOGI(TAG, "Task registered for notifications: %s", pcTaskGetName(task_handle_));
    }
    
    // Wait for notification. 
    // This will clear the notification value (pdTRUE) and wait indefinitely (portMAX_DELAY).
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void ESP32PeriodicTimer::timerCallback(void* arg) {
    auto* self = static_cast<ESP32PeriodicTimer*>(arg);
    if (self->task_handle_) {
        // Send notification to the registered task
        xTaskNotifyGive(self->task_handle_);
    }
}

}
