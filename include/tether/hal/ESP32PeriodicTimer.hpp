#pragma once
#include "hal/IPeriodicTimer.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace HAL {

class ESP32PeriodicTimer : public IPeriodicTimer {
public:
    ESP32PeriodicTimer();
    ~ESP32PeriodicTimer() override;

    bool init(uint32_t frequencyHz) override;
    void start() override;
    void stop() override;
    void waitForCycle() override;

private:
    static void timerCallback(void* arg);

    esp_timer_handle_t timer_handle_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    uint32_t period_us_ = 1000;
    bool running_ = false;
};

}
