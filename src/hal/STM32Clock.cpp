/**
 * @file STM32Clock.cpp
 * @brief STM32 clock and timer HAL implementation
 *
 * NOTE: This is a best-effort implementation without testing.
 */

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32_HAL)

#include "hal/IClock.hpp"
#include "hal/HALTypes.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <atomic>
#include <cstdint>

// STM32 HAL for hardware timer access
#if defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#endif

namespace EtherCAT {
namespace HAL {

// ============================================================================
// STM32 Clock Implementation
// ============================================================================

class STM32Clock : public IClock {
public:
    STM32Clock() {
        // Record startup time in milliseconds from HAL tick
        m_startTime = HAL_GetTick();
    }

    Nanoseconds now() const override {
        // HAL_GetTick() returns milliseconds since startup
        // For better resolution, we can use DWT cycle counter if available
#if defined(DWT) && defined(CoreDebug)
        static bool dwtInitialized = false;
        if (!dwtInitialized) {
            // Enable DWT cycle counter
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
            dwtInitialized = true;
        }
        
        // Calculate nanoseconds from cycle counter
        // Note: This assumes SystemCoreClock is set correctly
        uint32_t cycles = DWT->CYCCNT;
        uint64_t ns = static_cast<uint64_t>(cycles) * 1000000000ULL / SystemCoreClock;
        return ns;
#else
        // Fallback to millisecond resolution
        uint32_t ms = HAL_GetTick() - m_startTime;
        return static_cast<Nanoseconds>(ms) * 1000000;
#endif
    }

    Microseconds nowMicros() const override {
        return now() / 1000;
    }

    Milliseconds nowMillis() const override {
        return HAL_GetTick() - m_startTime;
    }

    void sleepFor(Nanoseconds ns) override {
        Milliseconds ms = ns / 1000000;
        if (ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(ms));
        } else {
            // For sub-millisecond delays, use busy loop
            busyWaitNanos(ns);
        }
    }

    void sleepMicros(Microseconds us) override {
        if (us >= 1000) {
            vTaskDelay(pdMS_TO_TICKS(us / 1000));
        } else {
            busyWaitNanos(us * 1000);
        }
    }

    void sleepMillis(Milliseconds ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void busyWaitNanos(Nanoseconds ns) override {
#if defined(DWT) && defined(CoreDebug)
        uint32_t startCycles = DWT->CYCCNT;
        uint32_t waitCycles = (ns * SystemCoreClock) / 1000000000ULL;
        while ((DWT->CYCCNT - startCycles) < waitCycles) {
            __NOP();
        }
#else
        // Without DWT, busy wait using loop
        volatile uint32_t count = (ns * SystemCoreClock) / 1000000000ULL / 10;
        while (count--) {
            __NOP();
        }
#endif
    }

    ClockSource getSource() const override {
        return ClockSource::Monotonic;
    }

    Nanoseconds getResolution() const override {
#if defined(DWT) && defined(CoreDebug)
        // Resolution is one CPU cycle
        return 1000000000ULL / SystemCoreClock;
#else
        // Millisecond resolution
        return 1000000;
#endif
    }

private:
    uint32_t m_startTime = 0;
};

// ============================================================================
// STM32 Periodic Timer Implementation (using FreeRTOS software timer)
// ============================================================================

class STM32PeriodicTimer : public IPeriodicTimer {
public:
    STM32PeriodicTimer() = default;

    ~STM32PeriodicTimer() override {
        stop();
    }

    Error start(Microseconds period_us, TimerCallback callback) override {
        if (m_running) return Error::AlreadyInitialized;
        if (!callback) return Error::InvalidParameter;

        m_callback = callback;
        m_period_us = period_us;
        m_running = true;

        // FreeRTOS software timers have millisecond resolution
        TickType_t period_ticks = pdMS_TO_TICKS(period_us / 1000);
        if (period_ticks == 0) period_ticks = 1;

        m_timer = xTimerCreate(
            "hal_periodic",
            period_ticks,
            pdTRUE,  // Auto-reload
            this,
            timerCallback
        );

        if (!m_timer) {
            m_running = false;
            return Error::NoMemory;
        }

        if (xTimerStart(m_timer, portMAX_DELAY) != pdPASS) {
            xTimerDelete(m_timer, portMAX_DELAY);
            m_timer = nullptr;
            m_running = false;
            return Error::InternalError;
        }

        return Error::OK;
    }

    Error stop() override {
        if (!m_running) return Error::OK;

        m_running = false;

        if (m_timer) {
            xTimerStop(m_timer, portMAX_DELAY);
            xTimerDelete(m_timer, portMAX_DELAY);
            m_timer = nullptr;
        }

        return Error::OK;
    }

    Error setPeriod(Microseconds period_us) override {
        if (!m_timer) return Error::NotInitialized;

        TickType_t period_ticks = pdMS_TO_TICKS(period_us / 1000);
        if (period_ticks == 0) period_ticks = 1;

        if (xTimerChangePeriod(m_timer, period_ticks, portMAX_DELAY) != pdPASS) {
            return Error::InternalError;
        }

        m_period_us = period_us;
        return Error::OK;
    }

    Microseconds getPeriod() const override {
        return m_period_us;
    }

    bool isRunning() const override {
        return m_running;
    }

    uint64_t getTickCount() const override {
        return m_tickCount;
    }

    void* nativeHandle() override {
        return m_timer;
    }

private:
    TimerHandle_t m_timer = nullptr;
    TimerCallback m_callback;
    std::atomic<bool> m_running{false};
    Microseconds m_period_us = 0;
    std::atomic<uint64_t> m_tickCount{0};

    static void timerCallback(TimerHandle_t xTimer) {
        auto* self = static_cast<STM32PeriodicTimer*>(pvTimerGetTimerID(xTimer));
        if (self && self->m_running && self->m_callback) {
            self->m_tickCount++;
            self->m_callback();
        }
    }
};

// ============================================================================
// STM32 One-Shot Timer Implementation
// ============================================================================

class STM32OneShotTimer : public IOneShotTimer {
public:
    STM32OneShotTimer() = default;

    ~STM32OneShotTimer() override {
        cancel();
    }

    Error schedule(Microseconds delay_us, TimerCallback callback) override {
        if (m_pending) {
            cancel();
        }

        m_callback = callback;
        m_pending = true;

        TickType_t delay_ticks = pdMS_TO_TICKS(delay_us / 1000);
        if (delay_ticks == 0) delay_ticks = 1;

        m_timer = xTimerCreate(
            "hal_oneshot",
            delay_ticks,
            pdFALSE,  // One-shot
            this,
            timerCallback
        );

        if (!m_timer) {
            m_pending = false;
            return Error::NoMemory;
        }

        if (xTimerStart(m_timer, portMAX_DELAY) != pdPASS) {
            xTimerDelete(m_timer, portMAX_DELAY);
            m_timer = nullptr;
            m_pending = false;
            return Error::InternalError;
        }

        return Error::OK;
    }

    Error cancel() override {
        if (!m_pending || !m_timer) return Error::OK;

        xTimerStop(m_timer, portMAX_DELAY);
        xTimerDelete(m_timer, portMAX_DELAY);
        m_timer = nullptr;
        m_pending = false;

        return Error::OK;
    }

    bool isPending() const override {
        return m_pending;
    }

    void* nativeHandle() override {
        return m_timer;
    }

private:
    TimerHandle_t m_timer = nullptr;
    TimerCallback m_callback;
    std::atomic<bool> m_pending{false};

    static void timerCallback(TimerHandle_t xTimer) {
        auto* self = static_cast<STM32OneShotTimer*>(pvTimerGetTimerID(xTimer));
        if (self) {
            self->m_pending = false;
            if (self->m_callback) {
                self->m_callback();
            }
            // Clean up timer
            xTimerDelete(self->m_timer, 0);
            self->m_timer = nullptr;
        }
    }
};

// ============================================================================
// STM32 High-Resolution Timer (using hardware timer - TIM2)
// For sub-millisecond periodic callbacks
// ============================================================================

class STM32HiResTimer : public IPeriodicTimer {
public:
    STM32HiResTimer() = default;

    ~STM32HiResTimer() override {
        stop();
    }

    Error start(Microseconds period_us, TimerCallback callback) override {
        if (m_running) return Error::AlreadyInitialized;
        if (!callback) return Error::InvalidParameter;

        m_callback = callback;
        m_period_us = period_us;
        
        // Store instance for interrupt handler
        s_instance = this;

        // Configure TIM2 for periodic interrupt
        __HAL_RCC_TIM2_CLK_ENABLE();

        m_htim.Instance = TIM2;
        m_htim.Init.Prescaler = (SystemCoreClock / 1000000) - 1;  // 1 MHz timer clock
        m_htim.Init.CounterMode = TIM_COUNTERMODE_UP;
        m_htim.Init.Period = period_us - 1;
        m_htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        m_htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

        if (HAL_TIM_Base_Init(&m_htim) != HAL_OK) {
            return Error::InternalError;
        }

        // Enable timer interrupt
        HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);

        if (HAL_TIM_Base_Start_IT(&m_htim) != HAL_OK) {
            return Error::InternalError;
        }

        m_running = true;
        return Error::OK;
    }

    Error stop() override {
        if (!m_running) return Error::OK;

        HAL_TIM_Base_Stop_IT(&m_htim);
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        HAL_TIM_Base_DeInit(&m_htim);
        __HAL_RCC_TIM2_CLK_DISABLE();

        s_instance = nullptr;
        m_running = false;
        return Error::OK;
    }

    Error setPeriod(Microseconds period_us) override {
        if (!m_running) return Error::NotInitialized;

        __HAL_TIM_SET_AUTORELOAD(&m_htim, period_us - 1);
        m_period_us = period_us;
        return Error::OK;
    }

    Microseconds getPeriod() const override {
        return m_period_us;
    }

    bool isRunning() const override {
        return m_running;
    }

    uint64_t getTickCount() const override {
        return m_tickCount;
    }

    void* nativeHandle() override {
        return &m_htim;
    }

    // Called from TIM2_IRQHandler
    void handleInterrupt() {
        if (__HAL_TIM_GET_FLAG(&m_htim, TIM_FLAG_UPDATE)) {
            __HAL_TIM_CLEAR_FLAG(&m_htim, TIM_FLAG_UPDATE);
            m_tickCount++;
            if (m_callback) {
                m_callback();
            }
        }
    }

    static STM32HiResTimer* getInstance() { return s_instance; }

private:
    TIM_HandleTypeDef m_htim{};
    TimerCallback m_callback;
    std::atomic<bool> m_running{false};
    Microseconds m_period_us = 0;
    std::atomic<uint64_t> m_tickCount{0};

    static STM32HiResTimer* s_instance;
};

STM32HiResTimer* STM32HiResTimer::s_instance = nullptr;

// TIM2 interrupt handler - must be implemented in user code or linked
extern "C" void TIM2_IRQHandler(void) {
    if (EtherCAT::HAL::STM32HiResTimer::getInstance()) {
        EtherCAT::HAL::STM32HiResTimer::getInstance()->handleInterrupt();
    }
}

// ============================================================================
// STM32 Clock Factory
// ============================================================================

class STM32ClockFactory : public IClockFactory {
public:
    std::unique_ptr<IClock> createClock(ClockSource source) override {
        (void)source;  // Only monotonic available
        return std::make_unique<STM32Clock>();
    }

    std::unique_ptr<IPeriodicTimer> createPeriodicTimer(bool highResolution) override {
        if (highResolution) {
            return std::make_unique<STM32HiResTimer>();
        }
        return std::make_unique<STM32PeriodicTimer>();
    }

    std::unique_ptr<IOneShotTimer> createOneShotTimer() override {
        return std::make_unique<STM32OneShotTimer>();
    }
};

// Clock factory singleton with explicit lifecycle.
// Use resetClockFactory() in tests to ensure clean state.
static std::unique_ptr<IClockFactory> g_clockFactory;

IClockFactory& getClockFactory() {
    if (!g_clockFactory) {
        g_clockFactory = std::make_unique<STM32ClockFactory>();
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

#endif // STM32F4 || STM32F7 || STM32H7 || STM32_HAL
