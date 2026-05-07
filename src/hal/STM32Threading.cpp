/**
 * @file STM32Threading.cpp
 * @brief STM32 FreeRTOS-based threading HAL implementation
 *
 * NOTE: This is a best-effort implementation without testing.
 */

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32_HAL)

#include "hal/IThreading.hpp"
#include "hal/HALTypes.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <cstring>
#include <atomic>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// STM32 Thread Implementation (using FreeRTOS)
// ============================================================================

class STM32Thread : public IThread {
public:
    explicit STM32Thread(const ThreadConfig& config) 
        : m_config(config), m_running(false), m_stopRequested(false) {}

    ~STM32Thread() override {
        if (m_running) {
            requestStop();
            // Note: FreeRTOS doesn't have a clean thread join mechanism
        }
    }

    Error start(ThreadFunc func) override {
        if (m_running) return Error::AlreadyInitialized;

        m_func = func;
        m_stopRequested = false;

        UBaseType_t priority = priorityToFreeRTOS(m_config.priority);
        configSTACK_DEPTH_TYPE stackDepth = m_config.stackSize / sizeof(StackType_t);

        BaseType_t ret = xTaskCreate(
            threadEntry,
            m_config.name ? m_config.name : "hal_task",
            stackDepth,
            this,
            priority,
            &m_taskHandle
        );

        if (ret != pdPASS) {
            return Error::NoMemory;
        }

        m_running = true;
        return Error::OK;
    }

    Error join() override {
        if (!m_running) return Error::NotInitialized;

        // FreeRTOS doesn't have native join - busy wait
        while (m_running) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        return Error::OK;
    }

    Error detach() override {
        // FreeRTOS tasks are always "detached"
        return Error::OK;
    }

    bool isRunning() const override {
        return m_running;
    }

    void requestStop() override {
        m_stopRequested = true;
    }

    bool stopRequested() const override {
        return m_stopRequested;
    }

    void* nativeHandle() override {
        return m_taskHandle;
    }

private:
    ThreadConfig m_config;
    TaskHandle_t m_taskHandle = nullptr;
    ThreadFunc m_func;
    std::atomic<bool> m_running;
    std::atomic<bool> m_stopRequested;

    static void threadEntry(void* arg) {
        auto* self = static_cast<STM32Thread*>(arg);
        
        if (self->m_func) {
            self->m_func();
        }
        
        self->m_running = false;
        vTaskDelete(nullptr);  // Delete self
    }

    static UBaseType_t priorityToFreeRTOS(ThreadPriority priority) {
        switch (priority) {
            case ThreadPriority::Idle:     return tskIDLE_PRIORITY;
            case ThreadPriority::Low:      return tskIDLE_PRIORITY + 1;
            case ThreadPriority::Normal:   return tskIDLE_PRIORITY + 2;
            case ThreadPriority::High:     return configMAX_PRIORITIES - 2;
            case ThreadPriority::Realtime: return configMAX_PRIORITIES - 1;
            default: return tskIDLE_PRIORITY + 2;
        }
    }
};

// ============================================================================
// STM32 Mutex Implementation (using FreeRTOS)
// ============================================================================

class STM32Mutex : public IMutex {
public:
    explicit STM32Mutex(MutexType type) {
        if (type == MutexType::Recursive) {
            m_mutex = xSemaphoreCreateRecursiveMutex();
            m_recursive = true;
        } else {
            m_mutex = xSemaphoreCreateMutex();
            m_recursive = false;
        }
    }

    ~STM32Mutex() override {
        if (m_mutex) {
            vSemaphoreDelete(m_mutex);
        }
    }

    Error lock() override {
        if (!m_mutex) return Error::NotInitialized;
        
        BaseType_t ret;
        if (m_recursive) {
            ret = xSemaphoreTakeRecursive(m_mutex, portMAX_DELAY);
        } else {
            ret = xSemaphoreTake(m_mutex, portMAX_DELAY);
        }
        
        return (ret == pdTRUE) ? Error::OK : Error::InternalError;
    }

    bool tryLock() override {
        if (!m_mutex) return false;
        
        if (m_recursive) {
            return xSemaphoreTakeRecursive(m_mutex, 0) == pdTRUE;
        }
        return xSemaphoreTake(m_mutex, 0) == pdTRUE;
    }

    Error tryLockFor(Milliseconds timeout_ms) override {
        if (!m_mutex) return Error::NotInitialized;
        
        TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
        BaseType_t ret;
        
        if (m_recursive) {
            ret = xSemaphoreTakeRecursive(m_mutex, ticks);
        } else {
            ret = xSemaphoreTake(m_mutex, ticks);
        }
        
        if (ret == pdTRUE) return Error::OK;
        return Error::Timeout;
    }

    Error unlock() override {
        if (!m_mutex) return Error::NotInitialized;
        
        BaseType_t ret;
        if (m_recursive) {
            ret = xSemaphoreGiveRecursive(m_mutex);
        } else {
            ret = xSemaphoreGive(m_mutex);
        }
        
        return (ret == pdTRUE) ? Error::OK : Error::InternalError;
    }

    void* nativeHandle() override {
        return m_mutex;
    }

private:
    SemaphoreHandle_t m_mutex = nullptr;
    bool m_recursive = false;
};

// ============================================================================
// STM32 Condition Variable Implementation
// ============================================================================

class STM32ConditionVariable : public IConditionVariable {
public:
    STM32ConditionVariable() {
        m_semaphore = xSemaphoreCreateBinary();
        m_waitCount = 0;
    }

    ~STM32ConditionVariable() override {
        if (m_semaphore) {
            vSemaphoreDelete(m_semaphore);
        }
    }

    Error wait(UniqueLock& lock) override {
        m_waitCount++;
        lock.unlock();
        
        xSemaphoreTake(m_semaphore, portMAX_DELAY);
        
        lock.lock();
        m_waitCount--;
        return Error::OK;
    }

    Error waitFor(UniqueLock& lock, Milliseconds timeout_ms) override {
        m_waitCount++;
        lock.unlock();
        
        TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
        BaseType_t ret = xSemaphoreTake(m_semaphore, ticks);
        
        lock.lock();
        m_waitCount--;
        
        return (ret == pdTRUE) ? Error::OK : Error::Timeout;
    }

    void notifyOne() override {
        if (m_waitCount > 0) {
            xSemaphoreGive(m_semaphore);
        }
    }

    void notifyAll() override {
        // Wake all waiters
        for (int i = 0; i < m_waitCount; i++) {
            xSemaphoreGive(m_semaphore);
        }
    }

    void* nativeHandle() override {
        return m_semaphore;
    }

private:
    SemaphoreHandle_t m_semaphore = nullptr;
    std::atomic<int> m_waitCount;
};

// ============================================================================
// STM32 Semaphore Implementation
// ============================================================================

class STM32Semaphore : public ISemaphore {
public:
    STM32Semaphore(int initialCount, int maxCount) {
        m_semaphore = xSemaphoreCreateCounting(maxCount, initialCount);
    }

    ~STM32Semaphore() override {
        if (m_semaphore) {
            vSemaphoreDelete(m_semaphore);
        }
    }

    Error wait() override {
        if (!m_semaphore) return Error::NotInitialized;
        
        if (xSemaphoreTake(m_semaphore, portMAX_DELAY) == pdTRUE) {
            return Error::OK;
        }
        return Error::InternalError;
    }

    Error waitFor(Milliseconds timeout_ms) override {
        if (!m_semaphore) return Error::NotInitialized;
        
        if (xSemaphoreTake(m_semaphore, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            return Error::OK;
        }
        return Error::Timeout;
    }

    bool tryWait() override {
        if (!m_semaphore) return false;
        return xSemaphoreTake(m_semaphore, 0) == pdTRUE;
    }

    Error signal() override {
        if (!m_semaphore) return Error::NotInitialized;
        
        if (xSemaphoreGive(m_semaphore) == pdTRUE) {
            return Error::OK;
        }
        return Error::InternalError;
    }

    int getCount() const override {
        if (!m_semaphore) return 0;
        return uxSemaphoreGetCount(m_semaphore);
    }

    void* nativeHandle() override {
        return m_semaphore;
    }

private:
    SemaphoreHandle_t m_semaphore = nullptr;
};

// ============================================================================
// STM32 Event Implementation
// ============================================================================

class STM32Event : public IEvent {
public:
    STM32Event(bool manualReset, bool initialState)
        : m_signaled(initialState), m_manualReset(manualReset) {
        m_mutex = xSemaphoreCreateMutex();
        m_event = xSemaphoreCreateBinary();
        if (initialState && m_event) {
            xSemaphoreGive(m_event);
        }
    }

    ~STM32Event() override {
        if (m_event) vSemaphoreDelete(m_event);
        if (m_mutex) vSemaphoreDelete(m_mutex);
    }

    Error wait() override {
        while (true) {
            xSemaphoreTake(m_mutex, portMAX_DELAY);
            if (m_signaled) {
                if (!m_manualReset) m_signaled = false;
                xSemaphoreGive(m_mutex);
                return Error::OK;
            }
            xSemaphoreGive(m_mutex);
            xSemaphoreTake(m_event, portMAX_DELAY);
        }
    }

    Error waitFor(Milliseconds timeout_ms) override {
        TickType_t start = xTaskGetTickCount();
        TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

        while (true) {
            xSemaphoreTake(m_mutex, portMAX_DELAY);
            if (m_signaled) {
                if (!m_manualReset) m_signaled = false;
                xSemaphoreGive(m_mutex);
                return Error::OK;
            }
            xSemaphoreGive(m_mutex);

            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) return Error::Timeout;

            if (xSemaphoreTake(m_event, timeout - elapsed) != pdTRUE) {
                return Error::Timeout;
            }
        }
    }

    void signal() override {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_signaled = true;
        xSemaphoreGive(m_mutex);
        xSemaphoreGive(m_event);
    }

    void reset() override {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_signaled = false;
        xSemaphoreGive(m_mutex);
    }

    bool isSignaled() const override {
        return m_signaled;
    }

private:
    SemaphoreHandle_t m_mutex = nullptr;
    SemaphoreHandle_t m_event = nullptr;
    bool m_signaled;
    bool m_manualReset;
};

// ============================================================================
// STM32 Threading Factory
// ============================================================================

class STM32ThreadingFactory : public IThreadingFactory {
public:
    std::unique_ptr<IThread> createThread(const ThreadConfig& config) override {
        return std::make_unique<STM32Thread>(config);
    }

    std::unique_ptr<IMutex> createMutex(MutexType type) override {
        return std::make_unique<STM32Mutex>(type);
    }

    std::unique_ptr<IConditionVariable> createConditionVariable() override {
        return std::make_unique<STM32ConditionVariable>();
    }

    std::unique_ptr<ISemaphore> createSemaphore(int initialCount, int maxCount) override {
        return std::make_unique<STM32Semaphore>(initialCount, maxCount);
    }

    std::unique_ptr<IEvent> createEvent(bool manualReset, bool initialState) override {
        return std::make_unique<STM32Event>(manualReset, initialState);
    }

    void sleep(Milliseconds ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void yield() override {
        taskYIELD();
    }

    uint32_t currentThreadId() override {
        return reinterpret_cast<uint32_t>(xTaskGetCurrentTaskHandle());
    }
};

// Threading factory singleton with explicit lifecycle.
// Use resetThreadingFactory() in tests to ensure clean state.
static std::unique_ptr<IThreadingFactory> g_threadingFactory;

IThreadingFactory& getThreadingFactory() {
    if (!g_threadingFactory) {
        g_threadingFactory = std::make_unique<STM32ThreadingFactory>();
    }
    return *g_threadingFactory;
}

void setThreadingFactory(std::unique_ptr<IThreadingFactory> factory) {
    g_threadingFactory = std::move(factory);
}

void resetThreadingFactory() {
    g_threadingFactory.reset();
}

} // namespace hal
} // namespace EtherCAT

#endif // STM32F4 || STM32F7 || STM32H7 || STM32_HAL
