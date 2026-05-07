/**
 * @file ESP32Threading.cpp
 * @brief ESP32 FreeRTOS-based threading HAL implementation
 *
 * Uses FreeRTOS primitives with pthread compatibility where available.
 */

#if defined(ESP_PLATFORM) || defined(ESP32)

#include "hal/IThreading.hpp"
#include "hal/HALTypes.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <cstring>
#include <atomic>

// ESP-IDF includes pthread support
#include <pthread.h>
#include <time.h>

static const char* TAG = "ESP32Thread";

namespace EtherCAT {
namespace HAL {

// ============================================================================
// ESP32 Thread Implementation (using pthread)
// ============================================================================

class ESP32Thread : public IThread {
public:
    explicit ESP32Thread(const ThreadConfig& config) 
        : m_config(config), m_running(false), m_stopRequested(false) {}

    ~ESP32Thread() override {
        if (m_running) {
            requestStop();
            join();
        }
    }

    Error start(ThreadFunc func) override {
        if (m_running) return Error::AlreadyInitialized;

        m_func = func;
        m_stopRequested = false;

        pthread_attr_t attr;
        pthread_attr_init(&attr);

        // Set stack size
        if (m_config.stackSize > 0) {
            pthread_attr_setstacksize(&attr, m_config.stackSize);
        }

        int ret = pthread_create(&m_thread, &attr, threadEntry, this);
        pthread_attr_destroy(&attr);

        if (ret != 0) {
            TETHER_LOGE(TAG, "pthread_create failed: %d", ret);
            return Error::InternalError;
        }

        m_running = true;

        // Set thread name via FreeRTOS (pthread doesn't expose this well)
        // The thread name is set inside the thread function instead

        return Error::OK;
    }

    Error join() override {
        if (!m_running) return Error::NotInitialized;

        int ret = pthread_join(m_thread, nullptr);
        m_running = false;

        return (ret == 0) ? Error::OK : Error::InternalError;
    }

    Error detach() override {
        if (!m_running) return Error::NotInitialized;

        int ret = pthread_detach(m_thread);
        if (ret == 0) {
            m_running = false;
        }

        return (ret == 0) ? Error::OK : Error::InternalError;
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
        return reinterpret_cast<void*>(m_thread);
    }

private:
    ThreadConfig m_config;
    pthread_t m_thread;
    ThreadFunc m_func;
    std::atomic<bool> m_running;
    std::atomic<bool> m_stopRequested;

    static void* threadEntry(void* arg) {
        auto* self = static_cast<ESP32Thread*>(arg);
        
        // Set task priority via FreeRTOS
        UBaseType_t priority = priorityToFreeRTOS(self->m_config.priority);
        vTaskPrioritySet(nullptr, priority);

        if (self->m_func) {
            self->m_func();
        }
        
        self->m_running = false;
        return nullptr;
    }

    static UBaseType_t priorityToFreeRTOS(ThreadPriority priority) {
        switch (priority) {
            case ThreadPriority::Idle:     return tskIDLE_PRIORITY;
            case ThreadPriority::Low:      return tskIDLE_PRIORITY + 2;
            case ThreadPriority::Normal:   return tskIDLE_PRIORITY + 5;
            case ThreadPriority::High:     return configMAX_PRIORITIES - 3;
            case ThreadPriority::Realtime: return configMAX_PRIORITIES - 1;
            default: return tskIDLE_PRIORITY + 5;
        }
    }
};

// ============================================================================
// ESP32 Mutex Implementation (using pthread)
// ============================================================================

class ESP32Mutex : public IMutex {
public:
    explicit ESP32Mutex(MutexType type) {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);

        if (type == MutexType::Recursive) {
            pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        }

        pthread_mutex_init(&m_mutex, &attr);
        pthread_mutexattr_destroy(&attr);
    }

    ~ESP32Mutex() override {
        pthread_mutex_destroy(&m_mutex);
    }

    Error lock() override {
        int ret = pthread_mutex_lock(&m_mutex);
        return (ret == 0) ? Error::OK : Error::InternalError;
    }

    bool tryLock() override {
        return pthread_mutex_trylock(&m_mutex) == 0;
    }

    Error tryLockFor(Milliseconds timeout_ms) override {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        int ret = pthread_mutex_timedlock(&m_mutex, &ts);
        if (ret == 0) return Error::OK;
        if (ret == ETIMEDOUT) return Error::Timeout;
        return Error::InternalError;
    }

    Error unlock() override {
        int ret = pthread_mutex_unlock(&m_mutex);
        return (ret == 0) ? Error::OK : Error::InternalError;
    }

    void* nativeHandle() override {
        return &m_mutex;
    }

private:
    pthread_mutex_t m_mutex;
};

// ============================================================================
// ESP32 Condition Variable Implementation
// ============================================================================

class ESP32ConditionVariable : public IConditionVariable {
public:
    ESP32ConditionVariable() {
        pthread_cond_init(&m_cond, nullptr);
    }

    ~ESP32ConditionVariable() override {
        pthread_cond_destroy(&m_cond);
    }

    Error wait(UniqueLock& lock) override {
        // Get the pthread_mutex from the IMutex
        // This assumes ESP32Mutex is being used
        pthread_mutex_t* mutex = static_cast<pthread_mutex_t*>(
            lock.m_mutex->nativeHandle());
        int ret = pthread_cond_wait(&m_cond, mutex);
        return (ret == 0) ? Error::OK : Error::InternalError;
    }

    Error waitFor(UniqueLock& lock, Milliseconds timeout_ms) override {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        pthread_mutex_t* mutex = static_cast<pthread_mutex_t*>(
            lock.m_mutex->nativeHandle());
        int ret = pthread_cond_timedwait(&m_cond, mutex, &ts);
        
        if (ret == 0) return Error::OK;
        if (ret == ETIMEDOUT) return Error::Timeout;
        return Error::InternalError;
    }

    void notifyOne() override {
        pthread_cond_signal(&m_cond);
    }

    void notifyAll() override {
        pthread_cond_broadcast(&m_cond);
    }

    void* nativeHandle() override {
        return &m_cond;
    }

private:
    pthread_cond_t m_cond;
};

// ============================================================================
// ESP32 Semaphore Implementation (using FreeRTOS)
// ============================================================================

class ESP32Semaphore : public ISemaphore {
public:
    ESP32Semaphore(int initialCount, int maxCount) : m_maxCount(maxCount) {
        m_semaphore = xSemaphoreCreateCounting(maxCount, initialCount);
    }

    ~ESP32Semaphore() override {
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
        
        TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
        if (xSemaphoreTake(m_semaphore, ticks) == pdTRUE) {
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
        return Error::InternalError;  // Semaphore at max count
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
    int m_maxCount;
};

// ============================================================================
// ESP32 Event Implementation (using FreeRTOS task notifications)
// ============================================================================

class ESP32Event : public IEvent {
public:
    ESP32Event(bool manualReset, bool initialState)
        : m_signaled(initialState), m_manualReset(manualReset) {
        m_mutex = xSemaphoreCreateMutex();
        m_event = xSemaphoreCreateBinary();
        if (initialState && m_event) {
            xSemaphoreGive(m_event);
        }
    }

    ~ESP32Event() override {
        if (m_event) vSemaphoreDelete(m_event);
        if (m_mutex) vSemaphoreDelete(m_mutex);
    }

    Error wait() override {
        if (!m_event) return Error::NotInitialized;
        
        while (true) {
            xSemaphoreTake(m_mutex, portMAX_DELAY);
            if (m_signaled) {
                if (!m_manualReset) {
                    m_signaled = false;
                }
                xSemaphoreGive(m_mutex);
                return Error::OK;
            }
            xSemaphoreGive(m_mutex);
            
            // Wait for signal
            xSemaphoreTake(m_event, portMAX_DELAY);
        }
    }

    Error waitFor(Milliseconds timeout_ms) override {
        if (!m_event) return Error::NotInitialized;
        
        TickType_t start = xTaskGetTickCount();
        TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
        
        while (true) {
            xSemaphoreTake(m_mutex, portMAX_DELAY);
            if (m_signaled) {
                if (!m_manualReset) {
                    m_signaled = false;
                }
                xSemaphoreGive(m_mutex);
                return Error::OK;
            }
            xSemaphoreGive(m_mutex);
            
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) {
                return Error::Timeout;
            }
            
            TickType_t remaining = timeout - elapsed;
            if (xSemaphoreTake(m_event, remaining) != pdTRUE) {
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
// ESP32 Threading Factory
// ============================================================================

class ESP32ThreadingFactory : public IThreadingFactory {
public:
    std::unique_ptr<IThread> createThread(const ThreadConfig& config) override {
        return std::make_unique<ESP32Thread>(config);
    }

    std::unique_ptr<IMutex> createMutex(MutexType type) override {
        return std::make_unique<ESP32Mutex>(type);
    }

    std::unique_ptr<IConditionVariable> createConditionVariable() override {
        return std::make_unique<ESP32ConditionVariable>();
    }

    std::unique_ptr<ISemaphore> createSemaphore(int initialCount, int maxCount) override {
        return std::make_unique<ESP32Semaphore>(initialCount, maxCount);
    }

    std::unique_ptr<IEvent> createEvent(bool manualReset, bool initialState) override {
        return std::make_unique<ESP32Event>(manualReset, initialState);
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
        g_threadingFactory = std::make_unique<ESP32ThreadingFactory>();
    }
    return *g_threadingFactory;
}

void setThreadingFactory(std::unique_ptr<IThreadingFactory> factory) {
    g_threadingFactory = std::move(factory);
}

void resetThreadingFactory() {
    g_threadingFactory.reset();
}

} // namespace HAL
} // namespace EtherCAT

#endif // ESP_PLATFORM || ESP32
