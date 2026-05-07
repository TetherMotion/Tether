/**
 * @file EspCompat.hpp
 * @brief ESP-IDF compatibility layer using HAL implementations
 * 
 * Provides drop-in replacements for FreeRTOS and ESP-IDF APIs using the HAL.
 * This allows code written for ESP32/FreeRTOS to work on other platforms.
 * 
 * On ESP32: Uses native FreeRTOS/ESP-IDF directly
 * On Linux/STM32: Uses HAL implementations (pthread, CMSIS-RTOS)
 */

#pragma once

#include "tether/platform/Platform.hpp"

// Only define these when not building for ESP-IDF
#ifndef ESP_PLATFORM

#include "hal/IThreading.hpp"
#include "hal/IClock.hpp"
#include <memory>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <thread>

//=============================================================================
// ESP Log Compatibility
//=============================================================================

#define ESP_LOGE(tag, format, ...) TETHER_LOGE(tag, format, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) TETHER_LOGW(tag, format, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) TETHER_LOGI(tag, format, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) TETHER_LOGD(tag, format, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) TETHER_LOGV(tag, format, ##__VA_ARGS__)

// Log buffer functions (minimal implementation)
#define ESP_LOG_BUFFER_HEX(tag, buf, len) do {} while(0)
#define ESP_LOG_BUFFER_HEXDUMP(tag, buf, len, level) do {} while(0)
#define ESP_LOG_BUFFER_HEX_LEVEL(tag, buf, len, level) do {} while(0)

// ESP log levels
#define ESP_LOG_NONE    0
#define ESP_LOG_ERROR   1
#define ESP_LOG_WARN    2
#define ESP_LOG_INFO    3
#define ESP_LOG_DEBUG   4
#define ESP_LOG_VERBOSE 5

// ESP log level type
typedef enum {
    ESP_LOG_LEVEL_NONE = ESP_LOG_NONE,
    ESP_LOG_LEVEL_ERROR = ESP_LOG_ERROR,
    ESP_LOG_LEVEL_WARN = ESP_LOG_WARN,
    ESP_LOG_LEVEL_INFO = ESP_LOG_INFO,
    ESP_LOG_LEVEL_DEBUG = ESP_LOG_DEBUG,
    ESP_LOG_LEVEL_VERBOSE = ESP_LOG_VERBOSE
} esp_log_level_t;

//=============================================================================
// ESP Error Codes
//=============================================================================

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT 0x107

inline const char* esp_err_to_name(esp_err_t err) {
    switch (err) {
        case ESP_OK: return "ESP_OK";
        case ESP_FAIL: return "ESP_FAIL";
        case ESP_ERR_NO_MEM: return "ESP_ERR_NO_MEM";
        case ESP_ERR_INVALID_ARG: return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_TIMEOUT: return "ESP_ERR_TIMEOUT";
        default: return "UNKNOWN";
    }
}

//=============================================================================
// FreeRTOS Types
//=============================================================================

typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* TaskHandle_t;
typedef void* EventGroupHandle_t;
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;

// FreeRTOS constants
#define portMAX_DELAY 0xFFFFFFFFUL
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS pdTRUE
#define pdFAIL pdFALSE
#define configTICK_RATE_HZ 1000
#define portTICK_PERIOD_MS (1000 / configTICK_RATE_HZ)
#define configMAX_PRIORITIES 25
#define tskIDLE_PRIORITY 0
#define tskNO_AFFINITY  ((UBaseType_t)0x7FFFFFFF)  // Task can run on any core

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms) / portTICK_PERIOD_MS)

//=============================================================================
// ESP Timer Compatibility (using HAL)
//=============================================================================

inline int64_t esp_timer_get_time() {
    return Tether::Platform::Clock::instance().getMicroseconds();
}

//=============================================================================
// ESP ROM Delay Compatibility (using HAL)
//=============================================================================

inline void esp_rom_delay_us(uint32_t us) {
    Tether::Platform::Clock::instance().delayMicroseconds(us);
}

//=============================================================================
// FreeRTOS Task Delay Compatibility (using HAL)
//=============================================================================

inline void vTaskDelay(TickType_t ticks) {
    if (ticks == 0) {
        EtherCAT::HAL::getThreadingFactory().yield();
    } else {
        EtherCAT::HAL::getThreadingFactory().sleep(ticks * portTICK_PERIOD_MS);
    }
}

inline void taskYIELD() {
    EtherCAT::HAL::getThreadingFactory().yield();
}

inline TickType_t xTaskGetTickCount() {
    return static_cast<TickType_t>(
        Tether::Platform::Clock::instance().getMilliseconds() / portTICK_PERIOD_MS);
}

//=============================================================================
// FreeRTOS Queue Implementation (using HAL IQueue)
//=============================================================================

namespace FreeRTOSCompat {

// Global registry of HAL objects (thread-safe)
class HalObjectRegistry {
public:
    static HalObjectRegistry& instance() {
        static HalObjectRegistry inst;
        return inst;
    }
    
    // Queue management
    QueueHandle_t createQueue(UBaseType_t length, UBaseType_t itemSize) {
        auto queue = EtherCAT::HAL::getThreadingFactory().createQueue(itemSize, length);
        if (!queue) return nullptr;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        auto ptr = queue.get();
        m_queues[ptr] = std::move(queue);
        return ptr;
    }
    
    EtherCAT::HAL::IQueue* getQueue(QueueHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_queues.find(handle);
        return (it != m_queues.end()) ? it->second.get() : nullptr;
    }
    
    void deleteQueue(QueueHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queues.erase(handle);
    }
    
    // Semaphore/Mutex management
    SemaphoreHandle_t createMutex() {
        auto mutex = EtherCAT::HAL::getThreadingFactory().createMutex(EtherCAT::HAL::MutexType::Recursive);
        if (!mutex) return nullptr;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        auto ptr = mutex.get();
        m_mutexes[ptr] = std::move(mutex);
        return ptr;
    }
    
    SemaphoreHandle_t createBinarySemaphore() {
        auto sem = EtherCAT::HAL::getThreadingFactory().createSemaphore(0, 1);
        if (!sem) return nullptr;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        auto ptr = sem.get();
        m_semaphores[ptr] = std::move(sem);
        return ptr;
    }
    
    EtherCAT::HAL::IMutex* getMutex(SemaphoreHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_mutexes.find(handle);
        return (it != m_mutexes.end()) ? it->second.get() : nullptr;
    }
    
    EtherCAT::HAL::ISemaphore* getSemaphore(SemaphoreHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_semaphores.find(handle);
        return (it != m_semaphores.end()) ? it->second.get() : nullptr;
    }
    
    void deleteSemaphore(SemaphoreHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_mutexes.erase(handle);
        m_semaphores.erase(handle);
    }
    
    // Task management
    struct TaskInfo {
        std::unique_ptr<EtherCAT::HAL::IThread> thread;
        std::unique_ptr<EtherCAT::HAL::IEvent> notifyEvent;
        std::atomic<uint32_t> notifyValue{0};
    };
    
    TaskHandle_t createTask(void (*fn)(void*), const char* name, uint32_t stackSize,
                            void* params, UBaseType_t prio, TaskHandle_t* handleOut, int core = -1) {
        EtherCAT::HAL::ThreadConfig config;
        config.name = name;
        config.stackSize = stackSize;
        config.cpuAffinity = core;
        
        // Map FreeRTOS priority to HAL priority
        if (prio <= tskIDLE_PRIORITY) {
            config.priority = EtherCAT::HAL::ThreadPriority::Idle;
        } else if (prio < configMAX_PRIORITIES / 3) {
            config.priority = EtherCAT::HAL::ThreadPriority::Low;
        } else if (prio < 2 * configMAX_PRIORITIES / 3) {
            config.priority = EtherCAT::HAL::ThreadPriority::Normal;
        } else if (prio < configMAX_PRIORITIES - 1) {
            config.priority = EtherCAT::HAL::ThreadPriority::High;
        } else {
            config.priority = EtherCAT::HAL::ThreadPriority::Realtime;
            config.useRealtimeScheduling = true;
        }
        
        auto thread = EtherCAT::HAL::getThreadingFactory().createThread(config);
        auto event = EtherCAT::HAL::getThreadingFactory().createEvent(false, false);
        if (!thread || !event) return nullptr;
        
        auto info = std::make_unique<TaskInfo>();
        info->thread = std::move(thread);
        info->notifyEvent = std::move(event);
        
        auto rawPtr = info.get();
        TaskHandle_t handle = rawPtr;
        
        // Start the thread with the user function
        auto err = info->thread->start([fn, params]() { fn(params); });
        if (err != EtherCAT::HAL::Error::OK) {
            return nullptr;
        }
        
        std::lock_guard<std::mutex> lock(m_mutex);
        m_tasks[handle] = std::move(info);
        
        if (handleOut) *handleOut = handle;
        return handle;
    }
    
    TaskInfo* getTask(TaskHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_tasks.find(handle);
        return (it != m_tasks.end()) ? it->second.get() : nullptr;
    }
    
    void deleteTask(TaskHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_tasks.find(handle);
        if (it != m_tasks.end()) {
            if (it->second->thread) {
                it->second->thread->requestStop();
                // Don't join here - could cause deadlock if task deletes itself
            }
            m_tasks.erase(it);
        }
    }
    
    // Current task tracking per thread
    void setCurrentTask(TaskHandle_t handle) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_currentTask[std::this_thread::get_id()] = handle;
    }
    
    TaskHandle_t getCurrentTask() {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_currentTask.find(std::this_thread::get_id());
        return (it != m_currentTask.end()) ? it->second : nullptr;
    }
    
private:
    std::mutex m_mutex;
    std::unordered_map<void*, std::unique_ptr<EtherCAT::HAL::IQueue>> m_queues;
    std::unordered_map<void*, std::unique_ptr<EtherCAT::HAL::IMutex>> m_mutexes;
    std::unordered_map<void*, std::unique_ptr<EtherCAT::HAL::ISemaphore>> m_semaphores;
    std::unordered_map<void*, std::unique_ptr<TaskInfo>> m_tasks;
    std::unordered_map<std::thread::id, TaskHandle_t> m_currentTask;
};

} // namespace FreeRTOSCompat

//=============================================================================
// FreeRTOS Queue API (using HAL)
//=============================================================================

inline QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t itemSize) {
    return FreeRTOSCompat::HalObjectRegistry::instance().createQueue(length, itemSize);
}

inline BaseType_t xQueueSend(QueueHandle_t queue, const void* item, TickType_t wait) {
    auto* q = FreeRTOSCompat::HalObjectRegistry::instance().getQueue(queue);
    if (!q) return pdFALSE;
    
    EtherCAT::HAL::Milliseconds timeout = (wait == portMAX_DELAY) ? UINT32_MAX : wait * portTICK_PERIOD_MS;
    auto err = q->send(item, timeout);
    return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
}

inline BaseType_t xQueueSendToBack(QueueHandle_t queue, const void* item, TickType_t wait) {
    return xQueueSend(queue, item, wait);
}

inline BaseType_t xQueueSendToFront(QueueHandle_t queue, const void* item, TickType_t wait) {
    auto* q = FreeRTOSCompat::HalObjectRegistry::instance().getQueue(queue);
    if (!q) return pdFALSE;
    
    EtherCAT::HAL::Milliseconds timeout = (wait == portMAX_DELAY) ? UINT32_MAX : wait * portTICK_PERIOD_MS;
    auto err = q->sendToFront(item, timeout);
    return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
}

inline BaseType_t xQueueReceive(QueueHandle_t queue, void* item, TickType_t wait) {
    auto* q = FreeRTOSCompat::HalObjectRegistry::instance().getQueue(queue);
    if (!q) return pdFALSE;
    
    EtherCAT::HAL::Milliseconds timeout = (wait == portMAX_DELAY) ? UINT32_MAX : wait * portTICK_PERIOD_MS;
    auto err = q->receive(item, timeout);
    return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
}

inline BaseType_t xQueuePeek(QueueHandle_t queue, void* item, TickType_t wait) {
    (void)wait; // Peek doesn't wait in our implementation
    auto* q = FreeRTOSCompat::HalObjectRegistry::instance().getQueue(queue);
    if (!q) return pdFALSE;
    
    auto err = q->peek(item);
    return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
}

inline void vQueueDelete(QueueHandle_t queue) {
    FreeRTOSCompat::HalObjectRegistry::instance().deleteQueue(queue);
}

inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue) {
    auto* q = FreeRTOSCompat::HalObjectRegistry::instance().getQueue(queue);
    return q ? static_cast<UBaseType_t>(q->getCount()) : 0;
}

inline UBaseType_t uxQueueSpacesAvailable(QueueHandle_t queue) {
    auto* q = FreeRTOSCompat::HalObjectRegistry::instance().getQueue(queue);
    return q ? static_cast<UBaseType_t>(q->getAvailableSpace()) : 0;
}

//=============================================================================
// FreeRTOS Semaphore/Mutex API (using HAL)
//=============================================================================

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
    return FreeRTOSCompat::HalObjectRegistry::instance().createMutex();
}

inline SemaphoreHandle_t xSemaphoreCreateBinary() {
    return FreeRTOSCompat::HalObjectRegistry::instance().createBinarySemaphore();
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t wait) {
    // Check if it's a mutex
    auto* mutex = FreeRTOSCompat::HalObjectRegistry::instance().getMutex(sem);
    if (mutex) {
        EtherCAT::HAL::Milliseconds timeout = (wait == portMAX_DELAY) ? UINT32_MAX : wait * portTICK_PERIOD_MS;
        auto err = mutex->tryLockFor(timeout);
        return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
    }
    
    // Check if it's a semaphore
    auto* semaphore = FreeRTOSCompat::HalObjectRegistry::instance().getSemaphore(sem);
    if (semaphore) {
        EtherCAT::HAL::Milliseconds timeout = (wait == portMAX_DELAY) ? UINT32_MAX : wait * portTICK_PERIOD_MS;
        auto err = semaphore->waitFor(timeout);
        return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
    }
    
    return pdFALSE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) {
    // Check if it's a mutex
    auto* mutex = FreeRTOSCompat::HalObjectRegistry::instance().getMutex(sem);
    if (mutex) {
        auto err = mutex->unlock();
        return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
    }
    
    // Check if it's a semaphore
    auto* semaphore = FreeRTOSCompat::HalObjectRegistry::instance().getSemaphore(sem);
    if (semaphore) {
        auto err = semaphore->signal();
        return (err == EtherCAT::HAL::Error::OK) ? pdTRUE : pdFALSE;
    }
    
    return pdFALSE;
}

inline void vSemaphoreDelete(SemaphoreHandle_t sem) {
    FreeRTOSCompat::HalObjectRegistry::instance().deleteSemaphore(sem);
}

//=============================================================================
// FreeRTOS Task API (using HAL)
//=============================================================================

inline BaseType_t xTaskCreate(void (*fn)(void*), const char* name, uint32_t stackSize,
                              void* params, UBaseType_t prio, TaskHandle_t* handle) {
    auto h = FreeRTOSCompat::HalObjectRegistry::instance().createTask(fn, name, stackSize, params, prio, handle);
    return (h != nullptr) ? pdPASS : pdFAIL;
}

inline BaseType_t xTaskCreatePinnedToCore(void (*fn)(void*), const char* name, uint32_t stackSize,
                                          void* params, UBaseType_t prio, TaskHandle_t* handle, int core) {
    auto h = FreeRTOSCompat::HalObjectRegistry::instance().createTask(fn, name, stackSize, params, prio, handle, core);
    return (h != nullptr) ? pdPASS : pdFAIL;
}

inline void vTaskDelete(TaskHandle_t task) {
    if (task == nullptr) {
        // Delete calling task - can't fully support this, but mark for deletion
        task = FreeRTOSCompat::HalObjectRegistry::instance().getCurrentTask();
    }
    if (task) {
        FreeRTOSCompat::HalObjectRegistry::instance().deleteTask(task);
    }
}

inline TaskHandle_t xTaskGetCurrentTaskHandle() {
    return FreeRTOSCompat::HalObjectRegistry::instance().getCurrentTask();
}

inline void xTaskNotifyGive(TaskHandle_t task) {
    auto* info = FreeRTOSCompat::HalObjectRegistry::instance().getTask(task);
    if (info && info->notifyEvent) {
        info->notifyValue.fetch_add(1);
        info->notifyEvent->signal();
    }
}

inline uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t wait) {
    auto* info = FreeRTOSCompat::HalObjectRegistry::instance().getTask(
        FreeRTOSCompat::HalObjectRegistry::instance().getCurrentTask());
    if (!info || !info->notifyEvent) return 0;
    
    EtherCAT::HAL::Milliseconds timeout = (wait == portMAX_DELAY) ? UINT32_MAX : wait * portTICK_PERIOD_MS;
    auto err = info->notifyEvent->waitFor(timeout);
    
    if (err == EtherCAT::HAL::Error::OK) {
        uint32_t value = info->notifyValue.load();
        if (clearOnExit) {
            info->notifyValue.store(0);
        }
        return value;
    }
    return 0;
}

// Ethernet ioctl commands
#define ETH_CMD_G_MAC_ADDR 0x01
#define ETH_CMD_S_PROMISCUOUS 0x02
#define ETH_CMD_G_PHY_ADDR 0x03

inline esp_err_t esp_eth_transmit(void* eth, void* buf, size_t len) {
    // This needs to be connected to the actual HAL ethernet implementation
    // For now, return fail - ethernet should be initialized through HAL
    (void)eth; (void)buf; (void)len;
    return ESP_FAIL;
}

inline esp_err_t esp_eth_ioctl(void* eth, int cmd, void* arg) {
    (void)eth; (void)cmd; (void)arg;
    return ESP_FAIL;
}

#endif // ESP_PLATFORM
