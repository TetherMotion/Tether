/**
 * @file LinuxThreading.cpp
 * @brief Linux pthread-based threading HAL implementation
 */

#ifdef __linux__

#include "hal/IThreading.hpp"
#include "hal/HALTypes.hpp"

#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <cerrno>
#include <climits>
#include <atomic>
#include <cstring>
#include <cstdio>

// For realtime scheduling
#include <sched.h>
#include <sys/resource.h>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Linux Thread Implementation
// ============================================================================

class LinuxThread : public IThread {
public:
    explicit LinuxThread(const ThreadConfig& config) 
        : m_config(config), m_running(false), m_stopRequested(false) {}

    ~LinuxThread() override {
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

        // Set scheduling policy for realtime
        if (m_config.useRealtimeScheduling) {
            pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
            struct sched_param param;
            param.sched_priority = priorityToLinux(m_config.priority);
            pthread_attr_setschedparam(&attr, &param);
            pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        }

        int ret = pthread_create(&m_thread, &attr, threadEntry, this);
        pthread_attr_destroy(&attr);

        if (ret != 0) {
            // If realtime scheduling failed due to insufficient permissions,
            // retry without realtime scheduling as a graceful fallback.
            if (ret == EPERM && m_config.useRealtimeScheduling) {
                fprintf(stderr, "[W] LinuxThread: SCHED_FIFO requires CAP_SYS_NICE; "
                        "falling back to normal scheduling for '%s'\n",
                        m_config.name ? m_config.name : "unnamed");
                pthread_attr_t attr2;
                pthread_attr_init(&attr2);
                if (m_config.stackSize > 0) {
                    pthread_attr_setstacksize(&attr2, m_config.stackSize);
                }
                ret = pthread_create(&m_thread, &attr2, threadEntry, this);
                pthread_attr_destroy(&attr2);
                if (ret != 0) {
                    return Error::InternalError;
                }
                // Fall through to success path
            } else {
                if (ret == EPERM) {
                    return Error::PermissionDenied;
                }
                return Error::InternalError;
            }
        }

        m_running = true;

        // Set thread name
        if (m_config.name) {
            pthread_setname_np(m_thread, m_config.name);
        }

        // Set CPU affinity
        if (m_config.cpuAffinity >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(m_config.cpuAffinity, &cpuset);
            pthread_setaffinity_np(m_thread, sizeof(cpuset), &cpuset);
        }

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
            m_running = false;  // We no longer own the thread
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
        auto* self = static_cast<LinuxThread*>(arg);
        if (self->m_func) {
            self->m_func();
        }
        // Don't set m_running = false here - that's done by join() or detach()
        // The thread is still joinable even after the function returns
        return nullptr;
    }

    static int priorityToLinux(ThreadPriority priority) {
        switch (priority) {
            case ThreadPriority::Idle:     return 1;
            case ThreadPriority::Low:      return 25;
            case ThreadPriority::Normal:   return 50;
            case ThreadPriority::High:     return 75;
            case ThreadPriority::Realtime: return 99;
            default: return 50;
        }
    }
};

// ============================================================================
// Linux Mutex Implementation
// ============================================================================

class LinuxMutex : public IMutex {
public:
    explicit LinuxMutex(MutexType type) {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);

        if (type == MutexType::Recursive) {
            pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        }

        pthread_mutex_init(&m_mutex, &attr);
        pthread_mutexattr_destroy(&attr);
    }

    ~LinuxMutex() override {
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
// Linux Condition Variable Implementation
// ============================================================================

class LinuxConditionVariable : public IConditionVariable {
public:
    LinuxConditionVariable() {
        pthread_cond_init(&m_cond, nullptr);
    }

    ~LinuxConditionVariable() override {
        pthread_cond_destroy(&m_cond);
    }

    Error wait(UniqueLock& lock) override {
        auto* mutex = static_cast<pthread_mutex_t*>(
            static_cast<LinuxMutex*>(lock.mutex())->nativeHandle());
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

        auto* mutex = static_cast<pthread_mutex_t*>(
            static_cast<LinuxMutex*>(lock.mutex())->nativeHandle());
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
// Linux Semaphore Implementation
// ============================================================================

class LinuxSemaphore : public ISemaphore {
public:
    LinuxSemaphore(int initialCount, int maxCount) 
        : m_count(initialCount), m_maxCount(maxCount) {
        pthread_mutex_init(&m_mutex, nullptr);
        pthread_cond_init(&m_cond, nullptr);
    }

    ~LinuxSemaphore() override {
        pthread_cond_destroy(&m_cond);
        pthread_mutex_destroy(&m_mutex);
    }

    Error wait() override {
        pthread_mutex_lock(&m_mutex);
        while (m_count <= 0) {
            pthread_cond_wait(&m_cond, &m_mutex);
        }
        m_count--;
        pthread_mutex_unlock(&m_mutex);
        return Error::OK;
    }

    Error waitFor(Milliseconds timeout_ms) override {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        pthread_mutex_lock(&m_mutex);
        while (m_count <= 0) {
            int ret = pthread_cond_timedwait(&m_cond, &m_mutex, &ts);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&m_mutex);
                return Error::Timeout;
            }
        }
        m_count--;
        pthread_mutex_unlock(&m_mutex);
        return Error::OK;
    }

    bool tryWait() override {
        pthread_mutex_lock(&m_mutex);
        if (m_count > 0) {
            m_count--;
            pthread_mutex_unlock(&m_mutex);
            return true;
        }
        pthread_mutex_unlock(&m_mutex);
        return false;
    }

    Error signal() override {
        pthread_mutex_lock(&m_mutex);
        if (m_count < m_maxCount) {
            m_count++;
            pthread_cond_signal(&m_cond);
        }
        pthread_mutex_unlock(&m_mutex);
        return Error::OK;
    }

    int getCount() const override {
        return m_count;
    }

    void* nativeHandle() override {
        return nullptr;  // No single native handle
    }

private:
    pthread_mutex_t m_mutex;
    pthread_cond_t m_cond;
    int m_count;
    int m_maxCount;
};

// ============================================================================
// Linux Event Implementation
// ============================================================================

class LinuxEvent : public IEvent {
public:
    LinuxEvent(bool manualReset, bool initialState)
        : m_signaled(initialState), m_manualReset(manualReset) {
        pthread_mutex_init(&m_mutex, nullptr);
        pthread_cond_init(&m_cond, nullptr);
    }

    ~LinuxEvent() override {
        pthread_cond_destroy(&m_cond);
        pthread_mutex_destroy(&m_mutex);
    }

    Error wait() override {
        pthread_mutex_lock(&m_mutex);
        while (!m_signaled) {
            pthread_cond_wait(&m_cond, &m_mutex);
        }
        if (!m_manualReset) {
            m_signaled = false;
        }
        pthread_mutex_unlock(&m_mutex);
        return Error::OK;
    }

    Error waitFor(Milliseconds timeout_ms) override {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        pthread_mutex_lock(&m_mutex);
        while (!m_signaled) {
            int ret = pthread_cond_timedwait(&m_cond, &m_mutex, &ts);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&m_mutex);
                return Error::Timeout;
            }
        }
        if (!m_manualReset) {
            m_signaled = false;
        }
        pthread_mutex_unlock(&m_mutex);
        return Error::OK;
    }

    void signal() override {
        pthread_mutex_lock(&m_mutex);
        m_signaled = true;
        if (m_manualReset) {
            pthread_cond_broadcast(&m_cond);
        } else {
            pthread_cond_signal(&m_cond);
        }
        pthread_mutex_unlock(&m_mutex);
    }

    void reset() override {
        pthread_mutex_lock(&m_mutex);
        m_signaled = false;
        pthread_mutex_unlock(&m_mutex);
    }

    bool isSignaled() const override {
        return m_signaled;
    }

private:
    pthread_mutex_t m_mutex;
    pthread_cond_t m_cond;
    bool m_signaled;
    bool m_manualReset;
};

// ============================================================================
// Linux Queue Implementation
// ============================================================================

/**
 * @brief Thread-safe queue using pthread mutex and condition variable
 * 
 * Implements FreeRTOS-compatible queue semantics with proper timeout support.
 */
class LinuxQueue : public IQueue {
public:
    LinuxQueue(size_t itemSize, size_t capacity)
        : m_itemSize(itemSize)
        , m_capacity(capacity)
        , m_head(0)
        , m_tail(0)
        , m_count(0)
    {
        // Allocate circular buffer
        m_buffer = std::make_unique<uint8_t[]>(itemSize * capacity);
        pthread_mutex_init(&m_mutex, nullptr);
        pthread_cond_init(&m_notEmpty, nullptr);
        pthread_cond_init(&m_notFull, nullptr);
    }
    
    ~LinuxQueue() override {
        pthread_cond_destroy(&m_notFull);
        pthread_cond_destroy(&m_notEmpty);
        pthread_mutex_destroy(&m_mutex);
    }
    
    Error send(const void* item, Milliseconds timeout_ms) override {
        if (!item) return Error::InvalidArgument;
        
        pthread_mutex_lock(&m_mutex);
        
        // Wait for space if full
        while (m_count >= m_capacity) {
            if (timeout_ms == 0) {
                pthread_mutex_unlock(&m_mutex);
                return Error::WouldBlock;
            }
            
            if (timeout_ms == UINT32_MAX) {
                pthread_cond_wait(&m_notFull, &m_mutex);
            } else {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_sec += timeout_ms / 1000;
                ts.tv_nsec += (timeout_ms % 1000) * 1000000;
                if (ts.tv_nsec >= 1000000000) {
                    ts.tv_sec++;
                    ts.tv_nsec -= 1000000000;
                }
                
                int ret = pthread_cond_timedwait(&m_notFull, &m_mutex, &ts);
                if (ret == ETIMEDOUT) {
                    pthread_mutex_unlock(&m_mutex);
                    return Error::Timeout;
                }
            }
        }
        
        // Copy item to tail
        memcpy(m_buffer.get() + (m_tail * m_itemSize), item, m_itemSize);
        m_tail = (m_tail + 1) % m_capacity;
        m_count++;
        
        pthread_cond_signal(&m_notEmpty);
        pthread_mutex_unlock(&m_mutex);
        
        return Error::OK;
    }
    
    Error sendToFront(const void* item, Milliseconds timeout_ms) override {
        if (!item) return Error::InvalidArgument;
        
        pthread_mutex_lock(&m_mutex);
        
        // Wait for space if full
        while (m_count >= m_capacity) {
            if (timeout_ms == 0) {
                pthread_mutex_unlock(&m_mutex);
                return Error::WouldBlock;
            }
            
            if (timeout_ms == UINT32_MAX) {
                pthread_cond_wait(&m_notFull, &m_mutex);
            } else {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_sec += timeout_ms / 1000;
                ts.tv_nsec += (timeout_ms % 1000) * 1000000;
                if (ts.tv_nsec >= 1000000000) {
                    ts.tv_sec++;
                    ts.tv_nsec -= 1000000000;
                }
                
                int ret = pthread_cond_timedwait(&m_notFull, &m_mutex, &ts);
                if (ret == ETIMEDOUT) {
                    pthread_mutex_unlock(&m_mutex);
                    return Error::Timeout;
                }
            }
        }
        
        // Move head back and copy item
        m_head = (m_head + m_capacity - 1) % m_capacity;
        memcpy(m_buffer.get() + (m_head * m_itemSize), item, m_itemSize);
        m_count++;
        
        pthread_cond_signal(&m_notEmpty);
        pthread_mutex_unlock(&m_mutex);
        
        return Error::OK;
    }
    
    Error receive(void* item, Milliseconds timeout_ms) override {
        if (!item) return Error::InvalidArgument;
        
        pthread_mutex_lock(&m_mutex);
        
        // Wait for data if empty
        while (m_count == 0) {
            if (timeout_ms == 0) {
                pthread_mutex_unlock(&m_mutex);
                return Error::WouldBlock;
            }
            
            if (timeout_ms == UINT32_MAX) {
                pthread_cond_wait(&m_notEmpty, &m_mutex);
            } else {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_sec += timeout_ms / 1000;
                ts.tv_nsec += (timeout_ms % 1000) * 1000000;
                if (ts.tv_nsec >= 1000000000) {
                    ts.tv_sec++;
                    ts.tv_nsec -= 1000000000;
                }
                
                int ret = pthread_cond_timedwait(&m_notEmpty, &m_mutex, &ts);
                if (ret == ETIMEDOUT) {
                    pthread_mutex_unlock(&m_mutex);
                    return Error::Timeout;
                }
            }
        }
        
        // Copy item from head
        memcpy(item, m_buffer.get() + (m_head * m_itemSize), m_itemSize);
        m_head = (m_head + 1) % m_capacity;
        m_count--;
        
        pthread_cond_signal(&m_notFull);
        pthread_mutex_unlock(&m_mutex);
        
        return Error::OK;
    }
    
    Error peek(void* item) const override {
        if (!item) return Error::InvalidArgument;

        pthread_mutex_lock(&m_mutex);

        if (m_count == 0) {
            pthread_mutex_unlock(&m_mutex);
            return Error::Empty;
        }

        memcpy(item, m_buffer.get() + (m_head * m_itemSize), m_itemSize);

        pthread_mutex_unlock(&m_mutex);
        return Error::OK;
    }

    size_t getCount() const override {
        pthread_mutex_lock(&m_mutex);
        size_t count = m_count;
        pthread_mutex_unlock(&m_mutex);
        return count;
    }
    
    size_t getCapacity() const override {
        return m_capacity;
    }
    
    size_t getItemSize() const override {
        return m_itemSize;
    }
    
    bool isEmpty() const override {
        return getCount() == 0;
    }
    
    bool isFull() const override {
        return getCount() >= m_capacity;
    }
    
    void clear() override {
        pthread_mutex_lock(&m_mutex);
        m_head = 0;
        m_tail = 0;
        m_count = 0;
        pthread_cond_broadcast(&m_notFull);
        pthread_mutex_unlock(&m_mutex);
    }
    
    size_t getAvailableSpace() const override {
        return m_capacity - getCount();
    }
    
private:
    size_t m_itemSize;
    size_t m_capacity;
    std::unique_ptr<uint8_t[]> m_buffer;
    size_t m_head;
    size_t m_tail;
    size_t m_count;
    mutable pthread_mutex_t m_mutex;
    pthread_cond_t m_notEmpty;
    pthread_cond_t m_notFull;
};

// ============================================================================
// Linux Threading Factory
// ============================================================================

class LinuxThreadingFactory : public IThreadingFactory {
public:
    std::unique_ptr<IThread> createThread(const ThreadConfig& config) override {
        return std::make_unique<LinuxThread>(config);
    }

    std::unique_ptr<IMutex> createMutex(MutexType type) override {
        return std::make_unique<LinuxMutex>(type);
    }

    std::unique_ptr<IConditionVariable> createConditionVariable() override {
        return std::make_unique<LinuxConditionVariable>();
    }

    std::unique_ptr<ISemaphore> createSemaphore(int initialCount, int maxCount) override {
        return std::make_unique<LinuxSemaphore>(initialCount, maxCount);
    }

    std::unique_ptr<IEvent> createEvent(bool manualReset, bool initialState) override {
        return std::make_unique<LinuxEvent>(manualReset, initialState);
    }

    std::unique_ptr<IQueue> createQueue(size_t itemSize, size_t capacity) override {
        return std::make_unique<LinuxQueue>(itemSize, capacity);
    }

    void sleep(Milliseconds ms) override {
        usleep(ms * 1000);
    }

    void yield() override {
        sched_yield();
    }

    uint32_t currentThreadId() override {
        return static_cast<uint32_t>(pthread_self());
    }
};

// Threading factory singleton with explicit lifecycle.
// Use resetThreadingFactory() in tests to ensure clean state.
static std::unique_ptr<IThreadingFactory> g_threadingFactory;

IThreadingFactory& getThreadingFactory() {
    if (!g_threadingFactory) {
        g_threadingFactory = std::make_unique<LinuxThreadingFactory>();
    }
    return *g_threadingFactory;
}

void setThreadingFactory(std::unique_ptr<IThreadingFactory> factory) {
    g_threadingFactory = std::move(factory);
}

void resetThreadingFactory() {
    g_threadingFactory.reset();
}

// Convenience factory functions for direct creation
std::unique_ptr<IMutex> createLinuxMutex() {
    return std::make_unique<LinuxMutex>(MutexType::Normal);
}

std::unique_ptr<ISemaphore> createLinuxSemaphore(int initialCount) {
    return std::make_unique<LinuxSemaphore>(initialCount, INT_MAX);
}

std::unique_ptr<IEvent> createLinuxEvent(bool manualReset, bool initialState) {
    return std::make_unique<LinuxEvent>(manualReset, initialState);
}

std::unique_ptr<IConditionVariable> createLinuxConditionVariable() {
    return std::make_unique<LinuxConditionVariable>();
}

std::unique_ptr<IThread> createLinuxThread(const ThreadConfig& config) {
    return std::make_unique<LinuxThread>(config);
}

} // namespace HAL
} // namespace EtherCAT

#endif // __linux__
