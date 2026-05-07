/**
 * @file IThreading.hpp
 * @brief Threading abstraction interfaces for the Hardware Abstraction Layer
 *
 * This header defines interfaces for threading primitives that can be implemented
 * for different platforms (Linux pthread, FreeRTOS, STM32).
 */

#pragma once

#include "hal/HALTypes.hpp"
#include <functional>
#include <memory>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Thread Interface
// ============================================================================

/**
 * @brief Thread priority levels
 */
enum class ThreadPriority {
    Idle,           ///< Lowest priority
    Low,            ///< Below normal priority
    Normal,         ///< Default priority
    High,           ///< Above normal priority
    Realtime,       ///< Highest priority (realtime if supported)
};

/**
 * @brief Thread configuration
 */
struct ThreadConfig {
    const char* name = "thread";      ///< Thread name (for debugging)
    size_t stackSize = 4096;          ///< Stack size in bytes
    ThreadPriority priority = ThreadPriority::Normal;
    int cpuAffinity = -1;             ///< CPU affinity (-1 = any)
    bool useRealtimeScheduling = false; ///< Use SCHED_FIFO on Linux
};

/**
 * @brief Thread function type
 */
using ThreadFunc = std::function<void()>;

/**
 * @brief Abstract thread interface
 */
class IThread {
public:
    virtual ~IThread() = default;
    
    /**
     * @brief Start the thread
     * @param func Function to execute
     * @return Error code
     */
    virtual Error start(ThreadFunc func) = 0;
    
    /**
     * @brief Wait for thread to complete
     * @return Error code
     */
    virtual Error join() = 0;
    
    /**
     * @brief Detach the thread
     * @return Error code
     */
    virtual Error detach() = 0;
    
    /**
     * @brief Check if thread is running
     */
    virtual bool isRunning() const = 0;
    
    /**
     * @brief Request thread to stop (cooperative)
     */
    virtual void requestStop() = 0;
    
    /**
     * @brief Check if stop was requested
     */
    virtual bool stopRequested() const = 0;
    
    /**
     * @brief Get native thread handle (platform-specific)
     */
    virtual void* nativeHandle() = 0;
};

// ============================================================================
// Mutex Interface
// ============================================================================

/**
 * @brief Mutex type
 */
enum class MutexType {
    Normal,         ///< Standard mutex
    Recursive,      ///< Can be locked multiple times by same thread
};

/**
 * @brief Abstract mutex interface
 */
class IMutex {
public:
    virtual ~IMutex() = default;
    
    /**
     * @brief Lock the mutex (blocking)
     * @return Error code
     */
    virtual Error lock() = 0;
    
    /**
     * @brief Try to lock the mutex (non-blocking)
     * @return true if locked, false if would block
     */
    virtual bool tryLock() = 0;
    
    /**
     * @brief Try to lock with timeout
     * @param timeout_ms Timeout in milliseconds
     * @return Error code (Timeout if expired)
     */
    virtual Error tryLockFor(Milliseconds timeout_ms) = 0;
    
    /**
     * @brief Unlock the mutex
     * @return Error code
     */
    virtual Error unlock() = 0;
    
    /**
     * @brief Get native mutex handle
     */
    virtual void* nativeHandle() = 0;
};

/**
 * @brief RAII lock guard for IMutex
 */
class LockGuard {
public:
    explicit LockGuard(IMutex& mutex) : m_mutex(mutex) {
        m_mutex.lock();
    }
    
    ~LockGuard() {
        m_mutex.unlock();
    }
    
    // Non-copyable
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;
    
private:
    IMutex& m_mutex;
};

/**
 * @brief RAII unique lock for IMutex (supports unlock/relock)
 */
class UniqueLock {
public:
    explicit UniqueLock(IMutex& mutex, bool doLock = true) 
        : m_mutex(&mutex), m_locked(false) {
        if (doLock) {
            m_mutex->lock();
            m_locked = true;
        }
    }
    
    ~UniqueLock() {
        if (m_locked) {
            m_mutex->unlock();
        }
    }
    
    void lock() {
        if (!m_locked) {
            m_mutex->lock();
            m_locked = true;
        }
    }
    
    void unlock() {
        if (m_locked) {
            m_mutex->unlock();
            m_locked = false;
        }
    }
    
    bool ownsLock() const { return m_locked; }
    
    /**
     * @brief Get the underlying mutex pointer
     * @return Pointer to the mutex, or nullptr if moved
     */
    IMutex* mutex() const { return m_mutex; }
    
    // Non-copyable
    UniqueLock(const UniqueLock&) = delete;
    UniqueLock& operator=(const UniqueLock&) = delete;
    
    // Moveable
    UniqueLock(UniqueLock&& other) noexcept 
        : m_mutex(other.m_mutex), m_locked(other.m_locked) {
        other.m_mutex = nullptr;
        other.m_locked = false;
    }
    
private:
    IMutex* m_mutex;
    bool m_locked;
};

// ============================================================================
// Condition Variable Interface
// ============================================================================

/**
 * @brief Abstract condition variable interface
 */
class IConditionVariable {
public:
    virtual ~IConditionVariable() = default;
    
    /**
     * @brief Wait on the condition variable
     * @param lock Lock to release while waiting
     * @return Error code
     */
    virtual Error wait(UniqueLock& lock) = 0;
    
    /**
     * @brief Wait with timeout
     * @param lock Lock to release while waiting
     * @param timeout_ms Timeout in milliseconds
     * @return Error code (Timeout if expired)
     */
    virtual Error waitFor(UniqueLock& lock, Milliseconds timeout_ms) = 0;
    
    /**
     * @brief Wait with predicate
     * @param lock Lock to release while waiting
     * @param pred Predicate that must return true to stop waiting
     * @return Error code
     */
    template<typename Predicate>
    Error waitUntil(UniqueLock& lock, Predicate pred) {
        while (!pred()) {
            Error err = wait(lock);
            if (err != Error::OK) return err;
        }
        return Error::OK;
    }
    
    /**
     * @brief Wait with timeout and predicate
     */
    template<typename Predicate>
    Error waitForUntil(UniqueLock& lock, Milliseconds timeout_ms, Predicate pred) {
        while (!pred()) {
            Error err = waitFor(lock, timeout_ms);
            if (err != Error::OK) return err;
        }
        return Error::OK;
    }
    
    /**
     * @brief Wake one waiting thread
     */
    virtual void notifyOne() = 0;
    
    /**
     * @brief Wake all waiting threads
     */
    virtual void notifyAll() = 0;
    
    /**
     * @brief Get native handle
     */
    virtual void* nativeHandle() = 0;
};

// ============================================================================
// Semaphore Interface
// ============================================================================

/**
 * @brief Abstract semaphore interface
 */
class ISemaphore {
public:
    virtual ~ISemaphore() = default;
    
    /**
     * @brief Wait (decrement) the semaphore
     * @return Error code
     */
    virtual Error wait() = 0;
    
    /**
     * @brief Wait with timeout
     * @param timeout_ms Timeout in milliseconds
     * @return Error code
     */
    virtual Error waitFor(Milliseconds timeout_ms) = 0;
    
    /**
     * @brief Try to wait (non-blocking)
     * @return true if decremented, false if would block
     */
    virtual bool tryWait() = 0;
    
    /**
     * @brief Signal (increment) the semaphore
     * @return Error code
     */
    virtual Error signal() = 0;
    
    /**
     * @brief Get current count
     */
    virtual int getCount() const = 0;
    
    /**
     * @brief Get native handle
     */
    virtual void* nativeHandle() = 0;
};

// ============================================================================
// Event (Binary Semaphore) Interface
// ============================================================================

/**
 * @brief Abstract event interface (like Windows event or FreeRTOS notification)
 */
class IEvent {
public:
    virtual ~IEvent() = default;
    
    /**
     * @brief Wait for event to be signaled
     * @return Error code
     */
    virtual Error wait() = 0;
    
    /**
     * @brief Wait with timeout
     * @param timeout_ms Timeout in milliseconds
     * @return Error code
     */
    virtual Error waitFor(Milliseconds timeout_ms) = 0;
    
    /**
     * @brief Signal the event (wake one or all waiters depending on type)
     */
    virtual void signal() = 0;
    
    /**
     * @brief Reset the event (for manual-reset events)
     */
    virtual void reset() = 0;
    
    /**
     * @brief Check if event is signaled
     */
    virtual bool isSignaled() const = 0;
};

// ============================================================================
// Queue Interface
// ============================================================================

/**
 * @brief Abstract thread-safe queue interface
 * 
 * Provides FreeRTOS-compatible queue semantics for inter-thread communication.
 * Items are copied into the queue (value semantics).
 */
class IQueue {
public:
    virtual ~IQueue() = default;
    
    /**
     * @brief Send an item to the back of the queue
     * @param item Pointer to item data (must be itemSize bytes)
     * @param timeout_ms Timeout in milliseconds (0 = non-blocking, UINT32_MAX = infinite)
     * @return Error::OK on success, Error::Timeout on timeout, Error::BufferFull if full
     */
    virtual Error send(const void* item, Milliseconds timeout_ms = UINT32_MAX) = 0;
    
    /**
     * @brief Send an item to the front of the queue (high priority)
     * @param item Pointer to item data
     * @param timeout_ms Timeout in milliseconds
     * @return Error code
     */
    virtual Error sendToFront(const void* item, Milliseconds timeout_ms = UINT32_MAX) = 0;
    
    /**
     * @brief Receive an item from the queue
     * @param item Pointer to buffer for item data (must be at least itemSize bytes)
     * @param timeout_ms Timeout in milliseconds (0 = non-blocking, UINT32_MAX = infinite)
     * @return Error::OK on success, Error::Timeout on timeout
     */
    virtual Error receive(void* item, Milliseconds timeout_ms = UINT32_MAX) = 0;
    
    /**
     * @brief Peek at the front item without removing it
     * @param item Pointer to buffer for item data
     * @return Error::OK if item available, Error::Empty if queue is empty
     */
    virtual Error peek(void* item) const = 0;
    
    /**
     * @brief Get number of items currently in the queue
     */
    virtual size_t getCount() const = 0;
    
    /**
     * @brief Get maximum queue capacity
     */
    virtual size_t getCapacity() const = 0;
    
    /**
     * @brief Get size of each item in bytes
     */
    virtual size_t getItemSize() const = 0;
    
    /**
     * @brief Check if queue is empty
     */
    virtual bool isEmpty() const = 0;
    
    /**
     * @brief Check if queue is full
     */
    virtual bool isFull() const = 0;
    
    /**
     * @brief Clear all items from queue
     */
    virtual void clear() = 0;
    
    /**
     * @brief Get number of free spaces in the queue
     */
    virtual size_t getAvailableSpace() const = 0;
};

// ============================================================================
// Factory Interface
// ============================================================================

/**
 * @brief Factory for creating threading primitives
 */
class IThreadingFactory {
public:
    virtual ~IThreadingFactory() = default;
    
    /**
     * @brief Create a thread
     */
    virtual std::unique_ptr<IThread> createThread(const ThreadConfig& config = {}) = 0;
    
    /**
     * @brief Create a mutex
     */
    virtual std::unique_ptr<IMutex> createMutex(MutexType type = MutexType::Normal) = 0;
    
    /**
     * @brief Create a condition variable
     */
    virtual std::unique_ptr<IConditionVariable> createConditionVariable() = 0;
    
    /**
     * @brief Create a semaphore
     */
    virtual std::unique_ptr<ISemaphore> createSemaphore(int initialCount = 0, int maxCount = 1) = 0;
    
    /**
     * @brief Create an event
     */
    virtual std::unique_ptr<IEvent> createEvent(bool manualReset = false, bool initialState = false) = 0;
    
    /**
     * @brief Create a queue
     * @param itemSize Size of each item in bytes
     * @param capacity Maximum number of items the queue can hold
     * @return New queue instance
     */
    virtual std::unique_ptr<IQueue> createQueue(size_t itemSize, size_t capacity) = 0;
    
    /**
     * @brief Sleep current thread
     */
    virtual void sleep(Milliseconds ms) = 0;
    
    /**
     * @brief Yield current thread
     */
    virtual void yield() = 0;
    
    /**
     * @brief Get current thread ID
     */
    virtual uint32_t currentThreadId() = 0;
};

// ============================================================================
// Global Factory Access
// ============================================================================

/**
 * @brief Get platform-specific threading factory
 */
IThreadingFactory& getThreadingFactory();

/**
 * @brief Set custom threading factory (for testing)
 */
void setThreadingFactory(std::unique_ptr<IThreadingFactory> factory);

/**
 * @brief Reset threading factory to platform default (for testing)
 *
 * Destroys the current threading factory singleton, allowing it to be
 * re-created with platform defaults on next access. Primarily used
 * in unit tests to ensure clean state between test cases.
 */
void resetThreadingFactory();

} // namespace HAL
} // namespace EtherCAT
