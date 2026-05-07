/**
 * @file test_threading.cpp
 * @brief Unit tests for threading primitives
 */

#include <gtest/gtest.h>
#include "tether/hal/IThreading.hpp"
#include <atomic>
#include <chrono>
#include <thread>

using namespace EtherCAT::HAL;

// Forward declaration for LinuxThreading factory functions
namespace EtherCAT {
namespace HAL {
std::unique_ptr<IMutex> createLinuxMutex();
std::unique_ptr<ISemaphore> createLinuxSemaphore(int initialCount);
std::unique_ptr<IEvent> createLinuxEvent(bool manualReset, bool initialState);
std::unique_ptr<IConditionVariable> createLinuxConditionVariable();
std::unique_ptr<IThread> createLinuxThread(const ThreadConfig& config);
}
}

// ============================================================================
// Mutex Tests
// ============================================================================

class MutexTest : public ::testing::Test {
protected:
    std::unique_ptr<IMutex> mutex;
    
    void SetUp() override {
        mutex = createLinuxMutex();
        ASSERT_NE(mutex, nullptr);
    }
};

TEST_F(MutexTest, LockUnlock) {
    EXPECT_EQ(mutex->lock(), Error::OK);
    EXPECT_EQ(mutex->unlock(), Error::OK);
}

TEST_F(MutexTest, TryLock) {
    EXPECT_TRUE(mutex->tryLock());
    // Already locked, tryLock should fail
    EXPECT_FALSE(mutex->tryLock());
    EXPECT_EQ(mutex->unlock(), Error::OK);
    // Now it should succeed
    EXPECT_TRUE(mutex->tryLock());
    EXPECT_EQ(mutex->unlock(), Error::OK);
}

TEST_F(MutexTest, LockGuardRAII) {
    {
        LockGuard guard(*mutex);
        // Mutex should be locked here
        EXPECT_FALSE(mutex->tryLock());
    }
    // Mutex should be unlocked here
    EXPECT_TRUE(mutex->tryLock());
    mutex->unlock();
}

TEST_F(MutexTest, UniqueLockRAII) {
    {
        UniqueLock lock(*mutex);
        EXPECT_TRUE(lock.ownsLock());
        EXPECT_FALSE(mutex->tryLock());
        
        lock.unlock();
        EXPECT_FALSE(lock.ownsLock());
        EXPECT_TRUE(mutex->tryLock());
        mutex->unlock();
        
        lock.lock();
        EXPECT_TRUE(lock.ownsLock());
    }
    // Mutex should be unlocked after UniqueLock destructor
    EXPECT_TRUE(mutex->tryLock());
    mutex->unlock();
}

// ============================================================================
// Semaphore Tests
// ============================================================================

class SemaphoreTest : public ::testing::Test {
protected:
    std::unique_ptr<ISemaphore> sem;
    
    void SetUp() override {
        sem = createLinuxSemaphore(0);
        ASSERT_NE(sem, nullptr);
    }
};

TEST_F(SemaphoreTest, BasicSignalWait) {
    EXPECT_EQ(sem->getCount(), 0);
    
    EXPECT_EQ(sem->signal(), Error::OK);
    EXPECT_EQ(sem->getCount(), 1);
    
    EXPECT_EQ(sem->wait(), Error::OK);
    EXPECT_EQ(sem->getCount(), 0);
}

TEST_F(SemaphoreTest, TryWait) {
    EXPECT_FALSE(sem->tryWait());  // Empty
    
    sem->signal();
    EXPECT_TRUE(sem->tryWait());   // Now has count
    EXPECT_FALSE(sem->tryWait()); // Empty again
}

TEST_F(SemaphoreTest, WaitForTimeout) {
    // Wait should timeout since semaphore is empty
    auto start = std::chrono::steady_clock::now();
    Error err = sem->waitFor(50);  // 50ms timeout
    auto end = std::chrono::steady_clock::now();
    
    EXPECT_EQ(err, Error::Timeout);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_GE(elapsed.count(), 40);  // Should wait at least 40ms
}

TEST_F(SemaphoreTest, CountingSemaphore) {
    auto countingSem = createLinuxSemaphore(3);
    EXPECT_EQ(countingSem->getCount(), 3);
    
    EXPECT_TRUE(countingSem->tryWait());
    EXPECT_TRUE(countingSem->tryWait());
    EXPECT_TRUE(countingSem->tryWait());
    EXPECT_FALSE(countingSem->tryWait());  // Now empty
    
    countingSem->signal();
    countingSem->signal();
    EXPECT_EQ(countingSem->getCount(), 2);
}

// ============================================================================
// Event Tests
// ============================================================================

class EventTest : public ::testing::Test {
protected:
    std::unique_ptr<IEvent> event;
    
    void SetUp() override {
        event = createLinuxEvent(false, false);  // auto-reset, not signaled
        ASSERT_NE(event, nullptr);
    }
};

TEST_F(EventTest, SignalAndWait) {
    EXPECT_FALSE(event->isSignaled());
    
    event->signal();
    EXPECT_TRUE(event->isSignaled());
    
    // Wait should succeed
    EXPECT_EQ(event->wait(), Error::OK);
}

TEST_F(EventTest, WaitForTimeout) {
    auto start = std::chrono::steady_clock::now();
    Error err = event->waitFor(50);  // 50ms timeout
    auto end = std::chrono::steady_clock::now();
    
    EXPECT_EQ(err, Error::Timeout);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_GE(elapsed.count(), 40);
}

TEST_F(EventTest, ManualResetEvent) {
    auto manualEvent = createLinuxEvent(true, false);  // manual-reset
    
    manualEvent->signal();
    EXPECT_TRUE(manualEvent->isSignaled());
    
    EXPECT_EQ(manualEvent->waitFor(10), Error::OK);
    // Manual reset - should still be signaled
    EXPECT_TRUE(manualEvent->isSignaled());
    
    manualEvent->reset();
    EXPECT_FALSE(manualEvent->isSignaled());
}

// ============================================================================
// Condition Variable Tests
// ============================================================================

class ConditionVariableTest : public ::testing::Test {
protected:
    std::unique_ptr<IMutex> mutex;
    std::unique_ptr<IConditionVariable> cv;
    
    void SetUp() override {
        mutex = createLinuxMutex();
        cv = createLinuxConditionVariable();
        ASSERT_NE(mutex, nullptr);
        ASSERT_NE(cv, nullptr);
    }
};

TEST_F(ConditionVariableTest, NotifyOne) {
    std::atomic<bool> ready{false};
    std::atomic<bool> processed{false};
    
    std::thread worker([this, &ready, &processed]() {
        UniqueLock lock(*mutex);
        while (!ready) {
            cv->waitFor(lock, 100);
        }
        processed = true;
    });
    
    // Give worker time to start waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    {
        LockGuard guard(*mutex);
        ready = true;
    }
    cv->notifyOne();
    
    worker.join();
    EXPECT_TRUE(processed);
}

TEST_F(ConditionVariableTest, NotifyAll) {
    std::atomic<int> counter{0};
    std::atomic<bool> ready{false};
    
    auto workerFunc = [this, &ready, &counter]() {
        UniqueLock lock(*mutex);
        while (!ready) {
            cv->waitFor(lock, 100);
        }
        counter++;
    };
    
    std::thread worker1(workerFunc);
    std::thread worker2(workerFunc);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    {
        LockGuard guard(*mutex);
        ready = true;
    }
    cv->notifyAll();
    
    worker1.join();
    worker2.join();
    EXPECT_EQ(counter.load(), 2);
}

// ============================================================================
// Thread Tests
// ============================================================================

TEST(ThreadTest, CreateAndJoin) {
    std::atomic<int> value{0};
    
    ThreadConfig config;
    config.name = "test_thread";
    config.priority = ThreadPriority::Normal;
    
    auto thread = createLinuxThread(config);
    ASSERT_NE(thread, nullptr);
    
    Error err = thread->start([&value]() {
        value++;
    });
    
    EXPECT_EQ(err, Error::OK);
    EXPECT_EQ(thread->join(), Error::OK);
    EXPECT_EQ(value.load(), 1);
}

TEST(ThreadTest, StopRequest) {
    ThreadConfig config;
    config.name = "stop_test";
    
    auto thread = createLinuxThread(config);
    ASSERT_NE(thread, nullptr);
    
    std::atomic<bool> started{false};
    
    Error err = thread->start([&thread, &started]() {
        started = true;
        while (!thread->stopRequested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    EXPECT_EQ(err, Error::OK);
    
    // Wait for thread to start
    while (!started) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    EXPECT_TRUE(thread->isRunning());
    thread->requestStop();
    EXPECT_EQ(thread->join(), Error::OK);
    EXPECT_FALSE(thread->isRunning());
}

// ============================================================================
// Integration Tests with multiple primitives
// ============================================================================

TEST(ThreadingIntegrationTest, ProducerConsumer) {
    auto sem = createLinuxSemaphore(0);
    
    const int itemCount = 10;
    std::atomic<int> consumed{0};
    
    ThreadConfig config;
    config.name = "producer";
    
    auto producerThread = createLinuxThread(config);
    
    Error err = producerThread->start([&sem]() {
        for (int i = 0; i < 10; i++) {
            sem->signal();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    
    EXPECT_EQ(err, Error::OK);
    
    // Consumer in main thread
    for (int i = 0; i < itemCount; i++) {
        Error waitErr = sem->waitFor(1000);
        if (waitErr == Error::OK) {
            consumed++;
        }
    }
    
    producerThread->join();
    EXPECT_EQ(consumed.load(), itemCount);
}
