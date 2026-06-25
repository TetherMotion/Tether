#pragma once

/**
 * @file CyclicTaskScheduler.hpp
 * @brief Pre-sorted, thread-safe cyclic task scheduler for realtime loops
 *
 * Tasks register with a phase position and priority. The scheduler maintains
 * a pre-sorted vector of raw pointers that the RT thread iterates without
 * any locks or allocations.
 */

#include <cstdint>
#include <mutex>
#include <vector>
#include <atomic>

namespace EtherCAT {

class DS402Master;

enum class TaskPhase : uint8_t {
    PreExchange   = 0,
    Exchange      = 10,
    PostExchange  = 20,
    MotionControl = 30,
    Diagnostics   = 40,
};

struct TaskHandle {
    uint32_t id = 0;
    bool valid() const { return id != 0; }
};

class ICyclicTask {
public:
    virtual ~ICyclicTask() = default;
    virtual bool update(DS402Master& master, double dt_seconds) = 0;
};

class CyclicTaskScheduler {
public:
    CyclicTaskScheduler() = default;
    ~CyclicTaskScheduler() = default;

    CyclicTaskScheduler(const CyclicTaskScheduler&) = delete;
    CyclicTaskScheduler& operator=(const CyclicTaskScheduler&) = delete;

    /// Register a task with phase + priority (lower = earlier within same phase).
    /// The task pointer must remain valid until removed.
    /// Thread-safe.
    TaskHandle addTask(ICyclicTask* task, TaskPhase phase, uint8_t priority = 128);

    /// Remove a task by handle. Thread-safe.
    bool removeTask(TaskHandle handle);

    /// Clear all tasks. Thread-safe.
    void clear();

    /// Execute all tasks in pre-sorted order. Called from RT thread.
    /// Lock-free: iterates a snapshot of raw pointers.
    bool executeAll(DS402Master& master, double dt_seconds);

    /// Number of registered tasks.
    size_t taskCount() const;

private:
    struct Entry {
        ICyclicTask* task = nullptr;
        TaskPhase phase = TaskPhase::MotionControl;
        uint8_t priority = 128;
        uint32_t id = 0;
    };

    static bool entryLess(const Entry& a, const Entry& b);

    void rebuildSchedule();

    mutable std::mutex mutex_;
    std::vector<Entry> entries_;
    std::vector<ICyclicTask*> schedule_;
    std::atomic<uint32_t> next_id_{1};
    std::atomic<bool> schedule_dirty_{true};
};

} // namespace EtherCAT
