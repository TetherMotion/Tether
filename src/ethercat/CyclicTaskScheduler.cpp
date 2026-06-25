/**
 * @file CyclicTaskScheduler.cpp
 * @brief CyclicTaskScheduler implementation
 */

#include "tether/ethercat/CyclicTaskScheduler.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"
#include <algorithm>

namespace EtherCAT {

bool CyclicTaskScheduler::entryLess(const Entry& a, const Entry& b) {
    if (static_cast<uint8_t>(a.phase) != static_cast<uint8_t>(b.phase)) {
        return static_cast<uint8_t>(a.phase) < static_cast<uint8_t>(b.phase);
    }
    return a.priority < b.priority;
}

void CyclicTaskScheduler::rebuildSchedule() {
    // Called with mutex_ held
    std::vector<ICyclicTask*> new_schedule;
    new_schedule.reserve(entries_.size());
    for (const auto& e : entries_) {
        if (e.task) {
            new_schedule.push_back(e.task);
        }
    }
    schedule_ = std::move(new_schedule);
    schedule_dirty_.store(false, std::memory_order_release);
}

TaskHandle CyclicTaskScheduler::addTask(ICyclicTask* task, TaskPhase phase, uint8_t priority) {
    if (!task) return {};

    std::lock_guard<std::mutex> lock(mutex_);
    uint32_t id = next_id_.fetch_add(1, std::memory_order_relaxed);
    if (id == 0) {
        id = next_id_.fetch_add(1, std::memory_order_relaxed);
    }

    entries_.push_back({task, phase, priority, id});
    std::sort(entries_.begin(), entries_.end(), entryLess);
    rebuildSchedule();

    return {id};
}

bool CyclicTaskScheduler::removeTask(TaskHandle handle) {
    if (!handle.valid()) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find_if(entries_.begin(), entries_.end(),
        [handle](const Entry& e) { return e.id == handle.id; });
    if (it == entries_.end()) return false;

    entries_.erase(it);
    rebuildSchedule();
    return true;
}

void CyclicTaskScheduler::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
    schedule_.clear();
    schedule_dirty_.store(true, std::memory_order_release);
}

bool CyclicTaskScheduler::executeAll(DS402Master& master, double dt_seconds) {
    if (schedule_dirty_.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(mutex_);
        rebuildSchedule();
    }

    for (ICyclicTask* task : schedule_) {
        if (task && !task->update(master, dt_seconds)) {
            return false;
        }
    }
    return true;
}

size_t CyclicTaskScheduler::taskCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
}

} // namespace EtherCAT
