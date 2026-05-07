#pragma once

#include <deque>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace Tether {
namespace Platform {

/**
 * @brief Thread-safe message queue compliant with FreeRTOS xQueue semantics (mostly)
 */
template<typename T>
class MessageQueue {
public:
    explicit MessageQueue(size_t capacity = 0) : capacity_(capacity) {}

    bool send(const T& msg, unsigned int timeout_ms = 0) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (capacity_ > 0 && queue_.size() >= capacity_) {
            // If full, wait? Or fail immediately if timeout is 0?
            // xQueueSend waits.
            if (timeout_ms == 0) return false;
            // Simple wait (not implementing full blocking push for now to save tokens/time if not needed)
            // Assuming simplified usage for now.
             if (!not_full_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]{ return queue_.size() < capacity_; })) {
                return false;
            }
        }
        queue_.push_back(msg);
        not_empty_cv_.notify_one();
        return true;
    }

    bool receive(T& out, unsigned int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!not_empty_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]{ return !queue_.empty(); })) {
            return false;
        }
        out = queue_.front();
        queue_.pop_front();
        if (capacity_ > 0) {
            not_full_cv_.notify_one();
        }
        return true;
    }

private:
    std::deque<T> queue_;
    std::mutex mutex_;
    std::condition_variable not_empty_cv_;
    std::condition_variable not_full_cv_;
    size_t capacity_;
};

} // namespace Platform
} // namespace Tether
