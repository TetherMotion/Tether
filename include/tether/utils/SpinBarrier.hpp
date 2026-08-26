#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

namespace EtherCAT {
namespace Utils {

/// Spin barrier for N-thread synchronization.
///
/// Uses a generation counter so the barrier can be reused across cycles
/// without reset races. No mutex, no priority inversion.
class SpinBarrier {
public:
    explicit SpinBarrier(int n) : target_(n) {}

    void arrive_and_wait() {
        const uint64_t gen = generation_.load(std::memory_order_acquire);
        const int old = arrived_.fetch_add(1, std::memory_order_acq_rel);
        if (old + 1 == target_) {
            arrived_.store(0, std::memory_order_relaxed);
            generation_.store(gen + 1, std::memory_order_release);
        } else {
            while (generation_.load(std::memory_order_acquire) == gen) {
                std::this_thread::yield();
            }
        }
    }

private:
    int target_;
    std::atomic<int> arrived_{0};
    std::atomic<uint64_t> generation_{0};
};

} // namespace Utils
} // namespace EtherCAT
