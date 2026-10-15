#pragma once

#include <atomic>
#include <chrono>
#include <functional>

#include "tether/platform/Platform.hpp"

namespace Tether {
namespace Platform {

/**
 * @brief Timed, cancellable periodic loop.
 *
 * Calls `body(elapsed_seconds)` immediately, then every `interval_ms`, until
 * `cancel` becomes true or `duration_s` elapses (`duration_s <= 0` runs until
 * cancelled).  Uses Clock::instance().delayMilliseconds() so embedded
 * platforms get their own delay primitive.
 *
 * @return the elapsed seconds at exit.
 */
inline double runCancellableLoop(std::atomic_bool& cancel, double duration_s,
                                 int interval_ms,
                                 const std::function<void(double)>& body) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto elapsed = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };
    while (!cancel.load(std::memory_order_relaxed)) {
        const double t = elapsed();
        if (duration_s > 0.0 && t >= duration_s) break;
        body(t);
        Clock::instance().delayMilliseconds(interval_ms);
    }
    return elapsed();
}

} // namespace Platform
} // namespace Tether
