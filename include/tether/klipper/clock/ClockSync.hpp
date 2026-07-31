/**
 * @file ClockSync.hpp
 * @brief Host-side clock synchronisation via decaying linear regression.
 *
 * @details
 * The host periodically sends `get_clock` commands to the device, which
 * responds with the current 32-bit MCU clock reading. The host records
 * (sendTime, recvTime, mcuClock) samples and fits a linear regression of
 * mcuClock vs host-time, with exponential decay weighting so recent samples
 * dominate. This yields a mapping from host time to MCU clock time, allowing
 * the host to schedule future steps in MCU clock units.
 *
 * The regression model: mcuClock ≈ a * hostTime + b
 * Weighted least squares with weight w_i = exp(-lambda * (t_now - t_i)).
 */

#pragma once

#include "tether/klipper/clock/McuClock.hpp"

#include <cstdint>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace tether::klipper::clock {

using HostClock = std::chrono::steady_clock;
using HostTime = HostClock::time_point;

/// @brief A clock-sync sample: (hostSendTime, hostRecvTime, mcuClock32).
struct ClockSample {
    HostTime sendTime;
    HostTime recvTime;
    uint32_t mcuClock = 0;
};

/**
 * @brief Host-side clock synchronisation using decaying linear regression.
 */
class ClockSync {
public:
    /**
     * @brief Construct with a decay constant and max sample count.
     * @param decayLambda Exponential decay rate (1/seconds). Higher = faster
     *                    adaptation. 0.1 means ~10s time constant.
     * @param maxSamples Maximum samples retained.
     */
    explicit ClockSync(double decayLambda = 0.1, size_t maxSamples = 64)
        : decayLambda_(decayLambda), maxSamples_(maxSamples) {}

    /**
     * @brief Add a clock sample from a `get_clock` exchange.
     * @param sendTime  Host time when the `get_clock` command was sent.
     * @param recvTime  Host time when the response was received.
     * @param mcuClock  32-bit MCU clock value from the response.
     */
    void addSample(HostTime sendTime, HostTime recvTime, uint32_t mcuClock) {
        ClockSample s{sendTime, recvTime, mcuClock};
        samples_.push_back(s);
        if (samples_.size() > maxSamples_) {
            samples_.erase(samples_.begin());
        }
        updateFit();
    }

    /// @return True if the fit has enough samples to be usable.
    bool isSynchronised() const { return samples_.size() >= 2 && slope_ != 0.0; }

    /// @return The estimated slope (MCU ticks per host second).
    double slope() const { return slope_; }

    /// @return The estimated offset (MCU ticks at host time zero).
    double offset() const { return offset_; }

    /**
     * @brief Convert a host time to the estimated MCU clock value (32-bit).
     * @param hostTime The host time to convert.
     * @return Estimated 32-bit MCU clock value.
     */
    uint32_t hostToMcu(HostTime hostTime) const {
        double hostSec = std::chrono::duration<double>(
            hostTime - HostTime::min()).count();
        // Use relative host time from the first sample for numerical stability.
        if (samples_.empty()) return 0;
        double relSec = std::chrono::duration<double>(
            hostTime - samples_[0].recvTime).count();
        double mcu = slope_ * relSec + offset_;
        return static_cast<uint32_t>(mcu);
    }

    /**
     * @brief Convert a host time duration from now to MCU clock ticks.
     * @param delay Host-time delay from now.
     * @return Estimated MCU clock ticks for that delay.
     */
    uint32_t hostDelayToMcuTicks(std::chrono::nanoseconds delay) const {
        double sec = std::chrono::duration<double>(delay).count();
        return static_cast<uint32_t>(sec * slope_);
    }

    /// @brief Reset all samples and the fit.
    void reset() { samples_.clear(); slope_ = 0; offset_ = 0; }

    /// @return The number of samples retained.
    size_t sampleCount() const { return samples_.size(); }

private:
    void updateFit() {
        if (samples_.size() < 2) return;
        // Use the midpoint of send/recv as the host timestamp for each sample.
        HostTime t0 = samples_[0].recvTime;
        double sumW = 0, sumWx = 0, sumWy = 0, sumWxx = 0, sumWxy = 0;
        HostTime now = HostClock::now();
        for (const auto& s : samples_) {
            double dt = std::chrono::duration<double>(now - s.recvTime).count();
            double w = std::exp(-decayLambda_ * dt);
            double x = std::chrono::duration<double>(s.recvTime - t0).count();
            double y = static_cast<double>(s.mcuClock);
            sumW += w;
            sumWx += w * x;
            sumWy += w * y;
            sumWxx += w * x * x;
            sumWxy += w * x * y;
        }
        double denom = sumW * sumWxx - sumWx * sumWx;
        if (std::abs(denom) < 1e-12) return;
        slope_ = (sumW * sumWxy - sumWx * sumWy) / denom;
        offset_ = (sumWy - slope_ * sumWx) / sumW;
    }

    double decayLambda_;
    size_t maxSamples_;
    std::vector<ClockSample> samples_;
    double slope_ = 0;
    double offset_ = 0;
};

} // namespace tether::klipper::clock
