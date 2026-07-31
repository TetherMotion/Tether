/**
 * @file RtoEstimator.hpp
 * @brief Round-trip-time estimator and retransmission-timeout (RTO) computation.
 *
 * @details
 * Implements an RFC 6298-style SRTT/RTTVAR/RTO estimator adapted for the
 * Klipper protocol's millisecond-scale command/response exchange. The RTO is
 * clamped to [kMinRtoSeconds, kMaxRtoSeconds]. The estimator is updated each
 * time a block is acknowledged with a measured RTT sample.
 */

#pragma once

#include "tether/klipper/protocol/Constants.hpp"

#include <cmath>
#include <algorithm>

namespace tether::klipper::reliability {

/**
 * @brief SRTT/RTTVAR/RTO estimator (RFC 6298-style).
 */
class RtoEstimator {
public:
    RtoEstimator() = default;

    /// @return The current retransmission timeout in seconds.
    double rto() const { return rto_; }

    /// @return The current smoothed RTT in seconds.
    double srtt() const { return srtt_; }

    /// @brief Update the estimator with a new RTT sample (seconds).
    void update(double rttSample) {
        if (!initialized_) {
            srtt_ = rttSample;
            rttvar_ = rttSample / 2.0;
            initialized_ = true;
        } else {
            const double alpha = 0.125;
            const double beta = 0.25;
            double delta = rttSample - srtt_;
            rttvar_ = (1.0 - beta) * rttvar_ + beta * std::abs(delta);
            srtt_ = (1.0 - alpha) * srtt_ + alpha * rttSample;
        }
        rto_ = std::clamp(srtt_ + 4.0 * rttvar_,
                          protocol::kMinRtoSeconds, protocol::kMaxRtoSeconds);
    }

    /// @brief Reset the estimator (e.g. after a connection reset).
    void reset() {
        srtt_ = 0;
        rttvar_ = 0;
        rto_ = 1.0;
        initialized_ = false;
    }

private:
    double srtt_ = 0;
    double rttvar_ = 0;
    double rto_ = 1.0;
    bool initialized_ = false;
};

} // namespace tether::klipper::reliability
