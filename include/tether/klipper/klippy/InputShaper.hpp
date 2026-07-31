#pragma once

/// @file InputShaper.hpp
/// @brief Input shaper for vibration compensation

#include <cmath>
#include <utility>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Input shaper type.
enum class ShaperType {
    None,
    ZV,         ///< Zero Vibration
    ZVD,        ///< Zero Vibration Derivative
    MZV,        ///< Modified Zero Vibration
    EI,         ///< Extra Insensitivity
    DampedEI,   ///< Damped Extra Insensitivity
};

/// @brief Input shaper parameters.
struct InputShaperParams {
    ShaperType type = ShaperType::None;
    double freq = 0.0;     ///< Shaping frequency (Hz)
    double damping = 0.0;  ///< Damping ratio (0.0 = no damping)
};

/// @brief Input shaper for vibration compensation.
///
/// Implements input shaping to cancel mechanical resonances.
/// The shaper modifies the acceleration profile of moves to
/// reduce vibrations at specific frequencies.
class InputShaper {
public:
    explicit InputShaper(InputShaperParams params = {})
        : params_(params) {
        computeCoefficients();
    }

    /// @brief Set the shaper parameters and recompute coefficients.
    void setParams(InputShaperParams params) {
        params_ = params;
        computeCoefficients();
    }

    /// @brief Get the current parameters.
    const InputShaperParams& params() const { return params_; }

    /// @brief Check if the shaper is active.
    bool isActive() const { return params_.type != ShaperType::None && params_.freq > 0.0; }

    /// @brief Get the shaping coefficients (A_i, t_i pairs).
    const std::vector<std::pair<double, double>>& coefficients() const {
        return coeffs_;
    }

    /// @brief Apply shaping to an acceleration command.
    /// @param accel Input acceleration (mm/s^2)
    /// @param time Current time (s)
    /// @return Shaped acceleration.
    double shapeAcceleration(double accel, double time) const {
        if (!isActive() || coeffs_.empty()) return accel;
        double result = 0.0;
        for (const auto& [a, t] : coeffs_) {
            double delayedTime = time - t;
            if (delayedTime >= 0.0) {
                result += a * accel;
            }
        }
        return result;
    }

    /// @brief Get the total shaping delay (seconds).
    double shapingDelay() const {
        if (coeffs_.empty()) return 0.0;
        return coeffs_.back().second;
    }

private:
    void computeCoefficients() {
        coeffs_.clear();
        if (params_.type == ShaperType::None || params_.freq <= 0.0) return;

        double f = params_.freq;
        double zeta = params_.damping;
        double omega = 2.0 * M_PI * f;
        double K = std::exp(-zeta * M_PI / std::sqrt(1.0 - zeta * zeta));
        double halfPeriod = M_PI / (omega * std::sqrt(1.0 - zeta * zeta));

        switch (params_.type) {
            case ShaperType::ZV:
                // Two impulses: A1=1/(1+K), A2=K/(1+K)
                coeffs_.push_back({1.0 / (1.0 + K), 0.0});
                coeffs_.push_back({K / (1.0 + K), halfPeriod});
                break;
            case ShaperType::ZVD:
                // Three impulses
                coeffs_.push_back({1.0 / ((1.0 + K) * (1.0 + K)), 0.0});
                coeffs_.push_back({2.0 * K / ((1.0 + K) * (1.0 + K)), halfPeriod});
                coeffs_.push_back({K * K / ((1.0 + K) * (1.0 + K)), 2.0 * halfPeriod});
                break;
            case ShaperType::MZV:
                // Modified ZV - three impulses with different weights
                coeffs_.push_back({0.5 * (1.0 + K * K) / ((1.0 + K) * (1.0 + K)), 0.0});
                coeffs_.push_back({K / ((1.0 + K) * (1.0 + K)), halfPeriod});
                coeffs_.push_back({0.5 * (1.0 + K * K) / ((1.0 + K) * (1.0 + K)), 2.0 * halfPeriod});
                break;
            case ShaperType::EI:
                // Extra insensitivity - 3-impulse with wider band
                coeffs_.push_back({0.25, 0.0});
                coeffs_.push_back({0.5, halfPeriod});
                coeffs_.push_back({0.25, 2.0 * halfPeriod});
                break;
            case ShaperType::DampedEI:
                // Damped EI - similar to EI but with damping
                coeffs_.push_back({0.25 * (1.0 - zeta), 0.0});
                coeffs_.push_back({0.5, halfPeriod});
                coeffs_.push_back({0.25 * (1.0 + zeta), 2.0 * halfPeriod});
                break;
            case ShaperType::None:
                break;
        }
    }

    InputShaperParams params_;
    std::vector<std::pair<double, double>> coeffs_; ///< (A_i, t_i) impulse pairs
};

/// @brief Convert ShaperType to string representation.
inline const char* shaperTypeToString(ShaperType type) {
    switch (type) {
        case ShaperType::None:      return "none";
        case ShaperType::ZV:        return "ZV";
        case ShaperType::ZVD:       return "ZVD";
        case ShaperType::MZV:       return "MZV";
        case ShaperType::EI:        return "EI";
        case ShaperType::DampedEI:  return "DampedEI";
    }
    return "unknown";
}

} // namespace tether::klipper::klippy
