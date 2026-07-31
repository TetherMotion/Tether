#pragma once

/// @file CaseLight.hpp
/// @brief Case light controller (M355)

#include <algorithm>

namespace tether::klipper::klippy {

/// @brief Case light controller (M355).
class CaseLight {
public:
    /// @brief Set case light state.
    void setState(bool on, double brightness) {
        on_ = on;
        brightness_ = std::clamp(brightness, 0.0, 1.0);
    }

    /// @brief Check if light is on.
    bool isOn() const { return on_; }

    /// @brief Get brightness (0.0 to 1.0).
    double brightness() const { return brightness_; }

private:
    bool on_ = false;
    double brightness_ = 1.0;
};

} // namespace tether::klipper::klippy
