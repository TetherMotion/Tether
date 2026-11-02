#pragma once

/**
 * @file CaseLight.hpp
 * @brief Case light controller (M355)
 */

#include <algorithm>

namespace tether::klipper::klippy {

/**@brief Case light controller (M355). */
class CaseLight {
public:
    /**
     * @brief Set case light state.
     * @param on          Power state.
     * @param brightness  0..1 intensity (clamped).
     */
    void setState(bool on, double brightness) {
        on_ = on;
        brightness_ = std::clamp(brightness, 0.0, 1.0);
    }

    /**
     * @brief Check if light is on.
     * @return power state.
     */
    bool isOn() const { return on_; }

    /**
     * @brief Get brightness (0.0 to 1.0).
     * @return intensity.
     */
    double brightness() const { return brightness_; }

private:
    bool on_ = false;
    double brightness_ = 1.0;
};

} // namespace tether::klipper::klippy
