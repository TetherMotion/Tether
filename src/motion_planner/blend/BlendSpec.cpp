/**
 * @file BlendSpec.cpp
 * @brief BlendSpec::validate implementation.
 */

#include "tether/motion_planner/blend/BlendSpec.hpp"

#include <cmath>
#include <stdexcept>

namespace tether::motion {

void BlendSpec::validate() const {
    if (mode == PathMode::Blend && tolerance == 0.0) {
        throw std::invalid_argument(
            "BlendSpec: Blend mode requires a non-zero tolerance");
    }
    if (!(maxBlendFraction > 0.0 && maxBlendFraction <= 1.0)) {
        throw std::invalid_argument(
            "BlendSpec: maxBlendFraction must be in (0, 1]");
    }
    if (!(minSegmentLength > 0.0)) {
        throw std::invalid_argument(
            "BlendSpec: minSegmentLength must be > 0");
    }
    if (!(minAngleRad > 0.0 && minAngleRad < maxAngleRad &&
          maxAngleRad < M_PI)) {
        throw std::invalid_argument(
            "BlendSpec: need 0 < minAngleRad < maxAngleRad < pi");
    }
    for (std::size_t i = 0; i < metric.size(); ++i) {
        if (metric[i] < 0.0) {
            throw std::invalid_argument(
                "BlendSpec: metric entries must be non-negative");
        }
    }
    if (certEpsilon < 0.0) {
        throw std::invalid_argument(
            "BlendSpec: certEpsilon must be >= 0");
    }
}

} // namespace tether::motion
