/**
 * @file MotionProfile.hpp
 * @brief Compatibility shim for the generic tether::common motion profiles
 *
 * @details
 * The profile hierarchy now lives in include/tether/common/MotionProfile.hpp
 * under namespace tether::common. This header re-exports the same names in the
 * CiA402 namespace so existing CiA 402 code continues to compile.
 */

#pragma once

#include "tether/common/MotionProfile.hpp"

namespace CiA402 {

using MotionProfile    = tether::common::MotionProfile;
using LinearProfile    = tether::common::LinearProfile;
using TrapezoidalProfile = tether::common::TrapezoidalProfile;
using TriangularProfile  = tether::common::TriangularProfile;
using SCurveProfile    = tether::common::SCurveProfile;
using PolynomialProfile = tether::common::PolynomialProfile;

using tether::common::createProfile;
using tether::common::selectOptimalProfile;

} // namespace CiA402
