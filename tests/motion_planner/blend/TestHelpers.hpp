/**
 * @file TestHelpers.hpp
 * @brief Shared helpers for the blend test suite.
 */

#pragma once

#include "tether/motion_planner/geometry/Vector.hpp"

#include <gtest/gtest.h>

namespace tether::motion::testing {

inline void expectVecNear(const RVec& a, const RVec& b, double tol) {
    ASSERT_EQ(a.dim(), b.dim());
    for (std::size_t i = 0; i < a.dim(); ++i) {
        EXPECT_NEAR(a[i], b[i], tol) << "component " << i;
    }
}

} // namespace tether::motion::testing
