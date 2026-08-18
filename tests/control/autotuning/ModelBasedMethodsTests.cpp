#include <gtest/gtest.h>
#include "tether/control/autotuning/model_based/DeadbeatControl.hpp"

// Legacy aggregated model-based tests were split into per-controller tests
// under tests/control/autotuning/model_based/. Keep a minimal placeholder
// here so the build system does not accidentally depend on the original
// monolithic test definitions.

TEST(ModelBasedMethodsMoved, Present) {
    // Ensure one model-based helper is available and callable (deadbeat design)
    // Note: computeGain expects 2x2 matrices (4 elements each for n=2)
    double A[] = {1.0, -0.5, 0.0, 0.0};
    double B[] = {0.0, 0.5, 0.0, 0.0};
    double C[] = {1.0, 0.0, 0.0, 0.0};
    auto coeffs = tether::control::Autotuning::DeadbeatControl::design(A, B, C, 2, 1, 1, 0.1, 1);
    EXPECT_FALSE(coeffs.empty());
}
