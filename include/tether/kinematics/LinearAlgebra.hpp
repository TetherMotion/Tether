/**
 * @file LinearAlgebra.hpp
 * @brief Small reusable linear algebra utilities for kinematics/dynamics
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace tether::kinematics {

/// @brief Solve a 3x3 linear system Ax = b via Gaussian elimination with partial pivoting.
/// @param A 3x3 coefficient matrix (row-major).
/// @param b Right-hand side vector (length 3).
/// @param x Output solution vector (length 3). Zeroed if A is singular.
inline void solve3x3(float A[3][3], float b[3], float x[3]) {
    float aug[3][4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            aug[i][j] = A[i][j];
        }
        aug[i][3] = b[i];
    }

    // Forward elimination
    for (int k = 0; k < 3; ++k) {
        // Find pivot
        int maxRow = k;
        for (int i = k + 1; i < 3; ++i) {
            if (std::abs(aug[i][k]) > std::abs(aug[maxRow][k])) {
                maxRow = i;
            }
        }
        std::swap(aug[k], aug[maxRow]);

        if (std::abs(aug[k][k]) < 1e-10f) {
            x[0] = x[1] = x[2] = 0;
            return;
        }

        for (int i = k + 1; i < 3; ++i) {
            float factor = aug[i][k] / aug[k][k];
            for (int j = k; j < 4; ++j) {
                aug[i][j] -= factor * aug[k][j];
            }
        }
    }

    // Back substitution
    for (int i = 2; i >= 0; --i) {
        x[i] = aug[i][3];
        for (int j = i + 1; j < 3; ++j) {
            x[i] -= aug[i][j] * x[j];
        }
        x[i] /= aug[i][i];
    }
}

} // namespace tether::kinematics
