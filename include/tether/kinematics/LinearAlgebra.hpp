/**
 * @file LinearAlgebra.hpp
 * @brief Small reusable linear algebra utilities for kinematics/dynamics
 * 
 * Reimplemented using Eigen for robust linear solves.
 */

#pragma once

#include <cmath>
#include <algorithm>

#include <Eigen/Dense>

namespace tether::kinematics {

/// @brief Solve a 3x3 linear system Ax = b.
/// @param A 3x3 coefficient matrix (row-major C array).
/// @param b Right-hand side vector (length 3).
/// @param x Output solution vector (length 3). Zeroed if A is singular.
inline void solve3x3(float A[3][3], float b[3], float x[3]) {
    // Map the row-major C arrays to Eigen fixed-size matrices
    // float[3][3] is row-major; Eigen::Matrix3f is column-major by default,
    // so we must specify Eigen::RowMajor in the Map
    using RowMajorMatrix3f = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;
    Eigen::Map<RowMajorMatrix3f> eigenA(&A[0][0]);
    Eigen::Map<Eigen::Vector3f> eigenB(b);
    Eigen::Map<Eigen::Vector3f> eigenX(x);

    if (std::abs(eigenA.determinant()) < 1e-10f) {
        eigenX.setZero();
        return;
    }

    eigenX = eigenA.partialPivLu().solve(eigenB);
}

} // namespace tether::kinematics
