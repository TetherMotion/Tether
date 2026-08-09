/**
 * @file MatrixUtils.cpp
 * @brief Matrix helper functions for state-space controllers
 * 
 * Split from StateSpaceControllers.cpp for maintainability.
 * 
 * Reimplemented using Eigen::Map to wrap the raw double* buffers,
 * providing optimized vectorized operations while maintaining the
 * same C API (row-major flat arrays).
 */

#include "control/StateSpaceControllers.hpp"
#include <cmath>
#include <cstring>

#include <Eigen/Dense>

namespace Control {

// ============================================================================
// Matrix Helper Functions (Eigen-backed)
// ============================================================================
//
// All functions accept row-major flat arrays (double*) with explicit
// dimensions. We wrap these with Eigen::Map<RowMajor> to leverage
// Eigen's optimized kernels.

// Matrix multiply: C = A * B (row-major)
void matMul(const double* A, const double* B, double* C, 
            int m, int n, int p) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, m, n);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenB(B, n, p);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenC(C, m, p);
    eigenC = eigenA * eigenB;
}

// Matrix transpose: B = A' where A is m×n, B is n×m
void matTranspose(const double* A, double* B, int m, int n) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, m, n);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenB(B, n, m);
    eigenB = eigenA.transpose();
}

// Matrix add: C = A + B
void matAdd(const double* A, const double* B, double* C, int m, int n) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, m, n);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenB(B, m, n);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenC(C, m, n);
    eigenC = eigenA + eigenB;
}

// Matrix subtract: C = A - B
void matSub(const double* A, const double* B, double* C, int m, int n) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, m, n);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenB(B, m, n);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenC(C, m, n);
    eigenC = eigenA - eigenB;
}

// Matrix scale: B = alpha * A
void matScale(const double* A, double alpha, double* B, int m, int n) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, m, n);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenB(B, m, n);
    eigenB = alpha * eigenA;
}

// Matrix copy
void matCopy(const double* src, double* dst, int m, int n) {
    std::memcpy(dst, src, m * n * sizeof(double));
}

// Identity matrix
void matIdentity(double* A, int n) {
    std::memset(A, 0, n * n * sizeof(double));
    for (int i = 0; i < n; i++) {
        A[i * n + i] = 1.0;
    }
}

// Matrix Frobenius norm
double matNorm(const double* A, int m, int n) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, m, n);
    return eigenA.norm();
}

// Solve linear system Ax = b using LU decomposition with partial pivoting
// b is overwritten with solution x. A is not modified (unlike the original
// Gaussian elimination which destroyed A in-place).
bool solveLinear(double* A, double* b, int n) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, n, n);
    Eigen::Map<Eigen::VectorXd> eigenB(b, n);

    Eigen::PartialPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> lu(eigenA);

    if (std::abs(lu.determinant()) < 1e-12) {
        return false;
    }

    eigenB = lu.solve(eigenB);
    return true;
}

// Invert matrix (small matrices only)
bool matInverse(const double* A, double* Ainv, int n) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenA(A, n, n);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        eigenAinv(Ainv, n, n);

    // Use LU decomposition with partial pivoting for inversion
    Eigen::PartialPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> lu(eigenA);
    
    if (lu.determinant() == 0.0) {
        return false;
    }
    
    // Compute inverse by solving A * Ainv = I
    eigenAinv = lu.inverse();
    
    return true;
}

} // namespace Control
