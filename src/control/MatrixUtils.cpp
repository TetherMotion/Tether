/**
 * @file MatrixUtils.cpp
 * @brief Matrix helper functions for state-space controllers
 * 
 * Split from StateSpaceControllers.cpp for maintainability.
 */

#include "control/StateSpaceControllers.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace Control {

// ============================================================================
// Matrix Helper Functions
// ============================================================================

// Matrix multiply: C = A * B (row-major)
void matMul(const double* A, const double* B, double* C, 
            int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            double sum = 0;
            for (int k = 0; k < n; k++) {
                sum += A[i * n + k] * B[k * p + j];
            }
            C[i * p + j] = sum;
        }
    }
}

// Matrix transpose: B = A' where A is m×n, B is n×m
void matTranspose(const double* A, double* B, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            B[j * m + i] = A[i * n + j];
        }
    }
}

// Matrix add: C = A + B
void matAdd(const double* A, const double* B, double* C, int m, int n) {
    for (int i = 0; i < m * n; i++) {
        C[i] = A[i] + B[i];
    }
}

// Matrix subtract: C = A - B
void matSub(const double* A, const double* B, double* C, int m, int n) {
    for (int i = 0; i < m * n; i++) {
        C[i] = A[i] - B[i];
    }
}

// Matrix scale: B = alpha * A
void matScale(const double* A, double alpha, double* B, int m, int n) {
    for (int i = 0; i < m * n; i++) {
        B[i] = alpha * A[i];
    }
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
    double sum = 0;
    for (int i = 0; i < m * n; i++) {
        sum += A[i] * A[i];
    }
    return std::sqrt(sum);
}

// Solve linear system Ax = b using Gaussian elimination with partial pivoting
bool solveLinear(double* A, double* b, int n) {
    // Forward elimination with partial pivoting
    for (int k = 0; k < n; k++) {
        // Find pivot
        int maxRow = k;
        double maxVal = std::abs(A[k * n + k]);
        for (int i = k + 1; i < n; i++) {
            double val = std::abs(A[i * n + k]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = i;
            }
        }
        
        if (maxVal < 1e-12) return false;
        
        // Swap rows
        if (maxRow != k) {
            for (int j = k; j < n; j++) {
                std::swap(A[k * n + j], A[maxRow * n + j]);
            }
            std::swap(b[k], b[maxRow]);
        }
        
        // Eliminate
        for (int i = k + 1; i < n; i++) {
            double factor = A[i * n + k] / A[k * n + k];
            for (int j = k; j < n; j++) {
                A[i * n + j] -= factor * A[k * n + j];
            }
            b[i] -= factor * b[k];
        }
    }
    
    // Back substitution
    for (int i = n - 1; i >= 0; i--) {
        for (int j = i + 1; j < n; j++) {
            b[i] -= A[i * n + j] * b[j];
        }
        double diag = A[i * n + i];
        if (std::abs(diag) < 1e-12) return false;
        b[i] /= diag;
    }
    
    return true;
}

// Invert matrix (small matrices only)
bool matInverse(const double* A, double* Ainv, int n) {
    // Create augmented matrix [A | I]
    std::vector<double> aug(n * 2 * n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * 2 * n + j] = A[i * n + j];
            aug[i * 2 * n + n + j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // Gauss-Jordan elimination
    for (int k = 0; k < n; k++) {
        // Find pivot
        int maxRow = k;
        double maxVal = std::abs(aug[k * 2 * n + k]);
        for (int i = k + 1; i < n; i++) {
            double val = std::abs(aug[i * 2 * n + k]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = i;
            }
        }
        
        if (maxVal < 1e-12) return false;
        
        // Swap rows
        if (maxRow != k) {
            for (int j = 0; j < 2 * n; j++) {
                std::swap(aug[k * 2 * n + j], aug[maxRow * 2 * n + j]);
            }
        }
        
        // Scale pivot row
        double pivot = aug[k * 2 * n + k];
        for (int j = 0; j < 2 * n; j++) {
            aug[k * 2 * n + j] /= pivot;
        }
        
        // Eliminate column
        for (int i = 0; i < n; i++) {
            if (i != k) {
                double factor = aug[i * 2 * n + k];
                for (int j = 0; j < 2 * n; j++) {
                    aug[i * 2 * n + j] -= factor * aug[k * 2 * n + j];
                }
            }
        }
    }
    
    // Extract inverse
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Ainv[i * n + j] = aug[i * 2 * n + n + j];
        }
    }
    
    return true;
}

} // namespace Control
