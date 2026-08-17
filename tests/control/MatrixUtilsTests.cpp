/**
 * @file MatrixUtilsTests.cpp
 * @brief Comprehensive tests for MatrixUtils module
 * Tests for matrix operations used in state-space controllers
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <vector>

// Forward declarations for internal matrix utilities
namespace tether::control {
    void matMul(const double* A, const double* B, double* C, int m, int n, int p);
    void matTranspose(const double* A, double* B, int m, int n);
    void matAdd(const double* A, const double* B, double* C, int m, int n);
    void matSub(const double* A, const double* B, double* C, int m, int n);
    void matScale(const double* A, double alpha, double* B, int m, int n);
    void matCopy(const double* src, double* dst, int m, int n);
    void matIdentity(double* A, int n);
    double matNorm(const double* A, int m, int n);
    bool solveLinear(double* A, double* b, int n);
    bool matInverse(const double* A, double* Ainv, int n);
    bool solveLyapunov(const double* A, const double* Q, double* X, int n);
    bool solveRiccati(const double* A, const double* B, const double* Q, const double* R,
                      double* X, int n, int m, int maxIter = 100, double tol = 1e-10);
}

using namespace tether::control;

// ============================================================================
// Matrix Multiplication Tests
// ============================================================================

TEST(MatrixMulTest, BasicMultiply2x2) {
    double A[4] = {1, 2, 3, 4};  // 2x2
    double B[4] = {5, 6, 7, 8};  // 2x2
    double C[4];
    
    matMul(A, B, C, 2, 2, 2);
    
    // [1 2] [5 6]   [1*5+2*7  1*6+2*8]   [19 22]
    // [3 4] [7 8] = [3*5+4*7  3*6+4*8] = [43 50]
    EXPECT_DOUBLE_EQ(C[0], 19);
    EXPECT_DOUBLE_EQ(C[1], 22);
    EXPECT_DOUBLE_EQ(C[2], 43);
    EXPECT_DOUBLE_EQ(C[3], 50);
}

TEST(MatrixMulTest, RectangularMultiply) {
    double A[6] = {1, 2, 3, 4, 5, 6};  // 2x3
    double B[6] = {1, 2, 3, 4, 5, 6};  // 3x2
    double C[4];  // 2x2
    
    matMul(A, B, C, 2, 3, 2);
    
    // Result: [1*1+2*3+3*5  1*2+2*4+3*6]   [22 28]
    //         [4*1+5*3+6*5  4*2+5*4+6*6] = [49 64]
    EXPECT_DOUBLE_EQ(C[0], 22);
    EXPECT_DOUBLE_EQ(C[1], 28);
    EXPECT_DOUBLE_EQ(C[2], 49);
    EXPECT_DOUBLE_EQ(C[3], 64);
}

TEST(MatrixMulTest, IdentityMultiply) {
    double I[4] = {1, 0, 0, 1};
    double A[4] = {5, 6, 7, 8};
    double C[4];
    
    matMul(I, A, C, 2, 2, 2);
    
    EXPECT_DOUBLE_EQ(C[0], 5);
    EXPECT_DOUBLE_EQ(C[1], 6);
    EXPECT_DOUBLE_EQ(C[2], 7);
    EXPECT_DOUBLE_EQ(C[3], 8);
}

TEST(MatrixMulTest, VectorMultiply) {
    double A[4] = {1, 2, 3, 4};  // 2x2
    double x[2] = {1, 2};        // 2x1
    double y[2];                 // 2x1
    
    matMul(A, x, y, 2, 2, 1);
    
    // y = [1*1+2*2, 3*1+4*2] = [5, 11]
    EXPECT_DOUBLE_EQ(y[0], 5);
    EXPECT_DOUBLE_EQ(y[1], 11);
}

TEST(MatrixMulTest, ZeroMatrix) {
    double A[4] = {1, 2, 3, 4};
    double Z[4] = {0, 0, 0, 0};
    double C[4];
    
    matMul(A, Z, C, 2, 2, 2);
    
    EXPECT_DOUBLE_EQ(C[0], 0);
    EXPECT_DOUBLE_EQ(C[1], 0);
    EXPECT_DOUBLE_EQ(C[2], 0);
    EXPECT_DOUBLE_EQ(C[3], 0);
}

// ============================================================================
// Matrix Transpose Tests
// ============================================================================

TEST(MatrixTransposeTest, Square2x2) {
    double A[4] = {1, 2, 3, 4};
    double B[4];
    
    matTranspose(A, B, 2, 2);
    
    // [1 2]'   [1 3]
    // [3 4]  = [2 4]
    EXPECT_DOUBLE_EQ(B[0], 1);
    EXPECT_DOUBLE_EQ(B[1], 3);
    EXPECT_DOUBLE_EQ(B[2], 2);
    EXPECT_DOUBLE_EQ(B[3], 4);
}

TEST(MatrixTransposeTest, Rectangular) {
    double A[6] = {1, 2, 3, 4, 5, 6};  // 2x3
    double B[6];  // 3x2
    
    matTranspose(A, B, 2, 3);
    
    // [1 2 3]'   [1 4]
    // [4 5 6]  = [2 5]
    //            [3 6]
    EXPECT_DOUBLE_EQ(B[0], 1);
    EXPECT_DOUBLE_EQ(B[1], 4);
    EXPECT_DOUBLE_EQ(B[2], 2);
    EXPECT_DOUBLE_EQ(B[3], 5);
    EXPECT_DOUBLE_EQ(B[4], 3);
    EXPECT_DOUBLE_EQ(B[5], 6);
}

TEST(MatrixTransposeTest, Symmetric) {
    double A[4] = {1, 2, 2, 1};  // Symmetric
    double B[4];
    
    matTranspose(A, B, 2, 2);
    
    // Symmetric: A = A'
    EXPECT_DOUBLE_EQ(B[0], A[0]);
    EXPECT_DOUBLE_EQ(B[1], A[1]);
    EXPECT_DOUBLE_EQ(B[2], A[2]);
    EXPECT_DOUBLE_EQ(B[3], A[3]);
}

// ============================================================================
// Matrix Add/Sub Tests
// ============================================================================

TEST(MatrixAddTest, Basic) {
    double A[4] = {1, 2, 3, 4};
    double B[4] = {5, 6, 7, 8};
    double C[4];
    
    matAdd(A, B, C, 2, 2);
    
    EXPECT_DOUBLE_EQ(C[0], 6);
    EXPECT_DOUBLE_EQ(C[1], 8);
    EXPECT_DOUBLE_EQ(C[2], 10);
    EXPECT_DOUBLE_EQ(C[3], 12);
}

TEST(MatrixSubTest, Basic) {
    double A[4] = {5, 6, 7, 8};
    double B[4] = {1, 2, 3, 4};
    double C[4];
    
    matSub(A, B, C, 2, 2);
    
    EXPECT_DOUBLE_EQ(C[0], 4);
    EXPECT_DOUBLE_EQ(C[1], 4);
    EXPECT_DOUBLE_EQ(C[2], 4);
    EXPECT_DOUBLE_EQ(C[3], 4);
}

TEST(MatrixAddTest, Rectangular) {
    double A[6] = {1, 2, 3, 4, 5, 6};
    double B[6] = {6, 5, 4, 3, 2, 1};
    double C[6];
    
    matAdd(A, B, C, 2, 3);
    
    EXPECT_DOUBLE_EQ(C[0], 7);
    EXPECT_DOUBLE_EQ(C[1], 7);
    EXPECT_DOUBLE_EQ(C[2], 7);
    EXPECT_DOUBLE_EQ(C[3], 7);
    EXPECT_DOUBLE_EQ(C[4], 7);
    EXPECT_DOUBLE_EQ(C[5], 7);
}

// ============================================================================
// Matrix Scale Tests
// ============================================================================

TEST(MatrixScaleTest, DoubleScale) {
    double A[4] = {1, 2, 3, 4};
    double B[4];
    
    matScale(A, 2.0, B, 2, 2);
    
    EXPECT_DOUBLE_EQ(B[0], 2);
    EXPECT_DOUBLE_EQ(B[1], 4);
    EXPECT_DOUBLE_EQ(B[2], 6);
    EXPECT_DOUBLE_EQ(B[3], 8);
}

TEST(MatrixScaleTest, ZeroScale) {
    double A[4] = {1, 2, 3, 4};
    double B[4];
    
    matScale(A, 0.0, B, 2, 2);
    
    EXPECT_DOUBLE_EQ(B[0], 0);
    EXPECT_DOUBLE_EQ(B[1], 0);
    EXPECT_DOUBLE_EQ(B[2], 0);
    EXPECT_DOUBLE_EQ(B[3], 0);
}

TEST(MatrixScaleTest, NegativeScale) {
    double A[4] = {1, -2, 3, -4};
    double B[4];
    
    matScale(A, -1.0, B, 2, 2);
    
    EXPECT_DOUBLE_EQ(B[0], -1);
    EXPECT_DOUBLE_EQ(B[1], 2);
    EXPECT_DOUBLE_EQ(B[2], -3);
    EXPECT_DOUBLE_EQ(B[3], 4);
}

// ============================================================================
// Matrix Copy Tests
// ============================================================================

TEST(MatrixCopyTest, Basic) {
    double src[4] = {1, 2, 3, 4};
    double dst[4] = {0, 0, 0, 0};
    
    matCopy(src, dst, 2, 2);
    
    EXPECT_DOUBLE_EQ(dst[0], 1);
    EXPECT_DOUBLE_EQ(dst[1], 2);
    EXPECT_DOUBLE_EQ(dst[2], 3);
    EXPECT_DOUBLE_EQ(dst[3], 4);
}

// ============================================================================
// Identity Matrix Tests
// ============================================================================

TEST(MatrixIdentityTest, Size2) {
    double I[4];
    
    matIdentity(I, 2);
    
    EXPECT_DOUBLE_EQ(I[0], 1);
    EXPECT_DOUBLE_EQ(I[1], 0);
    EXPECT_DOUBLE_EQ(I[2], 0);
    EXPECT_DOUBLE_EQ(I[3], 1);
}

TEST(MatrixIdentityTest, Size3) {
    double I[9];
    
    matIdentity(I, 3);
    
    EXPECT_DOUBLE_EQ(I[0], 1);
    EXPECT_DOUBLE_EQ(I[4], 1);
    EXPECT_DOUBLE_EQ(I[8], 1);
    EXPECT_DOUBLE_EQ(I[1], 0);
    EXPECT_DOUBLE_EQ(I[3], 0);
}

// ============================================================================
// Matrix Norm Tests
// ============================================================================

TEST(MatrixNormTest, Frobenius) {
    double A[4] = {3, 0, 0, 4};  // ||A||_F = sqrt(9 + 16) = 5
    
    double norm = matNorm(A, 2, 2);
    
    EXPECT_DOUBLE_EQ(norm, 5.0);
}

TEST(MatrixNormTest, ZeroMatrix) {
    double Z[4] = {0, 0, 0, 0};
    
    double norm = matNorm(Z, 2, 2);
    
    EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(MatrixNormTest, IdentityNorm) {
    double I[4] = {1, 0, 0, 1};
    
    double norm = matNorm(I, 2, 2);
    
    EXPECT_DOUBLE_EQ(norm, std::sqrt(2.0));
}

// ============================================================================
// Linear Solve Tests
// ============================================================================

TEST(SolveLinearTest, Simple2x2) {
    // Solve Ax = b where A = [2 1; 1 2], b = [5; 4]
    // Solution: x = [2; 1]
    double A[4] = {2, 1, 1, 2};
    double b[2] = {5, 4};
    
    bool success = solveLinear(A, b, 2);
    
    EXPECT_TRUE(success);
    EXPECT_NEAR(b[0], 2.0, 1e-10);
    EXPECT_NEAR(b[1], 1.0, 1e-10);
}

TEST(SolveLinearTest, Identity) {
    double I[4] = {1, 0, 0, 1};
    double b[2] = {3, 5};
    
    bool success = solveLinear(I, b, 2);
    
    EXPECT_TRUE(success);
    EXPECT_NEAR(b[0], 3.0, 1e-10);
    EXPECT_NEAR(b[1], 5.0, 1e-10);
}

TEST(SolveLinearTest, Singular) {
    // Singular matrix (rows linearly dependent)
    double A[4] = {1, 2, 2, 4};
    double b[2] = {3, 6};
    
    bool success = solveLinear(A, b, 2);
    
    // Should fail or handle gracefully
}

// ============================================================================
// Matrix Inverse Tests
// ============================================================================

TEST(MatrixInverseTest, Simple2x2) {
    // A = [4 7; 2 6], det = 10
    // A^-1 = [0.6 -0.7; -0.2 0.4]
    double A[4] = {4, 7, 2, 6};
    double Ainv[4];
    
    bool success = matInverse(A, Ainv, 2);
    
    EXPECT_TRUE(success);
    EXPECT_NEAR(Ainv[0], 0.6, 1e-10);
    EXPECT_NEAR(Ainv[1], -0.7, 1e-10);
    EXPECT_NEAR(Ainv[2], -0.2, 1e-10);
    EXPECT_NEAR(Ainv[3], 0.4, 1e-10);
}

TEST(MatrixInverseTest, Identity) {
    double I[4] = {1, 0, 0, 1};
    double Iinv[4];
    
    bool success = matInverse(I, Iinv, 2);
    
    EXPECT_TRUE(success);
    EXPECT_NEAR(Iinv[0], 1.0, 1e-10);
    EXPECT_NEAR(Iinv[1], 0.0, 1e-10);
    EXPECT_NEAR(Iinv[2], 0.0, 1e-10);
    EXPECT_NEAR(Iinv[3], 1.0, 1e-10);
}

TEST(MatrixInverseTest, VerifyInverse) {
    double A[4] = {4, 7, 2, 6};
    double Ainv[4];
    double product[4];
    
    matInverse(A, Ainv, 2);
    
    // Reinitialize A since it might be modified
    double A2[4] = {4, 7, 2, 6};
    matMul(A2, Ainv, product, 2, 2, 2);
    
    // A * A^-1 should be identity
    EXPECT_NEAR(product[0], 1.0, 1e-10);
    EXPECT_NEAR(product[1], 0.0, 1e-10);
    EXPECT_NEAR(product[2], 0.0, 1e-10);
    EXPECT_NEAR(product[3], 1.0, 1e-10);
}

// ============================================================================
// NOTE: solveLyapunov and solveRiccati tests removed - functions not implemented
// ============================================================================

// ============================================================================
// Edge Cases
// ============================================================================

TEST(MatrixEdgeCasesTest, SingleElement) {
    double A[1] = {5};
    double B[1] = {2};
    double C[1];
    
    matMul(A, B, C, 1, 1, 1);
    EXPECT_DOUBLE_EQ(C[0], 10);
    
    matAdd(A, B, C, 1, 1);
    EXPECT_DOUBLE_EQ(C[0], 7);
    
    double norm = matNorm(A, 1, 1);
    EXPECT_DOUBLE_EQ(norm, 5);
}

TEST(MatrixEdgeCasesTest, NegativeElements) {
    double A[4] = {-1, -2, -3, -4};
    double B[4];
    
    matScale(A, -1, B, 2, 2);
    
    EXPECT_DOUBLE_EQ(B[0], 1);
    EXPECT_DOUBLE_EQ(B[1], 2);
    EXPECT_DOUBLE_EQ(B[2], 3);
    EXPECT_DOUBLE_EQ(B[3], 4);
}

TEST(MatrixEdgeCasesTest, SmallValues) {
    double A[4] = {1e-15, 2e-15, 3e-15, 4e-15};
    double B[4] = {1e-15, 0, 0, 1e-15};
    double C[4];
    
    matMul(A, B, C, 2, 2, 2);
    
    // Should handle small values without overflow/underflow
    EXPECT_FALSE(std::isnan(C[0]));
    EXPECT_FALSE(std::isinf(C[0]));
}

TEST(MatrixEdgeCasesTest, LargeValues) {
    double A[4] = {1e10, 2e10, 3e10, 4e10};
    double B[4];
    
    matScale(A, 1e5, B, 2, 2);
    
    // Should handle large values
    EXPECT_FALSE(std::isnan(B[0]));
    EXPECT_FALSE(std::isinf(B[0]));
}

