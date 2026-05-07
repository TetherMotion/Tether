#pragma once

#include <cstddef>
#include <vector>

namespace Identification {

using Vector = std::vector<double>;
using Matrix = std::vector<Vector>;

struct EigenDecomposition {
    Vector values;
    Matrix vectors;
};

Matrix makeMatrix(size_t rows, size_t cols, double value = 0.0);
Matrix identityMatrix(size_t size);
Matrix transpose(const Matrix& matrix);
Matrix multiply(const Matrix& lhs, const Matrix& rhs);
Vector multiply(const Matrix& lhs, const Vector& rhs);
Matrix add(const Matrix& lhs, const Matrix& rhs);
Matrix subtract(const Matrix& lhs, const Matrix& rhs);
Matrix outerProduct(const Vector& lhs, const Vector& rhs);
Vector solveLinearSystem(Matrix A, Vector b, double regularization = 0.0);
Vector solveLeastSquares(const Matrix& A, const Vector& b, double regularization = 1e-8);
Matrix pseudoInverse(const Matrix& A, double regularization = 1e-8);
EigenDecomposition jacobiEigenDecomposition(const Matrix& symmetric,
                                            size_t max_iterations = 128,
                                            double tolerance = 1e-10);
double dot(const Vector& lhs, const Vector& rhs);
double norm(const Vector& vector);
double frobeniusNorm(const Matrix& matrix);
Vector column(const Matrix& matrix, size_t index);
void appendRow(Matrix& matrix, const Vector& row);
double conditionNumber(const Matrix& matrix, double regularization = 1e-8);

} // namespace Identification