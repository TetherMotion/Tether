#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

#include <tether/identification/DenseLinearAlgebra.hpp>

namespace Identification::detail {

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

inline size_t rowCount(const Matrix& matrix) {
    return matrix.size();
} // GCOVR_EXCL_LINE

inline size_t colCount(const Matrix& matrix) {
    return matrix.empty() ? 0 : matrix.front().size();
}

inline Vector addVector(const Vector& lhs, const Vector& rhs) {
    Vector result(lhs.size(), 0.0);
    for (size_t i = 0; i < lhs.size(); ++i) {
        result[i] = lhs[i] + rhs[i];
    }
    return result; // GCOVR_EXCL_LINE
}

inline Vector subtractVector(const Vector& lhs, const Vector& rhs) {
    Vector result(lhs.size(), 0.0);
    for (size_t i = 0; i < lhs.size(); ++i) {
        result[i] = lhs[i] - rhs[i];
    }
    return result;
}

inline Vector scaleVector(const Vector& vector, double scalar) {
    Vector result(vector.size(), 0.0);
    for (size_t i = 0; i < vector.size(); ++i) {
        result[i] = vector[i] * scalar;
    }
    return result;
}

inline double percentileAbs(const Vector& values, double percentile) {
    if (values.empty()) {
        return 0.0;
    }

    Vector sorted(values.size(), 0.0);
    for (size_t i = 0; i < values.size(); ++i) {
        sorted[i] = std::abs(values[i]);
    }
    std::sort(sorted.begin(), sorted.end());
    const double clamped = std::clamp(percentile, 0.0, 1.0);
    const size_t index = static_cast<size_t>(clamped * static_cast<double>(sorted.size() - 1));
    return sorted[index];
}

inline Matrix multiplyScalar(const Matrix& matrix, double scalar) {
    Matrix result = matrix;
    for (auto& row : result) {
        for (double& value : row) {
            value *= scalar;
        }
    }
    return result;
}

inline Matrix covariance(const Matrix& samples) {
    if (samples.empty()) {
        return {};
    }

    const size_t cols = colCount(samples);
    Vector mean(cols, 0.0);
    for (const auto& row : samples) {
        for (size_t i = 0; i < cols; ++i) {
            mean[i] += row[i];
        }
    }
    for (double& value : mean) {
        value /= static_cast<double>(samples.size());
    }

    Matrix centered = makeMatrix(samples.size(), cols, 0.0);
    for (size_t r = 0; r < samples.size(); ++r) {
        for (size_t c = 0; c < cols; ++c) {
            centered[r][c] = samples[r][c] - mean[c];
        }
    }

    Matrix cov = multiply(transpose(centered), centered);
    const double scale = samples.size() > 1 ? 1.0 / static_cast<double>(samples.size() - 1) : 1.0;
    return multiplyScalar(cov, scale);
}

inline Matrix inverseSqrtSymmetric(const Matrix& matrix) {
    if (matrix.empty()) {
        return {};
    }

    const EigenDecomposition eig = jacobiEigenDecomposition(matrix);
    Matrix result = makeMatrix(rowCount(matrix), colCount(matrix), 0.0);
    for (size_t k = 0; k < eig.values.size(); ++k) {
        const double lambda = std::max(eig.values[k], kEpsilon);
        const double scale = 1.0 / std::sqrt(lambda);
        Vector vec = column(eig.vectors, k);
        Matrix contribution = outerProduct(vec, vec);
        contribution = multiplyScalar(contribution, scale);
        result = add(result, contribution);
    }
    return result;
}

inline Matrix concatHorizontal(const Matrix& lhs, const Matrix& rhs) {
    if (lhs.empty()) {
        return rhs;
    }
    if (rhs.empty()) {
        return lhs;
    }

    Matrix result(lhs.size(), Vector(lhs.front().size() + rhs.front().size(), 0.0));
    for (size_t r = 0; r < lhs.size(); ++r) {
        size_t offset = 0;
        for (double value : lhs[r]) {
            result[r][offset++] = value;
        }
        for (double value : rhs[r]) {
            result[r][offset++] = value;
        }
    }
    return result;
}

inline Matrix leastSquaresMatrix(const Matrix& A, const Matrix& B, double regularization = 1e-8) {
    if (A.empty() || B.empty()) {
        return {};
    }

    const Matrix At = transpose(A);
    Matrix normal = multiply(At, A);
    for (size_t i = 0; i < std::min(rowCount(normal), colCount(normal)); ++i) {
        normal[i][i] += regularization;
    }

    const Matrix rhs = multiply(At, B);
    Matrix result = makeMatrix(rowCount(normal), colCount(rhs), 0.0);
    for (size_t c = 0; c < colCount(rhs); ++c) {
        Vector column_rhs(rowCount(rhs), 0.0);
        for (size_t r = 0; r < rowCount(rhs); ++r) {
            column_rhs[r] = rhs[r][c];
        }
        const Vector solution = solveLinearSystem(normal, column_rhs, 0.0);
        for (size_t r = 0; r < solution.size(); ++r) {
            result[r][c] = solution[r];
        }
    }
    return result;
}

inline Vector extractWindow(const Matrix& samples, size_t start, size_t length) {
    Vector result;
    for (size_t i = 0; i < length; ++i) {
        const auto& row = samples[start + i];
        result.insert(result.end(), row.begin(), row.end());
    }
    return result;
}

inline double computeFitPercent(const Vector& measured, const Vector& predicted) {
    if (measured.empty() || predicted.empty()) {
        return 0.0;
    }

    const double mean = std::accumulate(measured.begin(), measured.end(), 0.0) /
        static_cast<double>(measured.size());
    double err = 0.0;
    double sig = 0.0;
    for (size_t i = 0; i < measured.size() && i < predicted.size(); ++i) {
        const double error = measured[i] - predicted[i];
        err += error * error;
        const double centered = measured[i] - mean;
        sig += centered * centered;
    }

    return sig > kEpsilon ? 100.0 * std::max(0.0, 1.0 - std::sqrt(err / sig)) : 0.0;
}

inline Matrix choleskyLikeSqrt(const Matrix& covariance_matrix, double scale) {
    if (covariance_matrix.empty()) {
        return {};
    }

    const EigenDecomposition eig = jacobiEigenDecomposition(covariance_matrix);
    Matrix result = makeMatrix(rowCount(covariance_matrix), colCount(covariance_matrix), 0.0);
    for (size_t k = 0; k < eig.values.size(); ++k) {
        const double lambda = std::sqrt(std::max(eig.values[k], 0.0)) * scale;
        Vector vec = column(eig.vectors, k);
        Matrix contribution = outerProduct(vec, vec);
        contribution = multiplyScalar(contribution, lambda);
        result = add(result, contribution);
    }
    return result;
}

inline Matrix numericalJacobian(const std::function<Vector(const Vector&)>& function,
                                const Vector& state) {
    const Vector baseline = function(state);
    Matrix jacobian = makeMatrix(baseline.size(), state.size(), 0.0);
    for (size_t i = 0; i < state.size(); ++i) {
        Vector perturbed = state;
        const double delta = 1e-6 * std::max(1.0, std::abs(state[i]));
        perturbed[i] += delta;
        const Vector shifted = function(perturbed);
        for (size_t r = 0; r < baseline.size(); ++r) {
            jacobian[r][i] = (shifted[r] - baseline[r]) / delta;
        }
    }
    return jacobian;
}

inline Matrix subMatrixColumns(const Matrix& matrix, const std::vector<size_t>& columns) {
    Matrix result(matrix.size(), Vector(columns.size(), 0.0));
    for (size_t r = 0; r < matrix.size(); ++r) {
        for (size_t c = 0; c < columns.size(); ++c) {
            result[r][c] = matrix[r][columns[c]];
        }
    }
    return result;
}

} // namespace Identification::detail