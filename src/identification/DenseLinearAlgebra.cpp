#include <tether/identification/DenseLinearAlgebra.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace Identification {

namespace {

constexpr double kEpsilon = 1e-9;

size_t rowCount(const Matrix& matrix) {
    return matrix.size();
}

size_t colCount(const Matrix& matrix) {
    return matrix.empty() ? 0 : matrix.front().size();
}

} // namespace

Matrix makeMatrix(size_t rows, size_t cols, double value) {
    return Matrix(rows, Vector(cols, value));
}

Matrix identityMatrix(size_t size) {
    Matrix result = makeMatrix(size, size, 0.0);
    for (size_t i = 0; i < size; ++i) {
        result[i][i] = 1.0;
    }
    return result;
}

Matrix transpose(const Matrix& matrix) {
    if (matrix.empty()) {
        return {};
    }

    Matrix result = makeMatrix(colCount(matrix), rowCount(matrix), 0.0);
    for (size_t r = 0; r < rowCount(matrix); ++r) {
        for (size_t c = 0; c < colCount(matrix); ++c) {
            result[c][r] = matrix[r][c];
        }
    }
    return result;
}

Matrix multiply(const Matrix& lhs, const Matrix& rhs) {
    if (lhs.empty() || rhs.empty()) {
        return {};
    }

    Matrix result = makeMatrix(rowCount(lhs), colCount(rhs), 0.0);
    for (size_t r = 0; r < rowCount(lhs); ++r) {
        for (size_t k = 0; k < colCount(lhs); ++k) {
            for (size_t c = 0; c < colCount(rhs); ++c) {
                result[r][c] += lhs[r][k] * rhs[k][c];
            }
        }
    }
    return result;
}

Vector multiply(const Matrix& lhs, const Vector& rhs) {
    Vector result(rowCount(lhs), 0.0);
    for (size_t r = 0; r < rowCount(lhs); ++r) {
        for (size_t c = 0; c < colCount(lhs) && c < rhs.size(); ++c) {
            result[r] += lhs[r][c] * rhs[c];
        }
    }
    return result;
}

Matrix add(const Matrix& lhs, const Matrix& rhs) {
    Matrix result = lhs;
    for (size_t r = 0; r < rowCount(lhs); ++r) {
        for (size_t c = 0; c < colCount(lhs); ++c) {
            result[r][c] += rhs[r][c];
        }
    }
    return result;
}

Matrix subtract(const Matrix& lhs, const Matrix& rhs) {
    Matrix result = lhs;
    for (size_t r = 0; r < rowCount(lhs); ++r) {
        for (size_t c = 0; c < colCount(lhs); ++c) {
            result[r][c] -= rhs[r][c];
        }
    }
    return result;
}

Matrix outerProduct(const Vector& lhs, const Vector& rhs) {
    Matrix result = makeMatrix(lhs.size(), rhs.size(), 0.0);
    for (size_t r = 0; r < lhs.size(); ++r) {
        for (size_t c = 0; c < rhs.size(); ++c) {
            result[r][c] = lhs[r] * rhs[c];
        }
    }
    return result;
}

Vector solveLinearSystem(Matrix A, Vector b, double regularization) {
    if (A.empty()) {
        return {};
    }

    const size_t n = A.size();
    for (size_t i = 0; i < n; ++i) {
        A[i][i] += regularization;
    }

    for (size_t pivot = 0; pivot < n; ++pivot) {
        size_t best_row = pivot;
        double best_value = std::abs(A[pivot][pivot]);
        for (size_t row = pivot + 1; row < n; ++row) {
            const double candidate = std::abs(A[row][pivot]);
            if (candidate > best_value) {
                best_value = candidate;
                best_row = row;
            }
        }

        if (best_value < kEpsilon) {
            return Vector(n, 0.0);
        }
        if (best_row != pivot) {
            std::swap(A[pivot], A[best_row]);
            std::swap(b[pivot], b[best_row]);
        }

        const double diag = A[pivot][pivot];
        for (size_t col = pivot; col < n; ++col) {
            A[pivot][col] /= diag;
        }
        b[pivot] /= diag;

        for (size_t row = 0; row < n; ++row) {
            if (row == pivot) {
                continue;
            }

            const double factor = A[row][pivot];
            for (size_t col = pivot; col < n; ++col) {
                A[row][col] -= factor * A[pivot][col];
            }
            b[row] -= factor * b[pivot];
        }
    }

    return b;
}

Vector solveLeastSquares(const Matrix& A, const Vector& b, double regularization) {
    if (A.empty()) {
        return {};
    }

    const Matrix At = transpose(A);
    Matrix normal = multiply(At, A);
    Vector rhs = multiply(At, b);
    return solveLinearSystem(normal, rhs, regularization);
}

Matrix pseudoInverse(const Matrix& A, double regularization) {
    if (A.empty()) {
        return {};
    }

    if (rowCount(A) >= colCount(A)) {
        Matrix normal = multiply(transpose(A), A);
        for (size_t i = 0; i < std::min(rowCount(normal), colCount(normal)); ++i) {
            normal[i][i] += regularization;
        }

        Matrix result = makeMatrix(colCount(A), rowCount(A), 0.0);
        for (size_t c = 0; c < rowCount(A); ++c) {
            Vector rhs(colCount(A), 0.0);
            for (size_t i = 0; i < colCount(A); ++i) {
                rhs[i] = A[c][i];
            }
            const Vector solution = solveLinearSystem(normal, rhs, 0.0);
            for (size_t r = 0; r < solution.size(); ++r) {
                result[r][c] = solution[r];
            }
        }
        return result;
    }

    const Matrix At = transpose(A);
    Matrix normal = multiply(A, At);
    for (size_t i = 0; i < std::min(rowCount(normal), colCount(normal)); ++i) {
        normal[i][i] += regularization;
    }

    Matrix inverse_part = makeMatrix(rowCount(A), rowCount(A), 0.0);
    for (size_t c = 0; c < rowCount(A); ++c) {
        Vector e(rowCount(A), 0.0);
        e[c] = 1.0;
        const Vector column_solution = solveLinearSystem(normal, e, 0.0);
        for (size_t r = 0; r < rowCount(A); ++r) {
            inverse_part[r][c] = column_solution[r];
        }
    }
    return multiply(At, inverse_part);
}

EigenDecomposition jacobiEigenDecomposition(const Matrix& symmetric,
                                            size_t max_iterations,
                                            double tolerance) {
    EigenDecomposition result;
    if (symmetric.empty()) {
        return result;
    }

    const size_t n = symmetric.size();
    Matrix A = symmetric;
    Matrix V = identityMatrix(n);

    for (size_t iter = 0; iter < max_iterations; ++iter) {
        size_t p = 0;
        size_t q = 1;
        double max_offdiag = 0.0;
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const double value = std::abs(A[i][j]);
                if (value > max_offdiag) {
                    max_offdiag = value;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_offdiag < tolerance) {
            break;
        }

        const double app = A[p][p];
        const double aqq = A[q][q];
        const double apq = A[p][q];
        const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(phi);
        const double s = std::sin(phi);

        for (size_t i = 0; i < n; ++i) {
            const double aip = A[i][p];
            const double aiq = A[i][q];
            A[i][p] = c * aip - s * aiq;
            A[i][q] = s * aip + c * aiq;
        }
        for (size_t j = 0; j < n; ++j) {
            const double apj = A[p][j];
            const double aqj = A[q][j];
            A[p][j] = c * apj - s * aqj;
            A[q][j] = s * apj + c * aqj;
        }
        A[p][q] = 0.0;
        A[q][p] = 0.0;

        for (size_t i = 0; i < n; ++i) {
            const double vip = V[i][p];
            const double viq = V[i][q];
            V[i][p] = c * vip - s * viq;
            V[i][q] = s * vip + c * viq;
        }
    }

    std::vector<size_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t lhs, size_t rhs) {
        return A[lhs][lhs] > A[rhs][rhs];
    });

    result.values.resize(n, 0.0);
    result.vectors = makeMatrix(n, n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        result.values[i] = A[order[i]][order[i]];
        for (size_t r = 0; r < n; ++r) {
            result.vectors[r][i] = V[r][order[i]];
        }
    }
    return result;
}

double dot(const Vector& lhs, const Vector& rhs) {
    double result = 0.0;
    for (size_t i = 0; i < lhs.size() && i < rhs.size(); ++i) {
        result += lhs[i] * rhs[i];
    }
    return result;
}

double norm(const Vector& vector) {
    return std::sqrt(dot(vector, vector));
}

double frobeniusNorm(const Matrix& matrix) {
    double acc = 0.0;
    for (const auto& row : matrix) {
        for (double value : row) {
            acc += value * value;
        }
    }
    return std::sqrt(acc);
}

Vector column(const Matrix& matrix, size_t index) {
    Vector result(matrix.size(), 0.0);
    for (size_t i = 0; i < matrix.size(); ++i) {
        if (index < matrix[i].size()) {
            result[i] = matrix[i][index];
        }
    }
    return result;
}

void appendRow(Matrix& matrix, const Vector& row) {
    matrix.push_back(row);
}

double conditionNumber(const Matrix& matrix, double regularization) {
    if (matrix.empty()) {
        return 0.0;
    }

    Matrix gram = multiply(transpose(matrix), matrix);
    for (size_t i = 0; i < std::min(rowCount(gram), colCount(gram)); ++i) {
        gram[i][i] += regularization;
    }
    const EigenDecomposition eig = jacobiEigenDecomposition(gram);
    if (eig.values.empty()) {
        return 0.0;
    }

    const double largest = std::sqrt(std::max(eig.values.front(), regularization));
    double smallest = largest;
    for (double value : eig.values) {
        if (value > regularization) {
            smallest = std::sqrt(value);
        }
    }
    return smallest > kEpsilon ? largest / smallest : std::numeric_limits<double>::infinity();
}

} // namespace Identification