#include <tether/identification/DenseLinearAlgebra.hpp>

#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace Identification {

namespace {

constexpr double kEpsilon = 1e-9;

size_t rowCount(const Matrix& matrix) {
    return matrix.size();
}

size_t colCount(const Matrix& matrix) {
    return matrix.empty() ? 0 : matrix.front().size();
}

// --- Conversion helpers: std::vector<vector<double>> <-> Eigen::MatrixXd ---

Eigen::MatrixXd toEigen(const Matrix& matrix) {
    if (matrix.empty()) {
        return {};
    }
    const size_t rows = rowCount(matrix);
    const size_t cols = colCount(matrix);
    Eigen::MatrixXd result(rows, cols);
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            result(r, c) = matrix[r][c];
        }
    }
    return result;
}

Matrix fromEigen(const Eigen::MatrixXd& matrix) {
    Matrix result(static_cast<size_t>(matrix.rows()), Vector(matrix.cols(), 0.0));
    for (int r = 0; r < matrix.rows(); ++r) {
        for (int c = 0; c < matrix.cols(); ++c) {
            result[r][c] = matrix(r, c);
        }
    }
    return result;
}

Eigen::VectorXd toEigenVector(const Vector& vector) {
    Eigen::VectorXd result(vector.size());
    for (size_t i = 0; i < vector.size(); ++i) {
        result(i) = vector[i];
    }
    return result;
}

Vector fromEigenVector(const Eigen::VectorXd& vector) {
    Vector result(vector.size(), 0.0);
    for (int i = 0; i < vector.size(); ++i) {
        result[i] = vector(i);
    }
    return result;
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
    return fromEigen(toEigen(matrix).transpose());
}

Matrix multiply(const Matrix& lhs, const Matrix& rhs) {
    if (lhs.empty() || rhs.empty()) {
        return {};
    }
    return fromEigen(toEigen(lhs) * toEigen(rhs));
}

Vector multiply(const Matrix& lhs, const Vector& rhs) {
    if (lhs.empty()) {
        return {};
    }
    return fromEigenVector(toEigen(lhs) * toEigenVector(rhs));
}

Matrix add(const Matrix& lhs, const Matrix& rhs) {
    if (lhs.empty()) {
        return rhs;
    }
    if (rhs.empty()) {
        return lhs;
    }
    return fromEigen(toEigen(lhs) + toEigen(rhs));
}

Matrix subtract(const Matrix& lhs, const Matrix& rhs) {
    if (lhs.empty()) {
        return {};
    }
    if (rhs.empty()) {
        return lhs;
    }
    return fromEigen(toEigen(lhs) - toEigen(rhs));
}

Matrix outerProduct(const Vector& lhs, const Vector& rhs) {
    if (lhs.empty() || rhs.empty()) {
        return {};
    }
    return fromEigen(toEigenVector(lhs) * toEigenVector(rhs).transpose());
}

Vector solveLinearSystem(Matrix A, Vector b, double regularization) {
    if (A.empty()) {
        return {};
    }

    const size_t n = A.size();
    Eigen::MatrixXd eigenA = toEigen(A);
    Eigen::VectorXd eigenB = toEigenVector(b);

    for (size_t i = 0; i < n; ++i) {
        eigenA(i, i) += regularization;
    }

    // Use LDLT decomposition for symmetric systems, LU for general
    Eigen::VectorXd solution;
    if (n > 0 && eigenA.isApprox(eigenA.transpose(), kEpsilon)) {
        Eigen::LDLT<Eigen::MatrixXd> ldlt(eigenA);
        if (ldlt.info() != Eigen::Success || ldlt.vectorD().cwiseAbs().minCoeff() < kEpsilon) {
            return Vector(n, 0.0);
        }
        solution = ldlt.solve(eigenB);
    } else {
        Eigen::PartialPivLU<Eigen::MatrixXd> lu(eigenA);
        if (lu.determinant() == 0.0) {
            return Vector(n, 0.0);
        }
        solution = lu.solve(eigenB);
    }

    return fromEigenVector(solution);
}

Vector solveLeastSquares(const Matrix& A, const Vector& b, double regularization) {
    if (A.empty()) {
        return {};
    }

    Eigen::MatrixXd eigenA = toEigen(A);
    Eigen::VectorXd eigenB = toEigenVector(b);

    // Apply Tikhonov regularization: solve (A^T A + reg*I) x = A^T b
    // using Eigen's least-squares with regularization
    if (regularization > 0.0) {
        Eigen::MatrixXd AtA = eigenA.transpose() * eigenA;
        AtA += regularization * Eigen::MatrixXd::Identity(AtA.rows(), AtA.cols());
        Eigen::VectorXd Atb = eigenA.transpose() * eigenB;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(AtA);
        if (ldlt.info() != Eigen::Success) {
            return Vector(eigenA.cols(), 0.0);
        }
        return fromEigenVector(ldlt.solve(Atb));
    }

    // Use QR decomposition for unregularized least squares
    Eigen::VectorXd solution = eigenA.colPivHouseholderQr().solve(eigenB);
    return fromEigenVector(solution);
}

Matrix pseudoInverse(const Matrix& A, double regularization) {
    if (A.empty()) {
        return {};
    }

    Eigen::MatrixXd eigenA = toEigen(A);

    // Use complete orthogonal decomposition for pseudo-inverse
    // This handles both tall and wide matrices correctly
    if (regularization > 0.0) {
        // Tikhonov-regularized pseudo-inverse: (A^T A + reg I)^-1 A^T  or  A^T (A A^T + reg I)^-1
        if (eigenA.rows() >= eigenA.cols()) {
            Eigen::MatrixXd AtA = eigenA.transpose() * eigenA;
            AtA += regularization * Eigen::MatrixXd::Identity(AtA.rows(), AtA.cols());
            Eigen::LDLT<Eigen::MatrixXd> ldlt(AtA);
            if (ldlt.info() == Eigen::Success) {
                return fromEigen(ldlt.solve(eigenA.transpose()));
            }
        } else {
            Eigen::MatrixXd AAt = eigenA * eigenA.transpose();
            AAt += regularization * Eigen::MatrixXd::Identity(AAt.rows(), AAt.cols());
            Eigen::LDLT<Eigen::MatrixXd> ldlt(AAt);
            if (ldlt.info() == Eigen::Success) {
                return fromEigen(eigenA.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(AAt.rows(), AAt.cols())));
            }
        }
    }

    // Unregularized: use Jacobi SVD for robust pseudo-inverse
    // A+ = V * Sigma+ * U^T, where Sigma+ inverts non-zero singular values
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(eigenA, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& sv = svd.singularValues();
    const double pinv_tol = kEpsilon * sv.size() > 0 ? sv(0) * kEpsilon : kEpsilon;
    Eigen::VectorXd sinv(sv.size());
    for (int i = 0; i < sv.size(); ++i) {
        sinv(i) = (std::abs(sv(i)) > pinv_tol) ? 1.0 / sv(i) : 0.0;
    }
    return fromEigen(svd.matrixV() * sinv.asDiagonal() * svd.matrixU().transpose());
}

EigenDecomposition jacobiEigenDecomposition(const Matrix& symmetric,
                                            size_t /*max_iterations*/,
                                            double /*tolerance*/) {
    EigenDecomposition result;
    if (symmetric.empty()) {
        return result;
    }

    Eigen::MatrixXd eigenSym = toEigen(symmetric);

    // Use Eigen's SelfAdjointEigenSolver (equivalent to Jacobi but more robust)
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(eigenSym);

    if (solver.info() != Eigen::Success) {
        // Fallback: return zeros
        const size_t n = symmetric.size();
        result.values.resize(n, 0.0);
        result.vectors = identityMatrix(n);
        return result;
    }

    // SelfAdjointEigenSolver returns eigenvalues in ascending order;
    // we need descending order to match the original API
    const Eigen::VectorXd& eigenvalues = solver.eigenvalues();
    const Eigen::MatrixXd& eigenvectors = solver.eigenvectors();

    const int n = static_cast<int>(symmetric.size());
    result.values.resize(n, 0.0);
    result.vectors = makeMatrix(n, n, 0.0);

    // Reverse order (Eigen gives ascending, we want descending)
    for (int i = 0; i < n; ++i) {
        const int src_idx = n - 1 - i;
        result.values[i] = eigenvalues(src_idx);
        for (int r = 0; r < n; ++r) {
            result.vectors[r][i] = eigenvectors(r, src_idx);
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
    if (matrix.empty()) {
        return 0.0;
    }
    return toEigen(matrix).norm();
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
    if (matrix.empty() || colCount(matrix) == 0) {
        return 0.0;
    }

    Eigen::MatrixXd eigenA = toEigen(matrix);

    // Use SVD to compute singular values directly
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(eigenA);
    const Eigen::VectorXd& singularValues = svd.singularValues();

    if (singularValues.size() == 0) {
        return 0.0;
    }

    double largest = singularValues(0);
    double smallest = singularValues(singularValues.size() - 1);

    // Apply regularization to smallest singular value
    if (regularization > 0.0) {
        smallest = std::sqrt(smallest * smallest + regularization * regularization);
    }

    return smallest > kEpsilon ? largest / smallest : std::numeric_limits<double>::infinity();
}

} // namespace Identification
