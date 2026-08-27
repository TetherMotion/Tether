/**
 * @file NumericalUtils.hpp
 * @brief Numerical utilities for the analytical TOPPRA-equivalent profiler.
 *
 * @details
 * This file provides the core numerical building blocks used by the
 * analytical velocity profiler:
 *
 * - **LGL nodes and weights**: Legendre-Gauss-Lobatto quadrature nodes on
 *   [-1, 1], used as collocation points for spectral elements.
 * - **LGL derivative matrix**: The dense differentiation matrix D such that
 *   df/dxi at node i = sum_j D[i][j] * f(xj_j). Computed via barycentric
 *   interpolation formula (Canuto et al., "Spectral Methods").
 * - **Barycentric Lagrange interpolation**: Stable O(N) evaluation and
 *   differentiation of polynomial interpolants at arbitrary points.
 * - **Padé approximant**: Rational approximant R_[m/n](x) = P(x)/Q(x) with
 *   coefficients computed from Taylor series via the Padé equations.
 * - **RK4 ODE integrator**: Classical 4th-order Runge-Kutta for the
 *   arc-length dynamics system.
 * - **Bracketed Newton solver**: Safeguarded Newton-bisection root finding
 *   for monotone function inversion (used to invert t(s) -> s(t)).
 *
 * ## Mathematical Background
 *
 * ### LGL Spectral Elements
 *
 * On the reference interval [-1, 1], the N+1 LGL nodes are the zeros of
 * (1-xi^2) * L_N'(xi), where L_N is the Legendre polynomial of degree N.
 * Equivalently, they are xi_0 = -1, xi_N = 1, and the interior nodes are
 * the roots of L_N'(xi).
 *
 * The derivative matrix entries are:
 *   D[i][j] = L_j'(xi_i)
 * where L_j is the j-th Lagrange basis polynomial. Using barycentric form:
 *   D[i][j] = (lambda_j / lambda_i) / (xi_i - xi_j)   for i != j
 *   D[i][i] = -sum_{j != i} D[i][j]
 * where lambda_j are the barycentric weights.
 *
 * For LGL nodes, the barycentric weights are:
 *   lambda_0 = lambda_N = 1/(N*(N+1))
 *   lambda_j = 1/((N*(N+1)) * L_N(xi_j)^2)   for 0 < j < N
 *
 * ### Padé Approximation
 *
 * Given a Taylor series f(x) = sum_{k=0}^{inf} c_k x^k, the [m/n] Padé
 * approximant is:
 *   R(x) = (a_0 + a_1*x + ... + a_m*x^m) / (1 + b_1*x + ... + b_n*x^n)
 *
 * The coefficients satisfy:
 *   sum_{j=0}^{n} b_j * c_{i-j} = a_i   for i = 0..m   (with b_0 = 1)
 *   sum_{j=0}^{n} b_j * c_{i-j} = 0     for i = m+1..m+n
 *
 * The second set of equations is a linear system for b_1..b_n (the b_j with
 * j > i give zero contribution since c_k = 0 for k < 0). Once b is known,
 * a is computed from the first set.
 *
 * @see AnalyticalJerkLimitedTOPPRA.hpp for the profiler that uses these utilities.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <array>
#include <stdexcept>
#include <functional>
#include <string>

namespace MotionPlanner::analytical {

// ============================================================================
// Constants
// ============================================================================

namespace detail {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kEps = 1e-14;
}

// ============================================================================
// LGL (Legendre-Gauss-Lobatto) Nodes and Derivative Matrix
// ============================================================================

/**
 * @brief Compute N+1 Legendre-Gauss-Lobatto nodes on [-1, 1].
 *
 * The nodes are: xi_0 = -1, xi_N = 1, and the interior nodes are the
 * roots of L_N'(xi) (the derivative of the degree-N Legendre polynomial).
 *
 * Interior nodes are found by Newton iteration on the Legendre polynomial
 * recurrence:
 *   (N+1) * L_{N+1}(x) = (2N+1) * x * L_N(x) - N * L_{N-1}(x)
 *   (1-x^2) * L_N'(x) = N * (L_{N-1}(x) - x * L_N(x))
 *
 * @param N Polynomial degree (number of nodes = N+1, N >= 1)
 * @return Vector of N+1 nodes in [-1, 1], sorted ascending
 */
inline std::vector<double> lglNodes(int N) {
    if (N < 1) {
        throw std::invalid_argument("lglNodes: N must be >= 1");
    }
    if (N == 1) {
        return {-1.0, 1.0};
    }

    std::vector<double> nodes(N + 1);
    nodes[0] = -1.0;
    nodes[N] = 1.0;

    // Interior nodes: roots of L_N'(x).
    // Use the asymptotic approximation as initial guess, then Newton refine.
    for (int k = 1; k < N; ++k) {
        // Initial guess: Chebyshev-like distribution (good approximation)
        double x = -std::cos(detail::kPi * k / N);

        // Newton iteration on f(x) = L_N'(x) = 0
        // Using the identity: (1-x^2)*L_N'(x) = N*(L_{N-1}(x) - x*L_N(x))
        // So f(x) = L_{N-1}(x) - x*L_N(x), and we need f(x) = 0.
        // f'(x) = L_{N-1}'(x) - L_N(x) - x*L_N'(x)
        //       = -N*(N+1)*L_N(x)  [using Legendre ODE identities]
        for (int iter = 0; iter < 100; ++iter) {
            // Compute L_N(x) and L_{N-1}(x) via recurrence
            double Lnm1 = 1.0;  // L_0
            double Ln = x;       // L_1
            for (int n = 1; n < N; ++n) {
                double Lnp1 = ((2.0 * n + 1.0) * x * Ln - n * Lnm1) / (n + 1.0);
                Lnm1 = Ln;
                Ln = Lnp1;
            }
            // Now Ln = L_N(x), Lnm1 = L_{N-1}(x)
            double f = Lnm1 - x * Ln;           // = (1-x^2)*L_N'(x) / N
            double fp = -N * (N + 1.0) * Ln;    // derivative of f w.r.t. x

            if (std::abs(fp) < detail::kEps) break;
            double dx = f / fp;
            x -= dx;
            if (std::abs(dx) < detail::kEps) break;
        }
        nodes[k] = x;
    }

    // Ensure ascending order
    std::sort(nodes.begin(), nodes.end());
    return nodes;
}

/**
 * @brief Compute the barycentric weights for arbitrary interpolation nodes.
 *
 * The barycentric weights are:
 *   lambda_j = 1 / prod_{k != j} (x_j - x_k)
 *
 * This is computed directly in O(N^2) operations. For LGL nodes, a
 * closed-form exists involving Legendre polynomial values, but the
 * direct computation is simpler and always correct.
 *
 * @param nodes Interpolation nodes (sorted ascending)
 * @return Vector of barycentric weights
 */
inline std::vector<double> lglBarycentricWeights(const std::vector<double>& nodes, int /*N*/) {
    const int M = static_cast<int>(nodes.size());
    std::vector<double> w(M, 1.0);
    for (int j = 0; j < M; ++j) {
        for (int k = 0; k < M; ++k) {
            if (k == j) continue;
            w[j] /= (nodes[j] - nodes[k]);
        }
    }
    return w;
}

/**
 * @brief Compute the LGL derivative matrix D.
 *
 * D[i][j] = dL_j/dxi at xi_i, where L_j is the j-th Lagrange basis
 * polynomial for the LGL nodes.
 *
 * Using the barycentric formula:
 *   D[i][j] = (lambda_j / lambda_i) / (xi_i - xi_j)   for i != j
 *   D[i][i] = -sum_{j != i} D[i][j]
 *
 * @param nodes LGL nodes
 * @param N Polynomial degree (nodes.size() == N+1)
 * @return (N+1) x (N+1) derivative matrix
 */
inline std::vector<std::vector<double>> lglDerivativeMatrix(
    const std::vector<double>& nodes, int N) {
    const int M = N + 1;
    auto w = lglBarycentricWeights(nodes, N);

    std::vector<std::vector<double>> D(M, std::vector<double>(M, 0.0));

    for (int i = 0; i < M; ++i) {
        double sum = 0.0;
        for (int j = 0; j < M; ++j) {
            if (i == j) continue;
            double denom = nodes[i] - nodes[j];
            if (std::abs(denom) < detail::kEps) continue;
            D[i][j] = w[j] / (w[i] * denom);
            sum += D[i][j];
        }
        D[i][i] = -sum;
    }
    return D;
}

// ============================================================================
// Barycentric Lagrange Interpolation
// ============================================================================

/**
 * @brief Evaluate a polynomial interpolant at a point using barycentric form.
 *
 * Given nodes x_j, values f_j, and barycentric weights lambda_j:
 *   p(x) = (sum_j lambda_j * f_j / (x - x_j)) / (sum_j lambda_j / (x - x_j))
 *
 * If x coincides with a node, returns the exact value at that node.
 *
 * @param nodes Interpolation nodes
 * @param values Function values at nodes
 * @param weights Barycentric weights
 * @param x Evaluation point
 * @return Interpolated value
 */
inline double barycentricEvaluate(
    const std::vector<double>& nodes,
    const std::vector<double>& values,
    const std::vector<double>& weights,
    double x) {
    const int M = static_cast<int>(nodes.size());

    // Check for exact node match
    for (int j = 0; j < M; ++j) {
        if (std::abs(x - nodes[j]) < detail::kEps) {
            return values[j];
        }
    }

    double num = 0.0;
    double den = 0.0;
    for (int j = 0; j < M; ++j) {
        double w = weights[j] / (x - nodes[j]);
        num += w * values[j];
        den += w;
    }
    return num / den;
}

/**
 * @brief Evaluate the derivative of a polynomial interpolant at a point.
 *
 * Uses the formula:
 *   p'(x) = (sum_j lambda_j * f_j / (x - x_j)^2 * (something)...) 
 *
 * More precisely, for the barycentric form p(x) = N(x)/D(x) where:
 *   N(x) = sum_j lambda_j * f_j / (x - x_j)
 *   D(x) = sum_j lambda_j / (x - x_j)
 *
 * Then p'(x) = (N'(x)*D(x) - N(x)*D'(x)) / D(x)^2, where:
 *   N'(x) = -sum_j lambda_j * f_j / (x - x_j)^2
 *   D'(x) = -sum_j lambda_j / (x - x_j)^2
 *
 * @param nodes Interpolation nodes
 * @param values Function values at nodes
 * @param weights Barycentric weights
 * @param x Evaluation point
 * @return Derivative value
 */
inline double barycentricDerivative(
    const std::vector<double>& nodes,
    const std::vector<double>& values,
    const std::vector<double>& weights,
    double x) {
    const int M = static_cast<int>(nodes.size());

    // Check for exact node match: use derivative matrix row
    for (int i = 0; i < M; ++i) {
        if (std::abs(x - nodes[i]) < detail::kEps) {
            // p'(x_i) = sum_j D[i][j] * f_j
            //          = sum_{j!=i} D[i][j]*(f_j - f_i)
            // where D[i][j] = (lambda_j / lambda_i) / (x_i - x_j) for i != j
            double deriv = 0.0;
            for (int j = 0; j < M; ++j) {
                if (i == j) continue;
                double denom = nodes[i] - nodes[j];
                if (std::abs(denom) < detail::kEps) continue;
                double Dij = weights[j] / (weights[i] * denom);
                deriv += Dij * (values[j] - values[i]);
            }
            return deriv;
        }
    }

    // General point: use quotient rule on barycentric form
    // p(x) = N(x)/D(x) where:
    //   N(x) = sum_j lambda_j * f_j / (x - x_j)
    //   D(x) = sum_j lambda_j / (x - x_j)
    // p'(x) = (N'(x)*D(x) - N(x)*D'(x)) / D(x)^2
    //   N'(x) = -sum_j lambda_j * f_j / (x - x_j)^2
    //   D'(x) = -sum_j lambda_j / (x - x_j)^2
    double N = 0.0, D = 0.0;
    double Np = 0.0, Dp = 0.0;
    for (int j = 0; j < M; ++j) {
        double diff = x - nodes[j];
        double w = weights[j] / diff;
        double w2 = weights[j] / (diff * diff);
        N += w * values[j];
        D += w;
        Np -= w2 * values[j];
        Dp -= w2;
    }
    return (Np * D - N * Dp) / (D * D);
}

// ============================================================================
// Padé Approximant
// ============================================================================

/**
 * @brief Padé approximant R_[m/n](x) = P(x) / Q(x).
 *
 *   P(x) = a_0 + a_1*x + ... + a_m*x^m
 *   Q(x) = 1 + b_1*x + ... + b_n*x^n
 *
 * Coefficients are computed from the Taylor series of the target function
 * by solving the Padé linear system.
 */
class PadeApproximant {
public:
    int m;  // numerator degree
    int n;  // denominator degree
    std::vector<double> a;  // size m+1
    std::vector<double> b;  // size n+1, b[0] = 1

    /**
     * @brief Construct a [m/n] Padé approximant from Taylor coefficients.
     *
     * @param taylor Coefficients c_0, c_1, ..., c_{m+n} of the Taylor series
     *                f(x) = sum c_k x^k
     * @param mNum Numerator degree
     * @param nDen Denominator degree
     */
    PadeApproximant(const std::vector<double>& taylor, int mNum, int nDen)
        : m(mNum), n(nDen) {
        if (static_cast<int>(taylor.size()) < m + n + 1) {
            throw std::invalid_argument(
                "PadeApproximant: need at least m+n+1 Taylor coefficients");
        }
        a.assign(m + 1, 0.0);
        b.assign(n + 1, 0.0);
        b[0] = 1.0;

        if (n == 0) {
            // Pure polynomial: a_k = c_k
            for (int k = 0; k <= m; ++k) a[k] = taylor[k];
            return;
        }

        // Solve for b_1..b_n from:
        //   sum_{j=0}^{n} b_j * c_{i-j} = 0   for i = m+1..m+n
        // (with b_0 = 1, c_k = 0 for k < 0)
        //
        // This is an n x n linear system: A * b[1..n] = rhs
        // A[i-1][j-1] = c_{m+i-j}  for i=1..n, j=1..n
        // rhs[i-1] = -c_{m+i}
        std::vector<std::vector<double>> A(n, std::vector<double>(n, 0.0));
        std::vector<double> rhs(n, 0.0);
        for (int i = 1; i <= n; ++i) {
            rhs[i - 1] = -taylor[m + i];
            for (int j = 1; j <= n; ++j) {
                int idx = m + i - j;
                A[i - 1][j - 1] = (idx >= 0) ? taylor[idx] : 0.0;
            }
        }

        // Gaussian elimination with partial pivoting
        for (int col = 0; col < n; ++col) {
            // Find pivot
            int piv = col;
            double maxVal = std::abs(A[col][col]);
            for (int row = col + 1; row < n; ++row) {
                if (std::abs(A[row][col]) > maxVal) {
                    maxVal = std::abs(A[row][col]);
                    piv = row;
                }
            }
            if (maxVal < detail::kEps) {
                // Singular: fall back to lower-order denominator
                for (int k = 1; k <= n; ++k) b[k] = 0.0;
                // Compute a from truncated Taylor series
                for (int k = 0; k <= m; ++k) a[k] = taylor[k];
                return;
            }
            std::swap(A[col], A[piv]);
            std::swap(rhs[col], rhs[piv]);

            // Eliminate
            for (int row = col + 1; row < n; ++row) {
                double factor = A[row][col] / A[col][col];
                for (int k = col; k < n; ++k) {
                    A[row][k] -= factor * A[col][k];
                }
                rhs[row] -= factor * rhs[col];
            }
        }

        // Back-substitution
        for (int i = n - 1; i >= 0; --i) {
            double sum = rhs[i];
            for (int j = i + 1; j < n; ++j) {
                sum -= A[i][j] * b[j + 1];
            }
            b[i + 1] = sum / A[i][i];
        }

        // Compute a_k from: a_k = sum_{j=0}^{min(k,n)} b_j * c_{k-j}
        for (int k = 0; k <= m; ++k) {
            double sum = 0.0;
            for (int j = 0; j <= std::min(k, n); ++j) {
                sum += b[j] * taylor[k - j];
            }
            a[k] = sum;
        }
    }

    /** Default constructor (identity approximant R[0/0] = c_0) */
    PadeApproximant() : m(0), n(0), a(1, 0.0), b(1, 1.0) {}

    /**
     * @brief Evaluate R(x) = P(x) / Q(x).
     */
    double evaluate(double x) const {
        double num = 0.0;
        double xpow = 1.0;
        for (int k = 0; k <= m; ++k) {
            num += a[k] * xpow;
            xpow *= x;
        }
        double den = 1.0;
        xpow = x;
        for (int k = 1; k <= n; ++k) {
            den += b[k] * xpow;
            xpow *= x;
        }
        return num / den;
    }

    /**
     * @brief Evaluate R'(x) = (P'Q - PQ') / Q^2.
     */
    double evaluateDerivative(double x) const {
        // P(x) and P'(x)
        double P = a[m];
        for (int k = m - 1; k >= 0; --k) P = P * x + a[k];
        double Pp = 0.0;
        if (m >= 1) {
            Pp = m * a[m];
            for (int k = m - 1; k >= 1; --k) Pp = Pp * x + k * a[k];
        }
        // Q(x) and Q'(x)
        double Q = b[n];
        for (int k = n - 1; k >= 0; --k) Q = Q * x + b[k];
        double Qp = 0.0;
        if (n >= 1) {
            Qp = n * b[n];
            for (int k = n - 1; k >= 1; --k) Qp = Qp * x + k * b[k];
        }
        return (Pp * Q - P * Qp) / (Q * Q);
    }

    /**
     * @brief Evaluate R''(x).
     *
     * R'' = [(P''Q - PQ'')*Q^2 - 2*(P'Q - PQ')*Q*Q'] / Q^4
     *      = [(P''Q - PQ'')*Q - 2*(P'Q - PQ')*Q'] / Q^3
     */
    double evaluate2ndDerivative(double x) const {
        // P, P', P''
        double P = a[m];
        for (int k = m - 1; k >= 0; --k) P = P * x + a[k];
        double Pp = 0.0, Ppp = 0.0;
        if (m >= 1) {
            Pp = m * a[m];
            for (int k = m - 1; k >= 1; --k) Pp = Pp * x + k * a[k];
            if (m >= 2) {
                Ppp = m * (m - 1) * a[m];
                for (int k = m - 1; k >= 2; --k) Ppp = Ppp * x + k * (k - 1) * a[k];
            }
        }
        // Q, Q', Q''
        double Q = b[n];
        for (int k = n - 1; k >= 0; --k) Q = Q * x + b[k];
        double Qp = 0.0, Qpp = 0.0;
        if (n >= 1) {
            Qp = n * b[n];
            for (int k = n - 1; k >= 1; --k) Qp = Qp * x + k * b[k];
            if (n >= 2) {
                Qpp = n * (n - 1) * b[n];
                for (int k = n - 1; k >= 2; --k) Qpp = Qpp * x + k * (k - 1) * b[k];
            }
        }
        double Rprime = (Pp * Q - P * Qp) / (Q * Q);
        double RprimePrime_num = (Ppp * Q - P * Qpp) * Q - 2.0 * (Pp * Q - P * Qp) * Qp;
        return RprimePrime_num / (Q * Q * Q);
    }
};

// ============================================================================
// RK4 ODE Integrator
// ============================================================================

/**
 * @brief Classical 4th-order Runge-Kutta integrator for a system of ODEs.
 *
 * Integrates y' = f(t, y) with fixed step size h.
 *
 * @tparam State The state vector type (must support arithmetic: +, *, double)
 * @param f Right-hand side function: f(t, y) -> dy/dt
 * @param t Current time
 * @param y Current state
 * @param h Step size
 * @return State at t + h
 */
template<typename State, typename RHS>
inline State rk4Step(const RHS& f, double t, const State& y, double h) {
    State k1 = f(t, y);
    State k2 = f(t + h * 0.5, y + k1 * (h * 0.5));
    State k3 = f(t + h * 0.5, y + k2 * (h * 0.5));
    State k4 = f(t + h, y + k3 * h);
    return y + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
}

/**
 * @brief Adaptive RK4 integration with error estimation (Dormand-Prince style).
 *
 * Uses step doubling for error estimation: take one step of size h, and
 * two steps of size h/2. The error is estimated as |y_h - y_{h/2}| / 15.
 * The step size is adjusted to meet the tolerance.
 *
 * @tparam State State vector type
 * @param f RHS function
 * @param t Current time
 * @param y Current state
 * @param h Initial step size guess
 * @param absTol Absolute tolerance
 * @param relTol Relative tolerance
 * @return Pair of (state at t+h_actual, h_actual used)
 */
template<typename State, typename RHS>
inline std::pair<State, double> rk4AdaptiveStep(
    const RHS& f, double t, const State& y, double h,
    double absTol = 1e-12, double relTol = 1e-10) {
    // Take one big step
    State y1 = rk4Step(f, t, y, h);
    // Take two half steps
    State y2 = rk4Step(f, t, y, h * 0.5);
    y2 = rk4Step(f, t + h * 0.5, y2, h * 0.5);

    // Error estimate (Richardson extrapolation)
    State err = (y2 - y1) * (1.0 / 15.0);
    double errNorm = 0.0;
    for (const auto& e : err.components()) {
        errNorm += e * e;
    }
    errNorm = std::sqrt(errNorm);

    double scale = absTol + relTol * std::max(y1.norm(), y2.norm());
    double relErr = errNorm / scale;

    if (relErr <= 1.0) {
        // Step accepted; use Richardson extrapolation for better result
        return {y2 + err * 16.0, h};
    } else {
        // Step rejected; reduce h and retry
        double factor = 0.9 * std::pow(1.0 / relErr, 0.25);
        factor = std::max(factor, 0.1);
        return rk4AdaptiveStep(f, t, y, h * factor, absTol, relTol);
    }
}

// ============================================================================
// Bracketed Newton-Bisection Root Finder
// ============================================================================

/**
 * @brief Solve f(x) = target for x in [a, b], where f is monotone.
 *
 * Uses Newton iteration safeguarded by bisection. Requires f(a) and f(b)
 * to bracket the target (i.e., (f(a) - target) * (f(b) - target) <= 0).
 *
 * @param f Function to invert (monotone on [a, b])
 * @param df Derivative of f
 * @param target Target value
 * @param a Left bracket
 * @param b Right bracket
 * @param tol Tolerance on |f(x) - target|
 * @param maxIter Maximum iterations
 * @return Root x such that f(x) ≈ target
 */
inline double newtonBisection(
    const std::function<double(double)>& f,
    const std::function<double(double)>& df,
    double target,
    double a, double b,
    double tol = 1e-12, int maxIter = 100) {
    double fa = f(a) - target;
    double fb = f(b) - target;

    if (std::abs(fa) < tol) return a;
    if (std::abs(fb) < tol) return b;

    if (fa * fb > 0) {
        // Not bracketed — return the closer endpoint
        return (std::abs(fa) < std::abs(fb)) ? a : b;
    }

    double x = 0.5 * (a + b);
    double fx = f(x) - target;

    for (int iter = 0; iter < maxIter; ++iter) {
        if (std::abs(fx) < tol) return x;

        // Try Newton step
        double dfx = df(x);
        double xNewt = x - fx / dfx;

        // Check if Newton step is within bracket
        if (xNewt > a && xNewt < b && std::abs(dfx) > detail::kEps) {
            x = xNewt;
        } else {
            // Bisection
            if (fa * fx < 0) {
                b = x;
                fb = fx;
            } else {
                a = x;
                fa = fx;
            }
            x = 0.5 * (a + b);
        }

        fx = f(x) - target;
    }

    return x;
}

} // namespace MotionPlanner::analytical
