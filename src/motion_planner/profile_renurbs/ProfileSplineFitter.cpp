/**
 * @file ProfileSplineFitter.cpp
 * @brief Implementation of ProfileSplineFitter (see header for design).
 */

#include "tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tether::motion::profile_renurbs {

namespace {

constexpr double kTol = 1e-14;

// ============================================================================
// B-spline basis functions (Cox–de Boor)
// ============================================================================

/// Find the span index i such that knots[i] <= u < knots[i+1].
/// Returns the index of the left end of the span containing u.
int findSpan(const std::vector<double>& knots, int degree, double u) {
    const int n = static_cast<int>(knots.size()) - degree - 2; // #CP - 1
    if (u >= knots[n + 1]) return n;
    if (u <= knots[degree]) return degree;
    int lo = degree;
    int hi = n + 1;
    int mid = (lo + hi) / 2;
    while (u < knots[mid] || u >= knots[mid + 1]) {
        if (u < knots[mid]) hi = mid;
        else lo = mid;
        mid = (lo + hi) / 2;
    }
    return mid;
}

/// Evaluate all non-zero B-spline basis functions at u in span `span`.
/// Returns a vector of size degree+1: N_{span-degree+1}, ..., N_{span}.
/// (Piegl & Tiller A2.2.)
std::vector<double> basisFunctions(int span, double u, int degree,
                                   const std::vector<double>& knots) {
    std::vector<double> N(degree + 1, 0.0);
    std::vector<double> left(degree + 1, 0.0);
    std::vector<double> right(degree + 1, 0.0);
    N[0] = 1.0;
    for (int j = 1; j <= degree; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    return N;
}

/// Evaluate all non-zero B-spline basis function *derivatives* at u.
/// Returns the first derivative values: dN/du for the basis functions
/// N_{span-degree} .. N_{span} (degree+1 values).
/// Uses the direct formula: dN_{i,p}/du = p * (N_{i,p-1}/(u_{i+p}-u_i)
///                                       - N_{i+1,p-1}/(u_{i+p+1}-u_{i+1}))
std::vector<double> basisFunctionDerivatives(int span, double u, int degree,
                                             const std::vector<double>& knots) {
    std::vector<double> ders(degree + 1, 0.0);

    if (degree == 1) {
        // Linear B-spline: N_{span-1,1}' = -1/h, N_{span,1}' = 1/h
        double h = knots[span + 1] - knots[span];
        if (h > kTol) {
            ders[0] = -1.0 / h;
            ders[1] = 1.0 / h;
        }
        return ders;
    }

    // For degree >= 2: use the direct formula with degree-1 basis functions.
    int spanDm1 = findSpan(knots, degree - 1, u);
    auto Ndm1 = basisFunctions(spanDm1, u, degree - 1, knots);
    // Ndm1 has `degree` entries: N_{spanDm1-(degree-1)} .. N_{spanDm1}
    int loDm1 = spanDm1 - (degree - 1);

    for (int i = 0; i <= degree; ++i) {
        int idx = span - degree + i; // global index of N_{idx,degree}
        double term1 = 0.0, term2 = 0.0;
        double denom1 = knots[idx + degree] - knots[idx];
        double denom2 = knots[idx + degree + 1] - knots[idx + 1];

        // N_{idx, degree-1}
        if (idx >= loDm1 && idx < loDm1 + static_cast<int>(Ndm1.size())) {
            if (denom1 > kTol)
                term1 = Ndm1[idx - loDm1] / denom1;
        }
        // N_{idx+1, degree-1}
        if (idx + 1 >= loDm1 && idx + 1 < loDm1 + static_cast<int>(Ndm1.size())) {
            if (denom2 > kTol)
                term2 = Ndm1[idx + 1 - loDm1] / denom2;
        }
        ders[i] = static_cast<double>(degree) * (term1 - term2);
    }

    return ders;
}

// ============================================================================
// Knot vector construction (Piegl & Tiller §9.3.1)
// ============================================================================

/// Compute the averaged knot vector for global B-spline interpolation.
/// Given parameters u[0..n] and degree p, returns a clamped knot vector
/// of size n + p + 2.
std::vector<double> computeKnotVector(const std::vector<double>& u, int degree) {
    int n = static_cast<int>(u.size()) - 1; // n+1 samples
    int m = n + degree + 1; // knot vector size - 1
    std::vector<double> knots(m + 1, 0.0);

    // Clamped: p+1 repetitions of 0 and 1
    for (int i = 0; i <= degree; ++i) {
        knots[i] = 0.0;
        knots[m - i] = 1.0;
    }

    // Interior knots by averaging
    for (int j = 1; j <= n - degree; ++j) {
        double sum = 0.0;
        for (int i = j; i <= j + degree - 1; ++i) {
            sum += u[i];
        }
        knots[degree + j] = sum / degree;
    }

    return knots;
}

// ============================================================================
// Banded linear solve for global B-spline interpolation
// ============================================================================

/// Solve the interpolation system N^T * N * P = N^T * Q for control points P.
/// N is the (n+1) x (n+1) basis matrix: N[i][j] = N_{j,degree}(u_i).
/// We build N directly and solve via Gaussian elimination with partial
/// pivoting (n is small, ≤ 64, so O(n³) is fine).
std::vector<double> solveInterpolation(
    const std::vector<double>& u,
    const std::vector<double>& q,
    const std::vector<double>& knots,
    int degree) {
    int n = static_cast<int>(u.size()); // n+1 samples → n control points
    // Number of control points = n (same as samples for global interpolation)
    int ncp = n;

    // Build the basis matrix: A[i][j] = N_{j,degree}(u_i)
    std::vector<std::vector<double>> A(n, std::vector<double>(ncp, 0.0));
    for (int i = 0; i < n; ++i) {
        int span = findSpan(knots, degree, u[i]);
        auto N = basisFunctions(span, u[i], degree, knots);
        // N covers indices span-degree .. span
        for (int j = 0; j <= degree; ++j) {
            int col = span - degree + j;
            if (col >= 0 && col < ncp) {
                A[i][col] = N[j];
            }
        }
    }

    // Solve A * P = q via Gaussian elimination with partial pivoting
    std::vector<double> P(q.begin(), q.end());
    // Augmented matrix
    for (int i = 0; i < n; ++i) {
        A[i].push_back(P[i]);
    }

    // Forward elimination
    for (int k = 0; k < n; ++k) {
        // Pivot
        int piv = k;
        for (int i = k + 1; i < n; ++i) {
            if (std::abs(A[i][k]) > std::abs(A[piv][k])) piv = i;
        }
        if (piv != k) std::swap(A[k], A[piv]);
        if (std::abs(A[k][k]) < kTol) continue; // singular column

        for (int i = k + 1; i < n; ++i) {
            double factor = A[i][k] / A[k][k];
            for (int j = k; j <= n; ++j) {
                A[i][j] -= factor * A[k][j];
            }
        }
    }

    // Back substitution
    std::vector<double> result(ncp, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double sum = A[i][n]; // RHS
        for (int j = i + 1; j < ncp; ++j) {
            sum -= A[i][j] * result[j];
        }
        if (std::abs(A[i][i]) > kTol) {
            result[i] = sum / A[i][i];
        } else {
            result[i] = 0.0; // underdetermined; pick 0
        }
    }

    return result;
}

// ============================================================================
// Linear interpolation of samples (ground truth between samples)
// ============================================================================

double linearInterpSamples(const std::vector<double>& u,
                           const std::vector<double>& q, double uq) {
    if (u.empty()) return 0.0;
    if (uq <= u.front()) return q.front();
    if (uq >= u.back()) return q.back();
    // Binary search
    auto it = std::lower_bound(u.begin(), u.end(), uq);
    int idx = static_cast<int>(it - u.begin());
    if (idx == 0) return q[0];
    double alpha = (uq - u[idx - 1]) / (u[idx] - u[idx - 1]);
    return q[idx - 1] * (1.0 - alpha) + q[idx] * alpha;
}

double linearInterpLimit(const std::vector<double>& u,
                         const std::vector<double>& lim, double uq) {
    return linearInterpSamples(u, lim, uq);
}

// ============================================================================
// Convex-hull constraint clamping
// ============================================================================

/// For each span of the B-spline, clamp the control points so the convex
/// hull lies below the minimum of the limit over that span.
void clampControlPointsToLimit(
    std::vector<double>& cp,
    const std::vector<double>& knots,
    int degree,
    const std::vector<double>& u,
    const std::vector<double>& limit,
    double safetyMargin,
    std::optional<double> lowerBound) {
    int ncp = static_cast<int>(cp.size());
    int nspans = ncp - degree; // number of spans for a clamped B-spline

    // For each span [knots[degree + spanIdx], knots[degree + spanIdx + 1])
    // the control points involved are cp[spanIdx] .. cp[spanIdx + degree]
    for (int spanIdx = 0; spanIdx < nspans; ++spanIdx) {
        double spanStart = knots[degree + spanIdx];
        double spanEnd = knots[degree + spanIdx + 1];
        if (spanEnd - spanStart < kTol) continue;

        // Find the minimum of the limit over this span by sampling.
        // We sample at the original data points that fall in this span,
        // plus the span endpoints.
        double minLim = std::numeric_limits<double>::infinity();
        auto checkLim = [&](double uq) {
            if (uq < 0.0) uq = 0.0;
            if (uq > 1.0) uq = 1.0;
            double l = linearInterpLimit(u, limit, uq);
            if (l < minLim) minLim = l;
        };
        checkLim(spanStart);
        checkLim(spanEnd);
        for (std::size_t i = 0; i < u.size(); ++i) {
            if (u[i] >= spanStart - kTol && u[i] <= spanEnd + kTol) {
                checkLim(u[i]);
            }
        }
        // Also sample a few interior points
        for (int k = 1; k < 4; ++k) {
            double uq = spanStart + (spanEnd - spanStart) * k / 4.0;
            checkLim(uq);
        }

        double clampValue = minLim - safetyMargin;

        // Clamp each control point in this span's hull
        for (int j = spanIdx; j <= spanIdx + degree && j < ncp; ++j) {
            if (cp[j] > clampValue) {
                cp[j] = clampValue;
            }
            if (lowerBound && cp[j] < *lowerBound) {
                cp[j] = *lowerBound;
            }
        }
    }

    // Also enforce lower bound on all control points (not just clamped spans)
    if (lowerBound) {
        for (int j = 0; j < ncp; ++j) {
            if (cp[j] < *lowerBound) cp[j] = *lowerBound;
        }
    }
}

} // anonymous namespace

// ============================================================================
// Public API
// ============================================================================

double evaluateBSpline(const std::vector<double>& controlPoints,
                       const std::vector<double>& knots,
                       int degree, double u) {
    if (controlPoints.empty()) return 0.0;
    int n = static_cast<int>(controlPoints.size()) - 1;
    // Clamp u to domain
    double uMin = knots[degree];
    double uMax = knots[n + 1];
    if (u <= uMin) return controlPoints[0];
    if (u >= uMax) return controlPoints[n];

    int span = findSpan(knots, degree, u);
    auto N = basisFunctions(span, u, degree, knots);
    double val = 0.0;
    for (int j = 0; j <= degree; ++j) {
        int idx = span - degree + j;
        if (idx >= 0 && idx < static_cast<int>(controlPoints.size())) {
            val += N[j] * controlPoints[idx];
        }
    }
    return val;
}

double evaluateBSplineDerivative(const std::vector<double>& controlPoints,
                                 const std::vector<double>& knots,
                                 int degree, double u) {
    if (controlPoints.empty() || degree < 1) return 0.0;
    int n = static_cast<int>(controlPoints.size()) - 1;
    double uMin = knots[degree];
    double uMax = knots[n + 1];
    if (u <= uMin) u = uMin;
    if (u >= uMax) u = uMax;

    int span = findSpan(knots, degree, u);
    auto ders = basisFunctionDerivatives(span, u, degree, knots);
    double val = 0.0;
    for (int j = 0; j <= degree; ++j) {
        int idx = span - degree + j;
        if (idx >= 0 && idx < static_cast<int>(controlPoints.size())) {
            val += ders[j] * controlPoints[idx];
        }
    }
    return val;
}

std::pair<std::vector<double>, std::vector<double>>
insertKnot(const std::vector<double>& controlPoints,
           const std::vector<double>& knots,
           int degree, double u) {
    int n = static_cast<int>(controlPoints.size()) - 1;
    int span = findSpan(knots, degree, u);

    // New knot vector: insert u at position span+1
    std::vector<double> newKnots(knots.size() + 1);
    for (int i = 0; i <= span; ++i) newKnots[i] = knots[i];
    newKnots[span + 1] = u;
    for (std::size_t i = span + 1; i < knots.size(); ++i)
        newKnots[i + 1] = knots[i];

    // New control points (Boehm's algorithm, Piegl & Tiller A5.1)
    std::vector<double> newCP(controlPoints.size() + 1);
    for (int i = 0; i <= span - degree; ++i) newCP[i] = controlPoints[i];
    for (int i = span; i <= n; ++i) newCP[i + 1] = controlPoints[i];
    double alpha;
    for (int i = span - degree + 1; i <= span; ++i) {
        alpha = (u - knots[i]) / (knots[i + degree] - knots[i]);
        newCP[i] = alpha * controlPoints[i] + (1.0 - alpha) * controlPoints[i - 1];
    }

    return {newCP, newKnots};
}

SplineFitResult fitSplineThroughSamples(
    const std::vector<double>& u,
    const std::vector<double>& q,
    const SplineFitterConfig& config) {

    if (u.size() != q.size()) {
        throw std::invalid_argument(
            "fitSplineThroughSamples: u and q must have the same size");
    }
    if (u.size() < 2) {
        throw std::invalid_argument(
            "fitSplineThroughSamples: need at least 2 samples");
    }
    for (std::size_t i = 1; i < u.size(); ++i) {
        if (u[i] <= u[i - 1] + kTol) {
            throw std::invalid_argument(
                "fitSplineThroughSamples: u must be strictly increasing");
        }
    }

    SplineFitResult result;
    result.degree = config.degree;

    // Handle the trivial case: 2 samples → degree-1 line (or degree-p with
    // reduced degree if p > 1 and only 2 samples)
    if (u.size() == 2) {
        result.degree = 1;
        result.controlPoints = {q[0], q[1]};
        result.knots = {0.0, 0.0, 1.0, 1.0};
        result.maxResidual = 0.0;
        result.maxInterSampleResidual = 0.0;
        result.withinEpsilon = true;
        result.achievedContinuity = 0;
        return result;
    }

    // Reduce degree if we don't have enough samples
    int degree = config.degree;
    int maxDegreeForSamples = static_cast<int>(u.size()) - 1;
    if (degree > maxDegreeForSamples) {
        degree = maxDegreeForSamples;
        if (degree < 1) degree = 1;
    }
    result.degree = degree;

    // Handle constant samples: all q equal → constant spline
    bool allEqual = true;
    for (std::size_t i = 1; i < q.size(); ++i) {
        if (std::abs(q[i] - q[0]) > kTol) { allEqual = false; break; }
    }
    if (allEqual) {
        result.degree = 1;
        result.controlPoints = {q[0], q[0]};
        result.knots = {0.0, 0.0, 1.0, 1.0};
        result.maxResidual = 0.0;
        result.maxInterSampleResidual = 0.0;
        result.withinEpsilon = true;
        result.achievedContinuity = 0;
        // Apply lower bound if needed
        if (config.lowerBound && result.controlPoints[0] < *config.lowerBound) {
            result.controlPoints[0] = *config.lowerBound;
            result.controlPoints[1] = *config.lowerBound;
        }
        return result;
    }

    // Step 1: Initial knot vector and interpolation
    auto knots = computeKnotVector(u, degree);
    auto cp = solveInterpolation(u, q, knots, degree);

    // If the initial fit already exceeds the CP cap, use a coarser knot
    // vector (fewer CPs) by subsampling the parameters.
    if (cp.size() > config.maxControlPoints) {
        // Use fewer interior knots: aim for maxControlPoints CPs
        int targetCPs = static_cast<int>(config.maxControlPoints);
        int targetDegree = degree;
        if (targetCPs <= targetDegree + 1) {
            targetDegree = std::max(1, targetCPs - 1);
            targetCPs = targetDegree + 1;
        }
        // Build a reduced knot vector with (targetCPs - targetDegree - 1)
        // interior knots, placed at equal parameter intervals
        int nInterior = targetCPs - targetDegree - 1;
        std::vector<double> reducedU;
        if (nInterior <= 0 || targetCPs >= static_cast<int>(u.size())) {
            // Can't reduce further; just use the original
        } else {
            // Select targetCPs samples uniformly from the original
            std::vector<double> selU, selQ;
            selU.push_back(u.front());
            selQ.push_back(q.front());
            for (int i = 1; i < targetCPs - 1; ++i) {
                double t = static_cast<double>(i) / (targetCPs - 1);
                int idx = static_cast<int>(t * (u.size() - 1));
                selU.push_back(u[idx]);
                selQ.push_back(q[idx]);
            }
            selU.push_back(u.back());
            selQ.push_back(q.back());
            // Ensure strictly increasing
            bool ok = true;
            for (std::size_t i = 1; i < selU.size(); ++i) {
                if (selU[i] <= selU[i-1] + kTol) { ok = false; break; }
            }
            if (ok) {
                knots = computeKnotVector(selU, targetDegree);
                cp = solveInterpolation(selU, selQ, knots, targetDegree);
                degree = targetDegree;
                result.degree = degree;
            }
        }
    }

    // Step 2: Adaptive refinement loop
    // The goal is to reduce the inter-sample residual |B(u) - q_interp(u)|
    // where q_interp is the linear interpolation of samples (ground truth
    // between samples for the TOPP-RA use case).
    //
    // Strategy: insert knots at the midpoints of spans with worst residual,
    // then re-solve with a smoothness-regularized least-squares that
    // maintains exact interpolation at the samples.
    std::size_t testGridSize = u.size() * config.refinementGridMultiplier;
    if (testGridSize < 20) testGridSize = 20;

    auto computeResiduals = [&]() {
        double maxRes = 0.0;
        double maxInterRes = 0.0;
        for (std::size_t i = 0; i < u.size(); ++i) {
            double val = evaluateBSpline(cp, knots, degree, u[i]);
            double res = std::abs(val - q[i]);
            if (res > maxRes) maxRes = res;
        }
        for (std::size_t k = 0; k <= testGridSize; ++k) {
            double uq = static_cast<double>(k) / testGridSize;
            double val = evaluateBSpline(cp, knots, degree, uq);
            double truth = linearInterpSamples(u, q, uq);
            double res = std::abs(val - truth);
            if (res > maxInterRes) maxInterRes = res;
        }
        return std::make_pair(maxRes, maxInterRes);
    };

    /// Solve the interpolation or regularized least-squares problem.
    /// For ncp == nsamp: exact interpolation via A*P = q.
    /// For ncp > nsamp: smoothness-regularized least-squares:
    ///   min ||A*P - q||² + λ||D*P||²  where D is 2nd-difference operator.
    ///   Solution: (A^T A + λ D^T D) P = A^T q
    auto reSolve = [&]() {
        int ncp = static_cast<int>(cp.size());
        int nsamp = static_cast<int>(u.size());

        // Build basis matrix A (nsamp × ncp)
        std::vector<std::vector<double>> A(nsamp, std::vector<double>(ncp, 0.0));
        for (int i = 0; i < nsamp; ++i) {
            int span = findSpan(knots, degree, u[i]);
            auto N = basisFunctions(span, u[i], degree, knots);
            for (int j = 0; j <= degree; ++j) {
                int col = span - degree + j;
                if (col >= 0 && col < ncp) A[i][col] = N[j];
            }
        }

        if (ncp == nsamp) {
            // Exact interpolation
            cp = solveInterpolation(u, q, knots, degree);
            return;
        }

        // Regularized least-squares: (A^T A + λ D^T D) P = A^T q
        // D is the 2nd-difference operator (ncp-2 × ncp)
        double lambda = 1e-6; // small regularization for smoothness

        // Build A^T A and A^T q
        std::vector<std::vector<double>> M(ncp, std::vector<double>(ncp, 0.0));
        std::vector<double> rhs(ncp, 0.0);
        for (int i = 0; i < nsamp; ++i) {
            for (int j = 0; j < ncp; ++j) {
                if (std::abs(A[i][j]) < kTol) continue;
                rhs[j] += A[i][j] * q[i];
                for (int k = j; k < ncp; ++k) {
                    M[j][k] += A[i][j] * A[i][k];
                    M[k][j] = M[j][k];
                }
            }
        }

        // Add λ D^T D (2nd differences: D[i] = P[i] - 2*P[i+1] + P[i+2])
        for (int i = 0; i + 2 < ncp; ++i) {
            // D row i: 1 at i, -2 at i+1, 1 at i+2
            // D^T D contribution:
            M[i][i]     += lambda * 1.0;
            M[i][i+1]   += lambda * (-2.0);
            M[i][i+2]   += lambda * 1.0;
            M[i+1][i]   += lambda * (-2.0);
            M[i+1][i+1] += lambda * 4.0;
            M[i+1][i+2] += lambda * (-2.0);
            M[i+2][i]   += lambda * 1.0;
            M[i+2][i+1] += lambda * (-2.0);
            M[i+2][i+2] += lambda * 1.0;
        }

        // Solve M * P = rhs via Gaussian elimination with partial pivoting
        for (int j = 0; j < ncp; ++j) M[j].push_back(rhs[j]);
        for (int col = 0; col < ncp; ++col) {
            int piv = col;
            for (int i = col + 1; i < ncp; ++i)
                if (std::abs(M[i][col]) > std::abs(M[piv][col])) piv = i;
            if (piv != col) std::swap(M[col], M[piv]);
            if (std::abs(M[col][col]) < kTol) continue;
            for (int i = col + 1; i < ncp; ++i) {
                double f = M[i][col] / M[col][col];
                for (int j = col; j <= ncp; ++j)
                    M[i][j] -= f * M[col][j];
            }
        }
        cp.assign(ncp, 0.0);
        for (int i = ncp - 1; i >= 0; --i) {
            double s = M[i][ncp];
            for (int j = i + 1; j < ncp; ++j) s -= M[i][j] * cp[j];
            if (std::abs(M[i][i]) > kTol) cp[i] = s / M[i][i];
        }
    };

    auto [maxRes, maxInterRes] = computeResiduals();

    // Track inserted knot locations to avoid clustering
    std::vector<double> insertedKnots;

    auto isNearInserted = [&](double uq, double minDist) {
        for (double k : insertedKnots) {
            if (std::abs(k - uq) < minDist) return true;
        }
        return false;
    };

    // Refine until within epsilon or cap hit.
    // Note: we only refine based on SAMPLE residual (|B(u_i) - q_i|).
    // The inter-sample residual is reported but not used as a refinement
    // criterion, because a smooth spline naturally deviates from linear
    // interpolation between samples — that's the desired behavior.
    int maxRefinementIterations = 30;
    for (int iter = 0; iter < maxRefinementIterations; ++iter) {
        if (maxRes <= config.epsilon) break;
        if (cp.size() >= config.maxControlPoints) {
            result.controlPointCapHit = true;
            break;
        }

        // Find the worst inter-sample residual location, avoiding already-inserted knots
        double worstU = 0.5;
        double worstRes = 0.0;
        double minDist = 1.0 / (config.maxControlPoints * 2); // min distance between knots
        for (std::size_t k = 0; k <= testGridSize; ++k) {
            double uq = static_cast<double>(k) / testGridSize;
            if (isNearInserted(uq, minDist)) continue;
            double val = evaluateBSpline(cp, knots, degree, uq);
            double truth = linearInterpSamples(u, q, uq);
            double res = std::abs(val - truth);
            if (res > worstRes) { worstRes = res; worstU = uq; }
        }

        if (worstRes <= config.epsilon) break;

        // Insert a knot at the worst location
        double insertU = std::clamp(worstU, 1e-6, 1.0 - 1e-6);
        auto [newCP, newKnots] = insertKnot(cp, knots, degree, insertU);
        cp = std::move(newCP);
        knots = std::move(newKnots);
        insertedKnots.push_back(insertU);

        // Re-solve with the new knot structure
        reSolve();

        auto [r1, r2] = computeResiduals();
        maxRes = r1;
        maxInterRes = r2;
    }

    // Step 3: Constraint clamping (if upper limit provided)
    if (config.upperLimit && config.upperLimit->size() == u.size()) {
        clampControlPointsToLimit(cp, knots, degree, u, *config.upperLimit,
                                  config.safetyMargin, config.lowerBound);
        result.constraintClamped = true;

        // After clamping, re-check interpolation. If samples are now missed,
        // insert knots near the missed samples and re-fit.
        for (int repairIter = 0; repairIter < 10; ++repairIter) {
            double worstSampleRes = 0.0;
            int worstSampleIdx = -1;
            for (std::size_t i = 0; i < u.size(); ++i) {
                double val = evaluateBSpline(cp, knots, degree, u[i]);
                double res = std::abs(val - q[i]);
                if (res > worstSampleRes) {
                    worstSampleRes = res;
                    worstSampleIdx = static_cast<int>(i);
                }
            }
            if (worstSampleRes <= config.epsilon || worstSampleIdx < 0) break;
            if (cp.size() >= config.maxControlPoints) {
                result.controlPointCapHit = true;
                break;
            }
            // Insert a knot near the missed sample (offset slightly to
            // avoid inserting exactly at a sample, which can be degenerate)
            double insertU = u[worstSampleIdx];
            // Offset toward the midpoint with the neighbor
            if (worstSampleIdx > 0) {
                insertU = 0.5 * (u[worstSampleIdx - 1] + u[worstSampleIdx]);
            } else if (worstSampleIdx + 1 < static_cast<int>(u.size())) {
                insertU = 0.5 * (u[worstSampleIdx] + u[worstSampleIdx + 1]);
            }
            insertU = std::clamp(insertU, 1e-6, 1.0 - 1e-6);
            auto [newCP, newKnots] = insertKnot(cp, knots, degree, insertU);
            cp = std::move(newCP);
            knots = std::move(newKnots);
            // Re-solve using the regularized approach
            reSolve();
            // Re-clamp
            clampControlPointsToLimit(cp, knots, degree, u, *config.upperLimit,
                                      config.safetyMargin, config.lowerBound);
        }
    } else if (config.lowerBound) {
        // No upper limit, just enforce lower bound
        for (auto& c : cp) {
            if (c < *config.lowerBound) c = *config.lowerBound;
        }
    }

    // Final residual computation
    auto [finalMaxRes, finalMaxInterRes] = computeResiduals();
    result.controlPoints = std::move(cp);
    result.knots = std::move(knots);
    result.maxResidual = finalMaxRes;
    result.maxInterSampleResidual = finalMaxInterRes;
    result.withinEpsilon = (finalMaxRes <= config.epsilon);
    result.achievedContinuity = degree - 1;

    return result;
}

} // namespace tether::motion::profile_renurbs
