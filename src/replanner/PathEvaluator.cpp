/**
 * @file PathEvaluator.cpp
 * @brief Implementation of quantitative and qualitative path evaluators.
 */

#include "tether/motion_replanner/PathEvaluator.hpp"
#include "tether/motion_replanner/PathRelativeFFT.hpp" // SpectralEvaluation
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace tether::motion::replanner {

namespace {

//=============================================================================
// Utility functions
//=============================================================================

/// Linear interpolation between two values.
double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

/// Trapezoidal integration of y(x) given non-uniform x.
double trapezoidalIntegrate(const std::vector<double>& x,
                            const std::vector<double>& y) {
    if (x.size() < 2) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 1; i < x.size(); ++i) {
        double dx = x[i] - x[i - 1];
        sum += 0.5 * (y[i] + y[i - 1]) * dx;
    }
    return sum;
}

/// Trapezoidal integration of y(x) with a weighting function w(x).
double trapezoidalIntegrateWeighted(const std::vector<double>& x,
                                    const std::vector<double>& y,
                                    const std::vector<double>& w) {
    if (x.size() < 2) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 1; i < x.size(); ++i) {
        double dx = x[i] - x[i - 1];
        double yAvg = 0.5 * (y[i] + y[i - 1]);
        double wAvg = 0.5 * (w[i] + w[i - 1]);
        sum += yAvg * wAvg * dx;
    }
    return sum;
}

/// Compute the mean of a vector.
double mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

/// Compute the RMS of a vector.
double rms(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double sumSq = 0.0;
    for (double x : v) sumSq += x * x;
    return std::sqrt(sumSq / static_cast<double>(v.size()));
}

/// Compute the max absolute value of a vector.
double maxAbs(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double m = 0.0;
    for (double x : v) m = std::max(m, std::abs(x));
    return m;
}

/// Compute ErrorStatistics from a vector of error values.
MotionReplanner::ErrorStatistics computeErrorStats(const std::vector<double>& errors) {
    MotionReplanner::ErrorStatistics stats;
    stats.sampleCount = errors.size();
    if (errors.empty()) return stats;

    double sum = 0.0, sumSq = 0.0, sumLog = 0.0;
    double minVal = std::numeric_limits<double>::max();
    double maxVal = std::numeric_limits<double>::lowest();
    std::size_t posCount = 0;

    for (double e : errors) {
        double ae = std::abs(e);
        sum += ae;
        sumSq += ae * ae;
        if (ae > 0) { sumLog += std::log(ae); ++posCount; }
        minVal = std::min(minVal, ae);
        maxVal = std::max(maxVal, ae);
    }

    double n = static_cast<double>(errors.size());
    stats.minError = minVal;
    stats.maxError = maxVal;
    stats.meanError = sum / n;
    stats.rmsError = std::sqrt(sumSq / n);
    stats.geometricMean = (posCount > 0)
        ? std::exp(sumLog / static_cast<double>(posCount)) : 0.0;

    // Variance and std dev
    double variance = 0.0;
    for (double e : errors) {
        double ae = std::abs(e);
        double d = ae - stats.meanError;
        variance += d * d;
    }
    variance /= n;
    stats.stdDev = std::sqrt(variance);

    // Percentiles (need sorted copy)
    std::vector<double> sorted = errors;
    std::transform(sorted.begin(), sorted.end(), sorted.begin(),
                   [](double v) { return std::abs(v); });
    std::sort(sorted.begin(), sorted.end());

    auto percentile = [&](double p) -> double {
        if (sorted.empty()) return 0.0;
        double idx = p * (static_cast<double>(sorted.size()) - 1.0);
        auto lo = static_cast<std::size_t>(std::floor(idx));
        auto hi = static_cast<std::size_t>(std::ceil(idx));
        if (lo >= sorted.size()) lo = sorted.size() - 1;
        if (hi >= sorted.size()) hi = sorted.size() - 1;
        double frac = idx - static_cast<double>(lo);
        return lerp(sorted[lo], sorted[hi], frac);
    };

    stats.p95Error = percentile(0.95);
    stats.p99Error = percentile(0.99);

    return stats;
}

/// Discrete Frechet distance between two point sequences (O(n*m) DP).
/// Each point is a (x,y) pair; we use the first 2 active axes.
double discreteFrechet(const std::vector<double>& p1, const std::vector<double>& p2) {
    // p1, p2 are interleaved 3D points: [x0,y0,z0, x1,y1,z1, ...]
    auto n1 = p1.size() / 3;
    auto n2 = p2.size() / 3;
    if (n1 == 0 || n2 == 0) return 0.0;

    // dp[i][j] = Frechet distance between p1[0..i] and p2[0..j]
    std::vector<double> dp(n1 * n2, 0.0);

    auto dist = [&](std::size_t i, std::size_t j) -> double {
        double dx = p1[i * 3] - p2[j * 3];
        double dy = p1[i * 3 + 1] - p2[j * 3 + 1];
        double dz = p1[i * 3 + 2] - p2[j * 3 + 2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    dp[0] = dist(0, 0);
    for (std::size_t i = 1; i < n1; ++i)
        dp[i * n2] = std::max(dp[(i - 1) * n2], dist(i, 0));
    for (std::size_t j = 1; j < n2; ++j)
        dp[j] = std::max(dp[j - 1], dist(0, j));

    for (std::size_t i = 1; i < n1; ++i) {
        for (std::size_t j = 1; j < n2; ++j) {
            double d = dist(i, j);
            double m = std::min({dp[(i - 1) * n2 + j],
                                 dp[i * n2 + (j - 1)],
                                 dp[(i - 1) * n2 + (j - 1)]});
            dp[i * n2 + j] = std::max(m, d);
        }
    }
    return dp[(n1 - 1) * n2 + (n2 - 1)];
}

/// Dynamic time warping distance between two point sequences (O(n*m) DP).
/// Returns the minimum total alignment cost normalized by the longer path.
double dynamicTimeWarping(const std::vector<double>& p1, const std::vector<double>& p2) {
    auto n1 = p1.size() / 3;
    auto n2 = p2.size() / 3;
    if (n1 == 0 || n2 == 0) return 0.0;

    auto dist = [&](std::size_t i, std::size_t j) -> double {
        double dx = p1[i * 3] - p2[j * 3];
        double dy = p1[i * 3 + 1] - p2[j * 3 + 1];
        double dz = p1[i * 3 + 2] - p2[j * 3 + 2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    std::vector<double> dp(n1 * n2, 0.0);
    dp[0] = dist(0, 0);
    for (std::size_t i = 1; i < n1; ++i)
        dp[i * n2] = dp[(i - 1) * n2] + dist(i, 0);
    for (std::size_t j = 1; j < n2; ++j)
        dp[j] = dp[j - 1] + dist(0, j);

    for (std::size_t i = 1; i < n1; ++i) {
        for (std::size_t j = 1; j < n2; ++j) {
            double d = dist(i, j);
            double m = std::min({dp[(i - 1) * n2 + j],
                                 dp[i * n2 + (j - 1)],
                                 dp[(i - 1) * n2 + (j - 1)]});
            dp[i * n2 + j] = d + m;
        }
    }

    // Normalize by the length of the warping path (approx n1 + n2)
    return dp[(n1 - 1) * n2 + (n2 - 1)] / static_cast<double>(n1 + n2);
}

/// Cross-correlation: find the lag that maximizes correlation between
/// two signals. Returns (peakCorrelation, lagInSamples).
std::pair<double, std::size_t> crossCorrelate(
    const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) return {0.0, 0};

    double meanA = mean(a);
    double meanB = mean(b);

    double stdA = 0.0, stdB = 0.0;
    for (double v : a) stdA += (v - meanA) * (v - meanA);
    for (double v : b) stdB += (v - meanB) * (v - meanB);
    stdA = std::sqrt(stdA / static_cast<double>(a.size()));
    stdB = std::sqrt(stdB / static_cast<double>(b.size()));
    if (stdA < 1e-15 || stdB < 1e-15) return {0.0, 0};

    double bestCorr = -1.0;
    std::size_t bestLag = 0;
    std::size_t maxLag = std::min(a.size(), b.size()) / 2;

    for (std::size_t lag = 0; lag <= maxLag; ++lag) {
        double corr = 0.0;
        std::size_t count = 0;
        for (std::size_t i = lag; i < std::min(a.size(), b.size()); ++i) {
            corr += (a[i] - meanA) * (b[i - lag] - meanB);
            ++count;
        }
        if (count > 0) {
            corr /= static_cast<double>(count) * stdA * stdB;
            if (corr > bestCorr) {
                bestCorr = corr;
                bestLag = lag;
            }
        }
    }
    return {bestCorr, bestLag};
}

} // anonymous namespace

//=============================================================================
// PathEvaluator implementation
//=============================================================================

PathEvaluator::PathEvaluator(EvaluatorConfig config)
    : config_(config), grader_(config) {}

std::vector<int> PathEvaluator::detectActiveAxes(
    const std::vector<GCodeExport::TrajectorySample>& desired) const {
    if (!config_.activeAxes.empty()) return config_.activeAxes;

    // Auto-detect: find axes that vary across the trajectory.
    std::array<double, 9> minPos{}, maxPos{};
    for (double& v : minPos) v = std::numeric_limits<double>::max();
    for (double& v : maxPos) v = std::numeric_limits<double>::lowest();

    for (const auto& s : desired) {
        for (int i = 0; i < 9; ++i) {
            minPos[i] = std::min(minPos[i], s.position[i]);
            maxPos[i] = std::max(maxPos[i], s.position[i]);
        }
    }

    std::vector<int> active;
    for (int i = 0; i < 9; ++i) {
        if (maxPos[i] - minPos[i] > 1e-9) active.push_back(i);
    }
    if (active.empty()) active = {0}; // fallback: X axis
    return active;
}

PathEvaluator::PerSampleErrors PathEvaluator::computeErrors(
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual,
    const PiecewiseNurbsPath& path) const {

    PerSampleErrors errs;
    auto n = std::min(desired.size(), actual.size());
    if (n == 0) return errs;

    std::size_t dim = path.dim();
    auto activeAxes = detectActiveAxes(desired);
    // Ensure we use at most 'dim' axes (matching the path)
    std::vector<int> axes;
    for (int i = 0; i < static_cast<int>(activeAxes.size()) && i < static_cast<int>(dim); ++i) {
        axes.push_back(activeAxes[i]);
    }
    if (axes.size() < dim) {
        // Pad with X/Y/Z as needed
        for (int i = static_cast<int>(axes.size()); i < static_cast<int>(dim); ++i) {
            axes.push_back(std::min(i, 2));
        }
    }

    errs.arcLengths.reserve(n);
    errs.times.reserve(n);
    errs.contour.reserve(n);
    errs.lag.reserve(n);
    errs.combined.reserve(n);
    errs.actualPos3d.reserve(n * 3);
    errs.desiredPos3d.reserve(n * 3);
    errs.actualVel.reserve(n);
    errs.desiredVel.reserve(n);
    errs.actualAccel.reserve(n);
    errs.desiredAccel.reserve(n);
    errs.actualJerk.reserve(n);
    errs.desiredCurv.reserve(n);
    errs.actualCurv.reserve(n);

    // Compute the maximum pathPosition to rescale to NURBS arc length
    double maxPathPos = 0.0;
    for (const auto& d : desired) {
        maxPathPos = std::max(maxPathPos, d.pathPosition);
    }
    double pathTotalLen = path.totalLength();
    // Rescale factor: maps sample pathPosition to NURBS arc length
    double sScale = (maxPathPos > 1e-12) ? pathTotalLen / maxPathPos : 1.0;

    for (std::size_t i = 0; i < n; ++i) {
        const auto& des = desired[i];
        const auto& act = actual[i];

        double s = des.pathPosition; // Original arc length (for reporting)
        double sPath = s * sScale;   // Scaled to NURBS path arc length
        double t = des.time;
        errs.arcLengths.push_back(s);
        errs.times.push_back(t);

        // Build RVec for actual position (active axes only)
        RVec actualPos = RVec::zero(dim);
        for (std::size_t d = 0; d < dim; ++d) {
            actualPos[d] = act.position[axes[d]];
        }

        // Compute certified or tangent-projected contour error
        if (config_.useCertifiedContourError) {
            try {
                CertifiedContourError ce = computeCertifiedContourError(
                    path, actualPos, sPath);
                errs.contour.push_back(ce.contourError);
                errs.lag.push_back(ce.lagError);
            } catch (...) {
                // Fallback: Euclidean distance to desired position
                RVec desiredPos = path.evaluatePosition(sPath);
                errs.contour.push_back(actualPos.distanceTo(desiredPos));
                errs.lag.push_back(0.0);
            }
        } else {
            // Tangent-projection (cheap method)
            RVec desiredPos = path.evaluatePosition(sPath);
            double combined = actualPos.distanceTo(desiredPos);
            // Decompose using desired velocity tangent
            double vx = des.velocity[axes[0]];
            double vy = (dim > 1) ? des.velocity[axes[1]] : 0.0;
            double vz = (dim > 2) ? des.velocity[axes[2]] : 0.0;
            double vmag = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (vmag > 1e-9) {
                double tx = vx / vmag, ty = vy / vmag, tz = vz / vmag;
                double ex = act.position[axes[0]] - des.position[axes[0]];
                double ey = (dim > 1) ? act.position[axes[1]] - des.position[axes[1]] : 0.0;
                double ez = (dim > 2) ? act.position[axes[2]] - des.position[axes[2]] : 0.0;
                double lag = ex * tx + ey * ty + ez * tz;
                double lagX = lag * tx, lagY = lag * ty, lagZ = lag * tz;
                double contX = ex - lagX, contY = ey - lagY, contZ = ez - lagZ;
                double cont = std::sqrt(contX * contX + contY * contY + contZ * contZ);
                errs.contour.push_back(cont);
                errs.lag.push_back(lag);
            } else {
                errs.contour.push_back(combined);
                errs.lag.push_back(0.0);
            }
        }

        // Combined 3D error (always Euclidean to desired position)
        double dx = act.position[0] - des.position[0];
        double dy = act.position[1] - des.position[1];
        double dz = act.position[2] - des.position[2];
        errs.combined.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));

        // 3D positions (for shape distances)
        errs.actualPos3d.push_back(act.position[0]);
        errs.actualPos3d.push_back(act.position[1]);
        errs.actualPos3d.push_back(act.position[2]);
        errs.desiredPos3d.push_back(des.position[0]);
        errs.desiredPos3d.push_back(des.position[1]);
        errs.desiredPos3d.push_back(des.position[2]);

        // Velocity magnitudes
        double dvel = std::sqrt(des.velocity[0] * des.velocity[0] +
                                des.velocity[1] * des.velocity[1] +
                                des.velocity[2] * des.velocity[2]);
        double avel = std::sqrt(act.velocity[0] * act.velocity[0] +
                                act.velocity[1] * act.velocity[1] +
                                act.velocity[2] * act.velocity[2]);
        errs.desiredVel.push_back(dvel);
        errs.actualVel.push_back(avel);

        // Acceleration magnitudes
        double dacc = std::sqrt(des.acceleration[0] * des.acceleration[0] +
                                des.acceleration[1] * des.acceleration[1] +
                                des.acceleration[2] * des.acceleration[2]);
        double aacc = std::sqrt(act.acceleration[0] * act.acceleration[0] +
                                act.acceleration[1] * act.acceleration[1] +
                                act.acceleration[2] * act.acceleration[2]);
        errs.desiredAccel.push_back(dacc);
        errs.actualAccel.push_back(aacc);

        // Jerk (finite difference of acceleration)
        if (i > 0) {
            double dt = (act.time - actual[i - 1].time);
            if (dt > 1e-12) {
                double jx = (act.acceleration[0] - actual[i - 1].acceleration[0]) / dt;
                double jy = (act.acceleration[1] - actual[i - 1].acceleration[1]) / dt;
                double jz = (act.acceleration[2] - actual[i - 1].acceleration[2]) / dt;
                errs.actualJerk.push_back(std::sqrt(jx * jx + jy * jy + jz * jz));
            } else {
                errs.actualJerk.push_back(0.0);
            }
        } else {
            errs.actualJerk.push_back(0.0);
        }

        // Curvature
        errs.desiredCurv.push_back(des.curvature);
        // Actual curvature: estimate from 3-point circle through actual positions
        if (i > 0 && i < n - 1) {
            double x0 = actual[i - 1].position[0], y0 = actual[i - 1].position[1];
            double x1 = actual[i].position[0], y1 = actual[i].position[1];
            double x2 = actual[i + 1].position[0], y2 = actual[i + 1].position[1];
            double d01 = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
            double d12 = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            double d02 = std::sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0));
            if (d01 > 1e-9 && d12 > 1e-9 && d02 > 1e-9) {
                // Circumradius R = (a*b*c) / (4*Area)
                double s = (d01 + d12 + d02) / 2.0;
                double areaSq = s * (s - d01) * (s - d12) * (s - d02);
                if (areaSq > 1e-20) {
                    double R = (d01 * d12 * d02) / (4.0 * std::sqrt(areaSq));
                    if (R > 1e-9) errs.actualCurv.push_back(1.0 / R);
                    else errs.actualCurv.push_back(0.0);
                } else {
                    errs.actualCurv.push_back(0.0);
                }
            } else {
                errs.actualCurv.push_back(0.0);
            }
        } else {
            errs.actualCurv.push_back(0.0);
        }
    }

    return errs;
}

IntegralErrorMetrics PathEvaluator::computeIntegrals(
    const PerSampleErrors& errs) const {

    IntegralErrorMetrics m;

    // Spatial integrals (over arc length)
    std::vector<double> absContour(errs.contour.size());
    std::vector<double> sqContour(errs.contour.size());
    std::vector<double> absLag(errs.lag.size());
    std::vector<double> sqLag(errs.lag.size());

    for (std::size_t i = 0; i < errs.contour.size(); ++i) {
        absContour[i] = std::abs(errs.contour[i]);
        sqContour[i] = errs.contour[i] * errs.contour[i];
        absLag[i] = std::abs(errs.lag[i]);
        sqLag[i] = errs.lag[i] * errs.lag[i];
    }

    m.iae_s = trapezoidalIntegrate(errs.arcLengths, absContour);
    m.ise_s = trapezoidalIntegrate(errs.arcLengths, sqContour);
    m.iae_lag = trapezoidalIntegrate(errs.arcLengths, absLag);
    m.ise_lag = trapezoidalIntegrate(errs.arcLengths, sqLag);

    // Temporal integrals (time-weighted)
    std::vector<double> absCombined(errs.combined.size());
    std::vector<double> sqCombined(errs.combined.size());
    for (std::size_t i = 0; i < errs.combined.size(); ++i) {
        absCombined[i] = std::abs(errs.combined[i]);
        sqCombined[i] = errs.combined[i] * errs.combined[i];
    }

    m.itae_t = trapezoidalIntegrateWeighted(errs.times, absCombined, errs.times);
    m.itse_t = trapezoidalIntegrateWeighted(errs.times, sqCombined, errs.times);

    return m;
}

NormMetrics PathEvaluator::computeNorms(const PerSampleErrors& errs) const {
    NormMetrics m;
    double L = 0.0;
    if (errs.arcLengths.size() >= 2) {
        L = errs.arcLengths.back() - errs.arcLengths.front();
    }
    if (L < 1e-15) L = 1.0; // avoid division by zero

    // L1 = ∫|e|ds, L2 = sqrt(∫e²ds), L∞ = max|e|
    std::vector<double> absC(errs.contour.size()), sqC(errs.contour.size());
    std::vector<double> absL(errs.lag.size()), sqL(errs.lag.size());
    std::vector<double> absComb(errs.combined.size()), sqComb(errs.combined.size());

    for (std::size_t i = 0; i < errs.contour.size(); ++i) {
        absC[i] = std::abs(errs.contour[i]);
        sqC[i] = errs.contour[i] * errs.contour[i];
        absL[i] = std::abs(errs.lag[i]);
        sqL[i] = errs.lag[i] * errs.lag[i];
    }
    for (std::size_t i = 0; i < errs.combined.size(); ++i) {
        absComb[i] = std::abs(errs.combined[i]);
        sqComb[i] = errs.combined[i] * errs.combined[i];
    }

    m.l1_contour = trapezoidalIntegrate(errs.arcLengths, absC);
    m.l2_contour = std::sqrt(trapezoidalIntegrate(errs.arcLengths, sqC));
    m.linf_contour = maxAbs(errs.contour);
    m.l1_lag = trapezoidalIntegrate(errs.arcLengths, absL);
    m.l2_lag = std::sqrt(trapezoidalIntegrate(errs.arcLengths, sqL));
    m.linf_lag = maxAbs(errs.lag);
    m.l1_combined = trapezoidalIntegrate(errs.arcLengths, absComb);
    m.l2_combined = std::sqrt(trapezoidalIntegrate(errs.arcLengths, sqComb));
    m.linf_combined = maxAbs(errs.combined);

    return m;
}

ShapeDistanceMetrics PathEvaluator::computeShapeDistances(
    const PerSampleErrors& errs,
    const PiecewiseNurbsPath& path) const {

    ShapeDistanceMetrics m;

    // Downsample for O(n²) algorithms
    std::size_t step = std::max(std::size_t{1}, config_.shapeDistanceDownsample);

    // Build downsampled actual 3D point array
    std::vector<double> actualPts;
    std::size_t n3d = errs.actualPos3d.size() / 3;
    for (std::size_t i = 0; i < n3d; i += step) {
        actualPts.push_back(errs.actualPos3d[i * 3]);
        actualPts.push_back(errs.actualPos3d[i * 3 + 1]);
        actualPts.push_back(errs.actualPos3d[i * 3 + 2]);
    }

    // Build downsampled desired 3D point array from path evaluation
    std::vector<double> desiredPts;
    double totalLen = path.totalLength();
    auto numActual = actualPts.size() / 3;
    for (std::size_t i = 0; i < numActual; ++i) {
        double s = (numActual > 1)
            ? totalLen * static_cast<double>(i) / static_cast<double>(numActual - 1)
            : 0.0;
        RVec p = path.evaluatePosition(s);
        // Always output 3D (pad with 0 if dim < 3)
        desiredPts.push_back(p.dim() > 0 ? p.unchecked(0) : 0.0);
        desiredPts.push_back(p.dim() > 1 ? p.unchecked(1) : 0.0);
        desiredPts.push_back(p.dim() > 2 ? p.unchecked(2) : 0.0);
    }

    // Hausdorff: max over actual of min distance to desired
    m.hausdorff = 0.0;
    for (std::size_t i = 0; i < actualPts.size() / 3; ++i) {
        double minDist = std::numeric_limits<double>::max();
        for (std::size_t j = 0; j < desiredPts.size() / 3; ++j) {
            double dx = actualPts[i * 3] - desiredPts[j * 3];
            double dy = actualPts[i * 3 + 1] - desiredPts[j * 3 + 1];
            double dz = actualPts[i * 3 + 2] - desiredPts[j * 3 + 2];
            double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            minDist = std::min(minDist, d);
        }
        m.hausdorff = std::max(m.hausdorff, minDist);
    }

    // Frechet and DTW
    m.frechet = discreteFrechet(actualPts, desiredPts);
    m.dtw = dynamicTimeWarping(actualPts, desiredPts);

    // Path length ratio
    double actualLen = 0.0;
    for (std::size_t i = 1; i < errs.actualPos3d.size() / 3; ++i) {
        double dx = errs.actualPos3d[i * 3] - errs.actualPos3d[(i - 1) * 3];
        double dy = errs.actualPos3d[i * 3 + 1] - errs.actualPos3d[(i - 1) * 3 + 1];
        double dz = errs.actualPos3d[i * 3 + 2] - errs.actualPos3d[(i - 1) * 3 + 2];
        actualLen += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    m.pathLengthRatio = (totalLen > 1e-12) ? actualLen / totalLen : 1.0;

    // Curvature errors
    if (!errs.desiredCurv.empty()) {
        double maxCurvErr = 0.0, sumSq = 0.0;
        std::size_t count = 0;
        for (std::size_t i = 0; i < std::min(errs.desiredCurv.size(), errs.actualCurv.size()); ++i) {
            double d = std::abs(errs.actualCurv[i] - errs.desiredCurv[i]);
            maxCurvErr = std::max(maxCurvErr, d);
            sumSq += d * d;
            ++count;
        }
        m.curvatureErrorMax = maxCurvErr;
        m.curvatureErrorRms = (count > 0) ? std::sqrt(sumSq / static_cast<double>(count)) : 0.0;
    }

    return m;
}

KinematicTrackingMetrics PathEvaluator::computeKinematic(
    const PerSampleErrors& errs) const {

    KinematicTrackingMetrics m;
    auto n = std::min(errs.actualVel.size(), errs.desiredVel.size());

    if (n == 0) return m;

    std::vector<double> velErr(n), accErr(n);
    for (std::size_t i = 0; i < n; ++i) {
        velErr[i] = std::abs(errs.actualVel[i] - errs.desiredVel[i]);
        accErr[i] = std::abs(errs.actualAccel[i] - errs.desiredAccel[i]);
    }

    m.velocityTrackingRms = rms(velErr);
    m.velocityTrackingMax = maxAbs(velErr);
    m.accelTrackingRms = rms(accErr);
    m.accelTrackingMax = maxAbs(accErr);
    m.jerkActualMax = maxAbs(errs.actualJerk);
    m.jerkActualRms = rms(errs.actualJerk);

    // Smoothness index: ∫jerk² dt
    std::vector<double> jerkSq(errs.actualJerk.size());
    for (std::size_t i = 0; i < errs.actualJerk.size(); ++i) {
        jerkSq[i] = errs.actualJerk[i] * errs.actualJerk[i];
    }
    m.smoothnessIndex = trapezoidalIntegrate(errs.times, jerkSq);

    return m;
}

SurfaceFinishMetrics PathEvaluator::computeSurfaceFinish(
    const PerSampleErrors& errs) const {

    SurfaceFinishMetrics m;
    if (errs.contour.empty()) return m;

    double L = 0.0;
    if (errs.arcLengths.size() >= 2) {
        L = errs.arcLengths.back() - errs.arcLengths.front();
    }
    if (L < 1e-15) L = 1.0;

    // Ra = (1/L) ∫|e_contour| ds
    std::vector<double> absC(errs.contour.size());
    for (std::size_t i = 0; i < errs.contour.size(); ++i) {
        absC[i] = std::abs(errs.contour[i]);
    }
    double integral = trapezoidalIntegrate(errs.arcLengths, absC);
    m.ra = (integral / L) * 1000.0; // mm → µm

    // Rq = sqrt((1/L) ∫e_contour² ds)
    std::vector<double> sqC(errs.contour.size());
    for (std::size_t i = 0; i < errs.contour.size(); ++i) {
        sqC[i] = errs.contour[i] * errs.contour[i];
    }
    double integralSq = trapezoidalIntegrate(errs.arcLengths, sqC);
    m.rq = std::sqrt(integralSq / L) * 1000.0; // mm → µm

    // Rz = max - min of detrended contour error (in µm)
    double contourMean = mean(errs.contour);
    double minDet = std::numeric_limits<double>::max();
    double maxDet = std::numeric_limits<double>::lowest();
    std::size_t zeroCrossings = 0;
    bool prevPositive = false;
    bool hasPrev = false;
    for (double e : errs.contour) {
        double det = e - contourMean;
        minDet = std::min(minDet, det);
        maxDet = std::max(maxDet, det);
        bool isPositive = det > 0;
        if (hasPrev && isPositive != prevPositive) ++zeroCrossings;
        prevPositive = isPositive;
        hasPrev = true;
    }
    m.rz = (maxDet - minDet) * 1000.0; // mm → µm
    m.peakCount = zeroCrossings;

    return m;
}

FollowingErrorMetrics PathEvaluator::computeFollowing(
    const PerSampleErrors& errs) const {

    FollowingErrorMetrics m;
    if (errs.lag.empty()) return m;

    std::vector<double> absLag(errs.lag.size());
    for (std::size_t i = 0; i < errs.lag.size(); ++i) {
        absLag[i] = std::abs(errs.lag[i]);
    }

    m.maxFollowingError = maxAbs(errs.lag);
    m.meanFollowingError = mean(absLag);

    // Settling distance: find the worst corner region and measure how long
    // it takes to settle within tolerance. For simplicity, find the index
    // of max |lag| and scan forward until within tolerance.
    if (!absLag.empty()) {
        auto maxIdx = std::distance(absLag.begin(),
            std::max_element(absLag.begin(), absLag.end()));
        double settleStart = errs.arcLengths[static_cast<std::size_t>(maxIdx)];
        for (std::size_t i = static_cast<std::size_t>(maxIdx); i < absLag.size(); ++i) {
            if (absLag[i] < config_.settlingTolerance) {
                m.settlingDistance = errs.arcLengths[i] - settleStart;
                break;
            }
        }
    }

    // Cross-correlation of velocity profiles
    auto [peak, lagSamples] = crossCorrelate(errs.actualVel, errs.desiredVel);
    m.crossCorrelationPeak = peak;
    if (lagSamples > 0 && lagSamples < errs.times.size()) {
        m.crossCorrelationLag = errs.times[lagSamples] - errs.times[0];
    }

    return m;
}

QuantitativeEvaluation PathEvaluator::evaluateQuantitative(
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual) const {

    QuantitativeEvaluation result;

    if (desired.empty() || actual.empty()) return result;

    // Build the desired path as a PiecewiseNurbsPath
    PiecewiseNurbsPath path = convertTrajectory(desired);

    // Compute per-sample errors
    PerSampleErrors errs = computeErrors(desired, actual, path);

    result.sampleCount = errs.contour.size();
    result.pathLength = path.totalLength();
    if (!errs.times.empty()) {
        result.duration = errs.times.back() - errs.times.front();
    }

    // Compute all metric groups
    result.integrals = computeIntegrals(errs);
    result.norms = computeNorms(errs);
    result.shape = computeShapeDistances(errs, path);
    result.kinematic = computeKinematic(errs);
    result.surface = computeSurfaceFinish(errs);
    result.following = computeFollowing(errs);

    // Statistical summaries
    result.contourStats = computeErrorStats(errs.contour);
    result.lagStats = computeErrorStats(errs.lag);
    result.combinedStats = computeErrorStats(errs.combined);

    return result;
}

//=============================================================================
// Grading helpers
//=============================================================================

Grade PathEvaluator::gradeFromThresholds(double value,
                                         double threshA, double threshB,
                                         double threshC, double threshD) const {
    return grader_.gradeFromThresholds(value, threshA, threshB, threshC, threshD);
}

double PathEvaluator::gradeToScore(Grade g) const {
    return grader_.gradeToScore(g);
}

QualitativeAssessment PathEvaluator::makeAssessment(
    Grade g, const std::string& aspect,
    double value, const std::string& unit) const {
    return grader_.makeAssessment(g, aspect, value, unit);
}

QualitativeEvaluation PathEvaluator::evaluateQualitative(
    const QuantitativeEvaluation& quant,
    const SpectralEvaluation* spectral) const {
    return grader_.evaluateQualitative(quant, spectral);
}

} // namespace tether::motion::replanner
