/**
 * @file PathRelativeFFT.cpp
 * @brief Implementation of path-relative FFT-based oscillation detection.
 */

#include "tether/motion_replanner/PathRelativeFFT.hpp"
#include "tether/motion_replanner/FFTProcessor.hpp"
#include "tether/motion_replanner/CertifiedCornerDetection.hpp"
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

/// Next power of 2 >= n.
std::size_t nextPow2(std::size_t n) {
    if (n == 0) return 1;
    std::size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

/// Sign function.
double sign(double x) {
    if (x > 0) return 1.0;
    if (x < 0) return -1.0;
    return 0.0;
}

/// Pearson correlation coefficient between two vectors.
double pearsonCorrelation(const std::vector<double>& a, const std::vector<double>& b) {
    auto n = std::min(a.size(), b.size());
    if (n == 0) return 0.0;
    double ma = 0.0, mb = 0.0;
    for (std::size_t i = 0; i < n; ++i) { ma += a[i]; mb += b[i]; }
    ma /= static_cast<double>(n);
    mb /= static_cast<double>(n);
    double sxy = 0.0, sxx = 0.0, syy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        double da = a[i] - ma;
        double db = b[i] - mb;
        sxy += da * db;
        sxx += da * da;
        syy += db * db;
    }
    double denom = std::sqrt(sxx * syy);
    if (denom < 1e-15) return 0.0;
    return sxy / denom;
}

} // anonymous namespace

//=============================================================================
// PathRelativeFFT implementation
//=============================================================================

PathRelativeFFT::PathRelativeFFT(FFTConfig config)
    : config_(std::move(config)) {}

std::vector<int> PathRelativeFFT::detectActiveAxes(
    const std::vector<GCodeExport::TrajectorySample>& desired) const {
    if (!config_.activeAxes.empty()) return config_.activeAxes;

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
    if (active.empty()) active = {0};
    return active;
}

double PathRelativeFFT::linearInterpolate(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    double xq) const {

    if (xs.empty()) return 0.0;
    if (xs.size() == 1) return ys[0];

    // Clamp to range
    if (xq <= xs.front()) return ys.front();
    if (xq >= xs.back()) return ys.back();

    // Binary search for the interval
    auto it = std::upper_bound(xs.begin(), xs.end(), xq);
    std::size_t idx = static_cast<std::size_t>(it - xs.begin());
    if (idx == 0) return ys[0];
    --idx;

    double t = (xq - xs[idx]) / (xs[idx + 1] - xs[idx]);
    return ys[idx] + t * (ys[idx + 1] - ys[idx]);
}

double PathRelativeFFT::pchipInterpolate(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    double xq) const {

    if (xs.empty()) return 0.0;
    if (xs.size() == 1) return ys[0];
    if (xq <= xs.front()) return ys.front();
    if (xq >= xs.back()) return ys.back();

    // Find interval
    auto it = std::upper_bound(xs.begin(), xs.end(), xq);
    std::size_t i = static_cast<std::size_t>(it - xs.begin());
    if (i == 0) return ys[0];
    --i;
    if (i >= xs.size() - 1) i = xs.size() - 2;

    double h = xs[i + 1] - xs[i];
    if (h < 1e-15) return ys[i];

    double t = (xq - xs[i]) / h;
    double t2 = t * t;
    double t3 = t2 * t;

    // PCHIP: compute derivatives at endpoints (monotone)
    auto pchipSlope = [&](std::size_t k) -> double {
        if (k == 0 || k >= xs.size() - 1) return 0.0;
        double hk1 = xs[k] - xs[k - 1];
        double hk2 = xs[k + 1] - xs[k];
        double dk1 = (ys[k] - ys[k - 1]) / hk1;
        double dk2 = (ys[k + 1] - ys[k]) / hk2;
        if (sign(dk1) != sign(dk2)) return 0.0;
        double w1 = 2 * hk2 + hk1;
        double w2 = hk2 + 2 * hk1;
        return (w1 + w2) / (w1 / dk1 + w2 / dk2);
    };

    double d0 = pchipSlope(i);
    double d1 = pchipSlope(i + 1);

    // Hermite basis
    double h00 = 2 * t3 - 3 * t2 + 1;
    double h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2;
    double h11 = t3 - t2;

    return h00 * ys[i] + h10 * h * d0 + h01 * ys[i + 1] + h11 * h * d1;
}

std::vector<double> PathRelativeFFT::resampleUniform(
    const std::vector<double>& values,
    const std::vector<double>& abscissa,
    double xMin, double xMax,
    std::size_t numPoints) const {

    std::vector<double> result(numPoints, 0.0);
    if (numPoints == 0 || values.empty()) return result;
    if (numPoints == 1) {
        result[0] = values[0];
        return result;
    }

    double dx = (xMax - xMin) / static_cast<double>(numPoints - 1);
    if (dx < 1e-15) {
        std::fill(result.begin(), result.end(), values[0]);
        return result;
    }

    for (std::size_t k = 0; k < numPoints; ++k) {
        double xq = xMin + static_cast<double>(k) * dx;
        if (config_.interpolation == FFTConfig::Interpolation::CubicPCHIP) {
            result[k] = pchipInterpolate(abscissa, values, xq);
        } else {
            result[k] = linearInterpolate(abscissa, values, xq);
        }
    }
    return result;
}

PathRelativeFFT::FrenetErrors PathRelativeFFT::computeFrenetErrors(
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual,
    const PiecewiseNurbsPath& path) const {

    FrenetErrors errs;
    auto n = std::min(desired.size(), actual.size());
    if (n == 0) return errs;

    std::size_t dim = path.dim();
    auto activeAxes = detectActiveAxes(desired);
    std::vector<int> axes;
    for (int i = 0; i < static_cast<int>(activeAxes.size()) && i < static_cast<int>(dim); ++i) {
        axes.push_back(activeAxes[i]);
    }
    if (axes.size() < dim) {
        for (int i = static_cast<int>(axes.size()); i < static_cast<int>(dim); ++i) {
            axes.push_back(std::min(i, 2));
        }
    }

    errs.arcLengths.reserve(n);
    errs.times.reserve(n);
    errs.contour.reserve(n);
    errs.lag.reserve(n);
    errs.binormal.reserve(n);
    errs.combined.reserve(n);

    // Compute the maximum pathPosition to rescale to NURBS arc length
    double maxPathPos = 0.0;
    for (const auto& d : desired) {
        maxPathPos = std::max(maxPathPos, d.pathPosition);
    }
    double pathTotalLen = path.totalLength();
    double sScale = (maxPathPos > 1e-12) ? pathTotalLen / maxPathPos : 1.0;

    for (std::size_t i = 0; i < n; ++i) {
        const auto& des = desired[i];
        const auto& act = actual[i];

        double s = des.pathPosition;
        double sPath = s * sScale;
        double t = des.time;
        errs.arcLengths.push_back(s);
        errs.times.push_back(t);

        // Build RVec for actual position
        RVec actualPos = RVec::zero(dim);
        for (std::size_t d = 0; d < dim; ++d) {
            actualPos[d] = act.position[axes[d]];
        }

        // Compute contour and lag error
        double contourErr = 0.0, lagErr = 0.0;
        if (config_.useCertifiedContourError) {
            try {
                CertifiedContourError ce = computeCertifiedContourError(path, actualPos, sPath);
                contourErr = ce.contourError;
                lagErr = ce.lagError;
            } catch (...) {
                RVec desiredPos = path.evaluatePosition(sPath);
                contourErr = actualPos.distanceTo(desiredPos);
                lagErr = 0.0;
            }
        } else {
            // Tangent projection
            RVec desiredPos = path.evaluatePosition(sPath);
            double combined = actualPos.distanceTo(desiredPos);
            double vmag = std::sqrt(des.velocity[0] * des.velocity[0] +
                                    des.velocity[1] * des.velocity[1] +
                                    des.velocity[2] * des.velocity[2]);
            if (vmag > 1e-9) {
                double tx = des.velocity[0] / vmag;
                double ty = des.velocity[1] / vmag;
                double tz = des.velocity[2] / vmag;
                double ex = act.position[0] - des.position[0];
                double ey = act.position[1] - des.position[1];
                double ez = act.position[2] - des.position[2];
                lagErr = ex * tx + ey * ty + ez * tz;
                double cx = ex - lagErr * tx;
                double cy = ey - lagErr * ty;
                double cz = ez - lagErr * tz;
                contourErr = std::sqrt(cx * cx + cy * cy + cz * cz);
            } else {
                contourErr = combined;
                lagErr = 0.0;
            }
        }

        errs.contour.push_back(contourErr);
        errs.lag.push_back(lagErr);

        // Combined 3D error
        double dx = act.position[0] - des.position[0];
        double dy = act.position[1] - des.position[1];
        double dz = act.position[2] - des.position[2];
        errs.combined.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));

        // Binormal: for 3D paths, compute the component perpendicular to
        // both tangent and the contour direction. For 2D, binormal = 0.
        if (dim >= 3) {
            // Estimate binormal as the residual after removing contour and lag
            // from the 3D error. This is an approximation.
            double vmag = std::sqrt(des.velocity[0] * des.velocity[0] +
                                    des.velocity[1] * des.velocity[1] +
                                    des.velocity[2] * des.velocity[2]);
            if (vmag > 1e-9) {
                double tx = des.velocity[0] / vmag;
                double ty = des.velocity[1] / vmag;
                double tz = des.velocity[2] / vmag;
                double ex = act.position[0] - des.position[0];
                double ey = act.position[1] - des.position[1];
                double ez = act.position[2] - des.position[2];
                // Project out tangent
                double projT = ex * tx + ey * ty + ez * tz;
                double rx = ex - projT * tx;
                double ry = ey - projT * ty;
                double rz = ez - projT * tz;
                // The remaining component is mostly contour; binormal is
                // the part perpendicular to the path plane. For a 2D path
                // in XY, binormal = rz. For general 3D, we'd need the
                // actual normal vector. Use the magnitude of the residual
                // minus the contour as a rough binormal estimate.
                double resMag = std::sqrt(rx * rx + ry * ry + rz * rz);
                double binormal = std::max(0.0, resMag - contourErr);
                errs.binormal.push_back(binormal);
            } else {
                errs.binormal.push_back(0.0);
            }
        } else {
            errs.binormal.push_back(0.0);
        }
    }

    return errs;
}

ComponentSpectrum PathRelativeFFT::computeComponentSpectrum(
    const std::vector<double>& errorSignal,
    const std::vector<double>& abscissa,
    SpectralComponent component,
    SpectralDomain domain) const {

    ComponentSpectrum spec;
    spec.component = component;
    spec.domain = domain;
    spec.isCertified = config_.useCertifiedContourError;

    if (errorSignal.size() < 2 || abscissa.size() < 2) return spec;

    // Determine resampling parameters
    double xMin = abscissa.front();
    double xMax = abscissa.back();
    if (xMax - xMin < 1e-15) return spec;

    // Number of resampled points: next power of 2, clamped
    std::size_t nResampled = nextPow2(std::max(config_.minSamples,
        std::min(errorSignal.size(), config_.maxSamples)));

    // Resample to uniform grid
    std::vector<double> uniform = resampleUniform(
        errorSignal, abscissa, xMin, xMax, nResampled);

    // Detrend
    FFTProcessor::detrend(uniform, config_.removeDC, config_.removeLinearTrend);

    // Apply window
    FFTProcessor::applyWindow(uniform, config_.window);

    // Prepare complex input for FFT (zero-padded to nResampled which is already pow2)
    std::vector<std::complex<double>> data(nResampled);
    for (std::size_t i = 0; i < nResampled; ++i) {
        data[i] = std::complex<double>(uniform[i], 0.0);
    }

    // Run FFT
    FFTProcessor::fft(data);

    // Extract one-sided spectrum (positive frequencies)
    std::size_t numFreqs = nResampled / 2 + 1;
    spec.frequencies.resize(numFreqs);
    spec.magnitudes.resize(numFreqs);
    spec.phases.resize(numFreqs);
    spec.powerSpectralDensity.resize(numFreqs);

    // Frequency resolution
    double totalSpan = xMax - xMin;
    double df = 1.0 / totalSpan; // cycles per unit (cycles/mm or Hz)

    for (std::size_t k = 0; k < numFreqs; ++k) {
        spec.frequencies[k] = static_cast<double>(k) * df;
        double mag = std::abs(data[k]) / static_cast<double>(nResampled);
        spec.magnitudes[k] = mag;
        spec.phases[k] = std::arg(data[k]);
        spec.powerSpectralDensity[k] = mag * mag;
    }

    // DC component doesn't get doubled; others do (one-sided)
    for (std::size_t k = 1; k < numFreqs && k < nResampled / 2; ++k) {
        spec.magnitudes[k] *= 2.0;
        spec.powerSpectralDensity[k] *= 4.0; // (2*mag)² = 4*mag²
    }

    //--- Derived metrics ---

    // Total power
    spec.totalPower = std::accumulate(spec.powerSpectralDensity.begin(),
                                      spec.powerSpectralDensity.end(), 0.0);

    // RMS amplitude
    spec.rmsAmplitude = std::sqrt(spec.totalPower / static_cast<double>(numFreqs));

    // Find peaks
    spec.peaks = FFTProcessor::findPeaks(spec.frequencies, spec.magnitudes, spec.phases,
                           spec.powerSpectralDensity,
                           config_.maxPeaks, config_.peakProminenceThreshold);

    // Dominant frequency (highest peak, or highest PSD bin)
    if (!spec.peaks.empty()) {
        spec.dominantFrequency = spec.peaks[0].frequency;
        spec.dominantMagnitude = spec.peaks[0].magnitude;
    } else {
        // Find max PSD bin (excluding DC)
        std::size_t maxIdx = 1;
        for (std::size_t k = 2; k < numFreqs; ++k) {
            if (spec.powerSpectralDensity[k] > spec.powerSpectralDensity[maxIdx])
                maxIdx = k;
        }
        spec.dominantFrequency = spec.frequencies[maxIdx];
        spec.dominantMagnitude = spec.magnitudes[maxIdx];
    }

    // Spectral entropy
    if (spec.totalPower > 1e-15) {
        double entropy = 0.0;
        for (double p : spec.powerSpectralDensity) {
            if (p > 1e-15) {
                double pk = p / spec.totalPower;
                entropy -= pk * std::log2(pk);
            }
        }
        // Normalize to [0, 1]
        double maxEntropy = std::log2(static_cast<double>(numFreqs));
        spec.spectralEntropy = (maxEntropy > 1e-15) ? entropy / maxEntropy : 0.0;
    }

    // Oscillation index: PSD at dominant frequency / total PSD
    if (spec.totalPower > 1e-15 && !spec.peaks.empty()) {
        spec.oscillationIndex = spec.peaks[0].power / spec.totalPower;
    } else if (spec.totalPower > 1e-15) {
        // Use the max PSD bin
        std::size_t maxIdx = 1;
        for (std::size_t k = 2; k < numFreqs; ++k) {
            if (spec.powerSpectralDensity[k] > spec.powerSpectralDensity[maxIdx])
                maxIdx = k;
        }
        spec.oscillationIndex = spec.powerSpectralDensity[maxIdx] / spec.totalPower;
    }

    // Band power
    double lowCut = (domain == SpectralDomain::Spatial)
        ? config_.spatialLowBandCutoff : config_.temporalLowBandCutoff;
    double highCut = (domain == SpectralDomain::Spatial)
        ? config_.spatialHighBandCutoff : config_.temporalHighBandCutoff;

    for (std::size_t k = 0; k < numFreqs; ++k) {
        double f = spec.frequencies[k];
        double p = spec.powerSpectralDensity[k];
        if (f < lowCut) spec.lowBandPower += p;
        else if (f > highCut) spec.highBandPower += p;
        else spec.midBandPower += p;
    }

    // Harmonic distortion: (P_2nd + P_3rd) / P_fundamental
    if (!spec.peaks.empty() && spec.peaks[0].frequency > 1e-12) {
        double fFund = spec.peaks[0].frequency;
        double pFund = spec.peaks[0].power;
        if (pFund > 1e-15) {
            double p2nd = 0.0, p3rd = 0.0;
            // Find PSD at 2× and 3× fundamental frequency
            for (std::size_t k = 0; k < numFreqs; ++k) {
                double f = spec.frequencies[k];
                if (std::abs(f - 2.0 * fFund) < df * 0.5) p2nd = spec.powerSpectralDensity[k];
                if (std::abs(f - 3.0 * fFund) < df * 0.5) p3rd = spec.powerSpectralDensity[k];
            }
            spec.harmonicDistortion = (p2nd + p3rd) / pFund;
        }
    }

    return spec;
}

PathGeometryCorrelation PathRelativeFFT::computeGeometryCorrelation(
    const PiecewiseNurbsPath& path,
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<SpectralPeak>& spatialPeaks) const {

    PathGeometryCorrelation corr;

    // Corner frequency: 1 / average corner spacing
    if (path.numPieces() > 1) {
        try {
            CertifiedCornerDetection corners = detectCorners(path);
            if (corners.cornerCount > 0 && corners.junctions.size() > 0) {
                // Compute average spacing between corners
                std::vector<double> cornerArcLengths;
                for (const auto& j : corners.junctions) {
                    if (j.analysis.kind != tether::motion::CornerKind::Straight) {
                        // Approximate arc length at junction: sum of piece lengths up to pieceIn
                        double s = 0.0;
                        for (std::size_t i = 0; i <= j.pieceInIndex; ++i) {
                            s += path.piece(i).length();
                        }
                        cornerArcLengths.push_back(s);
                    }
                }
                if (cornerArcLengths.size() > 1) {
                    double totalSpacing = 0.0;
                    for (std::size_t i = 1; i < cornerArcLengths.size(); ++i) {
                        totalSpacing += cornerArcLengths[i] - cornerArcLengths[i - 1];
                    }
                    double avgSpacing = totalSpacing / static_cast<double>(cornerArcLengths.size() - 1);
                    if (avgSpacing > 1e-9) {
                        corr.cornerFrequency = 1.0 / avgSpacing;
                    }
                } else if (cornerArcLengths.size() == 1 && path.totalLength() > 1e-9) {
                    corr.cornerFrequency = 1.0 / path.totalLength();
                }
            }
        } catch (...) {
            // Corner detection may throw for paths with < 2 pieces
        }
    }

    // Segment frequency: 1 / average segment length
    if (path.numPieces() > 0) {
        double totalLen = path.totalLength();
        if (totalLen > 1e-9) {
            corr.segmentFrequency = static_cast<double>(path.numPieces()) / totalLen;
        }
    }

    // Arc frequency: 1 / average arc segment length
    double arcLenSum = 0.0;
    std::size_t arcCount = 0;
    for (const auto& s : desired) {
        if (s.motionType == 2 || s.motionType == 3) { // arcCW or arcCCW
            // Accumulate arc length per segment
            arcLenSum += s.pathPosition;
            ++arcCount;
        }
    }
    // Simplified: use segment-based arc frequency
    if (arcCount > 0 && path.totalLength() > 1e-9) {
        corr.arcFrequency = static_cast<double>(arcCount) / path.totalLength();
    }

    // Match peaks to geometry frequencies
    double tolerance = 0.1; // ±10% matching tolerance
    for (const auto& peak : spatialPeaks) {
        if (corr.cornerFrequency > 0 &&
            std::abs(peak.frequency - corr.cornerFrequency) / corr.cornerFrequency < tolerance) {
            corr.matchedPeaks.emplace_back(peak.frequency, "corner spacing");
        } else if (corr.segmentFrequency > 0 &&
                   std::abs(peak.frequency - corr.segmentFrequency) / corr.segmentFrequency < tolerance) {
            corr.matchedPeaks.emplace_back(peak.frequency, "segment length");
        } else if (corr.arcFrequency > 0 &&
                   std::abs(peak.frequency - corr.arcFrequency) / corr.arcFrequency < tolerance) {
            corr.matchedPeaks.emplace_back(peak.frequency, "arc length");
        }
    }

    return corr;
}

CrossDomainComparison PathRelativeFFT::computeCrossDomain(
    const ComponentSpectrum& spatial,
    const ComponentSpectrum& temporal,
    double avgVelocity) const {

    CrossDomainComparison cmp;

    // Feed-rate modulation index
    if (spatial.dominantFrequency > 1e-12 && avgVelocity > 1e-9) {
        double expectedTemporal = spatial.dominantFrequency * avgVelocity;
        if (expectedTemporal > 1e-12) {
            cmp.feedRateModulationIndex = temporal.dominantFrequency / expectedTemporal;
        }
    }

    // Spectral coherence: Pearson correlation of normalized spectra
    auto n = std::min(spatial.powerSpectralDensity.size(),
                      temporal.powerSpectralDensity.size());
    if (n > 2) {
        // Normalize both PSDs
        std::vector<double> normS(n), normT(n);
        double sMax = *std::max_element(spatial.powerSpectralDensity.begin(),
                                        spatial.powerSpectralDensity.begin() + n);
        double tMax = *std::max_element(temporal.powerSpectralDensity.begin(),
                                        temporal.powerSpectralDensity.begin() + n);
        if (sMax > 1e-15 && tMax > 1e-15) {
            for (std::size_t i = 0; i < n; ++i) {
                normS[i] = spatial.powerSpectralDensity[i] / sMax;
                normT[i] = temporal.powerSpectralDensity[i] / tMax;
            }
            cmp.spectralCoherence = pearsonCorrelation(normS, normT);
        }
    }

    // Path correlation assessment
    cmp.isPathCorrelated = (cmp.feedRateModulationIndex > 0.8 && cmp.feedRateModulationIndex < 1.2);

    // Interpretation
    if (cmp.isPathCorrelated) {
        cmp.interpretation = std::format(
            "Oscillation is path-correlated (modulation index {:.2f}). "
            "The spatial frequency {:.4f} cyc/mm maps to temporal frequency {:.2f} Hz "
            "at average velocity {:.1f} mm/s. This suggests a geometric resonance "
            "tied to path features.",
            cmp.feedRateModulationIndex,
            spatial.dominantFrequency, temporal.dominantFrequency, avgVelocity);
    } else if (cmp.feedRateModulationIndex > 0.01) {
        cmp.interpretation = std::format(
            "Oscillation has temporal modulation (modulation index {:.2f}). "
            "The temporal frequency ({:.2f} Hz) does not match the expected "
            "spatial frequency × velocity ({:.2f} Hz). This suggests a "
            "machine-dynamic resonance independent of path geometry.",
            cmp.feedRateModulationIndex,
            temporal.dominantFrequency,
            spatial.dominantFrequency * avgVelocity);
    } else {
        cmp.interpretation = "No significant oscillation detected.";
    }

    return cmp;
}

SpectralEvaluation PathRelativeFFT::evaluate(
    const std::vector<GCodeExport::TrajectorySample>& desired,
    const std::vector<GCodeExport::TrajectorySample>& actual) const {

    SpectralEvaluation eval;

    if (desired.empty() || actual.empty()) return eval;

    // Build the desired path
    PiecewiseNurbsPath path = convertTrajectory(desired);

    // Compute Frenet-decomposed errors
    FrenetErrors errs = computeFrenetErrors(desired, actual, path);

    if (errs.contour.empty()) return eval;

    // Compute average velocity for cross-domain comparison
    double avgVel = 0.0;
    if (!errs.times.empty() && errs.times.back() > errs.times.front()) {
        double totalDist = 0.0;
        for (std::size_t i = 1; i < errs.arcLengths.size(); ++i) {
            totalDist += std::abs(errs.arcLengths[i] - errs.arcLengths[i - 1]);
        }
        avgVel = totalDist / (errs.times.back() - errs.times.front());
    }

    //--- Spatial domain spectra ---
    eval.spatialContour = computeComponentSpectrum(
        errs.contour, errs.arcLengths, SpectralComponent::Contour, SpectralDomain::Spatial);
    eval.spatialLag = computeComponentSpectrum(
        errs.lag, errs.arcLengths, SpectralComponent::Lag, SpectralDomain::Spatial);
    eval.spatialBinormal = computeComponentSpectrum(
        errs.binormal, errs.arcLengths, SpectralComponent::Binormal, SpectralDomain::Spatial);
    eval.spatialCombined = computeComponentSpectrum(
        errs.combined, errs.arcLengths, SpectralComponent::Combined, SpectralDomain::Spatial);

    //--- Temporal domain spectra ---
    eval.temporalContour = computeComponentSpectrum(
        errs.contour, errs.times, SpectralComponent::Contour, SpectralDomain::Temporal);
    eval.temporalLag = computeComponentSpectrum(
        errs.lag, errs.times, SpectralComponent::Lag, SpectralDomain::Temporal);
    eval.temporalBinormal = computeComponentSpectrum(
        errs.binormal, errs.times, SpectralComponent::Binormal, SpectralDomain::Temporal);
    eval.temporalCombined = computeComponentSpectrum(
        errs.combined, errs.times, SpectralComponent::Combined, SpectralDomain::Temporal);

    //--- Cross-domain comparisons ---
    eval.contourComparison = computeCrossDomain(
        eval.spatialContour, eval.temporalContour, avgVel);
    eval.lagComparison = computeCrossDomain(
        eval.spatialLag, eval.temporalLag, avgVel);

    //--- Path geometry correlation ---
    // Use the spatial contour peaks for geometry matching
    eval.geometryCorrelation = computeGeometryCorrelation(
        path, desired, eval.spatialContour.peaks);

    //--- Overall oscillation assessment ---
    double maxOI = 0.0;
    std::string bestComponent;

    auto checkOI = [&](const ComponentSpectrum& s, const char* name) {
        if (s.oscillationIndex > maxOI) {
            maxOI = s.oscillationIndex;
            bestComponent = name;
        }
    };

    checkOI(eval.spatialContour, "spatial contour");
    checkOI(eval.spatialLag, "spatial lag");
    checkOI(eval.spatialBinormal, "spatial binormal");
    checkOI(eval.temporalContour, "temporal contour");
    checkOI(eval.temporalLag, "temporal lag");
    checkOI(eval.temporalBinormal, "temporal binormal");

    eval.oscillationSeverity = maxOI;
    eval.oscillationDetected = (maxOI > config_.oscillationIndexThreshold);

    if (eval.oscillationDetected) {
        // Find the spectrum with the highest OI for the description
        const ComponentSpectrum* worst = nullptr;
        std::string domain;
        if (eval.spatialContour.oscillationIndex == maxOI) {
            worst = &eval.spatialContour; domain = "spatial contour";
        } else if (eval.spatialLag.oscillationIndex == maxOI) {
            worst = &eval.spatialLag; domain = "spatial lag";
        } else if (eval.temporalContour.oscillationIndex == maxOI) {
            worst = &eval.temporalContour; domain = "temporal contour";
        } else if (eval.temporalLag.oscillationIndex == maxOI) {
            worst = &eval.temporalLag; domain = "temporal lag";
        } else if (eval.spatialBinormal.oscillationIndex == maxOI) {
            worst = &eval.spatialBinormal; domain = "spatial binormal";
        } else {
            worst = &eval.temporalBinormal; domain = "temporal binormal";
        }

        if (worst) {
            std::string unit = (worst->domain == SpectralDomain::Spatial) ? "cyc/mm" : "Hz";
            eval.oscillationDescription = std::format(
                "Oscillation detected in {} (OI={:.2f}): dominant frequency {:.4f} {} "
                "with magnitude {:.6f}. {}",
                domain, maxOI, worst->dominantFrequency, unit,
                worst->dominantMagnitude, eval.contourComparison.interpretation);
        }
    } else {
        eval.oscillationDescription = "No significant oscillation detected.";
    }

    return eval;
}

} // namespace tether::motion::replanner
