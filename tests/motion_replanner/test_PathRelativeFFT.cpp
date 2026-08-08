/**
 * @file test_PathRelativeFFT.cpp
 * @brief Tests for the PathRelativeFFT spectral analysis module.
 */

#include "tether/motion_replanner/PathRelativeFFT.hpp"
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace tether::motion::replanner;
using namespace tether::motion;
using GCodeExport::TrajectorySample;

namespace {

TrajectorySample makeSample(double t, double x, double y,
                            int32_t seg, uint8_t motionType,
                            double pathPos = 0.0) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, 0, 0, 0, 0, 0, 0, 0};
    s.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.acceleration = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    s.curvature = 0.0;
    return s;
}

/// Build a straight line path: (0,0)→(100,10) with uniform time sampling.
/// Both X and Y vary so dim=2.
std::vector<TrajectorySample> makeLinePath(int n = 64) {
    std::vector<TrajectorySample> samples;
    double dt = 0.01;
    double dx = 100.0 / static_cast<double>(n - 1);
    double dy = 10.0 / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        double x = i * dx;
        double y = i * dy;
        double s = std::sqrt(x * x + y * y);
        samples.push_back(makeSample(i * dt, x, y, 0, 1, s));
    }
    return samples;
}

/// Build an actual trajectory with a sinusoidal contour error.
/// The error oscillates at a known spatial frequency, perpendicular to the path.
std::vector<TrajectorySample> makeOscillatingActual(
    const std::vector<TrajectorySample>& desired,
    double amplitude, double spatialFrequency) {
    std::vector<TrajectorySample> actual = desired;
    // Normal direction to the (100,10) line: (-10, 100)/sqrt(10100) ≈ (-0.0995, 0.995)
    double nx = -0.0995, ny = 0.995;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        double s = desired[i].pathPosition;
        // Add sinusoidal offset perpendicular to the path
        double offset = amplitude * std::sin(2.0 * M_PI * spatialFrequency * s);
        actual[i].position[0] += offset * nx;
        actual[i].position[1] += offset * ny;
    }
    return actual;
}

} // anonymous namespace

//=============================================================================
// Basic spectral analysis tests
//=============================================================================

TEST(PathRelativeFFT, PerfectTracking_NoOscillation) {
    auto desired = makeLinePath(64);
    auto actual = desired; // Perfect tracking

    PathRelativeFFT fftEval;
    auto result = fftEval.evaluate(desired, actual);

    // No oscillation should be detected
    EXPECT_FALSE(result.oscillationDetected);
    EXPECT_LT(result.oscillationSeverity, 0.3);
}

TEST(PathRelativeFFT, SinusoidalError_DetectsFrequency) {
    auto desired = makeLinePath(128);
    double amplitude = 0.1; // 0.1mm
    double spatialFreq = 0.1; // 0.1 cycles/mm (period = 10mm)
    auto actual = makeOscillatingActual(desired, amplitude, spatialFreq);

    FFTConfig config;
    config.useCertifiedContourError = false;
    config.oscillationIndexThreshold = 0.1;
    PathRelativeFFT fftEval(config);
    auto result = fftEval.evaluate(desired, actual);

    // Should detect oscillation
    EXPECT_TRUE(result.oscillationDetected);
    EXPECT_GT(result.oscillationSeverity, 0.1);

    // The contour error is the absolute value of the perpendicular offset,
    // so a sinusoidal offset produces a rectified sine with 2× the frequency.
    // The dominant spatial frequency should be close to 2× the injected frequency
    EXPECT_NEAR(result.spatialContour.dominantFrequency, 2.0 * spatialFreq, spatialFreq * 0.5);
}

TEST(PathRelativeFFT, SpatialAndTemporalDomains) {
    auto desired = makeLinePath(64);
    auto actual = makeOscillatingActual(desired, 0.1, 0.1);

    FFTConfig config;
    config.useCertifiedContourError = false;
    PathRelativeFFT fftEval(config);
    auto result = fftEval.evaluate(desired, actual);

    // Both domains should have frequency data
    EXPECT_FALSE(result.spatialContour.frequencies.empty());
    EXPECT_FALSE(result.temporalContour.frequencies.empty());

    // Spatial frequencies should be in cycles/mm
    EXPECT_GT(result.spatialContour.frequencies.back(), 0.0);

    // Temporal frequencies should be in Hz
    EXPECT_GT(result.temporalContour.frequencies.back(), 0.0);
}

TEST(PathRelativeFFT, SpectralEntropy) {
    auto desired = makeLinePath(128);

    // Pure sinusoidal error → low entropy (concentrated spectrum)
    auto actualSin = makeOscillatingActual(desired, 0.1, 0.1);

    FFTConfig config;
    config.useCertifiedContourError = false;
    PathRelativeFFT fftEval(config);

    auto resultSin = fftEval.evaluate(desired, actualSin);

    // Sinusoidal error should have relatively low entropy
    EXPECT_LT(resultSin.spatialContour.spectralEntropy, 0.8);

    // Random noise → high entropy (broadband spectrum)
    std::vector<TrajectorySample> actualNoise = desired;
    for (auto& s : actualNoise) {
        s.position[1] = 0.1 * (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 2.0;
    }
    auto resultNoise = fftEval.evaluate(desired, actualNoise);

    // Noise should have higher entropy than sinusoid
    EXPECT_GT(resultNoise.spatialContour.spectralEntropy,
              resultSin.spatialContour.spectralEntropy);
}

TEST(PathRelativeFFT, BandPower) {
    auto desired = makeLinePath(128);
    auto actual = makeOscillatingActual(desired, 0.1, 0.1);

    FFTConfig config;
    config.useCertifiedContourError = false;
    PathRelativeFFT fftEval(config);
    auto result = fftEval.evaluate(desired, actual);

    // Total power should be the sum of band powers
    double totalBand = result.spatialContour.lowBandPower +
                       result.spatialContour.midBandPower +
                       result.spatialContour.highBandPower;
    EXPECT_NEAR(totalBand, result.spatialContour.totalPower,
                result.spatialContour.totalPower * 0.01);
}

TEST(PathRelativeFFT, PeakDetection) {
    auto desired = makeLinePath(128);
    auto actual = makeOscillatingActual(desired, 0.1, 0.1);

    FFTConfig config;
    config.useCertifiedContourError = false;
    config.maxPeaks = 3;
    PathRelativeFFT fftEval(config);
    auto result = fftEval.evaluate(desired, actual);

    // Should find at least one peak
    EXPECT_FALSE(result.spatialContour.peaks.empty());

    // The first peak should be at approximately 2× the injected frequency
    // (contour error is rectified)
    if (!result.spatialContour.peaks.empty()) {
        EXPECT_NEAR(result.spatialContour.peaks[0].frequency, 0.2, 0.1);
    }
}

TEST(PathRelativeFFT, CrossDomainComparison) {
    auto desired = makeLinePath(128);
    auto actual = makeOscillatingActual(desired, 0.1, 0.1);

    FFTConfig config;
    config.useCertifiedContourError = false;
    PathRelativeFFT fftEval(config);
    auto result = fftEval.evaluate(desired, actual);

    // Cross-domain comparison should have a modulation index
    EXPECT_GE(result.contourComparison.feedRateModulationIndex, 0.0);

    // Should have an interpretation string
    EXPECT_FALSE(result.contourComparison.interpretation.empty());
}

TEST(PathRelativeFFT, EmptyInput_ReturnsEmpty) {
    std::vector<TrajectorySample> empty;

    PathRelativeFFT fftEval;
    auto result = fftEval.evaluate(empty, empty);

    EXPECT_FALSE(result.oscillationDetected);
    EXPECT_TRUE(result.spatialContour.frequencies.empty());
}

TEST(PathRelativeFFT, ComponentSpectrum_DirectCall) {
    // Test the computeComponentSpectrum method directly
    std::vector<double> signal(64);
    std::vector<double> abscissa(64);
    for (std::size_t i = 0; i < 64; ++i) {
        double t = static_cast<double>(i);
        abscissa[i] = t;
        // 5 cycles over the signal
        signal[i] = std::sin(2.0 * M_PI * 5.0 * t / 63.0);
    }

    FFTConfig config;
    PathRelativeFFT fftEval(config);

    auto spec = fftEval.computeComponentSpectrum(
        signal, abscissa, SpectralComponent::Contour, SpectralDomain::Spatial);

    // Should have frequency data
    EXPECT_FALSE(spec.frequencies.empty());
    EXPECT_FALSE(spec.magnitudes.empty());

    // Dominant frequency should be approximately 5/63 ≈ 0.079 cycles/unit
    // (with some spectral leakage)
    EXPECT_GT(spec.dominantFrequency, 0.05);
    EXPECT_LT(spec.dominantFrequency, 0.15);
}

TEST(PathRelativeFFT, HarmonicDistortion) {
    // Create a signal with fundamental + 2nd harmonic
    std::vector<double> signal(256);
    std::vector<double> abscissa(256);
    for (std::size_t i = 0; i < 256; ++i) {
        double t = static_cast<double>(i);
        abscissa[i] = t;
        double fundamental = std::sin(2.0 * M_PI * 4.0 * t / 255.0);
        double second = 0.3 * std::sin(2.0 * M_PI * 8.0 * t / 255.0);
        signal[i] = fundamental + second;
    }

    FFTConfig config;
    PathRelativeFFT fftEval(config);

    auto spec = fftEval.computeComponentSpectrum(
        signal, abscissa, SpectralComponent::Contour, SpectralDomain::Spatial);

    // Should detect harmonic distortion
    EXPECT_GT(spec.harmonicDistortion, 0.0);
}

TEST(PathRelativeFFT, WindowFunctions) {
    std::vector<double> signal(64);
    std::vector<double> abscissa(64);
    for (std::size_t i = 0; i < 64; ++i) {
        double t = static_cast<double>(i);
        abscissa[i] = t;
        signal[i] = std::sin(2.0 * M_PI * 3.0 * t / 63.0);
    }

    // Test with different windows
    for (auto window : {FFTConfig::Window::Hann, FFTConfig::Window::Hamming,
                        FFTConfig::Window::Blackman, FFTConfig::Window::Rectangular}) {
        FFTConfig config;
        config.window = window;
        PathRelativeFFT fftEval(config);

        auto spec = fftEval.computeComponentSpectrum(
            signal, abscissa, SpectralComponent::Contour, SpectralDomain::Spatial);

        EXPECT_FALSE(spec.frequencies.empty());
        EXPECT_GT(spec.dominantMagnitude, 0.0);
    }
}

TEST(PathRelativeFFT, Detrending) {
    std::vector<double> signal(64);
    std::vector<double> abscissa(64);
    for (std::size_t i = 0; i < 64; ++i) {
        double t = static_cast<double>(i);
        abscissa[i] = t;
        // Signal with DC offset and linear trend + sinusoid
        signal[i] = 5.0 + 0.1 * t + std::sin(2.0 * M_PI * 3.0 * t / 63.0);
    }

    FFTConfig config;
    config.removeDC = true;
    config.removeLinearTrend = true;
    PathRelativeFFT fftEval(config);

    auto spec = fftEval.computeComponentSpectrum(
        signal, abscissa, SpectralComponent::Contour, SpectralDomain::Spatial);

    // After detrending, the DC component should be small
    if (!spec.magnitudes.empty()) {
        EXPECT_LT(spec.magnitudes[0], spec.dominantMagnitude * 0.5);
    }
}
