/**
 * @file test_GenericReNURBS.cpp
 * @brief Tests for the generic ReNURBS builder, certifier, and PA adapter.
 */

#include <gtest/gtest.h>

#include "tether/motion_planner/profile_renurbs/GenericReNURBSBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNURBSCertifier.hpp"
#include "tether/motion_planner/profile_renurbs/PressureAdvanceReNURBSAdapter.hpp"
#include "tether/motion_planner/profile_renurbs/ProfileSplineFitter.hpp"

#include <cmath>
#include <vector>

using namespace tether::motion::profile_renurbs;
using tether::motion::NurbsCurve;
using tether::motion::RVec;

// ============================================================================
// Helper: generate a smooth test curve
// ============================================================================

static std::vector<GenericSample> makeSmoothSamples(
    int n, double (*fn)(double), double tMax = 1.0) {
    std::vector<GenericSample> samples;
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1) * tMax;
        GenericSample s;
        s.parameter = t;
        s.quantities = {fn(t)};
        samples.push_back(s);
    }
    return samples;
}

// ============================================================================
// Generic Builder Tests
// ============================================================================

TEST(GenericReNURBSBuilderTest, BuildsSingleQuantitySmoothCurve) {
    auto samples = makeSmoothSamples(50, [](double t) {
        return std::sin(2.0 * M_PI * t);
    });

    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = false;
    QuantitySpec qs;
    qs.name = "test_quantity";
    qs.degree = 5;
    qs.epsilon = 1e-4;
    qs.limitType = LimitType::None;
    config.quantities = {qs};

    std::vector<SegmentInfo> segments;
    SegmentInfo si;
    si.paramStart = 0.0;
    si.paramEnd = 1.0;
    segments.push_back(si);

    auto profile = buildGenericReNURBSProfile(samples, segments, config);

    ASSERT_EQ(profile.numSegments(), 1u);
    ASSERT_EQ(profile.numQuantities(), 1u);
    EXPECT_EQ(profile.quantityNames[0], "test_quantity");
    ASSERT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());

    // Check interpolation at sample points
    const auto& curve = *profile.perSegment[0].quantities[0].curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();
    double segLen = segments[0].paramEnd - segments[0].paramStart;
    for (const auto& s : samples) {
        double u = (s.parameter - segments[0].paramStart) / segLen;
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_NEAR(val, s.quantities[0], 1e-3);
    }
}

TEST(GenericReNURBSBuilderTest, EmptySamplesReturnsEmptyProfile) {
    std::vector<GenericSample> samples;
    GenericReNURBSConfig config;
    config.enabled = true;
    config.quantities = {{"q", 1e-4, 1e-4, 5, std::nullopt, LimitType::None}};

    auto profile = buildGenericReNURBSProfile(samples, {}, config);
    EXPECT_TRUE(profile.empty());
}

TEST(GenericReNURBSBuilderTest, SingleSampleProducesConstant) {
    std::vector<GenericSample> samples = {
        {1.0, {42.0}, {}}
    };
    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = false;
    config.quantities = {{"q", 1e-4, 1e-4, 5, std::nullopt, LimitType::None}};

    auto profile = buildGenericReNURBSProfile(samples, {}, config);
    ASSERT_EQ(profile.numSegments(), 1u);
    ASSERT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());
    const auto& curve = *profile.perSegment[0].quantities[0].curve;
    EXPECT_NEAR(curve.evaluate(curve.knotMin())[0], 42.0, 1e-10);
    EXPECT_NEAR(curve.evaluate(curve.knotMax())[0], 42.0, 1e-10);
}

TEST(GenericReNURBSBuilderTest, MultipleQuantities) {
    // 2 quantities: sin and cos
    std::vector<GenericSample> samples;
    for (int i = 0; i < 50; ++i) {
        double t = static_cast<double>(i) / 49;
        GenericSample s;
        s.parameter = t;
        s.quantities = {std::sin(2 * M_PI * t), std::cos(2 * M_PI * t)};
        samples.push_back(s);
    }

    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = false;
    config.quantities = {
        {"sin_q", 1e-4, 1e-4, 5, std::nullopt, LimitType::None},
        {"cos_q", 1e-4, 1e-4, 5, std::nullopt, LimitType::None}
    };

    std::vector<SegmentInfo> segments = {{0.0, 1.0, {}}};

    auto profile = buildGenericReNURBSProfile(samples, segments, config);
    ASSERT_EQ(profile.numQuantities(), 2u);
    EXPECT_EQ(profile.quantityNames[0], "sin_q");
    EXPECT_EQ(profile.quantityNames[1], "cos_q");
    ASSERT_EQ(profile.perSegment[0].quantities.size(), 2u);
    EXPECT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());
    EXPECT_TRUE(profile.perSegment[0].quantities[1].curve.has_value());
}

TEST(GenericReNURBSBuilderTest, NoSegmentsUsesFullRange) {
    auto samples = makeSmoothSamples(20, [](double t) { return t * t; });
    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = false;
    config.quantities = {{"q", 1e-4, 1e-4, 3, std::nullopt, LimitType::None}};

    // No segments provided — should auto-create one covering [0, 1]
    auto profile = buildGenericReNURBSProfile(samples, {}, config);
    ASSERT_EQ(profile.numSegments(), 1u);
    EXPECT_NEAR(profile.perSegment[0].paramStart, 0.0, 1e-10);
    EXPECT_NEAR(profile.perSegment[0].paramEnd, 1.0, 1e-10);
}

// ============================================================================
// Constraint Tests
// ============================================================================

TEST(GenericReNURBSBuilderTest, SymmetricUniformLimitRespected) {
    // Generate samples that are within ±1.0
    auto samples = makeSmoothSamples(50, [](double t) {
        return 0.9 * std::sin(2 * M_PI * t);
    });

    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = true;
    config.certifyThrowOnFailure = false;
    QuantitySpec qs;
    qs.name = "bounded";
    qs.degree = 5;
    qs.epsilon = 1e-4;
    qs.safetyMargin = 0.01;
    qs.limitType = LimitType::SymmetricUniform;
    qs.uniformLimit = 1.0;
    config.quantities = {qs};

    std::vector<SegmentInfo> segments = {{0.0, 1.0, {}}};

    auto profile = buildGenericReNURBSProfile(samples, segments, config);

    ASSERT_EQ(profile.numSegments(), 1u);
    ASSERT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());

    // Check that the curve stays within ±(1.0 - safetyMargin + tol)
    const auto& curve = *profile.perSegment[0].quantities[0].curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();
    for (int k = 0; k <= 200; ++k) {
        double u = static_cast<double>(k) / 200.0;
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_LE(val, 1.0 + 1e-6) << "Exceeded upper limit at u=" << u;
        EXPECT_GE(val, -1.0 - 1e-6) << "Exceeded lower limit at u=" << u;
    }
}

TEST(GenericReNURBSBuilderTest, UpperPerSampleLimitRespected) {
    // Samples with a varying upper limit
    std::vector<GenericSample> samples;
    for (int i = 0; i < 50; ++i) {
        double t = static_cast<double>(i) / 49;
        GenericSample s;
        s.parameter = t;
        s.quantities = {0.5 * (1.0 - std::cos(2 * M_PI * t))};
        s.limits = {0.8 + 0.1 * t}; // limit increases from 0.8 to 0.9
        samples.push_back(s);
    }

    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = true;
    config.certifyThrowOnFailure = false;
    QuantitySpec qs;
    qs.name = "limited";
    qs.degree = 5;
    qs.epsilon = 1e-4;
    qs.safetyMargin = 0.01;
    qs.limitType = LimitType::UpperPerSample;
    config.quantities = {qs};

    std::vector<SegmentInfo> segments = {{0.0, 1.0, {}}};

    auto profile = buildGenericReNURBSProfile(samples, segments, config);
    ASSERT_EQ(profile.numSegments(), 1u);
    ASSERT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());

    // Check the curve stays below the limit at a dense grid
    const auto& curve = *profile.perSegment[0].quantities[0].curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();
    for (int k = 0; k <= 200; ++k) {
        double u = static_cast<double>(k) / 200.0;
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        double limit = 0.8 + 0.1 * u; // interpolated limit
        EXPECT_LE(val, limit + 1e-6)
            << "Exceeded limit at u=" << u << " val=" << val << " lim=" << limit;
    }
}

// ============================================================================
// Generic Certifier Tests
// ============================================================================

TEST(GenericReNURBSCertifierTest, CertifiesCompliantProfile) {
    auto samples = makeSmoothSamples(50, [](double t) {
        return 0.5 * std::sin(2 * M_PI * t);
    });

    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = false;
    QuantitySpec qs;
    qs.name = "q";
    qs.degree = 5;
    qs.epsilon = 1e-4;
    qs.limitType = LimitType::SymmetricUniform;
    qs.uniformLimit = 1.0;
    qs.safetyMargin = 0.01;
    config.quantities = {qs};

    std::vector<SegmentInfo> segments = {{0.0, 1.0, {}}};
    auto profile = buildGenericReNURBSProfile(samples, segments, config);

    auto cert = certifyGenericReNURBSProfile(profile, samples, segments, config);
    EXPECT_TRUE(cert.compliant);
    EXPECT_TRUE(cert.violations.empty());
}

TEST(GenericReNURBSCertifierTest, DetectsViolation) {
    // Build a profile with a tight limit, then manually violate it
    auto samples = makeSmoothSamples(50, [](double t) {
        return 0.5 * std::sin(2 * M_PI * t);
    });

    GenericReNURBSConfig config;
    config.enabled = true;
    config.certify = false;
    QuantitySpec qs;
    qs.name = "q";
    qs.degree = 5;
    qs.epsilon = 1e-4;
    qs.limitType = LimitType::SymmetricUniform;
    qs.uniformLimit = 1.0;
    qs.safetyMargin = 0.01;
    config.quantities = {qs};

    std::vector<SegmentInfo> segments = {{0.0, 1.0, {}}};
    auto profile = buildGenericReNURBSProfile(samples, segments, config);

    // Manually replace a control point to violate the limit
    ASSERT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());
    const auto& origCurve = *profile.perSegment[0].quantities[0].curve;
    auto cps = origCurve.controlPoints();
    auto knots = origCurve.knots();
    auto weights = origCurve.weights();
    if (cps.size() > 2) {
        cps[1] = RVec{5.0}; // way above limit
    }
    profile.perSegment[0].quantities[0].curve =
        NurbsCurve(cps, weights, knots, origCurve.degree());

    auto cert = certifyGenericReNURBSProfile(profile, samples, segments, config);
    EXPECT_FALSE(cert.compliant);
    EXPECT_FALSE(cert.violations.empty());
    EXPECT_EQ(cert.violations[0].quantityName, "q");
}

// ============================================================================
// Pressure Advance Adapter Tests
// ============================================================================

TEST(PressureAdvanceReNURBSAdapterTest, BuildsFromOffsetSeries) {
    // Simulate a PA offset series: smooth ramp up and down
    std::vector<double> velocities;
    std::vector<double> offsets;
    double dt = 0.001; // 1ms
    int n = 200;
    for (int i = 0; i < n; ++i) {
        double t = i * dt;
        double v = 50.0 * std::sin(M_PI * t / 0.2); // 0→50→0 over 0.2s
        velocities.push_back(v);
        // Linear PA: offset = PA * v, with PA = 0.05
        offsets.push_back(0.05 * v);
    }

    double maxComp = 0.5;
    PressureAdvanceReNURBSConfig config;
    config.certify = false;
    config.epsilon = 1e-6;

    auto profile = buildPressureAdvanceReNURBS(
        offsets, dt, maxComp, config);

    ASSERT_EQ(profile.numSegments(), 1u);
    ASSERT_EQ(profile.numQuantities(), 1u);
    EXPECT_EQ(profile.quantityNames[0], "pressure_offset");
    ASSERT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());

    // Check interpolation at a few sample points
    const auto& curve = *profile.perSegment[0].quantities[0].curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();
    double pStart = profile.perSegment[0].paramStart;
    double pEnd = profile.perSegment[0].paramEnd;
    double segLen = pEnd - pStart;
    for (int i = 0; i < n; i += 20) {
        double p = i * dt;
        double u = (p - pStart) / segLen;
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_NEAR(val, offsets[i], 1e-3)
            << "At sample " << i << " (t=" << p << ")";
    }
}

TEST(PressureAdvanceReNURBSAdapterTest, RespectsMaxCompensation) {
    // Generate offsets that are pre-clamped to ±maxCompensation (as a
    // real PA model would do before producing the output).
    std::vector<double> offsets;
    double dt = 0.001;
    int n = 200;
    double maxComp = 0.5;
    for (int i = 0; i < n; ++i) {
        double t = i * dt;
        double raw = 10.0 * std::sin(M_PI * t / 0.2);
        // Clamp to ±maxComp (as PA models do)
        if (raw > maxComp) raw = maxComp;
        if (raw < -maxComp) raw = -maxComp;
        offsets.push_back(raw);
    }

    PressureAdvanceReNURBSConfig config;
    config.certify = true;
    config.certifyThrowOnFailure = false;
    config.safetyMargin = 0.01;

    auto profile = buildPressureAdvanceReNURBS(
        offsets, dt, maxComp, config);

    ASSERT_EQ(profile.numSegments(), 1u);
    ASSERT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());

    // The NURBS curve should stay within ±(maxComp + tolerance).
    // The spline may slightly overshoot near the discontinuity where
    // the clamped offset transitions from flat to ramping.
    const auto& curve = *profile.perSegment[0].quantities[0].curve;
    double uMin = curve.knotMin();
    double uMax = curve.knotMax();
    double tolerance = 0.01; // 1% of maxComp
    for (int k = 0; k <= 200; ++k) {
        double u = static_cast<double>(k) / 200.0;
        double uu = uMin + u * (uMax - uMin);
        double val = curve.evaluate(uu)[0];
        EXPECT_LE(val, maxComp + tolerance)
            << "Exceeded maxCompensation at u=" << u;
        EXPECT_GE(val, -maxComp - tolerance)
            << "Exceeded -maxCompensation at u=" << u;
    }
}

TEST(PressureAdvanceReNURBSAdapterTest, TwoQuantityWithVelocity) {
    std::vector<double> offsets, velocities;
    double dt = 0.001;
    int n = 100;
    for (int i = 0; i < n; ++i) {
        double t = i * dt;
        double v = 50.0 * std::sin(M_PI * t / 0.1);
        velocities.push_back(v);
        offsets.push_back(0.05 * v);
    }

    PressureAdvanceReNURBSConfig config;
    config.certify = false;

    auto profile = buildPressureAdvanceReNURBS(
        offsets, velocities, dt, 0.5, config);

    ASSERT_EQ(profile.numQuantities(), 2u);
    EXPECT_EQ(profile.quantityNames[0], "pressure_offset");
    EXPECT_EQ(profile.quantityNames[1], "extruder_velocity");
    ASSERT_EQ(profile.perSegment[0].quantities.size(), 2u);
    EXPECT_TRUE(profile.perSegment[0].quantities[0].curve.has_value());
    EXPECT_TRUE(profile.perSegment[0].quantities[1].curve.has_value());
}

TEST(PressureAdvanceReNURBSAdapterTest, EmptyOffsetsReturnsEmpty) {
    std::vector<double> offsets;
    PressureAdvanceReNURBSConfig config;
    auto profile = buildPressureAdvanceReNURBS(offsets, 0.001, 0.5, config);
    EXPECT_TRUE(profile.empty());
}

// ============================================================================
// Conversion Tests (generic ↔ velocity-specific)
// ============================================================================

TEST(GenericReNURBSConversionTest, RoundTripPreservesData) {
    // Create a generic profile with 4 quantities
    GenericReNURBSProfile generic;
    generic.quantityNames = {"velocity", "acceleration", "jerk", "time"};

    GenericSegmentProfile seg;
    seg.segmentIndex = 0;
    seg.paramStart = 0.0;
    seg.paramEnd = 100.0;
    for (int i = 0; i < 4; ++i) {
        GenericQuantityCurve q;
        std::vector<RVec> cps = {RVec{static_cast<double>(i)},
                                  RVec{static_cast<double>(i + 1)}};
        q.curve = NurbsCurve(cps, {1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, 1);
        q.numControlPoints = 2;
        seg.quantities.push_back(q);
        seg.boundaryContinuity.push_back(ContinuityClass::C0);
    }
    generic.perSegment.push_back(std::move(seg));

    // Convert to velocity-specific and back
    auto velocity = toVelocityProfile(generic);
    ASSERT_EQ(velocity.numSegments(), 1u);
    EXPECT_NEAR(velocity.perSegment[0].sStart, 0.0, 1e-10);
    EXPECT_NEAR(velocity.perSegment[0].sEnd, 100.0, 1e-10);
    ASSERT_TRUE(velocity.perSegment[0].velocity.curve.has_value());
    ASSERT_TRUE(velocity.perSegment[0].acceleration.curve.has_value());

    auto roundTrip = fromVelocityProfile(velocity);
    ASSERT_EQ(roundTrip.numQuantities(), 4u);
    EXPECT_EQ(roundTrip.quantityNames[0], "velocity");
    EXPECT_EQ(roundTrip.quantityNames[3], "time");
    ASSERT_EQ(roundTrip.perSegment[0].quantities.size(), 4u);
}
