/**
 * @file test_MultiAxisPath.cpp
 * @brief Tests for CiA402 Multi-Axis Path classes
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia402/MultiAxisPath.hpp"
#include <cmath>
#include <array>

using namespace CiA402;

// ============================================================================
// Enums
// ============================================================================

TEST(PathEnums, ArcDirection) {
    EXPECT_NE(static_cast<int>(ArcDirection::CW), static_cast<int>(ArcDirection::CCW));
}

TEST(PathEnums, Plane) {
    EXPECT_NE(static_cast<int>(Plane::XY), static_cast<int>(Plane::XZ));
    EXPECT_NE(static_cast<int>(Plane::XZ), static_cast<int>(Plane::YZ));
}

TEST(PathEnums, PathType) {
    EXPECT_NE(static_cast<int>(PathType::Linear), static_cast<int>(PathType::Circular));
    EXPECT_NE(static_cast<int>(PathType::Helical), static_cast<int>(PathType::BSpline));
    EXPECT_NE(static_cast<int>(PathType::NURBS), static_cast<int>(PathType::Bezier));
    (void)PathType::Polynomial;
}

TEST(PathEnums, BlendMode) {
    EXPECT_NE(static_cast<int>(BlendMode::None), static_cast<int>(BlendMode::Corner));
    EXPECT_NE(static_cast<int>(BlendMode::Arc), static_cast<int>(BlendMode::Spline));
}

// ============================================================================
// PathPoint struct
// ============================================================================

TEST(PathPointTest, DefaultValues) {
    PathPoint p{};
    EXPECT_DOUBLE_EQ(p.parameter, 0.0);
    EXPECT_DOUBLE_EQ(p.pathVelocity, 0.0);
    EXPECT_DOUBLE_EQ(p.curvature, 0.0);
}

// ============================================================================
// Factory function
// ============================================================================

TEST(PathFactoryTest, CreateAllTypes) {
    auto linear = createPathSegment(PathType::Linear);
    ASSERT_NE(linear, nullptr);
    EXPECT_EQ(linear->getType(), PathType::Linear);

    auto circular = createPathSegment(PathType::Circular);
    ASSERT_NE(circular, nullptr);
    EXPECT_EQ(circular->getType(), PathType::Circular);

    auto helical = createPathSegment(PathType::Helical);
    ASSERT_NE(helical, nullptr);
    EXPECT_EQ(helical->getType(), PathType::Helical);

    auto bspline = createPathSegment(PathType::BSpline);
    ASSERT_NE(bspline, nullptr);
    EXPECT_EQ(bspline->getType(), PathType::BSpline);

    auto nurbs = createPathSegment(PathType::NURBS);
    ASSERT_NE(nurbs, nullptr);
    EXPECT_EQ(nurbs->getType(), PathType::NURBS);

    auto bezier = createPathSegment(PathType::Bezier);
    ASSERT_NE(bezier, nullptr);
    EXPECT_EQ(bezier->getType(), PathType::Bezier);
}

// ============================================================================
// LinearPath
// ============================================================================

class LinearPathTest : public ::testing::Test {
protected:
    void SetUp() override {
        LinearConfig cfg{};
        cfg.numAxes = 3;
        cfg.start[0] = 0; cfg.start[1] = 0; cfg.start[2] = 0;
        cfg.end[0] = 10;  cfg.end[1] = 0;   cfg.end[2] = 0;
        path_.configure(cfg);
    }
    LinearPath path_;
};

TEST_F(LinearPathTest, Type) {
    EXPECT_EQ(path_.getType(), PathType::Linear);
}

TEST_F(LinearPathTest, NumAxes) {
    EXPECT_EQ(path_.getNumAxes(), 3u);
}

TEST_F(LinearPathTest, Length) {
    EXPECT_NEAR(path_.getLength(), 10.0, 0.01);
}

TEST_F(LinearPathTest, EvaluateStart) {
    auto p = path_.evaluate(0.0);
    EXPECT_NEAR(p.position[0], 0.0, 0.01);
}

TEST_F(LinearPathTest, EvaluateEnd) {
    auto p = path_.evaluate(1.0);
    EXPECT_NEAR(p.position[0], 10.0, 0.01);
}

TEST_F(LinearPathTest, EvaluateMid) {
    auto p = path_.evaluate(0.5);
    EXPECT_NEAR(p.position[0], 5.0, 0.01);
}

TEST_F(LinearPathTest, StartEndPoints) {
    auto s = path_.getStartPoint();
    auto e = path_.getEndPoint();
    EXPECT_NEAR(s.position[0], 0.0, 0.01);
    EXPECT_NEAR(e.position[0], 10.0, 0.01);
}

TEST_F(LinearPathTest, Sample) {
    auto pts = path_.sample(5);
    EXPECT_EQ(pts.size(), 5u);
}

TEST_F(LinearPathTest, ArcLength) {
    double len = path_.arcLength(1.0);
    EXPECT_NEAR(len, 10.0, 0.1);
}

TEST_F(LinearPathTest, ParameterAtLength) {
    double u = path_.parameterAtLength(5.0);
    EXPECT_NEAR(u, 0.5, 0.05);
}

TEST_F(LinearPathTest, DefaultConstructor) {
    LinearPath p;
    EXPECT_EQ(p.getType(), PathType::Linear);
}

TEST_F(LinearPathTest, DiagonalPath) {
    LinearConfig cfg{};
    cfg.numAxes = 2;
    cfg.start[0] = 0; cfg.start[1] = 0;
    cfg.end[0] = 3;   cfg.end[1] = 4;
    LinearPath p;
    p.configure(cfg);
    EXPECT_NEAR(p.getLength(), 5.0, 0.01);
}

// ============================================================================
// CircularPath
// ============================================================================

class CircularPathTest : public ::testing::Test {
protected:
    void SetUp() override {
        CircularConfig cfg{};
        cfg.center[0] = 0; cfg.center[1] = 0; cfg.center[2] = 0;
        cfg.start[0] = 1;  cfg.start[1] = 0;  cfg.start[2] = 0;
        cfg.end[0] = 0;    cfg.end[1] = 1;    cfg.end[2] = 0;
        cfg.radius = 1.0;
        cfg.plane = Plane::XY;
        cfg.direction = ArcDirection::CCW;
        cfg.useAngles = false;
        path_.configure(cfg);
    }
    CircularPath path_;
};

TEST_F(CircularPathTest, Type) {
    EXPECT_EQ(path_.getType(), PathType::Circular);
}

TEST_F(CircularPathTest, NumAxes) {
    EXPECT_EQ(path_.getNumAxes(), 3u);
}

TEST_F(CircularPathTest, Radius) {
    EXPECT_NEAR(path_.getRadius(), 1.0, 0.01);
}

TEST_F(CircularPathTest, ArcAngle) {
    double angle = path_.getArcAngle();
    EXPECT_GT(std::abs(angle), 0.0);
}

TEST_F(CircularPathTest, Length) {
    double len = path_.getLength();
    EXPECT_GT(len, 0.0);
}

TEST_F(CircularPathTest, EvaluateStartEnd) {
    auto s = path_.evaluate(0.0);
    auto e = path_.evaluate(1.0);
    (void)s;
    (void)e;
}

TEST_F(CircularPathTest, DefaultConstructor) {
    CircularPath p;
    EXPECT_EQ(p.getType(), PathType::Circular);
}

TEST_F(CircularPathTest, ConfigureFromPoints) {
    CircularPath p;
    std::array<double,3> center = {0, 0, 0};
    std::array<double,3> start = {1, 0, 0};
    std::array<double,3> end = {0, 1, 0};
    p.configureFromPoints(center, start, end, ArcDirection::CCW, Plane::XY);
    EXPECT_NEAR(p.getRadius(), 1.0, 0.01);
}

TEST_F(CircularPathTest, ConfigureFromRadius) {
    CircularPath p;
    std::array<double,3> start = {1, 0, 0};
    std::array<double,3> end = {-1, 0, 0};
    p.configureFromRadius(start, end, 1.0, false, ArcDirection::CCW, Plane::XY);
    EXPECT_GT(p.getLength(), 0.0);
}

TEST_F(CircularPathTest, ClockwiseDirection) {
    CircularConfig cfg{};
    cfg.center[0] = 0; cfg.center[1] = 0; cfg.center[2] = 0;
    cfg.start[0] = 1;  cfg.start[1] = 0;  cfg.start[2] = 0;
    cfg.end[0] = 0;    cfg.end[1] = 1;    cfg.end[2] = 0;
    cfg.radius = 1.0;
    cfg.plane = Plane::XY;
    cfg.direction = ArcDirection::CW;
    cfg.useAngles = false;
    CircularPath p(cfg);
    EXPECT_GT(p.getLength(), 0.0);
}

TEST_F(CircularPathTest, WithAngles) {
    CircularConfig cfg{};
    cfg.center[0] = 0; cfg.center[1] = 0; cfg.center[2] = 0;
    cfg.radius = 1.0;
    cfg.startAngle = 0.0;
    cfg.endAngle = M_PI / 2.0;
    cfg.plane = Plane::XY;
    cfg.direction = ArcDirection::CCW;
    cfg.useAngles = true;
    CircularPath p(cfg);
    EXPECT_GT(p.getLength(), 0.0);
    EXPECT_NEAR(p.getArcAngle(), M_PI / 2.0, 0.01);
}

TEST_F(CircularPathTest, XZPlane) {
    CircularConfig cfg{};
    cfg.center[0] = 0; cfg.center[1] = 0; cfg.center[2] = 0;
    cfg.start[0] = 1;  cfg.start[1] = 0;  cfg.start[2] = 0;
    cfg.end[0] = 0;    cfg.end[1] = 0;    cfg.end[2] = 1;
    cfg.radius = 1.0;
    cfg.plane = Plane::XZ;
    cfg.direction = ArcDirection::CCW;
    cfg.useAngles = false;
    CircularPath p(cfg);
    EXPECT_EQ(p.getType(), PathType::Circular);
}

TEST_F(CircularPathTest, YZPlane) {
    CircularConfig cfg{};
    cfg.center[0] = 0; cfg.center[1] = 0; cfg.center[2] = 0;
    cfg.start[0] = 0;  cfg.start[1] = 1;  cfg.start[2] = 0;
    cfg.end[0] = 0;    cfg.end[1] = 0;    cfg.end[2] = 1;
    cfg.radius = 1.0;
    cfg.plane = Plane::YZ;
    cfg.direction = ArcDirection::CCW;
    cfg.useAngles = false;
    CircularPath p(cfg);
    EXPECT_EQ(p.getType(), PathType::Circular);
}

// ============================================================================
// HelicalPath
// ============================================================================

TEST(HelicalPathTest, Type) {
    HelicalPath p;
    EXPECT_EQ(p.getType(), PathType::Helical);
}

TEST(HelicalPathTest, NumAxes) {
    HelicalPath p;
    EXPECT_EQ(p.getNumAxes(), 3u);
}

TEST(HelicalPathTest, Configure) {
    HelicalConfig cfg{};
    cfg.center[0] = 0; cfg.center[1] = 0; cfg.center[2] = 0;
    cfg.radius = 1.0;
    cfg.pitch = 2.0;
    cfg.startAngle = 0.0;
    cfg.totalAngle = 2.0 * M_PI;
    cfg.plane = Plane::XY;
    cfg.direction = ArcDirection::CCW;
    HelicalPath p(cfg);
    EXPECT_GT(p.getLength(), 0.0);
    auto s = p.evaluate(0.0);
    auto e = p.evaluate(1.0);
    (void)s;
    (void)e;
}

TEST(HelicalPathTest, CWDirection) {
    HelicalConfig cfg{};
    cfg.center[0] = 0; cfg.center[1] = 0; cfg.center[2] = 0;
    cfg.radius = 1.0;
    cfg.pitch = 1.0;
    cfg.startAngle = 0.0;
    cfg.totalAngle = M_PI;
    cfg.plane = Plane::XY;
    cfg.direction = ArcDirection::CW;
    HelicalPath p(cfg);
    EXPECT_GT(p.getLength(), 0.0);
}

// ============================================================================
// BSplinePath
// ============================================================================

TEST(BSplinePathTest, Type) {
    BSplinePath p;
    EXPECT_EQ(p.getType(), PathType::BSpline);
}

TEST(BSplinePathTest, Configure) {
    BSplineConfig cfg{};
    cfg.numAxes = 2;
    cfg.degree = 2;
    std::array<double, MAX_PATH_AXES> p0{}, p1{}, p2{}, p3{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 1; p1[1] = 1;
    p2[0] = 2; p2[1] = 0;
    p3[0] = 3; p3[1] = 1;
    cfg.controlPoints = {p0, p1, p2, p3};
    BSplinePath p(cfg);
    EXPECT_EQ(p.getDegree(), 2);
    EXPECT_GT(p.getLength(), 0.0);
}

TEST(BSplinePathTest, AddControlPoints) {
    BSplinePath p;
    std::array<double, MAX_PATH_AXES> pt{};
    pt[0] = 0; pt[1] = 0;
    p.addControlPoint(pt);
    pt[0] = 1; pt[1] = 1;
    p.addControlPoint(pt);
    pt[0] = 2; pt[1] = 0;
    p.addControlPoint(pt);
    p.clearControlPoints();
}

TEST(BSplinePathTest, EvaluateConfigured) {
    BSplineConfig cfg{};
    cfg.numAxes = 2;
    cfg.degree = 2;
    std::array<double, MAX_PATH_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 1; p1[1] = 2;
    p2[0] = 2; p2[1] = 0;
    cfg.controlPoints = {p0, p1, p2};
    BSplinePath p(cfg);
    auto s = p.evaluate(0.0);
    auto e = p.evaluate(1.0);
    EXPECT_NEAR(s.position[0], 0.0, 0.1);
    EXPECT_NEAR(e.position[0], 2.0, 0.1);
}

// ============================================================================
// NURBSPath
// ============================================================================

TEST(NURBSPathTest, Type) {
    NURBSPath p;
    EXPECT_EQ(p.getType(), PathType::NURBS);
}

TEST(NURBSPathTest, Configure) {
    NURBSConfig cfg{};
    cfg.numAxes = 2;
    cfg.degree = 2;
    std::array<double, MAX_PATH_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 1; p1[1] = 1;
    p2[0] = 2; p2[1] = 0;
    cfg.controlPoints = {p0, p1, p2};
    cfg.weights = {1.0, 1.0, 1.0};
    NURBSPath p(cfg);
    auto s = p.evaluate(0.0);
    auto e = p.evaluate(1.0);
    (void)s;
    (void)e;
    EXPECT_GT(p.getLength(), 0.0);
}

// ============================================================================
// BezierPath
// ============================================================================

TEST(BezierPathTest, Type) {
    BezierPath p;
    EXPECT_EQ(p.getType(), PathType::Bezier);
}

TEST(BezierPathTest, ConfigureCubic) {
    BezierPath p;
    std::array<double, MAX_PATH_AXES> p0{}, p1{}, p2{}, p3{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 1; p1[1] = 2;
    p2[0] = 3; p2[1] = 2;
    p3[0] = 4; p3[1] = 0;
    p.configureCubic(p0, p1, p2, p3, 2);
    EXPECT_EQ(p.getDegree(), 3);
    EXPECT_GT(p.getLength(), 0.0);
}

TEST(BezierPathTest, Configure) {
    BezierConfig cfg{};
    cfg.numAxes = 2;
    std::array<double, MAX_PATH_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 1; p1[1] = 1;
    p2[0] = 2; p2[1] = 0;
    cfg.controlPoints = {p0, p1, p2};
    BezierPath p(cfg);
    EXPECT_EQ(p.getDegree(), 2);
    auto s = p.evaluate(0.0);
    auto e = p.evaluate(1.0);
    EXPECT_NEAR(s.position[0], 0.0, 0.01);
    EXPECT_NEAR(e.position[0], 2.0, 0.01);
}

TEST(BezierPathTest, EvalMidpoint) {
    BezierConfig cfg{};
    cfg.numAxes = 2;
    std::array<double, MAX_PATH_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 1; p1[1] = 2;
    p2[0] = 2; p2[1] = 0;
    cfg.controlPoints = {p0, p1, p2};
    BezierPath p(cfg);
    auto m = p.evaluate(0.5);
    EXPECT_NEAR(m.position[0], 1.0, 0.01);
    EXPECT_NEAR(m.position[1], 1.0, 0.01);
}

// ============================================================================
// MultiSegmentPath
// ============================================================================

TEST(MultiSegmentPathTest, AddSegments) {
    MultiSegmentPath msp;
    auto seg1 = std::make_shared<LinearPath>();
    LinearConfig c1{};
    c1.numAxes = 2;
    c1.start[0] = 0; c1.start[1] = 0;
    c1.end[0] = 10;  c1.end[1] = 0;
    seg1->configure(c1);

    auto seg2 = std::make_shared<LinearPath>();
    LinearConfig c2{};
    c2.numAxes = 2;
    c2.start[0] = 10; c2.start[1] = 0;
    c2.end[0] = 10;   c2.end[1] = 10;
    seg2->configure(c2);

    msp.addSegment(seg1);
    msp.addSegment(seg2);
    EXPECT_EQ(msp.getSegmentCount(), 2u);
    EXPECT_NEAR(msp.getTotalLength(), 20.0, 0.1);
}

TEST(MultiSegmentPathTest, Clear) {
    MultiSegmentPath msp;
    auto seg = std::make_shared<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 5;   c.end[1] = 0;
    seg->configure(c);
    msp.addSegment(seg);
    EXPECT_EQ(msp.getSegmentCount(), 1u);
    msp.clear();
    EXPECT_EQ(msp.getSegmentCount(), 0u);
}

TEST(MultiSegmentPathTest, BlendMode) {
    MultiSegmentPath msp;
    msp.setBlendMode(BlendMode::Arc);
    msp.setBlendTolerance(0.1);
    (void)msp;
}

TEST(MultiSegmentPathTest, Plan) {
    MultiSegmentPath msp;
    auto seg = std::make_shared<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 10;  c.end[1] = 0;
    seg->configure(c);
    msp.addSegment(seg);

    MotionLimits limits{};
    limits.maxVelocity = 100.0;
    limits.maxAcceleration = 1000.0;
    limits.maxDeceleration = 1000.0;
    msp.plan(50.0, limits);
    EXPECT_GT(msp.getDuration(), 0.0);
}

TEST(MultiSegmentPathTest, SamplePlannedPath) {
    MultiSegmentPath msp;
    auto seg = std::make_shared<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 10;  c.end[1] = 0;
    seg->configure(c);
    msp.addSegment(seg);

    MotionLimits limits{};
    limits.maxVelocity = 100.0;
    limits.maxAcceleration = 1000.0;
    limits.maxDeceleration = 1000.0;
    msp.plan(50.0, limits);

    auto p = msp.sample(0.0);
    EXPECT_NEAR(p.position[0], 0.0, 1.0);
}

TEST(MultiSegmentPathTest, GetType) {
    MultiSegmentPath msp;
    EXPECT_EQ(msp.getType(), PathType::Linear);
}

// ============================================================================
// PathSampler
// ============================================================================

TEST(PathSamplerTest, DefaultConstruction) {
    PathSampler ps;
    EXPECT_DOUBLE_EQ(ps.getDuration(), 0.0);
}

TEST(PathSamplerTest, WithPath) {
    auto path = std::make_unique<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 100; c.end[1] = 0;
    path->configure(c);
    PathSampler ps(std::move(path), 50.0);
    EXPECT_GT(ps.getDuration(), 0.0);
}

TEST(PathSamplerTest, ConfigureAndPlan) {
    auto path = std::make_shared<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 100; c.end[1] = 0;
    path->configure(c);
    auto profile = std::make_shared<LinearProfile>();
    MotionLimits l{};
    l.maxVelocity = 50.0;
    profile->setLimits(l);

    PathSampler ps;
    ps.configure(path, profile);
    ps.plan(50.0);
    EXPECT_GT(ps.getDuration(), 0.0);
}

TEST(PathSamplerTest, SampleAtTime) {
    auto path = std::make_unique<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 100; c.end[1] = 0;
    path->configure(c);
    PathSampler ps(std::move(path), 50.0);
    auto p = ps.sampleAtTime(0.0);
    EXPECT_NEAR(p.position[0], 0.0, 1.0);
}

TEST(PathSamplerTest, IsComplete) {
    auto path = std::make_unique<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 100; c.end[1] = 0;
    path->configure(c);
    PathSampler ps(std::move(path), 50.0);
    EXPECT_FALSE(ps.isComplete(0.0));
    EXPECT_TRUE(ps.isComplete(ps.getDuration() + 1.0));
}

TEST(PathSamplerTest, GetParameter) {
    auto path = std::make_unique<LinearPath>();
    LinearConfig c{};
    c.numAxes = 2;
    c.start[0] = 0; c.start[1] = 0;
    c.end[0] = 100; c.end[1] = 0;
    path->configure(c);
    PathSampler ps(std::move(path), 50.0);
    double u = ps.getParameter(0.0);
    EXPECT_GE(u, 0.0);
}

// ============================================================================
// Config structs
// ============================================================================

TEST(LinearConfigTest, Default) {
    LinearConfig c{};
    EXPECT_EQ(c.numAxes, 2u);
}

TEST(CircularConfigTest, Default) {
    CircularConfig c{};
    EXPECT_DOUBLE_EQ(c.radius, 0.0);
    EXPECT_FALSE(c.useAngles);
}

TEST(HelicalConfigTest, Default) {
    HelicalConfig c{};
    EXPECT_DOUBLE_EQ(c.radius, 100.0);
    EXPECT_DOUBLE_EQ(c.pitch, 10.0);
}

TEST(BSplineConfigTest, Default) {
    BSplineConfig c{};
    EXPECT_TRUE(c.controlPoints.empty());
    EXPECT_TRUE(c.knots.empty());
}

TEST(NURBSConfigTest, Default) {
    NURBSConfig c{};
    EXPECT_TRUE(c.controlPoints.empty());
    EXPECT_TRUE(c.weights.empty());
}

TEST(BezierConfigTest, Default) {
    BezierConfig c{};
    EXPECT_TRUE(c.controlPoints.empty());
}
