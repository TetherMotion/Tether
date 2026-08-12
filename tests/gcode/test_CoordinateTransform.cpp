/**
 * @file test_CoordinateTransform.cpp
 * @brief Unit tests for the CoordinateTransform (scale + rotation + offset).
 */

#include "tether/gcode/motion/CoordinateTransform.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace GCode;

namespace {
constexpr double kTol = 1e-9;
}

// ============================================================================
// Identity
// ============================================================================

TEST(CoordinateTransform, IdentityIsIdentity) {
    CoordinateTransform t;
    EXPECT_TRUE(t.isIdentity());
    auto m = t.toMachineXYZ(10, 20, 30);
    EXPECT_NEAR(m[0], 10, kTol);
    EXPECT_NEAR(m[1], 20, kTol);
    EXPECT_NEAR(m[2], 30, kTol);
}

TEST(CoordinateTransform, IdentityRoundTrip) {
    CoordinateTransform t;
    auto m = t.toMachineXYZ(1, 2, 3);
    auto p = t.toProgramXYZ(m[0], m[1], m[2]);
    EXPECT_NEAR(p[0], 1, kTol);
    EXPECT_NEAR(p[1], 2, kTol);
    EXPECT_NEAR(p[2], 3, kTol);
}

// ============================================================================
// Pure offset (WCS)
// ============================================================================

TEST(CoordinateTransform, WCSOffset) {
    CoordinateTransform t;
    t.setWCSOffset({10, 20, 30});
    auto m = t.toMachineXYZ(0, 0, 0);
    EXPECT_NEAR(m[0], 10, kTol);
    EXPECT_NEAR(m[1], 20, kTol);
    EXPECT_NEAR(m[2], 30, kTol);
    auto p = t.toProgramXYZ(10, 20, 30);
    EXPECT_NEAR(p[0], 0, kTol);
    EXPECT_NEAR(p[1], 0, kTol);
    EXPECT_NEAR(p[2], 0, kTol);
}

TEST(CoordinateTransform, G52Offset) {
    CoordinateTransform t;
    t.setG52Offset({5, -5, 0});
    auto m = t.toMachineXYZ(10, 10, 10);
    EXPECT_NEAR(m[0], 15, kTol);
    EXPECT_NEAR(m[1], 5,  kTol);
    EXPECT_NEAR(m[2], 10, kTol);
}

TEST(CoordinateTransform, G92Offset) {
    CoordinateTransform t;
    t.setG92Offset({100, 50, 25});
    auto m = t.toMachineXYZ(0, 0, 0);
    EXPECT_NEAR(m[0], 100, kTol);
    EXPECT_NEAR(m[1], 50,  kTol);
    EXPECT_NEAR(m[2], 25,  kTol);
}

TEST(CoordinateTransform, CombinedG52G92WCS) {
    CoordinateTransform t;
    t.setG52Offset({1, 2, 3});
    t.setG92Offset({10, 20, 30});
    t.setWCSOffset({100, 200, 300});
    // P_machine = T_wcs + (P_program + T_g52 + T_g92)
    // (0,0,0) -> (0+1+10+100, 0+2+20+200, 0+3+30+300) = (111, 222, 333)
    auto m = t.toMachineXYZ(0, 0, 0);
    EXPECT_NEAR(m[0], 111, kTol);
    EXPECT_NEAR(m[1], 222, kTol);
    EXPECT_NEAR(m[2], 333, kTol);
    auto p = t.toProgramXYZ(111, 222, 333);
    EXPECT_NEAR(p[0], 0, kTol);
    EXPECT_NEAR(p[1], 0, kTol);
    EXPECT_NEAR(p[2], 0, kTol);
}

// ============================================================================
// Scaling
// ============================================================================

TEST(CoordinateTransform, UniformScale) {
    CoordinateTransform t;
    t.setScale(2, 2, 2);
    auto m = t.toMachineXYZ(10, 20, 30);
    EXPECT_NEAR(m[0], 20, kTol);
    EXPECT_NEAR(m[1], 40, kTol);
    EXPECT_NEAR(m[2], 60, kTol);
    auto p = t.toProgramXYZ(20, 40, 60);
    EXPECT_NEAR(p[0], 10, kTol);
    EXPECT_NEAR(p[1], 20, kTol);
    EXPECT_NEAR(p[2], 30, kTol);
}

TEST(CoordinateTransform, PerAxisScale) {
    CoordinateTransform t;
    t.setScale(2, 0.5, 1);
    auto m = t.toMachineXYZ(10, 20, 30);
    EXPECT_NEAR(m[0], 20, kTol);
    EXPECT_NEAR(m[1], 10, kTol);
    EXPECT_NEAR(m[2], 30, kTol);
}

TEST(CoordinateTransform, ClearScale) {
    CoordinateTransform t;
    t.setScale(3, 3, 3);
    EXPECT_FALSE(t.isIdentity());
    t.clearScale();
    auto m = t.toMachineXYZ(10, 10, 10);
    EXPECT_NEAR(m[0], 10, kTol);
    EXPECT_NEAR(m[1], 10, kTol);
    EXPECT_NEAR(m[2], 10, kTol);
}

// ============================================================================
// 2D rotation
// ============================================================================

TEST(CoordinateTransform, Rotation2D_XY_90AboutOrigin) {
    CoordinateTransform t;
    t.setRotation2D(90, Plane::XY, 0, 0);
    // (1, 0, 0) rotated 90 deg CCW about Z -> (0, 1, 0)
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 1, kTol);
    EXPECT_NEAR(m[2], 0, kTol);
}

TEST(CoordinateTransform, Rotation2D_XY_45AboutOrigin) {
    CoordinateTransform t;
    t.setRotation2D(45, Plane::XY, 0, 0);
    const double s = std::sqrt(2.0) / 2.0;
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], s, kTol);
    EXPECT_NEAR(m[1], s, kTol);
    EXPECT_NEAR(m[2], 0, kTol);
}

TEST(CoordinateTransform, Rotation2D_AboutPivot) {
    CoordinateTransform t;
    // Rotate 90 deg about pivot (10, 0, 0) in XY.
    t.setRotation2D(90, Plane::XY, 10, 0);
    // Point (10, 10, 0) relative to pivot is (0, 10, 0).
    // Rotated 90 CCW -> (-10, 0, 0). Add pivot back -> (0, 0, 0).
    auto m = t.toMachineXYZ(10, 10, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 0, kTol);
    EXPECT_NEAR(m[2], 0, kTol);
    // Inverse round trip
    auto p = t.toProgramXYZ(0, 0, 0);
    EXPECT_NEAR(p[0], 10, kTol);
    EXPECT_NEAR(p[1], 10, kTol);
    EXPECT_NEAR(p[2], 0,  kTol);
}

TEST(CoordinateTransform, Rotation2D_ZXPlane) {
    // G18 (ZX plane): rotation about Y.
    CoordinateTransform t;
    t.setRotation2D(90, Plane::ZX, 0, 0);
    // (1, 0, 0) rotated 90 deg about Y -> (0, 0, -1) (right-hand rule)
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 0, kTol);
    EXPECT_NEAR(m[2], -1, kTol);
}

TEST(CoordinateTransform, Rotation2D_YZPlane) {
    // G19 (YZ plane): rotation about X.
    CoordinateTransform t;
    t.setRotation2D(90, Plane::YZ, 0, 0);
    // (0, 1, 0) rotated 90 deg about X -> (0, 0, 1) (right-hand rule)
    auto m = t.toMachineXYZ(0, 1, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 0, kTol);
    EXPECT_NEAR(m[2], 1, kTol);
}

TEST(CoordinateTransform, ClearRotation) {
    CoordinateTransform t;
    t.setRotation2D(45, Plane::XY, 0, 0);
    EXPECT_EQ(t.rotationMode(), RotationMode::PLANE_2D);
    t.clearRotation();
    EXPECT_EQ(t.rotationMode(), RotationMode::NONE);
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 1, kTol);
    EXPECT_NEAR(m[1], 0, kTol);
}

// ============================================================================
// 3D rotation (Euler)
// ============================================================================

TEST(CoordinateTransform, Rotation3D_Euler_X90) {
    CoordinateTransform t;
    t.setRotation3DEuler(90, 0, 0, {0, 0, 0});
    // (0, 1, 0) rotated 90 deg about X -> (0, 0, 1)
    auto m = t.toMachineXYZ(0, 1, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 0, kTol);
    EXPECT_NEAR(m[2], 1, kTol);
}

TEST(CoordinateTransform, Rotation3D_Euler_Y90) {
    CoordinateTransform t;
    t.setRotation3DEuler(0, 90, 0, {0, 0, 0});
    // (1, 0, 0) rotated 90 deg about Y -> (0, 0, -1)
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 0, kTol);
    EXPECT_NEAR(m[2], -1, kTol);
}

TEST(CoordinateTransform, Rotation3D_Euler_Z90) {
    CoordinateTransform t;
    t.setRotation3DEuler(0, 0, 90, {0, 0, 0});
    // (1, 0, 0) rotated 90 deg about Z -> (0, 1, 0)
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 1, kTol);
    EXPECT_NEAR(m[2], 0, kTol);
}

TEST(CoordinateTransform, Rotation3D_Euler_RoundTrip) {
    CoordinateTransform t;
    t.setRotation3DEuler(30, 45, 60, {5, -3, 2});
    auto p = std::array{10.0, 20.0, 30.0};
    auto m = t.toMachineXYZ(p[0], p[1], p[2]);
    auto r = t.toProgramXYZ(m[0], m[1], m[2]);
    EXPECT_NEAR(r[0], p[0], kTol);
    EXPECT_NEAR(r[1], p[1], kTol);
    EXPECT_NEAR(r[2], p[2], kTol);
}

// ============================================================================
// 3D rotation (axis-angle)
// ============================================================================

TEST(CoordinateTransform, Rotation3D_AxisAngle_Z) {
    CoordinateTransform t;
    // Rotate 90 deg about Z axis -> same as 2D XY 90
    t.setRotation3DAxisAngle({0, 0, 1}, 90, {0, 0, 0});
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 1, kTol);
    EXPECT_NEAR(m[2], 0, kTol);
}

TEST(CoordinateTransform, Rotation3D_AxisAngle_Arbitrary) {
    CoordinateTransform t;
    // Rotate 180 deg about (1,1,0) normalized.
    t.setRotation3DAxisAngle({1, 1, 0}, 180, {0, 0, 0});
    // (1, 0, 0) -> reflected across the axis line.
    // For 180 about (1,1,0)/sqrt(2): (1,0,0) -> (0,1,0)
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 0, kTol);
    EXPECT_NEAR(m[1], 1, kTol);
    EXPECT_NEAR(m[2], 0, kTol);
}

TEST(CoordinateTransform, Rotation3D_AxisAngle_RoundTrip) {
    CoordinateTransform t;
    t.setRotation3DAxisAngle({1, 2, 3}, 47.5, {-2, 4, 1});
    auto p = std::array{15.0, -8.0, 22.0};
    auto m = t.toMachineXYZ(p[0], p[1], p[2]);
    auto r = t.toProgramXYZ(m[0], m[1], m[2]);
    EXPECT_NEAR(r[0], p[0], kTol);
    EXPECT_NEAR(r[1], p[1], kTol);
    EXPECT_NEAR(r[2], p[2], kTol);
}

TEST(CoordinateTransform, AxisAngle_ZeroAxisIsIdentity) {
    CoordinateTransform t;
    t.setRotation3DAxisAngle({0, 0, 0}, 90, {0, 0, 0});
    auto m = t.toMachineXYZ(1, 2, 3);
    EXPECT_NEAR(m[0], 1, kTol);
    EXPECT_NEAR(m[1], 2, kTol);
    EXPECT_NEAR(m[2], 3, kTol);
}

// ============================================================================
// Composed transforms
// ============================================================================

TEST(CoordinateTransform, ScaleThenRotateThenOffset) {
    // Scale by 2, rotate 90 about Z, offset by (100, 200, 300).
    CoordinateTransform t;
    t.setScale(2, 2, 2);
    t.setRotation2D(90, Plane::XY, 0, 0);
    t.setWCSOffset({100, 200, 300});
    // (1, 0, 0) -> scale -> (2, 0, 0) -> rotate 90 -> (0, 2, 0) -> offset -> (100, 202, 300)
    auto m = t.toMachineXYZ(1, 0, 0);
    EXPECT_NEAR(m[0], 100, kTol);
    EXPECT_NEAR(m[1], 202, kTol);
    EXPECT_NEAR(m[2], 300, kTol);
    auto p = t.toProgramXYZ(100, 202, 300);
    EXPECT_NEAR(p[0], 1, kTol);
    EXPECT_NEAR(p[1], 0, kTol);
    EXPECT_NEAR(p[2], 0, kTol);
}

TEST(CoordinateTransform, FullComposedRoundTrip) {
    CoordinateTransform t;
    t.setG52Offset({1, 2, 3});
    t.setG92Offset({10, 20, 30});
    t.setWCSOffset({100, 200, 300});
    t.setScale(1.5, 0.8, 2.0);
    t.setRotation3DEuler(25, 35, 55, {7, -4, 11});
    auto p = std::array{42.0, -17.0, 99.0};
    auto m = t.toMachineXYZ(p[0], p[1], p[2]);
    auto r = t.toProgramXYZ(m[0], m[1], m[2]);
    EXPECT_NEAR(r[0], p[0], 1e-7);
    EXPECT_NEAR(r[1], p[1], 1e-7);
    EXPECT_NEAR(r[2], p[2], 1e-7);
}

// ============================================================================
// Velocity transform
// ============================================================================

TEST(CoordinateTransform, VelocityIdentity) {
    CoordinateTransform t;
    auto v = t.transformVelocity(10, 20, 30);
    EXPECT_NEAR(v[0], 10, kTol);
    EXPECT_NEAR(v[1], 20, kTol);
    EXPECT_NEAR(v[2], 30, kTol);
}

TEST(CoordinateTransform, VelocityScaleOnly) {
    CoordinateTransform t;
    t.setScale(2, 3, 4);
    auto v = t.transformVelocity(1, 1, 1);
    EXPECT_NEAR(v[0], 2, kTol);
    EXPECT_NEAR(v[1], 3, kTol);
    EXPECT_NEAR(v[2], 4, kTol);
}

TEST(CoordinateTransform, VelocityRotateOnly) {
    CoordinateTransform t;
    t.setRotation2D(90, Plane::XY, 100, 200); // pivot should not matter
    auto v = t.transformVelocity(1, 0, 0);
    EXPECT_NEAR(v[0], 0, kTol);
    EXPECT_NEAR(v[1], 1, kTol);
    EXPECT_NEAR(v[2], 0, kTol);
}

TEST(CoordinateTransform, VelocityScaleAndRotate) {
    CoordinateTransform t;
    t.setScale(2, 2, 2);
    t.setRotation2D(90, Plane::XY, 50, 50);
    auto v = t.transformVelocity(1, 0, 0);
    EXPECT_NEAR(v[0], 0, kTol);
    EXPECT_NEAR(v[1], 2, kTol);
    EXPECT_NEAR(v[2], 0, kTol);
}

// ============================================================================
// Full Position (9-axis)
// ============================================================================

TEST(CoordinateTransform, FullPositionXYZTransformed) {
    CoordinateTransform t;
    t.setWCSOffset({10, 20, 30});
    t.setScale(2, 2, 2);
    Position p;
    p.x() = 5; p.y() = 5; p.z() = 5;
    p[3] = 100; // A
    p[6] = 50;  // U
    auto m = t.toMachine(p);
    EXPECT_NEAR(m.x(), 20, kTol); // (5*2)+10
    EXPECT_NEAR(m.y(), 30, kTol); // (5*2)+20
    EXPECT_NEAR(m.z(), 40, kTol); // (5*2)+30
    // A and U: scale only (default ext scale = 1)
    EXPECT_NEAR(m[3], 100, kTol);
    EXPECT_NEAR(m[6], 50, kTol);
}

TEST(CoordinateTransform, FullPositionExtendedScale) {
    CoordinateTransform t;
    t.setExtendedScale({2, 3, 4, 5, 6, 7}); // A,B,C,U,V,W
    Position p;
    p[3] = 10; // A -> 20
    p[4] = 10; // B -> 30
    p[5] = 10; // C -> 40
    p[6] = 10; // U -> 50
    p[7] = 10; // V -> 60
    p[8] = 10; // W -> 70
    auto m = t.toMachine(p);
    EXPECT_NEAR(m[3], 20, kTol);
    EXPECT_NEAR(m[4], 30, kTol);
    EXPECT_NEAR(m[5], 40, kTol);
    EXPECT_NEAR(m[6], 50, kTol);
    EXPECT_NEAR(m[7], 60, kTol);
    EXPECT_NEAR(m[8], 70, kTol);
    auto r = t.toProgram(m);
    EXPECT_NEAR(r[3], 10, kTol);
    EXPECT_NEAR(r[4], 10, kTol);
    EXPECT_NEAR(r[5], 10, kTol);
    EXPECT_NEAR(r[6], 10, kTol);
    EXPECT_NEAR(r[7], 10, kTol);
    EXPECT_NEAR(r[8], 10, kTol);
}

TEST(CoordinateTransform, FullPositionRoundTrip) {
    CoordinateTransform t;
    t.setWCSOffset({10, 20, 30});
    t.setScale(1.5, 0.8, 2.0);
    t.setRotation3DEuler(20, 30, 40, {1, 2, 3});
    t.setExtendedScale({2, 3, 4, 5, 6, 7});
    Position p;
    p.x() = 42; p.y() = -17; p.z() = 99;
    p[3] = 100; p[4] = 200; p[5] = 300;
    p[6] = 400; p[7] = 500; p[8] = 600;
    auto m = t.toMachine(p);
    auto r = t.toProgram(m);
    EXPECT_NEAR(r.x(), 42, 1e-7);
    EXPECT_NEAR(r.y(), -17, 1e-7);
    EXPECT_NEAR(r.z(), 99, 1e-7);
    EXPECT_NEAR(r[3], 100, kTol);
    EXPECT_NEAR(r[4], 200, kTol);
    EXPECT_NEAR(r[5], 300, kTol);
    EXPECT_NEAR(r[6], 400, kTol);
    EXPECT_NEAR(r[7], 500, kTol);
    EXPECT_NEAR(r[8], 600, kTol);
}

// ============================================================================
// Reset
// ============================================================================

TEST(CoordinateTransform, ResetClearsEverything) {
    CoordinateTransform t;
    t.setWCSOffset({1, 2, 3});
    t.setG52Offset({4, 5, 6});
    t.setG92Offset({7, 8, 9});
    t.setScale(2, 3, 4);
    t.setExtendedScale({2, 2, 2, 2, 2, 2});
    t.setRotation3DEuler(10, 20, 30, {1, 2, 3});
    EXPECT_FALSE(t.isIdentity());
    t.reset();
    EXPECT_TRUE(t.isIdentity());
    auto m = t.toMachineXYZ(10, 20, 30);
    EXPECT_NEAR(m[0], 10, kTol);
    EXPECT_NEAR(m[1], 20, kTol);
    EXPECT_NEAR(m[2], 30, kTol);
}

// ============================================================================
// Tool length offset (G43)
// ============================================================================

TEST(CoordinateTransform, ToolLengthOffset) {
    CoordinateTransform t;
    t.setToolLengthOffset(25.0);
    // Z is shifted by TLO, X/Y unchanged
    auto m = t.toMachineXYZ(10, 20, 30);
    EXPECT_NEAR(m[0], 10, kTol);
    EXPECT_NEAR(m[1], 20, kTol);
    EXPECT_NEAR(m[2], 55, kTol);  // 30 + 25
}

TEST(CoordinateTransform, ToolLengthOffsetClear) {
    CoordinateTransform t;
    t.setToolLengthOffset(25.0);
    t.clearToolLengthOffset();
    auto m = t.toMachineXYZ(10, 20, 30);
    EXPECT_NEAR(m[0], 10, kTol);
    EXPECT_NEAR(m[1], 20, kTol);
    EXPECT_NEAR(m[2], 30, kTol);
}

TEST(CoordinateTransform, ToolLengthOffsetWithWCS) {
    // TLO is applied after WCS: machine = WCS + TLO + program
    CoordinateTransform t;
    t.setWCSOffset({100, 200, 300});
    t.setToolLengthOffset(50.0);
    auto m = t.toMachineXYZ(0, 0, 0);
    EXPECT_NEAR(m[0], 100, kTol);
    EXPECT_NEAR(m[1], 200, kTol);
    EXPECT_NEAR(m[2], 350, kTol);  // 300 + 50
}

TEST(CoordinateTransform, ToolLengthOffsetRoundTrip) {
    CoordinateTransform t;
    t.setWCSOffset({10, 20, 30});
    t.setToolLengthOffset(15.0);
    t.setScale(2, 2, 2);
    t.setRotation2D(45, Plane::XY, 5, 5);
    auto m = t.toMachineXYZ(3, 4, 5);
    auto p = t.toProgramXYZ(m[0], m[1], m[2]);
    EXPECT_NEAR(p[0], 3, 1e-7);
    EXPECT_NEAR(p[1], 4, 1e-7);
    EXPECT_NEAR(p[2], 5, 1e-7);
}

TEST(CoordinateTransform, ToolLengthOffsetReset) {
    CoordinateTransform t;
    t.setToolLengthOffset(25.0);
    EXPECT_FALSE(t.isIdentity());
    t.reset();
    EXPECT_TRUE(t.isIdentity());
    EXPECT_NEAR(t.toolLengthOffset(), 0.0, kTol);
}

// ============================================================================
// RS274 transform order: G52 after WCS+rotation
// ============================================================================

TEST(CoordinateTransform, G52AfterWCS_NoRotation) {
    // Without rotation, G52 + WCS + program all add up (commutative)
    CoordinateTransform t;
    t.setG52Offset({1, 2, 3});
    t.setWCSOffset({100, 200, 300});
    auto m = t.toMachineXYZ(10, 20, 30);
    EXPECT_NEAR(m[0], 111, kTol);  // 10 + 100 + 1
    EXPECT_NEAR(m[1], 222, kTol);
    EXPECT_NEAR(m[2], 333, kTol);
}

TEST(CoordinateTransform, G52AfterWCSWithRotation) {
    // With rotation, G52 is applied AFTER rotation+WCS.
    // Transform: T(g52) * T(wcs) * R * P
    // Point (0,0,0): R*(0,0,0)=(0,0,0), +WCS=(100,200,0), +G52=(101,202,0)
    CoordinateTransform t;
    t.setG52Offset({1, 2, 0});
    t.setWCSOffset({100, 200, 0});
    t.setRotation2D(90, Plane::XY, 0, 0);
    auto m = t.toMachineXYZ(0, 0, 0);
    EXPECT_NEAR(m[0], 101, kTol);
    EXPECT_NEAR(m[1], 202, kTol);
    // Point (10, 0, 0): R*(10,0,0)=(0,10,0), +WCS=(100,210,0), +G52=(101,212,0)
    auto m2 = t.toMachineXYZ(10, 0, 0);
    EXPECT_NEAR(m2[0], 101, kTol);
    EXPECT_NEAR(m2[1], 212, kTol);
}

TEST(CoordinateTransform, G52NotAffectedByScale) {
    // G52 is applied after scale, so G52 offset is NOT scaled.
    // Transform: T(g52) * S * P
    // Point (10,0,0): S*(10,0,0)=(20,0,0), +G52=(22,1,0)
    CoordinateTransform t;
    t.setG52Offset({2, 1, 0});
    t.setScale(2, 2, 1);
    auto m = t.toMachineXYZ(10, 0, 0);
    EXPECT_NEAR(m[0], 22, kTol);  // 10*2 + 2
    EXPECT_NEAR(m[1], 1, kTol);   // 0*2 + 1
}

TEST(CoordinateTransform, G92BeforeScale) {
    // G92 is applied BEFORE scale, so G92 offset IS scaled.
    // Transform: S * T(g92) * P
    // Point (0,0,0): T(g92)=(10,0,0), S=(20,0,0)
    CoordinateTransform t;
    t.setG92Offset({10, 0, 0});
    t.setScale(2, 2, 1);
    auto m = t.toMachineXYZ(0, 0, 0);
    EXPECT_NEAR(m[0], 20, kTol);  // (0+10)*2
}

TEST(CoordinateTransform, G52AfterTLO) {
    // G52 is applied after TLO.
    // Transform: T(g52) * T(tlo) * P
    CoordinateTransform t;
    t.setG52Offset({5, 0, 0});
    t.setToolLengthOffset(10.0);
    auto m = t.toMachineXYZ(0, 0, 5);
    EXPECT_NEAR(m[0], 5, kTol);   // 0 + 5 (G52)
    EXPECT_NEAR(m[1], 0, kTol);
    EXPECT_NEAR(m[2], 15, kTol);  // 5 + 10 (TLO)
}

// ============================================================================
// Full composed transform with TLO and G52 in correct order
// ============================================================================

TEST(CoordinateTransform, FullComposedWithTLO) {
    // Full transform: T(g52) * T(tlo) * T(wcs) * T(pivot) * R * T(-pivot) * S * T(g92)
    CoordinateTransform t;
    t.setG92Offset({1, 2, 3});
    t.setScale(2, 2, 2);
    t.setRotation2D(90, Plane::XY, 0, 0);
    t.setWCSOffset({100, 200, 300});
    t.setToolLengthOffset(50.0);
    t.setG52Offset({10, 20, 30});

    // Point (0,0,0):
    // T(g92): (1,2,3)
    // S: (2,4,6)
    // R(90° about Z): (-4,2,6)
    // T(wcs): (96,202,306)
    // T(tlo): (96,202,356)
    // T(g52): (106,222,386)
    auto m = t.toMachineXYZ(0, 0, 0);
    EXPECT_NEAR(m[0], 106, kTol);
    EXPECT_NEAR(m[1], 222, kTol);
    EXPECT_NEAR(m[2], 386, kTol);

    // Round trip
    auto p = t.toProgramXYZ(m[0], m[1], m[2]);
    EXPECT_NEAR(p[0], 0, 1e-7);
    EXPECT_NEAR(p[1], 0, 1e-7);
    EXPECT_NEAR(p[2], 0, 1e-7);
}
