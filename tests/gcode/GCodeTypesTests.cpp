/**
 * @file GCodeTypesTests.cpp
 * @brief Comprehensive unit tests for GCode Types
 *
 * This file provides complete coverage for GCode type definitions:
 * - Position class operations
 * - Axis enumeration
 * - Error codes and handling
 * - WordLetter utilities
 * - Motion modes
 * - Constants
 */

#include <gtest/gtest.h>
#include <tether/gcode/GCodeTypes.hpp>
#include <cmath>
#include <limits>

namespace GCode {
namespace test {

// ============================================================================
// Position Tests
// ============================================================================

class PositionTest : public ::testing::Test {
protected:
    Position origin;
    Position p1, p2;

    void SetUp() override {
        origin = Position{};
        p1.coords = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
        p2.coords = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0};
    }
};

TEST_F(PositionTest, DefaultConstruction) {
    Position p;
    for (size_t i = 0; i < MAX_AXES; ++i) {
        EXPECT_DOUBLE_EQ(p.coords[i], 0.0);
    }
}

TEST_F(PositionTest, AxisIndexing) {
    EXPECT_DOUBLE_EQ(p1[Axis::X], 1.0);
    EXPECT_DOUBLE_EQ(p1[Axis::Y], 2.0);
    EXPECT_DOUBLE_EQ(p1[Axis::Z], 3.0);
    EXPECT_DOUBLE_EQ(p1[Axis::A], 4.0);
    EXPECT_DOUBLE_EQ(p1[Axis::B], 5.0);
    EXPECT_DOUBLE_EQ(p1[Axis::C], 6.0);
    EXPECT_DOUBLE_EQ(p1[Axis::U], 7.0);
    EXPECT_DOUBLE_EQ(p1[Axis::V], 8.0);
    EXPECT_DOUBLE_EQ(p1[Axis::W], 9.0);
}

TEST_F(PositionTest, IntegerIndexing) {
    EXPECT_DOUBLE_EQ(p1[0], 1.0);
    EXPECT_DOUBLE_EQ(p1[1], 2.0);
    EXPECT_DOUBLE_EQ(p1[2], 3.0);
}

TEST_F(PositionTest, XYZAccessors) {
    EXPECT_DOUBLE_EQ(p1.x(), 1.0);
    EXPECT_DOUBLE_EQ(p1.y(), 2.0);
    EXPECT_DOUBLE_EQ(p1.z(), 3.0);
}

TEST_F(PositionTest, XYZMutators) {
    Position p;
    p.x() = 100.0;
    p.y() = 200.0;
    p.z() = 300.0;

    EXPECT_DOUBLE_EQ(p.x(), 100.0);
    EXPECT_DOUBLE_EQ(p.y(), 200.0);
    EXPECT_DOUBLE_EQ(p.z(), 300.0);
}

TEST_F(PositionTest, Addition) {
    Position result = p1 + p2;

    EXPECT_DOUBLE_EQ(result[Axis::X], 11.0);
    EXPECT_DOUBLE_EQ(result[Axis::Y], 22.0);
    EXPECT_DOUBLE_EQ(result[Axis::Z], 33.0);
}

TEST_F(PositionTest, Subtraction) {
    Position result = p2 - p1;

    EXPECT_DOUBLE_EQ(result[Axis::X], 9.0);
    EXPECT_DOUBLE_EQ(result[Axis::Y], 18.0);
    EXPECT_DOUBLE_EQ(result[Axis::Z], 27.0);
}

TEST_F(PositionTest, ScalarMultiplication) {
    Position result = p1 * 2.0;

    EXPECT_DOUBLE_EQ(result[Axis::X], 2.0);
    EXPECT_DOUBLE_EQ(result[Axis::Y], 4.0);
    EXPECT_DOUBLE_EQ(result[Axis::Z], 6.0);
}

TEST_F(PositionTest, DotProduct) {
    Position a, b;
    a.coords = {1.0, 2.0, 3.0, 0, 0, 0, 0, 0, 0};
    b.coords = {4.0, 5.0, 6.0, 0, 0, 0, 0, 0, 0};

    double dot = a.dot(b);
    EXPECT_DOUBLE_EQ(dot, 1*4 + 2*5 + 3*6); // 32
}

TEST_F(PositionTest, Magnitude) {
    Position p;
    p.coords = {3.0, 4.0, 0, 0, 0, 0, 0, 0, 0};

    EXPECT_DOUBLE_EQ(p.magnitude(), 5.0);
}

TEST_F(PositionTest, MagnitudeZero) {
    Position p;
    EXPECT_DOUBLE_EQ(p.magnitude(), 0.0);
}

TEST_F(PositionTest, Normalized) {
    Position p;
    p.coords = {3.0, 4.0, 0, 0, 0, 0, 0, 0, 0};

    Position n = p.normalized();
    EXPECT_NEAR(n[Axis::X], 0.6, 1e-9);
    EXPECT_NEAR(n[Axis::Y], 0.8, 1e-9);
}

TEST_F(PositionTest, NormalizedZeroVector) {
    Position p;
    Position n = p.normalized();

    // Should return zero vector without division by zero
    EXPECT_DOUBLE_EQ(n.magnitude(), 0.0);
}

TEST_F(PositionTest, LinearDistance) {
    Position a, b;
    a.coords = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    b.coords = {3, 4, 0, 0, 0, 0, 0, 0, 0};

    EXPECT_DOUBLE_EQ(a.linearDistance(b), 5.0);
}

TEST_F(PositionTest, LinearDistanceIgnoresRotary) {
    Position a, b;
    a.coords = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    b.coords = {3, 4, 0, 100, 200, 300, 0, 0, 0}; // A, B, C are large

    // Should only consider X, Y, Z
    EXPECT_DOUBLE_EQ(a.linearDistance(b), 5.0);
}

TEST_F(PositionTest, LinearDistance3D) {
    Position a, b;
    a.coords = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    b.coords = {1, 2, 2, 0, 0, 0, 0, 0, 0};

    EXPECT_DOUBLE_EQ(a.linearDistance(b), 3.0);
}

// ============================================================================
// Axis Tests
// ============================================================================

TEST(AxisTest, AxisCount) {
    EXPECT_EQ(static_cast<size_t>(Axis::COUNT), 9u);
    EXPECT_EQ(MAX_AXES, 9u);
}

TEST(AxisTest, AxisValues) {
    EXPECT_EQ(static_cast<uint8_t>(Axis::X), 0);
    EXPECT_EQ(static_cast<uint8_t>(Axis::Y), 1);
    EXPECT_EQ(static_cast<uint8_t>(Axis::Z), 2);
    EXPECT_EQ(static_cast<uint8_t>(Axis::A), 3);
    EXPECT_EQ(static_cast<uint8_t>(Axis::B), 4);
    EXPECT_EQ(static_cast<uint8_t>(Axis::C), 5);
    EXPECT_EQ(static_cast<uint8_t>(Axis::U), 6);
    EXPECT_EQ(static_cast<uint8_t>(Axis::V), 7);
    EXPECT_EQ(static_cast<uint8_t>(Axis::W), 8);
}

// ============================================================================
// Constants Tests
// ============================================================================

TEST(ConstantsTest, MaxAxes) {
    EXPECT_EQ(MAX_AXES, 9u);
}

TEST(ConstantsTest, MaxCoordSystems) {
    EXPECT_EQ(MAX_COORD_SYSTEMS, 9u);
}

TEST(ConstantsTest, MaxTools) {
    EXPECT_EQ(MAX_TOOLS, 256u);
}

TEST(ConstantsTest, MaxCallDepth) {
    EXPECT_EQ(MAX_CALL_DEPTH, 10u);
}

TEST(ConstantsTest, MaxLocalParams) {
    EXPECT_EQ(MAX_LOCAL_PARAMS, 30u);
}

TEST(ConstantsTest, MaxGlobalParams) {
    EXPECT_EQ(MAX_GLOBAL_PARAMS, 6000u);
}

TEST(ConstantsTest, MaxLookahead) {
    EXPECT_EQ(MAX_LOOKAHEAD, 50u);
}

TEST(ConstantsTest, MaxLookbehind) {
    EXPECT_EQ(MAX_LOOKBEHIND, 20u);
}

TEST(ConstantsTest, ArcRadiusTolerance) {
    EXPECT_DOUBLE_EQ(ARC_RADIUS_TOLERANCE, 0.005);
}

TEST(ConstantsTest, Epsilon) {
    EXPECT_DOUBLE_EQ(EPSILON, 1e-9);
}

// ============================================================================
// MotionMode Tests
// ============================================================================

TEST(MotionModeTest, MotionModeValues) {
    EXPECT_EQ(static_cast<uint16_t>(MotionMode::RAPID), 0);
    EXPECT_EQ(static_cast<uint16_t>(MotionMode::LINEAR), 1);
    EXPECT_EQ(static_cast<uint16_t>(MotionMode::CW_ARC), 2);
    EXPECT_EQ(static_cast<uint16_t>(MotionMode::CCW_ARC), 3);
    EXPECT_EQ(static_cast<uint16_t>(MotionMode::DWELL), 4);
    EXPECT_EQ(static_cast<uint16_t>(MotionMode::THREADING), 33);
}

// ============================================================================
// Error Tests
// ============================================================================

TEST(ErrorTest, DefaultError) {
    Error err;
    EXPECT_EQ(err.code, ErrorCode::OK);
    EXPECT_EQ(err.line, 0u);
}

TEST(ErrorTest, OkCheck) {
    Error err;
    EXPECT_TRUE(err.ok());
    EXPECT_FALSE(err);

    err.code = ErrorCode::SYNTAX_ERROR;
    EXPECT_FALSE(err.ok());
    EXPECT_TRUE(static_cast<bool>(err));
}

TEST(ErrorTest, ErrorCodeValues) {
    // Verify error codes exist
    EXPECT_EQ(static_cast<int>(ErrorCode::OK), 0);
    EXPECT_NE(static_cast<int>(ErrorCode::SYNTAX_ERROR), 0);
    EXPECT_NE(static_cast<int>(ErrorCode::INVALID_LINE_NUMBER), 0);
    EXPECT_NE(static_cast<int>(ErrorCode::INVALID_OCODE), 0);
    EXPECT_NE(static_cast<int>(ErrorCode::UNDEFINED_SUBROUTINE), 0);
    EXPECT_NE(static_cast<int>(ErrorCode::FILE_NOT_FOUND), 0);
    EXPECT_NE(static_cast<int>(ErrorCode::END), 0);
}

// ============================================================================
// WordLetter Tests
// ============================================================================

TEST(WordLetterTest, WordLetterValues) {
    // A through Z should be 0-25
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::A), 0);
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::B), 1);
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::C), 2);
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::G), 6);
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::M), 12);
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::X), 23);
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::Y), 24);
    EXPECT_EQ(static_cast<uint8_t>(WordLetter::Z), 25);
}

TEST(WordLetterTest, SpecialWordLetters) {
    // INVALID should be distinguishable
    EXPECT_NE(WordLetter::INVALID, WordLetter::A);
}

// ============================================================================
// OCodeType Tests
// ============================================================================

TEST(OCodeTypeTest, OCodeTypeValues) {
    // Verify O-code types exist
    EXPECT_EQ(static_cast<int>(OCodeType::SUB), 0);
    EXPECT_NE(static_cast<int>(OCodeType::ENDSUB), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::CALL), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::IF), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::ELSE), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::ENDIF), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::WHILE), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::ENDWHILE), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::DO), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::RETURN), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::BREAK), static_cast<int>(OCodeType::SUB));
    EXPECT_NE(static_cast<int>(OCodeType::CONTINUE), static_cast<int>(OCodeType::SUB));
}

// ============================================================================
// Block Tests
// ============================================================================

TEST(BlockTest, DefaultConstruction) {
    Block block;

    EXPECT_FALSE(block.hasComment);
    EXPECT_FALSE(block.hasOCode);
    EXPECT_FALSE(block.blockDelete);
    EXPECT_EQ(block.sourceLineNumber, 0u);
}

TEST(BlockTest, HasWord) {
    Block block;
    block.words[static_cast<size_t>(WordLetter::X)].present = true;

    EXPECT_TRUE(block.hasWord(WordLetter::X));
    EXPECT_FALSE(block.hasWord(WordLetter::Y));
}

TEST(BlockTest, GetSetWord) {
    Block block;
    block.words[static_cast<size_t>(WordLetter::X)].present = true;
    block.words[static_cast<size_t>(WordLetter::X)].value = 100.0;

    EXPECT_TRUE(block.hasWord(WordLetter::X));
    EXPECT_DOUBLE_EQ(block.getWord(WordLetter::X), 100.0);
}

TEST(BlockTest, GetWordDefault) {
    Block block;

    // Should return default value for unset word
    EXPECT_DOUBLE_EQ(block.getWord(WordLetter::X), 0.0);
    EXPECT_DOUBLE_EQ(block.getWord(WordLetter::Y, 99.0), 99.0);
}

TEST(BlockTest, CommentStorage) {
    Block block;
    block.hasComment = true;
    std::snprintf(block.comment.data(), block.comment.size(), "test comment");

    EXPECT_STREQ(block.comment.data(), "test comment");
}

TEST(BlockTest, OCodeStorage) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeIsNamed = false;

    EXPECT_EQ(block.oCodeNumber, 100);
    EXPECT_FALSE(block.oCodeIsNamed);
}

TEST(BlockTest, OriginalTextStorage) {
    Block block;
    std::snprintf(block.originalText.data(), block.originalText.size(), "G1 X100 F1000");

    EXPECT_STREQ(block.originalText.data(), "G1 X100 F1000");
}

// ============================================================================
// Position Arithmetic Edge Cases
// ============================================================================

TEST(PositionEdgeCaseTest, VeryLargeValues) {
    Position p;
    p.coords = {1e15, 1e15, 1e15, 0, 0, 0, 0, 0, 0};

    Position doubled = p * 2.0;
    EXPECT_DOUBLE_EQ(doubled[Axis::X], 2e15);
}

TEST(PositionEdgeCaseTest, VerySmallValues) {
    Position p;
    p.coords = {1e-15, 1e-15, 1e-15, 0, 0, 0, 0, 0, 0};

    EXPECT_GT(p.magnitude(), 0.0);
}

TEST(PositionEdgeCaseTest, NegativeValues) {
    Position a, b;
    a.coords = {-10.0, -20.0, -30.0, 0, 0, 0, 0, 0, 0};
    b.coords = {10.0, 20.0, 30.0, 0, 0, 0, 0, 0, 0};

    Position sum = a + b;
    for (size_t i = 0; i < MAX_AXES; ++i) {
        EXPECT_DOUBLE_EQ(sum.coords[i], 0.0);
    }
}

TEST(PositionEdgeCaseTest, MixedSigns) {
    Position p;
    p.coords = {-1.0, 2.0, -3.0, 4.0, -5.0, 6.0, -7.0, 8.0, -9.0};

    Position neg = p * -1.0;
    EXPECT_DOUBLE_EQ(neg[Axis::X], 1.0);
    EXPECT_DOUBLE_EQ(neg[Axis::Y], -2.0);
}

// ============================================================================
// Position Vector Operations
// ============================================================================

TEST(PositionVectorTest, UnitVectorX) {
    Position p;
    p.coords = {1.0, 0, 0, 0, 0, 0, 0, 0, 0};

    Position n = p.normalized();
    EXPECT_DOUBLE_EQ(n.magnitude(), 1.0);
    EXPECT_DOUBLE_EQ(n[Axis::X], 1.0);
}

TEST(PositionVectorTest, UnitVectorDiagonal) {
    Position p;
    p.coords = {1.0, 1.0, 1.0, 0, 0, 0, 0, 0, 0};

    Position n = p.normalized();
    EXPECT_NEAR(n.magnitude(), 1.0, 1e-9);
}

TEST(PositionVectorTest, OrthogonalDotProduct) {
    Position a, b;
    a.coords = {1.0, 0, 0, 0, 0, 0, 0, 0, 0};
    b.coords = {0, 1.0, 0, 0, 0, 0, 0, 0, 0};

    EXPECT_DOUBLE_EQ(a.dot(b), 0.0);
}

TEST(PositionVectorTest, ParallelDotProduct) {
    Position a, b;
    a.coords = {2.0, 0, 0, 0, 0, 0, 0, 0, 0};
    b.coords = {3.0, 0, 0, 0, 0, 0, 0, 0, 0};

    EXPECT_DOUBLE_EQ(a.dot(b), 6.0);
}

// ============================================================================
// Parameterized Position Tests
// ============================================================================

class PositionDistanceTest : public ::testing::TestWithParam<std::tuple<double, double, double, double>> {};

TEST_P(PositionDistanceTest, LinearDistanceCalculation) {
    auto [x, y, z, expected] = GetParam();

    Position a;
    Position b;
    b.coords = {x, y, z, 0, 0, 0, 0, 0, 0};

    EXPECT_NEAR(a.linearDistance(b), expected, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
    Distances,
    PositionDistanceTest,
    ::testing::Values(
        std::make_tuple(0.0, 0.0, 0.0, 0.0),
        std::make_tuple(1.0, 0.0, 0.0, 1.0),
        std::make_tuple(0.0, 1.0, 0.0, 1.0),
        std::make_tuple(0.0, 0.0, 1.0, 1.0),
        std::make_tuple(3.0, 4.0, 0.0, 5.0),
        std::make_tuple(1.0, 2.0, 2.0, 3.0),
        std::make_tuple(10.0, 0.0, 0.0, 10.0),
        std::make_tuple(-3.0, -4.0, 0.0, 5.0),
        std::make_tuple(1.0, 1.0, 1.0, std::sqrt(3.0))
    )
);

// ============================================================================
// Block WordMask Tests
// ============================================================================

TEST(BlockWordMaskTest, SetMultipleWords) {
    Block block;
    // Set X
    block.words[static_cast<size_t>(WordLetter::X)].present = true;
    block.words[static_cast<size_t>(WordLetter::X)].value = 1.0;
    // Set Y
    block.words[static_cast<size_t>(WordLetter::Y)].present = true;
    block.words[static_cast<size_t>(WordLetter::Y)].value = 2.0;
    // Set Z
    block.words[static_cast<size_t>(WordLetter::Z)].present = true;
    block.words[static_cast<size_t>(WordLetter::Z)].value = 3.0;
    // Set F
    block.words[static_cast<size_t>(WordLetter::F)].present = true;
    block.words[static_cast<size_t>(WordLetter::F)].value = 1000.0;

    EXPECT_TRUE(block.hasWord(WordLetter::X));
    EXPECT_TRUE(block.hasWord(WordLetter::Y));
    EXPECT_TRUE(block.hasWord(WordLetter::Z));
    EXPECT_TRUE(block.hasWord(WordLetter::F));
    EXPECT_FALSE(block.hasWord(WordLetter::A));
    EXPECT_FALSE(block.hasWord(WordLetter::S));
}

TEST(BlockWordMaskTest, OverwriteWord) {
    Block block;
    block.words[static_cast<size_t>(WordLetter::X)].present = true;
    block.words[static_cast<size_t>(WordLetter::X)].value = 100.0;
    block.words[static_cast<size_t>(WordLetter::X)].value = 200.0;

    EXPECT_DOUBLE_EQ(block.getWord(WordLetter::X), 200.0);
}

} // namespace test
} // namespace GCode
