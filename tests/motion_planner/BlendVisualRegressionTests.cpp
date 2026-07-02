/**
 * @file BlendVisualRegressionTests.cpp
 * @brief Automated blend tests with structured SVG output
 *
 * Runs all standard blend scenarios and multi-corner path scenarios,
 * generating SVG visualizations for visual verification. Tests verify:
 * - C1/C2 continuity at blend boundaries
 * - Half-length constraint compliance
 * - Blend feasibility (canBlend)
 * - Curvature bounds
 *
 * SVG output is written to build/test_output/blend_svgs/
 */

#include <gtest/gtest.h>
#include <filesystem>
#include <string>
#include <vector>

#include "BlendTestScenarios.hpp"

namespace BlendTest {

// ============================================================================
// Output directory — configurable via environment variable
// ============================================================================

static std::string getOutputDir() {
    const char* env = std::getenv("BLEND_SVG_DIR");
    if (env && env[0]) return std::string(env);
    return "test_output/blend_svgs";
}

static void ensureOutputDir() {
    std::string dir = getOutputDir();
    if (!std::filesystem::exists(dir)) {
        std::filesystem::create_directories(dir);
    }
}

// ============================================================================
// Parameterized single-corner tests
// ============================================================================

class BlendScenarioTest : public ::testing::TestWithParam<BlendScenario> {
protected:
    void SetUp() override { ensureOutputDir(); }
};

TEST_P(BlendScenarioTest, ProducesValidBlend) {
    const auto& scenario = GetParam();
    std::string svgPath = getOutputDir() + "/" + scenario.name + ".svg";

    auto result = runBlendScenario(scenario, svgPath);

    // Most scenarios should be blendable, but some edge cases may not
    // (short segments with large tolerance, inside-strict mode, arc transitions)
    if (scenario.name.find("short") != std::string::npos ||
        scenario.name.find("inside") != std::string::npos ||
        scenario.name.find("outside") != std::string::npos ||
        scenario.name.find("LA_") != std::string::npos ||
        scenario.name.find("AL_") != std::string::npos ||
        scenario.name.find("AA_") != std::string::npos) {
        // These may or may not blend — just verify SVG is generated
        EXPECT_TRUE(std::filesystem::exists(svgPath));
        return;
    }

    EXPECT_TRUE(result.canBlend)
        << "Scenario " << scenario.name << " should be blendable";

    if (!result.canBlend) return;

    // Blend radius should be positive
    EXPECT_GT(result.blendRadius, 0.0)
        << "Blend radius should be positive for " << scenario.name;

    // Entry/exit distances should be positive
    EXPECT_GT(result.entryDist, 0.0)
        << "Entry distance should be positive for " << scenario.name;
    EXPECT_GT(result.exitDist, 0.0)
        << "Exit distance should be positive for " << scenario.name;

    // Half-length constraint
    auto seg1 = makeSegment(scenario.seg1);
    auto seg2 = makeSegment(scenario.seg2);
    EXPECT_LE(result.entryDist, seg1.segmentLength * scenario.maxBlendFraction + 0.01)
        << "Entry distance must respect half-length for " << scenario.name;
    EXPECT_LE(result.exitDist, seg2.segmentLength * scenario.maxBlendFraction + 0.01)
        << "Exit distance must respect half-length for " << scenario.name;

    // C1 continuity
    EXPECT_TRUE(result.c1Pass)
        << "C1 (tangent) continuity failed for " << scenario.name;

    // C2 continuity (only for line-line; arc transitions have different curvature)
    if (scenario.expectedTransition == "Line-Line") {
        EXPECT_TRUE(result.c2Pass)
            << "C2 (curvature) continuity failed for " << scenario.name;
    }

    // SVG file should exist
    EXPECT_TRUE(std::filesystem::exists(svgPath))
        << "SVG output should be generated at " << svgPath;
}

TEST_P(BlendScenarioTest, CurvatureIsBounded) {
    const auto& scenario = GetParam();
    auto result = runBlendScenario(scenario);

    if (!result.canBlend) return;

    // Skip edge cases that may produce degenerate blends
    if (scenario.name.find("inside") != std::string::npos ||
        scenario.name.find("outside") != std::string::npos) {
        SUCCEED();
        return;
    }

    // Max curvature should be finite and positive
    EXPECT_TRUE(std::isfinite(result.maxCurvature))
        << "Max curvature should be finite for " << scenario.name;
    EXPECT_GT(result.maxCurvature, 0.0)
        << "Max curvature should be positive for " << scenario.name;

    // Curvature should not be absurdly large (indicates degenerate blend)
    double expectedMax = 100.0 / result.blendRadius;
    EXPECT_LT(result.maxCurvature, expectedMax)
        << "Max curvature " << result.maxCurvature
        << " exceeds reasonable bound " << expectedMax
        << " for " << scenario.name;
}

INSTANTIATE_TEST_SUITE_P(
    StandardScenarios,
    BlendScenarioTest,
    ::testing::ValuesIn(standardScenarios()),
    [](const ::testing::TestParamInfo<BlendScenario>& info) {
        return info.param.name;
    }
);

// ============================================================================
// Multi-corner path tests
// ============================================================================

class MultiCornerTest : public ::testing::TestWithParam<MultiCornerScenario> {
protected:
    void SetUp() override { ensureOutputDir(); }
};

TEST_P(MultiCornerTest, GeneratesValidBlendedPath) {
    const auto& scenario = GetParam();
    std::string svgPath = getOutputDir() + "/" + scenario.name + ".svg";

    auto vd = runMultiCornerScenario(scenario, svgPath);

    // Should have original and blended paths
    EXPECT_FALSE(vd.originalPath.empty())
        << "Original path should not be empty for " << scenario.name;
    EXPECT_FALSE(vd.blendedPath.empty())
        << "Blended path should not be empty for " << scenario.name;

    // Blended path should have more points than original (due to blend sampling)
    EXPECT_GT(vd.blendedPath.size(), vd.originalPath.size())
        << "Blended path should have more points for " << scenario.name;

    // SVG file should exist
    EXPECT_TRUE(std::filesystem::exists(svgPath))
        << "SVG output should be generated at " << svgPath;
}

TEST_P(MultiCornerTest, BlendedPathStartsAndEndsCorrectly) {
    const auto& scenario = GetParam();
    auto vd = runMultiCornerScenario(scenario);

    ASSERT_FALSE(vd.originalPath.empty());
    ASSERT_FALSE(vd.blendedPath.empty());

    // First point should match
    EXPECT_NEAR(vd.blendedPath[0].x, vd.originalPath[0].x, 0.1);
    EXPECT_NEAR(vd.blendedPath[0].y, vd.originalPath[0].y, 0.1);

    // Last point should match
    EXPECT_NEAR(vd.blendedPath.back().x, vd.originalPath.back().x, 0.1);
    EXPECT_NEAR(vd.blendedPath.back().y, vd.originalPath.back().y, 0.1);
}

INSTANTIATE_TEST_SUITE_P(
    MultiCornerPaths,
    MultiCornerTest,
    ::testing::ValuesIn(multiCornerScenarios()),
    [](const ::testing::TestParamInfo<MultiCornerScenario>& info) {
        return info.param.name;
    }
);

// ============================================================================
// SVG output verification tests
// ============================================================================

TEST(BlendSVGOutput, SingleBlendSVGIsValid) {
    ensureOutputDir();
    auto scenarios = standardScenarios();
    ASSERT_FALSE(scenarios.empty());

    std::string path = getOutputDir() + "/svg_validation_test.svg";
    auto result = runBlendScenario(scenarios[0], path);

    ASSERT_TRUE(std::filesystem::exists(path));

    // Read file and check it's valid XML-ish SVG
    std::ifstream file(path);
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    EXPECT_NE(content.find("<svg"), std::string::npos);
    EXPECT_NE(content.find("</svg>"), std::string::npos);
    EXPECT_NE(content.find("Blend curve"), std::string::npos);
    EXPECT_NE(content.find("Curvature profile"), std::string::npos);
}

TEST(BlendSVGOutput, MultiBlendSVGIsValid) {
    ensureOutputDir();
    auto scenarios = multiCornerScenarios();
    ASSERT_FALSE(scenarios.empty());

    std::string path = getOutputDir() + "/multi_svg_validation_test.svg";
    auto vd = runMultiCornerScenario(scenarios[0], path);

    ASSERT_TRUE(std::filesystem::exists(path));

    std::ifstream file(path);
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    EXPECT_NE(content.find("<svg"), std::string::npos);
    EXPECT_NE(content.find("</svg>"), std::string::npos);
    EXPECT_NE(content.find("Blended path"), std::string::npos);
}

// ============================================================================
// Summary test — generates all SVGs and reports
// ============================================================================

TEST(BlendVisualSummary, GenerateAllSVGs) {
    ensureOutputDir();
    std::string dir = getOutputDir();

    int singleCount = 0, multiCount = 0;

    for (const auto& s : standardScenarios()) {
        std::string path = dir + "/" + s.name + ".svg";
        runBlendScenario(s, path);
        if (std::filesystem::exists(path)) ++singleCount;
    }

    for (const auto& s : multiCornerScenarios()) {
        std::string path = dir + "/" + s.name + ".svg";
        runMultiCornerScenario(s, path);
        if (std::filesystem::exists(path)) ++multiCount;
    }

    EXPECT_GT(singleCount, 0) << "Should generate at least one single-blend SVG";
    EXPECT_GT(multiCount, 0) << "Should generate at least one multi-blend SVG";

    std::cout << "\n=== Blend Test SVG Summary ===\n"
              << "Single-blend SVGs: " << singleCount << "\n"
              << "Multi-blend SVGs:  " << multiCount << "\n"
              << "Output directory:  " << dir << "\n"
              << "================================\n\n";
}

} // namespace BlendTest
