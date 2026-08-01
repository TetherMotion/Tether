/**
 * @file test_klipper_gcode_executor.cpp
 * @brief Tests for G-code parser and executor.
 */

#include "tether/klipper/klippy/GCodeExecutor.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace tether::klipper::klippy;

// ============================================================================
// G-code parser tests
// ============================================================================

TEST(KlipperGcodeParser, ParseG1) {
    auto g = parseGcodeLine("G1 X10 Y20 Z0.3 E5 F1500");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G1");
    EXPECT_TRUE(g->has('X'));
    EXPECT_EQ(g->get('X'), 10.0);
    EXPECT_EQ(g->get('Y'), 20.0);
    EXPECT_NEAR(g->get('Z'), 0.3, 0.001);
    EXPECT_EQ(g->get('E'), 5.0);
    EXPECT_EQ(g->get('F'), 1500.0);
}

TEST(KlipperGcodeParser, ParseG28) {
    auto g = parseGcodeLine("G28 X Y");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G28");
    EXPECT_TRUE(g->has('X'));
    EXPECT_TRUE(g->has('Y'));
    EXPECT_FALSE(g->has('Z'));
}

TEST(KlipperGcodeParser, ParseComment) {
    auto g = parseGcodeLine("G1 X10 ; move to X10");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G1");
    EXPECT_EQ(g->get('X'), 10.0);
    EXPECT_EQ(g->comment, " move to X10");
}

TEST(KlipperGcodeParser, EmptyLine) {
    auto g = parseGcodeLine("");
    EXPECT_FALSE(g.has_value());
}

TEST(KlipperGcodeParser, CommentOnly) {
    auto g = parseGcodeLine("; just a comment");
    EXPECT_FALSE(g.has_value());
}

TEST(KlipperGcodeParser, WhitespaceOnly) {
    auto g = parseGcodeLine("   \t  ");
    EXPECT_FALSE(g.has_value());
}

TEST(KlipperGcodeParser, ParseM104) {
    auto g = parseGcodeLine("M104 S200 T1");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "M104");
    EXPECT_EQ(g->get('S'), 200.0);
    EXPECT_EQ(g->get('T'), 1.0);
}

TEST(KlipperGcodeParser, ParseNegativeParams) {
    auto g = parseGcodeLine("G1 X-10 Y-20.5");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->get('X'), -10.0);
    EXPECT_NEAR(g->get('Y'), -20.5, 0.001);
}

TEST(KlipperGcodeParser, ParseLowercase) {
    auto g = parseGcodeLine("g1 x10 y20");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G1");
    EXPECT_EQ(g->get('X'), 10.0);
    EXPECT_EQ(g->get('Y'), 20.0);
}

TEST(KlipperGcodeParser, ParseG28All) {
    auto g = parseGcodeLine("G28");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G28");
    EXPECT_TRUE(g->params.empty());
}

TEST(KlipperGcodeParser, ParseM112) {
    auto g = parseGcodeLine("M112");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "M112");
}

TEST(KlipperGcodeParser, ParseG29) {
    auto g = parseGcodeLine("G29 ; bed level");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G29");
}

TEST(KlipperGcodeParser, ParseT0) {
    auto g = parseGcodeLine("T0");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "T0");
}

TEST(KlipperGcodeParser, ParseG92) {
    auto g = parseGcodeLine("G92 X0 Y0 Z0 E0");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G92");
    EXPECT_EQ(g->get('X'), 0.0);
    EXPECT_EQ(g->get('E'), 0.0);
}

TEST(KlipperGcodeParser, ParseG4Dwell) {
    auto g = parseGcodeLine("G4 P500");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G4");
    EXPECT_EQ(g->get('P'), 500.0);
}

// --- Tests verifying unification with tether::gcode lexer ---

/// @brief Verify the parser uses the shared GCode::Lexer for tokenization
/// by checking subcode handling (G38.2) which the lexer supports natively.
TEST(KlipperGcodeParser, ParseSubcodeViaLexer) {
    auto g = parseGcodeLine("G38.2 X10 Y20");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G38.2");
    EXPECT_EQ(g->get('X'), 10.0);
    EXPECT_EQ(g->get('Y'), 20.0);
}

/// @brief Verify extended commands (SET_SERVO) are parsed by the fallback path.
TEST(KlipperGcodeParser, ParseExtendedCommand) {
    auto g = parseGcodeLine("SET_SERVO SERVO=my_servo ANGLE=45");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "SET_SERVO");
    EXPECT_TRUE(g->isExtendedCommand());
    EXPECT_EQ(g->getNamed("SERVO"), "my_servo");
    EXPECT_EQ(g->getNamedDouble("ANGLE"), 45.0);
}

/// @brief Verify extended command with quoted string parameter.
TEST(KlipperGcodeParser, ParseExtendedCommandQuoted) {
    auto g = parseGcodeLine("SET_GCODE_VARIABLE MACRO=START_PRINT VARIABLE=enable VALUE=1");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "SET_GCODE_VARIABLE");
    EXPECT_EQ(g->getNamed("MACRO"), "START_PRINT");
    EXPECT_EQ(g->getNamed("VARIABLE"), "enable");
    EXPECT_EQ(g->getNamed("VALUE"), "1");
}

/// @brief Verify M117 captures remaining text as message.
TEST(KlipperGcodeParser, ParseM117Text) {
    auto g = parseGcodeLine("M117 Hello World");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "M117");
    EXPECT_EQ(g->text, "Hello World");
    EXPECT_TRUE(g->params.empty());
}

/// @brief Verify bare letters and letter+number mix (Klipper flavor).
TEST(KlipperGcodeParser, ParseMixedBareAndNumbered) {
    auto g = parseGcodeLine("G28 X Y Z0");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G28");
    EXPECT_TRUE(g->has('X'));
    EXPECT_TRUE(g->has('Y'));
    EXPECT_TRUE(g->has('Z'));
    EXPECT_EQ(g->get('Z'), 0.0);
}

/// @brief Verify the parser handles G-code with multiple parameters
/// including negative and decimal values (lexer tokenization).
TEST(KlipperGcodeParser, ParseComplexLine) {
    auto g = parseGcodeLine("G1 X-10.5 Y20 Z0.3 E1.5 F1500");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G1");
    EXPECT_NEAR(g->get('X'), -10.5, 0.001);
    EXPECT_EQ(g->get('Y'), 20.0);
    EXPECT_NEAR(g->get('Z'), 0.3, 0.001);
    EXPECT_NEAR(g->get('E'), 1.5, 0.001);
    EXPECT_EQ(g->get('F'), 1500.0);
}

// ============================================================================
// G-code executor tests
// ============================================================================

class GcodeExecutorTest : public ::testing::Test {
protected:
    struct MoveCall {
        double x, y, z, e, speed;
    };

    std::vector<MoveCall> moves;
    std::vector<std::string> homeCalls;
    std::vector<std::pair<int, double>> hotendTemps;
    std::vector<double> bedTemps;
    std::vector<double> fanSpeeds;
    std::vector<std::pair<std::string, bool>> motorEnables;
    bool emergencyStopCalled = false;
    double probeResult = NAN;
    std::vector<std::array<double, 4>> setPositions;
    std::vector<double> dwells;
    std::vector<std::string> outputs;

    std::unique_ptr<GCodeExecutor> executor;

    void SetUp() override {
        moves.clear();
        homeCalls.clear();
        hotendTemps.clear();
        bedTemps.clear();
        fanSpeeds.clear();
        motorEnables.clear();
        emergencyStopCalled = false;
        probeResult = NAN;
        setPositions.clear();
        dwells.clear();
        outputs.clear();

        GcodeCallbacks cb;
        cb.move = [this](double x, double y, double z, double e, double speed) {
            moves.push_back({x, y, z, e, speed});
        };
        cb.home = [this](const std::string& axes) {
            homeCalls.push_back(axes);
        };
        cb.setHotendTemp = [this](int ext, double temp, bool wait) {
            hotendTemps.push_back({ext, temp});
        };
        cb.setBedTemp = [this](double temp, bool wait) {
            bedTemps.push_back(temp);
        };
        cb.setFanSpeed = [this](double speed) {
            fanSpeeds.push_back(speed);
        };
        cb.setMotorEnable = [this](const std::string& axes, bool enable) {
            motorEnables.push_back({axes, enable});
        };
        cb.emergencyStop = [this]() {
            emergencyStopCalled = true;
        };
        cb.probe = [this]() { return probeResult; };
        cb.setPosition = [this](double x, double y, double z, double e) {
            setPositions.push_back({x, y, z, e});
        };
        cb.dwell = [this](double s) { dwells.push_back(s); };
        cb.output = [this](const std::string& msg) { outputs.push_back(msg); };

        executor = std::make_unique<GCodeExecutor>(std::move(cb));
    }
};

TEST_F(GcodeExecutorTest, ExecuteG1Move) {
    executor->executeLine("G1 X10 Y20 Z0.3 F1500");
    ASSERT_EQ(moves.size(), 1u);
    EXPECT_NEAR(moves[0].x, 10.0, 0.001);
    EXPECT_NEAR(moves[0].y, 20.0, 0.001);
    EXPECT_NEAR(moves[0].z, 0.3, 0.001);
    // F1500 mm/min = 25 mm/s
    EXPECT_NEAR(moves[0].speed, 25.0, 0.1);
}

TEST_F(GcodeExecutorTest, ExecuteG0Move) {
    executor->executeLine("G0 X50 Y50");
    ASSERT_EQ(moves.size(), 1u);
    EXPECT_NEAR(moves[0].x, 50.0, 0.001);
}

TEST_F(GcodeExecutorTest, ExecuteG28HomeAll) {
    executor->executeLine("G28");
    ASSERT_EQ(homeCalls.size(), 1u);
    EXPECT_EQ(homeCalls[0], "xyz");
}

TEST_F(GcodeExecutorTest, ExecuteG28HomeX) {
    executor->executeLine("G28 X");
    ASSERT_EQ(homeCalls.size(), 1u);
    EXPECT_EQ(homeCalls[0], "x");
}

TEST_F(GcodeExecutorTest, ExecuteG28HomeXY) {
    executor->executeLine("G28 X Y");
    ASSERT_EQ(homeCalls.size(), 1u);
    EXPECT_EQ(homeCalls[0], "xy");
}

TEST_F(GcodeExecutorTest, ExecuteM104) {
    executor->executeLine("M104 S200");
    ASSERT_EQ(hotendTemps.size(), 1u);
    // No T parameter specified: extruder index is -1 (use active extruder)
    EXPECT_EQ(hotendTemps[0].first, -1);
    EXPECT_EQ(hotendTemps[0].second, 200.0);
}

TEST_F(GcodeExecutorTest, ExecuteM104WithTool) {
    executor->executeLine("M104 S210 T1");
    ASSERT_EQ(hotendTemps.size(), 1u);
    EXPECT_EQ(hotendTemps[0].first, 1);
    EXPECT_EQ(hotendTemps[0].second, 210.0);
}

TEST_F(GcodeExecutorTest, ExecuteM109) {
    executor->executeLine("M109 S200");
    ASSERT_EQ(hotendTemps.size(), 1u);
    EXPECT_EQ(hotendTemps[0].second, 200.0);
}

TEST_F(GcodeExecutorTest, ExecuteM140) {
    executor->executeLine("M140 S60");
    ASSERT_EQ(bedTemps.size(), 1u);
    EXPECT_EQ(bedTemps[0], 60.0);
}

TEST_F(GcodeExecutorTest, ExecuteM190) {
    executor->executeLine("M190 S60");
    ASSERT_EQ(bedTemps.size(), 1u);
    EXPECT_EQ(bedTemps[0], 60.0);
}

TEST_F(GcodeExecutorTest, ExecuteM106) {
    executor->executeLine("M106 S128");
    ASSERT_EQ(fanSpeeds.size(), 1u);
    EXPECT_NEAR(fanSpeeds[0], 128.0 / 255.0, 0.01);
}

TEST_F(GcodeExecutorTest, ExecuteM107) {
    executor->executeLine("M107");
    ASSERT_EQ(fanSpeeds.size(), 1u);
    EXPECT_NEAR(fanSpeeds[0], 0.0, 0.01);
}

TEST_F(GcodeExecutorTest, ExecuteM112) {
    executor->executeLine("M112");
    EXPECT_TRUE(emergencyStopCalled);
}

TEST_F(GcodeExecutorTest, ExecuteM17) {
    executor->executeLine("M17");
    ASSERT_EQ(motorEnables.size(), 1u);
    EXPECT_TRUE(motorEnables[0].second);
}

TEST_F(GcodeExecutorTest, ExecuteM84) {
    executor->executeLine("M84");
    ASSERT_EQ(motorEnables.size(), 1u);
    EXPECT_FALSE(motorEnables[0].second);
}

TEST_F(GcodeExecutorTest, ExecuteG92) {
    executor->executeLine("G92 X0 Y0 Z0 E0");
    ASSERT_EQ(setPositions.size(), 1u);
    EXPECT_NEAR(setPositions[0][0], 0.0, 0.001);
    EXPECT_NEAR(setPositions[0][3], 0.0, 0.001);
}

TEST_F(GcodeExecutorTest, ExecuteG4DwellMs) {
    executor->executeLine("G4 P500");
    ASSERT_EQ(dwells.size(), 1u);
    EXPECT_NEAR(dwells[0], 0.5, 0.001);
}

TEST_F(GcodeExecutorTest, ExecuteG4DwellSeconds) {
    executor->executeLine("G4 S2");
    ASSERT_EQ(dwells.size(), 1u);
    EXPECT_NEAR(dwells[0], 2.0, 0.001);
}

TEST_F(GcodeExecutorTest, ExecuteG90) {
    executor->executeLine("G90");
    EXPECT_TRUE(executor->state().absoluteCoordinates());
    // Verify enum-based modal state (unified with tether::gcode)
    EXPECT_EQ(executor->state().distanceMode, GCode::DistanceMode::ABSOLUTE);
}

TEST_F(GcodeExecutorTest, ExecuteG91) {
    executor->executeLine("G91");
    EXPECT_FALSE(executor->state().absoluteCoordinates());
    // Verify enum-based modal state (unified with tether::gcode)
    EXPECT_EQ(executor->state().distanceMode, GCode::DistanceMode::INCREMENTAL);
}

// --- Tests verifying enum-based modal state (unified with tether::gcode) ---

TEST_F(GcodeExecutorTest, ExecuteG20SetsUnitsEnum) {
    executor->executeLine("G20");
    EXPECT_EQ(executor->state().units, GCode::Units::INCH);
}

TEST_F(GcodeExecutorTest, ExecuteG21SetsUnitsEnum) {
    executor->executeLine("G21");
    EXPECT_EQ(executor->state().units, GCode::Units::MM);
}

TEST_F(GcodeExecutorTest, ExecuteG17SetsPlaneEnum) {
    executor->executeLine("G17");
    EXPECT_EQ(executor->state().plane, GCode::Plane::XY);
}

TEST_F(GcodeExecutorTest, ExecuteG18SetsPlaneEnum) {
    executor->executeLine("G18");
    EXPECT_EQ(executor->state().plane, GCode::Plane::ZX);
}

TEST_F(GcodeExecutorTest, ExecuteG19SetsPlaneEnum) {
    executor->executeLine("G19");
    EXPECT_EQ(executor->state().plane, GCode::Plane::YZ);
}

TEST_F(GcodeExecutorTest, ExecuteM82) {
    executor->executeLine("M82");
    EXPECT_TRUE(executor->state().absoluteExtrude);
}

TEST_F(GcodeExecutorTest, ExecuteM83) {
    executor->executeLine("M83");
    EXPECT_FALSE(executor->state().absoluteExtrude);
}

TEST_F(GcodeExecutorTest, ExecuteRelativeMove) {
    executor->executeLine("G91"); // Relative
    executor->executeLine("G1 X10");
    ASSERT_EQ(moves.size(), 1u);
    EXPECT_NEAR(moves[0].x, 10.0, 0.001);

    executor->executeLine("G1 X5");
    ASSERT_EQ(moves.size(), 2u);
    EXPECT_NEAR(moves[1].x, 15.0, 0.001); // 10 + 5
}

TEST_F(GcodeExecutorTest, ExecuteMultiLineScript) {
    executor->execute("G28\nG1 X10 Y10 F1500\nM104 S200\nM106 S255");
    EXPECT_EQ(homeCalls.size(), 1u);
    EXPECT_EQ(moves.size(), 1u);
    EXPECT_EQ(hotendTemps.size(), 1u);
    EXPECT_EQ(fanSpeeds.size(), 1u);
}

TEST_F(GcodeExecutorTest, ExecuteG30Probe) {
    probeResult = -0.5;
    executor->executeLine("G30");
    // State should have Z updated
    EXPECT_NEAR(executor->state().position[2], -0.5, 0.001);
}

TEST_F(GcodeExecutorTest, ExecuteM220SpeedFactor) {
    executor->executeLine("M220 S50");
    EXPECT_NEAR(executor->state().speedFactor, 0.5, 0.01);
}

TEST_F(GcodeExecutorTest, ExecuteM221ExtrudeFactor) {
    executor->executeLine("M221 S150");
    EXPECT_NEAR(executor->state().extrudeFactor, 1.5, 0.01);
}

TEST_F(GcodeExecutorTest, UnknownCommandIgnored) {
    EXPECT_TRUE(executor->executeLine("M999"));
    EXPECT_TRUE(executor->executeLine("G999"));
}

TEST_F(GcodeExecutorTest, EmptyLineOk) {
    EXPECT_TRUE(executor->executeLine(""));
    EXPECT_TRUE(executor->executeLine("; comment"));
}

TEST_F(GcodeExecutorTest, StateUpdatesPosition) {
    executor->executeLine("G1 X10 Y20 Z30");
    auto& s = executor->state();
    EXPECT_NEAR(s.position[0], 10.0, 0.001);
    EXPECT_NEAR(s.position[1], 20.0, 0.001);
    EXPECT_NEAR(s.position[2], 30.0, 0.001);
}

TEST_F(GcodeExecutorTest, HomedAxesUpdated) {
    executor->executeLine("G28 X");
    EXPECT_TRUE(executor->state().homedAxes.find('x') != std::string::npos);

    executor->executeLine("G28 Y");
    EXPECT_TRUE(executor->state().homedAxes.find('y') != std::string::npos);
}

TEST_F(GcodeExecutorTest, FeedratePersists) {
    executor->executeLine("G1 X10 F3000");
    EXPECT_NEAR(executor->state().feedrate, 3000.0, 0.1);

    executor->executeLine("G1 X20");
    // Feedrate should still be 3000
    EXPECT_NEAR(executor->state().feedrate, 3000.0, 0.1);
}

TEST_F(GcodeExecutorTest, CustomCommandCallback) {
    bool customCalled = false;
    std::string customCode;

    GcodeCallbacks cb;
    cb.custom = [&](const GcodeLine& line) {
        customCalled = true;
        customCode = line.code;
    };

    GCodeExecutor exec(std::move(cb));
    exec.executeLine("M999");
    EXPECT_TRUE(customCalled);
    EXPECT_EQ(customCode, "M999");
}
