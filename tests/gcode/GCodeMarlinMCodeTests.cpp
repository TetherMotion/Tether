/**
 * @file GCodeMarlinMCodeTests.cpp
 * @brief Tests for Marlin/RepRap M-code coverage in the interpreter.
 *
 * Verifies that printer-domain M-codes are routed through
 * MarlinMCodeHandler, update MarlinMachineState, emit report
 * messages, and still reach the user M-code hook.
 */

#include "tether/gcode/GCodeInterpreter.hpp"

#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace GCode;

class MarlinMCodeTest : public ::testing::Test {
protected:
    Interpreter interp;
    std::vector<std::string> messages;
    std::vector<int> userMCodes;

    MarlinMCodeTest() {
        interp.setMessageCallback(
            [this](const std::string& m) { messages.push_back(m); });
        interp.setMCodeCallback(
            [this](int32_t m, std::optional<double>, std::optional<double>) {
                userMCodes.push_back(m);
                return Error{};
            });
    }

    bool run(const std::string& program) {
        if (!interp.loadString(program).ok()) return false;
        return interp.run().ok();
    }

    const MarlinMachineState& ms() const { return interp.marlinState(); }
};

TEST_F(MarlinMCodeTest, M82M83_ExtruderDistanceMode) {
    ASSERT_TRUE(run("M83\n"));
    EXPECT_FALSE(ms().extruderAbsolute);
    ASSERT_TRUE(run("M82\n"));
    EXPECT_TRUE(ms().extruderAbsolute);
}

TEST_F(MarlinMCodeTest, M92_StepsPerMm) {
    ASSERT_TRUE(run("M92 X160 Y80 Z800 E415\n"));
    EXPECT_DOUBLE_EQ(ms().stepsPerMm[0], 160.0);
    EXPECT_DOUBLE_EQ(ms().stepsPerMm[1], 80.0);
    EXPECT_DOUBLE_EQ(ms().stepsPerMm[2], 800.0);
    EXPECT_DOUBLE_EQ(ms().stepsPerMm[3], 415.0);
}

TEST_F(MarlinMCodeTest, M114_ReportsPosition) {
    ASSERT_TRUE(run("G1 X10 Y20 Z5 F1500\nM114\n"));
    ASSERT_FALSE(messages.empty());
    const std::string& report = messages.back();
    EXPECT_NE(report.find("X:10"), std::string::npos);
    EXPECT_NE(report.find("Y:20"), std::string::npos);
    EXPECT_NE(report.find("Z:5"), std::string::npos);
}

TEST_F(MarlinMCodeTest, M201_M203_M204_PerAxisLimits) {
    ASSERT_TRUE(run("M201 X9000 Y9000 Z100 E5000\n"
                    "M203 X500 Y500 Z12 E120\n"
                    "M204 P1500 T2000 R3000\n"));
    const auto& lim = ms().kinematicLimits;
    EXPECT_DOUBLE_EQ(lim.axisMaxAcceleration[0], 9000.0);
    EXPECT_DOUBLE_EQ(lim.axisMaxAcceleration[2], 100.0);
    EXPECT_DOUBLE_EQ(ms().extruderMaxAcceleration, 5000.0);
    EXPECT_DOUBLE_EQ(lim.axisMaxVelocity[0], 500.0 * 60.0); // stored mm/min
    EXPECT_DOUBLE_EQ(ms().extruderMaxVelocity, 120.0);
    EXPECT_DOUBLE_EQ(lim.maxAcceleration, 1500.0);
    EXPECT_DOUBLE_EQ(ms().travelAcceleration, 2000.0);
    EXPECT_DOUBLE_EQ(ms().retractAcceleration, 3000.0);
}

TEST_F(MarlinMCodeTest, M205_AdvancedSettings) {
    ASSERT_TRUE(run("M205 X8 Y8 Z0.4 E5 J0.02 S1 T2 B20\n"));
    const auto& lim = ms().kinematicLimits;
    EXPECT_DOUBLE_EQ(lim.axisMaxJerk[0], 8.0);
    EXPECT_DOUBLE_EQ(lim.axisMaxJerk[2], 0.4);
    EXPECT_DOUBLE_EQ(ms().extruderMaxJerk, 5.0);
    EXPECT_DOUBLE_EQ(ms().junctionDeviation, 0.02);
    EXPECT_DOUBLE_EQ(ms().minFeedrate, 1.0);
    EXPECT_DOUBLE_EQ(ms().minTravelFeedrate, 2.0);
    EXPECT_DOUBLE_EQ(ms().minSegmentTime, 20.0);
}

TEST_F(MarlinMCodeTest, M220_FeedOverrideAppliesToMachineState) {
    ASSERT_TRUE(run("M220 S150\n"));
    EXPECT_DOUBLE_EQ(ms().feedOverridePercent, 150.0);
    EXPECT_DOUBLE_EQ(interp.getMachineState().feedOverride, 1.5);
}

TEST_F(MarlinMCodeTest, M221_FlowOverride) {
    ASSERT_TRUE(run("M221 S95\n"));
    EXPECT_DOUBLE_EQ(ms().flowOverridePercent, 95.0);
}

TEST_F(MarlinMCodeTest, M206_HomeOffset) {
    ASSERT_TRUE(run("M206 X-1.5 Y2 Z-0.25\n"));
    EXPECT_DOUBLE_EQ(ms().homeOffset[0], -1.5);
    EXPECT_DOUBLE_EQ(ms().homeOffset[1], 2.0);
    EXPECT_DOUBLE_EQ(ms().homeOffset[2], -0.25);
}

TEST_F(MarlinMCodeTest, M218_ToolOffset) {
    ASSERT_TRUE(run("M218 T1 X12.5 Y-3 Z0.5\n"));
    EXPECT_DOUBLE_EQ(ms().toolOffsets[1][0], 12.5);
    EXPECT_DOUBLE_EQ(ms().toolOffsets[1][1], -3.0);
    EXPECT_DOUBLE_EQ(ms().toolOffsets[1][2], 0.5);
    // No T word: applies to the current tool (T0).
    ASSERT_TRUE(run("M218 X1\n"));
    EXPECT_DOUBLE_EQ(ms().toolOffsets[0][0], 1.0);
}

TEST_F(MarlinMCodeTest, M218_ToolIndexOutOfRange) {
    EXPECT_FALSE(run("M218 T99 X1\n"));
}

TEST_F(MarlinMCodeTest, M280_ServoPosition) {
    ASSERT_TRUE(run("M280 P0 S90\n"));
    EXPECT_DOUBLE_EQ(ms().servoAngles[0], 90.0);
}

TEST_F(MarlinMCodeTest, M280_MissingParamsFails) {
    EXPECT_FALSE(run("M280 P0\n"));
    EXPECT_FALSE(run("M280 S45\n"));
}

TEST_F(MarlinMCodeTest, M301_M304_PidGains) {
    ASSERT_TRUE(run("M301 P22.2 I1.08 D114\nM304 P60 I10 D300\n"));
    EXPECT_DOUBLE_EQ(ms().hotendPid[0], 22.2);
    EXPECT_DOUBLE_EQ(ms().hotendPid[1], 1.08);
    EXPECT_DOUBLE_EQ(ms().hotendPid[2], 114.0);
    EXPECT_DOUBLE_EQ(ms().bedPid[0], 60.0);
    EXPECT_DOUBLE_EQ(ms().bedPid[1], 10.0);
    EXPECT_DOUBLE_EQ(ms().bedPid[2], 300.0);
}

TEST_F(MarlinMCodeTest, M401_M402_ProbeDeployStow) {
    ASSERT_TRUE(run("M401\n"));
    EXPECT_TRUE(ms().probeDeployed);
    ASSERT_TRUE(run("M402\n"));
    EXPECT_FALSE(ms().probeDeployed);
}

TEST_F(MarlinMCodeTest, M420_BedLevelingState) {
    ASSERT_TRUE(run("M420 S1\n"));
    EXPECT_TRUE(ms().bedLevelingEnabled);
    ASSERT_TRUE(run("M420 S0\n"));
    EXPECT_FALSE(ms().bedLevelingEnabled);
}

TEST_F(MarlinMCodeTest, M421_SetsMeshPoint) {
    ASSERT_TRUE(run("M421 I2 J3 Z-0.05\n"));
    EXPECT_DOUBLE_EQ(ms().bedMesh.at(2 * 1024 + 3), -0.05);
}

TEST_F(MarlinMCodeTest, M421_MissingParamsFails) {
    EXPECT_FALSE(run("M421 I1 J2\n"));
}

TEST_F(MarlinMCodeTest, M851_ProbeOffsetReports) {
    ASSERT_TRUE(run("M851 Z-1.85\n"));
    EXPECT_DOUBLE_EQ(ms().probeOffset[2], -1.85);
    ASSERT_FALSE(messages.empty());
    EXPECT_NE(messages.back().find("Z-1.85"), std::string::npos);
}

TEST_F(MarlinMCodeTest, M117_DisplaysMessage) {
    ASSERT_TRUE(run("M117 Heating up\n"));
    ASSERT_FALSE(messages.empty());
    EXPECT_NE(messages.back().find("Heating up"), std::string::npos);
}

TEST_F(MarlinMCodeTest, M503_ReportsSettings) {
    ASSERT_TRUE(run("M92 X80\nM301 P30 I2 D90\nM503\n"));
    bool found = false;
    for (const auto& m : messages) {
        if (m.find("M301 P30") != std::string::npos) found = true;
    }
    EXPECT_TRUE(found);
}

TEST_F(MarlinMCodeTest, UserCallbackStillInvoked) {
    ASSERT_TRUE(run("M220 S110\nM114\n"));
    ASSERT_EQ(userMCodes.size(), 2u);
    EXPECT_EQ(userMCodes[0], 220);
    EXPECT_EQ(userMCodes[1], 114);
}

TEST_F(MarlinMCodeTest, MarlinStateTracksPositionBeforeM114) {
    ASSERT_TRUE(run("G90\nG0 X7 Y8\nM114\n"));
    ASSERT_FALSE(messages.empty());
    EXPECT_NE(messages.back().find("X:7"), std::string::npos);
}
