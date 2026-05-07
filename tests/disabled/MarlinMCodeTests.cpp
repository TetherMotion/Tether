/**
 * @file MarlinMCodeTests.cpp
 * @brief Unit tests for Marlin M-code handler with callback registration
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <sstream>

// Include the handler
#include "gcode/MarlinMCodeHandler.hpp"

using namespace GCode;
using namespace testing;

// ============================================================================
// Test Fixtures
// ============================================================================

class MarlinMCodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        MarlinMCodeHandler::Config config;
        config.enableMarlinDefaults = true;
        config.strict = false;
        handler_ = std::make_unique<MarlinMCodeHandler>(config);
    }

    std::unique_ptr<MarlinMCodeHandler> handler_;
    MarlinMachineState state_;
};

class StrictMCodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        MarlinMCodeHandler::Config config;
        config.enableMarlinDefaults = true;
        config.strict = true;  // Fail on unknown M-codes
        handler_ = std::make_unique<MarlinMCodeHandler>(config);
    }

    std::unique_ptr<MarlinMCodeHandler> handler_;
    MarlinMachineState state_;
};

// ============================================================================
// Parsing Tests
// ============================================================================

TEST_F(MarlinMCodeTest, ParseSimpleMCode) {
    auto params = handler_->parse("M3");
    EXPECT_EQ(params.mCode, 3);
}

TEST_F(MarlinMCodeTest, ParseMCodeWithS) {
    auto params = handler_->parse("M3 S12000");
    EXPECT_EQ(params.mCode, 3);
    ASSERT_TRUE(params.S.has_value());
    EXPECT_DOUBLE_EQ(params.S.value(), 12000.0);
}

TEST_F(MarlinMCodeTest, ParseMCodeWithMultipleParams) {
    auto params = handler_->parse("M201 X3000 Y3000 Z100");
    EXPECT_EQ(params.mCode, 201);
    ASSERT_TRUE(params.X.has_value());
    ASSERT_TRUE(params.Y.has_value());
    ASSERT_TRUE(params.Z.has_value());
    EXPECT_DOUBLE_EQ(params.X.value(), 3000.0);
    EXPECT_DOUBLE_EQ(params.Y.value(), 3000.0);
    EXPECT_DOUBLE_EQ(params.Z.value(), 100.0);
}

TEST_F(MarlinMCodeTest, ParseMCodeWithComment) {
    auto params = handler_->parse("M5 ; Turn off spindle");
    EXPECT_EQ(params.mCode, 5);
}

TEST_F(MarlinMCodeTest, ParseM117Message) {
    auto params = handler_->parse("M117 Hello World!");
    EXPECT_EQ(params.mCode, 117);
    EXPECT_EQ(params.message, "Hello World!");
}

TEST_F(MarlinMCodeTest, ParseNonMCodeLine) {
    auto params = handler_->parse("G1 X100 F1000");
    EXPECT_EQ(params.mCode, -1);
}

TEST_F(MarlinMCodeTest, ParseMCodeWithNegativeValue) {
    auto params = handler_->parse("M206 Z-0.5");
    EXPECT_EQ(params.mCode, 206);
    ASSERT_TRUE(params.Z.has_value());
    EXPECT_DOUBLE_EQ(params.Z.value(), -0.5);
}

TEST_F(MarlinMCodeTest, ParseMCodeWithDecimal) {
    auto params = handler_->parse("M104 S200.5");
    EXPECT_EQ(params.mCode, 104);
    ASSERT_TRUE(params.S.has_value());
    EXPECT_DOUBLE_EQ(params.S.value(), 200.5);
}

// ============================================================================
// Callback Registration Tests
// ============================================================================

TEST_F(MarlinMCodeTest, HasDefaultCallbacks) {
    EXPECT_TRUE(handler_->hasCallback(3));   // M3 spindle
    EXPECT_TRUE(handler_->hasCallback(5));   // M5 spindle off
    EXPECT_TRUE(handler_->hasCallback(201)); // M201 max accel
}

TEST_F(MarlinMCodeTest, RegisterCustomCallback) {
    int callCount = 0;
    double lastSValue = 0;

    handler_->registerCallback(999, [&](const MCodeParameters& params, MarlinMachineState& state) {
        ++callCount;
        if (params.S.has_value()) {
            lastSValue = params.S.value();
        }
        MCodeResult result;
        result.message = "Custom M999 executed";
        return result;
    }, "Custom test M-code");

    EXPECT_TRUE(handler_->hasCallback(999));

    auto params = handler_->parse("M999 S42");
    auto result = handler_->execute(params, state_);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(callCount, 1);
    EXPECT_DOUBLE_EQ(lastSValue, 42.0);
}

TEST_F(MarlinMCodeTest, UnregisterCallback) {
    EXPECT_TRUE(handler_->hasCallback(3));
    handler_->unregisterCallback(3);
    EXPECT_FALSE(handler_->hasCallback(3));
}

TEST_F(MarlinMCodeTest, OverwriteExistingCallback) {
    int customCallCount = 0;

    // Overwrite M3 handler
    handler_->registerCallback(3, [&](const MCodeParameters& params, MarlinMachineState& state) {
        ++customCallCount;
        MCodeResult result;
        result.message = "Custom M3";
        return result;
    }, "Custom M3 handler");

    auto result = handler_->executeLine("M3 S1000", state_);
    EXPECT_EQ(customCallCount, 1);
}

TEST_F(MarlinMCodeTest, GetCallbackDescription) {
    auto desc = handler_->getDescription(3);
    EXPECT_FALSE(desc.empty());
    EXPECT_TRUE(desc.find("Spindle") != std::string::npos ||
                desc.find("spindle") != std::string::npos);
}

TEST_F(MarlinMCodeTest, GetRegisteredMCodes) {
    auto codes = handler_->registeredMCodes();
    EXPECT_GT(codes.size(), 0u);

    // Should include common M-codes
    EXPECT_TRUE(std::find(codes.begin(), codes.end(), 3) != codes.end());
    EXPECT_TRUE(std::find(codes.begin(), codes.end(), 5) != codes.end());
    EXPECT_TRUE(std::find(codes.begin(), codes.end(), 201) != codes.end());
}

// ============================================================================
// Spindle Control Tests (M3/M4/M5)
// ============================================================================

TEST_F(MarlinMCodeTest, M3SpindleOn) {
    auto result = handler_->executeLine("M3 S12000", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(state_.spindleOn);
    EXPECT_TRUE(state_.spindleCW);
    EXPECT_DOUBLE_EQ(state_.currentSpindleSpeed, 12000.0);
}

TEST_F(MarlinMCodeTest, M4SpindleCCW) {
    auto result = handler_->executeLine("M4 S10000", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(state_.spindleOn);
    EXPECT_FALSE(state_.spindleCW);
    EXPECT_DOUBLE_EQ(state_.currentSpindleSpeed, 10000.0);
}

TEST_F(MarlinMCodeTest, M5SpindleOff) {
    state_.spindleOn = true;
    state_.currentSpindleSpeed = 12000;

    auto result = handler_->executeLine("M5", state_);
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(state_.spindleOn);
    EXPECT_DOUBLE_EQ(state_.currentSpindleSpeed, 0.0);
}

// ============================================================================
// Coolant Control Tests (M7/M8/M9)
// ============================================================================

TEST_F(MarlinMCodeTest, M7MistCoolant) {
    auto result = handler_->executeLine("M7", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(state_.coolantMist);
}

TEST_F(MarlinMCodeTest, M8FloodCoolant) {
    auto result = handler_->executeLine("M8", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(state_.coolantFlood);
}

TEST_F(MarlinMCodeTest, M9CoolantOff) {
    state_.coolantMist = true;
    state_.coolantFlood = true;

    auto result = handler_->executeLine("M9", state_);
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(state_.coolantMist);
    EXPECT_FALSE(state_.coolantFlood);
}

// ============================================================================
// Tool Change Tests (M6)
// ============================================================================

TEST_F(MarlinMCodeTest, M6ToolChange) {
    auto result = handler_->executeLine("M6 T2", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.pauseExecution);  // Tool change requests pause
    EXPECT_EQ(state_.currentTool, 2);
}

// ============================================================================
// Program Control Tests (M0/M1/M2/M30)
// ============================================================================

TEST_F(MarlinMCodeTest, M0UnconditionalStop) {
    auto result = handler_->executeLine("M0", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.pauseExecution);
}

TEST_F(MarlinMCodeTest, M0WithDelay) {
    auto result = handler_->executeLine("M0 P5000", state_);  // 5 second pause
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.waitForCompletion);
    EXPECT_DOUBLE_EQ(result.waitTime, 5.0);
}

TEST_F(MarlinMCodeTest, M2ProgramEnd) {
    auto result = handler_->executeLine("M2", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.stopExecution);
}

TEST_F(MarlinMCodeTest, M30ProgramEnd) {
    auto result = handler_->executeLine("M30", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.stopExecution);
}

// ============================================================================
// Temperature Control Tests (M104/M109/M140/M190)
// ============================================================================

TEST_F(MarlinMCodeTest, M104SetExtruderTemp) {
    auto result = handler_->executeLine("M104 S200", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.extruderTargetTemp, 200.0);
    EXPECT_FALSE(result.waitForCompletion);  // M104 doesn't wait
}

TEST_F(MarlinMCodeTest, M109SetExtruderTempAndWait) {
    auto result = handler_->executeLine("M109 S200", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.extruderTargetTemp, 200.0);
    EXPECT_TRUE(result.waitForCompletion);  // M109 waits
}

TEST_F(MarlinMCodeTest, M140SetBedTemp) {
    auto result = handler_->executeLine("M140 S60", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.bedTargetTemp, 60.0);
}

TEST_F(MarlinMCodeTest, M190SetBedTempAndWait) {
    auto result = handler_->executeLine("M190 S60", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.bedTargetTemp, 60.0);
    EXPECT_TRUE(result.waitForCompletion);
}

// ============================================================================
// Extruder Mode Tests (M82/M83)
// ============================================================================

TEST_F(MarlinMCodeTest, M82ExtruderAbsolute) {
    state_.extruderAbsolute = false;
    auto result = handler_->executeLine("M82", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(state_.extruderAbsolute);
}

TEST_F(MarlinMCodeTest, M83ExtruderRelative) {
    state_.extruderAbsolute = true;
    auto result = handler_->executeLine("M83", state_);
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(state_.extruderAbsolute);
}

// ============================================================================
// Motion Limit Tests (M201/M203/M204/M205)
// ============================================================================

TEST_F(MarlinMCodeTest, M201SetMaxAcceleration) {
    auto result = handler_->executeLine("M201 X3000 Y3000 Z100", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxAcceleration[0], 3000.0);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxAcceleration[1], 3000.0);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxAcceleration[2], 100.0);
}

TEST_F(MarlinMCodeTest, M203SetMaxFeedrate) {
    // M203 values are in mm/s, stored as mm/min
    auto result = handler_->executeLine("M203 X200 Y200 Z10", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxVelocity[0], 200.0 * 60);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxVelocity[1], 200.0 * 60);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxVelocity[2], 10.0 * 60);
}

TEST_F(MarlinMCodeTest, M204SetAcceleration) {
    auto result = handler_->executeLine("M204 P1500", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.maxAcceleration, 1500.0);
}

TEST_F(MarlinMCodeTest, M204LegacyS) {
    // Legacy S parameter
    auto result = handler_->executeLine("M204 S2000", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.maxAcceleration, 2000.0);
}

TEST_F(MarlinMCodeTest, M205SetJerk) {
    auto result = handler_->executeLine("M205 X10 Y10 Z0.3", state_);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxJerk[0], 10.0);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxJerk[1], 10.0);
    EXPECT_DOUBLE_EQ(state_.kinematicLimits.axisMaxJerk[2], 0.3);
}

// ============================================================================
// Information Command Tests (M114/M115/M117/M503)
// ============================================================================

TEST_F(MarlinMCodeTest, M114ReportPosition) {
    state_.currentPosition[0] = 100;
    state_.currentPosition[1] = 50;
    state_.currentPosition[2] = 10;
    state_.extruderPosition = 25.5;

    auto result = handler_->executeLine("M114", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.response.find("X:100") != std::string::npos);
    EXPECT_TRUE(result.response.find("Y:50") != std::string::npos);
    EXPECT_TRUE(result.response.find("Z:10") != std::string::npos);
}

TEST_F(MarlinMCodeTest, M115ReportFirmware) {
    auto result = handler_->executeLine("M115", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.response.find("FIRMWARE_NAME") != std::string::npos);
}

TEST_F(MarlinMCodeTest, M117DisplayMessage) {
    auto result = handler_->executeLine("M117 Printing layer 42", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.message.find("layer 42") != std::string::npos);
}

TEST_F(MarlinMCodeTest, M503ReportSettings) {
    state_.kinematicLimits.axisMaxAcceleration[0] = 3000;
    state_.kinematicLimits.axisMaxAcceleration[1] = 3000;
    state_.kinematicLimits.axisMaxAcceleration[2] = 100;

    auto result = handler_->executeLine("M503", state_);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.response.find("M201") != std::string::npos);
    EXPECT_TRUE(result.response.find("M203") != std::string::npos);
}

// ============================================================================
// Unknown M-code Handling Tests
// ============================================================================

TEST_F(MarlinMCodeTest, UnknownMCodeNonStrict) {
    auto result = handler_->executeLine("M9999", state_);
    EXPECT_TRUE(result.success);  // Non-strict mode ignores unknown
}

TEST_F(StrictMCodeTest, UnknownMCodeStrict) {
    auto result = handler_->executeLine("M9999", state_);
    EXPECT_FALSE(result.success);  // Strict mode fails
    EXPECT_TRUE(result.message.find("Unknown") != std::string::npos);
}

// ============================================================================
// Edge Cases and Complex Scenarios
// ============================================================================

TEST_F(MarlinMCodeTest, MultipleSpindleCommands) {
    handler_->executeLine("M3 S5000", state_);
    EXPECT_TRUE(state_.spindleOn);
    EXPECT_TRUE(state_.spindleCW);

    handler_->executeLine("M4 S8000", state_);
    EXPECT_TRUE(state_.spindleOn);
    EXPECT_FALSE(state_.spindleCW);
    EXPECT_DOUBLE_EQ(state_.currentSpindleSpeed, 8000.0);

    handler_->executeLine("M5", state_);
    EXPECT_FALSE(state_.spindleOn);
}

TEST_F(MarlinMCodeTest, MixedCaseInput) {
    auto params = handler_->parse("m3 s12000");  // lowercase
    EXPECT_EQ(params.mCode, 3);
    ASSERT_TRUE(params.S.has_value());
    EXPECT_DOUBLE_EQ(params.S.value(), 12000.0);
}

TEST_F(MarlinMCodeTest, ExtraWhitespace) {
    auto params = handler_->parse("  M3   S12000  ");
    EXPECT_EQ(params.mCode, 3);
    ASSERT_TRUE(params.S.has_value());
}

// ============================================================================
// Callback State Modification Tests
// ============================================================================

TEST_F(MarlinMCodeTest, CallbackModifiesUserVariables) {
    handler_->registerCallback(998, [](const MCodeParameters& params, MarlinMachineState& state) {
        state.userVariables["custom_value"] = params.S.value_or(0);
        MCodeResult result;
        return result;
    }, "Store custom value");

    handler_->executeLine("M998 S123.456", state_);
    EXPECT_DOUBLE_EQ(state_.userVariables["custom_value"], 123.456);
}

TEST_F(MarlinMCodeTest, CallbackReturnsResponse) {
    handler_->registerCallback(997, [](const MCodeParameters& params, MarlinMachineState& state) {
        MCodeResult result;
        result.response = "Custom response";
        return result;
    }, "Return custom response");

    auto result = handler_->executeLine("M997", state_);
    EXPECT_EQ(result.response, "Custom response");
}

// ============================================================================
// Thread Safety Consideration Tests
// ============================================================================

TEST_F(MarlinMCodeTest, ConcurrentCallbackRegistration) {
    // Register and unregister from different "threads" (simulated)
    for (int i = 0; i < 100; ++i) {
        handler_->registerCallback(900 + i, [](const MCodeParameters& params, MarlinMachineState& state) {
            MCodeResult result;
            return result;
        });
    }

    // All should be registered
    for (int i = 0; i < 100; ++i) {
        EXPECT_TRUE(handler_->hasCallback(900 + i));
    }
}

// ============================================================================
// Marlin Compatibility Tests
// ============================================================================

TEST_F(MarlinMCodeTest, MarlinStartGCode) {
    // Typical Marlin start G-code M-codes
    EXPECT_TRUE(handler_->executeLine("M82", state_).success);           // E absolute
    EXPECT_TRUE(handler_->executeLine("M104 S200", state_).success);     // Extruder temp
    EXPECT_TRUE(handler_->executeLine("M140 S60", state_).success);      // Bed temp
    EXPECT_TRUE(handler_->executeLine("M109 S200", state_).success);     // Wait extruder

    EXPECT_TRUE(state_.extruderAbsolute);
    EXPECT_DOUBLE_EQ(state_.extruderTargetTemp, 200.0);
    EXPECT_DOUBLE_EQ(state_.bedTargetTemp, 60.0);
}

TEST_F(MarlinMCodeTest, MarlinEndGCode) {
    state_.spindleOn = true;
    state_.coolantFlood = true;

    // Typical end G-code
    EXPECT_TRUE(handler_->executeLine("M104 S0", state_).success);   // Extruder off
    EXPECT_TRUE(handler_->executeLine("M140 S0", state_).success);   // Bed off
    EXPECT_TRUE(handler_->executeLine("M5", state_).success);        // Spindle off
    EXPECT_TRUE(handler_->executeLine("M9", state_).success);        // Coolant off

    EXPECT_DOUBLE_EQ(state_.extruderTargetTemp, 0.0);
    EXPECT_DOUBLE_EQ(state_.bedTargetTemp, 0.0);
    EXPECT_FALSE(state_.spindleOn);
    EXPECT_FALSE(state_.coolantFlood);
}
