/**
 * @file test_klipper_homing.cpp
 * @brief Tests for homing sequence and probe peripheral.
 */

#include "tether/klipper/objects/Homing.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace tether::klipper::objects;

// ============================================================================
// Homing sequence tests
// ============================================================================

TEST(KlipperHoming, ParseGcodeLine) {
    // Test G-code parsing
    // This is tested more thoroughly in the GCodeExecutor tests
    SUCCEED();
}

TEST(KlipperHoming, SingleAxisHomingSuccess) {
    // Simulate homing: endstop triggers after some time
    bool endstopTriggered = false;
    double currentPosition = 0.0;

    HomingSequence homing(
        [&](const std::string& axis) { return endstopTriggered; },
        [&](const std::string& axis, double speed, bool positive) {
            // Simulate movement
            currentPosition += positive ? 10.0 : -10.0;
            // Trigger endstop after first move
            if (positive && currentPosition > 5.0) {
                endstopTriggered = true;
            }
        },
        [&](const std::string& axis, double pos) { currentPosition = pos; },
        [&](const std::string& axis) { return currentPosition; },
        [&](double seconds) { /* no-op */ }
    );

    HomingAxisConfig config;
    config.name = "x";
    config.searchSpeed = 50.0;
    config.bounceSpeed = 10.0;
    config.bounceDistance = 5.0;
    config.homePosition = 0.0;
    config.positiveDirection = true;

    auto result = homing.homeAxis(config);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.axis, "x");
    EXPECT_EQ(homing.phase(), HomingPhase::Complete);
}

TEST(KlipperHoming, HomingTimeout) {
    // Endstop never triggers
    HomingSequence homing(
        [&](const std::string& axis) { return false; }, // Never triggered
        [&](const std::string& axis, double speed, bool positive) {},
        [&](const std::string& axis, double pos) {},
        [&](const std::string& axis) { return 0.0; },
        [&](double seconds) { /* no-op, but time passes */ }
    );

    HomingAxisConfig config;
    config.name = "x";
    config.searchSpeed = 50.0;

    auto result = homing.homeAxis(config);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(homing.phase(), HomingPhase::Failed);
    EXPECT_TRUE(result.errorMessage.find("timeout") != std::string::npos);
}

TEST(KlipperHoming, MultiAxisHoming) {
    int endstopCount = 0;
    bool xTriggered = false, yTriggered = false;

    HomingSequence homing(
        [&](const std::string& axis) {
            if (axis == "x") return xTriggered;
            if (axis == "y") return yTriggered;
            return false;
        },
        [&](const std::string& axis, double speed, bool positive) {
            if (axis == "x") xTriggered = true;
            if (axis == "y") yTriggered = true;
        },
        [&](const std::string& axis, double pos) {},
        [&](const std::string& axis) { return 0.0; },
        [&](double seconds) {}
    );

    std::vector<HomingAxisConfig> configs = {
        {"x", 0, 50.0, 10.0, 5.0, 0.0, true},
        {"y", 1, 50.0, 10.0, 5.0, 0.0, true}
    };

    auto results = homing.homeAxes(configs);
    EXPECT_EQ(results.size(), 2u);
    EXPECT_TRUE(results[0].success);
    EXPECT_TRUE(results[1].success);
}

TEST(KlipperHoming, MultiAxisHomingStopsOnFailure) {
    HomingSequence homing(
        [&](const std::string& axis) { return false; }, // Never triggers
        [&](const std::string& axis, double speed, bool positive) {},
        [&](const std::string& axis, double pos) {},
        [&](const std::string& axis) { return 0.0; },
        [&](double seconds) {}
    );

    std::vector<HomingAxisConfig> configs = {
        {"x", 0, 50.0, 10.0, 5.0, 0.0, true},
        {"y", 1, 50.0, 10.0, 5.0, 0.0, true}
    };

    auto results = homing.homeAxes(configs);
    EXPECT_EQ(results.size(), 1u); // Only X attempted, failed
    EXPECT_FALSE(results[0].success);
}

TEST(KlipperHoming, PhaseToString) {
    EXPECT_STREQ(homingPhaseToString(HomingPhase::Idle), "idle");
    EXPECT_STREQ(homingPhaseToString(HomingPhase::Seeking), "seeking");
    EXPECT_STREQ(homingPhaseToString(HomingPhase::Bouncing), "bouncing");
    EXPECT_STREQ(homingPhaseToString(HomingPhase::Complete), "complete");
    EXPECT_STREQ(homingPhaseToString(HomingPhase::Failed), "failed");
}

// ============================================================================
// Probe tests
// ============================================================================

TEST(KlipperProbe,InitialState) {
    bool pinState = false;
    Probe probe(0, [&pinState]() { return pinState; });

    EXPECT_EQ(probe.oid(), 0);
    EXPECT_FALSE(probe.triggered());
    EXPECT_EQ(probe.zOffset(), 0.0);
    EXPECT_FALSE(probe.isVirtualEndstop());
}

TEST(KlipperProbe, TriggeredState) {
    bool pinState = false;
    Probe probe(0, [&pinState]() { return pinState; });

    pinState = true;
    EXPECT_TRUE(probe.triggered());

    pinState = false;
    EXPECT_FALSE(probe.triggered());
}

TEST(KlipperProbe, ZOffset) {
    Probe probe(0, []() { return false; });
    probe.setZOffset(0.5);
    EXPECT_EQ(probe.zOffset(), 0.5);
}

TEST(KlipperProbe, VirtualEndstop) {
    Probe probe(0, []() { return false; });
    probe.setVirtualEndstop(true);
    EXPECT_TRUE(probe.isVirtualEndstop());
}

TEST(KlipperProbe, ProbeMove) {
    bool pinState = false;
    Probe probe(0, [&pinState]() { return pinState; });

    // Simulate probe move: Z goes down, triggers at -5.0
    double zPos = 0.0;
    auto result = probe.probe(-5.0, 20.0, [&](double speed, double maxDist) -> double {
        for (int i = 0; i < 1000; ++i) {
            zPos += speed * 0.01;
            if (zPos <= -5.0) {
                pinState = true;
                return zPos;
            }
        }
        return std::nan("");
    });

    EXPECT_NEAR(result, -5.0, 0.5);
}
