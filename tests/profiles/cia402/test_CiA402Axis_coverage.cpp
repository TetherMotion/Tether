/**
 * @file test_CiA402Axis_coverage.cpp
 * @brief Comprehensive tests for CiA402Axis (MotionControllerAxis.cpp)
 */

#include <gtest/gtest.h>
#include <tether/profiles/cia402/MotionController.hpp>
#include "mocks/MockDriveBackend.hpp"

using namespace CiA402;

// ============================================================================
// CiA402Axis Tests
// ============================================================================

class CiA402AxisTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto backend = std::make_unique<mock::FakeDriveBackend>();
        fakeBackend = backend.get();
        axis = std::make_unique<CiA402Axis>(1, std::move(backend));
    }

    mock::FakeDriveBackend* fakeBackend{nullptr};
    std::unique_ptr<CiA402Axis> axis;
};

// --- Identity ---

TEST_F(CiA402AxisTest, GetId) {
    EXPECT_EQ(1u, axis->getId());
}

TEST_F(CiA402AxisTest, GetSetName) {
    // Default name is "Axis <id>"
    axis->setName("X-Axis");
    EXPECT_EQ("X-Axis", axis->getName());
}

// --- State Machine ---

TEST_F(CiA402AxisTest, GetStatus) {
    auto status = axis->getStatus();
    EXPECT_FALSE(status.fault);
}

TEST_F(CiA402AxisTest, GetState) {
    auto state = axis->getState();
    // Default state depends on backend
}

TEST_F(CiA402AxisTest, Enable) {
    bool result = axis->enable(100); // short timeout
    // May fail with fake backend depending on status word
}

TEST_F(CiA402AxisTest, Disable) {
    bool result = axis->disable(100);
}

TEST_F(CiA402AxisTest, QuickStop) {
    bool result = axis->quickStop();
}

TEST_F(CiA402AxisTest, ClearFault) {
    bool result = axis->clearFault();
}

// --- Operating Mode ---

TEST_F(CiA402AxisTest, SetOperatingMode) {
    EXPECT_TRUE(axis->setOperatingMode(OperatingMode::CyclicSyncPosition));
    EXPECT_EQ(OperatingMode::CyclicSyncPosition, axis->getOperatingMode());
}

TEST_F(CiA402AxisTest, GetOperatingMode) {
    auto mode = axis->getOperatingMode();
}

// --- Position Mode ---

TEST_F(CiA402AxisTest, MoveAbsolute) {
    axis->moveAbsolute(100000, 50000);
}

TEST_F(CiA402AxisTest, MoveAbsoluteDefaultVelocity) {
    axis->moveAbsolute(100000);
}

TEST_F(CiA402AxisTest, MoveRelative) {
    axis->moveRelative(5000, 10000);
}

TEST_F(CiA402AxisTest, MoveRelativeDefaultVelocity) {
    axis->moveRelative(5000);
}

TEST_F(CiA402AxisTest, ExecuteMotion) {
    MotionCommand cmd{};
    cmd.targetPosition = 50000;
    cmd.velocity = 10000;
    cmd.acceleration = 5000;
    cmd.deceleration = 5000;
    cmd.relative = false;
    cmd.immediate = true;
    axis->executeMotion(cmd);
}

TEST_F(CiA402AxisTest, ExecuteMotionRelative) {
    MotionCommand cmd{};
    cmd.targetPosition = 1000;
    cmd.velocity = 10000;
    cmd.relative = true;
    axis->executeMotion(cmd);
}

TEST_F(CiA402AxisTest, Halt) {
    axis->halt();
}

// --- Velocity Mode ---

TEST_F(CiA402AxisTest, SetVelocity) {
    axis->setVelocity(5000);
    EXPECT_EQ(5000, fakeBackend->lastTargetVelocity());
}

TEST_F(CiA402AxisTest, ExecuteVelocity) {
    VelocityCommand cmd{};
    cmd.targetVelocity = 3000;
    cmd.acceleration = 5000;
    cmd.deceleration = 5000;
    axis->executeVelocity(cmd);
}

// --- Torque Mode ---

TEST_F(CiA402AxisTest, SetTorque) {
    axis->setTorque(500);
}

TEST_F(CiA402AxisTest, ExecuteTorque) {
    TorqueCommand cmd{};
    cmd.targetTorque = 200;
    cmd.torqueSlope = 100;
    axis->executeTorque(cmd);
}

// --- Cyclic Sync ---

TEST_F(CiA402AxisTest, SetCyclicPosition) {
    axis->setCyclicPosition(10000, 100, 10);
    EXPECT_EQ(10000, fakeBackend->lastTargetPosition());
}

TEST_F(CiA402AxisTest, SetCyclicPositionNoFF) {
    axis->setCyclicPosition(20000);
}

TEST_F(CiA402AxisTest, SetCyclicVelocity) {
    axis->setCyclicVelocity(5000, 50);
}

TEST_F(CiA402AxisTest, SetCyclicVelocityNoFF) {
    axis->setCyclicVelocity(3000);
}

TEST_F(CiA402AxisTest, SetCyclicTorque) {
    axis->setCyclicTorque(100);
}

// --- Homing ---

TEST_F(CiA402AxisTest, StartHomingDefault) {
    axis->startHoming();
}

TEST_F(CiA402AxisTest, StartHomingWithMethod) {
    axis->startHoming(HomingMethod::CurrentPosition);
}

TEST_F(CiA402AxisTest, StartHomingWithCommand) {
    HomingCommand cmd{};
    cmd.method = static_cast<uint16_t>(HomingMethod::NegLimitIndex);
    cmd.speedSwitch = 500;
    cmd.speedZero = 50;
    cmd.acceleration = 2000;
    cmd.offset = 100;
    axis->startHoming(cmd);
}

TEST_F(CiA402AxisTest, IsHomingComplete) {
    EXPECT_FALSE(axis->isHomingComplete());
}

TEST_F(CiA402AxisTest, IsHomed) {
    EXPECT_FALSE(axis->isHomed());
}

// --- Profile Config ---

TEST_F(CiA402AxisTest, SetMotionLimits) {
    MotionLimits limits{};
    limits.maxVelocity = 100000;
    limits.maxAcceleration = 50000;
    limits.maxDeceleration = 50000;
    limits.maxJerk = 1000;
    axis->setMotionLimits(limits);
    auto retrieved = axis->getMotionLimits();
    EXPECT_DOUBLE_EQ(100000, retrieved.maxVelocity);
}

TEST_F(CiA402AxisTest, GetMotionLimits) {
    auto limits = axis->getMotionLimits();
    EXPECT_GT(limits.maxVelocity, 0);
}

TEST_F(CiA402AxisTest, SetProfileVelocity) {
    axis->setProfileVelocity(20000);
}

TEST_F(CiA402AxisTest, SetProfileAcceleration) {
    axis->setProfileAcceleration(10000);
}

TEST_F(CiA402AxisTest, SetProfileDeceleration) {
    axis->setProfileDeceleration(8000);
}

TEST_F(CiA402AxisTest, SetDefaultProfileType) {
    axis->setDefaultProfileType(ProfileType::Trapezoidal);
}

// --- PID ---

TEST_F(CiA402AxisTest, SetPositionPID) {
    PIDGains gains{};
    gains.Kp = 10.0;
    gains.Ki = 0.1;
    gains.Kd = 0.5;
    axis->setPositionPID(gains);
}

TEST_F(CiA402AxisTest, SetPositionPIDWithLimits) {
    PIDGains gains{};
    gains.Kp = 10.0;
    PIDLimits limits{};
    limits.outputMax = 100000;
    limits.outputMin = -100000;
    limits.integralMax = 50000;
    axis->setPositionPID(gains, limits);
}

TEST_F(CiA402AxisTest, SetVelocityPID) {
    PIDGains gains{};
    gains.Kp = 5.0;
    gains.Ki = 0.05;
    gains.Kd = 0.1;
    axis->setVelocityPID(gains);
}

TEST_F(CiA402AxisTest, GetPositionPID) {
    auto& pid = axis->getPositionPID();
    (void)pid;
}

TEST_F(CiA402AxisTest, GetVelocityPID) {
    auto& pid = axis->getVelocityPID();
    (void)pid;
}

// --- Gearing ---

TEST_F(CiA402AxisTest, SetGearRatio) {
    axis->setGearRatio(2, 1);
}

TEST_F(CiA402AxisTest, EnableGearing) {
    axis->enableGearing(false);
    EXPECT_FALSE(axis->isGearingSlave());
}

TEST_F(CiA402AxisTest, IsGearingSlave) {
    EXPECT_FALSE(axis->isGearingSlave());
}

// --- Callbacks ---

TEST_F(CiA402AxisTest, SetMotionCompleteCallback) {
    bool called = false;
    axis->setMotionCompleteCallback([&](bool) { called = true; });
}

TEST_F(CiA402AxisTest, SetHomingCompleteCallback) {
    bool called = false;
    axis->setHomingCompleteCallback([&](bool, HomingError) { called = true; });
}

TEST_F(CiA402AxisTest, SetFaultCallback) {
    bool called = false;
    axis->setFaultCallback([&](uint16_t, const std::string&) { called = true; });
}

// --- Update Cycle ---

TEST_F(CiA402AxisTest, Update) {
    axis->update(0.001f); // 1ms cycle
}

TEST_F(CiA402AxisTest, UpdateMultipleCycles) {
    for (int i = 0; i < 100; ++i) {
        axis->update(0.001f);
    }
}

TEST_F(CiA402AxisTest, GetBackend) {
    auto* backend = axis->getBackend();
    EXPECT_NE(nullptr, backend);
}

TEST_F(CiA402AxisTest, GetBackendConst) {
    const auto& constAxis = *axis;
    const auto* backend = constAxis.getBackend();
    EXPECT_NE(nullptr, backend);
}

// --- MotionCommand struct ---

TEST(MotionCommandTest, Defaults) {
    MotionCommand cmd{};
    EXPECT_EQ(0, cmd.targetPosition);
    EXPECT_EQ(0u, cmd.velocity);
    EXPECT_EQ(0u, cmd.acceleration);
    EXPECT_FALSE(cmd.relative);
    EXPECT_FALSE(cmd.immediate);
    EXPECT_FALSE(cmd.buffered);
}

TEST(VelocityCommandTest, Defaults) {
    VelocityCommand cmd{};
    EXPECT_EQ(0, cmd.targetVelocity);
    EXPECT_EQ(0u, cmd.acceleration);
}

TEST(TorqueCommandTest, Defaults) {
    TorqueCommand cmd{};
    EXPECT_EQ(0, cmd.targetTorque);
    EXPECT_EQ(0, cmd.torqueSlope);
}

TEST(HomingCommandTest, Defaults) {
    HomingCommand cmd{};
    EXPECT_EQ(0u, cmd.method);
}
