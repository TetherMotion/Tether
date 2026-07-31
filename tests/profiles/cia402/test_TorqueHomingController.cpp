/**
 * @file test_TorqueHomingController.cpp
 * @brief Tests for CiA402::TorqueHomingController
 *
 * Uses a small in-test functional fake drive backend that simulates
 * free motion and a mechanical stop, so the stall-detection and
 * back-off/re-approach logic can be exercised end-to-end.
 */

#include <gtest/gtest.h>

#include "tether/profiles/cia402/TorqueHomingController.hpp"
#include "tether/profiles/cia402/DriveBackend.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

using namespace CiA402;

namespace {

/// @brief Functional fake drive backend with simple physics.
///
/// Motion model:
///   velTarget = targetTorque * motorConstant   (units/s per 0.1% torque)
///   if pushing into the stop:  actualVel -> 0, actualTorque = targetTorque
///   else:                      actualVel -> velTarget (first-order lag),
///                              actualTorque = -sign(vel) * frictionTorque
///   pos += actualVel * dt
class FakeDriveBackend : public DriveBackend {
public:
    double motorConstant{1.0};        ///< units/s per 0.1% torque
    int16_t frictionTorque{20};       ///< load torque when moving freely
    int32_t stopPosition{100000};     ///< mechanical stop location
    int32_t stopBand{5};              ///< |pos - stopPosition| <= band => at stop
    double velLag{0.5};               ///< first-order velocity lag factor per tick
    uint32_t cycleMs{1};              ///< simulated cycle time

    int32_t actualPosition{0};
    int32_t actualVelocity{0};
    int16_t actualTorque{0};
    int16_t targetTorque{0};
    OperatingMode operatingMode{OperatingMode::NoMode};
    OperatingMode displayedMode{OperatingMode::NoMode};
    uint8_t errorRegister{0};
    bool connected{true};
    uint64_t simTimeMs{0};          ///< Simulated clock, advanced each tick

    /// Advance physics by one cycle.
    void tick() {
        simTimeMs += cycleMs;
        // Velocity follows commanded torque with a first-order lag.
        double velCmd = static_cast<double>(targetTorque) * motorConstant;
        double v = static_cast<double>(actualVelocity) * velLag +
                   velCmd * (1.0 - velLag);

        // Integrate position.
        double dt = static_cast<double>(cycleMs) / 1000.0;
        double newPos = static_cast<double>(actualPosition) + v * dt;

        // Clamp at the mechanical stop: the stop only blocks motion past
        // it in the approach direction (sign of stopPosition). Back-off
        // motion (opposite sign) is always allowed.
        bool pushingIntoStop = (stopPosition >= 0)
            ? (targetTorque > 0 && newPos >= stopPosition)
            : (targetTorque < 0 && newPos <= stopPosition);
        if (pushingIntoStop) {
            actualPosition = stopPosition;
            actualVelocity = 0;
            actualTorque = targetTorque;  // saturated against the stop
            return;
        }

        actualVelocity = static_cast<int32_t>(v);
        actualPosition = static_cast<int32_t>(newPos);
        // When moving freely, the motor sees a small friction load.
        actualTorque = (v >= 0) ? -frictionTorque : frictionTorque;
    }

    // --- DriveBackend interface (only the bits we care about) ---
    bool initialize() override { return true; }
    void deinitialize() override {}
    bool isConnected() const override { return connected; }
    std::string getName() const override { return "FakeDrive"; }
    bool updateInputs() override { return true; }
    bool updateOutputs() override { return true; }
    DriveState getState() const override { DriveState s; return s; }
    uint16_t readStatusWord() override { return 0; }
    void writeControlWord(uint16_t) override {}
    uint16_t readControlWord() const override { return 0; }
    bool setOperatingMode(OperatingMode m) override {
        operatingMode = m;
        displayedMode = m;
        return true;
    }
    OperatingMode getOperatingMode() const override { return operatingMode; }
    OperatingMode getDisplayedMode() const override { return displayedMode; }
    void setTargetPosition(int32_t) override {}
    int32_t getActualPosition() const override { return actualPosition; }
    int32_t getPositionDemand() const override { return actualPosition; }
    int32_t getFollowingError() const override { return 0; }
    void setPositionOffset(int32_t) override {}
    void setTargetVelocity(int32_t) override {}
    int32_t getActualVelocity() const override { return actualVelocity; }
    int32_t getVelocityDemand() const override { return actualVelocity; }
    void setVelocityOffset(int32_t) override {}
    void setTargetTorque(int16_t t) override { targetTorque = t; }
    int16_t getActualTorque() const override { return actualTorque; }
    void setTorqueOffset(int16_t) override {}
    void setProfileVelocity(uint32_t) override {}
    void setProfileAcceleration(uint32_t) override {}
    void setProfileDeceleration(uint32_t) override {}
    void setMotionProfileType(int16_t) override {}
    bool configureHoming(const HomingParams&) override { return true; }
    HomingParams getHomingParams() const override { return HomingParams{}; }
    bool configureInterpolation(const InterpolationParams&) override { return true; }
    bool addInterpolationPoint(int32_t) override { return true; }
    void clearInterpolationBuffer() override {}
    SDOResult readSDO(uint16_t, uint8_t, void*, size_t) override {
        return SDOResult{};
    }
    SDOResult writeSDO(uint16_t, uint8_t, const void*, size_t) override {
        return SDOResult{};
    }
    bool configure(const DriveConfig&) override { return true; }
    DriveConfig getConfiguration() const override { return DriveConfig{}; }
    bool storeParameters() override { return true; }
    bool restoreParameters() override { return true; }
    uint16_t getErrorCode() const override { return 0; }
    uint8_t getErrorRegister() const override { return errorRegister; }
    std::vector<uint16_t> getErrorHistory() const override { return {}; }
    bool clearErrorHistory() override { return true; }
    void setStateChangeCallback(StateChangeCallback) override {}
    void setErrorCallback(ErrorCallback) override {}
    void setWarningCallback(WarningCallback) override {}
    void setSyncCallback(SyncCallback) override {}
    uint32_t getCycleTimeUs() const override { return cycleMs * 1000; }
    bool setCycleTimeUs(uint32_t) override { return true; }
    uint64_t getLastUpdateTimestamp() const override { return 0; }
};

/// @brief Build callbacks that bind to a FakeDriveBackend.
TorqueHomingCallbacks makeCallbacks(FakeDriveBackend& fb,
                                     int32_t* homeOut = nullptr) {
    TorqueHomingCallbacks cbs;
    cbs.getActualPosition  = [&]() { return fb.getActualPosition(); };
    cbs.getActualVelocity  = [&]() { return fb.getActualVelocity(); };
    cbs.getActualTorque    = [&]() { return fb.getActualTorque(); };
    cbs.hasDriveFault      = [&]() { return fb.getErrorRegister() != 0; };
    cbs.setTargetTorque    = [&](int16_t t) { fb.setTargetTorque(t); };
    cbs.setOperatingMode   = [&](OperatingMode m) { return fb.setOperatingMode(m); };
    cbs.getOperatingMode   = [&]() { return fb.getOperatingMode(); };
    cbs.stopMotion         = [&]() { fb.setTargetTorque(0); };
    cbs.setHomePosition    = [&, homeOut](int32_t p) { if (homeOut) *homeOut = p; };
    cbs.getTimeMs          = [&]() { return fb.simTimeMs; };
    return cbs;
}

/// @brief Run the controller until it reports finished, ticking the fake
///        drive each cycle. Bounded by maxCycles to avoid infinite loops.
void runToCompletion(TorqueHomingController& ctrl, FakeDriveBackend& fb,
                     size_t maxCycles = 200000) {
    for (size_t i = 0; i < maxCycles; ++i) {
        fb.tick();
        if (!ctrl.update()) break;
    }
}

} // namespace

// ============================================================================
// Enums & structs
// ============================================================================

TEST(TorqueHomingEnums, StatesDistinct) {
    EXPECT_NE(static_cast<int>(TorqueHomingState::Idle),
              static_cast<int>(TorqueHomingState::Configuring));
    EXPECT_NE(static_cast<int>(TorqueHomingState::CoarseApproach),
              static_cast<int>(TorqueHomingState::BackOff));
    EXPECT_NE(static_cast<int>(TorqueHomingState::FineApproach),
              static_cast<int>(TorqueHomingState::Settling));
    EXPECT_NE(static_cast<int>(TorqueHomingState::Attained),
              static_cast<int>(TorqueHomingState::Error));
}

TEST(TorqueHomingEnums, ErrorsDistinct) {
    EXPECT_NE(static_cast<int>(TorqueHomingError::None),
              static_cast<int>(TorqueHomingError::Timeout));
    EXPECT_NE(static_cast<int>(TorqueHomingError::PositionDeviationExceeded),
              static_cast<int>(TorqueHomingError::PassTimeout));
    EXPECT_NE(static_cast<int>(TorqueHomingError::Aborted),
              static_cast<int>(TorqueHomingError::DriveFault));
}

TEST(TorqueHomingPassConfig, Defaults) {
    TorqueHomingPassConfig pc;
    EXPECT_EQ(pc.targetTorque, 0);
    EXPECT_EQ(pc.velocityLimit, 0);
    EXPECT_EQ(pc.stallWindowMs, 200u);
    EXPECT_EQ(pc.stallPositionThreshold, 5);
}

TEST(TorqueHomingConfig, Defaults) {
    TorqueHomingConfig cfg;
    EXPECT_TRUE(cfg.usePositionStallDetection);
    EXPECT_FALSE(cfg.useTorqueSaturationDetection);
    EXPECT_TRUE(cfg.usePositionDeviationLimit);
    EXPECT_EQ(cfg.velocityStrategy,
              VelocityLimitStrategy::ProportionalTorqueReduction);
    EXPECT_TRUE(cfg.restorePreviousMode);
    EXPECT_TRUE(cfg.setHomePosition);
    EXPECT_EQ(cfg.totalTimeoutMs, 60000u);
}

// ============================================================================
// Construction & validation
// ============================================================================

TEST(TorqueHomingControllerTest, DefaultConstruction) {
    TorqueHomingController ctrl;
    EXPECT_EQ(ctrl.getState(), TorqueHomingState::Idle);
    EXPECT_FALSE(ctrl.isComplete());
    EXPECT_FALSE(ctrl.hasError());
    EXPECT_EQ(ctrl.getLastError(), TorqueHomingError::None);
    EXPECT_EQ(ctrl.getCurrentPass(), -1);
}

TEST(TorqueHomingControllerTest, StartWithoutConfigFails) {
    TorqueHomingController ctrl;
    EXPECT_FALSE(ctrl.start());
    EXPECT_TRUE(ctrl.hasError());
    EXPECT_EQ(ctrl.getLastError(), TorqueHomingError::InvalidConfig);
}

TEST(TorqueHomingControllerTest, StartWithoutPositionSourceFails) {
    TorqueHomingController ctrl;
    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 500;
    ctrl.setConfig(cfg);
    // No backend, no callbacks.
    EXPECT_FALSE(ctrl.start());
    EXPECT_EQ(ctrl.getLastError(), TorqueHomingError::NotInitialized);
}

// ============================================================================
// Coarse-only homing
// ============================================================================

TEST(TorqueHomingControllerTest, CoarseOnlyReachesStopAndCompletes) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 100000;
    fb->motorConstant = 50.0;   // 50 units/s per 0.1% torque

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    int32_t homePos = INT32_MIN;
    auto cbs = makeCallbacks(*fb, &homePos);
    ctrl.setCallbacks(cbs);

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 200;          // +20% rated, +dir
    // velocityLimit well above free-running speed (200*50=10000) so the
    // controller does not bang-bang torque during the approach.
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 20000;
    cfg.usePositionDeviationLimit = false;      // disable for this test
    cfg.totalTimeoutMs = 30000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.isComplete());
    EXPECT_FALSE(ctrl.hasError());
    EXPECT_EQ(ctrl.getCurrentPass(), -1);
    EXPECT_NEAR(ctrl.getHomePosition(), fb->stopPosition, 5);
    EXPECT_NEAR(homePos, fb->stopPosition, 5);
    // Mode restored to NoMode (the fake's initial mode).
    EXPECT_EQ(fb->operatingMode, OperatingMode::NoMode);
}

// ============================================================================
// Coarse + fine passes
// ============================================================================

TEST(TorqueHomingControllerTest, CoarsePlusTwoFinePasses) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 50000;
    fb->motorConstant = 40.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    int32_t homePos = INT32_MIN;
    ctrl.setCallbacks(makeCallbacks(*fb, &homePos));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 300;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 20000;
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 60000;

    TorqueHomingPassConfig fine;
    fine.targetTorque = 100;
    fine.velocityLimit = 100000;
    fine.backOffDistance = -500;   // opposite sign of approach
    fine.stallWindowMs = 50;
    fine.stallPositionThreshold = 3;
    fine.passTimeoutMs = 15000;
    cfg.finePasses.push_back(fine);
    cfg.finePasses.push_back(fine);
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.isComplete());
    EXPECT_FALSE(ctrl.hasError());
    // After two fine passes the controller executed pass index 1 (0-based).
    EXPECT_EQ(ctrl.getCurrentPass(), 1);
    EXPECT_NEAR(ctrl.getHomePosition(), fb->stopPosition, 3);
    EXPECT_NEAR(homePos, fb->stopPosition, 3);
}

// ============================================================================
// Position-deviation limit
// ============================================================================

TEST(TorqueHomingControllerTest, PositionDeviationExceededErrors) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 1'000'000;  // very far away — axis will run far
    fb->motorConstant = 100.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 500;
    cfg.coarsePass.velocityLimit = 100000;  // high cap so it actually moves
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 60000;
    cfg.usePositionDeviationLimit = true;
    cfg.maxPositionDeviation = 5000;        // small travel limit
    cfg.totalTimeoutMs = 60000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.hasError());
    EXPECT_EQ(ctrl.getLastError(),
              TorqueHomingError::PositionDeviationExceeded);
    // Torque must be zeroed on error.
    EXPECT_EQ(fb->targetTorque, 0);
}

// ============================================================================
// Per-pass timeout (no stall reached)
// ============================================================================

TEST(TorqueHomingControllerTest, PassTimeoutWhenNoStall) {
    auto fb = std::make_shared<FakeDriveBackend>();
    // Stop far away and a high motor constant so the axis never stalls
    // within the per-pass timeout.
    fb->stopPosition = 10'000'000;
    fb->motorConstant = 100.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 100;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 100;       // very short
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 60000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.hasError());
    EXPECT_EQ(ctrl.getLastError(), TorqueHomingError::PassTimeout);
}

// ============================================================================
// Velocity limiting
// ============================================================================

TEST(TorqueHomingControllerTest, VelocityLimitReducesTorqueNearCap) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 1'000'000;  // far; we won't reach it
    // motorConstant chosen so free-running speed (500*10 = 5000) equals the
    // velocity cap. The controller must reduce torque as v approaches the cap.
    fb->motorConstant = 10.0;
    fb->velLag = 0.3;              // smoother approach

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 500;
    cfg.coarsePass.velocityLimit = 5000;     // cap well below free-running speed
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 10000;
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 30000;
    cfg.velocityStrategy = VelocityLimitStrategy::ProportionalTorqueReduction;
    cfg.softStartFraction = 0.9;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());

    // Run a few cycles to let velocity build toward the cap.
    bool sawReducedTorque = false;
    bool sawZeroTorque = false;
    for (size_t i = 0; i < 5000; ++i) {
        fb->tick();
        if (!ctrl.update()) break;
        int16_t t = fb->targetTorque;
        if (std::abs(fb->getActualVelocity()) > 4500 && t > 0 && t < 500) {
            sawReducedTorque = true;
        }
        if (std::abs(fb->getActualVelocity()) >= 5000 && t == 0) {
            sawZeroTorque = true;
        }
    }
    EXPECT_TRUE(sawReducedTorque)
        << "Proportional reduction should engage before the cap";
    // The controller should keep velocity near the cap; sawZeroTorque may or
    // may not trigger depending on lag, so we don't assert it.
    (void)sawZeroTorque;
}

// ============================================================================
// Abort restores mode
// ============================================================================

TEST(TorqueHomingControllerTest, AbortRestoresMode) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 100000;
    fb->motorConstant = 50.0;
    fb->operatingMode = OperatingMode::CyclicSyncPosition; // pretend prior mode

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 200;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 30000;
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 60000;
    cfg.restorePreviousMode = true;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    // Run a few cycles so we enter CoarseApproach and CST is active.
    for (int i = 0; i < 10; ++i) { fb->tick(); ctrl.update(); }
    EXPECT_EQ(fb->operatingMode, OperatingMode::CyclicSyncTorque);

    ctrl.abort();
    // Finish draining the error state.
    for (int i = 0; i < 5; ++i) { fb->tick(); ctrl.update(); }

    EXPECT_TRUE(ctrl.hasError());
    EXPECT_EQ(ctrl.getLastError(), TorqueHomingError::Aborted);
    EXPECT_EQ(fb->operatingMode, OperatingMode::CyclicSyncPosition);
    EXPECT_EQ(fb->targetTorque, 0);
}

// ============================================================================
// Drive fault
// ============================================================================

TEST(TorqueHomingControllerTest, DriveFaultDuringApproachErrors) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 100000;
    fb->motorConstant = 50.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 200;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 30000;
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 60000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    for (int i = 0; i < 20; ++i) { fb->tick(); ctrl.update(); }
    // Inject a fault.
    fb->errorRegister = 0x01;
    fb->tick();
    ctrl.update();

    EXPECT_TRUE(ctrl.hasError());
    EXPECT_EQ(ctrl.getLastError(), TorqueHomingError::DriveFault);
}

// ============================================================================
// Torque-saturation detector only
// ============================================================================

TEST(TorqueHomingControllerTest, TorqueSaturationOnlyDetectsStall) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 20000;
    fb->motorConstant = 50.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 200;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.stallTorqueTolerance = 30;
    cfg.coarsePass.passTimeoutMs = 20000;
    cfg.usePositionDeviationLimit = false;
    cfg.usePositionStallDetection = false;
    cfg.useTorqueSaturationDetection = true;
    cfg.totalTimeoutMs = 30000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.isComplete());
    EXPECT_FALSE(ctrl.hasError());
    EXPECT_NEAR(ctrl.getHomePosition(), fb->stopPosition, 5);
}

// ============================================================================
// Position-derivative detector only (default)
// ============================================================================

TEST(TorqueHomingControllerTest, PositionDerivativeOnlyDetectsStall) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 30000;
    fb->motorConstant = 60.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 250;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 20000;
    cfg.usePositionDeviationLimit = false;
    cfg.usePositionStallDetection = true;
    cfg.useTorqueSaturationDetection = false;
    cfg.totalTimeoutMs = 30000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.isComplete());
    EXPECT_FALSE(ctrl.hasError());
    EXPECT_NEAR(ctrl.getHomePosition(), fb->stopPosition, 5);
}

// ============================================================================
// Direction override
// ============================================================================

TEST(TorqueHomingControllerTest, DirectionOverrideFlipsTorqueSign) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = -50000;   // stop in the negative direction
    fb->motorConstant = 50.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    ctrl.setCallbacks(makeCallbacks(*fb));

    TorqueHomingConfig cfg;
    // Positive torque but we will pass direction = -1.
    cfg.coarsePass.targetTorque = 200;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 20000;
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 30000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start(-1));
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.isComplete());
    EXPECT_NEAR(ctrl.getHomePosition(), fb->stopPosition, 5);
}

// ============================================================================
// Home offset
// ============================================================================

TEST(TorqueHomingControllerTest, HomeOffsetApplied) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 12345;
    fb->motorConstant = 50.0;

    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    int32_t homePos = INT32_MIN;
    ctrl.setCallbacks(makeCallbacks(*fb, &homePos));

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 200;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 20000;
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 30000;
    cfg.homeOffset = 1000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    EXPECT_TRUE(ctrl.isComplete());
    EXPECT_NEAR(ctrl.getHomePosition(), fb->stopPosition + 1000, 5);
    EXPECT_NEAR(homePos, fb->stopPosition + 1000, 5);
}

// ============================================================================
// State change callback
// ============================================================================

TEST(TorqueHomingControllerTest, StateChangeCallbackFired) {
    auto fb = std::make_shared<FakeDriveBackend>();
    fb->stopPosition = 10000;
    fb->motorConstant = 50.0;

    std::vector<TorqueHomingState> seen;
    TorqueHomingController ctrl;
    ctrl.setBackend(fb);
    auto cbs = makeCallbacks(*fb);
    cbs.onStateChange = [&](TorqueHomingState s) { seen.push_back(s); };
    ctrl.setCallbacks(cbs);

    TorqueHomingConfig cfg;
    cfg.coarsePass.targetTorque = 200;
    cfg.coarsePass.velocityLimit = 100000;
    cfg.coarsePass.stallWindowMs = 50;
    cfg.coarsePass.stallPositionThreshold = 5;
    cfg.coarsePass.passTimeoutMs = 20000;
    cfg.usePositionDeviationLimit = false;
    cfg.totalTimeoutMs = 30000;
    ctrl.setConfig(cfg);

    EXPECT_TRUE(ctrl.start());
    runToCompletion(ctrl, *fb);

    ASSERT_FALSE(seen.empty());
    EXPECT_EQ(seen.front(), TorqueHomingState::Configuring);
    // Must eventually reach Attained.
    bool attained = false;
    for (auto s : seen) if (s == TorqueHomingState::Attained) attained = true;
    EXPECT_TRUE(attained);
}
