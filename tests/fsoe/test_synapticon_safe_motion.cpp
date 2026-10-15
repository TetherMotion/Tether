#include <gtest/gtest.h>

#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"

using namespace EtherCAT::Drives::Synapticon::SafeMotion;

namespace {

void advance(MainInstance& main,
             SafeMotionServoEmulator& servo,
             uint64_t& now_ms,
             int cycles,
             double requested_velocity)
{
    for (int cycle = 0; cycle < cycles; ++cycle) {
        servo.step(main.motionAllowed() ? requested_velocity : 0.0, 0.015);
        now_ms += 15;
        ASSERT_TRUE(main.exchangeWith(servo, now_ms));
    }
}

class CountingTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    explicit CountingTask(int& count_ref)
        : count(count_ref)
    {
    }

    bool update(EtherCAT::DS402Master&, double) override
    {
        ++count;
        return true;
    }

    int& count;
};

} // namespace

TEST(SynapticonSafeMotion, TimingConstantsMatchVendorDocumentation) {
    EXPECT_EQ(Timing::kMinimumWatchdogTimeMs, 200);
    EXPECT_EQ(Timing::kTypicalMasterToSlaveDelayMs, 8);
    EXPECT_EQ(Timing::kTypicalSlaveToMasterDelayMs, 7);
    EXPECT_EQ(Timing::kTypicalRoundTripTimeMs, 15);
    EXPECT_EQ(Timing::kInternalDriveDelayMs, 2);
}

TEST(SynapticonSafeMotion, DisabledFeatureDoesNotGateMotion) {
    MainConfig main_config;
    main_config.feature_enabled = false;

    ServoEmulatorConfig servo_config;
    servo_config.connection_id = main_config.connection_id;

    MainInstance main(main_config);
    SafeMotionServoEmulator servo(servo_config);
    ASSERT_TRUE(main.initialize());
    ASSERT_TRUE(servo.initialize());

    EXPECT_TRUE(main.motionAllowed());
    EXPECT_TRUE(main.exchangeWith(servo, 15));
    EXPECT_TRUE(main.motionAllowed());
}

TEST(SynapticonSafeMotion, EnabledFeatureReleasesMotionAfterHandshake) {
    MainConfig main_config;
    main_config.feature_enabled = true;
    main_config.connection_id = 0x2222;

    ServoEmulatorConfig servo_config;
    servo_config.connection_id = main_config.connection_id;

    MainInstance main(main_config);
    SafeMotionServoEmulator servo(servo_config);
    ASSERT_TRUE(main.initialize());
    ASSERT_TRUE(servo.initialize());

    main.requestMotionEnabled();
    uint64_t now_ms = 0;
    advance(main, servo, now_ms, 8, 1200.0);

    ASSERT_TRUE(main.hasStatus());
    EXPECT_TRUE(main.motionAllowed());
    EXPECT_TRUE(main.status().safe_position_valid);
    EXPECT_TRUE(main.status().safe_velocity_valid);
    EXPECT_GT(servo.status().safe_position, 0);
    EXPECT_GT(servo.status().safe_velocity, 0);
}

TEST(SynapticonSafeMotion, ErrorAndRestartAcknowledgementRecoverMotion) {
    MainConfig main_config;
    main_config.feature_enabled = true;
    main_config.connection_id = 0x3333;

    ServoEmulatorConfig servo_config;
    servo_config.connection_id = main_config.connection_id;

    MainInstance main(main_config);
    SafeMotionServoEmulator servo(servo_config);
    ASSERT_TRUE(main.initialize());
    ASSERT_TRUE(servo.initialize());

    main.requestMotionEnabled();

    uint64_t now_ms = 0;
    advance(main, servo, now_ms, 8, 1500.0);
    ASSERT_TRUE(main.motionAllowed());

    servo.injectError(true);
    advance(main, servo, now_ms, 1, 1500.0);
    EXPECT_TRUE(main.status().error_active);
    EXPECT_FALSE(main.motionAllowed());

    main.pulseErrorAcknowledge();
    advance(main, servo, now_ms, 1, 1500.0);
    EXPECT_FALSE(main.status().error_active);
    EXPECT_TRUE(main.status().restart_acknowledge_required);
    EXPECT_FALSE(main.motionAllowed());

    main.pulseRestartAcknowledge();
    advance(main, servo, now_ms, 2, 1500.0);
    EXPECT_TRUE(main.motionAllowed());
    EXPECT_FALSE(main.status().restart_acknowledge_required);
}

TEST(SynapticonSafeMotion, StatusMotionAllowedCoversEveryStopFunction) {
    Status released{};
    released.sto_active = released.ss1_active = released.ss2_active =
        released.sos_active = released.brake_engaged = false;
    EXPECT_TRUE(released.motionAllowed());

    const auto expect_gated = [](const auto& mutate) {
        Status s{};
        s.sto_active = s.ss1_active = s.ss2_active = s.sos_active =
            s.brake_engaged = false;
        mutate(s);
        EXPECT_FALSE(s.motionAllowed());
    };
    expect_gated([](Status& s) { s.sto_active = true; });
    expect_gated([](Status& s) { s.ss1_active = true; });
    expect_gated([](Status& s) { s.ss2_active = true; });
    expect_gated([](Status& s) { s.sos_active = true; });
    expect_gated([](Status& s) { s.brake_engaged = true; });   // SBC
    expect_gated([](Status& s) { s.error_active = true; });
    expect_gated([](Status& s) { s.restart_acknowledge_required = true; });
}

TEST(SynapticonSafeMotion, DemandedStopFunctionsHaltEmulatedMotion) {
    // Every demanded stop function must hold the axis completely:
    // STO, SS1, SS2, SOS, SBC (brake) and SLS (no modelled speed limit
    // → halt).  The gate must fire twice — at the master the instant
    // the demand is set (before the slave's status echo), and inside
    // the servo even when a velocity is still fed in (the blackchannel
    // motion thread commands unconditionally).
    for (int function = 0; function < 6; ++function) {
        MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.connection_id = 0x4444;

        ServoEmulatorConfig servo_config;
        servo_config.connection_id = main_config.connection_id;

        MainInstance main(main_config);
        SafeMotionServoEmulator servo(servo_config);
        ASSERT_TRUE(main.initialize());
        ASSERT_TRUE(servo.initialize());

        main.requestMotionEnabled();
        uint64_t now_ms = 0;
        advance(main, servo, now_ms, 8, 1000.0);
        ASSERT_TRUE(main.motionAllowed()) << "function " << function;
        const int32_t pos_before = servo.status().safe_position;
        ASSERT_GT(pos_before, 0) << "function " << function;

        // Demand a single stop function; everything else stays released.
        Command stop = Command::motionEnabled();
        switch (function) {
            case 0: stop.sto = true; break;
            case 1: stop.ss1 = true; break;
            case 2: stop.ss2 = true; break;
            case 3: stop.sos = true; break;
            case 4: stop.brake_engage = true; break;   // SBC
            case 5: stop.sls[0] = true; break;         // SLS1
        }
        main.setCommand(stop);

        // The master gate drops immediately — no slave ack needed.
        EXPECT_FALSE(main.motionAllowed()) << "function " << function;

        advance(main, servo, now_ms, 4, 1000.0);
        EXPECT_EQ(servo.status().safe_position, pos_before)
            << "function " << function;
        EXPECT_EQ(servo.status().safe_velocity, 0) << "function " << function;
        EXPECT_FALSE(servo.controlMotionAllowed()) << "function " << function;

        // Feed a velocity straight into the servo — its internal gate
        // must hold the axis regardless of the caller.
        servo.step(1000.0, 0.015);
        EXPECT_EQ(servo.status().safe_position, pos_before)
            << "function " << function;
    }
}

TEST(SynapticonSafeMotion, SuppressDemandsReleasesGatingButNotErrorRestart) {
    MainConfig main_config;
    main_config.feature_enabled = true;
    main_config.connection_id = 0x6666;

    ServoEmulatorConfig servo_config;
    servo_config.connection_id = main_config.connection_id;
    servo_config.suppress_motion_demands = true;

    MainInstance main(main_config);
    SafeMotionServoEmulator servo(servo_config);
    ASSERT_TRUE(main.initialize());
    ASSERT_TRUE(servo.initialize());

    main.requestMotionEnabled();
    uint64_t now_ms = 0;
    advance(main, servo, now_ms, 8, 800.0);
    ASSERT_TRUE(servo.controlMotionAllowed());

    // Demanded STO: the published status still mirrors the demand (the
    // master sees a compliant drive) but the emulated power stage is
    // released — the --override-safety semantics.
    Command stop = Command::motionEnabled();
    stop.sto = true;
    main.setCommand(stop);
    advance(main, servo, now_ms, 3, 800.0);
    EXPECT_TRUE(servo.status().sto_active);
    EXPECT_TRUE(servo.controlMotionAllowed());

    const int32_t pos = servo.status().safe_position;
    servo.step(800.0, 0.015);
    EXPECT_GT(servo.status().safe_position, pos);

    // Internal error/restart conditions still gate under suppression.
    servo.injectError(true);
    EXPECT_FALSE(servo.controlMotionAllowed());
}

TEST(SynapticonSafeMotion, OneActiveCommandPolarityNormalizesDecode) {
    // Expose the protected decode-normalization hook.
    struct TestServo : SafeMotionServoEmulator {
        using SafeMotionServoEmulator::SafeMotionServoEmulator;
        using SafeMotionServoEmulator::normalizeCommand;
    };

    // A one-active "all functions demanded" wire word decodes to
    // "nothing demanded" under the standard zero-active semantics —
    // normalization must flip the function bits back to demanded while
    // leaving the one-active handshake bits untouched.
    ServoEmulatorConfig cfg;
    cfg.command_active_high = true;
    TestServo servo(cfg);

    Command decoded = Command::motionEnabled();
    decoded.error_acknowledge = true;
    servo.normalizeCommand(decoded);
    EXPECT_TRUE(decoded.sto);
    EXPECT_TRUE(decoded.ss1);
    EXPECT_TRUE(decoded.ss2);
    EXPECT_TRUE(decoded.sos);
    EXPECT_TRUE(decoded.brake_engage);
    for (const bool s : decoded.sls) {
        EXPECT_TRUE(s);
    }
    EXPECT_TRUE(decoded.error_acknowledge);
    EXPECT_FALSE(decoded.motionPermitted());

    // Default (zero-active) polarity leaves the command untouched.
    TestServo zero_polarity(ServoEmulatorConfig{});
    Command kept = Command::motionEnabled();
    zero_polarity.normalizeCommand(kept);
    EXPECT_TRUE(kept.motionPermitted());
}

TEST(DS402Master, CyclicTaskExecutesWithoutDriveControllers) {
    EtherCAT::DS402Master master;
    int count = 0;
    auto task = std::make_unique<CountingTask>(count);

    ASSERT_TRUE(master.addCyclicTask(std::move(task)));
    EXPECT_TRUE(master.updateMotionControllers(0.001));
    EXPECT_EQ(count, 1);

    master.clearCyclicTasks();
    EXPECT_TRUE(master.updateMotionControllers(0.001));
    EXPECT_EQ(count, 1);
}