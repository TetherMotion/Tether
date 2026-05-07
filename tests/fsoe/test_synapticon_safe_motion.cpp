#include <gtest/gtest.h>

#include "tether/drives/Synapticon/SafeMotionFSoE.hpp"

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
    EXPECT_EQ(Timing::kMinimumWatchdogTimeMs, 15);
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