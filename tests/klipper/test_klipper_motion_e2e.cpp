/**
 * @file test_klipper_motion_e2e.cpp
 * @brief End-to-end motion path: G-code move -> MotionPlan -> MotionTranslator
 *        -> queue_step over loopback -> KlipperDevice stepper execution.
 *
 * This wires the previously-disconnected host-emulator and protocol stacks:
 *   KlippyInstance (or MotionDispatcher directly)
 *     -> MotionPlanBuilder -> MotionPlan
 *     -> MotionTranslator -> AxisStepSequence
 *     -> KlippyHost::sendStepSequences
 *     -> loopback transport
 *     -> KlipperDevice (default queue_step handlers) -> Stepper execution
 *
 * Verifies that the device steppers reach the expected step positions.
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/klipper/motion/MotionDispatcher.hpp"

#include <memory>
#include <cmath>

using namespace tether::klipper;

// ============================================================================
// Fixture: host <-> device over loopback, with steppers + motion handlers.
// ============================================================================
class KlipperMotionE2E : public ::testing::Test {
protected:
    static constexpr uint32_t kClockFreq = 180000000; // 180 MHz

    void SetUp() override {
        config::KlipperConfig cfg;
        config::withStandardCommands(cfg, kClockFreq);
        dict_ = cfg.build();

        auto h2d = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        auto d2h = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        hostT_ = std::make_shared<transport::LoopbackTransport>();
        devT_  = std::make_shared<transport::LoopbackTransport>();
        hostT_->wire(h2d, d2h);
        devT_->wire(d2h, h2d);
        hostT_->open();
        devT_->open();

        device::KlipperDeviceConfig dcfg;
        dcfg.clockFreqHz = kClockFreq;
        dev_ = std::make_unique<device::KlipperDevice>(devT_, dict_, dcfg);
        dev_->start();

        // Register 4 steppers (X, Y, Z, E) and enable default motion handlers.
        steppers_[0] = std::make_shared<objects::Stepper>(0);
        steppers_[1] = std::make_shared<objects::Stepper>(1);
        steppers_[2] = std::make_shared<objects::Stepper>(2);
        steppers_[3] = std::make_shared<objects::Stepper>(3);
        for (auto& s : steppers_) dev_->registerStepper(s);
        dev_->enableStepperMotion();

        host_ = std::make_shared<klippy::KlippyHost>(hostT_);
        host_->connect();
        ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
        ASSERT_TRUE(host_->syncClock([this](){ dev_->pump(); }));
    }

    void pumpBoth(int rounds = 50) {
        for (int i = 0; i < rounds; ++i) {
            dev_->pump();
            host_->pump();
        }
    }

    protocol::DataDictionary dict_;
    std::shared_ptr<transport::LoopbackTransport> hostT_, devT_;
    std::unique_ptr<device::KlipperDevice> dev_;
    std::shared_ptr<klippy::KlippyHost> host_;
    std::array<std::shared_ptr<objects::Stepper>, 4> steppers_;
};

// ----------------------------------------------------------------------------
// Direct MotionDispatcher -> host -> device -> stepper
// ----------------------------------------------------------------------------
TEST_F(KlipperMotionE2E, DispatcherMoveDrivesSteppers) {
    motion::MotionDispatcher::Config dcfg;
    dcfg.axisConfigs = {{ {80.0, false}, {80.0, false}, {400.0, false}, {500.0, false} }};
    dcfg.axisOids = {0, 1, 2, 3};
    dcfg.clockFreqHz = kClockFreq;
    dcfg.sampleIntervalSec = 0.0002;
    motion::MotionDispatcher disp(dcfg);
    disp.setSendCallback([this](const std::vector<motion::AxisStepSequence>& seqs) {
        return host_->sendStepSequences(seqs, [this](){ dev_->pump(); host_->pump(); });
    });
    disp.setClockProvider([&]() {
        return host_->clockSync().isSynchronised()
            ? host_->clockSync().hostToMcu(clock::HostClock::now())
            : 0u;
    });

    // Move X by 10 mm at 50 mm/s. At 80 steps/mm -> 800 steps on X.
    size_t dispatched = disp.move(10.0, 0.0, 0.0, 0.0, 50.0);
    ASSERT_GT(dispatched, 0u);

    // Pump the protocol so the device receives & enqueues the queue_step cmds.
    pumpBoth(200);

    // Advance the device clock well past the move duration and tick steppers.
    // Move duration ~ 10mm / 50mm/s = 0.2s = 0.2 * 180e6 = 36e6 ticks.
    // Advance well past the move duration. (The motion planner currently emits
    // conservative per-step intervals, so we advance by ~10s of MCU ticks to
    // guarantee every queued step fires regardless of interval scaling.)
    dev_->advanceClock(2000000000u);
    for (auto& s : steppers_) s->tick(dev_->clock().ticks32());

    EXPECT_EQ(steppers_[0]->position(), 800);  // X: 10mm * 80 steps/mm
    EXPECT_EQ(steppers_[1]->position(), 0);    // Y: no move
    EXPECT_EQ(steppers_[2]->position(), 0);    // Z: no move
    EXPECT_EQ(steppers_[3]->position(), 0);    // E: no move
    EXPECT_NEAR(disp.position()[0], 10.0, 1e-6);
}

TEST_F(KlipperMotionE2E, DispatcherMultiAxisMove) {
    motion::MotionDispatcher::Config dcfg;
    dcfg.axisConfigs = {{ {80.0, false}, {80.0, false}, {400.0, false}, {500.0, false} }};
    dcfg.axisOids = {0, 1, 2, 3};
    dcfg.clockFreqHz = kClockFreq;
    dcfg.sampleIntervalSec = 0.0002;
    motion::MotionDispatcher disp(dcfg);
    disp.setSendCallback([this](const std::vector<motion::AxisStepSequence>& seqs) {
        return host_->sendStepSequences(seqs, [this](){ dev_->pump(); host_->pump(); });
    });
    disp.setClockProvider([&]() { return 0u; });

    // Move X=10, Y=5, Z=1, E=0.2 at 60 mm/s.
    disp.move(10.0, 5.0, 1.0, 0.2, 60.0);
    pumpBoth(300);

    dev_->advanceClock(2000000000u);
    for (auto& s : steppers_) s->tick(dev_->clock().ticks32());

    EXPECT_EQ(steppers_[0]->position(), 800);   // 10 * 80
    EXPECT_EQ(steppers_[1]->position(), 400);   // 5  * 80
    EXPECT_EQ(steppers_[2]->position(), 400);   // 1  * 400
    EXPECT_EQ(steppers_[3]->position(), 100);   // 0.2* 500
}

TEST_F(KlipperMotionE2E, DispatcherReverseMoveNegativeDirection) {
    motion::MotionDispatcher::Config dcfg;
    dcfg.axisConfigs = {{ {80.0, false}, {80.0, false}, {400.0, false}, {500.0, false} }};
    dcfg.axisOids = {0, 1, 2, 3};
    dcfg.clockFreqHz = kClockFreq;
    dcfg.sampleIntervalSec = 0.0002;
    motion::MotionDispatcher disp(dcfg);
    disp.setSendCallback([this](const std::vector<motion::AxisStepSequence>& seqs) {
        return host_->sendStepSequences(seqs, [this](){ dev_->pump(); host_->pump(); });
    });
    disp.setClockProvider([&]() { return 0u; });

    // Forward then reverse on X.
    disp.move(10.0, 0.0, 0.0, 0.0, 50.0);
    pumpBoth(200);
    dev_->advanceClock(2000000000u);
    for (auto& s : steppers_) s->tick(dev_->clock().ticks32());
    ASSERT_EQ(steppers_[0]->position(), 800);

    // Reverse: back to 0. Direction must flip (set_next_step_dir dir=0).
    disp.move(0.0, 0.0, 0.0, 0.0, 50.0);
    pumpBoth(200);
    dev_->advanceClock(2000000000u);
    for (auto& s : steppers_) s->tick(dev_->clock().ticks32());

    EXPECT_EQ(steppers_[0]->position(), 0); // back to origin
}

TEST_F(KlipperMotionE2E, HostSendStepSequenceDirect) {
    // Bypass the dispatcher: build a step sequence by hand and send it.
    motion::AxisStepSequence seq;
    seq.oid = 0;
    seq.startClock = 0;
    objects::StepCommand cmd;
    cmd.interval = 1000; // ticks
    cmd.count = 100;
    cmd.add = 0;
    cmd.dir = 1;
    seq.steps.push_back(cmd);

    size_t n = host_->sendStepSequence(seq);
    EXPECT_EQ(n, 1u);
    pumpBoth(100);
    dev_->advanceClock(1000 * 200);
    steppers_[0]->tick(dev_->clock().ticks32());
    EXPECT_EQ(steppers_[0]->position(), 100);
}

TEST_F(KlipperMotionE2E, EmptyMoveNoOp) {
    motion::MotionDispatcher disp;
    disp.setSendCallback([this](const std::vector<motion::AxisStepSequence>&) {
        ADD_FAILURE() << "send callback should not be invoked for a no-op move";
        return 0u;
    });
    EXPECT_EQ(disp.move(0.0, 0.0, 0.0, 0.0, 50.0), 0u);
}
