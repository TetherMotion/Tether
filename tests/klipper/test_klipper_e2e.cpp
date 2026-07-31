/**
 * @file test_klipper_e2e.cpp
 * @brief End-to-end tests: host connects to device, downloads dict, syncs clock.
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/motion/MotionReconstructor.hpp"
#include "tether/klipper/motion/MotionBlockSink.hpp"

#include <memory>

using namespace tether::klipper;

class KlipperE2E : public ::testing::Test {
protected:
    void SetUp() override {
        config::KlipperConfig cfg;
        config::withStandardCommands(cfg, 180000000);
        dict_ = cfg.build();

        hostToDev_ = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        devToHost_ = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        hostT_ = std::make_shared<transport::LoopbackTransport>();
        devT_ = std::make_shared<transport::LoopbackTransport>();
        hostT_->wire(hostToDev_, devToHost_);
        devT_->wire(devToHost_, hostToDev_);
        hostT_->open();
        devT_->open();

        device::KlipperDeviceConfig dcfg;
        dcfg.clockFreqHz = 180000000;
        dev_ = std::make_unique<device::KlipperDevice>(devT_, dict_, dcfg);
        dev_->start();

        host_ = std::make_shared<klippy::KlippyHost>(hostT_);
        host_->connect();
    }

    protocol::DataDictionary dict_;
    std::shared_ptr<transport::LoopbackTransport::SharedBuffer> hostToDev_, devToHost_;
    std::shared_ptr<transport::LoopbackTransport> hostT_, devT_;
    std::unique_ptr<device::KlipperDevice> dev_;
    std::shared_ptr<klippy::KlippyHost> host_;
};

TEST_F(KlipperE2E, DictionaryDownload) {
    bool ok = host_->downloadDictionary([this](){ dev_->pump(); });
    ASSERT_TRUE(ok);
    EXPECT_EQ(host_->dictionary().messages().size(), dict_.messages().size());
}

TEST_F(KlipperE2E, ClockSync) {
    ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    for (int i = 0; i < 10; ++i) {
        dev_->advanceClock(180000000); // 1 second
        host_->syncClock([this](){ dev_->pump(); });
    }
    EXPECT_TRUE(host_->clockSync().isSynchronised());
    EXPECT_GE(host_->clockSync().sampleCount(), 5u);
}

TEST_F(KlipperE2E, SendCommand) {
    ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    // Send a get_clock command via sendCommand.
    bool ok = host_->sendCommand("get_clock", {});
    EXPECT_TRUE(ok);
    // Pump to get the response.
    for (int i = 0; i < 100; ++i) {
        dev_->pump();
        host_->pump();
    }
}

TEST(KlipperMotionReconstructor, BasicReconstruction) {
    using namespace motion;
    using namespace objects;
    std::vector<StepCommand> steps = {{1000, 5, 0}, {800, 3, 0}};
    auto traj = MotionReconstructor::reconstruct(steps, 0, 80.0);
    EXPECT_EQ(traj.size(), 8u);
    // First step at clock 1000.
    EXPECT_EQ(traj[0].clock, 1000u);
    // Last step at clock 5000 + 3*800 = 7400.
    EXPECT_EQ(traj[7].clock, 7400u);
}

TEST(KlipperMotionBlock, TotalSteps) {
    using namespace motion;
    using namespace objects;
    MotionBlock block;
    block.steps = {{1000, 5, 0}, {800, 3, 0}};
    EXPECT_EQ(block.totalSteps(), 8u);
    EXPECT_EQ(block.totalDuration(), 7400u);
}

TEST(KlipperRecordingSink, RecordsBlocks) {
    using namespace motion;
    RecordingSink sink;
    MotionBlock block;
    block.oid = 0;
    block.steps = {{1000, 5, 0}};
    sink.emit(block);
    EXPECT_EQ(sink.blocks().size(), 1u);
    EXPECT_EQ(sink.blocks()[0].oid, 0u);
}
