/**
 * @file test_klipper_klippy_host.cpp
 * @brief Direct unit tests for KlippyHost.
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"

#include <memory>

using namespace tether::klipper;

class KlippyHostTest : public ::testing::Test {
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
    }

    protocol::DataDictionary dict_;
    std::shared_ptr<transport::LoopbackTransport::SharedBuffer> hostToDev_, devToHost_;
    std::shared_ptr<transport::LoopbackTransport> hostT_, devT_;
    std::unique_ptr<device::KlipperDevice> dev_;
    std::shared_ptr<klippy::KlippyHost> host_;
};

TEST_F(KlippyHostTest, Connect) {
    EXPECT_TRUE(host_->connect());
}

TEST_F(KlippyHostTest, IsReadyBeforeDownload) {
    EXPECT_FALSE(host_->isReady());
}

TEST_F(KlippyHostTest, DownloadDictionary) {
    host_->connect();
    bool ok = host_->downloadDictionary([this](){ dev_->pump(); });
    EXPECT_TRUE(ok);
    EXPECT_EQ(host_->dictionary().messages().size(), dict_.messages().size());
    EXPECT_TRUE(host_->isReady());
}

TEST_F(KlippyHostTest, SyncClock) {
    host_->connect();
    ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    for (int i = 0; i < 5; ++i) {
        dev_->advanceClock(180000000);
        EXPECT_TRUE(host_->syncClock([this](){ dev_->pump(); }));
    }
    EXPECT_TRUE(host_->clockSync().isSynchronised());
}

TEST_F(KlippyHostTest, SendCommand) {
    host_->connect();
    ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    EXPECT_TRUE(host_->sendCommand("get_clock", {}));
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
}

TEST_F(KlippyHostTest, AllocateOid) {
    host_->connect();
    ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    uint8_t oid = host_->allocateOid("stepper");
    EXPECT_LT(oid, 255u);
}

TEST_F(KlippyHostTest, OnResponse) {
    host_->connect();
    ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    bool responseReceived = false;
    host_->onResponse("clock clock=%u",
        [&responseReceived](const std::vector<protocol::ParamValue>&) {
            responseReceived = true;
        });
    host_->sendCommand("get_clock", {});
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
    EXPECT_TRUE(responseReceived);
}

TEST_F(KlippyHostTest, CheckTimeouts) {
    host_->connect();
    ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    host_->checkTimeouts();
    // checkTimeouts should not crash and should not change pending count.
    EXPECT_EQ(host_->serialQueue().pendingCount(), 0u);
}

TEST_F(KlippyHostTest, SerialQueueAccessible) {
    host_->connect();
    EXPECT_NO_THROW(host_->serialQueue());
}

TEST_F(KlippyHostTest, PumpOnEmptyTransport) {
    host_->connect();
    EXPECT_NO_THROW(host_->pump());
    // Pump on empty transport should not crash.
    // isReady() is false because dictionary is not downloaded yet.
    EXPECT_FALSE(host_->isReady());
}
