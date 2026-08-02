/**
 * @file test_klipper_device_core.cpp
 * @brief Direct unit tests for KlipperDevice core command handlers.
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"

#include <memory>

using namespace tether::klipper;

class KlipperDeviceCoreTest : public ::testing::Test {
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
        ASSERT_TRUE(host_->downloadDictionary([this](){ dev_->pump(); }));
    }

    protocol::DataDictionary dict_;
    std::shared_ptr<transport::LoopbackTransport::SharedBuffer> hostToDev_, devToHost_;
    std::shared_ptr<transport::LoopbackTransport> hostT_, devT_;
    std::unique_ptr<device::KlipperDevice> dev_;
    std::shared_ptr<klippy::KlippyHost> host_;
};

TEST_F(KlipperDeviceCoreTest, InitialState) {
    EXPECT_FALSE(dev_->isShutdown());
    EXPECT_FALSE(dev_->isConfigFinalized());
    EXPECT_EQ(dev_->allocatedOidCount(), 0u);
    EXPECT_NE(dev_->configCrc(), 0u);
}

TEST_F(KlipperDeviceCoreTest, AllocateOids) {
    bool ok = host_->sendCommand("allocate_oids oid=%c", {protocol::ParamValue{5}});
    EXPECT_TRUE(ok);
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
    EXPECT_EQ(dev_->allocatedOidCount(), 5u);
}

TEST_F(KlipperDeviceCoreTest, GetStatus) {
    dev_->advanceClock(180000000);
    bool ok = host_->sendCommand("get_status", {});
    EXPECT_TRUE(ok);
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
    // Status response should be received; no crash.
    SUCCEED();
}

TEST_F(KlipperDeviceCoreTest, GetConfig) {
    bool ok = host_->sendCommand("get_config", {});
    EXPECT_TRUE(ok);
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
    SUCCEED();
}

TEST_F(KlipperDeviceCoreTest, ShutdownCommand) {
    bool ok = host_->sendCommand("shutdown", {});
    EXPECT_TRUE(ok);
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
    EXPECT_TRUE(dev_->isShutdown());
}

TEST_F(KlipperDeviceCoreTest, FinalizeConfig) {
    bool ok = host_->sendCommand("finalize_config crc=%u",
        {protocol::ParamValue{static_cast<int32_t>(0x12345678)}});
    EXPECT_TRUE(ok);
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
    EXPECT_TRUE(dev_->isConfigFinalized());
}

TEST_F(KlipperDeviceCoreTest, EnableDefaultCommandsIdempotent) {
    dev_->enableDefaultCommands();
    dev_->enableDefaultCommands();
    SUCCEED();
}

TEST_F(KlipperDeviceCoreTest, LastReceivedSeq) {
    // After dictionary download, the device has received at least one message.
    // The exact value depends on the protocol exchange, so we just verify
    // it's a valid 4-bit sequence number.
    EXPECT_LE(dev_->lastReceivedSeq(), 15u);
}

TEST_F(KlipperDeviceCoreTest, ConfigCrcNonZero) {
    EXPECT_NE(dev_->configCrc(), 0u);
}

TEST_F(KlipperDeviceCoreTest, ClockAdvance) {
    EXPECT_EQ(dev_->clock().ticks32(), 0u);
    dev_->advanceClock(1000);
    EXPECT_EQ(dev_->clock().ticks32(), 1000u);
}

TEST_F(KlipperDeviceCoreTest, DictionaryAccessible) {
    EXPECT_EQ(dev_->dictionary().messages().size(), dict_.messages().size());
}
