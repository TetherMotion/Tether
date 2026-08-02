/**
 * @file test_klipper_expected.cpp
 * @brief Tests for std::expected-based error handling wrappers.
 */

#include <gtest/gtest.h>
#include "tether/klipper/KlipperError.hpp"
#include "tether/klipper/KlippyExpected.hpp"
#include "tether/klipper/TransportExpected.hpp"
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"

#include <memory>

using namespace tether::klipper;

// --- KlipperError tests ---

TEST(KlipperErrorTest, DefaultConstructor) {
    KlipperError err;
    EXPECT_EQ(err.category(), ErrorCategory::Unknown);
    EXPECT_TRUE(err.message().empty());
    EXPECT_EQ(err.code(), 0);
}

TEST(KlipperErrorTest, TransportFactory) {
    auto err = KlipperError::transport("connection refused", 42);
    EXPECT_EQ(err.category(), ErrorCategory::Transport);
    EXPECT_EQ(err.message(), "connection refused");
    EXPECT_EQ(err.code(), 42);
    EXPECT_TRUE(err.isTransport());
}

TEST(KlipperErrorTest, ProtocolFactory) {
    auto err = KlipperError::protocol("bad CRC");
    EXPECT_EQ(err.category(), ErrorCategory::Protocol);
    EXPECT_TRUE(err.isProtocol());
}

TEST(KlipperErrorTest, TimeoutFactory) {
    auto err = KlipperError::timeout("clock sync timeout");
    EXPECT_EQ(err.category(), ErrorCategory::Timeout);
    EXPECT_TRUE(err.isTimeout());
}

TEST(KlipperErrorTest, DictionaryFactory) {
    auto err = KlipperError::dictionary("unknown format string");
    EXPECT_EQ(err.category(), ErrorCategory::Dictionary);
}

TEST(KlipperErrorTest, ClockSyncFactory) {
    auto err = KlipperError::clockSync("insufficient samples");
    EXPECT_EQ(err.category(), ErrorCategory::ClockSync);
}

TEST(KlipperErrorTest, GcodeFactory) {
    auto err = KlipperError::gcode("unknown command");
    EXPECT_EQ(err.category(), ErrorCategory::Gcode);
}

TEST(KlipperErrorTest, UdsFactory) {
    auto err = KlipperError::uds("endpoint not found");
    EXPECT_EQ(err.category(), ErrorCategory::Uds);
}

TEST(KlipperErrorTest, DeviceFactory) {
    auto err = KlipperError::device("shutdown");
    EXPECT_EQ(err.category(), ErrorCategory::Device);
}

TEST(KlipperErrorTest, ConfigFactory) {
    auto err = KlipperError::config("file not found");
    EXPECT_EQ(err.category(), ErrorCategory::Config);
}

TEST(KlipperErrorTest, FormatWithCode) {
    auto err = KlipperError::transport("timeout", 110);
    std::string s = err.format();
    EXPECT_NE(s.find("Transport"), std::string::npos);
    EXPECT_NE(s.find("timeout"), std::string::npos);
    EXPECT_NE(s.find("110"), std::string::npos);
}

TEST(KlipperErrorTest, FormatWithoutCode) {
    auto err = KlipperError::protocol("bad CRC");
    std::string s = err.format();
    EXPECT_NE(s.find("Protocol"), std::string::npos);
    EXPECT_NE(s.find("bad CRC"), std::string::npos);
    EXPECT_EQ(s.find("code="), std::string::npos);
}

TEST(KlipperErrorTest, CategoryToString) {
    EXPECT_EQ(categoryToString(ErrorCategory::Transport), "Transport");
    EXPECT_EQ(categoryToString(ErrorCategory::Protocol), "Protocol");
    EXPECT_EQ(categoryToString(ErrorCategory::Dictionary), "Dictionary");
    EXPECT_EQ(categoryToString(ErrorCategory::Config), "Config");
    EXPECT_EQ(categoryToString(ErrorCategory::ClockSync), "ClockSync");
    EXPECT_EQ(categoryToString(ErrorCategory::Gcode), "Gcode");
    EXPECT_EQ(categoryToString(ErrorCategory::Uds), "Uds");
    EXPECT_EQ(categoryToString(ErrorCategory::Device), "Device");
    EXPECT_EQ(categoryToString(ErrorCategory::Timeout), "Timeout");
    EXPECT_EQ(categoryToString(ErrorCategory::Unknown), "Unknown");
}

// --- Transport expected wrappers ---

class TransportExpectedTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto h2d = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        auto d2h = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        a_ = std::make_shared<transport::LoopbackTransport>();
        b_ = std::make_shared<transport::LoopbackTransport>();
        a_->wire(h2d, d2h);
        b_->wire(d2h, h2d);
    }
    std::shared_ptr<transport::LoopbackTransport> a_, b_;
};

TEST_F(TransportExpectedTest, TryOpenSuccess) {
    auto result = transport::tryOpen(*a_);
    EXPECT_TRUE(result.has_value());
}

TEST_F(TransportExpectedTest, TryWriteSuccess) {
    a_->open();
    b_->open();
    std::vector<uint8_t> data = {1, 2, 3, 4, 5};
    auto result = transport::tryWrite(*a_, data);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, data.size());
}

TEST_F(TransportExpectedTest, TryWriteOnClosedTransport) {
    std::vector<uint8_t> data = {1, 2, 3};
    auto result = transport::tryWrite(*a_, data);
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(result.error().isTransport());
}

TEST_F(TransportExpectedTest, TryReadSuccess) {
    a_->open();
    b_->open();
    std::vector<uint8_t> data = {10, 20, 30};
    a_->write(data);
    auto result = transport::tryRead(*b_, 100);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), data.size());
    EXPECT_EQ((*result)[0], 10);
    EXPECT_EQ((*result)[1], 20);
    EXPECT_EQ((*result)[2], 30);
}

TEST_F(TransportExpectedTest, TryReadOnClosedTransport) {
    auto result = transport::tryRead(*b_, 100);
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(result.error().isTransport());
}

TEST_F(TransportExpectedTest, TryReadAllSuccess) {
    a_->open();
    b_->open();
    std::vector<uint8_t> data = {1, 2, 3, 4};
    a_->write(data);
    auto result = transport::tryReadAll(*b_);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), data.size());
}

TEST_F(TransportExpectedTest, TryReadAllOnClosedTransport) {
    auto result = transport::tryReadAll(*b_);
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(result.error().isTransport());
}

// --- KlippyHost expected wrappers ---

class KlippyExpectedTest : public ::testing::Test {
protected:
    void SetUp() override {
        config::KlipperConfig cfg;
        config::withStandardCommands(cfg, 180000000);
        dict_ = cfg.build();

        auto h2d = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        auto d2h = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        hostT_ = std::make_shared<transport::LoopbackTransport>();
        devT_ = std::make_shared<transport::LoopbackTransport>();
        hostT_->wire(h2d, d2h);
        devT_->wire(d2h, h2d);
        hostT_->open();
        devT_->open();

        device::KlipperDeviceConfig dcfg;
        dcfg.clockFreqHz = 180000000;
        dev_ = std::make_unique<device::KlipperDevice>(devT_, dict_, dcfg);
        dev_->start();

        host_ = std::make_shared<klippy::KlippyHost>(hostT_);
    }

    protocol::DataDictionary dict_;
    std::shared_ptr<transport::LoopbackTransport> hostT_, devT_;
    std::unique_ptr<device::KlipperDevice> dev_;
    std::shared_ptr<klippy::KlippyHost> host_;
};

TEST_F(KlippyExpectedTest, TryConnectSuccess) {
    auto result = klippy::tryConnect(*host_);
    EXPECT_TRUE(result.has_value());
}

TEST_F(KlippyExpectedTest, TryDownloadDictionarySuccess) {
    klippy::tryConnect(*host_);
    auto result = klippy::tryDownloadDictionary(*host_, [this](){ dev_->pump(); });
    EXPECT_TRUE(result.has_value());
}

TEST_F(KlippyExpectedTest, TryDownloadDictionaryFailure) {
    // Don't connect first, and don't pump the device.
    auto result = klippy::tryDownloadDictionary(*host_, nullptr);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error().category(), ErrorCategory::Dictionary);
}

TEST_F(KlippyExpectedTest, TrySyncClockSuccess) {
    klippy::tryConnect(*host_);
    ASSERT_TRUE(klippy::tryDownloadDictionary(*host_, [this](){ dev_->pump(); }).has_value());
    dev_->advanceClock(180000000);
    auto result = klippy::trySyncClock(*host_, [this](){ dev_->pump(); });
    EXPECT_TRUE(result.has_value());
}

TEST_F(KlippyExpectedTest, TrySendCommandSuccess) {
    klippy::tryConnect(*host_);
    ASSERT_TRUE(klippy::tryDownloadDictionary(*host_, [this](){ dev_->pump(); }).has_value());
    auto result = klippy::trySendCommand(*host_, "get_clock", {});
    EXPECT_TRUE(result.has_value());
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
}

TEST_F(KlippyExpectedTest, TrySendCommandWithParams) {
    klippy::tryConnect(*host_);
    ASSERT_TRUE(klippy::tryDownloadDictionary(*host_, [this](){ dev_->pump(); }).has_value());
    auto result = klippy::trySendCommand(*host_, "allocate_oids oid=%c",
        {protocol::ParamValue{static_cast<int32_t>(3)}});
    EXPECT_TRUE(result.has_value());
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
}

TEST_F(KlippyExpectedTest, TrySendCommandNoParams) {
    klippy::tryConnect(*host_);
    ASSERT_TRUE(klippy::tryDownloadDictionary(*host_, [this](){ dev_->pump(); }).has_value());
    auto result = klippy::trySendCommand(*host_, "get_status");
    EXPECT_TRUE(result.has_value());
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
}

TEST_F(KlippyExpectedTest, FullExpectedPipeline) {
    // Use the expected-based API for the full connect/download/sync pipeline.
    auto connectResult = klippy::tryConnect(*host_);
    ASSERT_TRUE(connectResult.has_value()) << connectResult.error().format();

    auto dictResult = klippy::tryDownloadDictionary(*host_, [this](){ dev_->pump(); });
    ASSERT_TRUE(dictResult.has_value()) << dictResult.error().format();

    dev_->advanceClock(180000000);
    auto syncResult = klippy::trySyncClock(*host_, [this](){ dev_->pump(); });
    ASSERT_TRUE(syncResult.has_value()) << syncResult.error().format();

    auto cmdResult = klippy::trySendCommand(*host_, "get_clock", {});
    EXPECT_TRUE(cmdResult.has_value());
    for (int i = 0; i < 50; ++i) { dev_->pump(); host_->pump(); }
}
