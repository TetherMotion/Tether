/**
 * @file test_sdo_abort_report.cpp
 * @brief Coverage for reportSdoAbort() — the user-facing CoE abort dump.
 *
 * Real aborts are produced by the emulator (missing object, read-only
 * object) so the report reads genuine lastSdoAbortCode() /
 * lastSdoWasDownload() / lastSdoAttemptedLength() state.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/SdoAbortReport.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

std::vector<uint8_t> le(uint32_t v, size_t n) {
    std::vector<uint8_t> d(n);
    for (size_t i = 0; i < n; ++i) d[i] = static_cast<uint8_t>(v >> (8 * i));
    return d;
}

class SdoAbortReportTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0x0442, "esc211");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    void addU16(uint16_t idx, uint8_t sub, uint16_t v, uint8_t access = 0x03) {
        slave_->addODEntry({idx, sub, 0x0006, 16, access, le(v, 2), "u16"});
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
};

}  // namespace

TEST_F(SdoAbortReportTest, NoAbortReturnsZero) {
    EXPECT_EQ(reportSdoAbort(master_.slave(0), "test"), 0u);
}

TEST_F(SdoAbortReportTest, UploadAbortMissingObject) {
    uint8_t buf[4] = {};
    size_t len = sizeof(buf);
    auto err = master_.slave(0).sdoRead(0x9999, 0, buf, len);
    ASSERT_NE(err, SlaveError::Ok);

    EXPECT_EQ(reportSdoAbort(master_.slave(0), "test"), 0x06020000u);
}

TEST_F(SdoAbortReportTest, DownloadAbortMissingObject) {
    const uint8_t payload[2] = {1, 2};
    auto err = master_.slave(0).sdoWrite(0x9999, 0, payload, sizeof(payload));
    ASSERT_NE(err, SlaveError::Ok);

    EXPECT_EQ(reportSdoAbort(master_.slave(0), "test"), 0x06020000u);
}

TEST_F(SdoAbortReportTest, DownloadAbortReadOnlyObject) {
    addU16(0x7000, 0, 0x1234, 0x01);  // read-only
    const uint8_t payload[2] = {0xAA, 0xBB};
    auto err = master_.slave(0).sdoWrite(0x7000, 0, payload, sizeof(payload));
    ASSERT_NE(err, SlaveError::Ok);

    EXPECT_EQ(reportSdoAbort(master_.slave(0), "test"), 0x06010002u);
}

TEST_F(SdoAbortReportTest, SuccessfulSdoLeavesNoAbort) {
    addU16(0x7001, 0, 0x1234);
    uint8_t buf[2] = {};
    size_t len = sizeof(buf);
    ASSERT_EQ(master_.slave(0).sdoRead(0x7001, 0, buf, len),
              SlaveError::Ok);
    EXPECT_EQ(reportSdoAbort(master_.slave(0), "test"), 0u);
}
