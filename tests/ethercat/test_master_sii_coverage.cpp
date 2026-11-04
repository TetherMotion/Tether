/**
 * @file test_master_sii_coverage.cpp
 * @brief Emulator-backed coverage for Master SII helpers — mailbox
 *        auto-configuration (SII and ESI driven), process-data SM
 *        configuration from ESI, the discovery summary log and SII
 *        identity verification.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/ethercat/SlaveIdentity.hpp"
#include "tether/sii/SIIReader.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

constexpr uint32_t kVendor = 0x00000002;
constexpr uint32_t kProduct = 0x0FA13052;

class MasterSiiCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = std::make_unique<SlaveEmulator>();
        Emulator::SIIConfig cfg;
        cfg.vendor_id = kVendor;
        cfg.product_code = kProduct;
        cfg.mailbox_protocol = 0x04;
        cfg.sync_managers.push_back(
            {0x1000, 128,
             std::bit_cast<EtherCAT::SyncManager::SMControlReg>(uint8_t(0x26)),
             std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(uint8_t(0x01))});
        cfg.sync_managers.push_back(
            {0x1080, 128,
             std::bit_cast<EtherCAT::SyncManager::SMControlReg>(uint8_t(0x22)),
             std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(uint8_t(0x01))});
        slave->setSIIConfig(cfg);
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
    }

    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    static ESI::DeviceInfo makeDevice() {
        ESI::DeviceInfo dev;
        dev.name = "test-device";
        dev.vendorId = kVendor;
        dev.productCode = kProduct;
        return dev;
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
};

// ============================================================================
// autoConfigureMailbox — SII-driven
// ============================================================================

TEST_F(MasterSiiCoverageTest, AutoConfigureMailboxFromSii) {
    EXPECT_TRUE(master_.autoConfigureMailbox(SlaveAddress(0),
                                             Tether::Platform::LogLevel::Debug));
}

TEST_F(MasterSiiCoverageTest, AutoConfigureMailboxMissingSlaveFails) {
    EXPECT_FALSE(master_.autoConfigureMailbox(SlaveAddress(9),
                                              Tether::Platform::LogLevel::Info));
}

// ============================================================================
// autoConfigureMailbox — ESI-driven
// ============================================================================

TEST_F(MasterSiiCoverageTest, AutoConfigureMailboxFromEsi) {
    auto dev = makeDevice();
    dev.mailbox.protocols = 0x0004;
    ESI::SyncManagerEntry mbxOut{0x1000, 128, {}, {}, "MBoxOut"};
    ESI::SyncManagerEntry mbxIn{0x1080, 128, {}, {}, "MBoxIn"};
    dev.syncManagers = {mbxOut, mbxIn};
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};

    EXPECT_TRUE(master_.autoConfigureMailbox(esi, SlaveAddress(0),
                                             Tether::Platform::LogLevel::Debug));
}

TEST_F(MasterSiiCoverageTest, AutoConfigureMailboxFromEsiDefaults) {
    auto dev = makeDevice();   // no sync managers — MBoxOut/In defaults arm
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};
    EXPECT_TRUE(master_.autoConfigureMailbox(esi, SlaveAddress(0),
                                             Tether::Platform::LogLevel::Info));
}

TEST_F(MasterSiiCoverageTest, AutoConfigureMailboxEsiEmptyFails) {
    ESIFile esi{std::vector<ESI::DeviceInfo>{}};
    EXPECT_FALSE(master_.autoConfigureMailbox(esi, SlaveAddress(0),
                                              Tether::Platform::LogLevel::Info));
}

TEST_F(MasterSiiCoverageTest, AutoConfigureMailboxEsiMissingSlaveFails) {
    auto dev = makeDevice();
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};
    EXPECT_FALSE(master_.autoConfigureMailbox(esi, SlaveAddress(9),
                                              Tether::Platform::LogLevel::Info));
}

// ============================================================================
// configureProcessDataSyncManagersFromESI
// ============================================================================

TEST_F(MasterSiiCoverageTest, ProcessDataSmsFromEsi) {
    auto dev = makeDevice();
    ESI::SyncManagerEntry outputs{0x1100, 12, {}, {}, "Outputs"};
    ESI::SyncManagerEntry inputs{0x1180, 12, {}, {}, "Inputs"};
    dev.syncManagers = {outputs, inputs};
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};

    EXPECT_TRUE(master_.configureProcessDataSyncManagersFromESI(
        esi, SlaveAddress(0)));
}

TEST_F(MasterSiiCoverageTest, ProcessDataSmsFromEsiMissingSmFails) {
    auto dev = makeDevice();   // no Outputs/Inputs entries
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};
    EXPECT_FALSE(master_.configureProcessDataSyncManagersFromESI(
        esi, SlaveAddress(0)));
}

TEST_F(MasterSiiCoverageTest, ProcessDataSmsEsiEmptyFails) {
    ESIFile esi{std::vector<ESI::DeviceInfo>{}};
    EXPECT_FALSE(master_.configureProcessDataSyncManagersFromESI(
        esi, SlaveAddress(0)));
}

TEST_F(MasterSiiCoverageTest, ProcessDataSmsMissingSlaveFails) {
    auto dev = makeDevice();
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};
    EXPECT_FALSE(master_.configureProcessDataSyncManagersFromESI(
        esi, SlaveAddress(9)));
}

// ============================================================================
// logDiscoveredSlavesSummary
// ============================================================================

TEST_F(MasterSiiCoverageTest, LogDiscoveredSlavesSummary) {
    master_.logDiscoveredSlavesSummary("t");
}

// ============================================================================
// verifySlaveIdentity
// ============================================================================

TEST_F(MasterSiiCoverageTest, VerifyIdentityMatch) {
    EtherCAT::SII::SIIIdentity id{};
    ASSERT_TRUE(EtherCAT::SII::readSIIIdentity(master_, 0, id));
    Identity::SlaveIdentity expected;
    expected.vendor_id = id.vendor_id;
    expected.product_code = id.product_code;
    expected.revision_number = id.revision_number;
    expected.serial_number = id.serial_number;
    EXPECT_TRUE(master_.verifySlaveIdentity(0, expected, false, "t"));
}

TEST_F(MasterSiiCoverageTest, VerifyIdentityMismatch) {
    Identity::SlaveIdentity expected;
    expected.vendor_id = 0xDEADBEEF;
    expected.product_code = 0x12345678;
    expected.revision_number = 0x00010002;
    expected.serial_number = 0xAABBCCDD;
    EXPECT_FALSE(master_.verifySlaveIdentity(0, expected, false, "t"));
}

TEST_F(MasterSiiCoverageTest, VerifyIdentityNoFieldsAlwaysOk) {
    Identity::SlaveIdentity expected;   // all optional fields empty
    EXPECT_TRUE(master_.verifySlaveIdentity(0, expected, false, "t"));
}

TEST_F(MasterSiiCoverageTest, VerifyIdentityMissingSlaveFails) {
    Identity::SlaveIdentity expected;
    EXPECT_FALSE(master_.verifySlaveIdentity(9, expected, false, "t"));
}

// ============================================================================
// siiReadString / configureMailboxFromSii delegation
// ============================================================================

TEST_F(MasterSiiCoverageTest, SiiReadStringDelegates) {
    char buf[64] = {};
    // Returns whatever the emulator's SII strings contain — the call must
    // simply not crash.
    (void)master_.siiReadString(0, 0, buf, sizeof(buf));
}

TEST_F(MasterSiiCoverageTest, ConfigureMailboxFromSiiDelegates) {
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;
    (void)master_.configureMailboxFromSii(0, &wr_addr, &wr_len,
                                           &rd_addr, &rd_len, &proto);
}

}  // namespace
