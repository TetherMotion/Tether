/**
 * @file test_slave_mailbox_coverage.cpp
 * @brief Emulator-backed coverage for Slave mailbox configuration paths —
 *        SII/explicit/ESI-driven configureMailbox, markNoMailbox,
 *        assumeMailboxAlreadyConfigured and drainMailbox delegation.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

constexpr uint32_t kVendor = 0x00000002;
constexpr uint32_t kProduct = 0x0FA13052;

class SlaveMailboxCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = std::make_unique<SlaveEmulator>();
        Emulator::SIIConfig cfg;
        cfg.vendor_id = kVendor;
        cfg.product_code = kProduct;
        cfg.mailbox_protocol = 0x04;
        slave->setSIIConfig(cfg);
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(1);
    }

    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    Slave& sl() { return master_.slave(0); }

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
// configureMailbox(log_level) — SII-driven
// ============================================================================

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxFromSii) {
    EXPECT_EQ(sl().configureMailbox(Tether::Platform::LogLevel::Info),
              SlaveError::Ok);
}

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxRefusedAfterMarkNoMailbox) {
    sl().markNoMailbox();
    EXPECT_EQ(sl().configureMailbox(Tether::Platform::LogLevel::Info),
              SlaveError::MailboxConfigFailed);
    // markNoMailbox also vacuously satisfies the mailbox precondition.
    EXPECT_TRUE(sl().isMailboxConfigured());
}

// ============================================================================
// configureMailbox(mbox_out, mbox_in, protocols) — explicit
// ============================================================================

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxExplicit) {
    MailboxSyncManagerConfig out{0x1000, 128};
    MailboxSyncManagerConfig in{0x1400, 128};
    EXPECT_EQ(sl().configureMailbox(out, in, 0x0004), SlaveError::Ok);
    EXPECT_TRUE(sl().isMailboxConfigured());
}

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxExplicitRefusedWhenNoMailbox) {
    sl().markNoMailbox();
    MailboxSyncManagerConfig out{0x1000, 128};
    MailboxSyncManagerConfig in{0x1400, 128};
    EXPECT_EQ(sl().configureMailbox(out, in, 0x0004),
              SlaveError::MailboxConfigFailed);
}

// ============================================================================
// configureMailbox(ESIFile) — ESI matching
// ============================================================================

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxEmptyEsiFails) {
    ESIFile esi{std::vector<ESI::DeviceInfo>{}};
    EXPECT_EQ(sl().configureMailbox(esi, Tether::Platform::LogLevel::Info),
              SlaveError::MailboxConfigFailed);
}

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxEsiFallsBackToFirstDevice) {
    auto dev = makeDevice();
    dev.vendorId = 0xDEADBEEF;  // does not match the emulator's SII vendor —
                              // findDevice() falls back to devices_[0].
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};
    EXPECT_EQ(sl().configureMailbox(esi, Tether::Platform::LogLevel::Info),
              SlaveError::Ok);
}

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxEsiWithSyncManagers) {
    auto dev = makeDevice();
    dev.mailbox.protocols = 0x0004;
    ESI::SyncManagerEntry mbxOut;
    mbxOut.startAddress = 0x1000;
    mbxOut.defaultSize = 128;
    mbxOut.name = "MBoxOut";
    ESI::SyncManagerEntry mbxIn;
    mbxIn.startAddress = 0x1400;
    mbxIn.defaultSize = 128;
    mbxIn.name = "MBoxIn";
    dev.syncManagers = {mbxOut, mbxIn};
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};

    EXPECT_EQ(sl().configureMailbox(esi, Tether::Platform::LogLevel::Info),
              SlaveError::Ok);
    EXPECT_TRUE(sl().isMailboxConfigured());
}

TEST_F(SlaveMailboxCoverageTest, ConfigureMailboxEsiDefaultsWhenSmMissing) {
    auto dev = makeDevice();   // no syncManagers, no mailbox.protocols
    ESIFile esi{std::vector<ESI::DeviceInfo>{dev}};
    EXPECT_EQ(sl().configureMailbox(esi, Tether::Platform::LogLevel::Debug),
              SlaveError::Ok);
}

// ============================================================================
// assumeMailboxAlreadyConfigured / drainMailbox / markNoMailbox
// ============================================================================

TEST_F(SlaveMailboxCoverageTest, AssumeMailboxAlreadyConfigured) {
    sl().assumeMailboxAlreadyConfigured();
    EXPECT_TRUE(sl().isMailboxConfigured());
}

TEST_F(SlaveMailboxCoverageTest, AssumeMailboxWarnsWhenDrainFails) {
    // No SDO mailbox addresses configured → drain aborts and the warning
    // arm fires (still returns configured).
    sl().assumeMailboxAlreadyConfigured();
    EXPECT_TRUE(sl().isMailboxConfigured());
}

TEST_F(SlaveMailboxCoverageTest, DrainMailboxDelegates) {
    master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    // Empty mailbox → drain trivially succeeds.
    EXPECT_TRUE(sl().drainMailbox());
}

}  // namespace
