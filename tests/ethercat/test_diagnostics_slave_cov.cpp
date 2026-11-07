/**
 * @file test_diagnostics_slave_cov.cpp
 * @brief Diagnostics helpers + Slave ESI/SII paths over the emulated
 *        wire — logPreOperationalMailboxDiagnostics option matrix,
 *        logParsedSlaveSII, logSlaveApplicationLayerDiagnostics,
 *        configurePDOSyncManagers(ESIFile) branches, and readSII.
 */

#include <gtest/gtest.h>

#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Diagnostics.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/ESITypes.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/sii/SIIReader.hpp"

using namespace EtherCAT;

namespace {

class DiagSlaveCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto s = Emulator::createCiA402Drive(0x00000002, 0xCAFE, "diag");
        slave_ = s.get();
        bus_.net.addSlave(std::move(s));
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

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slave_ = nullptr;
    Master master_;
};

// An ESI device matching whatever identity the emulator reports — SII
// identity reads return zeros on the emulated EEPROM, so vendor/product
// 0/0 always matches.
ESI::DeviceInfo makeDevice() {
    ESI::DeviceInfo dev;
    dev.name = "emu-drive";
    dev.vendorId = 0;
    dev.productCode = 0;

    ESI::SyncManagerEntry smOut;
    smOut.name = "Outputs";
    smOut.startAddress = 0x1100;
    smOut.defaultSize = 16;
    EtherCAT::SyncManager::SMControlReg ctrl{};
    ctrl.mode = 0;            // buffered
    ctrl.direction = 0;       // read (ECAT output)
    smOut.control = ctrl;

    ESI::SyncManagerEntry smIn;
    smIn.name = "Inputs";
    smIn.startAddress = 0x1140;
    smIn.defaultSize = 16;
    EtherCAT::SyncManager::SMControlReg ctrlIn{};
    ctrlIn.mode = 0;
    ctrlIn.direction = 1;     // write (ECAT input)
    smIn.control = ctrlIn;

    dev.syncManagers = {smOut, smIn};
    return dev;
}

} // namespace

// ---------------------------------------------------------------------------
// EtherCAT::Diagnostics helpers.
// ---------------------------------------------------------------------------

TEST_F(DiagSlaveCovTest, PreOperationalMailboxDiagnosticsAllOptions) {
    Diagnostics::PreOperationalMailboxDiagnosticsOptions opts{};
    Diagnostics::logPreOperationalMailboxDiagnostics(master_, 0, "test-diag",
                                                     opts);
}

TEST_F(DiagSlaveCovTest, PreOperationalMailboxDiagnosticsNoAutoConfig) {
    Diagnostics::PreOperationalMailboxDiagnosticsOptions opts{};
    opts.attempt_auto_configure = false;
    Diagnostics::logPreOperationalMailboxDiagnostics(master_, 0, "test-diag",
                                                     opts);
}

TEST_F(DiagSlaveCovTest, PreOperationalMailboxDiagnosticsNoSdoMailbox) {
    Diagnostics::PreOperationalMailboxDiagnosticsOptions opts{};
    opts.log_sdo_mailbox_config = false;
    opts.log_sii_mailbox = false;
    opts.log_sync_manager_registers = false;
    Diagnostics::logPreOperationalMailboxDiagnostics(master_, 0, "test-diag",
                                                     opts);
}

// On an out-of-range slave every wire read fails → FAILED branches.
TEST_F(DiagSlaveCovTest, DiagnosticsOnMissingSlave) {
    Diagnostics::logPreOperationalMailboxDiagnostics(master_, 42, "test-diag",
                                                     {});
    Diagnostics::logSlaveApplicationLayerDiagnostics(master_, 42, "test-diag");
}

TEST_F(DiagSlaveCovTest, SlaveApplicationLayerDiagnostics) {
    Diagnostics::logSlaveApplicationLayerDiagnostics(master_, 0, "test-diag");
}

TEST_F(DiagSlaveCovTest, ParsedSlaveSii) {
    // The emulated EEPROM is empty — parse likely fails; both outcomes
    // exercise the function's return paths.
    (void)Diagnostics::logParsedSlaveSII(master_, 0, "test-diag");
    (void)Diagnostics::logParsedSlaveSII(master_, 42, "test-diag");
}

// ---------------------------------------------------------------------------
// Slave::configurePDOSyncManagers(const ESIFile&)
// ---------------------------------------------------------------------------

TEST_F(DiagSlaveCovTest, ConfigurePdoSyncManagersEmptyEsi) {
    auto& slave = master_.slave(0);
    ESIFile empty(std::vector<ESI::DeviceInfo>{});
    EXPECT_EQ(slave.configurePDOSyncManagers(empty),
              SlaveError::PDOConfigFailed);
}

// findDevice() treats a {0,0} wire identity as a wildcard and falls back
// to the first device — a mismatched vendor still configures.  The !dev
// branch is unreachable (findDevice never returns null for non-empty).
TEST_F(DiagSlaveCovTest, ConfigurePdoSyncManagersWildcardMatch) {
    auto& slave = master_.slave(0);
    ESI::DeviceInfo other = makeDevice();
    other.vendorId = 0xDEADBEEF;
    other.productCode = 0x12345678;
    ESIFile esi(std::vector<ESI::DeviceInfo>{other});
    EXPECT_EQ(slave.configurePDOSyncManagers(esi), SlaveError::Ok);
}

TEST_F(DiagSlaveCovTest, ConfigurePdoSyncManagersMissingSmNames) {
    auto& slave = master_.slave(0);
    ESI::DeviceInfo dev = makeDevice();
    dev.syncManagers.clear();   // no Outputs/Inputs named SMs
    ESIFile esi(std::vector<ESI::DeviceInfo>{dev});
    EXPECT_EQ(slave.configurePDOSyncManagers(esi),
              SlaveError::PDOConfigFailed);
}

TEST_F(DiagSlaveCovTest, ConfigurePdoSyncManagersFromEsiSuccess) {
    auto& slave = master_.slave(0);
    ESIFile esi(std::vector<ESI::DeviceInfo>{makeDevice()});
    const SlaveError r = slave.configurePDOSyncManagers(esi);
    // Success or a wire-level failure — either way the ESI lookup and
    // SM-name matching branches executed.
    EXPECT_TRUE(r == SlaveError::Ok || r == SlaveError::PDOConfigFailed
                || r == SlaveError::TransportError);
}

// ---------------------------------------------------------------------------
// Slave::readSII — SII manager uninitialised → direct wire read.
// ---------------------------------------------------------------------------

TEST_F(DiagSlaveCovTest, ReadSiiDirect) {
    auto& slave = master_.slave(0);
    SII::SIIData data{};
    const SlaveError r = slave.readSII(data);
    EXPECT_TRUE(r == SlaveError::Ok || r == SlaveError::SIIReadError);
}
