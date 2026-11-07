/**
 * @file test_slavegroup_mailbox_cov.cpp
 * @brief Coverage for NonExistingSlave stub methods, SlaveGroup
 *        requestAlControl kick/broadcast paths, and the Mailbox::Utils
 *        diagnostic helpers (statusword edges + header dump over wire).
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/Mailbox.hpp"
#include "tether/ethercat/Mailbox/Utils.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/ethercat/SlaveGroup.hpp"
#include "logging/Logger.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

using namespace EtherCAT;

namespace {

class SlaveGroupMailboxTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto s0 = Emulator::createCiA402Drive(0x00000002, 0xCAFE, "sg-a");
        auto s1 = Emulator::createCiA402Drive(0x00000002, 0xCAFE, "sg-b");
        bus_.net.addSlave(std::move(s0));
        bus_.net.addSlave(std::move(s1));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 2u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
        master_.sdoManager(1).configureMailbox(0x1000, 128, 0x1080, 128);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Master master_;
};

// ---------------------------------------------------------------------------
// CiA402::setOperatingModeVerified — group write 0x6060 then verify 0x6061
// on every member over the emulated wire.
// ---------------------------------------------------------------------------

TEST_F(SlaveGroupMailboxTest, SetOperatingModeVerifiedWire) {
    SlaveGroup group(master_, {0, 1});

    EtherCAT::CoE::CoETransactionOptions opts{};
    opts.timeout_ms = 2000;
    // The emulated CiA402 drive mirrors 0x6060 into 0x6061.
    EXPECT_TRUE(CiA402::setOperatingModeVerified(
        group, CiA402::OperatingMode::ProfilePosition, opts,
        /*settle_ms=*/1, "sg-test"));

    // A mode the OD write rejects would surface as false; a display
    // readback mismatch likewise — verify the function returns the
    // aggregate AND result via a successful write+verify round first.
}

} // namespace

// ---------------------------------------------------------------------------
// NonExistingSlave — every stub logs CRITICAL and returns SlaveNotFound.
// master_.slave(out_of_range) returns the sentinel bound to this master.
// ---------------------------------------------------------------------------

TEST_F(SlaveGroupMailboxTest, NonExistingSlaveStubsAllFail) {
    auto& slave = master_.slave(42);

    EXPECT_EQ(slave.configureMailbox(Tether::Platform::LogLevel::Info),
              SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.configureMailbox(MailboxSyncManagerConfig{},
                                     MailboxSyncManagerConfig{}, 8),
              SlaveError::SlaveNotFound);
    ESIFile empty_esi(std::vector<ESI::DeviceInfo>{});
    EXPECT_EQ(slave.configureMailbox(empty_esi,
                                     Tether::Platform::LogLevel::Info),
              SlaveError::SlaveNotFound);
    slave.assumeMailboxAlreadyConfigured();
    slave.markNoMailbox();
    EXPECT_FALSE(slave.drainMailbox(10));

    EXPECT_EQ(slave.configurePDOSyncManagers(), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.configurePDOSyncManagers(0x1800, 32, 0x27,
                                             0x1C00, 32, 0x27),
              SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.configurePDOSyncManagers(empty_esi),
              SlaveError::SlaveNotFound);
    slave.assumePDOAlreadyConfigured();

    SIIPDOConfig pdo_cfg{};
    EXPECT_EQ(slave.registerPDOsFromSII(pdo_cfg), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.registerPDOsFromESI(empty_esi, pdo_cfg),
              SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.assignPDOs(pdo_cfg), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.registerFixedPDOs(pdo_cfg), SlaveError::SlaveNotFound);

    ObjectDictionary::ObjectDictionaryEntry od{
        .index = 0x6040, .subindex = 0, .name = "controlword",
        .data_type = ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
        .default_value = 0};
    CustomPDOMappingEntry entry(&od, 2);
    EXPECT_EQ(slave.configureCustomRxPDO(0x1600, {entry}),
              SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.configureCustomTxPDO(0x1A00, {entry}),
              SlaveError::SlaveNotFound);
    const std::array<CustomPDOMappingEntry, 1> span_entries{entry};
    EXPECT_EQ(slave.configureCustomTxPDO(0x1A00,
                                         std::span<const CustomPDOMappingEntry>(
                                             span_entries),
                                         PDO::PDODirection::TxPDO),
              SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.applyCustomPDOs(), SlaveError::SlaveNotFound);
    slave.clearCustomPDOs();
    EXPECT_EQ(slave.registerExistingRxPDO(0x1600), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.registerExistingTxPDO(0x1A00), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.configureMultiPDOs(MultiPDOAssignment{}),
              SlaveError::SlaveNotFound);

    EXPECT_EQ(slave.transitionTo(SlaveState::PRE_OP), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.transitionToInit(), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.transitionToPreOp(), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.transitionToSafeOp(), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.transitionToOp(), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.transitionToBoot(), SlaveError::SlaveNotFound);

    SlaveState st = SlaveState::INIT;
    EXPECT_EQ(slave.readState(st), SlaveError::SlaveNotFound);
    uint16_t al_code = 0;
    EXPECT_EQ(slave.readALStatusCode(al_code), SlaveError::SlaveNotFound);

    EXPECT_EQ(slave.configureWatchdogs(100, 100), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.disableWatchdogs(), SlaveError::SlaveNotFound);
    uint8_t wd0 = 0, wd1 = 0, wd2 = 0;
    EXPECT_EQ(slave.readWatchdogStatus(wd0, wd1, wd2),
              SlaveError::SlaveNotFound);

    uint8_t buf8[8] = {};
    size_t len = sizeof(buf8);
    EXPECT_EQ(slave.sdoRead(0x1000, 0, buf8, len), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.sdoWrite(0x1000, 0, buf8, 1), SlaveError::SlaveNotFound);
    uint8_t u8 = 0;
    uint16_t u16 = 0;
    uint32_t u32 = 0;
    EXPECT_EQ(slave.sdoReadU8(0x1000, 0, u8), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.sdoReadU16(0x1000, 0, u16), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.sdoReadU32(0x1000, 0, u32), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.sdoWriteU8(0x1000, 0, 1), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.sdoWriteU16(0x1000, 0, 1), SlaveError::SlaveNotFound);
    EXPECT_EQ(slave.sdoWriteU32(0x1000, 0, 1), SlaveError::SlaveNotFound);

#if TETHER_ENABLE_SII
    SII::SIIData sii{};
    EXPECT_EQ(slave.readSII(sii), SlaveError::SlaveNotFound);
    slave.logSIISummary("test");
#endif

    // sm() returns an accessor bound to the sentinel — call it for coverage.
    auto accessor = slave.sm(0);
    (void)accessor;
}

// ---------------------------------------------------------------------------
// SlaveGroup::requestAlControl — kick mode (per-slave APWR+APRD piggyback)
// and broadcast mode (single BWR via ::all()).
// ---------------------------------------------------------------------------

TEST_F(SlaveGroupMailboxTest, RequestStateKickPath) {
    master_.setAlControlStatusKick(true);
    SlaveGroup group(master_, {0, 1});
    const uint16_t acked =
        group.requestState(SlaveState::PRE_OP, false, 500);
    EXPECT_EQ(acked, 2u);
}

TEST_F(SlaveGroupMailboxTest, RequestStateBroadcastPath) {
    auto all = SlaveGroup::all(master_);
    const uint16_t acked =
        all.requestState(SlaveState::PRE_OP, false, 500);
    EXPECT_EQ(acked, 2u);
}

TEST_F(SlaveGroupMailboxTest, RequestStateNoKickSequentialPath) {
    master_.setAlControlStatusKick(false);
    SlaveGroup group(master_, {0, 1});
    const uint16_t acked =
        group.requestState(SlaveState::PRE_OP, false, 500);
    EXPECT_EQ(acked, 2u);
}

TEST_F(SlaveGroupMailboxTest, RequestStateEmptyGroup) {
    SlaveGroup group(master_, std::span<const uint16_t>{});
    EXPECT_EQ(group.requestState(SlaveState::PRE_OP, false, 100), 0u);
}

// ---------------------------------------------------------------------------
// Mailbox::Utils::logStatuswordDiagnostics — edge-triggered warning state,
// following-error (bit13) and internal-limit (bit11) modulo branches.
// ---------------------------------------------------------------------------

TEST(MailboxUtilsTest, StatuswordWarningEdges) {
    bool active = false;
    uint64_t first = 0;
    // Rising edge: warning bit set, state was clear.
    Mailbox::Utils::logStatuswordDiagnostics(0x0080, active, first, 42);
    EXPECT_TRUE(active);
    EXPECT_EQ(first, 42u);
    // Held: still active, first_cycle unchanged.
    Mailbox::Utils::logStatuswordDiagnostics(0x0080, active, first, 43);
    EXPECT_TRUE(active);
    EXPECT_EQ(first, 42u);
    // Falling edge: cleared.
    Mailbox::Utils::logStatuswordDiagnostics(0x0000, active, first, 100);
    EXPECT_FALSE(active);
    // Steady clear: no transition.
    Mailbox::Utils::logStatuswordDiagnostics(0x0000, active, first, 101);
    EXPECT_FALSE(active);
}

TEST(MailboxUtilsTest, StatuswordErrorBits) {
    bool active = false;
    uint64_t first = 0;
    // Following error (bit13): logs at cycle % 500 == 0.
    Mailbox::Utils::logStatuswordDiagnostics(0x2000, active, first, 500);
    Mailbox::Utils::logStatuswordDiagnostics(0x2000, active, first, 501);
    // Internal limit (bit11): logs at cycle % 1000 == 0.
    Mailbox::Utils::logStatuswordDiagnostics(0x0800, active, first, 1000);
    Mailbox::Utils::logStatuswordDiagnostics(0x0800, active, first, 1001);
    // Both bits + warning together.
    Mailbox::Utils::logStatuswordDiagnostics(0x2880, active, first, 0);
    EXPECT_TRUE(active);
}

// ---------------------------------------------------------------------------
// Mailbox::Utils::dumpHeaderAndStatus / dumpSlaveSyncAndMailboxInfo —
// drive the mailbox-configured branch over the emulated wire.
// ---------------------------------------------------------------------------

TEST_F(SlaveGroupMailboxTest, DumpHeaderAndStatusConfiguredMailbox) {
    // Fixture configured SM0/SM1 mailbox — mbx_cfg branch + both header
    // read paths execute over the wire.
    Mailbox::Utils::dumpHeaderAndStatus(master_, 0, "test-mbx");
    Mailbox::Utils::dumpHeaderAndStatus(master_, 1, "test-mbx");
}

TEST_F(SlaveGroupMailboxTest, DumpSlaveSyncAndMailboxInfo) {
    CiA402Drive drive(master_, 0);
    Mailbox::Utils::dumpSlaveSyncAndMailboxInfo(drive, "test-drv");
}
