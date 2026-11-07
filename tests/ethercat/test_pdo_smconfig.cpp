/**
 * @file test_pdo_smconfig.cpp
 * @brief Coverage for PDOManager sync-manager configuration
 *        (src/ethercat/raw/PDOManager_smconfig.cpp): disable/block writes,
 *        the batched SM-activation path (pre-registered waiters + one
 *        multi-datagram frame), all three fallback triggers (no pre-reg
 *        support, multi-send failure, waiter timeout), sequential-fallback
 *        activation failures, and the per-slave debug-flag log switches.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <set>
#include <vector>

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/IPDOTransport.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;

namespace {

// SM register window: 0x0800 + 8*sm, +6 = activate byte.
constexpr uint16_t kSmBase = 0x0800;
constexpr uint16_t kSmActivateOff = 0x06;

uint16_t activateAddr(uint8_t sm) {
    return static_cast<uint16_t>(kSmBase + sm * 8 + kSmActivateOff);
}

/// PDO transport with call recording, write-failure scripting and
/// optional response-waiter pre-registration support.
class SmConfigTransport : public IPDOTransport {
public:
    struct Write { uint16_t adp, ado, len; uint8_t first_byte; };
    std::vector<Write> writes;
    std::set<size_t>  fail_write_calls;   // 0-based call indices to fail
    size_t write_calls = 0;

    bool support_prereg = false;
    size_t multi_send_ret = 0;            // what sendMultiDatagram returns
    bool   wait_ok = true;
    size_t prereg_calls = 0;
    size_t cancel_calls = 0;
    size_t multi_specs = 0;               // count seen by sendMultiDatagram
    uint8_t next_idx = 0x40;

    bool writeRegister(uint16_t adp, uint16_t ado, const void* data,
                       uint16_t len, unsigned int) override {
        const size_t call = write_calls++;
        uint8_t first = 0;
        if (data && len) first = *static_cast<const uint8_t*>(data);
        writes.push_back({adp, ado, len, first});
        return !fail_write_calls.count(call);
    }
    bool readRegister(uint16_t, uint16_t, void* data, uint16_t len,
                      unsigned int) override {
        if (data && len) std::memset(data, 0, len);
        return true;
    }
    bool sendSingleDatagram(Command, uint8_t, uint16_t, uint16_t,
                            const void*, uint16_t, bool) override { return true; }
    size_t sendMultiDatagram(const MultiDatagramSpec* specs, size_t count) override {
        multi_specs = count;
        (void)specs;
        return multi_send_ret;
    }
    bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram& out) override {
        out.wkc = 1;
        return true;
    }
    size_t preRegisterResponseWaiter(uint8_t, uint8_t*, size_t) override {
        if (!support_prereg) return kPreRegInvalid;
        return prereg_calls++;
    }
    bool waitForPreRegistered(size_t, unsigned int, RxDatagram& out) override {
        out.wkc = 1;
        return wait_ok;
    }
    void cancelPreRegistered(size_t) override { ++cancel_calls; }
    uint8_t allocIdx() override { return next_idx++; }
    uint16_t adpForSlaveIndex(uint16_t i) override {
        return static_cast<uint16_t>(0x1000 + i);
    }
};

void seedAllFourSMs(PDOManager& mgr) {
    auto* cfg = mgr.slaveConfigs();
    cfg[0].sm[0] = PDO::SyncManagerConfig::mailbox_write(0x1000, 128);
    cfg[0].sm[1] = PDO::SyncManagerConfig::mailbox_read(0x1080, 128);
    cfg[0].sm[2] = PDO::SyncManagerConfig::process_output(0x1800, 8);
    cfg[0].sm[3] = PDO::SyncManagerConfig::process_input(0x1C00, 8);
}

size_t activateWriteCount(const SmConfigTransport& t) {
    // Sequential fallback activate writes are 1-byte writes to base+6 whose
    // data byte is the enable flag — identical address to the disable write.
    // Distinguish by position: each SM is touched twice before activation
    // (disable then block), so a *third* 1-byte write to the activate
    // offset for the same SM is a sequential activate.
    size_t n = 0;
    for (const auto& w : t.writes)
        if (w.len == 1 && (w.ado & 7u) == kSmActivateOff) ++n;
    return n;   // includes the 4 disable writes; sequential adds 4 more
}

// ============================================================================
// Basic paths
// ============================================================================

TEST(PdoSmConfig, InvalidSlaveIndex) {
    SmConfigTransport t;
    PDOManager mgr(t);
    EXPECT_FALSE(mgr.configureSlavesSMs(static_cast<uint16_t>(PDO::kMaxPDOSlaves)));
    EXPECT_FALSE(mgr.configureSlavesSMs(0xFFFF));
}

TEST(PdoSmConfig, NoSMsConfiguredIsNoOp) {
    SmConfigTransport t;
    PDOManager mgr(t);
    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_TRUE(t.writes.empty());
}

TEST(PdoSmConfig, DisableWriteFailureContinues) {
    SmConfigTransport t;
    t.support_prereg = true;
    t.multi_send_ret = 4;
    PDOManager mgr(t);
    seedAllFourSMs(mgr);
    // Call 0 = SM0 disable write — fails, logged, configuration proceeds.
    t.fail_write_calls.insert(0);
    EXPECT_TRUE(mgr.configureSlavesSMs(0));
}

TEST(PdoSmConfig, BlockWriteFailureAborts) {
    SmConfigTransport t;
    PDOManager mgr(t);
    seedAllFourSMs(mgr);
    // Call 1 = SM0 register-block write — failure returns false.
    t.fail_write_calls.insert(1);
    EXPECT_FALSE(mgr.configureSlavesSMs(0));
}

// ============================================================================
// Batched activation
// ============================================================================

TEST(PdoSmConfig, BatchActivateHappyPath) {
    SmConfigTransport t;
    t.support_prereg = true;
    t.multi_send_ret = 4;
    PDOManager mgr(t);
    seedAllFourSMs(mgr);

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_EQ(t.prereg_calls, 4u);
    EXPECT_EQ(t.multi_specs, 4u);
    EXPECT_EQ(t.cancel_calls, 0u);
    // 4 disables + 4 blocks, zero sequential activate writes.
    EXPECT_EQ(t.writes.size(), 8u);
}

TEST(PdoSmConfig, PreRegUnsupportedFallsBack) {
    SmConfigTransport t;   // support_prereg = false
    PDOManager mgr(t);
    seedAllFourSMs(mgr);

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_EQ(t.multi_specs, 0u);              // multi never attempted
    EXPECT_EQ(t.writes.size(), 12u);           // 8 + 4 sequential activates
    EXPECT_EQ(activateWriteCount(t), 8u);      // 4 disables + 4 activates
}

TEST(PdoSmConfig, MultiSendFailureFallsBack) {
    SmConfigTransport t;
    t.support_prereg = true;
    t.multi_send_ret = 0;                      // frame send failed
    PDOManager mgr(t);
    seedAllFourSMs(mgr);

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_EQ(t.cancel_calls, 4u);             // all slots released
    EXPECT_EQ(activateWriteCount(t), 8u);      // sequential fallback ran
}

TEST(PdoSmConfig, WaiterTimeoutFallsBack) {
    SmConfigTransport t;
    t.support_prereg = true;
    t.multi_send_ret = 4;
    t.wait_ok = false;                         // responses never arrive
    PDOManager mgr(t);
    seedAllFourSMs(mgr);

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_EQ(t.cancel_calls, 4u);
    EXPECT_EQ(activateWriteCount(t), 8u);
}

TEST(PdoSmConfig, FallbackActivateWriteFails) {
    SmConfigTransport t;   // no pre-reg -> sequential path
    PDOManager mgr(t);
    seedAllFourSMs(mgr);
    // Calls 8..11 are the sequential activate writes — fail the first.
    t.fail_write_calls.insert(8);
    EXPECT_FALSE(mgr.configureSlavesSMs(0));
}

// ============================================================================
// configureAllSlaveSMs
// ============================================================================

TEST(PdoSmConfig, ConfigureAllCountsSuccesses) {
    SmConfigTransport t;
    PDOManager mgr(t);
    seedAllFourSMs(mgr);
    // Slave 1's SM0 block write fails -> it doesn't count.
    // Slave 0 uses calls 0-7 (no pre-reg -> 12 calls actually).
    // Simpler: give slave 0 SMs, leave slave 1 unused, slave 2 fails.
    auto* cfg = mgr.slaveConfigs();
    cfg[1] = PDO::SlaveConfig{};               // all Unused -> trivially ok
    cfg[2].sm[0] = PDO::SyncManagerConfig::mailbox_write(0x1000, 64);
    // Ordering: slave0 = calls 0..11, slave1 = none, slave2 = 12,13,14
    t.fail_write_calls.insert(13);             // slave2 SM0 block write

    EXPECT_EQ(mgr.configureAllSlaveSMs(3), 2u);
}

TEST(PdoSmConfig, ConfigureAllCapsAtMaxSlaves) {
    SmConfigTransport t;
    PDOManager mgr(t);
    const uint16_t over = static_cast<uint16_t>(PDO::kMaxPDOSlaves + 4);
    // All slave configs unused -> every in-range call succeeds trivially.
    EXPECT_EQ(mgr.configureAllSlaveSMs(over), PDO::kMaxPDOSlaves);
}

// ============================================================================
// Debug-flag log switches
// ============================================================================

TEST(PdoSmConfig, AllDebugFlagsExerciseLogPaths) {
    SmConfigTransport t;
    t.support_prereg = true;
    t.multi_send_ret = 4;
    PDOManager mgr(t);
    seedAllFourSMs(mgr);

    // Add IRQ + watchdog bits to SM2 control so the IRQ log lines run.
    auto* cfg = mgr.slaveConfigs();
    uint8_t ctrl = std::bit_cast<uint8_t>(cfg[0].sm[2].control);
    ctrl |= PDO::SM_CTRL_IRQ_ECAT | PDO::SM_CTRL_IRQ_PDI | PDO::SM_CTRL_WATCHDOG;
    cfg[0].sm[2].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(ctrl);
    cfg[0].vendor_id = 0x00000002;
    cfg[0].product_code = 0xDEADBEEF;

    EtherCATMasterDebugFlags flags;
    flags.rxPDO = true;
    flags.txPDO = true;
    flags.pdoSm = true;
    flags.mailboxConfiguration = true;
    mgr.setDebugFlags(&flags);

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_EQ(t.multi_specs, 4u);
}

TEST(PdoSmConfig, UnusedSmSlotsLoggedWhenDebugOn) {
    SmConfigTransport t;
    t.support_prereg = true;
    t.multi_send_ret = 1;
    PDOManager mgr(t);
    // Only SM0 configured — SM1..3 hit the "SM{}: unused" log line.
    auto* cfg = mgr.slaveConfigs();
    cfg[0].sm[0] = PDO::SyncManagerConfig::process_output(0x1800, 8);

    EtherCATMasterDebugFlags flags;
    flags.rxPDO = true;
    mgr.setDebugFlags(&flags);

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_EQ(t.multi_specs, 1u);
}

TEST(PdoSmConfig, MailboxCfgDebugOnly) {
    SmConfigTransport t;
    t.support_prereg = true;
    t.multi_send_ret = 4;
    PDOManager mgr(t);
    // Only mailbox SMs -> is_mbx branch selects mailboxCfgDebug.
    auto* cfg = mgr.slaveConfigs();
    cfg[0].sm[0] = PDO::SyncManagerConfig::mailbox_write(0x1000, 128);
    cfg[0].sm[1] = PDO::SyncManagerConfig::mailbox_read(0x1080, 128);

    EtherCATMasterDebugFlags flags;
    flags.mailboxConfiguration = true;
    mgr.setDebugFlags(&flags);

    EXPECT_TRUE(mgr.configureSlavesSMs(0));
    EXPECT_EQ(t.multi_specs, 2u);
}

}  // namespace
