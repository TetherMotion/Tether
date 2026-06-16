#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Headers & APIs under test
#include "tether/ethercat/Reset.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/DCConsistency.hpp"
#include "tether/ethercat/ConditionalPacketRouter.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Retry.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SDOManager.hpp"

// Raw module APIs
#include "tether/ethercat/Raw.hpp"

using namespace EtherCAT;

// Minimal stub IPDOTransport for local PDOManager instances
namespace {
class StubPDOTransport : public IPDOTransport {
public:
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t, unsigned int) override { return false; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t, unsigned int) override { return false; }
    bool sendSingleDatagram(Command, uint8_t, uint16_t, uint16_t, const void*, uint16_t, bool) override { return false; }
    bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram&) override { return false; }
    uint8_t allocIdx() override { return 0; }
    uint16_t adpForSlaveIndex(uint16_t idx) override { return static_cast<uint16_t>(0u - idx); }
};
} // namespace

TEST(Reset_Header, NamesAndDescriptions) {
    for (int i = 0; i <= static_cast<int>(ResetLevel::HardwareReset); ++i) {
        auto name = getResetLevelName(static_cast<ResetLevel>(i));
        auto desc = getResetLevelDescription(static_cast<ResetLevel>(i));
        EXPECT_NE(name, nullptr);
        EXPECT_NE(desc, nullptr);
    }
}

TEST(PDO_Header, SyncManagerConfigAndConsts) {
    EXPECT_GT(PDO::kMaxPDOEntries, 0u);
    EXPECT_GT(PDO::kMaxPDOSize, 0u);
    EXPECT_GT(PDO::kMaxPDOSlaves, 0u);

    auto mwr = PDO::SyncManagerConfig::mailbox_write(0x1000, 64);
    EXPECT_TRUE(mwr.enable);
    EXPECT_EQ(mwr.type, PDO::SyncManagerType::MailboxWrite);

    auto mrd = PDO::SyncManagerConfig::mailbox_read(0x1002, 32);
    EXPECT_TRUE(mrd.enable);
    EXPECT_EQ(mrd.type, PDO::SyncManagerType::MailboxRead);

    auto pout = PDO::SyncManagerConfig::process_output(0x2000, 16);
    EXPECT_TRUE(pout.enable);
    EXPECT_EQ(pout.type, PDO::SyncManagerType::ProcessOutput);
}

TEST(DCConsistency_Header, BasicHelpers) {
    char buf[64] = {0};
    size_t n = DC::dc_format_time(0, buf, sizeof(buf));
    EXPECT_GT(n, 0u);

    DC::DCConsistencyReport r;
    r.check_count = 0;
    r.passed_count = 0;
    r.failed_count = 0;
    EXPECT_TRUE(r.all_passed());
}

TEST(ConditionalPacketRouter_Header, PacketFilterFactories) {
    auto f1 = PacketFilter::byIndex(5);
    EXPECT_TRUE(f1.match_idx);
    EXPECT_EQ(f1.idx, 5u);

    auto f2 = PacketFilter::aprd(0, 0x1234, 7);
    EXPECT_TRUE(f2.match_command);
    EXPECT_TRUE(f2.match_ado);
    EXPECT_EQ(f2.ado, 0x1234u);

    ConditionalPacketRouter router;
    auto s = router.getStats();
    EXPECT_EQ(s.packets_routed, 0u);
    EXPECT_EQ(router.waiterCount(), 0u);
}

TEST(DC_Header, StateName) {
    EXPECT_STREQ(DC::dc_state_name(DC::DCState::Disabled), "Disabled");
    EXPECT_STREQ(DC::dc_state_name(DC::DCState::Error), "Error");
}

// Lightweight tests against PDO APIs using a local PDOManager instance
TEST(PDO_Runtime, StatsAndModes) {
    StubPDOTransport transport;
    PDOManager mgr(transport);
    mgr.init();

    auto s = mgr.getStats();
    (void)s; // ensure callable

    mgr.setSeparateMode(true);
    EXPECT_TRUE(mgr.getSeparateMode());
    mgr.setSeparateMode(false);
    EXPECT_FALSE(mgr.getSeparateMode());

    mgr.setPhysicalMode(true);
    EXPECT_TRUE(mgr.getPhysicalMode());
    mgr.setPhysicalMode(false);
    EXPECT_FALSE(mgr.getPhysicalMode());

    // Configure 0 slaves should return 0 successes
    EXPECT_EQ(mgr.configureAllSlaveSMs(0), 0u);

    mgr.deinit();
}

TEST(PDO_Stubs, CallGetMapping) {
    // Use a local PDOManager instance to access mapping
    StubPDOTransport transport;
    PDOManager mgr(transport);
    mgr.init();

    auto& m = mgr.mapping();
    EXPECT_GE(m.total_rxpdo_bytes(), 0u);

    mgr.deinit();
}

TEST(Raw_EEPROM_SII, ReadStringFailsSafely) {
    EtherCAT::Master master;
    char out[8] = {0};
    // With no device, expect false
    EXPECT_FALSE(master.siiReadString(0xFFFF, 1, out, sizeof(out)));
}

TEST(SDO_Async, InitDeinit) {
    // Use a local Master which owns an SDOManager
    EtherCAT::Master master;
    auto& sdo_mgr = master.sdoManager();
    bool started = sdo_mgr.init();
    // It may return false in host environment; ensure deinit is safe
    sdo_mgr.deinit();
    (void)started;
    EXPECT_NO_THROW(sdo_mgr.deinit());
}

// Basic include sanity tests for other headers
TEST(Header_Sanity, BasicCompileChecks) {
    // Basic check: compile-time existence of a few types/constants in namespaces
    EXPECT_NO_THROW((void)PDO::PDOMapping{});
    EXPECT_NO_THROW((void)DC::DCConfig::defaults());
}
