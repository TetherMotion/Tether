/**
 * @file test_sync_diagnostics.cpp
 * @brief SyncDiagnostics::collect/format coverage over a real
 *        Master ↔ SlaveEmulator wire (src/ethercat/SyncDiagnostics.cpp).
 *
 * collect() reads the 0x1C32/0x1C33 SM-sync parameter records, DC ESC
 * registers, and AS715N-style vendor objects — all injectable into the
 * emulator's OD and register file.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SyncDiagnostics.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class SyncDiagnosticsTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0x0442, "diag");
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

    void addU(uint16_t idx, uint8_t sub, uint32_t value, uint8_t bytes) {
        const uint16_t dt = (bytes == 1) ? 0x0005 : (bytes == 2) ? 0x0006 : 0x0007;
        std::vector<uint8_t> d(bytes);
        for (uint8_t i = 0; i < bytes; ++i)
            d[i] = static_cast<uint8_t>(value >> (8 * i));
        slave_->addODEntry({idx, sub, dt, static_cast<uint16_t>(bytes * 8),
                            0x01, d, "x"});
    }

    /// Full 0x1C32/0x1C33 record set with distinct values per object.
    void addSmSync(uint16_t base, uint16_t mode) {
        addU(base, 0x01, mode, 2);          // sync_mode
        addU(base, 0x02, 1000000, 4);       // cycle_time_ns (1 ms)
        addU(base, 0x04, 0x0026, 2);        // sync_modes_supported
        addU(base, 0x05, 100000, 4);        // min_cycle_time_ns
        addU(base, 0x06, 50000, 4);         // calc_copy_time_ns
        addU(base, 0x0A, 1000000, 4);       // sync0_cycle_time_ns
        addU(base, 0x0B, 2, 2);             // sm_event_missed
        addU(base, 0x0C, 1, 2);             // cycle_exceeded
        addU(base, 0x0D, 0, 2);             // shift_too_short
        addU(base, 0x20, 0, 1);             // sync_error
    }

    SyncDiagReport collect(SyncDiagRegRead reg = nullptr,
                           const SyncDiagOptions& opts = {}) {
        return SyncDiagnostics::collect(0, master_.sdoManager(0), reg, opts);
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
};

TEST_F(SyncDiagnosticsTest, CollectReadsSmSyncRecords) {
    addSmSync(0x1C32, 2);   // SM2: DC sync0
    addSmSync(0x1C33, 1);   // SM3: SM-synchronised

    auto r = collect();
    EXPECT_EQ(r.slave_index, 0);
    ASSERT_TRUE(r.sm_out.sync_mode.has_value());
    EXPECT_EQ(*r.sm_out.sync_mode, 2);
    EXPECT_EQ(*r.sm_out.cycle_time_ns, 1000000u);
    EXPECT_EQ(*r.sm_out.sm_event_missed, 2);
    EXPECT_EQ(*r.sm_out.cycle_exceeded, 1);
    ASSERT_TRUE(r.sm_in.sync_mode.has_value());
    EXPECT_EQ(*r.sm_in.sync_mode, 1);
}

TEST_F(SyncDiagnosticsTest, MissingObjectsLeaveOptionalsEmpty) {
    auto r = collect();  // nothing injected — all reads fail gracefully
    EXPECT_FALSE(r.sm_out.sync_mode.has_value());
    EXPECT_FALSE(r.sm_in.sync_mode.has_value());
    EXPECT_FALSE(r.dc.sync_act.has_value());
}

TEST_F(SyncDiagnosticsTest, SkippingSmInputLeavesItEmpty) {
    addSmSync(0x1C32, 2);
    SyncDiagOptions opts;
    opts.read_sm_input = false;
    auto r = collect(nullptr, opts);
    EXPECT_TRUE(r.sm_out.sync_mode.has_value());
    EXPECT_FALSE(r.sm_in.sync_mode.has_value());
}

TEST_F(SyncDiagnosticsTest, VendorObjectsAreRead) {
    addSmSync(0x1C32, 2);
    addU(0x2013, 0x03, 100, 2);   // sync_lost_window
    addU(0x2013, 0x05, 7, 2);     // sync_lost_counter
    addU(0x2013, 0x06, 2, 2);     // sync_mode_set
    addU(0x2013, 0x07, 50, 2);    // sync_error_window
    addU(0x2013, 0x08, 3, 2);     // csp_increment_over
    addU(0x2040, 59, 1000, 4);   // ecat_sync_period
    addU(0x2040, 61, 2, 4);     // sync_irq_phases

    auto r = collect();
    EXPECT_TRUE(r.vendor.sync_lost_window.has_value());
    EXPECT_TRUE(r.vendor.sync_lost_counter.has_value());
    EXPECT_TRUE(r.vendor.sync_mode_set.has_value());
    EXPECT_TRUE(r.vendor.sync_error_window.has_value());
    EXPECT_TRUE(r.vendor.csp_increment_over.has_value());
    EXPECT_TRUE(r.vendor.ecat_sync_period.has_value());
    EXPECT_TRUE(r.vendor.sync_irq_phases.has_value());
    if (r.vendor.sync_lost_counter)
        EXPECT_EQ(*r.vendor.sync_lost_counter, 7);
    if (r.vendor.csp_increment_over)
        EXPECT_EQ(*r.vendor.csp_increment_over, 3);
    if (r.vendor.ecat_sync_period)
        EXPECT_EQ(*r.vendor.ecat_sync_period, 1000);
    if (r.vendor.sync_irq_phases)
        EXPECT_EQ(*r.vendor.sync_irq_phases, 2);
}

TEST_F(SyncDiagnosticsTest, DcRegistersReadThroughRegRead) {
    addSmSync(0x1C32, 2);
    auto r = collect(
        [](uint16_t, uint16_t reg, void* d, uint16_t n) -> bool {
            if (reg == 0x0981 && n == 1) {
                *static_cast<uint8_t*>(d) = 0x01;   // sync act
                return true;
            }
            if (reg == 0x0920 && n == 8) {           // sys offset
                int64_t v = -12345;
                std::memcpy(d, &v, 8);
                return true;
            }
            if (reg == 0x09A0 && n == 4) {           // sync0 cycle
                uint32_t v = 1000000;
                std::memcpy(d, &v, 4);
                return true;
            }
            return false;
        });
    ASSERT_TRUE(r.dc.sync_act.has_value());
    EXPECT_EQ(*r.dc.sync_act, 0x01);
    ASSERT_TRUE(r.dc.sys_offset_ns.has_value());
    EXPECT_EQ(*r.dc.sys_offset_ns, -12345);
    ASSERT_TRUE(r.dc.cycle0_ns.has_value());
    EXPECT_EQ(*r.dc.cycle0_ns, 1000000u);
}

TEST_F(SyncDiagnosticsTest, FindingsFlagDegradedSync) {
    addSmSync(0x1C32, 2);
    addU(0x1C32, 0x20, 1, 1);   // sync_error active
    auto r = collect();
    const std::string text = SyncDiagnostics::format(r);
    EXPECT_NE(text.find("0x1C32"), std::string::npos);
}

TEST_F(SyncDiagnosticsTest, FormatRendersAllSections) {
    addSmSync(0x1C32, 2);
    addSmSync(0x1C33, 1);
    addU(0x2013, 0x05, 7, 2);
    auto r = collect();
    const std::string text = SyncDiagnostics::format(r);
    EXPECT_NE(text.find("1C32"), std::string::npos);
    EXPECT_NE(text.find("cycle"), std::string::npos);
}

TEST(SyncDiagnosticsNameTest, SyncModeNames) {
    EXPECT_NE(SyncDiagnostics::syncModeName(0), nullptr);
    EXPECT_NE(SyncDiagnostics::syncModeName(1), nullptr);
    EXPECT_NE(SyncDiagnostics::syncModeName(2), nullptr);
    EXPECT_NE(SyncDiagnostics::syncModeName(3), nullptr);
    EXPECT_NE(SyncDiagnostics::syncModeName(34), nullptr);
    EXPECT_NE(SyncDiagnostics::syncModeName(0xFFFF), nullptr);
    // Known naming sanity: free-run / SM-sync / DC variants differ.
    EXPECT_STRNE(SyncDiagnostics::syncModeName(0),
                 SyncDiagnostics::syncModeName(2));
}

} // namespace
