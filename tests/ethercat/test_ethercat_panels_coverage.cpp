/**
 * @file test_ethercat_panels_coverage.cpp
 * @brief Coverage for the Master→UI adapters in
 *        tether/ui/EthercatPanels.hpp — MasterEntityProvider polls AL
 *        state per discovered slave; SdoRegisterReader does typed and
 *        raw SDO reads — all over an emulated bus.
 */

#include <gtest/gtest.h>

#include <mutex>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ui/EthercatPanels.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
namespace UI = Tether::UI;

namespace {

class EthercatPanelsTest : public ::testing::Test {
protected:
    void SetUp() override {
        for (int i = 0; i < 2; ++i) {
            auto slave =
                Emulator::createCiA402Drive(0x00000002,
                                            static_cast<uint32_t>(0x0442 + i),
                                            i == 0 ? "esc211" : "synapticon");
            if (i == 0) slave_ = slave.get();
            bus_.net.addSlave(std::move(slave));
        }
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 2u);
        // SDO needs a mailbox — configure it on both slaves.
        for (uint16_t i = 0; i < 2; ++i)
            master_.sdoManager(i).configureMailbox(0x1000, 128, 0x1080, 128);
        // A >4-byte string object for the raw-upload path.
        slave_->addODEntry({0x9000, 0,
                            0x0009 /* VISIBLE_STRING */,
                            9 * 8 /* bits */, 0x01 /* read access */,
                            {'T', 'e', 't', 'h', 'e', 'r', '-', 'U', 'I'},
                            "devname"});
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
};

}  // namespace

// ============================================================================
// MasterEntityProvider
// ============================================================================

TEST_F(EthercatPanelsTest, ProviderListsDiscoveredSlaves) {
    UI::MasterEntityProvider prov(master_);
    auto ents = prov.entities();
    ASSERT_EQ(ents.size(), 2u);
    EXPECT_EQ(ents[0].index, 0);
    EXPECT_EQ(ents[1].index, 1);
    EXPECT_FALSE(ents[0].state.empty());          // AL state resolved
    EXPECT_EQ(ents[0].status_code, 0);
    EXPECT_FALSE(ents[0].error);
    EXPECT_FALSE(ents[0].suspended);
}

TEST_F(EthercatPanelsTest, ProviderDetailFnAppendsColumn) {
    UI::MasterEntityProvider prov(master_);
    prov.detail_fn = [](uint16_t i) {
        return "slot" + std::to_string(i);
    };
    auto ents = prov.entities();
    ASSERT_EQ(ents.size(), 2u);
    EXPECT_EQ(ents[0].detail, "slot0");
    EXPECT_EQ(ents[1].detail, "slot1");
}

TEST_F(EthercatPanelsTest, ProviderHoldsMutexDuringPoll) {
    std::mutex m;
    UI::MasterEntityProvider prov(master_, &m);
    bool lock_observed = false;
    prov.detail_fn = [&](uint16_t) {
        // If the provider holds the mutex, try_lock must fail.
        lock_observed = !m.try_lock();
        return "";
    };
    prov.entities();
    EXPECT_TRUE(lock_observed);
}

TEST_F(EthercatPanelsTest, ProviderErrorFlagMirrorsStatusCode) {
    // Whatever AL status the emulated slaves report, the provider maps
    // status_code != 0 → error consistently.
    UI::MasterEntityProvider prov(master_);
    auto ents = prov.entities();
    ASSERT_EQ(ents.size(), 2u);
    for (const auto& e : ents)
        EXPECT_EQ(e.error, e.status_code != 0);
}

TEST_F(EthercatPanelsTest, ProviderSuspendedFlag) {
    UI::MasterEntityProvider prov(master_);
    auto ents = prov.entities();
    EXPECT_FALSE(ents[0].suspended);   // supervisor running normally
}

// ============================================================================
// SdoRegisterReader — typed + raw SDO reads over the wire
// ============================================================================

TEST_F(EthercatPanelsTest, SdoReadsTypedValues) {
    UI::SdoRegisterReader rd(master_);
    // CiA402 drive standard objects, each a different width.
    auto r8 = rd.read(0, {.index = 0x6060, .subindex = 0,
                          .name = "opmode", .bits = 8});
    ASSERT_TRUE(r8.ok) << r8.error;
    EXPECT_NE(r8.value.find("0x"), std::string::npos);
    EXPECT_NE(r8.value.find('('), std::string::npos);  // "0xNN (dec)"

    auto r16 = rd.read(0, {.index = 0x6041, .subindex = 0,
                           .name = "statusword", .bits = 16});
    ASSERT_TRUE(r16.ok) << r16.error;
    EXPECT_NE(r16.value.find("0x"), std::string::npos);

    auto r32 = rd.read(0, {.index = 0x6064, .subindex = 0,
                           .name = "position", .bits = 32});
    ASSERT_TRUE(r32.ok) << r32.error;
    EXPECT_NE(r32.value.find("0x"), std::string::npos);
}

TEST_F(EthercatPanelsTest, SdoReadsRawUnknownSize) {
    UI::SdoRegisterReader rd(master_);
    // bits=0 → raw upload path (hex + ASCII of the injected string).
    auto r = rd.read(0, {.index = 0x9000, .subindex = 0,
                         .name = "devname", .bits = 0});
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_FALSE(r.value.empty());
    // Printable payloads render as a quoted string, not hex.
    EXPECT_NE(r.value.find("Tether-UI"), std::string::npos) << r.value;
}

TEST_F(EthercatPanelsTest, SdoReadErrorCarriesAbortCode) {
    UI::SdoRegisterReader rd(master_);
    auto r = rd.read(0, {.index = 0x9999, .subindex = 0,
                         .name = "missing", .bits = 16});
    ASSERT_FALSE(r.ok);
    EXPECT_FALSE(r.error.empty());
    EXPECT_NE(r.error.find("0x"), std::string::npos);  // abort code appended
}

TEST_F(EthercatPanelsTest, SdoReadMutexPath) {
    std::mutex m;
    UI::SdoRegisterReader rd(master_, &m);
    auto r = rd.read(0, {.index = 0x6041, .subindex = 0,
                         .name = "sw", .bits = 16});
    EXPECT_TRUE(r.ok);
}

TEST_F(EthercatPanelsTest, SdoReadSecondSlave) {
    UI::SdoRegisterReader rd(master_);
    auto r = rd.read(1, {.index = 0x6041, .subindex = 0,
                         .name = "sw", .bits = 16});
    EXPECT_TRUE(r.ok);
}
