/**
 * @file test_SlaveIdentification_bus.cpp
 * @brief Emulator-backed coverage for SlaveIdentifier — identify(),
 *        identifyAll() and the individual SDO read helpers
 *        (identity record, device type, strings incl. subindex fallback
 *        and trimming, error register, supported drive modes).
 */

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/profiles/cia301/SlaveIdentification.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class SlaveIdentifierBusTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0x0442, "ident");
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

    void addStr(uint16_t idx, uint8_t sub, const std::string& s) {
        slave_->addODEntry({idx, sub, 0x0009,
                            static_cast<uint16_t>(s.size() * 8), 0x01,
                            std::vector<uint8_t>(s.begin(), s.end()), "s"});
    }
    void addU32(uint16_t idx, uint8_t sub, uint32_t v) {
        slave_->addODEntry({idx, sub, 0x0007, 32, 0x01,
                            {static_cast<uint8_t>(v),
                             static_cast<uint8_t>(v >> 8),
                             static_cast<uint8_t>(v >> 16),
                             static_cast<uint8_t>(v >> 24)}, "u"});
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
};

} // namespace

TEST_F(SlaveIdentifierBusTest, IdentifyPopulatesIdentity) {
    // The emulator ships 0x1018 with zeroed data — fill it in.
    addU32(0x1018, 1, 0x00000002);
    addU32(0x1018, 2, 0x00000442);
    addU32(0x1018, 3, 0x00010000);
    addU32(0x1018, 4, 0xABCDEF01);
    addStr(0x1008, 0, "TestDrive");
    addStr(0x1009, 0, "HW 1.2");
    addStr(0x100A, 0, "SW 3.4");

    SlaveIdentifier idf(master_);
    SlaveIdentity ident;
    EXPECT_TRUE(idf.identify(0, ident));
    EXPECT_TRUE(ident.isValid());
    EXPECT_EQ(ident.slaveIndex(), 0);
    // The CiA402 emulator ships 0x1018 vendor/product entries.
    EXPECT_EQ(ident.identityRecord().vendor_id, 0x00000002u);
    EXPECT_EQ(ident.identityRecord().product_code, 0x00000442u);
    EXPECT_EQ(ident.identityRecord().revision_number, 0x00010000u);
    EXPECT_EQ(ident.identityRecord().serial_number, 0xABCDEF01u);
    EXPECT_STREQ(ident.deviceName(), "TestDrive");
    EXPECT_STREQ(ident.hardwareVersion(), "HW 1.2");
    EXPECT_STREQ(ident.softwareVersion(), "SW 3.4");
}

TEST_F(SlaveIdentifierBusTest, IdentifyAllCountsSuccesses) {
    addU32(0x1018, 1, 0x00000002);
    SlaveIdentifier idf(master_);
    SlaveIdentity ids[2];
    EXPECT_EQ(idf.identifyAll(ids, 2, 1), 1u);
    EXPECT_TRUE(ids[0].isValid());
    // Degenerate calls return 0.
    EXPECT_EQ(idf.identifyAll(nullptr, 2, 1), 0u);
    EXPECT_EQ(idf.identifyAll(ids, 0, 1), 0u);
}

TEST_F(SlaveIdentifierBusTest, ReadIdentityRecordAndDeviceType) {
    addU32(0x1018, 1, 0x00000002);
    SlaveIdentifier idf(master_);
    IdentityRecord rec;
    EXPECT_TRUE(idf.readIdentityRecord(0, rec));
    EXPECT_EQ(rec.vendor_id, 0x00000002u);

    DeviceTypeInfo type;
    EXPECT_TRUE(idf.readDeviceType(0, type));
}

TEST_F(SlaveIdentifierBusTest, ReadStringVariants) {
    SlaveIdentifier idf(master_);
    char buf[64];
    size_t len = 0;

    // String at subindex 0.
    addStr(0x2000, 0, "hello  \r\n");
    EXPECT_TRUE(idf.readString(0, 0x2000, buf, sizeof(buf), &len));
    EXPECT_STREQ(buf, "hello");   // trailing whitespace trimmed
    EXPECT_EQ(len, 5u);

    // String only at subindex 1 — the subindex-0 read fails, fallback wins.
    addStr(0x2001, 1, "fallback");
    EXPECT_TRUE(idf.readString(0, 0x2001, buf, sizeof(buf), &len));
    EXPECT_STREQ(buf, "fallback");

    // Missing object entirely.
    EXPECT_FALSE(idf.readString(0, 0x2002, buf, sizeof(buf), &len));

    // Null / zero-size buffers are rejected without touching the bus.
    EXPECT_FALSE(idf.readString(0, 0x2000, nullptr, 16, &len));
    EXPECT_FALSE(idf.readString(0, 0x2000, buf, 0, &len));
}

TEST_F(SlaveIdentifierBusTest, ReadErrorRegisterAndDriveModes) {
    SlaveIdentifier idf(master_);
    uint8_t err = 0xFF;
    EXPECT_TRUE(idf.readErrorRegister(0, err));
    EXPECT_EQ(err, 0);

    addU32(0x6502, 0, 0x000000ED);   // supported drive modes
    uint32_t modes = 0;
    EXPECT_TRUE(idf.readSupportedDriveModes(0, modes));
    EXPECT_EQ(modes, 0x000000EDu);
}
