/**
 * @file test_slave_pdo_custom_wire.cpp
 * @brief Slave custom-PDO paths over the emulated wire —
 *        configureCustomRxPDO/TxPDO SDO writes, registerExistingPDO
 *        reads, accessor methods, applyCustomPDOs assignment SDOs.
 */

#include <gtest/gtest.h>

#include <array>
#include <span>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/CustomPDOMapping.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/ethercat/PDOTypes.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;

namespace {

void addOD(Emulator::SlaveEmulator* s, uint16_t index, uint8_t sub,
           uint16_t data_type, uint16_t bits, uint8_t access,
           std::vector<uint8_t> data = {}) {
    Emulator::ODEntry e{};
    e.index = index;
    e.subindex = sub;
    e.data_type = data_type;
    e.bit_length = bits;
    e.access = access;   // 0x01 read | 0x02 write
    e.data = std::move(data);
    if (e.data.empty()) e.data.assign((bits + 7) / 8, 0);
    e.name = "od";
    s->addODEntry(e);
}

// Register the SDO objects the custom-PDO paths touch on the emulator:
// 0x1600/0x1A00 mapping objects (U8 count + U32 entries) and the
// 0x1C12/0x1C13 SM assignment objects (U8 count + U16 entries).
void addPdoMappingObjects(Emulator::SlaveEmulator* s) {
    for (uint16_t idx : {0x1600, 0x1A00}) {
        addOD(s, idx, 0, 0x0005, 8, 0x03);                 // count U8
        for (uint8_t sub = 1; sub <= 8; ++sub)
            addOD(s, idx, sub, 0x0007, 32, 0x03);          // entry U32
    }
    for (uint16_t idx : {0x1C12, 0x1C13}) {
        addOD(s, idx, 0, 0x0005, 8, 0x03);                 // count U8
        for (uint8_t sub = 1; sub <= 8; ++sub)
            addOD(s, idx, sub, 0x0006, 16, 0x03);          // assign U16
    }
}

class CustomPdoWireTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0xCAFE,
                                                 "pdotest");
        slave_ = slave.get();
        addPdoMappingObjects(slave_);
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

    // Fake OD entries used only to describe field layout/sizes — their
    // pointers are borrowed by the mapping info, never dereferenced for
    // the SDO writes.
    ObjectDictionary::ObjectDictionaryEntry e1_{
        .index = 0x6040, .subindex = 0, .name = "controlword",
        .data_type = ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
        .default_value = 0};
    ObjectDictionary::ObjectDictionaryEntry e2_{
        .index = 0x6064, .subindex = 0, .name = "target pos",
        .data_type = ObjectDictionary::ObjectDictionaryDataType::Integer32,
        .default_value = 0};
    ObjectDictionary::ObjectDictionaryEntry e3_{
        .index = 0x6041, .subindex = 0, .name = "statusword",
        .data_type = ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
        .default_value = 0};

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slave_ = nullptr;
    Master master_;
};

} // namespace

// configureCustomRxPDO writes count-clear, entries, count over SDO.
TEST_F(CustomPdoWireTest, ConfigureCustomRxPDOOverWire) {
    auto& slave = master_.slave(0);
    ASSERT_EQ(slave.configureCustomRxPDO(0x1600, {&e1_, &e2_}),
              SlaveError::Ok);

    // Emulator OD should now hold the written mapping.
    const auto* cnt = slave_->odEntry(0x1600, 0);
    ASSERT_NE(cnt, nullptr);
    ASSERT_EQ(cnt->data.size(), 1u);
    EXPECT_EQ(cnt->data[0], 2);
    const auto* m1 = slave_->odEntry(0x1600, 1);
    ASSERT_NE(m1, nullptr);
    ASSERT_EQ(m1->data.size(), 4u);
    // 0x6040 sub0 16 bits → 0x60400010 little-endian.
    EXPECT_EQ(m1->data[0], 0x10);
    EXPECT_EQ(m1->data[1], 0x00);
    EXPECT_EQ(m1->data[2], 0x40);
    EXPECT_EQ(m1->data[3], 0x60);

    EXPECT_EQ(slave.customPDOSize(0x1600), 6);   // 2 + 4 bytes
    // Mapping storage isn't allocated until applyCustomPDOs().
    EXPECT_EQ(slave.customPDOData(0x1600), nullptr);
    EXPECT_EQ(slave.customPDOField<uint16_t>(0x1600, 0), nullptr);
    EXPECT_EQ(slave.customPDOField(0x1600, &e1_), nullptr);
}

TEST_F(CustomPdoWireTest, ConfigureCustomTxPDOOverWire) {
    auto& slave = master_.slave(0);
    ASSERT_EQ(slave.configureCustomTxPDO(0x1A00, {&e3_}),
              SlaveError::Ok);
    const auto* cnt = slave_->odEntry(0x1A00, 0);
    ASSERT_NE(cnt, nullptr);
    EXPECT_EQ(cnt->data[0], 1);
    EXPECT_EQ(slave.customPDOSize(0x1A00), 2);
}

// The span overload accepts dynamically-built tables and an explicit
// direction; an entry whose size can't be inferred fails validation.
TEST_F(CustomPdoWireTest, ConfigureSpanOverloadAndUnresolved) {
    auto& slave = master_.slave(0);

    std::array<CustomPDOMappingEntry, 1> ok{CustomPDOMappingEntry(&e1_)};
    EXPECT_EQ(slave.configureCustomTxPDO(
                  0x1A00, std::span<const CustomPDOMappingEntry>(ok),
                  PDO::PDODirection::TxPDO),
              SlaveError::Ok);
    slave.clearCustomPDOs();

    // OctetString size cannot be inferred and no explicit size was given.
    ObjectDictionary::ObjectDictionaryEntry bad{
        .index = 0x1234, .subindex = 0, .name = "bad",
        .data_type = ObjectDictionary::ObjectDictionaryDataType::OctetString,
        .default_value = 0};
    CustomPDOMappingEntry be(&bad, 0);
    EXPECT_EQ(be.resolvedSize(), 0);
    std::array<CustomPDOMappingEntry, 1> arr{be};
    EXPECT_EQ(slave.configureCustomTxPDO(
                  0x1A00, std::span<const CustomPDOMappingEntry>(arr),
                  PDO::PDODirection::TxPDO),
              SlaveError::PDOMappingFailed);
}

TEST_F(CustomPdoWireTest, ConfigureEmptyFailsBeforeWire) {
    auto& slave = master_.slave(0);
    EXPECT_EQ(slave.configureCustomRxPDO(0x1600, {}),
              SlaveError::PDOMappingFailed);
}

TEST_F(CustomPdoWireTest, ConfigureMissingObjectAborts) {
    auto& slave = master_.slave(0);
    // 0x16F0 has no OD entry on the emulator → first SDO write aborts.
    EXPECT_NE(slave.configureCustomRxPDO(0x16F0, {&e1_}),
              SlaveError::Ok);
}

// registerExistingPDO reads the device-side mapping and parses entries.
TEST_F(CustomPdoWireTest, RegisterExistingPDOReadsMapping) {
    auto& slave = master_.slave(0);

    // Preload a valid mapping on the emulator: count=2, two U32 entries.
    addOD(slave_, 0x1A00, 0, 0x0005, 8, 0x03, {2});
    addOD(slave_, 0x1A00, 1, 0x0007, 32, 0x03,
          {0x10, 0x00, 0x41, 0x60});        // 0x6041:00 16 bits
    addOD(slave_, 0x1A00, 2, 0x0007, 32, 0x03,
          {0x20, 0x00, 0x64, 0x60});        // 0x6064:00 32 bits

    ASSERT_EQ(slave.registerExistingTxPDO(0x1A00), SlaveError::Ok);
    EXPECT_EQ(slave.customPDOSize(0x1A00), 6);   // 16+32 bits
    // Fields resolve to nullptr until applyCustomPDOs allocates storage.
    EXPECT_EQ(slave.customPDOField<uint16_t>(0x1A00, 0), nullptr);
}

TEST_F(CustomPdoWireTest, RegisterExistingPDOErrors) {
    auto& slave = master_.slave(0);

    // Missing mapping object → SDO read fails.
    EXPECT_EQ(slave.registerExistingRxPDO(0x16EE),
              SlaveError::SDOError);

    // Empty mapping (count=0) → PDOMappingFailed.
    addOD(slave_, 0x1A00, 0, 0x0005, 8, 0x03, {0});
    EXPECT_EQ(slave.registerExistingTxPDO(0x1A00),
              SlaveError::PDOMappingFailed);

    // All-zero entries → uninitialized mapping.
    addOD(slave_, 0x1A00, 0, 0x0005, 8, 0x03, {2});
    addOD(slave_, 0x1A00, 1, 0x0007, 32, 0x03, {0, 0, 0, 0});
    addOD(slave_, 0x1A00, 2, 0x0007, 32, 0x03, {0, 0, 0, 0});
    EXPECT_EQ(slave.registerExistingTxPDO(0x1A00),
              SlaveError::PDOMappingFailed);

    // Non-byte-aligned bit length → unsupported.
    addOD(slave_, 0x1A00, 1, 0x0007, 32, 0x03,
          {0x03, 0x00, 0x41, 0x60});        // 3 bits — not aligned
    EXPECT_EQ(slave.registerExistingTxPDO(0x1A00),
              SlaveError::PDOMappingFailed);
}

TEST_F(CustomPdoWireTest, AccessorsAndClear) {
    auto& slave = master_.slave(0);
    EXPECT_EQ(slave.customPDOData(0x1600), nullptr);
    EXPECT_EQ(slave.customPDOSize(0x1600), 0);
    EXPECT_EQ(slave.customPDOField<uint16_t>(0x1600, 0), nullptr);
    EXPECT_EQ(slave.customPDOField(0x1600, &e1_), nullptr);

    ASSERT_EQ(slave.configureCustomRxPDO(0x1600, {&e1_}),
              SlaveError::Ok);
    EXPECT_EQ(slave.customPDOSize(0x1600), 2);

    slave.clearCustomPDOs();
    EXPECT_EQ(slave.customPDOSize(0x1600), 0);
    EXPECT_EQ(slave.customPDOField<uint16_t>(0x1600, 0), nullptr);
}

// applyCustomPDOs registers entries into the PDO mapping, allocates
// storage, and writes 0x1C12/0x1C13 SM assignments over SDO.
TEST_F(CustomPdoWireTest, ApplyCustomPDOsAssignsSMs) {
    auto& slave = master_.slave(0);
    ASSERT_EQ(slave.configureCustomRxPDO(0x1600, {&e1_, &e2_}),
              SlaveError::Ok);
    ASSERT_EQ(slave.configureCustomTxPDO(0x1A00, {&e3_}),
              SlaveError::Ok);

    ASSERT_EQ(slave.applyCustomPDOs(), SlaveError::Ok);

    // SM2 assignment holds 0x1600; SM3 holds 0x1A00.
    const auto* sm2 = slave_->odEntry(0x1C12, 1);
    ASSERT_NE(sm2, nullptr);
    ASSERT_EQ(sm2->data.size(), 2u);
    EXPECT_EQ(sm2->data[0], 0x00);
    EXPECT_EQ(sm2->data[1], 0x16);
    const auto* sm3 = slave_->odEntry(0x1C13, 1);
    ASSERT_NE(sm3, nullptr);
    EXPECT_EQ(sm3->data[0], 0x00);
    EXPECT_EQ(sm3->data[1], 0x1A);

    // Storage is now live — field accessors resolve into the buffer.
    ASSERT_NE(slave.customPDOData(0x1600), nullptr);
    EXPECT_NE(slave.customPDOField<uint16_t>(0x1600, 0), nullptr);
    EXPECT_NE(slave.customPDOField<int32_t>(0x1600, 1), nullptr);
    EXPECT_EQ(slave.customPDOField<uint16_t>(0x1600, 2), nullptr); // OOB
    EXPECT_NE(slave.customPDOField(0x1600, &e1_), nullptr);
    EXPECT_NE(slave.customPDOField(0x1600, &e2_), nullptr);
    // e3_ belongs to the TxPDO, not 0x1600.
    EXPECT_EQ(slave.customPDOField(0x1600, &e3_), nullptr);
}

// applyCustomPDOs with nothing configured is a warned no-op.
TEST_F(CustomPdoWireTest, ApplyEmptyIsNoop) {
    auto& slave = master_.slave(0);
    slave.clearCustomPDOs();
    EXPECT_EQ(slave.applyCustomPDOs(), SlaveError::Ok);
}

// configureMultiPDOs writes SM registers over the wire and assigns
// PDOs to 0x1C12/0x1C13 via SDO.
TEST_F(CustomPdoWireTest, ConfigureMultiPDOsOverWire) {
    auto& slave = master_.slave(0);

    MultiPDOAssignment cfg;
    MultiPDOAssignment::SMConfig sm2{};
    sm2.sm_index = 2;
    sm2.phys_start_addr = 0x1800;
    sm2.control_byte = 0x26;   // buffered 3-buffer + write direction (bit2)
    sm2.pdo_mappings = {PDO::PDOMappingRegion(0x1600, 6)};
    MultiPDOAssignment::SMConfig sm3{};
    sm3.sm_index = 3;
    sm3.phys_start_addr = 0x1C00;
    sm3.control_byte = 0x22;   // buffered, read direction
    sm3.pdo_mappings = {PDO::PDOMappingRegion(0x1A00, 2)};
    cfg.sm_configs = {sm2, sm3};

    ASSERT_EQ(slave.configureMultiPDOs(cfg), SlaveError::Ok);

    // PDO assignment objects written over SDO.
    const auto* sm2c = slave_->odEntry(0x1C12, 0);
    ASSERT_NE(sm2c, nullptr);
    EXPECT_EQ(sm2c->data[0], 1);
    const auto* sm2a = slave_->odEntry(0x1C12, 1);
    ASSERT_NE(sm2a, nullptr);
    EXPECT_EQ(sm2a->data[0], 0x00);
    EXPECT_EQ(sm2a->data[1], 0x16);

    // SM2 registers written over the wire: start addr, length, control.
    uint8_t reg[8] = {};
    ASSERT_TRUE(master_.readRegister(EtherCAT::SlaveAddress(0),
                                     0x0810, reg, 8, 200));
    EXPECT_EQ(reg[0] | (reg[1] << 8), 0x1800);   // phys start
    EXPECT_EQ(reg[2] | (reg[3] << 8), 6);        // length
    EXPECT_EQ(reg[4], 0x26);                    // control
}

TEST_F(CustomPdoWireTest, ConfigureMultiPDOsEmptyIsNoop) {
    auto& slave = master_.slave(0);
    EXPECT_EQ(slave.configureMultiPDOs(MultiPDOAssignment{}),
              SlaveError::Ok);
}
