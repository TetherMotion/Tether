/**
 * @file test_slave_sdo_coverage.cpp
 * @brief Emulator-backed coverage for Slave's SDO convenience APIs —
 *        typed reads/writes, object-dictionary entry accessors and the
 *        last-abort introspection helpers.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
using D = ObjectDictionary::ObjectDictionaryDataType;

namespace {

class SlaveSdoCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000F0A, 0x9900, "sdo");
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

    Slave& sl() { return master_.slave(0); }

    // A read-only or read-write OD entry on the emulator.
    void addEntry(uint16_t index, uint8_t sub, uint16_t dt,
                  std::vector<uint8_t> data, uint8_t access = 0x03) {
        slave_->addODEntry({index, sub, dt,
                            static_cast<uint16_t>(data.size() * 8),
                            access, std::move(data), "t"});
    }

    static ObjectDictionary::ObjectDictionaryEntry entry(
        uint16_t index, uint8_t sub, D dt) {
        ObjectDictionary::ObjectDictionaryEntry e{};
        e.index = index;
        e.subindex = sub;
        e.data_type = dt;
        return e;
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
};

// ============================================================================
// Typed SDO convenience
// ============================================================================

TEST_F(SlaveSdoCoverageTest, SdoReadWriteGenericRoundTrip) {
    addEntry(0x2000, 0, 0x0007, {0x78, 0x56, 0x34, 0x12});
    uint32_t v = 0;
    size_t size = sizeof(v);
    ASSERT_EQ(sl().sdoRead(0x2000, 0, &v, size), SlaveError::Ok);
    EXPECT_EQ(v, 0x12345678u);
    EXPECT_EQ(size, 4u);

    const uint32_t nv = 0xCAFEBABEu;
    ASSERT_EQ(sl().sdoWrite(0x2000, 0, &nv, sizeof(nv)), SlaveError::Ok);
    v = 0;
    size = sizeof(v);
    ASSERT_EQ(sl().sdoRead(0x2000, 0, &v, size), SlaveError::Ok);
    EXPECT_EQ(v, nv);
}

TEST_F(SlaveSdoCoverageTest, SdoTypedReads) {
    addEntry(0x2001, 0, 0x0005, {0x42});
    addEntry(0x2002, 0, 0x0006, {0x34, 0x12});
    addEntry(0x2003, 0, 0x0007, {0xEF, 0xBE, 0xAD, 0xDE});
    uint8_t u8 = 0;
    uint16_t u16 = 0;
    uint32_t u32 = 0;
    EXPECT_EQ(sl().sdoReadU8(0x2001, 0, u8), SlaveError::Ok);
    EXPECT_EQ(sl().sdoReadU16(0x2002, 0, u16), SlaveError::Ok);
    EXPECT_EQ(sl().sdoReadU32(0x2003, 0, u32), SlaveError::Ok);
    EXPECT_EQ(u8, 0x42);
    EXPECT_EQ(u16, 0x1234);
    EXPECT_EQ(u32, 0xDEADBEEFu);
}

TEST_F(SlaveSdoCoverageTest, SdoTypedWrites) {
    addEntry(0x2010, 0, 0x0005, {0x00});
    addEntry(0x2011, 0, 0x0006, {0x00, 0x00});
    addEntry(0x2012, 0, 0x0007, {0x00, 0x00, 0x00, 0x00});
    EXPECT_EQ(sl().sdoWriteU8(0x2010, 0, 0xAB), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteU16(0x2011, 0, 0xBEEF), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteU32(0x2012, 0, 0x01020304), SlaveError::Ok);
    EXPECT_EQ(slave_->odEntry(0x2010, 0)->data[0], 0xAB);
    EXPECT_EQ(slave_->odEntry(0x2011, 0)->data[0], 0xEF);
    EXPECT_EQ(slave_->odEntry(0x2011, 0)->data[1], 0xBE);
    EXPECT_EQ(slave_->odEntry(0x2012, 0)->data[3], 0x01);
}

TEST_F(SlaveSdoCoverageTest, SdoAbortOnMissingObject) {
    uint8_t buf[4] = {};
    size_t size = sizeof(buf);
    EXPECT_EQ(sl().sdoRead(0x9999, 0, buf, size), SlaveError::SDOAborted);
    EXPECT_EQ(sl().sdoWrite(0x9999, 0, buf, sizeof(buf)), SlaveError::SDOAborted);

    uint8_t u8 = 0;
    EXPECT_EQ(sl().sdoReadU8(0x9999, 0, u8), SlaveError::SDOAborted);
    EXPECT_EQ(sl().sdoWriteU8(0x9999, 0, 1), SlaveError::SDOAborted);
}

TEST_F(SlaveSdoCoverageTest, SdoErrorOnTransportFailure) {
    EtherCAT::Emulator::ErrorInjection inj;
    inj.inject_timeout = true;   // all reads fail → transport error, no abort
    slave_->setErrorInjection(inj);

    uint8_t buf[4] = {};
    size_t size = sizeof(buf);
    EXPECT_EQ(sl().sdoRead(0x1000, 0, buf, size), SlaveError::SDOError);
    EXPECT_EQ(sl().sdoWrite(0x1000, 0, buf, sizeof(buf)), SlaveError::SDOError);
    uint8_t u8 = 0;
    EXPECT_EQ(sl().sdoReadU8(0x1001, 0, u8), SlaveError::SDOError);
    EXPECT_EQ(sl().sdoWriteU8(0x1001, 0, 1), SlaveError::SDOError);
}

TEST_F(SlaveSdoCoverageTest, MailboxNotConfiguredInSafeOp) {
    // mailboxAvailable() returns false while the slave sits in SAFE_OP
    // without the mailbox serviced.
    sl().setMailboxServicedInSafeOp(false);
    if (slave_->getState() != SlaveState::PRE_OP)
        slave_->requestState(SlaveState::PRE_OP);
    slave_->requestState(SlaveState::SAFE_OP);
    ASSERT_EQ(slave_->getState(), SlaveState::SAFE_OP);

    uint8_t buf[4] = {};
    size_t size = sizeof(buf);
    EXPECT_EQ(sl().sdoRead(0x1000, 0, buf, size), SlaveError::MailboxNotConfigured);
    EXPECT_EQ(sl().sdoWrite(0x1000, 0, buf, sizeof(buf)), SlaveError::MailboxNotConfigured);
    uint8_t u8 = 0;
    EXPECT_EQ(sl().sdoReadU8(0x1001, 0, u8), SlaveError::MailboxNotConfigured);
}

// ============================================================================
// sdoReadEntry — per-data-type arms
// ============================================================================

TEST_F(SlaveSdoCoverageTest, ReadEntryUnsignedWidths) {
    addEntry(0x3000, 0, 0x0005, {0x42});                       // U8
    addEntry(0x3001, 0, 0x0006, {0x34, 0x12});                 // U16
    addEntry(0x3002, 0, 0x0007, {0xEF, 0xBE, 0xAD, 0xDE});     // U32
    addEntry(0x3003, 0, 0x0001, {0x01});                       // Boolean

    uint64_t out = 0;
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3000, 0, D::Unsigned8), out), SlaveError::Ok);
    EXPECT_EQ(out, 0x42u);
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3001, 0, D::Unsigned16), out), SlaveError::Ok);
    EXPECT_EQ(out, 0x1234u);
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3002, 0, D::Unsigned32), out), SlaveError::Ok);
    EXPECT_EQ(out, 0xDEADBEEFu);
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3003, 0, D::Boolean), out), SlaveError::Ok);
    EXPECT_EQ(out, 1u);
}

TEST_F(SlaveSdoCoverageTest, ReadEntrySignedWidthsSignExtend) {
    addEntry(0x3010, 0, 0x0002, {0xFE});                       // I8  = -2
    addEntry(0x3011, 0, 0x0003, {0xFC, 0xFF});                 // I16 = -4
    addEntry(0x3012, 0, 0x0004, {0xFB, 0xFF, 0xFF, 0xFF});     // I32 = -5

    uint64_t out = 0;
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3010, 0, D::Integer8), out), SlaveError::Ok);
    EXPECT_EQ(out, static_cast<uint64_t>(static_cast<int64_t>(-2)));
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3011, 0, D::Integer16), out), SlaveError::Ok);
    EXPECT_EQ(out, static_cast<uint64_t>(static_cast<int64_t>(-4)));
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3012, 0, D::Integer32), out), SlaveError::Ok);
    EXPECT_EQ(out, static_cast<uint64_t>(static_cast<int64_t>(-5)));
}

TEST_F(SlaveSdoCoverageTest, ReadEntryRealAndWideTypes) {
    addEntry(0x3020, 0, 0x0008, {0x00, 0x00, 0x80, 0x3F});     // Real32 = 1.0f
    addEntry(0x3021, 0, 0x001B,
             {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}); // U64

    uint64_t out = 0;
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3020, 0, D::Real32), out), SlaveError::Ok);
    EXPECT_EQ(out, 0x3F800000u);
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3021, 0, D::Unsigned64), out), SlaveError::Ok);
    EXPECT_EQ(out, 0x8877665544332211ull);
}

TEST_F(SlaveSdoCoverageTest, ReadEntryUnsupportedTypeFails) {
    // VisibleString has no fixed inferByteSize → SDOError without any wire IO.
    uint64_t out = 0;
    EXPECT_EQ(sl().sdoReadEntry(entry(0x1000, 0, D::VisibleString), out),
              SlaveError::SDOError);
}

TEST_F(SlaveSdoCoverageTest, ReadEntryPropagatesAbort) {
    uint64_t out = 0;
    EXPECT_EQ(sl().sdoReadEntry(entry(0x9999, 0, D::Unsigned8), out),
              SlaveError::SDOAborted);
}

TEST_F(SlaveSdoCoverageTest, ReadEntrySubindexOverride) {
    addEntry(0x3030, 7, 0x0005, {0x77});
    uint64_t out = 0;
    EXPECT_EQ(sl().sdoReadEntry(entry(0x3030, 0, D::Unsigned8), 7, out),
              SlaveError::Ok);
    EXPECT_EQ(out, 0x77u);
}

// ============================================================================
// sdoWriteEntry — per-data-type arms
// ============================================================================

TEST_F(SlaveSdoCoverageTest, WriteEntryWidths) {
    addEntry(0x3100, 0, 0x0005, {0x00});
    addEntry(0x3101, 0, 0x0006, {0x00, 0x00});
    addEntry(0x3102, 0, 0x0007, {0x00, 0x00, 0x00, 0x00});
    addEntry(0x3103, 0, 0x001B, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});

    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3100, 0, D::Unsigned8), 0xAB), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3100, 0, D::Integer8), 0xCD), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3101, 0, D::Unsigned16), 0xBEEF), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3101, 0, D::Integer16), 0xFEED), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3102, 0, D::Unsigned32), 0x01020304), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3102, 0, D::Real32), 0xDEADBEEF), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3102, 0, D::Integer32), 0x0BADF00D), SlaveError::Ok);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3103, 0, D::Unsigned64), 0x8877665544332211ull),
              SlaveError::Ok);

    EXPECT_EQ(slave_->odEntry(0x3100, 0)->data[0], 0xCD);
    EXPECT_EQ(slave_->odEntry(0x3101, 0)->data[0], 0xED);
    EXPECT_EQ(slave_->odEntry(0x3101, 0)->data[1], 0xFE);
    EXPECT_EQ(slave_->odEntry(0x3102, 0)->data[0], 0x0D);
    EXPECT_EQ(slave_->odEntry(0x3103, 0)->data[0], 0x11);
    EXPECT_EQ(slave_->odEntry(0x3103, 0)->data[7], 0x88);
}

TEST_F(SlaveSdoCoverageTest, WriteEntryBooleanAndUnsupported) {
    addEntry(0x3110, 0, 0x0001, {0x00});
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3110, 0, D::Boolean), 1), SlaveError::Ok);
    EXPECT_EQ(slave_->odEntry(0x3110, 0)->data[0], 0x01);

    EXPECT_EQ(sl().sdoWriteEntry(entry(0x1000, 0, D::VisibleString), 0),
              SlaveError::SDOError);
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x9999, 0, D::Unsigned8), 1),
              SlaveError::SDOAborted);
}

TEST_F(SlaveSdoCoverageTest, WriteEntrySubindexOverride) {
    addEntry(0x3120, 9, 0x0005, {0x00});
    EXPECT_EQ(sl().sdoWriteEntry(entry(0x3120, 0, D::Unsigned8), 9, 0x55),
              SlaveError::Ok);
    EXPECT_EQ(slave_->odEntry(0x3120, 9)->data[0], 0x55);
}

// ============================================================================
// Abort introspection
// ============================================================================

TEST_F(SlaveSdoCoverageTest, LastAbortAccessors) {
    uint8_t buf[4] = {};
    size_t size = sizeof(buf);
    ASSERT_EQ(sl().sdoWrite(0x9999, 0, buf, sizeof(buf)), SlaveError::SDOAborted);
    EXPECT_NE(sl().lastSdoAbortCode(), 0u);
    EXPECT_TRUE(sl().lastSdoWasDownload());
    EXPECT_EQ(sl().lastSdoAttemptedLength(), sizeof(buf));

    ASSERT_EQ(sl().sdoRead(0x9999, 0, buf, size), SlaveError::SDOAborted);
    EXPECT_NE(sl().lastSdoAbortCode(), 0u);
    EXPECT_FALSE(sl().lastSdoWasDownload());
}

}  // namespace
