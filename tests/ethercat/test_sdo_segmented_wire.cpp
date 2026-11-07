/**
 * @file test_sdo_segmented_wire.cpp
 * @brief Segmented CoE SDO upload/download over the emulated wire —
 *        objects larger than one mailbox frame exercise the segmented
 *        transfer paths in SDOUpload/SDODownload_Segmented plus abort
 *        handling.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "raw/RawConstants.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;

namespace {

class SegmentedWireTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0xBEEF,
                                                 "segtest");
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

    void addBigObject(uint16_t index, size_t bytes, uint8_t access = 0x03) {
        Emulator::ODEntry e{};
        e.index = index;
        e.subindex = 0;
        e.data_type = 0x000F;   // OCTET_STRING
        e.bit_length = static_cast<uint16_t>(bytes * 8);
        e.access = access;
        e.data.assign(bytes, 0xA5);
        e.name = "big object";
        slave_->addODEntry(e);
    }

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slave_ = nullptr;
    Master master_;
};

} // namespace

// Segmented upload: 400-byte object over 128-byte mailboxes requires
// multiple upload-segment response round-trips.
TEST_F(SegmentedWireTest, SegmentedUploadLargeObject) {
    addBigObject(0x7000, 400);
    std::vector<uint8_t> buf(400, 0);
    size_t got = buf.size();
    ASSERT_EQ(master_.slave(0).sdoRead(0x7000, 0, buf.data(), got),
              SlaveError::Ok);
    EXPECT_EQ(got, 400u);
    for (size_t i = 0; i < got; ++i) EXPECT_EQ(buf[i], 0xA5) << i;
}

// Segmented download: write a large object through several
// download-segment request round-trips, then read it back.
TEST_F(SegmentedWireTest, SegmentedDownloadThenReadBack) {
    addBigObject(0x7001, 400);
    std::vector<uint8_t> wr(400);
    for (size_t i = 0; i < wr.size(); ++i) wr[i] = i & 0xFF;
    ASSERT_EQ(master_.slave(0).sdoWrite(0x7001, 0, wr.data(), wr.size()),
              SlaveError::Ok);

    std::vector<uint8_t> rd(400, 0);
    size_t got = rd.size();
    ASSERT_EQ(master_.slave(0).sdoRead(0x7001, 0, rd.data(), got),
              SlaveError::Ok);
    EXPECT_EQ(rd, wr);
}

// Upload a missing object → slave aborts (0x06020000 object not found).
TEST_F(SegmentedWireTest, UploadNonexistentAborts) {
    std::array<uint8_t, 8> buf{};
    size_t got = buf.size();
    const auto err = master_.slave(0).sdoRead(0x77FE, 0, buf.data(), got);
    EXPECT_NE(err, SlaveError::Ok);
}

// Write to a read-only object → abort.
TEST_F(SegmentedWireTest, DownloadToReadOnlyAborts) {
    addBigObject(0x7002, 64, /*access=*/0x01);   // read-only
    std::array<uint8_t, 16> wr{};
    EXPECT_NE(master_.slave(0).sdoWrite(0x7002, 0, wr.data(), wr.size()),
              SlaveError::Ok);
}

// Medium object (~100 bytes): fits roughly the segmented threshold edge.
TEST_F(SegmentedWireTest, BoundarySizeUpload) {
    addBigObject(0x7003, 100);
    std::vector<uint8_t> buf(100, 0);
    size_t got = buf.size();
    ASSERT_EQ(master_.slave(0).sdoRead(0x7003, 0, buf.data(), got),
              SlaveError::Ok);
    EXPECT_EQ(got, 100u);
}

// Zero-length read on an existing object.
TEST_F(SegmentedWireTest, TinyUpload) {
    Emulator::ODEntry e{};
    e.index = 0x7004;
    e.subindex = 0;
    e.data_type = 0x0006;
    e.bit_length = 16;
    e.access = 0x03;
    e.data = {0x34, 0x12};
    e.name = "u16";
    slave_->addODEntry(e);

    uint16_t v = 0;
    EXPECT_EQ(master_.slave(0).sdoReadU16(0x7004, 0, v), SlaveError::Ok);
    EXPECT_EQ(v, 0x1234);

    EXPECT_EQ(master_.slave(0).sdoWriteU16(0x7004, 0, 0xCAFE),
              SlaveError::Ok);
    v = 0;
    EXPECT_EQ(master_.slave(0).sdoReadU16(0x7004, 0, v), SlaveError::Ok);
    EXPECT_EQ(v, 0xCAFE);
}

// requestCancel() before the upload hits the early isCancelRequested()
// checks in SDOUpload::execute / the mailbox wait loop.
TEST_F(SegmentedWireTest, UploadCancelledBeforeStart) {
    addBigObject(0x7010, 400);
    master_.requestCancel();
    std::vector<uint8_t> buf(400, 0);
    size_t got = buf.size();
    const auto err = master_.slave(0).sdoRead(0x7010, 0, buf.data(), got);
    EXPECT_NE(err, SlaveError::Ok);
    master_.clearCancel();
}

// Same for the segmented download path.
TEST_F(SegmentedWireTest, DownloadCancelledBeforeStart) {
    addBigObject(0x7011, 400);
    master_.requestCancel();
    std::vector<uint8_t> wr(400, 0x11);
    const auto err = master_.slave(0).sdoWrite(0x7011, 0, wr.data(), wr.size());
    EXPECT_NE(err, SlaveError::Ok);
    master_.clearCancel();
}

// An object with a declared size of 0 makes the expedited response report
// total_size==0 → SDOUpload returns 0 bytes without entering the segment
// loop.
TEST_F(SegmentedWireTest, UploadZeroSizeObject) {
    Emulator::ODEntry e{};
    e.index = 0x7012;
    e.subindex = 0;
    e.data_type = 0x000F;
    e.bit_length = 0;
    e.access = 0x03;
    e.data = {};
    e.name = "empty";
    slave_->addODEntry(e);

    std::array<uint8_t, 16> buf{};
    size_t got = buf.size();
    const auto err = master_.slave(0).sdoRead(0x7012, 0, buf.data(), got);
    EXPECT_EQ(err, SlaveError::Ok);
    EXPECT_LE(got, buf.size());
}

// Enabling the CoE RX-packet debug flag + a gate condition exercises the
// logCoeMbxPacket / onCoERead hook calls inside the segment loop.
TEST_F(SegmentedWireTest, UploadWithCoeRxPacketDebugAndGateCondition) {
    addBigObject(0x7013, 400);
    master_.debugFlags().coeRxPackets = true;
    auto cond = DebugGate::parseCondition("checkpoint:seg-upload");
    ASSERT_NE(cond, nullptr);
    master_.debugGate().addGlobalStart(std::move(cond));

    std::vector<uint8_t> buf(400, 0);
    size_t got = buf.size();
    ASSERT_EQ(master_.slave(0).sdoRead(0x7013, 0, buf.data(), got),
              SlaveError::Ok);
    EXPECT_EQ(got, 400u);
}

// A mid-transfer mailbox stall (slave stops filling SM1) hits the
// pollSm1Full timeout / retry tail of the segment loop. inject_timeout
// drops the emulator's register responses so the SM1-full poll fails.
TEST_F(SegmentedWireTest, UploadMailboxStallTimesOut) {
    addBigObject(0x7014, 400);
    Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    inj.timeout_register = Raw::sm_status_address(1);
    slave_->setErrorInjection(inj);
    std::vector<uint8_t> buf(400, 0);
    size_t got = buf.size();
    const auto err = master_.slave(0).sdoRead(0x7014, 0, buf.data(), got);
    EXPECT_NE(err, SlaveError::Ok);
    slave_->setErrorInjection(Emulator::ErrorInjection{});
}

// Same stall on the segmented download path — first segment request
// lands, but the response wait times out.
TEST_F(SegmentedWireTest, DownloadMailboxStallTimesOut) {
    addBigObject(0x7015, 400);
    Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    inj.timeout_register = Raw::sm_status_address(1);
    slave_->setErrorInjection(inj);
    std::vector<uint8_t> wr(400, 0x22);
    const auto err = master_.slave(0).sdoWrite(0x7015, 0, wr.data(), wr.size());
    EXPECT_NE(err, SlaveError::Ok);
    slave_->setErrorInjection(Emulator::ErrorInjection{});
}
