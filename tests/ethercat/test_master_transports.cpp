/**
 * @file test_master_transports.cpp
 * @brief Coverage for the Master→interface transport adapters
 *        (src/ethercat/raw/MasterTransports.cpp): every forwarded method
 *        of MasterPDOTransport, MasterFaultTransport and MasterSDOTransport
 *        over an emulated wire.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "mocks/EmulatedBus.hpp"
#include "raw/MasterTransports.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/CoeSDOChannel.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/IPDOTransport.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;

namespace {

class MasterTransportsTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0xBEEF,
                                                 "transporttest");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slave_ = nullptr;
    Master master_;
};

// ============================================================================
// MasterPDOTransport — register/datagram surface
// ============================================================================

TEST_F(MasterTransportsTest, PdoTransportRegisterRoundTrips) {
    auto t = makeMasterPDOTransport(master_);

    uint8_t out = 0;
    EXPECT_TRUE(t->readRegister(0, 0x0130, &out, 1, 100));

    // 0x0F00 is plain ESC RAM — stored verbatim by the emulator
    // (0x0120 is AL_CONTROL: writes request state transitions).
    uint16_t wval = 0;
    EXPECT_TRUE(t->writeRegister(0, 0x0F00, "\x34\x12", 2, 100));
    EXPECT_TRUE(t->readRegister(0, 0x0F00, &wval, 2, 100));
    EXPECT_EQ(wval, 0x1234u);

    EXPECT_EQ(t->adpForSlaveIndex(0), 0x0000u);
    EXPECT_EQ(t->adpForSlaveIndex(1), 0xFFFFu);
}

TEST_F(MasterTransportsTest, PdoTransportSingleAndMultiDatagrams) {
    auto t = makeMasterPDOTransport(master_);

    uint8_t idx = t->allocIdx();
    uint8_t buf[4] = {};
    EXPECT_TRUE(t->sendSingleDatagram(Command::APRD, idx, 0, 0x0130,
                                      buf, sizeof(buf), true));

    RxDatagram rx{};
    EXPECT_TRUE(t->waitForResponseIdx(idx, 200, rx));
    EXPECT_GE(rx.wkc, 1u);

    MultiDatagramSpec specs[2]{};
    uint8_t d0[2]{}, d1[2]{};
    specs[0] = {Command::APRD, t->allocIdx(), 0, 0x0130, d0, 2, true};
    specs[1] = {Command::APRD, t->allocIdx(), 0, 0x0132, d1, 2, true};
    // Return value is frames sent — both datagrams pack into one frame.
    EXPECT_EQ(t->sendMultiDatagram(specs, 2), 1u);
}

TEST_F(MasterTransportsTest, PdoTransportPreRegisteredWaiter) {
    auto t = makeMasterPDOTransport(master_);

    const uint8_t idx = t->allocIdx();
    uint8_t resp[64]{};
    const size_t slot = t->preRegisterResponseWaiter(idx, resp, sizeof(resp));
    ASSERT_NE(slot, IPDOTransport::kPreRegInvalid);
    ASSERT_NE(slot, IPDOTransport::kPreRegBusy);

    uint8_t data[2]{};
    EXPECT_TRUE(t->sendSingleDatagram(Command::APRD, idx, 0, 0x0130,
                                      data, sizeof(data), false));

    RxDatagram rx{};
    EXPECT_TRUE(t->waitForPreRegistered(slot, 200, rx));
    EXPECT_EQ(rx.idx, idx);

    // Second claim on a different idx works; cancelling is harmless.
    const uint8_t idx2 = t->allocIdx();
    const size_t slot2 = t->preRegisterResponseWaiter(idx2, resp, sizeof(resp));
    ASSERT_NE(slot2, IPDOTransport::kPreRegInvalid);
    t->cancelPreRegistered(slot2);
}

TEST_F(MasterTransportsTest, PdoTransportMaintenanceAndInfo) {
    auto t = makeMasterPDOTransport(master_);

    EXPECT_GE(t->drainWire(4), 0);
    t->purgePendingResponses();
    EXPECT_NO_THROW(t->txFailureDiagnostics());
    EXPECT_GE(t->lastSendErrno(), -1);
    EXPECT_FALSE(t->isCancelRequested());
    EXPECT_GT(t->maxEtherCATPayloadPerFrame(), 0u);
}

// ============================================================================
// MasterPDOTransport — cyclic + slice fast-path forwarders
// ============================================================================

TEST_F(MasterTransportsTest, PdoTransportCyclicSurface) {
    auto t = makeMasterPDOTransport(master_);

    (void)t->supportsCyclicFastPath();
    (void)t->cyclicSlotToken(0);
    (void)t->cyclicSlotGen(0);

    uint8_t data[4]{};
    (void)t->sendCyclicDatagram(Command::APRD, 0, 0, 0x0130,
                                data, sizeof(data), true);

    RxDatagram rx{};
    (void)t->waitCyclicSlot(0, 0, 1000, rx);

    CyclicSlotView view{};
    (void)t->waitCyclicSlotView(0, 0, 1000, view);
    CyclicSlotView views[2]{};
    uint64_t tokens[2]{};
    (void)t->waitCyclicSlotMask(0x3, tokens, 1000, views);

    uint8_t* frame = t->acquireCyclicTxFrame();
    if (frame) {
        t->composeCyclicHeader(frame, Command::APRD, 0, 0, 0x0130, 4, true);
        (void)t->sendCyclicFrame(64);
    }
    (void)t->cyclicChannel();
}

TEST_F(MasterTransportsTest, PdoTransportSliceSurface) {
    auto t = makeMasterPDOTransport(master_);

    (void)t->sliceSlotToken(0);
    (void)t->sliceSlotGen(0);

    const int slot = t->acquireSliceSlot();
    if (slot >= 0) t->releaseSliceSlot(static_cast<uint8_t>(slot));

    uint8_t data[4]{};
    (void)t->sendSliceDatagram(Command::APRD, 0, 0, 0x0130,
                               data, sizeof(data), true);

    CyclicSlotView view{};
    (void)t->waitSliceSlotView(0, 0, 1000, view);
    CyclicSlotView views[2]{};
    uint64_t tokens[2]{};
    (void)t->waitSliceSlotMask(0x3, tokens, 1000, views);
    (void)t->cyclicPayloadOffset();

    CyclicDgramSpec dgs[1]{};
    uint8_t payload[4]{};
    dgs[0].data = payload;
    dgs[0].datalen = sizeof(payload);
    (void)t->sendPoolFrame(dgs, 1);

    uint8_t positions[1]{0};
    bool arrived[1]{false};
    (void)t->waitCyclicPool(positions, tokens, 1, 1000, views, arrived);
}

// ============================================================================
// MasterFaultTransport
// ============================================================================

TEST_F(MasterTransportsTest, FaultTransportSurface) {
    auto t = makeMasterFaultTransport(master_);

    uint8_t out = 0;
    EXPECT_TRUE(t->readRegister(0, 0x0130, &out, 1));
    uint8_t w = 0x55;
    EXPECT_TRUE(t->writeRegister(0, 0x0F00, &w, 1));

    EXPECT_GT(t->getTimestampMs(), 0u);
    t->delayMs(0);
}

// ============================================================================
// MasterSDOTransport
// ============================================================================

TEST_F(MasterTransportsTest, SdoTransportSurface) {
    auto t = makeMasterSDOTransport(master_);

    uint8_t reg = 0;
    EXPECT_TRUE(t->readSlaveRegister(0, 0x0130, &reg, 1, 100));
    EXPECT_GT(t->getMicroseconds(), 0u);
    EXPECT_EQ(t->lastAbortCode(), 0u);
    EXPECT_FALSE(t->isCancelRequested());
}

TEST_F(MasterTransportsTest, SdoTransportUploadDownload) {
    master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    auto t = makeMasterSDOTransport(master_);

    // Upload the device-type object (0x1000:00) via the adapter.
    uint8_t counter = 0;
    uint8_t buf[8]{};
    size_t got = 0;
    EXPECT_TRUE(t->sdoUpload(0, &counter, 0x1000, 128, 0x1080, 128,
                             0x1000, 0, buf, sizeof(buf), &got));
    EXPECT_GT(got, 0u);

    // Download to a writable CiA402 object (controlword 0x6040:00).
    uint8_t cw[2] = {0x06, 0x00};
    EXPECT_TRUE(t->sdoDownload(0, &counter, 0x1000, 128, 0x1080, 128,
                               0x6040, 0, cw, sizeof(cw)));
}

}  // namespace
