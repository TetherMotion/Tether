/**
 * @file test_eeprom_reactor.cpp
 * @brief Comprehensive tests for the EEPROM reactor and state machine
 *
 * Tests cover:
 *   - State machine initialization and state transitions (single word-pair)
 *   - Datagram spec generation for each protocol step
 *   - Completion handling (success, timeout, error, NACK retry)
 *   - DONE state and reinit for next word-pair
 *   - Cache population on READ_EEPDAT
 *   - Reactor integration with a loopback network interface
 *   - Multi-slave concurrent reads
 *   - Cancellation and cleanup
 *   - Memory safety (slot cleanup on destruction)
 */

#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <vector>

#include "tether/sii/EEPROMReactor.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/LinuxDevNullNetworkInterface.hpp"
#include "tether/sii/SIIManager.hpp"
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT;
using namespace EtherCAT::SII;
using namespace EtherCAT::Raw;  // for le16_to_host, host_to_le16, etc.

// ============================================================================
// Helpers
// ============================================================================

/// Create a WaitResult simulating a successful response.
static WaitResult makeSuccess(uint16_t wkc, uint16_t data_len, uint8_t idx) {
    return WaitResult::Success(wkc, data_len, Command::APRD, 0, 0, idx);
}

/// Create a WaitResult simulating a timeout/failure.
static WaitResult makeFailure() {
    WaitResult r{};
    r.timeout = true;
    return r;
}

// ============================================================================
// State Machine Unit Tests
// ============================================================================
//
// The state machine reads a SINGLE word-pair per init/reinit cycle.
// After DONE, the reactor calls reinit() for the next word-pair.

class EEPROMStateMachineTest : public ::testing::Test {
protected:
    EEPROMReadStateMachine sm;
};

TEST_F(EEPROMStateMachineTest, Init_StartsAtWriteEepAddr) {
    sm.init(0, {0x0040});
    EXPECT_EQ(sm.state(), EEPROMState::WRITE_EEPADDR);
    EXPECT_FALSE(sm.isFinished());
    EXPECT_TRUE(sm.needsSend());
    EXPECT_EQ(sm.slaveIndex(), 0u);
    EXPECT_EQ(sm.currentWord(), 0x0040u);
    EXPECT_EQ(sm.totalWords(), 1u);
    EXPECT_EQ(sm.wordsRead(), 0u);
    EXPECT_EQ(sm.remaining(), 1u);
}

TEST_F(EEPROMStateMachineTest, Init_EmptyVector_IsDone) {
    sm.init(0, {});
    EXPECT_EQ(sm.state(), EEPROMState::DONE);
    EXPECT_TRUE(sm.isFinished());
    EXPECT_FALSE(sm.needsSend());
    EXPECT_EQ(sm.totalWords(), 0u);
}

TEST_F(EEPROMStateMachineTest, Init_AlignsWordToEven) {
    sm.init(0, {0x0041});  // Odd address
    // currentWord() returns the raw vector value; alignment happens
    // in buildDatagram() via & 0xFFFEu.
    EXPECT_EQ(sm.currentWord(), 0x0041u);
    auto spec = sm.buildDatagram(1);
    ASSERT_EQ(spec.cmd, Command::APWR);
    ASSERT_EQ(spec.ado, EEPROMProtocol::REG_EEPADDR);
    ASSERT_EQ(spec.datalen, 2u);
    uint16_t addr = le16_to_host(
        *reinterpret_cast<const uint16_t*>(spec.data));
    EXPECT_EQ(addr, 0x0040u);  // Aligned to even in the datagram
}

TEST_F(EEPROMStateMachineTest, BuildDatagram_WriteEepAddr) {
    sm.init(2, {0x0080});
    auto spec = sm.buildDatagram(42);
    EXPECT_EQ(spec.cmd, Command::APWR);
    EXPECT_EQ(spec.idx, 42);
    EXPECT_EQ(spec.ado, EEPROMProtocol::REG_EEPADDR);
    EXPECT_EQ(spec.datalen, 2u);
    EXPECT_NE(spec.data, nullptr);
    EXPECT_TRUE(spec.roundtrip);
}

TEST_F(EEPROMStateMachineTest, BuildDatagram_WriteEepCtlRead) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    ASSERT_EQ(sm.state(), EEPROMState::WRITE_EEPCTL_READ);

    auto spec = sm.buildDatagram(2);
    EXPECT_EQ(spec.cmd, Command::APWR);
    EXPECT_EQ(spec.idx, 2);
    EXPECT_EQ(spec.ado, EEPROMProtocol::REG_EEPCTL);
    EXPECT_EQ(spec.datalen, 2u);
    EXPECT_NE(spec.data, nullptr);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, BuildDatagram_PollEepStat) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    sm.buildDatagram(2);
    sm.onComplete(makeSuccess(1, 0, 2), master);
    ASSERT_EQ(sm.state(), EEPROMState::POLL_EEPSTAT);

    auto spec = sm.buildDatagram(3);
    EXPECT_EQ(spec.cmd, Command::APRD);
    EXPECT_EQ(spec.idx, 3);
    EXPECT_EQ(spec.ado, EEPROMProtocol::REG_EEPSTAT);
    EXPECT_EQ(spec.datalen, 2u);
    EXPECT_EQ(spec.data, nullptr);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, BuildDatagram_ReadEepDat) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    sm.buildDatagram(2);
    sm.onComplete(makeSuccess(1, 0, 2), master);
    sm.buildDatagram(3);
    sm.response().data[0] = 0;
    sm.response().data[1] = 0;
    sm.onComplete(makeSuccess(1, 2, 3), master);
    ASSERT_EQ(sm.state(), EEPROMState::READ_EEPDAT);

    auto spec = sm.buildDatagram(4);
    EXPECT_EQ(spec.cmd, Command::APRD);
    EXPECT_EQ(spec.idx, 4);
    EXPECT_EQ(spec.ado, EEPROMProtocol::REG_EEPDAT);
    EXPECT_EQ(spec.datalen, 4u);
    EXPECT_EQ(spec.data, nullptr);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, FullProtocolCycle_ReadsOneWordPair) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    ASSERT_EQ(sm.state(), EEPROMState::WRITE_EEPCTL_READ);

    sm.buildDatagram(2);
    sm.onComplete(makeSuccess(1, 0, 2), master);
    ASSERT_EQ(sm.state(), EEPROMState::POLL_EEPSTAT);

    sm.buildDatagram(3);
    sm.response().data[0] = 0;
    sm.response().data[1] = 0;
    sm.onComplete(makeSuccess(1, 2, 3), master);
    ASSERT_EQ(sm.state(), EEPROMState::READ_EEPDAT);

    sm.buildDatagram(4);
    uint32_t data = 0xDEADBEEF;
    std::memcpy(sm.response().data, &data, 4);
    sm.onComplete(makeSuccess(1, 4, 4), master);

    EXPECT_EQ(sm.state(), EEPROMState::DONE);
    EXPECT_TRUE(sm.isFinished());
    EXPECT_EQ(sm.wordsRead(), 1u);

    // Verify cache was populated
    uint32_t cached = 0;
    EXPECT_TRUE(master.slave(0).sii().cache().getWordPair(0x0040, cached));
    EXPECT_EQ(cached & 0xFFFF, 0xBEEFu);
    EXPECT_EQ((cached >> 16) & 0xFFFF, 0xDEADu);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, MultipleWordPairs_AdvancesCorrectly) {
    sm.init(0, {0x0040, 0x0042, 0x0044});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(sm.state(), EEPROMState::WRITE_EEPADDR);
        sm.buildDatagram(static_cast<uint8_t>(i * 4 + 1));
        sm.onComplete(makeSuccess(1, 0, static_cast<uint8_t>(i * 4 + 1)), master);

        ASSERT_EQ(sm.state(), EEPROMState::WRITE_EEPCTL_READ);
        sm.buildDatagram(static_cast<uint8_t>(i * 4 + 2));
        sm.onComplete(makeSuccess(1, 0, static_cast<uint8_t>(i * 4 + 2)), master);

        ASSERT_EQ(sm.state(), EEPROMState::POLL_EEPSTAT);
        sm.buildDatagram(static_cast<uint8_t>(i * 4 + 3));
        sm.response().data[0] = 0;
        sm.response().data[1] = 0;
        sm.onComplete(makeSuccess(1, 2, static_cast<uint8_t>(i * 4 + 3)), master);

        ASSERT_EQ(sm.state(), EEPROMState::READ_EEPDAT);
        sm.buildDatagram(static_cast<uint8_t>(i * 4 + 4));
        uint32_t data = static_cast<uint32_t>(0x1000 + i);
        std::memcpy(sm.response().data, &data, 4);
        sm.onComplete(makeSuccess(1, 4, static_cast<uint8_t>(i * 4 + 4)), master);

        if (i < 2) {
            EXPECT_EQ(sm.state(), EEPROMState::WRITE_EEPADDR);
        }
    }
    EXPECT_EQ(sm.state(), EEPROMState::DONE);
    EXPECT_EQ(sm.wordsRead(), 3u);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, PollBusy_RetriesPoll) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    sm.buildDatagram(2);
    sm.onComplete(makeSuccess(1, 0, 2), master);
    ASSERT_EQ(sm.state(), EEPROMState::POLL_EEPSTAT);

    // First poll: busy
    sm.buildDatagram(3);
    sm.response().data[0] = static_cast<uint8_t>(EEPROMProtocol::ESTAT_BUSY & 0xFF);
    sm.response().data[1] = static_cast<uint8_t>((EEPROMProtocol::ESTAT_BUSY >> 8) & 0xFF);
    sm.onComplete(makeSuccess(1, 2, 3), master);
    EXPECT_EQ(sm.state(), EEPROMState::POLL_EEPSTAT);

    // Second poll: not busy
    sm.buildDatagram(4);
    sm.response().data[0] = 0;
    sm.response().data[1] = 0;
    sm.onComplete(makeSuccess(1, 2, 4), master);
    EXPECT_EQ(sm.state(), EEPROMState::READ_EEPDAT);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, PollBusyTooManyTimes_Fails) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    sm.buildDatagram(2);
    sm.onComplete(makeSuccess(1, 0, 2), master);

    for (int i = 0; i < EEPROMProtocol::MAX_BUSY_POLLS; ++i) {
        if (sm.state() != EEPROMState::POLL_EEPSTAT) break;
        sm.buildDatagram(static_cast<uint8_t>(i + 3));
        sm.response().data[0] = static_cast<uint8_t>(EEPROMProtocol::ESTAT_BUSY & 0xFF);
        sm.response().data[1] = static_cast<uint8_t>((EEPROMProtocol::ESTAT_BUSY >> 8) & 0xFF);
        sm.onComplete(makeSuccess(1, 2, static_cast<uint8_t>(i + 3)), master);
    }
    EXPECT_EQ(sm.state(), EEPROMState::FAILED);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, NackError_RetriesFromWriteEepAddr) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    sm.buildDatagram(2);
    sm.onComplete(makeSuccess(1, 0, 2), master);

    sm.buildDatagram(3);
    sm.response().data[0] = static_cast<uint8_t>(EEPROMProtocol::ESTAT_NACK & 0xFF);
    sm.response().data[1] = static_cast<uint8_t>((EEPROMProtocol::ESTAT_NACK >> 8) & 0xFF);
    sm.onComplete(makeSuccess(1, 2, 3), master);

    EXPECT_EQ(sm.state(), EEPROMState::WRITE_EEPADDR);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, NackTooManyRetries_Fails) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    for (int nack = 0; nack < EEPROMProtocol::MAX_NACK_RETRIES; ++nack) {
        if (sm.state() != EEPROMState::WRITE_EEPADDR) break;
        sm.buildDatagram(static_cast<uint8_t>(nack + 1));
        sm.onComplete(makeSuccess(1, 0, static_cast<uint8_t>(nack + 1)), master);
        if (sm.state() != EEPROMState::WRITE_EEPCTL_READ) break;
        sm.buildDatagram(static_cast<uint8_t>(nack + 10));
        sm.onComplete(makeSuccess(1, 0, static_cast<uint8_t>(nack + 10)), master);
        if (sm.state() != EEPROMState::POLL_EEPSTAT) break;
        sm.buildDatagram(static_cast<uint8_t>(nack + 20));
        sm.response().data[0] = static_cast<uint8_t>(EEPROMProtocol::ESTAT_NACK & 0xFF);
        sm.response().data[1] = static_cast<uint8_t>((EEPROMProtocol::ESTAT_NACK >> 8) & 0xFF);
        sm.onComplete(makeSuccess(1, 2, static_cast<uint8_t>(nack + 20)), master);
    }
    EXPECT_EQ(sm.state(), EEPROMState::FAILED);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, NonNackError_Fails) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeSuccess(1, 0, 1), master);
    sm.buildDatagram(2);
    sm.onComplete(makeSuccess(1, 0, 2), master);

    sm.buildDatagram(3);
    sm.response().data[0] = static_cast<uint8_t>(EEPROMProtocol::ESTAT_CRC_ERR & 0xFF);
    sm.response().data[1] = static_cast<uint8_t>((EEPROMProtocol::ESTAT_CRC_ERR >> 8) & 0xFF);
    sm.onComplete(makeSuccess(1, 2, 3), master);

    EXPECT_EQ(sm.state(), EEPROMState::FAILED);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, TimeoutOnAnyStep_Fails) {
    sm.init(0, {0x0040});
    Master master;
    master.packetRouter().init();
    master.initSlaves(1);

    sm.buildDatagram(1);
    sm.onComplete(makeFailure(), master);
    EXPECT_EQ(sm.state(), EEPROMState::FAILED);

    master.packetRouter().shutdown();
}

TEST_F(EEPROMStateMachineTest, Cancel_TransitionsToFailed) {
    sm.init(0, {0x0040});
    sm.cancel();
    EXPECT_EQ(sm.state(), EEPROMState::FAILED);
    EXPECT_TRUE(sm.isFinished());
}

TEST_F(EEPROMStateMachineTest, Cancel_OnAlreadyDone_NoChange) {
    sm.init(0, {});
    ASSERT_EQ(sm.state(), EEPROMState::DONE);
    sm.cancel();
    EXPECT_EQ(sm.state(), EEPROMState::DONE);
}

TEST_F(EEPROMStateMachineTest, InFlightFlag) {
    sm.init(0, {0x0040});
    EXPECT_FALSE(sm.isInFlight());
    sm.markInFlight();
    EXPECT_TRUE(sm.isInFlight());
    EXPECT_FALSE(sm.needsSend());
}

TEST_F(EEPROMStateMachineTest, Reset_ClearsToIdle) {
    sm.init(0, {0x0040, 0x0042});
    sm.reset();
    EXPECT_EQ(sm.state(), EEPROMState::IDLE);
    EXPECT_EQ(sm.totalWords(), 0u);
    EXPECT_FALSE(sm.isFinished());  // IDLE is not DONE or FAILED
}

// ============================================================================
// Reactor Integration Tests (with dev-null network)
// ============================================================================

class EEPROMReactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        master_.start(devnull_.iface(), mac_);
    }

    void TearDown() override {
        master_.packetRouter().shutdown();
    }

    LinuxDevNullNetworkInterface devnull_;
    Master master_;
    uint8_t mac_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
};

TEST_F(EEPROMReactorTest, EmptyReactor_ReturnsTrue) {
    EEPROMReactor reactor(master_);
    EXPECT_TRUE(reactor.run(100));
    EXPECT_EQ(reactor.successCount(), 0u);
    EXPECT_EQ(reactor.failureCount(), 0u);
}

TEST_F(EEPROMReactorTest, AddSlave_IncreasesSlaveCount) {
    EEPROMReactor reactor(master_);
    EXPECT_EQ(reactor.slaveCount(), 0u);
    reactor.addSlave(0, CAT_MASK_ALL);
    EXPECT_EQ(reactor.slaveCount(), 1u);
    reactor.addSlave(1, CAT_MASK_ALL);
    EXPECT_EQ(reactor.slaveCount(), 2u);
}

TEST_F(EEPROMReactorTest, StateMachineAccessible_AfterAdd) {
    EEPROMReactor reactor(master_);
    reactor.addSlave(3, CAT_MASK_ALL);
    ASSERT_EQ(reactor.slaveCount(), 1u);
    const auto& sm = reactor.stateMachine(0);
    EXPECT_EQ(sm.slaveIndex(), 3u);
    // SM is initialized with an empty vector (DONE) until run() fills it
    EXPECT_EQ(sm.state(), EEPROMState::DONE);
}

TEST_F(EEPROMReactorTest, Destructor_CancelsPendingSlots) {
    master_.initSlaves(2);

    {
        EEPROMReactor reactor(master_);
        reactor.addSlave(0, CAT_MASK_ALL);
        reactor.addSlave(1, CAT_MASK_ALL);
        // Don't call run() — just destruct.
    }
    SUCCEED();
}

TEST_F(EEPROMReactorTest, SingleSlave_NoResponses_TimesOut) {
    master_.initSlaves(1);

    EEPROMReactor reactor(master_);
    reactor.addSlave(0, CAT_MASK_ALL);

    bool ok = reactor.run(50);
    // Without responses, all slaves should fail.
    EXPECT_FALSE(ok);
    EXPECT_EQ(reactor.failureCount(), 1u);
    EXPECT_EQ(reactor.successCount(), 0u);
}

// ============================================================================
// Reactor with loopback network interface
// ============================================================================

class LoopbackNetworkInterface {
public:
    LoopbackNetworkInterface() {
        iface_.send = [this](const uint8_t* data, size_t len) -> bool {
            tx_count_.fetch_add(1, std::memory_order_relaxed);
            return true;
        };
        iface_.receive = [](uint8_t*, size_t, size_t* out_len) -> bool {
            if (out_len) *out_len = 0;
            return false;
        };
    }

    NetworkInterface& iface() { return iface_; }
    uint64_t txCount() const { return tx_count_.load(std::memory_order_relaxed); }

private:
    NetworkInterface iface_{};
    std::atomic<uint64_t> tx_count_{0};
};

class EEPROMReactorLoopbackTest : public ::testing::Test {
protected:
    void SetUp() override {
        master_.start(loopback_.iface(), mac_);
        master_.initSlaves(2);
    }

    void TearDown() override {
        master_.packetRouter().shutdown();
    }

    LoopbackNetworkInterface loopback_;
    Master master_;
    uint8_t mac_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
};

TEST_F(EEPROMReactorLoopbackTest, MultiSlave_Timeout_AllFail) {
    EEPROMReactor reactor(master_);
    reactor.addSlave(0, CAT_MASK_ALL);
    reactor.addSlave(1, CAT_MASK_ALL);

    bool ok = reactor.run(50);
    EXPECT_FALSE(ok);
    EXPECT_EQ(reactor.failureCount(), 2u);
    EXPECT_EQ(reactor.successCount(), 0u);
}

TEST_F(EEPROMReactorLoopbackTest, MultiSlave_FramesSent) {
    EEPROMReactor reactor(master_);
    reactor.addSlave(0, CAT_MASK_ALL);
    reactor.addSlave(1, CAT_MASK_ALL);

    uint64_t before = loopback_.txCount();
    reactor.run(50);
    uint64_t after = loopback_.txCount();

    // Should have sent at least one frame per slave
    EXPECT_GT(after, before);
}
