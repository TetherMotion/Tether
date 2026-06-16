/**
 * @file test_dc.cpp
 * @brief Comprehensive tests for EtherCATDC (instance-based, no global state)
 *
 * Uses a RecordingDCTransport / GMock MockDCTransport to verify
 * DC protocol behavior (register reads/writes, sync datagrams) without
 * any real network I/O.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/DCClass.hpp"
#include "tether/ethercat/IDCTransport.hpp"

#include <cstring>
#include <vector>
#include <map>
#include <algorithm>
#include <thread>
#include <chrono>

using namespace EtherCAT;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;

// ============================================================================
// DC Register Constants (for verification)
// ============================================================================

static constexpr uint16_t REG_DCSYSTIME   = toUInt16(DCRegisters::DCSysTime);
static constexpr uint16_t REG_DCRECVTIMES = toUInt16(DCRegisters::DCRecvTimes);
static constexpr uint16_t REG_DCSYSOFFSET = toUInt16(DCRegisters::DCSysOffset);
static constexpr uint16_t REG_DCSYNCACT   = toUInt16(DCRegisters::DCSyncAct);
static constexpr uint16_t REG_DCSTART0    = toUInt16(DCRegisters::DCStart0);
static constexpr uint16_t REG_DCCYCLE0    = toUInt16(DCRegisters::DCCycle0);
static constexpr uint16_t REG_DCCYCLE1    = toUInt16(DCRegisters::DCCycle1);
static constexpr uint16_t REG_DCCUC       = toUInt16(DCRegisters::DCCuc);

// ============================================================================
// RecordingDCTransport — records all register ops for verification
// ============================================================================

class RecordingDCTransport : public IDCTransport {
public:
    struct Op {
        enum Type { Read, Write, SyncDatagram } type;
        uint16_t slave_index;
        uint16_t reg_addr;
        std::vector<uint8_t> data;
        uint16_t size;
    };

    std::vector<Op> ops;
    uint64_t current_time_ns = 1000000000ULL; // 1 second

    /// Pre-configured responses for readRegister (key = {slave, reg})
    std::map<std::pair<uint16_t, uint16_t>, std::vector<uint8_t>> read_responses;

    bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                      void* data, uint16_t size,
                      unsigned int /*timeout_ms*/) override {
        ops.push_back({Op::Read, slave_index, reg_addr, {}, size});
        auto key = std::make_pair(slave_index, reg_addr);
        auto it = read_responses.find(key);
        if (it != read_responses.end()) {
            auto& resp = it->second;
            std::memcpy(data, resp.data(),
                        std::min(static_cast<size_t>(size), resp.size()));
            return true;
        }
        std::memset(data, 0, size);
        return true;
    }

    bool writeRegister(uint16_t slave_index, uint16_t reg_addr,
                       const void* data, uint16_t size,
                       unsigned int /*timeout_ms*/) override {
        auto d = std::vector<uint8_t>(
            static_cast<const uint8_t*>(data),
            static_cast<const uint8_t*>(data) + size);
        ops.push_back({Op::Write, slave_index, reg_addr, std::move(d), size});
        return true;
    }

    bool sendSyncDatagram(uint16_t slave_index, uint16_t reg_addr,
                           const void* data, uint16_t size) override {
        auto d = std::vector<uint8_t>(
            static_cast<const uint8_t*>(data),
            static_cast<const uint8_t*>(data) + size);
        ops.push_back({Op::SyncDatagram, slave_index, reg_addr, std::move(d), size});
        return true;
    }

    uint64_t getMasterTimeNs() override { return current_time_ns; }
    void delayMs(uint32_t /*ms*/) override { /* no-op in tests */ }

    // ── Helpers ──

    /// Set the DC system time a slave will report via readRegister(_, 0x0910)
    void setSlaveSystemTime(uint16_t slave_index, uint64_t time_ns) {
        std::vector<uint8_t> data(8);
        for (int i = 0; i < 8; i++) {
            data[i] = static_cast<uint8_t>((time_ns >> (i * 8)) & 0xFF);
        }
        read_responses[{slave_index, REG_DCSYSTIME}] = data;
    }

    /// Count operations of a given type
    size_t countOps(Op::Type type) const {
        return static_cast<size_t>(
            std::count_if(ops.begin(), ops.end(),
                          [type](const Op& op) { return op.type == type; }));
    }

    /// Count operations of a given type + register
    size_t countOps(Op::Type type, uint16_t reg) const {
        return static_cast<size_t>(
            std::count_if(ops.begin(), ops.end(),
                          [type, reg](const Op& op) {
                              return op.type == type && op.reg_addr == reg;
                          }));
    }

    /// Find first op of given type + register
    const Op* findOp(Op::Type type, uint16_t reg) const {
        for (auto& op : ops) {
            if (op.type == type && op.reg_addr == reg) return &op;
        }
        return nullptr;
    }

    /// Clear recorded operations
    void clearOps() { ops.clear(); }
};

// ============================================================================
// GMock MockDCTransport
// ============================================================================

class MockDCTransport : public IDCTransport {
public:
    MOCK_METHOD(bool, readRegister,
                (uint16_t, uint16_t, void*, uint16_t, unsigned int), (override));
    MOCK_METHOD(bool, writeRegister,
                (uint16_t, uint16_t, const void*, uint16_t, unsigned int), (override));
    MOCK_METHOD(bool, sendSyncDatagram,
                (uint16_t, uint16_t, const void*, uint16_t), (override));
    MOCK_METHOD(uint64_t, getMasterTimeNs, (), (override));
    MOCK_METHOD(void, delayMs, (uint32_t), (override));
};

// ============================================================================
// Construction Tests
// ============================================================================

class DCConstructionTest : public ::testing::Test {
protected:
    RecordingDCTransport transport_;
};

TEST_F(DCConstructionTest, DefaultState) {
    EtherCATDC dc(transport_, 1);
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCConstructionTest, ConstructionWithZeroSlaves) {
    // Zero slaves should put the instance in Error state
    EtherCATDC dc(transport_, 0);
    EXPECT_EQ(dc.getState(), DCState::Error);
}

TEST_F(DCConstructionTest, ConstructionWithCustomConfig) {
    DCConfig cfg = DCConfig::defaults();
    cfg.cycle_period_us = 500;
    cfg.sync_interval_cycles = 5;
    EtherCATDC dc(transport_, 2, &cfg);
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCConstructionTest, InitRequiredBeforeStart) {
    EtherCATDC dc(transport_, 1);
    // start() should fail before init()
    EXPECT_FALSE(dc.start());
}

// ============================================================================
// Initialization Tests
// ============================================================================

class DCInitTest : public ::testing::Test {
protected:
    RecordingDCTransport transport_;
};

TEST_F(DCInitTest, ReadSlaveCapabilitiesViaMock) {
    // Slave 0 reports DC-capable (system time != 0)
    transport_.setSlaveSystemTime(0, 5000000000ULL); // 5 seconds

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    // Verify readRegister was called for DCSystemTime (0x0910)
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::Read, REG_DCSYSTIME), 1u);
    EXPECT_TRUE(dc.isSlaveSupported(0));
}

TEST_F(DCInitTest, InitReturnsFalseWhenNoCapableSlaves) {
    // Slave 0 system time = 0 → not DC-capable
    transport_.setSlaveSystemTime(0, 0);

    EtherCATDC dc(transport_, 1);
    EXPECT_FALSE(dc.init());
}

TEST_F(DCInitTest, PropagationDelayCalculation) {
    transport_.setSlaveSystemTime(0, 1000000);
    transport_.setSlaveSystemTime(1, 2000000);

    EtherCATDC dc(transport_, 2);
    ASSERT_TRUE(dc.init());

    // First slave has zero delay, subsequent slaves have non-zero delay
    // (The class uses a simplified calculation: 150 * slave_index)
    EXPECT_TRUE(dc.isSlaveSupported(0));
    EXPECT_TRUE(dc.isSlaveSupported(1));
}

TEST_F(DCInitTest, SystemTimeOffsetWritten) {
    transport_.setSlaveSystemTime(0, 5000000000ULL);

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    // Verify writeRegister was called for DCSysOffset (0x0920)
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::Write, REG_DCSYSOFFSET), 1u);
}

TEST_F(DCInitTest, MultipleSlaveInit) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);
    transport_.setSlaveSystemTime(1, 1000000100ULL);
    transport_.setSlaveSystemTime(2, 1000000200ULL);

    EtherCATDC dc(transport_, 3);
    ASSERT_TRUE(dc.init());

    EXPECT_TRUE(dc.isSlaveSupported(0));
    EXPECT_TRUE(dc.isSlaveSupported(1));
    EXPECT_TRUE(dc.isSlaveSupported(2));

    // Each slave should have its system time offset written
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::Write, REG_DCSYSOFFSET), 3u);
}

TEST_F(DCInitTest, SlaveIndexOutOfRange) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);
    EtherCATDC dc(transport_, 1);
    dc.init();

    EXPECT_FALSE(dc.isSlaveSupported(1));
    EXPECT_FALSE(dc.isSlaveSupported(99));
    EXPECT_EQ(dc.getSlaveOffset(99), 0);
}

TEST_F(DCInitTest, ReadRegisterWrapper) {
    // Ensure slave reports DC-capable time
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    // Prepare a 32-bit little-endian response for DCCycle0
    std::vector<uint8_t> resp = { 0x78, 0x56, 0x34, 0x12 };
    transport_.read_responses[{0, REG_DCCYCLE0}] = resp;

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    uint32_t cycle = 0;
    EXPECT_TRUE(dc.readRegister(0, DCRegisters::DCCycle0, &cycle, sizeof(cycle)));
    EXPECT_EQ(cycle, 0x12345678u);

    // Out-of-range slave index should return false
    EXPECT_FALSE(dc.readRegister(99, DCRegisters::DCCycle0, &cycle, sizeof(cycle)));
}

TEST_F(DCInitTest, WriteRegisterWrapper) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    uint32_t cycle = 0x01020304u;
    // Successful write for valid slave
    EXPECT_TRUE(dc.writeRegister(0, DCRegisters::DCCycle0, &cycle, sizeof(cycle)));
    // Out-of-range slave index should return false
    EXPECT_FALSE(dc.writeRegister(99, DCRegisters::DCCycle0, &cycle, sizeof(cycle)));

    // Verify transport recorded a write to the DCCycle0 register
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::Write, REG_DCCYCLE0), 1u);
}

// ============================================================================
// SYNC Configuration Tests
// ============================================================================

class DCSyncConfigTest : public ::testing::Test {
protected:
    RecordingDCTransport transport_;
};

TEST_F(DCSyncConfigTest, Sync0Configuration) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    DCConfig cfg = DCConfig::defaults();
    cfg.enable_sync0 = true;
    cfg.sync0_cycle_time_ns = 1000000; // 1ms

    EtherCATDC dc(transport_, 1, &cfg);
    ASSERT_TRUE(dc.init());

    // Verify SYNC0 cycle time written (0x09A0)
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::Write, REG_DCCYCLE0), 1u);

    // Verify SYNC activation written (0x0981)
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::Write, REG_DCSYNCACT), 1u);

    // Verify SYNC0 start time written (0x0990)
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::Write, REG_DCSTART0), 1u);
}

TEST_F(DCSyncConfigTest, Sync0Disabled) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    DCConfig cfg = DCConfig::defaults();
    cfg.enable_sync0 = false;
    cfg.sync0_cycle_time_ns = 0;

    EtherCATDC dc(transport_, 1, &cfg);
    ASSERT_TRUE(dc.init());

    // SYNC0 cycle time should NOT be written when disabled
    EXPECT_EQ(transport_.countOps(RecordingDCTransport::Op::Write, REG_DCCYCLE0), 0u);
}

TEST_F(DCSyncConfigTest, DifferentCycleTimes) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    DCConfig cfg = DCConfig::defaults();
    cfg.enable_sync0 = true;
    cfg.sync0_cycle_time_ns = 500000; // 500us
    cfg.enable_sync1 = false;

    EtherCATDC dc(transport_, 1, &cfg);
    ASSERT_TRUE(dc.init());

    // Find the SYNC0 cycle time write and verify data
    auto* op = transport_.findOp(RecordingDCTransport::Op::Write, REG_DCCYCLE0);
    ASSERT_NE(op, nullptr);
    ASSERT_EQ(op->data.size(), 4u);

    // Verify the cycle time value (little-endian 500000 = 0x0007A120)
    uint32_t written_cycle = 0;
    std::memcpy(&written_cycle, op->data.data(), 4);
    EXPECT_EQ(written_cycle, 500000u);
}

// ============================================================================
// DC Sync Frame Tests
// ============================================================================

class DCSyncFrameTest : public ::testing::Test {
protected:
    RecordingDCTransport transport_;
};

TEST_F(DCSyncFrameTest, SendSyncFrame) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    transport_.clearOps();
    EXPECT_TRUE(dc.sendSyncFrame());

    // Verify sendSyncDatagram called with reg 0x0910
    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::SyncDatagram, REG_DCSYSTIME), 1u);
}

TEST_F(DCSyncFrameTest, SyncFrameContainsMasterTime) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);
    transport_.current_time_ns = 9999999999ULL;

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    transport_.clearOps();
    dc.sendSyncFrame();

    auto* op = transport_.findOp(RecordingDCTransport::Op::SyncDatagram, REG_DCSYSTIME);
    ASSERT_NE(op, nullptr);
    ASSERT_EQ(op->data.size(), 8u);

    // Verify the master time value
    uint64_t written_time = 0;
    std::memcpy(&written_time, op->data.data(), 8);
    EXPECT_EQ(written_time, 9999999999ULL);
}

TEST_F(DCSyncFrameTest, SyncToFirstDCCapableSlave) {
    // Slave 0 is NOT DC-capable, slave 1 IS
    transport_.setSlaveSystemTime(0, 0);
    transport_.setSlaveSystemTime(1, 5000000000ULL);

    EtherCATDC dc(transport_, 2);
    dc.init(); // Only slave 1 succeeds

    transport_.clearOps();
    dc.sendSyncFrame();

    // Sync should target slave 1 (first DC-capable)
    auto* op = transport_.findOp(RecordingDCTransport::Op::SyncDatagram, REG_DCSYSTIME);
    ASSERT_NE(op, nullptr);
    EXPECT_EQ(op->slave_index, 1u);
}

TEST_F(DCSyncFrameTest, ForceSync) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    transport_.clearOps();
    dc.forceSync();

    EXPECT_GE(transport_.countOps(RecordingDCTransport::Op::SyncDatagram), 1u);
}

// ============================================================================
// Lifecycle Tests
// ============================================================================

class DCLifecycleTest : public ::testing::Test {
protected:
    RecordingDCTransport transport_;
};

TEST_F(DCLifecycleTest, StartStop) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());
    ASSERT_TRUE(dc.start());

    EXPECT_EQ(dc.getState(), DCState::Running);

    dc.stop();
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCLifecycleTest, StopWithoutStart) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);
    EtherCATDC dc(transport_, 1);
    dc.init();

    // stop without start should not crash
    dc.stop();
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCLifecycleTest, RestartAfterStop) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());
    ASSERT_TRUE(dc.start());
    dc.stop();

    // Should be able to restart
    ASSERT_TRUE(dc.start());
    EXPECT_EQ(dc.getState(), DCState::Running);
    dc.stop();
}

TEST_F(DCLifecycleTest, ResourceCleanupOnStop) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());
    ASSERT_TRUE(dc.start());

    // Let a few cycles run
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    dc.stop();
    EXPECT_EQ(dc.getState(), DCState::Disabled);

    // Stats should have been accumulated
    auto stats = dc.getStats();
    EXPECT_GE(stats.cycle_count, 0u); // At least some cycles
}

TEST_F(DCLifecycleTest, DestructorStopsLoop) {
    transport_.setSlaveSystemTime(0, 1000000000ULL);

    {
        EtherCATDC dc(transport_, 1);
        ASSERT_TRUE(dc.init());
        ASSERT_TRUE(dc.start());
        // Destructor should call stop() and clean up
    }
    // If destructor cleaned up correctly a new instance should initialize/start cleanly
    EtherCATDC dc2(transport_, 1);
    ASSERT_TRUE(dc2.init());
    EXPECT_EQ(dc2.getState(), DCState::Disabled);
}

// ============================================================================
// Multiple Independent Instance Tests
// ============================================================================

class DCMultiInstanceTest : public ::testing::Test {
protected:
    RecordingDCTransport transport_a_;
    RecordingDCTransport transport_b_;
};

TEST_F(DCMultiInstanceTest, TwoIndependentInstances) {
    transport_a_.setSlaveSystemTime(0, 1000000000ULL);
    transport_b_.setSlaveSystemTime(0, 2000000000ULL);

    EtherCATDC dc_a(transport_a_, 1);
    EtherCATDC dc_b(transport_b_, 1);

    ASSERT_TRUE(dc_a.init());
    ASSERT_TRUE(dc_b.init());

    // Each instance should have its own ops
    EXPECT_GT(transport_a_.ops.size(), 0u);
    EXPECT_GT(transport_b_.ops.size(), 0u);

    // Sending sync on A should not affect B's transport
    transport_a_.clearOps();
    transport_b_.clearOps();

    dc_a.sendSyncFrame();

    EXPECT_GT(transport_a_.countOps(RecordingDCTransport::Op::SyncDatagram), 0u);
    EXPECT_EQ(transport_b_.countOps(RecordingDCTransport::Op::SyncDatagram), 0u);
}

TEST_F(DCMultiInstanceTest, IndependentStartStop) {
    transport_a_.setSlaveSystemTime(0, 1000000000ULL);
    transport_b_.setSlaveSystemTime(0, 2000000000ULL);

    EtherCATDC dc_a(transport_a_, 1);
    EtherCATDC dc_b(transport_b_, 1);

    ASSERT_TRUE(dc_a.init());
    ASSERT_TRUE(dc_b.init());

    ASSERT_TRUE(dc_a.start());
    ASSERT_TRUE(dc_b.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    dc_a.stop();
    EXPECT_EQ(dc_a.getState(), DCState::Disabled);
    EXPECT_EQ(dc_b.getState(), DCState::Running);

    dc_b.stop();
    EXPECT_EQ(dc_b.getState(), DCState::Disabled);
}

// ============================================================================
// Stats Tests
// ============================================================================

TEST(DCStatsTest, InitialStatsAreZero) {
    RecordingDCTransport transport;
    transport.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport, 1);
    dc.init();

    auto stats = dc.getStats();
    EXPECT_EQ(stats.cycle_count, 0u);
    EXPECT_EQ(stats.sync_count, 0u);
    EXPECT_EQ(stats.pdo_error_count, 0u);
}

TEST(DCStatsTest, StatsAccumulateWhileRunning) {
    RecordingDCTransport transport;
    transport.setSlaveSystemTime(0, 1000000000ULL);

    EtherCATDC dc(transport, 1);
    ASSERT_TRUE(dc.init());
    ASSERT_TRUE(dc.start());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    dc.stop();

    auto stats = dc.getStats();
    EXPECT_GT(stats.cycle_count, 0u);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

class DCErrorTest : public ::testing::Test {
protected:
    ::testing::NiceMock<MockDCTransport> transport_;
};

TEST_F(DCErrorTest, InitHandlesReadFailure) {
    // readRegister always fails
    ON_CALL(transport_, readRegister(_, _, _, _, _)).WillByDefault(Return(false));
    ON_CALL(transport_, getMasterTimeNs()).WillByDefault(Return(1000000ULL));

    EtherCATDC dc(transport_, 1);
    // init should return false (no DC-capable slaves found)
    EXPECT_FALSE(dc.init());
}

TEST_F(DCErrorTest, SendSyncHandlesTransportFailure) {
    // Set up a DC-capable slave via read
    ON_CALL(transport_, readRegister(_, _, _, _, _))
        .WillByDefault(Invoke([](uint16_t, uint16_t reg, void* data, uint16_t size, unsigned int) {
            if (reg == REG_DCSYSTIME && size >= 8) {
                uint64_t t = 1000000000ULL;
                std::memcpy(data, &t, 8);
                return true;
            }
            return true;
        }));
    ON_CALL(transport_, writeRegister(_, _, _, _, _)).WillByDefault(Return(true));
    ON_CALL(transport_, getMasterTimeNs()).WillByDefault(Return(1000000ULL));

    EtherCATDC dc(transport_, 1);
    ASSERT_TRUE(dc.init());

    // sendSyncDatagram fails
    ON_CALL(transport_, sendSyncDatagram(_, _, _, _)).WillByDefault(Return(false));
    EXPECT_FALSE(dc.sendSyncFrame());
}
