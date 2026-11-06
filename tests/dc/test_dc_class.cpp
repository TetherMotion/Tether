/**
 * @file test_dc_class.cpp
 * @brief Unit tests for class-based EtherCATDC implementation
 *
 * Tests the EtherCATDC class focusing on:
 * - Multiple independent DC instances
 * - Instance-based state management
 * - Thread-safety and RAII
 *
 * Note: Move semantics are deleted (reference member prevents move).
 * All tests now use IDCTransport-based constructor.
 */

#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>
#include <atomic>
#include <map>
#include <tuple>

#include "tether/ethercat/DCClass.hpp"
#include "tether/ethercat/IDCTransport.hpp"

using namespace EtherCAT;

// ============================================================================
// Minimal stub transport for these tests
// ============================================================================

class StubDCTransport : public IDCTransport {
public:
    bool readRegister(uint16_t, uint16_t, void* data, uint16_t size, unsigned int) override {
        std::memset(data, 0, size); return true;
    }
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t, unsigned int) override {
        return true;
    }
    bool sendSyncDatagram(uint16_t, uint16_t, const void*, uint16_t) override {
        return true;
    }
    uint64_t getMasterTimeNs() override { return 1000000000ULL; }
    void delayMs(uint32_t) override {}
};

// ============================================================================
// Test Fixture
// ============================================================================

class DCClassTest : public ::testing::Test {
protected:
    StubDCTransport transport_;
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(DCClassTest, ConstructorWithDefaults) {
    EtherCATDC dc(transport_, 1, nullptr);
    EXPECT_EQ(dc.getState(), DCState::Disabled);
    EXPECT_FALSE(dc.isPDOEnabled());
}

TEST_F(DCClassTest, ConstructorWithCustomConfig) {
    DCConfig cfg = DCConfig::defaults();
    cfg.cycle_period_us = 2000;
    cfg.sync_interval_cycles = 5;
    EtherCATDC dc(transport_, 1, &cfg);
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCClassTest, ConstructorZeroSlaves) {
    EtherCATDC dc(transport_, 0, nullptr);
    EXPECT_EQ(dc.getState(), DCState::Error);
}

// ============================================================================
// Multiple Instance Tests
// ============================================================================

TEST_F(DCClassTest, MultipleIndependentInstances) {
    StubDCTransport transport_b;

    EtherCATDC dc1(transport_, 1);
    EtherCATDC dc2(transport_b, 2);

    EXPECT_EQ(dc1.getState(), DCState::Disabled);
    EXPECT_EQ(dc2.getState(), DCState::Disabled);
}

// ============================================================================
// State Management Tests
// ============================================================================

TEST_F(DCClassTest, StatsIndependence) {
    StubDCTransport transport_b;
    EtherCATDC dc1(transport_, 1);
    EtherCATDC dc2(transport_b, 1);

    DCLoopStats stats1 = dc1.getStats();
    DCLoopStats stats2 = dc2.getStats();
    EXPECT_EQ(stats1.cycle_count, 0u);
    EXPECT_EQ(stats2.cycle_count, 0u);
}

// ============================================================================
// RAII Tests
// ============================================================================

TEST_F(DCClassTest, DestructorStopsAutomatically) {
    {
        EtherCATDC dc(transport_, 1);
        dc.init();
        dc.start();
    }
    // destructor completed without hang — create a fresh instance to validate transport still usable
    EtherCATDC dc2(transport_, 1);
    EXPECT_EQ(dc2.getState(), DCState::Disabled);
}

TEST_F(DCClassTest, ExplicitStopBeforeDestroy) {
    EtherCATDC dc(transport_, 1);
    dc.init();
    dc.start();
    dc.stop();
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

// ============================================================================
// Thread-Safety Tests
// ============================================================================

TEST_F(DCClassTest, ConcurrentGetState) {
    EtherCATDC dc(transport_, 1);

    std::vector<std::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([&dc]() {
            for (int j = 0; j < 100; j++) {
                auto state = dc.getState();
                (void)state;
            }
        });
    }
    for (auto& t : threads) t.join();
    // Ensure calls completed and object still in valid state
    EXPECT_EQ(dc.getState(), DCState::Disabled);
}

TEST_F(DCClassTest, ConcurrentGetStats) {
    EtherCATDC dc(transport_, 1);

    std::vector<std::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([&dc]() {
            for (int j = 0; j < 100; j++) {
                auto stats = dc.getStats();
                (void)stats;
            }
        });
    }
    for (auto& t : threads) t.join();
    auto stats = dc.getStats();
    EXPECT_EQ(stats.cycle_count, 0u);
}

TEST_F(DCClassTest, ConcurrentPDOEnableDisable) {
    EtherCATDC dc(transport_, 1);

    std::atomic<bool> stop{false};
    std::vector<std::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([&dc, &stop, i]() {
            while (!stop.load()) {
                dc.setPDOEnabled(i % 2 == 0);
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stop.store(true);
    for (auto& t : threads) t.join();
    // After concurrent toggles, PDO should remain disabled when loop not running
    EXPECT_FALSE(dc.isPDOEnabled());
}

// ============================================================================
// Slave Query Tests
// ============================================================================

TEST_F(DCClassTest, IsSlaveSupported) {
    EtherCATDC dc(transport_, 3);
    EXPECT_FALSE(dc.isSlaveSupported(0));
    EXPECT_FALSE(dc.isSlaveSupported(1));
    EXPECT_FALSE(dc.isSlaveSupported(2));
    EXPECT_FALSE(dc.isSlaveSupported(999));
}

TEST_F(DCClassTest, GetSlaveOffset) {
    EtherCATDC dc(transport_, 1);
    EXPECT_EQ(dc.getSlaveOffset(0), 0);
    EXPECT_EQ(dc.getSlaveOffset(999), 0);
}

// ============================================================================
// Control Functions Tests
// ============================================================================

TEST_F(DCClassTest, ForceSync) {
    EtherCATDC dc(transport_, 1);
    dc.forceSync();
    // forceSync is a no-op in this context; ensure call is safe
    EXPECT_NO_THROW(dc.forceSync());
}

TEST_F(DCClassTest, PDOEnableDisable) {
    EtherCATDC dc(transport_, 1);
    EXPECT_FALSE(dc.isPDOEnabled());
    dc.setPDOEnabled(true);
    // PDO only enabled when realtime loop is running
    // Without start(), there's no loop to delegate to
    EXPECT_FALSE(dc.isPDOEnabled());
}

TEST_F(DCClassTest, ReconfigureSync) {
    EtherCATDC dc(transport_, 1);
    bool result = dc.reconfigureSync(0);
    EXPECT_TRUE(result);
    bool result_invalid = dc.reconfigureSync(999);
    EXPECT_FALSE(result_invalid);
}


// ============================================================================
// Coverage: initialize() sequence, topology measurement, sync configuration,
// start/stop loop, re-sync paths — driven through a register-aware transport.
// ============================================================================

namespace {

// Answers register reads from a per-(slave, addr) value table; records all
// writes and sync datagrams. Register values are stored little-endian-ready
// (byte vector contents are memcpy'd straight out).
class FakeDCTransport : public IDCTransport {
public:
    FakeDCTransport() = default;
    FakeDCTransport(FakeDCTransport&& o) noexcept {
        std::lock_guard lk(o.mu_);
        reads = std::move(o.reads);
        writes = std::move(o.writes);
        syncDatagrams.store(o.syncDatagrams.load());
        masterTime = o.masterTime;
    }
    FakeDCTransport& operator=(FakeDCTransport&& o) noexcept {
        if (this != &o) {
            std::scoped_lock lk(mu_, o.mu_);
            reads = std::move(o.reads);
            writes = std::move(o.writes);
            syncDatagrams.store(o.syncDatagrams.load());
            masterTime = o.masterTime;
        }
        return *this;
    }
    FakeDCTransport(const FakeDCTransport&)            = delete;
    FakeDCTransport& operator=(const FakeDCTransport&) = delete;

    std::map<std::pair<uint16_t, uint16_t>, std::vector<uint8_t>> reads;
    std::vector<std::tuple<uint16_t, uint16_t, std::vector<uint8_t>>> writes;
    // Atomics/guards: the DC loop thread calls writeRegister()/
    // sendSyncDatagram() while the test thread reads them.
    std::atomic<int> syncDatagrams{0};
    uint64_t masterTime = 5'000'000'000ULL;

    /// Snapshot of recorded writes — safe to iterate while the loop runs.
    std::vector<std::tuple<uint16_t, uint16_t, std::vector<uint8_t>>>
    writesSnapshot() const {
        std::lock_guard lk(mu_);
        return writes;
    }

    void setReg(uint16_t slave, DCRegisters reg, const void* data, uint16_t size) {
        std::lock_guard lk(mu_);
        auto& v = reads[{slave, toUInt16(reg)}];
        v.assign(static_cast<const uint8_t*>(data),
                 static_cast<const uint8_t*>(data) + size);
    }
    void setReg64(uint16_t slave, DCRegisters reg, uint64_t value) {
        uint8_t b[8];
        for (int i = 0; i < 8; i++) { b[i] = static_cast<uint8_t>(value & 0xFF); value >>= 8; }
        setReg(slave, reg, b, sizeof(b));
    }
    void setReg32(uint16_t slave, DCRegisters reg, uint32_t value) {
        uint8_t b[4];
        for (int i = 0; i < 4; i++) { b[i] = static_cast<uint8_t>(value & 0xFF); value >>= 8; }
        setReg(slave, reg, b, sizeof(b));
    }

    bool readRegister(uint16_t slave, uint16_t addr, void* data, uint16_t size, unsigned int) override {
        std::lock_guard lk(mu_);
        auto it = reads.find({slave, addr});
        if (it == reads.end()) { std::memset(data, 0, size); return true; }
        std::memcpy(data, it->second.data(), std::min<uint16_t>(size, it->second.size()));
        return true;
    }
    bool writeRegister(uint16_t slave, uint16_t addr, const void* data, uint16_t size, unsigned int) override {
        std::lock_guard lk(mu_);
        writes.emplace_back(slave, addr,
            std::vector<uint8_t>(static_cast<const uint8_t*>(data),
                                 static_cast<const uint8_t*>(data) + size));
        return true;
    }
    bool sendSyncDatagram(uint16_t, uint16_t, const void*, uint16_t) override {
        ++syncDatagrams; return true;
    }
    uint64_t getMasterTimeNs() override { return masterTime; }
    void delayMs(uint32_t) override {}

private:
    mutable std::mutex mu_;
};

// Linear 2-slave chain on the wire:
//   slave0 (reference): port0 entry @1000ns, port1 return @2000ns
//   slave1:             port0 entry @1300ns (leaf, no downstream port)
FakeDCTransport makeTwoSlaveChain() {
    FakeDCTransport t;
    for (uint16_t i = 0; i < 2; i++) t.setReg64(i, DCRegisters::DCSysTime, 1'000'000 + i);
    uint32_t rt0[4] = {1000, 2000, 0, 0};
    uint32_t rt1[4] = {1300, 0, 0, 0};
    t.setReg(0, DCRegisters::DCRecvTimes, rt0, sizeof(rt0));
    t.setReg(1, DCRegisters::DCRecvTimes, rt1, sizeof(rt1));
    t.setReg64(0, DCRegisters::DCSysOffset, 0);
    t.setReg64(1, DCRegisters::DCSysOffset, 0);
    return t;
}

class DCInitCoverageTest : public ::testing::Test {
protected:
    FakeDCTransport t_ = makeTwoSlaveChain();
};

} // namespace

TEST_F(DCInitCoverageTest, InitializeRunsFullSequence) {
    EtherCATDC dc(t_, 2);
    EXPECT_TRUE(dc.init());
    // both slaves read as DC-capable; slave0 is the reference
    EXPECT_TRUE(dc.isSlaveSupported(0));
    EXPECT_TRUE(dc.isSlaveSupported(1));
    EXPECT_EQ(dc.referenceSlave(), 0);
    // topology reconstructed: slave1 hangs off slave0's port 1
    const auto& topo = dc.topology();
    EXPECT_EQ(topo.links[1].parent, 0);
    EXPECT_EQ(topo.links[1].parent_port, 1);
    // register writes happened: delay, offset, sync activation
    bool wroteDelay = false, wroteOffset = false, wroteSyncAct = false;
    for (const auto& [slave, addr, bytes] : t_.writesSnapshot()) {
        if (addr == toUInt16(DCRegisters::DCSysTxTime)) wroteDelay = true;
        if (addr == toUInt16(DCRegisters::DCSysOffset)) wroteOffset = true;
        if (addr == toUInt16(DCRegisters::DCSyncAct)) wroteSyncAct = true;
    }
    EXPECT_TRUE(wroteDelay);
    EXPECT_TRUE(wroteOffset);
    EXPECT_TRUE(wroteSyncAct);
    // offset programming read the master time: slave0 local=1e6 -> offset=5e9-1e6
    EXPECT_EQ(dc.getSlaveOffset(0), 1'000'000 - 5'000'000'000LL);
}

TEST_F(DCInitCoverageTest, SyncDisabledFilter) {
    DCConfig cfg = DCConfig::defaults();
    cfg.sync_disabled_slaves = {1};
    EtherCATDC dc(t_, 2, &cfg);
    EXPECT_TRUE(dc.init());
    EXPECT_TRUE(dc.isSlaveSupported(0));
    // slave1 was explicitly disabled, never capability-probed
    EXPECT_FALSE(dc.isSlaveSupported(1));
    bool zeroedSyncAct = false;
    for (const auto& [slave, addr, bytes] : t_.writesSnapshot()) {
        if (slave == 1 && addr == toUInt16(DCRegisters::DCSyncAct) &&
            !bytes.empty() && bytes[0] == 0x00) zeroedSyncAct = true;
    }
    EXPECT_TRUE(zeroedSyncAct);
}

TEST_F(DCInitCoverageTest, NoDcCapableSlaves) {
    FakeDCTransport t;  // all-zero sysTime -> nothing DC-capable
    EtherCATDC dc(t, 2);
    EXPECT_FALSE(dc.init());
    EXPECT_EQ(dc.getSlaveOffset(0), 0);
    EXPECT_EQ(dc.referenceSlave(), -1);
}

TEST_F(DCInitCoverageTest, CapabilityReadFailureIsTolerated) {
    // slave1 answers sysTime=0 -> not DC-supported; init still succeeds
    t_.setReg64(1, DCRegisters::DCSysTime, 0);
    EtherCATDC dc(t_, 2);
    EXPECT_TRUE(dc.init());
    EXPECT_TRUE(dc.isSlaveSupported(0));
    EXPECT_FALSE(dc.isSlaveSupported(1));
}

TEST_F(DCInitCoverageTest, StartStopLoopSendsSyncFrames) {
    EtherCATDC dc(t_, 2);
    ASSERT_TRUE(dc.init());
    std::atomic<int> pdoCalls{0};
    EXPECT_TRUE(dc.start([&pdoCalls]() { ++pdoCalls; return true; }));
    // PDO exchange only runs once enabled on the live loop
    EXPECT_TRUE(dc.isPDOEnabled() == false);
    dc.setPDOEnabled(true);
    for (int i = 0; i < 200 && pdoCalls.load() == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_GT(pdoCalls.load(), 0);
    // the dc sync cycle sends datagrams through the transport
    for (int i = 0; i < 200 && t_.syncDatagrams == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_GE(t_.syncDatagrams, 1);
    auto stats = dc.getStats();
    EXPECT_GE(stats.cycle_count, 0u);
    dc.stop();
    // stop is idempotent
    dc.stop();
}

TEST_F(DCInitCoverageTest, SynchronizeSlaveClock) {
    EtherCATDC dc(t_, 2);
    ASSERT_TRUE(dc.init());
    EXPECT_TRUE(dc.synchronizeSlaveClock(0));
    EXPECT_FALSE(dc.synchronizeSlaveClock(9));   // out of range
    t_.setReg64(1, DCRegisters::DCSysTime, 0);
    // slave with no DC support is rejected by the public guard
    EtherCATDC dc2(t_, 2);
    dc2.init();
    EXPECT_FALSE(dc2.synchronizeSlaveClock(1));
}

TEST_F(DCInitCoverageTest, ReadSyncConfigAndReconfigure) {
    EtherCATDC dc(t_, 2);
    ASSERT_TRUE(dc.init());
    dc.setDebugLogging(true);
    dc.readSyncConfig(0);                        // reads DCCycle0 through transport
    dc.readSyncConfig(9);                        // out of range: no-op
    dc.setDebugLogging(false);
    EXPECT_TRUE(dc.reconfigureSync(0));
}

TEST_F(DCInitCoverageTest, RegisterWrappers) {
    EtherCATDC dc(t_, 2);
    uint8_t buf[8] = {0};
    EXPECT_TRUE(dc.readRegister(0, DCRegisters::DCSysTime, buf, sizeof(buf)));
    EXPECT_FALSE(dc.readRegister(9, DCRegisters::DCSysTime, buf, sizeof(buf)));
    uint8_t one = 0x01;
    EXPECT_TRUE(dc.writeRegister(0, DCRegisters::DCSyncAct, &one, 1));
    EXPECT_FALSE(dc.writeRegister(9, DCRegisters::DCSyncAct, &one, 1));
    EXPECT_GT(dc.getMasterTimeNs(), 0u);
    // sendSyncFrame targets the reference slave after init
    ASSERT_TRUE(dc.init());
    EXPECT_TRUE(dc.sendSyncFrame());
    EXPECT_GE(t_.syncDatagrams, 1);
}

TEST_F(DCInitCoverageTest, ForceSyncAndPDOEnable) {
    EtherCATDC dc(t_, 2);
    ASSERT_TRUE(dc.init());
    dc.forceSync();
    EXPECT_GE(t_.syncDatagrams, 1);
    // PDO enable state lives on the loop — requires start()
    EXPECT_TRUE(dc.start());
    dc.setPDOEnabled(false);
    EXPECT_FALSE(dc.isPDOEnabled());
    dc.setPDOEnabled(true);
    EXPECT_TRUE(dc.isPDOEnabled());
    dc.stop();
    // once stopped, isPDOEnabled reports false
    EXPECT_FALSE(dc.isPDOEnabled());
}
