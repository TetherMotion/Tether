/**
 * @file test_fault_detector_coverage.cpp
 * @brief Coverage tests for FaultDetector methods in EtherCATFaultDetection.cpp
 *
 * Uses a mock IFaultTransport to exercise poll(), pollAll(), getState(),
 * anyActive(), clear(), setCallback(), diagnose(), and diagnoseNoSync().
 */
#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include <cstring>
#include <functional>

using namespace EtherCAT;

// ============================================================================
// Mock transport
// ============================================================================

class MockFaultTransport : public IFaultTransport {
public:
    // Configurable register data the mock will return
    uint16_t al_status = 0x0008;  // OP state, no error
    uint16_t al_status_code = 0x0000; // NoError

    bool read_fail = false;
    bool write_fail = false;

    bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                      void* data, uint16_t size) override {
        if (read_fail) return false;
        if (reg_addr == 0x0130 && size == 2) {
            std::memcpy(data, &al_status, 2);
            return true;
        }
        if (reg_addr == 0x0134 && size == 2) {
            std::memcpy(data, &al_status_code, 2);
            return true;
        }
        return false;
    }

    bool writeRegister(uint16_t slave_index, uint16_t reg_addr,
                       const void* data, uint16_t size) override {
        return !write_fail;
    }

    uint64_t getTimestampMs() override { return 12345; }
    void delayMs(uint32_t ms) override { /* no-op */ }
};

// ============================================================================
// FaultDetector — lifecycle
// ============================================================================

TEST(FaultDetectorCoverage, ConstructAndInit) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    EXPECT_FALSE(fd.isInitialized());
    EXPECT_TRUE(fd.init(4));
    EXPECT_TRUE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 4u);
}

TEST(FaultDetectorCoverage, InitIdempotent) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    EXPECT_TRUE(fd.init(4));
    EXPECT_TRUE(fd.init(8)); // idempotent, slave_count stays 4
    EXPECT_EQ(fd.slaveCount(), 4u);
}

TEST(FaultDetectorCoverage, Shutdown) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(4);
    fd.shutdown();
    EXPECT_FALSE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 0u);
}

// ============================================================================
// poll — various scenarios
// ============================================================================

TEST(FaultDetectorCoverage, PollNotInitialized) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    auto state = fd.poll(0);
    EXPECT_FALSE(state.has_fault);
}

TEST(FaultDetectorCoverage, PollOutOfRange) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);
    auto state = fd.poll(5); // out of range
    EXPECT_FALSE(state.has_fault);
}

TEST(FaultDetectorCoverage, PollNoFault) {
    MockFaultTransport transport;
    transport.al_status = 0x0008; // OP, no error
    transport.al_status_code = 0x0000;
    FaultDetector fd(transport);
    fd.init(2);
    auto state = fd.poll(0);
    EXPECT_FALSE(state.has_fault);
}

TEST(FaultDetectorCoverage, PollWithErrorBit) {
    MockFaultTransport transport;
    transport.al_status = 0x0018; // OP + error bit
    transport.al_status_code = 0x001A; // SynchronizationError
    FaultDetector fd(transport);
    fd.init(2);
    auto state = fd.poll(0);
    EXPECT_TRUE(state.has_fault);
    EXPECT_GT(state.sync_error_count, 0u);
}

TEST(FaultDetectorCoverage, PollSyncErrors) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);

    // SynchronizationError
    transport.al_status = 0x0012; // PRE_OP + error
    transport.al_status_code = static_cast<uint16_t>(ALStatusCode::SynchronizationError);
    fd.poll(0);

    // NoSyncError
    transport.al_status_code = static_cast<uint16_t>(ALStatusCode::NoSyncError);
    fd.poll(0);

    // FatalSyncError
    transport.al_status_code = static_cast<uint16_t>(ALStatusCode::FatalSyncError);
    fd.poll(0);

    auto* stored = fd.getState(0);
    ASSERT_NE(stored, nullptr);
    EXPECT_GE(stored->sync_error_count, 3u);
}

TEST(FaultDetectorCoverage, PollWatchdogErrors) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);

    transport.al_status = 0x0014; // SAFE_OP + error
    transport.al_status_code = static_cast<uint16_t>(ALStatusCode::SyncManagerWatchdog);
    fd.poll(0);

    transport.al_status_code = static_cast<uint16_t>(ALStatusCode::BackgroundWatchdog);
    fd.poll(0);

    auto* stored = fd.getState(0);
    ASSERT_NE(stored, nullptr);
    EXPECT_GE(stored->watchdog_count, 2u);
}

// ============================================================================
// Callback
// ============================================================================

TEST(FaultDetectorCoverage, CallbackOnNewFault) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);

    bool called = false;
    uint16_t cb_slave = 0xFFFF;
    fd.setCallback([&](uint16_t idx, const SlaveFaultState& s) {
        called = true;
        cb_slave = idx;
    });

    // First poll: no fault
    transport.al_status = 0x0008;
    transport.al_status_code = 0x0000;
    fd.poll(0);
    EXPECT_FALSE(called);

    // Second poll: fault appears
    transport.al_status = 0x0018; // error bit set
    transport.al_status_code = 0x0001; // UnspecifiedError
    fd.poll(0);
    EXPECT_TRUE(called);
    EXPECT_EQ(cb_slave, 0u);
}

TEST(FaultDetectorCoverage, CallbackNotCalledOnSameFault) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);

    int call_count = 0;
    fd.setCallback([&](uint16_t, const SlaveFaultState&) { call_count++; });

    transport.al_status = 0x0018;
    transport.al_status_code = 0x0001;
    fd.poll(0);
    EXPECT_EQ(call_count, 1);

    // Already faulted, callback should not fire again
    fd.poll(0);
    EXPECT_EQ(call_count, 1);
}

// ============================================================================
// pollAll
// ============================================================================

TEST(FaultDetectorCoverage, PollAllNoFaults) {
    MockFaultTransport transport;
    transport.al_status = 0x0008;
    transport.al_status_code = 0x0000;
    FaultDetector fd(transport);
    fd.init(4);
    EXPECT_EQ(fd.pollAll(), 0u);
}

TEST(FaultDetectorCoverage, PollAllWithFaults) {
    MockFaultTransport transport;
    transport.al_status = 0x0018;
    transport.al_status_code = 0x0001;
    FaultDetector fd(transport);
    fd.init(3);
    EXPECT_EQ(fd.pollAll(), 3u); // All 3 slaves faulted
}

// ============================================================================
// getState
// ============================================================================

TEST(FaultDetectorCoverage, GetStateNotInitialized) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    EXPECT_EQ(fd.getState(0), nullptr);
}

TEST(FaultDetectorCoverage, GetStateOutOfRange) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);
    EXPECT_EQ(fd.getState(5), nullptr);
}

TEST(FaultDetectorCoverage, GetStateValid) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);
    EXPECT_NE(fd.getState(0), nullptr);
}

// ============================================================================
// anyActive
// ============================================================================

TEST(FaultDetectorCoverage, AnyActiveNone) {
    MockFaultTransport transport;
    transport.al_status = 0x0008;
    transport.al_status_code = 0x0000;
    FaultDetector fd(transport);
    fd.init(2);
    fd.pollAll();
    EXPECT_FALSE(fd.anyActive());
}

TEST(FaultDetectorCoverage, AnyActiveWithFault) {
    MockFaultTransport transport;
    transport.al_status = 0x0018;
    transport.al_status_code = 0x0001;
    FaultDetector fd(transport);
    fd.init(2);
    fd.poll(0);
    EXPECT_TRUE(fd.anyActive());
}

// ============================================================================
// clear
// ============================================================================

TEST(FaultDetectorCoverage, ClearNotInitialized) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    EXPECT_FALSE(fd.clear(0));
}

TEST(FaultDetectorCoverage, ClearOutOfRange) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);
    EXPECT_FALSE(fd.clear(5));
}

TEST(FaultDetectorCoverage, ClearWriteFails) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);
    transport.write_fail = true;
    EXPECT_FALSE(fd.clear(0));
}

TEST(FaultDetectorCoverage, ClearSucceeds) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);

    // First make a fault
    transport.al_status = 0x0018;
    transport.al_status_code = 0x0001;
    fd.poll(0);
    EXPECT_TRUE(fd.anyActive());

    // Now clear — mock returns no fault on re-poll
    transport.al_status = 0x0008;
    transport.al_status_code = 0x0000;
    EXPECT_TRUE(fd.clear(0));
    EXPECT_FALSE(fd.anyActive());
}

TEST(FaultDetectorCoverage, ClearFailsStillFaulted) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);

    transport.al_status = 0x0018;
    transport.al_status_code = 0x0001;
    fd.poll(0);

    // Clear attempt but fault persists
    EXPECT_FALSE(fd.clear(0));
}

// ============================================================================
// diagnose and diagnoseNoSync
// ============================================================================

TEST(FaultDetectorCoverage, DiagnoseNotInitialized) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.diagnose(0); // logs error, no crash
}

TEST(FaultDetectorCoverage, DiagnoseNoFault) {
    MockFaultTransport transport;
    transport.al_status = 0x0008;
    transport.al_status_code = 0x0000;
    FaultDetector fd(transport);
    fd.init(2);
    fd.diagnose(0);
}

TEST(FaultDetectorCoverage, DiagnoseWithSyncError) {
    MockFaultTransport transport;
    transport.al_status = 0x0014;
    transport.al_status_code = static_cast<uint16_t>(ALStatusCode::NoSyncError);
    FaultDetector fd(transport);
    fd.init(2);
    fd.diagnose(0); // should also call diagnoseNoSync
}

TEST(FaultDetectorCoverage, DiagnoseWithSyncErrorAlt) {
    MockFaultTransport transport;
    transport.al_status = 0x0014;
    transport.al_status_code = static_cast<uint16_t>(ALStatusCode::SynchronizationError);
    FaultDetector fd(transport);
    fd.init(2);
    fd.diagnose(0);
}

TEST(FaultDetectorCoverage, DiagnoseNoSyncDirect) {
    MockFaultTransport transport;
    FaultDetector fd(transport);
    fd.init(2);
    fd.diagnoseNoSync(0); // just exercises the logging
}
