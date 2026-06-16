/**
 * @file test_ethercat_slave.cpp
 * @brief Comprehensive tests for Slave, NonExistingSlave, CachedSIIReader
 *
 * Tests the per-slave state-machine guards, configuration bookkeeping,
 * the NonExistingSlave sentinel pattern, and CachedSIIReader caching.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/CachedSIIReader.hpp"
#include "tether/sii/SIIReader.hpp"

#include <cstring>
#include <vector>

using namespace EtherCAT;
using namespace ::testing;

// ============================================================================
// Test fixture — uses real Master with test callbacks
// ============================================================================

class EtherCATSlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default: all APRD/APWR succeed
        master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return true;
        });
        master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
            return true;
        });
        // Discover one slave so initSlaves(1) creates the slave object
        master_.initSlaves(1);
    }

    Master master_;
};

// ============================================================================
// SlaveError enum and slaveErrorToString
// ============================================================================

TEST(SlaveErrorTest, AllErrorsHaveStrings) {
    // slaveErrorToString should return non-null for every value
    EXPECT_STREQ(slaveErrorToString(SlaveError::Ok), "Ok");
    EXPECT_STREQ(slaveErrorToString(SlaveError::MailboxNotConfigured),
        "Mailbox (SM0/SM1) not configured — call configureMailbox() or assumeMailboxAlreadyConfigured() first");
    EXPECT_STREQ(slaveErrorToString(SlaveError::PDONotConfigured),
        "PDO sync-managers not configured — call configurePDOSyncManagers() first");
    EXPECT_STREQ(slaveErrorToString(SlaveError::InvalidStateTransition),
        "Invalid EtherCAT state transition");
    EXPECT_STREQ(slaveErrorToString(SlaveError::SlaveNotFound),
        "Slave index does not exist — check getDiscoveredSlaveCount()");
    EXPECT_STREQ(slaveErrorToString(SlaveError::TransportError),
        "Transport send/receive failure");
    EXPECT_STREQ(slaveErrorToString(SlaveError::Timeout),
        "Slave did not respond in time");
    EXPECT_STREQ(slaveErrorToString(SlaveError::ALStatusError),
        "Slave reported AL Status error");
    EXPECT_STREQ(slaveErrorToString(SlaveError::WorkingCounterMismatch),
        "Working counter mismatch");
    EXPECT_STREQ(slaveErrorToString(SlaveError::MailboxConfigFailed),
        "Failed to write mailbox SM registers");
    EXPECT_STREQ(slaveErrorToString(SlaveError::SDOError),
        "SDO operation failed");
    EXPECT_STREQ(slaveErrorToString(SlaveError::SIIReadError),
        "SII EEPROM read failed");
    EXPECT_STREQ(slaveErrorToString(SlaveError::PDOConfigFailed),
        "PDO sync-manager configuration failed");
    EXPECT_STREQ(slaveErrorToString(SlaveError::PDOMappingFailed),
        "PDO mapping finalization failed");
    EXPECT_STREQ(slaveErrorToString(SlaveError::NotInitialized),
        "Master not initialized");
    EXPECT_STREQ(slaveErrorToString(SlaveError::InternalError),
        "Internal error");
}

TEST(SlaveErrorTest, UnknownErrorReturnsDefault) {
    SlaveError unknown = static_cast<SlaveError>(0xFF);
    EXPECT_STREQ(slaveErrorToString(unknown), "Unknown error");
}

// ============================================================================
// Slave identification
// ============================================================================

TEST_F(EtherCATSlaveTest, IndexReturnsCorrectValue) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.index(), 0u);
}

TEST_F(EtherCATSlaveTest, AdpReturnsNegatedIndex) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.adp(), Master::adpForSlaveIndex(0));
}

TEST_F(EtherCATSlaveTest, MasterAccessorReturnsOwnMaster) {
    auto& s = master_.slave(0);
    EXPECT_EQ(&s.master(), &master_);
}

TEST_F(EtherCATSlaveTest, ConstMasterAccessorReturnsOwnMaster) {
    const auto& s = master_.slave(0);
    EXPECT_EQ(&s.master(), &master_);
}

// ============================================================================
// Mailbox configuration flags
// ============================================================================

TEST_F(EtherCATSlaveTest, InitiallyMailboxNotConfigured) {
    auto& s = master_.slave(0);
    EXPECT_FALSE(s.isMailboxConfigured());
}

TEST_F(EtherCATSlaveTest, AssumeMailboxSetsFlag) {
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured();
    EXPECT_TRUE(s.isMailboxConfigured());
}

TEST_F(EtherCATSlaveTest, ConfigureMailboxManualSetsFlag) {
    auto& s = master_.slave(0);
    auto err = s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    EXPECT_EQ(err, SlaveError::Ok);
    EXPECT_TRUE(s.isMailboxConfigured());
}

// ============================================================================
// PDO configuration flags
// ============================================================================

TEST_F(EtherCATSlaveTest, InitiallyPDONotConfigured) {
    auto& s = master_.slave(0);
    EXPECT_FALSE(s.isPDOConfigured());
}

TEST_F(EtherCATSlaveTest, AssumePDOSetsFlag) {
    auto& s = master_.slave(0);
    s.assumePDOAlreadyConfigured();
    EXPECT_TRUE(s.isPDOConfigured());
}

// ============================================================================
// State transition guards
// ============================================================================

TEST_F(EtherCATSlaveTest, PreOpBlockedWithoutMailbox) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToPreOp(), SlaveError::MailboxNotConfigured);
}

TEST_F(EtherCATSlaveTest, PreOpSucceedsAfterMailboxConfig) {
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(s.transitionToPreOp(), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, SafeOpBlockedWithoutPDO) {
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured();
    // PDO not yet configured
    EXPECT_EQ(s.transitionToSafeOp(), SlaveError::PDONotConfigured);
}

TEST_F(EtherCATSlaveTest, SafeOpSucceedsAfterPDOConfig) {
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured();
    s.assumePDOAlreadyConfigured();
    EXPECT_EQ(s.transitionToSafeOp(), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, OpAllowedWithoutPDO) {
    // OP warns but does NOT block when PDO is not configured
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToOp(), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, OpSucceedsWithPDO) {
    auto& s = master_.slave(0);
    s.assumePDOAlreadyConfigured();
    EXPECT_EQ(s.transitionToOp(), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, TransitionToInitResetsFlags) {
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured();
    s.assumePDOAlreadyConfigured();
    EXPECT_TRUE(s.isMailboxConfigured());
    EXPECT_TRUE(s.isPDOConfigured());
    EXPECT_EQ(s.transitionToInit(), SlaveError::Ok);
    EXPECT_FALSE(s.isMailboxConfigured());
    EXPECT_FALSE(s.isPDOConfigured());
}

TEST_F(EtherCATSlaveTest, TransitionToBootSucceeds) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToBoot(), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, TransitionToViaSwitchPreOp) {
    // transitionTo(SlaveState::PRE_OP) should check mailbox guard
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionTo(SlaveState::PRE_OP), SlaveError::MailboxNotConfigured);
    s.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(s.transitionTo(SlaveState::PRE_OP), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, TransitionToViaSwitchSafeOp) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionTo(SlaveState::SAFE_OP), SlaveError::PDONotConfigured);
    s.assumePDOAlreadyConfigured();
    EXPECT_EQ(s.transitionTo(SlaveState::SAFE_OP), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, TransitionToViaSwitchInit) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionTo(SlaveState::INIT), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, TransitionToViaSwitchOp) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionTo(SlaveState::OP), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, TransitionToViaSwitchBoot) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionTo(SlaveState::BOOT), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, TransitionToInvalidState) {
    auto& s = master_.slave(0);
    auto err = s.transitionTo(static_cast<SlaveState>(0xFF));
    EXPECT_EQ(err, SlaveError::InvalidStateTransition);
}

// ============================================================================
// Transport failures on state transitions
// ============================================================================

TEST_F(EtherCATSlaveTest, PreOpTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false; // transport fails
    });
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured();
    EXPECT_EQ(s.transitionToPreOp(), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, SafeOpTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    s.assumePDOAlreadyConfigured();
    EXPECT_EQ(s.transitionToSafeOp(), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, OpTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToOp(), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, InitTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToInit(), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, BootTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToBoot(), SlaveError::TransportError);
}

// ============================================================================
// readState / readALStatusCode
// ============================================================================

TEST_F(EtherCATSlaveTest, ReadStateSuccess) {
    // Return INIT state (0x01) in the APRD response
    master_.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 1) {
            *static_cast<uint8_t*>(out) = 0x01; // INIT
        }
        return true;
    });
    auto& s = master_.slave(0);
    SlaveState st = SlaveState::OP; // should be overwritten
    EXPECT_EQ(s.readState(st), SlaveError::Ok);
    EXPECT_EQ(st, SlaveState::INIT);
}

TEST_F(EtherCATSlaveTest, ReadStateTransportFailure) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    SlaveState st = SlaveState::OP;
    EXPECT_EQ(s.readState(st), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, ReadALStatusCodeSuccess) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 2) {
            *static_cast<uint16_t*>(out) = 0x001E; // some AL status code
        }
        return true;
    });
    auto& s = master_.slave(0);
    uint16_t code = 0;
    EXPECT_EQ(s.readALStatusCode(code), SlaveError::Ok);
    EXPECT_EQ(code, 0x001E);
}

TEST_F(EtherCATSlaveTest, ReadALStatusCodeTransportFailure) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    uint16_t code = 0;
    EXPECT_EQ(s.readALStatusCode(code), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, ALStateSuccess) {
    // APRD returns one byte (AL Status register)
    master_.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 1) {
            *static_cast<uint8_t*>(out) = 0x01; // INIT
        }
        return true;
    });
    auto& s = master_.slave(0);
    auto st = s.ALState();
    ASSERT_TRUE(st.has_value());
    EXPECT_EQ(st.value(), SlaveState::INIT);
}

TEST_F(EtherCATSlaveTest, ALStateTransportFailure) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    auto st = s.ALState();
    EXPECT_FALSE(st.has_value());
}

TEST_F(EtherCATSlaveTest, ALCodeSuccess) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 2) {
            *static_cast<uint16_t*>(out) = 0x001E;
        }
        return true;
    });
    auto& s = master_.slave(0);
    auto c = s.ALCode();
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c.value(), 0x001E);
}

TEST_F(EtherCATSlaveTest, ALCodeTransportFailure) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    auto c = s.ALCode();
    EXPECT_FALSE(c.has_value());
}

// ============================================================================
// Watchdog
// ============================================================================

TEST_F(EtherCATSlaveTest, ConfigureWatchdogsSuccess) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.configureWatchdogs(1000, 2000), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, ConfigureWatchdogsTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    EXPECT_EQ(s.configureWatchdogs(1000, 2000), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, DisableWatchdogsSuccess) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.disableWatchdogs(), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, DisableWatchdogsTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    EXPECT_EQ(s.disableWatchdogs(), SlaveError::TransportError);
}

TEST_F(EtherCATSlaveTest, ReadWatchdogStatusSuccess) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        std::memset(out, 0, len);
        return true;
    });
    auto& s = master_.slave(0);
    uint8_t wd = 0, pdi = 0, pd = 0;
    EXPECT_EQ(s.readWatchdogStatus(wd, pdi, pd), SlaveError::Ok);
}

TEST_F(EtherCATSlaveTest, ReadWatchdogStatusTransportFailure) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    uint8_t wd = 0, pdi = 0, pd = 0;
    EXPECT_EQ(s.readWatchdogStatus(wd, pdi, pd), SlaveError::TransportError);
}

// ============================================================================
// SII cache accessor
// ============================================================================

TEST_F(EtherCATSlaveTest, SIICacheIsInitialized) {
    auto& s = master_.slave(0);
    EXPECT_TRUE(s.siiCache().isInitialized());
    EXPECT_EQ(s.siiCache().slaveIndex(), 0u);
}

// ============================================================================
// Full lifecycle (happy path)
// ============================================================================

TEST_F(EtherCATSlaveTest, FullLifecycleHappyPath) {
    auto& s = master_.slave(0);

    // Start in INIT
    EXPECT_FALSE(s.isMailboxConfigured());
    EXPECT_FALSE(s.isPDOConfigured());

    // Configure mailbox (manual params)
    EXPECT_EQ(s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C), SlaveError::Ok);
    EXPECT_TRUE(s.isMailboxConfigured());

    // Transition to PRE_OP
    EXPECT_EQ(s.transitionToPreOp(), SlaveError::Ok);

    // Configure PDO (assume)
    s.assumePDOAlreadyConfigured();
    EXPECT_TRUE(s.isPDOConfigured());

    // Transition to SAFE_OP
    EXPECT_EQ(s.transitionToSafeOp(), SlaveError::Ok);

    // Transition to OP
    EXPECT_EQ(s.transitionToOp(), SlaveError::Ok);

    // Back to INIT resets flags
    EXPECT_EQ(s.transitionToInit(), SlaveError::Ok);
    EXPECT_FALSE(s.isMailboxConfigured());
    EXPECT_FALSE(s.isPDOConfigured());
}

// ============================================================================
// NonExistingSlave — all methods return SlaveNotFound
// ============================================================================

class NonExistingSlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        // No slaves discovered
        master_.initSlaves(0);
    }

    Master master_;
};

TEST_F(NonExistingSlaveTest, SlaveOutOfRangeReturnsNonExisting) {
    auto& s = master_.slave(99);
    // Should return SlaveNotFound for any method
    EXPECT_EQ(s.configureMailbox(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, ConfigureMailboxManual) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, AssumeMailboxDoesNotCrash) {
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured(); // logs critical, doesn't crash
}

TEST_F(NonExistingSlaveTest, ConfigurePDOSyncManagers) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.configurePDOSyncManagers(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, ConfigurePDOSyncManagersManual) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.configurePDOSyncManagers(0x1100, 10, 0x26, 0x1180, 10, 0x22),
              SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, AssumePDODoesNotCrash) {
    auto& s = master_.slave(0);
    s.assumePDOAlreadyConfigured(); // logs critical, doesn't crash
}

TEST_F(NonExistingSlaveTest, TransitionTo) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionTo(SlaveState::INIT), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, TransitionToInit) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToInit(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, TransitionToPreOp) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToPreOp(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, TransitionToSafeOp) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToSafeOp(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, TransitionToOp) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToOp(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, TransitionToBoot) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToBoot(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, ReadState) {
    auto& s = master_.slave(0);
    SlaveState st;
    EXPECT_EQ(s.readState(st), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, ReadALStatusCode) {
    auto& s = master_.slave(0);
    uint16_t code;
    EXPECT_EQ(s.readALStatusCode(code), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, ConfigureWatchdogs) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.configureWatchdogs(0, 0), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, DisableWatchdogs) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.disableWatchdogs(), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, ReadWatchdogStatus) {
    auto& s = master_.slave(0);
    uint8_t wd, pdi, pd;
    EXPECT_EQ(s.readWatchdogStatus(wd, pdi, pd), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoRead) {
    auto& s = master_.slave(0);
    uint8_t buf[4];
    size_t sz = sizeof(buf);
    EXPECT_EQ(s.sdoRead(0x1018, 0x01, buf, sz), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoWrite) {
    auto& s = master_.slave(0);
    uint8_t val = 1;
    EXPECT_EQ(s.sdoWrite(0x6060, 0x00, &val, 1), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoReadU8) {
    auto& s = master_.slave(0);
    uint8_t v;
    EXPECT_EQ(s.sdoReadU8(0x1018, 0x01, v), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoReadU16) {
    auto& s = master_.slave(0);
    uint16_t v;
    EXPECT_EQ(s.sdoReadU16(0x1018, 0x01, v), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoReadU32) {
    auto& s = master_.slave(0);
    uint32_t v;
    EXPECT_EQ(s.sdoReadU32(0x1018, 0x01, v), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoWriteU8) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.sdoWriteU8(0x6060, 0x00, 1), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoWriteU16) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.sdoWriteU16(0x6060, 0x00, 100), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, SdoWriteU32) {
    auto& s = master_.slave(0);
    EXPECT_EQ(s.sdoWriteU32(0x6040, 0x00, 0x0006), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, ReadSII) {
    auto& s = master_.slave(0);
    SII::SIIData data;
    EXPECT_EQ(s.readSII(data), SlaveError::SlaveNotFound);
}

TEST_F(NonExistingSlaveTest, LogSIISummaryDoesNotCrash) {
    auto& s = master_.slave(0);
    s.logSIISummary("TestTag"); // logs critical, doesn't crash
}

TEST_F(NonExistingSlaveTest, DifferentIndicesWork) {
    // Each call with a different invalid index should work
    auto& s1 = master_.slave(1);
    EXPECT_EQ(s1.transitionToInit(), SlaveError::SlaveNotFound);

    auto& s2 = master_.slave(42);
    EXPECT_EQ(s2.transitionToInit(), SlaveError::SlaveNotFound);

    auto& s3 = master_.slave(65535);
    EXPECT_EQ(s3.transitionToInit(), SlaveError::SlaveNotFound);
}

// ============================================================================
// Master slave management
// ============================================================================

class MasterSlaveManagementTest : public ::testing::Test {
protected:
    Master master_;
};

TEST_F(MasterSlaveManagementTest, InitSlavesCreatesCorrectCount) {
    master_.initSlaves(5);
    // Valid indices should return real slaves
    for (uint16_t i = 0; i < 5; ++i) {
        auto& s = master_.slave(i);
        EXPECT_EQ(s.index(), i);
    }
    // Index 5 should be NonExistingSlave
    auto& inv = master_.slave(5);
    EXPECT_EQ(inv.transitionToInit(), SlaveError::SlaveNotFound);
}

TEST_F(MasterSlaveManagementTest, InitSlavesZero) {
    master_.initSlaves(0);
    auto& s = master_.slave(0);
    EXPECT_EQ(s.transitionToInit(), SlaveError::SlaveNotFound);
}

TEST_F(MasterSlaveManagementTest, ReinitSlavesReplacesOldSlaves) {
    master_.initSlaves(3);
    EXPECT_EQ(master_.slave(0).index(), 0u);
    EXPECT_EQ(master_.slave(2).index(), 2u);

    // Reinitialize with fewer
    master_.initSlaves(1);
    EXPECT_EQ(master_.slave(0).index(), 0u);
    EXPECT_EQ(master_.slave(1).transitionToInit(), SlaveError::SlaveNotFound);
}

TEST_F(MasterSlaveManagementTest, SIIReaderCreatedLazily) {
    auto& reader = master_.siiReader();
    // Second call returns same instance
    auto& reader2 = master_.siiReader();
    EXPECT_EQ(&reader, &reader2);
}

// ============================================================================
// CachedSIIReader — unit tests (independent of master/bus)
// ============================================================================

namespace {

/// Minimal mock SIIReader for testing CachedSIIReader
class MockSIIReader : public SII::SIIReader {
public:
    // SIIReader requires Master& — we use a dummy
    MockSIIReader(Master& m) : SIIReader(m) {}

    // We'll override via test callbacks
    int read_count_ = 0;
};

} // anonymous namespace

class CachedSIIReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        reader_ = std::make_unique<SII::SIIReader>(master_);
    }

    Master master_;
    std::unique_ptr<SII::SIIReader> reader_;
};

TEST_F(CachedSIIReaderTest, DefaultConstructedIsNotInitialized) {
    SII::CachedSIIReader cache;
    EXPECT_FALSE(cache.isInitialized());
    EXPECT_EQ(cache.cacheSize(), 0u);
    EXPECT_FALSE(cache.isFullParseDone());
}

TEST_F(CachedSIIReaderTest, ConstructWithReaderIsInitialized) {
    SII::CachedSIIReader cache(*reader_, 0);
    EXPECT_TRUE(cache.isInitialized());
    EXPECT_EQ(cache.slaveIndex(), 0u);
}

TEST_F(CachedSIIReaderTest, DeferredInit) {
    SII::CachedSIIReader cache;
    EXPECT_FALSE(cache.isInitialized());
    cache.init(*reader_, 3);
    EXPECT_TRUE(cache.isInitialized());
    EXPECT_EQ(cache.slaveIndex(), 3u);
}

TEST_F(CachedSIIReaderTest, ReadWordWithoutInitFails) {
    SII::CachedSIIReader cache;
    uint16_t val = 0;
    EXPECT_FALSE(cache.readWord(0x0000, val));
}

TEST_F(CachedSIIReaderTest, ReadDWordWithoutInitFails) {
    SII::CachedSIIReader cache;
    uint32_t val = 0;
    EXPECT_FALSE(cache.readDWord(0x0000, val));
}

TEST_F(CachedSIIReaderTest, ReadStringWithoutInitFails) {
    SII::CachedSIIReader cache;
    char buf[32];
    EXPECT_FALSE(cache.readString(1, buf, sizeof(buf)));
}

TEST_F(CachedSIIReaderTest, ParseFullWithoutInitFails) {
    SII::CachedSIIReader cache;
    SII::SIIData data;
    EXPECT_FALSE(cache.parseFull(data));
}

TEST_F(CachedSIIReaderTest, InvalidateClears) {
    SII::CachedSIIReader cache(*reader_, 0);
    // Force the internal state (by reading a word - will fail because no bus,
    // but the cache object itself should track invalidation correctly)
    cache.invalidate();
    EXPECT_EQ(cache.cacheSize(), 0u);
    EXPECT_FALSE(cache.isFullParseDone());
}

TEST_F(CachedSIIReaderTest, ReadWordsWithoutInitReturnsZero) {
    SII::CachedSIIReader cache;
    uint16_t buf[4];
    EXPECT_EQ(cache.readWords(0, buf, 4), 0u);
}

// ============================================================================
// Additional coverage: configureMailbox auto-detect failure
// ============================================================================

TEST_F(EtherCATSlaveTest, AutoConfigureMailboxFallbackToDefaults) {
    // autoConfigureMailbox falls back to default mailbox params when SII read fails,
    // so it still succeeds — just uses default addresses.
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return true; // writes succeed
    });
    auto& s = master_.slave(0);
    auto err = s.configureMailbox();
    EXPECT_EQ(err, SlaveError::Ok);
    EXPECT_TRUE(s.isMailboxConfigured()); // fallback succeeded
}

// ============================================================================
// PDO sync-manager configuration via SII (auto)
// ============================================================================

TEST_F(EtherCATSlaveTest, ConfigurePDOSyncManagersFromSiiWithFallback) {
    // configurePDOSyncManagersFromSii reads SII and falls back to defaults
    // when SII doesn't have SM2/SM3 data. The fallback succeeds.
    auto& s = master_.slave(0);
    auto err = s.configurePDOSyncManagers();
    EXPECT_EQ(err, SlaveError::Ok);
    EXPECT_TRUE(s.isPDOConfigured());
}

TEST_F(EtherCATSlaveTest, ConfigurePDOSyncManagersFromSiiTransportFallback) {
    // Even when reads fail, configurePDOSyncManagersFromSii has extensive
    // fallback logic that uses default SM addresses and still succeeds.
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    auto& s = master_.slave(0);
    auto err = s.configurePDOSyncManagers();
    // Fallback logic sets default SM2/SM3 and writes them
    EXPECT_EQ(err, SlaveError::Ok);
    EXPECT_TRUE(s.isPDOConfigured());
}

// ============================================================================
// PDO sync-manager configuration via manual params
// ============================================================================

TEST_F(EtherCATSlaveTest, ConfigurePDOSyncManagersManualSuccess) {
    auto& s = master_.slave(0);
    auto err = s.configurePDOSyncManagers(
        0x1100, 10, 0x26,   // SM2
        0x1180, 10, 0x22);  // SM3
    EXPECT_EQ(err, SlaveError::Ok);
    EXPECT_TRUE(s.isPDOConfigured());
}

TEST_F(EtherCATSlaveTest, ConfigurePDOSyncManagersManualTransportFailure) {
    master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
        return false; // SM write fails
    });
    auto& s = master_.slave(0);
    auto err = s.configurePDOSyncManagers(
        0x1100, 10, 0x26,
        0x1180, 10, 0x22);
    EXPECT_EQ(err, SlaveError::PDOConfigFailed);
    EXPECT_FALSE(s.isPDOConfigured());
}

TEST_F(EtherCATSlaveTest, ConfigurePDOSyncManagersExceedsMax) {
    // Test with a slave index >= kMaxPDOSlaves
    // We need to create more slaves than kMaxPDOSlaves
    master_.initSlaves(EtherCAT::PDO::kMaxPDOSlaves + 1);
    auto& s = master_.slave(EtherCAT::PDO::kMaxPDOSlaves);
    auto err = s.configurePDOSyncManagers(
        0x1100, 10, 0x26,
        0x1180, 10, 0x22);
    EXPECT_EQ(err, SlaveError::PDOConfigFailed);
}

// ============================================================================
// SDO convenience methods — exercise each direction
// ============================================================================

TEST_F(EtherCATSlaveTest, SdoReadFails) {
    // Without proper mailbox setup, SDO reads should fail
    auto& s = master_.slave(0);
    s.assumeMailboxAlreadyConfigured(); // needed so sdoManager has config
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    uint8_t buf[4] = {};
    size_t sz = sizeof(buf);
    // SDO readSync will fail because there's no real transport
    auto err = s.sdoRead(0x1018, 0x01, buf, sz);
    EXPECT_EQ(err, SlaveError::SDOError);
}

TEST_F(EtherCATSlaveTest, SdoWriteFails) {
    auto& s = master_.slave(0);
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    uint8_t val = 1;
    auto err = s.sdoWrite(0x6060, 0x00, &val, 1);
    EXPECT_EQ(err, SlaveError::SDOError);
}

TEST_F(EtherCATSlaveTest, SdoReadU8Fails) {
    auto& s = master_.slave(0);
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    uint8_t v = 0;
    EXPECT_EQ(s.sdoReadU8(0x1018, 0x01, v), SlaveError::SDOError);
}

TEST_F(EtherCATSlaveTest, SdoReadU16Fails) {
    auto& s = master_.slave(0);
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    uint16_t v = 0;
    EXPECT_EQ(s.sdoReadU16(0x1018, 0x01, v), SlaveError::SDOError);
}

TEST_F(EtherCATSlaveTest, SdoReadU32Fails) {
    auto& s = master_.slave(0);
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    uint32_t v = 0;
    EXPECT_EQ(s.sdoReadU32(0x1018, 0x01, v), SlaveError::SDOError);
}

TEST_F(EtherCATSlaveTest, SdoWriteU8Fails) {
    auto& s = master_.slave(0);
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    EXPECT_EQ(s.sdoWriteU8(0x6060, 0x00, 1), SlaveError::SDOError);
}

TEST_F(EtherCATSlaveTest, SdoWriteU16Fails) {
    auto& s = master_.slave(0);
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    EXPECT_EQ(s.sdoWriteU16(0x6060, 0x00, 100), SlaveError::SDOError);
}

TEST_F(EtherCATSlaveTest, SdoWriteU32Fails) {
    auto& s = master_.slave(0);
    s.configureMailbox(0x1000, 128, 0x1400, 128, 0x000C);
    EXPECT_EQ(s.sdoWriteU32(0x6040, 0x00, 0x0006), SlaveError::SDOError);
}

// ============================================================================
// SII convenience methods
// ============================================================================

TEST_F(EtherCATSlaveTest, ReadSIIWithCacheInitialized) {
    auto& s = master_.slave(0);
    SII::SIIData data;
    // Cache is initialized but SII parse will read from bus —
    // result depends on mock-bus data (all zeros).
    auto err = s.readSII(data);
    // Either Ok (parsed empty data) or SIIReadError — exercise the path
    EXPECT_TRUE(err == SlaveError::Ok || err == SlaveError::SIIReadError);
}

TEST_F(EtherCATSlaveTest, ReadSIIWithoutCacheInitialized) {
    // Create a slave with uninitialized SII cache — exercises the fallback
    // path where readSII logs a warning and does a direct SII::readSII().
    Slave fresh(master_, 0); // cache not initialized
    EXPECT_FALSE(fresh.siiCache().isInitialized());
    SII::SIIData data;
    auto err = fresh.readSII(data);
    // Without cache init, falls back to direct SII read via master
    // Result depends on mock-bus data
    EXPECT_TRUE(err == SlaveError::Ok || err == SlaveError::SIIReadError);
}

TEST_F(EtherCATSlaveTest, LogSIISummaryFails) {
    // Use a fresh slave without cache init — readSII will fail,
    // and logSIISummary should log a warning.
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    Slave fresh(master_, 0); // no cache init
    fresh.logSIISummary("TestTag"); // exercises the warning branch
}

TEST_F(EtherCATSlaveTest, LogSIISummaryDefaultTag) {
    auto& s = master_.slave(0);
    s.logSIISummary(); // uses default tag "EtherCAT"
}
