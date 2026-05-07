/**
 * @file test_sync_manager.cpp
 * @brief Comprehensive tests for SyncManagerAccessor and EtherCAT::SyncManager namespace.
 *
 * Tests cover:
 * - EtherCAT::SyncManager namespace constants and helper functions
 * - CiA301 backward-compatible aliases
 * - SyncManagerAccessor construction and identity methods
 * - readHardwareConfig() with simulated APRD responses
 * - validate() logic for correct and incorrect SM configurations
 * - validateCommType() using SDO mock
 * - EtherCATSlave::sm() accessor
 * - NonExistingSlave::sm() sentinel behaviour
 * - formatConfig() string generation
 * - dump() and dumpMailboxStatus() smoke tests (no crash)
 * - dumpPDOAssignments() smoke test
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "profiles/cia301/CiA301Defs.hpp"  // for backward-compat alias check

#include <cstring>
#include <string>

using namespace EtherCAT;
using namespace ::testing;

// ============================================================================
// Helpers
// ============================================================================

/// Build a raw 8-byte SM register block from fields.
static void buildRawSMBlock(uint8_t out[8],
                             uint16_t start, uint16_t len,
                             uint8_t ctrl, uint8_t stat,
                             uint8_t act, uint8_t pdi)
{
    out[0] = static_cast<uint8_t>(start & 0xFF);
    out[1] = static_cast<uint8_t>((start >> 8) & 0xFF);
    out[2] = static_cast<uint8_t>(len & 0xFF);
    out[3] = static_cast<uint8_t>((len >> 8) & 0xFF);
    out[4] = ctrl;
    out[5] = stat;
    out[6] = act;
    out[7] = pdi;
}

// ============================================================================
// Fixture
// ============================================================================

class SyncManagerAccessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return true;
        });
        master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
            return true;
        });
        master_.initSlaves(1);
    }

    EtherCATMaster master_;
};

// ============================================================================
// EtherCAT::SyncManager namespace — constants
// ============================================================================

TEST(SyncManagerNamespace, RegisterBaseCalculation) {
    EXPECT_EQ(EtherCAT::SyncManager::registerBase(0), 0x0800u);
    EXPECT_EQ(EtherCAT::SyncManager::registerBase(1), 0x0808u);
    EXPECT_EQ(EtherCAT::SyncManager::registerBase(2), 0x0810u);
    EXPECT_EQ(EtherCAT::SyncManager::registerBase(3), 0x0818u);
    EXPECT_EQ(EtherCAT::SyncManager::registerBase(7), 0x0838u);
}

TEST(SyncManagerNamespace, PDOAssignIndexCalculation) {
    EXPECT_EQ(EtherCAT::SyncManager::pdoAssignIndex(0), 0x1C10u);
    EXPECT_EQ(EtherCAT::SyncManager::pdoAssignIndex(1), 0x1C11u);
    EXPECT_EQ(EtherCAT::SyncManager::pdoAssignIndex(2), 0x1C12u);
    EXPECT_EQ(EtherCAT::SyncManager::pdoAssignIndex(3), 0x1C13u);
    EXPECT_EQ(EtherCAT::SyncManager::pdoAssignIndex(7), 0x1C17u);
}

TEST(SyncManagerNamespace, SyncParamIndexCalculation) {
    EXPECT_EQ(EtherCAT::SyncManager::syncParamIndex(2), 0x1C32u);
    EXPECT_EQ(EtherCAT::SyncManager::syncParamIndex(3), 0x1C33u);
}

TEST(SyncManagerNamespace, CommTypeIndex) {
    EXPECT_EQ(EtherCAT::SyncManager::kCommTypeIndex, 0x1C00u);
}

TEST(SyncManagerNamespace, CommTypeValues) {
    EXPECT_EQ(EtherCAT::SyncManager::CommType::NotUsed,        0x00u);
    EXPECT_EQ(EtherCAT::SyncManager::CommType::MailboxReceive, 0x01u);
    EXPECT_EQ(EtherCAT::SyncManager::CommType::MailboxSend,    0x02u);
    EXPECT_EQ(EtherCAT::SyncManager::CommType::ProcessOutput,  0x03u);
    EXPECT_EQ(EtherCAT::SyncManager::CommType::ProcessInput,   0x04u);
}

TEST(SyncManagerNamespace, SyncSubIndices) {
    EXPECT_EQ(EtherCAT::SyncManager::SyncSub::SyncMode,              0x01u);
    EXPECT_EQ(EtherCAT::SyncManager::SyncSub::CycleTime,             0x02u);
    EXPECT_EQ(EtherCAT::SyncManager::SyncSub::SupportedSyncTypes,    0x04u);
    EXPECT_EQ(EtherCAT::SyncManager::SyncSub::MinimumCycleTime,      0x05u);
    EXPECT_EQ(EtherCAT::SyncManager::SyncSub::SMEventMissedCounter,  0x0Bu);
    EXPECT_EQ(EtherCAT::SyncManager::SyncSub::SyncError,             0x20u);
}

TEST(SyncManagerNamespace, MakeMappingEntryRoundTrip) {
    const uint32_t entry = EtherCAT::SyncManager::makeMappingEntry(0x6040, 0x00, 16);
    EXPECT_EQ(EtherCAT::SyncManager::mappingIndex(entry),    0x6040u);
    EXPECT_EQ(EtherCAT::SyncManager::mappingSubindex(entry), 0x00u);
    EXPECT_EQ(EtherCAT::SyncManager::mappingBits(entry),     16u);
}

TEST(SyncManagerNamespace, MakeMappingEntryAllFields) {
    const uint32_t entry = EtherCAT::SyncManager::makeMappingEntry(0x1234, 0xAB, 0x08);
    EXPECT_EQ(EtherCAT::SyncManager::mappingIndex(entry),    0x1234u);
    EXPECT_EQ(EtherCAT::SyncManager::mappingSubindex(entry), 0xABu);
    EXPECT_EQ(EtherCAT::SyncManager::mappingBits(entry),     0x08u);
}

// ============================================================================
// CiA301 backward-compatibility aliases
// ============================================================================

TEST(CiA301Aliases, SyncManagerCommType) {
    EXPECT_EQ(CiA301::SyncManagerCommType,   0x1C00u);
}

TEST(CiA301Aliases, PDOAssignIndices) {
    EXPECT_EQ(CiA301::SyncManager0PDOAssign, 0x1C10u);
    EXPECT_EQ(CiA301::SyncManager1PDOAssign, 0x1C11u);
    EXPECT_EQ(CiA301::SyncManager2PDOAssign, 0x1C12u);
    EXPECT_EQ(CiA301::SyncManager3PDOAssign, 0x1C13u);
    EXPECT_EQ(CiA301::SyncManager4PDOAssign, 0x1C14u);
    EXPECT_EQ(CiA301::SyncManager5PDOAssign, 0x1C15u);
    EXPECT_EQ(CiA301::SyncManager6PDOAssign, 0x1C16u);
    EXPECT_EQ(CiA301::SyncManager7PDOAssign, 0x1C17u);
}

TEST(CiA301Aliases, SM2SM3Synchronization) {
    EXPECT_EQ(CiA301::SM2Synchronization, 0x1C32u);
    EXPECT_EQ(CiA301::SM3Synchronization, 0x1C33u);
}

TEST(CiA301Aliases, SyncManagerTypeNamespace) {
    // Verify that the namespace alias works correctly
    EXPECT_EQ(CiA301::SyncManagerType::NotUsed,        0x00u);
    EXPECT_EQ(CiA301::SyncManagerType::MailboxReceive,  0x01u);
    EXPECT_EQ(CiA301::SyncManagerType::MailboxSend,     0x02u);
    EXPECT_EQ(CiA301::SyncManagerType::ProcessDataOutput, 0x03u);
    EXPECT_EQ(CiA301::SyncManagerType::ProcessDataInput,  0x04u);
}

TEST(CiA301Aliases, SMSyncSubNamespace) {
    EXPECT_EQ(CiA301::SMSyncSub::SyncMode,      0x01u);
    EXPECT_EQ(CiA301::SMSyncSub::CycleTime,     0x02u);
    EXPECT_EQ(CiA301::SMSyncSub::SyncError,     0x20u);
}

TEST(CiA301Aliases, PDOMappingHelpers) {
    const uint32_t entry = CiA301::PDO_MAPPING_ENTRY(0x6040, 0x00, 16);
    EXPECT_EQ(CiA301::PDO_MAPPING_INDEX(entry),    0x6040u);
    EXPECT_EQ(CiA301::PDO_MAPPING_SUBINDEX(entry), 0x00u);
    EXPECT_EQ(CiA301::PDO_MAPPING_BITS(entry),     16u);
}

// ============================================================================
// SyncManagerAccessor identity
// ============================================================================

TEST_F(SyncManagerAccessorTest, IndexReturnsCorrectValue) {
    auto acc0 = master_.slave(0).sm(0);
    auto acc1 = master_.slave(0).sm(1);
    auto acc2 = master_.slave(0).sm(2);
    auto acc3 = master_.slave(0).sm(3);

    EXPECT_EQ(acc0.index(), 0u);
    EXPECT_EQ(acc1.index(), 1u);
    EXPECT_EQ(acc2.index(), 2u);
    EXPECT_EQ(acc3.index(), 3u);
}

TEST_F(SyncManagerAccessorTest, PhysRegisterBaseIsCorrect) {
    EXPECT_EQ(master_.slave(0).sm(0).physRegisterBase(), 0x0800u);
    EXPECT_EQ(master_.slave(0).sm(1).physRegisterBase(), 0x0808u);
    EXPECT_EQ(master_.slave(0).sm(2).physRegisterBase(), 0x0810u);
    EXPECT_EQ(master_.slave(0).sm(3).physRegisterBase(), 0x0818u);
}

// ============================================================================
// readHardwareConfig()
// ============================================================================

TEST_F(SyncManagerAccessorTest, ReadHardwareConfigSuccess) {
    // Simulate a mailbox SM0 hardware register block
    uint8_t raw[8];
    buildRawSMBlock(raw, 0x1000, 128, 0x26u, 0x00u, 0x01u, 0x00u);

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) {
            std::memcpy(out, raw, 8);
        }
        return true;
    });

    auto acc = master_.slave(0).sm(0);
    auto cfg = acc.readHardwareConfig(200);

    EXPECT_TRUE(cfg.read_ok);
    EXPECT_EQ(cfg.start_addr, 0x1000u);
    EXPECT_EQ(cfg.length,     128u);
    EXPECT_EQ(cfg.control,    0x26u);
    EXPECT_EQ(cfg.status,     0x00u);
    EXPECT_EQ(cfg.activate,   0x01u);
    EXPECT_EQ(cfg.pdi_ctrl,   0x00u);
}

TEST_F(SyncManagerAccessorTest, ReadHardwareConfigFailure) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });

    auto cfg = master_.slave(0).sm(0).readHardwareConfig(200);
    EXPECT_FALSE(cfg.read_ok);
}

// ============================================================================
// RawHWConfig helpers
// ============================================================================

TEST_F(SyncManagerAccessorTest, RawHWConfigIsEnabledFlag) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok  = true;
    cfg.activate = 0x01;
    EXPECT_TRUE(cfg.isEnabled());

    cfg.activate = 0x00;
    EXPECT_FALSE(cfg.isEnabled());

    cfg.read_ok  = false;
    cfg.activate = 0x01;
    EXPECT_FALSE(cfg.isEnabled());   // read_ok must be true
}

TEST_F(SyncManagerAccessorTest, RawHWConfigMailboxMode) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok = true;
    cfg.control = PDO::SM_CTRL_MODE_MAILBOX;
    EXPECT_TRUE(cfg.isMailboxMode());
    EXPECT_FALSE(cfg.isBufferedMode());
}

TEST_F(SyncManagerAccessorTest, RawHWConfigBufferedMode) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok = true;
    cfg.control = PDO::SM_CTRL_MODE_BUFFERED;
    EXPECT_FALSE(cfg.isMailboxMode());
    EXPECT_TRUE(cfg.isBufferedMode());
}

TEST_F(SyncManagerAccessorTest, RawHWConfigDirection) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok = true;

    cfg.control = PDO::SM_CTRL_DIR_WRITE;
    EXPECT_TRUE(cfg.isMasterToSlave());
    EXPECT_FALSE(cfg.isSlaveToMaster());

    cfg.control = 0x00;  // DIR_READ
    EXPECT_FALSE(cfg.isMasterToSlave());
    EXPECT_TRUE(cfg.isSlaveToMaster());
}

// ============================================================================
// validate()
// ============================================================================

TEST_F(SyncManagerAccessorTest, ValidateCorrectConfig) {
    uint8_t raw[8];
    buildRawSMBlock(raw, 0x1000, 128, 0x26u, 0x00u, 0x01u, 0x00u);

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) std::memcpy(out, raw, 8);
        return true;
    });

    PDO::SyncManagerConfig expected;
    expected.phys_start_addr = 0x1000;
    expected.length          = 128;
    expected.control         = 0x26;
    expected.enable          = true;

    auto result = master_.slave(0).sm(0).validate(expected);
    EXPECT_TRUE(result.valid) << result.message;
}

TEST_F(SyncManagerAccessorTest, ValidateStartAddrMismatch) {
    uint8_t raw[8];
    buildRawSMBlock(raw, 0x1000, 128, 0x26u, 0x00u, 0x01u, 0x00u);

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) std::memcpy(out, raw, 8);
        return true;
    });

    PDO::SyncManagerConfig expected;
    expected.phys_start_addr = 0x2000;  // wrong
    expected.length          = 128;
    expected.control         = 0x26;
    expected.enable          = true;

    auto result = master_.slave(0).sm(0).validate(expected);
    EXPECT_FALSE(result.valid);
    EXPECT_THAT(result.message, HasSubstr("start_addr"));
}

TEST_F(SyncManagerAccessorTest, ValidateLengthMismatch) {
    uint8_t raw[8];
    buildRawSMBlock(raw, 0x1000, 128, 0x26u, 0x00u, 0x01u, 0x00u);

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) std::memcpy(out, raw, 8);
        return true;
    });

    PDO::SyncManagerConfig expected;
    expected.phys_start_addr = 0x1000;
    expected.length          = 256;   // wrong length
    expected.control         = 0x26;
    expected.enable          = true;

    auto result = master_.slave(0).sm(0).validate(expected);
    EXPECT_FALSE(result.valid);
    EXPECT_THAT(result.message, HasSubstr("length"));
}

TEST_F(SyncManagerAccessorTest, ValidateControlMismatch) {
    uint8_t raw[8];
    buildRawSMBlock(raw, 0x1000, 128, 0x26u, 0x00u, 0x01u, 0x00u);

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) std::memcpy(out, raw, 8);
        return true;
    });

    PDO::SyncManagerConfig expected;
    expected.phys_start_addr = 0x1000;
    expected.length          = 128;
    expected.control         = 0x22;  // wrong ctrl
    expected.enable          = true;

    auto result = master_.slave(0).sm(0).validate(expected);
    EXPECT_FALSE(result.valid);
    EXPECT_THAT(result.message, HasSubstr("control"));
}

TEST_F(SyncManagerAccessorTest, ValidateNotEnabled) {
    uint8_t raw[8];
    // activate bit 0 is 0 → not enabled
    buildRawSMBlock(raw, 0x1000, 128, 0x26u, 0x00u, 0x00u, 0x00u);

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) std::memcpy(out, raw, 8);
        return true;
    });

    PDO::SyncManagerConfig expected;
    expected.phys_start_addr = 0x1000;
    expected.length          = 128;
    expected.control         = 0x26;
    expected.enable          = true;

    auto result = master_.slave(0).sm(0).validate(expected);
    EXPECT_FALSE(result.valid);
    EXPECT_THAT(result.message, HasSubstr("ENABLED"));
}

TEST_F(SyncManagerAccessorTest, ValidateDisabledExpectedSkipped) {
    // APRD returns failure — but since expected.enable is false, validate() should pass early.
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;  // would fail readHardwareConfig
    });

    PDO::SyncManagerConfig expected;
    expected.enable = false;

    auto result = master_.slave(0).sm(0).validate(expected);
    // Should pass (nothing to validate for a disabled SM)
    EXPECT_TRUE(result.valid);
}

TEST_F(SyncManagerAccessorTest, ValidateReadFailure) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });

    PDO::SyncManagerConfig expected;
    expected.phys_start_addr = 0x1000;
    expected.length          = 128;
    expected.control         = 0x26;
    expected.enable          = true;

    auto result = master_.slave(0).sm(0).validate(expected);
    EXPECT_FALSE(result.valid);
    EXPECT_THAT(result.message, HasSubstr("read failed"));
}

// ============================================================================
// validateCommType()
// ============================================================================

TEST_F(SyncManagerAccessorTest, ValidateCommTypeCorrect) {
    // SDO read will fail (no real SDO stack in unit test) — check error path
    auto result = master_.slave(0).sm(2).validateCommType(
        EtherCAT::SyncManager::CommType::ProcessOutput);
    // In test environment SDO fails → validation fails
    EXPECT_FALSE(result.valid);
    EXPECT_THAT(result.message, HasSubstr("SDO"));
}

// ============================================================================
// formatConfig()
// ============================================================================

TEST_F(SyncManagerAccessorTest, FormatConfigMailboxSM0FallbackDetected) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok   = true;
    cfg.start_addr = 0x1000;
    cfg.length     = 256;
    cfg.control    = 0x26;  // mailbox-write ctrl
    cfg.status     = 0x00;
    cfg.activate   = 0x01;
    cfg.pdi_ctrl   = 0x00;

    auto acc = master_.slave(0).sm(0);
    std::string fmt = acc.formatConfig(cfg);

    EXPECT_THAT(fmt, HasSubstr("SM0"));
    EXPECT_THAT(fmt, HasSubstr("MAILBOX"));
    EXPECT_THAT(fmt, HasSubstr("MASTER->SLAVE"));
    EXPECT_THAT(fmt, HasSubstr("ENABLED"));
    EXPECT_THAT(fmt, HasSubstr("fallback"));
}

TEST_F(SyncManagerAccessorTest, FormatConfigBufferedProcessSM2) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok    = true;
    cfg.start_addr = 0x1100;
    cfg.length     = 8;
    // Buffered + DIR_WRITE + IRQ_PDI + WATCHDOG = 0x24 | 0x20 | 0x40 = 0x64
    cfg.control    = static_cast<uint8_t>(PDO::SM_CTRL_MODE_BUFFERED |
                                          PDO::SM_CTRL_DIR_WRITE |
                                          PDO::SM_CTRL_IRQ_PDI |
                                          PDO::SM_CTRL_WATCHDOG);
    cfg.activate   = 0x01;

    auto acc = master_.slave(0).sm(2);
    std::string fmt = acc.formatConfig(cfg);

    EXPECT_THAT(fmt, HasSubstr("SM2"));
    EXPECT_THAT(fmt, HasSubstr("BUFFERED"));
    EXPECT_THAT(fmt, HasSubstr("MASTER->SLAVE"));
    EXPECT_THAT(fmt, HasSubstr("WATCHDOG"));
    EXPECT_THAT(fmt, HasSubstr("ENABLED"));
    EXPECT_THAT(fmt, Not(HasSubstr("fallback")));
}

TEST_F(SyncManagerAccessorTest, FormatConfigReadFailed) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok = false;

    auto acc  = master_.slave(0).sm(1);
    std::string fmt = acc.formatConfig(cfg);

    EXPECT_THAT(fmt, HasSubstr("SM1"));
    EXPECT_THAT(fmt, HasSubstr("read failed"));
}

TEST_F(SyncManagerAccessorTest, FormatConfigSM1FallbackDetected) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok    = true;
    cfg.start_addr = 0x1400;
    cfg.length     = 256;
    cfg.control    = 0x22;
    cfg.activate   = 0x01;

    auto acc  = master_.slave(0).sm(1);
    std::string fmt = acc.formatConfig(cfg);

    EXPECT_THAT(fmt, HasSubstr("SM1"));
    EXPECT_THAT(fmt, HasSubstr("fallback"));
}

TEST_F(SyncManagerAccessorTest, FormatConfigDisabled) {
    SyncManagerAccessor::RawHWConfig cfg{};
    cfg.read_ok    = true;
    cfg.start_addr = 0x1000;
    cfg.length     = 128;
    cfg.control    = 0x26;
    cfg.activate   = 0x00;  // not enabled

    auto acc  = master_.slave(0).sm(0);
    std::string fmt = acc.formatConfig(cfg);

    EXPECT_THAT(fmt, HasSubstr("disabled"));
}

// ============================================================================
// dump() — smoke test (should not throw / crash)
// ============================================================================

TEST_F(SyncManagerAccessorTest, DumpSuccessNoThrow) {
    uint8_t raw[8];
    buildRawSMBlock(raw, 0x1000, 128, 0x26u, 0x00u, 0x01u, 0x00u);

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) std::memcpy(out, raw, 8);
        return true;
    });

    EXPECT_NO_THROW(master_.slave(0).sm(0).dump("TestSM"));
}

TEST_F(SyncManagerAccessorTest, DumpFailNoThrow) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    EXPECT_NO_THROW(master_.slave(0).sm(0).dump("TestSM"));
}

// ============================================================================
// dumpMailboxStatus() — smoke test
// ============================================================================

TEST_F(SyncManagerAccessorTest, DumpMailboxStatusNoThrow) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    EXPECT_NO_THROW(master_.slave(0).sm(0).dumpMailboxStatus("TestSM"));
    EXPECT_NO_THROW(master_.slave(0).sm(1).dumpMailboxStatus("TestSM"));
}

TEST_F(SyncManagerAccessorTest, DumpMailboxStatusFailNoThrow) {
    master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
        return false;
    });
    EXPECT_NO_THROW(master_.slave(0).sm(0).dumpMailboxStatus("TestSM"));
}

// ============================================================================
// dumpPDOAssignments() — smoke test (SDO will fail in test env)
// ============================================================================

TEST_F(SyncManagerAccessorTest, DumpPDOAssignmentsNoThrow) {
    EXPECT_NO_THROW(master_.slave(0).sm(2).dumpPDOAssignments("TestSM"));
}

// ============================================================================
// EtherCATSlave::sm() accessor API
// ============================================================================

TEST_F(SyncManagerAccessorTest, SmAccessorViaSlaveRef) {
    auto& slave = master_.slave(0);
    auto acc    = slave.sm(2);
    EXPECT_EQ(acc.index(), 2u);
    EXPECT_EQ(acc.physRegisterBase(), 0x0810u);
}

TEST_F(SyncManagerAccessorTest, SmAccessorChainedDump) {
    // Tests that API master.slave(0).sm(2).dump() compiles and runs
    EXPECT_NO_THROW(master_.slave(0).sm(2).dump("Chain"));
}

// ============================================================================
// NonExistingSlave::sm() — returns accessor, logs CRITICAL but doesn't crash
// ============================================================================

class NonExistingSlaveTest : public ::testing::Test {
protected:
    void SetUp() override {
        master_.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return true;
        });
        master_.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
            return true;
        });
        master_.initSlaves(0);  // zero slaves — slave(0) returns NonExistingSlave
    }
    EtherCATMaster master_;
};

TEST_F(NonExistingSlaveTest, SmReturnsAccessorWithCorrectIndex) {
    // Should not crash; NonExistingSlave::sm() still returns a usable accessor
    auto acc = master_.slave(0).sm(2);
    EXPECT_EQ(acc.index(), 2u);
}

TEST_F(NonExistingSlaveTest, SmDumpNoThrow) {
    EXPECT_NO_THROW(master_.slave(0).sm(0).dump("NonExistSM"));
}

TEST_F(NonExistingSlaveTest, SmDumpMailboxStatusNoThrow) {
    EXPECT_NO_THROW(master_.slave(0).sm(0).dumpMailboxStatus("NonExistSM"));
}

TEST_F(NonExistingSlaveTest, SmDumpPDOAssignmentsNoThrow) {
    EXPECT_NO_THROW(master_.slave(0).sm(2).dumpPDOAssignments("NonExistSM"));
}

// ============================================================================
// Multiple SM accessors: each gets correct register base
// ============================================================================

TEST_F(SyncManagerAccessorTest, AllFourSMsHaveCorrectRegBases) {
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t expected = static_cast<uint16_t>(0x0800u + i * 8u);
        EXPECT_EQ(master_.slave(0).sm(i).physRegisterBase(), expected)
            << "Wrong register base for SM" << static_cast<int>(i);
    }
}

// ============================================================================
// APRD byte ordering
// ============================================================================

TEST_F(SyncManagerAccessorTest, ReadHardwareConfigByteOrdering) {
    // Verify that start_addr and length are decoded little-endian
    uint8_t raw[8];
    // start=0xABCD → byte[0]=0xCD, byte[1]=0xAB
    // len=0x1234   → byte[2]=0x34, byte[3]=0x12
    raw[0] = 0xCDu; raw[1] = 0xABu;
    raw[2] = 0x34u; raw[3] = 0x12u;
    raw[4] = 0x26u; raw[5] = 0x00u; raw[6] = 0x01u; raw[7] = 0x00u;

    master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t len, unsigned int) {
        if (len >= 8) std::memcpy(out, raw, 8);
        return true;
    });

    auto cfg = master_.slave(0).sm(0).readHardwareConfig();
    EXPECT_EQ(cfg.start_addr, 0xABCDu);
    EXPECT_EQ(cfg.length,     0x1234u);
}

// ============================================================================
// Validate across all SM indices
// ============================================================================

TEST_F(SyncManagerAccessorTest, ValidateCorrectConfigOnAllSMs) {
    for (uint8_t smIdx = 0; smIdx < 4; ++smIdx) {
        const uint16_t addr = static_cast<uint16_t>(0x1000u + smIdx * 0x100u);
        const uint16_t len  = 64u;
        const uint8_t  ctrl = (smIdx <= 1) ? 0x26u : 0x64u;

        uint8_t raw[8];
        buildRawSMBlock(raw, addr, len, ctrl, 0x00u, 0x01u, 0x00u);

        master_.setAprdTestCallback([&raw](uint16_t, uint16_t, void* out, uint16_t l, unsigned int) {
            if (l >= 8) std::memcpy(out, raw, 8);
            return true;
        });

        PDO::SyncManagerConfig expected;
        expected.phys_start_addr = addr;
        expected.length          = len;
        expected.control         = ctrl;
        expected.enable          = true;

        auto result = master_.slave(0).sm(smIdx).validate(expected);
        EXPECT_TRUE(result.valid)
            << "SM" << static_cast<int>(smIdx) << " validation failed: " << result.message;
    }
}
