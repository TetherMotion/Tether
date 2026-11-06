/**
 * @file test_ecat_misc_utils.cpp
 * @brief Coverage for small master-side EtherCAT utility TUs:
 *        raw/CoEErrorStrings.cpp (coeErrorStr, coeErrorToAbortCode),
 *        raw/SDOErrorDecoder.cpp (abort/mailbox error decoders),
 *        mailbox/Utils.cpp (logStatuswordDiagnostics state tracking),
 *        NoDistributedClockConfigured.cpp (sentinel returns).
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "tether/ethercat/CoETypes.hpp"
#include "tether/ethercat/DCClass.hpp"
#include "tether/ethercat/Mailbox/Utils.hpp"
#include "tether/ethercat/SDOAbortCodes.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "raw/CoEErrorStrings.hpp"

using namespace EtherCAT;
using CoECode = CoE::CoEErrorCode;

namespace {

// ============================================================================
// CoE::coeErrorStr
// ============================================================================

TEST(EcatMiscCov, CoEErrorStrAllCodes) {
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::Ok}), "Ok");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::Timeout}), "Timeout");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::Aborted}), "Aborted");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::TransportError}), "Transport error");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::QueueFull}), "Queue full");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::NotConfigured}), "Mailbox not configured");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::ShuttingDown}), "Shutting down");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::SlaveNotFound}), "Slave not found");
    EXPECT_STREQ(CoE::coeErrorStr({CoECode::InternalError}), "Internal error");
    EXPECT_STREQ(CoE::coeErrorStr({static_cast<CoECode>(0xEE)}), "Unknown error");
}

TEST(EcatMiscCov, CoEErrorStrAbortedDecodesRealAbortCode) {
    CoE::CoEError err{CoECode::Aborted, SDOAbortCode::ObjectNotFound, 0x06020000};
    // Nonzero abort_code → decoded SDO abort string, not bare "Aborted"
    EXPECT_STRNE(CoE::coeErrorStr(err), "Aborted");
    EXPECT_NE(std::strstr(CoE::coeErrorStr(err), "exist"), nullptr);

    // abort_code == 0 falls through to the generic switch
    err.abort_code = 0;
    EXPECT_STREQ(CoE::coeErrorStr(err), "Aborted");
}

// ============================================================================
// CoE::coeErrorToAbortCode
// ============================================================================

TEST(EcatMiscCov, CoEErrorToAbortCodeMapping) {
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::Ok}), SDOAbortCode::Success);
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::Timeout}), SDOAbortCode::Timeout);
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::NotConfigured}), SDOAbortCode::DeviceStateError);
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::TransportError}), SDOAbortCode::GeneralError);
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::QueueFull}), SDOAbortCode::OutOfMemory);
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::ShuttingDown}), SDOAbortCode::DeviceStateError);
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::SlaveNotFound}), SDOAbortCode::ObjectNotFound);
    EXPECT_EQ(CoE::coeErrorToAbortCode({CoECode::InternalError}), SDOAbortCode::InternalError);
}

TEST(EcatMiscCov, CoEErrorToAbortCodePreservesSlaveAbort) {
    CoE::CoEError err{CoECode::Aborted, SDOAbortCode::InvalidValue, 0x06090030};
    EXPECT_EQ(CoE::coeErrorToAbortCode(err), SDOAbortCode::InvalidValue);
    err.abort_code = 0;
    EXPECT_EQ(CoE::coeErrorToAbortCode(err), SDOAbortCode::TransferAborted);
}

// ============================================================================
// SDOErrorDecoder
// ============================================================================

TEST(EcatMiscCov, SDODecoderAbortStr) {
    Raw::SDOErrorDecoder d;
    ASSERT_NE(d.sdoAbortCodeStr(0x06020000), nullptr);
    EXPECT_STREQ(d.sdoAbortCodeStr(0x06020000),
                 sdoAbortCodeStr(0x06020000));
    ASSERT_NE(d.sdoAbortCodeStr(0xDEADBEEF), nullptr);   // unknown code path
}

TEST(EcatMiscCov, SDODecoderMbxErrorCodes) {
    Raw::SDOErrorDecoder d;
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0000), "No error");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0001), "Syntax error in mailbox message");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0002), "Unsupported protocol");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0003), "Invalid channel");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0004), "Service not supported");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0005), "Invalid header");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0006), "Size too short");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0007), "No more memory");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0008), "Invalid size");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x0009), "Service in work");
    EXPECT_STREQ(d.mbxErrorCodeStr(0x00AA), "Unknown mailbox error");
}

TEST(EcatMiscCov, SDODecoderMbxDetailStr) {
    Raw::SDOErrorDecoder d;
    // errCode 0x0001: detail selects header region
    EXPECT_STREQ(d.mbxErrorDetailStr(0x0001, 0), "offset in mailbox header");
    EXPECT_STREQ(d.mbxErrorDetailStr(0x0001, 5), "offset in mailbox header");
    EXPECT_STREQ(d.mbxErrorDetailStr(0x0001, 6), "offset in CoE header");
    EXPECT_STREQ(d.mbxErrorDetailStr(0x0001, 7), "offset in CoE header");
    EXPECT_STREQ(d.mbxErrorDetailStr(0x0001, 8), "offset in SDO payload");
    EXPECT_STREQ(d.mbxErrorDetailStr(0x0001, 99), "offset in SDO payload");
    // Any other errCode → generic
    EXPECT_STREQ(d.mbxErrorDetailStr(0x0002, 0), "protocol-specific detail");
}

// ============================================================================
// Mailbox::Utils::logStatuswordDiagnostics
// ============================================================================

TEST(EcatMiscCov, StatuswordWarningRaisesAndClears) {
    bool warning_active = false;
    uint64_t first = 0;
    // Bit 7 → warning starts, first_cycle recorded
    Mailbox::Utils::logStatuswordDiagnostics(0x0080, warning_active, first, 10, "t");
    EXPECT_TRUE(warning_active);
    EXPECT_EQ(first, 10u);
    // Still set → stays active, no re-arm
    Mailbox::Utils::logStatuswordDiagnostics(0x0080, warning_active, first, 11, "t");
    EXPECT_TRUE(warning_active);
    EXPECT_EQ(first, 10u);
    // Cleared → warning_active reset
    Mailbox::Utils::logStatuswordDiagnostics(0x0000, warning_active, first, 12, "t");
    EXPECT_FALSE(warning_active);
}

TEST(EcatMiscCov, StatuswordErrorBitsModuloCycles) {
    bool warning_active = false;
    uint64_t first = 0;
    // Bit 13 (following error) logs every 500 cycles; bit 11 every 1000.
    // Cover both the modulo-hit and modulo-miss arms.
    Mailbox::Utils::logStatuswordDiagnostics(0x2000, warning_active, first, 500, "t");
    Mailbox::Utils::logStatuswordDiagnostics(0x2000, warning_active, first, 501, "t");
    Mailbox::Utils::logStatuswordDiagnostics(0x0800, warning_active, first, 1000, "t");
    Mailbox::Utils::logStatuswordDiagnostics(0x0800, warning_active, first, 1001, "t");
    Mailbox::Utils::logStatuswordDiagnostics(0x2800, warning_active, first, 0, "t");
    SUCCEED();
}

// ============================================================================
// NoDistributedClockConfigured — sentinel returns (logs CRITICAL to stderr)
// ============================================================================

TEST(EcatMiscCov, NoDCAllMethodsFail) {
    NoDistributedClockConfigured dc;
    EXPECT_FALSE(dc.start());
    EXPECT_FALSE(dc.start([] { return true; }));
    EXPECT_FALSE(dc.init());
    dc.stop();
    EXPECT_EQ(dc.getState(), DCState::Disabled);
    EXPECT_EQ(dc.getStats().cycle_count, 0u);
    dc.forceSync();
    dc.setPDOEnabled(true);
    EXPECT_FALSE(dc.isPDOEnabled());
    EXPECT_FALSE(dc.isSlaveSupported(0));
    EXPECT_EQ(dc.getSlaveOffset(0), 0);
    dc.readSyncConfig(0);
    EXPECT_FALSE(dc.reconfigureSync(0));
    EXPECT_FALSE(dc.synchronizeSlaveClock(0));
    EXPECT_EQ(dc.getMasterTimeNs(), 0u);
    uint8_t buf[4] = {};
    EXPECT_FALSE(dc.readRegister(0, DCRegisters::DCSysTime, buf, 4, 0));
    EXPECT_FALSE(dc.writeRegister(0, DCRegisters::DCSysTime, buf, 4, 0));
    EXPECT_FALSE(dc.sendSyncFrame());
}

}  // namespace
