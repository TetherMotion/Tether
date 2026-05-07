/**
 * @file test_EtherCATReset_coverage.cpp
 * @brief Coverage tests for EtherCATReset: free functions, structs, enum helpers
 *        Tests that don't require SDOManager (ResetResult, ResetPolicy, name functions)
 */

#include "tether/ethercat/EtherCATReset.hpp"
#include <gtest/gtest.h>
#include <cstring>

using namespace EtherCAT;

// ============================================================================
// ResetLevel name/description
// ============================================================================

TEST(ResetLevelCovTest, AllNames) {
    EXPECT_NE(getResetLevelName(ResetLevel::SoftReset), nullptr);
    EXPECT_NE(getResetLevelName(ResetLevel::CommunicationReset), nullptr);
    EXPECT_NE(getResetLevelName(ResetLevel::ApplicationReset), nullptr);
    EXPECT_NE(getResetLevelName(ResetLevel::StateMachineReset), nullptr);
    EXPECT_NE(getResetLevelName(ResetLevel::ESCHardwareReset), nullptr);
    EXPECT_NE(getResetLevelName(ResetLevel::HardwareReset), nullptr);
    // Unknown
    EXPECT_NE(getResetLevelName(static_cast<ResetLevel>(99)), nullptr);
}

TEST(ResetLevelCovTest, NamesAreDistinct) {
    EXPECT_STRNE(getResetLevelName(ResetLevel::SoftReset),
                 getResetLevelName(ResetLevel::HardwareReset));
}

TEST(ResetLevelCovTest, AllDescriptions) {
    EXPECT_NE(getResetLevelDescription(ResetLevel::SoftReset), nullptr);
    EXPECT_NE(getResetLevelDescription(ResetLevel::CommunicationReset), nullptr);
    EXPECT_NE(getResetLevelDescription(ResetLevel::ApplicationReset), nullptr);
    EXPECT_NE(getResetLevelDescription(ResetLevel::StateMachineReset), nullptr);
    EXPECT_NE(getResetLevelDescription(ResetLevel::ESCHardwareReset), nullptr);
    EXPECT_NE(getResetLevelDescription(ResetLevel::HardwareReset), nullptr);
    EXPECT_NE(getResetLevelDescription(static_cast<ResetLevel>(99)), nullptr);
}

TEST(ResetLevelCovTest, DescriptionsAreDistinct) {
    EXPECT_STRNE(getResetLevelDescription(ResetLevel::SoftReset),
                 getResetLevelDescription(ResetLevel::HardwareReset));
}

// ============================================================================
// ALStatusCode names
// ============================================================================

TEST(ALStatusCodeCovTest, KnownCodes) {
    EXPECT_NE(getALStatusCodeName(ALStatusCode::NoError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::UnspecifiedError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::NoMemory), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidStateChange), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::UnknownStateRequested), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::BootNotSupported), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::NoValidFirmware), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidMailboxConfig), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidSMConfig), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::NoValidInputs), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::NoValidOutputs), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::SyncError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::SMWatchdog), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidSMTypes), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidOutputConfig), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidInputConfig), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::SlaveNeedsColdStart), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::SlaveNeedsInit), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::SlaveNeedsPreOp), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::SlaveNeedsSafeOp), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidInputMapping), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidOutputMapping), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InconsistentSettings), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::FreeRunNotSupported), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::SyncNotSupported), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::BackgroundWatchdog), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::NoValidInputsOutputs), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::FatalSyncError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::InvalidDCCycleTime), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::MBXAoeError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::MBXCoeError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::MBXFoeError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::MBXVoeError), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::VendorSpecificStart), nullptr);
}

TEST(ALStatusCodeCovTest, UnknownCode) {
    EXPECT_NE(getALStatusCodeName(0xFFFF), nullptr);
}

// ============================================================================
// ResetResult struct
// ============================================================================

TEST(ResetResultCovTest, DefaultValues) {
    ResetResult r{};
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.al_status, 0u);
    EXPECT_EQ(r.al_status_code, 0u);
    EXPECT_EQ(r.duration_us, 0u);
}

TEST(ResetResultCovTest, IsComplete) {
    ResetResult r{};
    r.success = true;
    r.requested_level = ResetLevel::SoftReset;
    r.achieved_level = ResetLevel::SoftReset;
    EXPECT_TRUE(r.isComplete());
}

TEST(ResetResultCovTest, IsPartial) {
    ResetResult r{};
    r.success = true;
    r.requested_level = ResetLevel::HardwareReset;
    r.achieved_level = ResetLevel::SoftReset;
    EXPECT_TRUE(r.isPartial());
}

TEST(ResetResultCovTest, FailedIsNotComplete) {
    ResetResult r{};
    r.success = false;
    r.requested_level = ResetLevel::SoftReset;
    r.achieved_level = ResetLevel::SoftReset;
    EXPECT_FALSE(r.isComplete());
}

TEST(ResetResultCovTest, ErrorMessage) {
    ResetResult r{};
    r.error_message = "Test error";
    EXPECT_EQ(r.error_message, "Test error");
}

// ============================================================================
// ResetPolicy struct
// ============================================================================

TEST(ResetPolicyCovTest, DefaultValues) {
    ResetPolicy p{};
    EXPECT_EQ(p.max_auto_attempts, 3u);
    EXPECT_EQ(p.retry_delay_ms, 100u);
    EXPECT_TRUE(p.escalate_on_failure);
    EXPECT_TRUE(p.auto_retry_transient);
    EXPECT_FALSE(p.auto_reenable_drive);
}

TEST(ResetPolicyCovTest, CustomValues) {
    ResetPolicy p{};
    p.max_auto_attempts = 5;
    p.retry_delay_ms = 500;
    p.escalate_on_failure = false;
    p.starting_level = ResetLevel::CommunicationReset;
    p.max_level = ResetLevel::ESCHardwareReset;
    p.auto_retry_transient = false;
    p.auto_reenable_drive = true;
    
    EXPECT_EQ(p.max_auto_attempts, 5u);
    EXPECT_FALSE(p.escalate_on_failure);
    EXPECT_EQ(p.starting_level, ResetLevel::CommunicationReset);
    EXPECT_TRUE(p.auto_reenable_drive);
}

TEST(ResetPolicyCovTest, ShouldContinueCallback) {
    ResetPolicy p{};
    int callCount = 0;
    p.should_continue = [&](const ResetResult&, uint8_t attempt) -> bool {
        callCount++;
        return attempt < 3;
    };
    
    ResetResult r{};
    EXPECT_TRUE(p.should_continue(r, 1));
    EXPECT_TRUE(p.should_continue(r, 2));
    EXPECT_FALSE(p.should_continue(r, 3));
    EXPECT_EQ(callCount, 3);
}

// ============================================================================
// ALControl namespace constants
// ============================================================================

TEST(ALControlCovTest, Constants) {
    EXPECT_EQ(ALControl::StateMask, 0x000F);
    EXPECT_EQ(ALControl::AckError, 0x0010);
    EXPECT_EQ(ALControl::RequestId, 0x0020);
    EXPECT_EQ(ALControl::ESCReset, 0x0040);
}

// ============================================================================
// CiA301Reset namespace constants
// ============================================================================

TEST(CiA301ResetCovTest, Constants) {
    EXPECT_EQ(CiA301Reset::StoreParameters, 0x1010);
    EXPECT_EQ(CiA301Reset::RestoreParameters, 0x1011);
    EXPECT_EQ(CiA301Reset::AllParameters, 0x01);
    EXPECT_EQ(CiA301Reset::CommunicationParams, 0x02);
    EXPECT_EQ(CiA301Reset::ApplicationParams, 0x03);
    EXPECT_EQ(CiA301Reset::ManufacturerParams, 0x04);
}

// ============================================================================
// CiA402Reset namespace constants
// ============================================================================

TEST(CiA402ResetCovTest, Constants) {
    EXPECT_EQ(CiA402Reset::FaultReset, 0x0080);
    EXPECT_EQ(CiA402Reset::Halt, 0x0100);
    EXPECT_EQ(CiA402Reset::QuickStopActive, 0x0000);
    EXPECT_EQ(CiA402Reset::QuickStopInactive, 0x0004);
    EXPECT_EQ(CiA402Reset::EnableOperation, 0x0008);
    EXPECT_EQ(CiA402Reset::SwitchOn, 0x0001);
    EXPECT_EQ(CiA402Reset::EnableVoltage, 0x0002);
}

// ============================================================================
// ALState enum values
// ============================================================================

TEST(ALStateCovTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(ALState::Init), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(ALState::PreOp), 0x02);
    EXPECT_EQ(static_cast<uint8_t>(ALState::Bootstrap), 0x03);
    EXPECT_EQ(static_cast<uint8_t>(ALState::SafeOp), 0x04);
    EXPECT_EQ(static_cast<uint8_t>(ALState::Op), 0x08);
    EXPECT_EQ(static_cast<uint8_t>(ALState::ErrorFlag), 0x10);
}

// ============================================================================
// NMTCommand enum values
// ============================================================================

TEST(NMTCommandCovTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::StartNode), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::StopNode), 0x02);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::EnterPreOp), 0x80);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::ResetNode), 0x81);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::ResetCommunication), 0x82);
}

// ============================================================================
// CiA402State enum (in reset context)
// ============================================================================

TEST(CiA402ResetStateCovTest, Values) {
    // These are status word patterns, not sequential values
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::NotReadyToSwitchOn), 0x00);
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::SwitchOnDisabled), 0x40);
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::ReadyToSwitchOn), 0x21);
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::SwitchedOn), 0x23);
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::OperationEnabled), 0x27);
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::QuickStopActive), 0x07);
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::FaultReactionActive), 0x0F);
    EXPECT_EQ(static_cast<uint8_t>(CiA402State::Fault), 0x08);
}

// ============================================================================
// ResetLevel enum values
// ============================================================================

TEST(ResetLevelCovTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(ResetLevel::SoftReset), 0);
    EXPECT_EQ(static_cast<uint8_t>(ResetLevel::CommunicationReset), 1);
    EXPECT_EQ(static_cast<uint8_t>(ResetLevel::ApplicationReset), 2);
    EXPECT_EQ(static_cast<uint8_t>(ResetLevel::StateMachineReset), 3);
    EXPECT_EQ(static_cast<uint8_t>(ResetLevel::ESCHardwareReset), 4);
    EXPECT_EQ(static_cast<uint8_t>(ResetLevel::HardwareReset), 5);
}
