/**
 * @file test_fault_names_coverage.cpp
 * @brief Tests to achieve 100% coverage on getALStatusCodeName, getCiA402ErrorCodeName,
 *        ManufacturerFault::parse/format, and al_status_get_state_name.
 */
#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include <cstring>

using namespace EtherCAT;

// ============================================================================
// getALStatusCodeName — every enum case
// ============================================================================

TEST(ALStatusCodeNames, AllEnumCases) {
    // Each case returns a non-null, non-empty string
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::NoError), "No error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::UnspecifiedError), "Unspecified error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::NoMemory), "No memory");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidRequestedStateChange), "Invalid requested state change");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::UnknownRequestedState), "Unknown requested state");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::BootstrapNotSupported), "Bootstrap not supported");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::NoValidFirmware), "No valid firmware");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidMailboxConfig), "Invalid mailbox configuration");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidMailboxConfigPreOp), "Invalid mailbox configuration (PRE_OP)");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidSyncManagerConfig), "Invalid Sync Manager configuration");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::NoValidInputs), "No valid inputs available");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::NoValidOutputs), "No valid outputs");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SynchronizationError), "Synchronization error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SyncManagerWatchdog), "Sync Manager watchdog");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidSyncManagerTypes), "Invalid Sync Manager types");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidOutputConfig), "Invalid output configuration");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidInputConfig), "Invalid input configuration");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidWatchdogConfig), "Invalid watchdog configuration");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SlaveNeedsColdStart), "Slave needs cold start");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SlaveNeedsInit), "Slave needs INIT");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SlaveNeedsPreOp), "Slave needs PRE_OP");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SlaveNeedsSafeOp), "Slave needs SAFE_OP");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidInputMapping), "Invalid input mapping");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidOutputMapping), "Invalid output mapping");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InconsistentSettings), "Inconsistent settings");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::FreeRunNotSupported), "FreeRun not supported");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SyncModeNotSupported), "Sync mode not supported");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::FreeRunNeeds3BufferMode), "FreeRun needs 3-buffer mode");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::BackgroundWatchdog), "Background watchdog");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::NoValidInputsAndOutputs), "No valid inputs and outputs");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::FatalSyncError), "Fatal sync error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::NoSyncError), "No sync error (Err74.1)");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidDCConfig), "Invalid DC configuration");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidDCSyncUnit), "Invalid DC sync unit");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidDCCycleTime), "Invalid DC cycle time");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidDCLatchConfig), "Invalid DC latch configuration");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::PLLError), "PLL error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::DCSync1CycleTime), "DC SYNC1 cycle time");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::MBoxEoE), "Mailbox EoE error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::MBoxCoE), "Mailbox CoE error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::MBoxFoE), "Mailbox FoE error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::MBoxSoE), "Mailbox SoE error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::MBoxVoE), "Mailbox VoE error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::EEPROMNoAccess), "EEPROM no access");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::EEPROMError), "EEPROM error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::ExternalHardwareNotReady), "External hardware not ready");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::SlaveRestartedLocally), "Slave restarted locally");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::DeviceIdUpdateError), "Device ID update error");
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::ApplicationControllerAvail), "Application controller available");
}

TEST(ALStatusCodeNames, DefaultCase) {
    EXPECT_STREQ(getALStatusCodeName(static_cast<ALStatusCode>(0xFFFF)), "Unknown error");
}

TEST(ALStatusCodeNames, Uint16Overload) {
    EXPECT_STREQ(getALStatusCodeName(static_cast<uint16_t>(0x0000)), "No error");
    EXPECT_STREQ(getALStatusCodeName(static_cast<uint16_t>(0xFFFF)), "Unknown error");
}

// ============================================================================
// getCiA402ErrorCodeName — every enum case
// ============================================================================

TEST(CiA402ErrorCodeNames, AllEnumCases) {
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::NoError), "No error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::GenericError), "Generic error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverCurrent), "Over current");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverCurrentInternal), "Over current internal");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverCurrentOutputA), "Over current output A");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverCurrentOutputB), "Over current output B");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverVoltage), "Over voltage");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverVoltageSupply), "Over voltage supply");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::UnderVoltage), "Under voltage");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::UnderVoltageSupply), "Under voltage supply");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverTemperature), "Over temperature");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverTemperatureMotor), "Over temperature motor");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverTemperatureDrive), "Over temperature drive");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::SupplyVoltageFailure), "Supply voltage failure");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::InternalSupplyFailed), "Internal supply failed");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OutputStageProtection), "Output stage protection");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::PositionLimitExceeded), "Position limit exceeded");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::PositionSensorError), "Position sensor error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::EncoderError), "Encoder error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::FollowingError), "Following error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::VelocityTooHigh), "Velocity too high");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::ExternalError), "External error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::SoftwareError), "Software error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::SoftwareReset), "Software reset");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::ObjectDictionaryError), "Object dictionary error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::ObjectDictionaryMissing), "Object dictionary missing");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::CANopenError), "CANopen error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::PDOLengthError), "PDO length error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::EmergencyBufferFull), "Emergency buffer full");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::CommWatchdogError), "Communication watchdog error");
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::CommError), "Communication error");
}

TEST(CiA402ErrorCodeNames, DefaultCase) {
    EXPECT_STREQ(getCiA402ErrorCodeName(static_cast<CiA402ErrorCode>(0x9999)), "Unknown CiA 402 error");
}

TEST(CiA402ErrorCodeNames, Uint16Overload) {
    EXPECT_STREQ(getCiA402ErrorCodeName(static_cast<uint16_t>(0x0000)), "No error");
    EXPECT_STREQ(getCiA402ErrorCodeName(static_cast<uint16_t>(0x9999)), "Unknown CiA 402 error");
}

// ============================================================================
// ManufacturerFault::parse — all known codes + default + class decomposition
// ============================================================================

TEST(ManufacturerFaultParse, KnownCodes) {
    auto f741 = ManufacturerFault::parse(741, 0, 0);
    EXPECT_EQ(f741.raw_code, 741u);
    EXPECT_STREQ(f741.description, "No Sync (Err74.1)");
    EXPECT_EQ(f741.class_code, 74);
    EXPECT_EQ(f741.sub_code, 1);

    EXPECT_STREQ(ManufacturerFault::parse(740, 0, 0).description, "DC Sync Error (Err74.0)");
    EXPECT_STREQ(ManufacturerFault::parse(200, 0, 0).description, "Over current");
    EXPECT_STREQ(ManufacturerFault::parse(201, 0, 0).description, "Over current (output A)");
    EXPECT_STREQ(ManufacturerFault::parse(202, 0, 0).description, "Over current (output B)");
    EXPECT_STREQ(ManufacturerFault::parse(300, 0, 0).description, "Over voltage");
    EXPECT_STREQ(ManufacturerFault::parse(310, 0, 0).description, "Under voltage");
    EXPECT_STREQ(ManufacturerFault::parse(400, 0, 0).description, "Over temperature");
    EXPECT_STREQ(ManufacturerFault::parse(410, 0, 0).description, "Motor over temperature");
    EXPECT_STREQ(ManufacturerFault::parse(500, 0, 0).description, "Encoder error");
    EXPECT_STREQ(ManufacturerFault::parse(510, 0, 0).description, "Encoder loss");
    EXPECT_STREQ(ManufacturerFault::parse(600, 0, 0).description, "Following error");
    EXPECT_STREQ(ManufacturerFault::parse(610, 0, 0).description, "Velocity error");
    EXPECT_STREQ(ManufacturerFault::parse(700, 0, 0).description, "Communication error");
    EXPECT_STREQ(ManufacturerFault::parse(710, 0, 0).description, "CAN error");
    EXPECT_STREQ(ManufacturerFault::parse(720, 0, 0).description, "EtherCAT error");
}

TEST(ManufacturerFaultParse, DefaultCode) {
    auto f = ManufacturerFault::parse(999, 0, 0);
    EXPECT_STREQ(f.description, "Unknown manufacturer fault");
}

TEST(ManufacturerFaultParse, SmallCode) {
    // Code < 100: class_code = raw_code, sub_code = 0
    auto f = ManufacturerFault::parse(42, 0, 0);
    EXPECT_EQ(f.class_code, 42);
    EXPECT_EQ(f.sub_code, 0);
}

// ============================================================================
// ManufacturerFault::format — both branches
// ============================================================================

TEST(ManufacturerFaultFormat, WithSubCode) {
    ManufacturerFault f = {};
    f.class_code = 74;
    f.sub_code = 1;
    char buf[32];
    size_t n = f.format(buf, sizeof(buf));
    EXPECT_GT(n, 0u);
    EXPECT_STREQ(buf, "Err74.1");
}

TEST(ManufacturerFaultFormat, WithoutSubCode) {
    ManufacturerFault f = {};
    f.class_code = 20;
    f.sub_code = 0;
    char buf[32];
    size_t n = f.format(buf, sizeof(buf));
    EXPECT_GT(n, 0u);
    EXPECT_STREQ(buf, "Err20");
}

TEST(ManufacturerFaultFormat, NullBuffer) {
    ManufacturerFault f = {};
    EXPECT_EQ(f.format(nullptr, 0), 0u);
    char buf[4];
    EXPECT_EQ(f.format(buf, 0), 0u);
}

// ============================================================================
// al_status_get_state_name — all cases
// ============================================================================

TEST(ALStatusStateName, AllStates) {
    EXPECT_STREQ(al_status_get_state_name(0x0001), "INIT");
    EXPECT_STREQ(al_status_get_state_name(0x0002), "PRE_OP");
    EXPECT_STREQ(al_status_get_state_name(0x0003), "BOOTSTRAP");
    EXPECT_STREQ(al_status_get_state_name(0x0004), "SAFE_OP");
    EXPECT_STREQ(al_status_get_state_name(0x0008), "OP");
    EXPECT_STREQ(al_status_get_state_name(0x0000), "UNKNOWN");
    EXPECT_STREQ(al_status_get_state_name(0x0005), "UNKNOWN");
    // With error bit set (0x0010) — only lower nibble matters
    EXPECT_STREQ(al_status_get_state_name(0x0011), "INIT");
    EXPECT_STREQ(al_status_get_state_name(0x0018), "OP");
}
