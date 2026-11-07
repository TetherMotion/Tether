/**
 * @file test_misc_state_utils.cpp
 * @brief Coverage for CiA402StateUtils name/decode helpers, the ESIFile
 *        path constructor and PDOKeepAlive's background exchange loop.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "raw/CoEErrorStrings.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/ethercat/SlaveGroup.hpp"
#include "tether/profiles/cia402/CiA402Config.hpp"
#include "tether/profiles/cia402/CiA402StateUtils.hpp"
#include "tether/profiles/cia402/60xx-Parameters.hpp"

using namespace EtherCAT;

// ============================================================================
// CiA402StateUtils — name tables and decoders
// ============================================================================

TEST(StateUtilsCoverageTest, ECStateNames) {
    EXPECT_STREQ(getECStateName(ECState::Init), "INIT");
    EXPECT_STREQ(getECStateName(ECState::PreOp), "PRE_OP");
    EXPECT_STREQ(getECStateName(ECState::Bootstrap), "BOOTSTRAP");
    EXPECT_STREQ(getECStateName(ECState::SafeOp), "SAFE_OP");
    EXPECT_STREQ(getECStateName(ECState::Op), "OP");
    EXPECT_STREQ(getECStateName(static_cast<ECState>(0xFF)), "UNKNOWN");
}

TEST(StateUtilsCoverageTest, DriveStateNames) {
    EXPECT_STREQ(getDriveStateName(DriveState::NotReadyToSwitchOn),
                 "Not Ready to Switch On");
    EXPECT_STREQ(getDriveStateName(DriveState::SwitchOnDisabled), "Switch On Disabled");
    EXPECT_STREQ(getDriveStateName(DriveState::ReadyToSwitchOn), "Ready to Switch On");
    EXPECT_STREQ(getDriveStateName(DriveState::SwitchedOn), "Switched On");
    EXPECT_STREQ(getDriveStateName(DriveState::OperationEnabled), "Operation Enabled");
    EXPECT_STREQ(getDriveStateName(DriveState::QuickStopActive), "Quick Stop Active");
    EXPECT_STREQ(getDriveStateName(DriveState::FaultReactionActive),
                 "Fault Reaction Active");
    EXPECT_STREQ(getDriveStateName(DriveState::Fault), "Fault");
    EXPECT_STREQ(getDriveStateName(static_cast<DriveState>(0xEE)), "Unknown");
}

TEST(StateUtilsCoverageTest, DecodeDriveState) {
    EXPECT_EQ(decodeDriveState(0x0008), DriveState::Fault);
    EXPECT_EQ(decodeDriveState(0x000F), DriveState::FaultReactionActive);
    EXPECT_EQ(decodeDriveState(0x0000), DriveState::NotReadyToSwitchOn);
    EXPECT_EQ(decodeDriveState(0x0040), DriveState::SwitchOnDisabled);
    EXPECT_EQ(decodeDriveState(0x0021), DriveState::ReadyToSwitchOn);
    EXPECT_EQ(decodeDriveState(0x0023), DriveState::SwitchedOn);
    EXPECT_EQ(decodeDriveState(0x0027), DriveState::OperationEnabled);
    EXPECT_EQ(decodeDriveState(0x0007), DriveState::QuickStopActive);
    EXPECT_EQ(decodeDriveState(0xFFFF), DriveState::Unknown);
}

TEST(StateUtilsCoverageTest, OperatingModeNames) {
    EXPECT_STREQ(CiA402::getOperatingModeName(1), "PP");
    EXPECT_STREQ(CiA402::getOperatingModeName(2), "VL");
    EXPECT_STREQ(CiA402::getOperatingModeName(3), "PV");
    EXPECT_STREQ(CiA402::getOperatingModeName(4), "PT");
    EXPECT_STREQ(CiA402::getOperatingModeName(6), "HM");
    EXPECT_STREQ(CiA402::getOperatingModeName(7), "IP");
    EXPECT_STREQ(CiA402::getOperatingModeName(8), "CSP");
    EXPECT_STREQ(CiA402::getOperatingModeName(9), "CSV");
    EXPECT_STREQ(CiA402::getOperatingModeName(10), "CST");
    EXPECT_STREQ(CiA402::getOperatingModeName(0), "Unknown");
}

TEST(StateUtilsCoverageTest, ErrorNames) {
    EXPECT_STREQ(CiA402::errorToString(0x0000), "No Error");
    EXPECT_STREQ(CiA402::errorToString(0x1000), "Generic Error");
    EXPECT_STREQ(CiA402::errorToString(0x2310), "Overcurrent");
    EXPECT_STREQ(CiA402::errorToString(0x3210), "Overvoltage");
    EXPECT_STREQ(CiA402::errorToString(0x3220), "Undervoltage");
    EXPECT_STREQ(CiA402::errorToString(0x4210), "Overtemperature");
    EXPECT_STREQ(CiA402::errorToString(0x4310), "Supply Temperature Error");
    EXPECT_STREQ(CiA402::errorToString(0x5110), "Encoder Error");
    EXPECT_STREQ(CiA402::errorToString(0x5441), "Motor Blocked");
    EXPECT_STREQ(CiA402::errorToString(0x8611), "Following Error");
    EXPECT_STREQ(CiA402::errorToString(0x8612), "Position Limit");
    EXPECT_STREQ(CiA402::errorToString(0x8613), "Velocity Limit");
    EXPECT_STREQ(CiA402::errorToString(0x8100), "Communication Error");
    EXPECT_STREQ(CiA402::errorToString(0x8620), "Homing Error");
    EXPECT_STREQ(CiA402::errorToString(0xFFFF), "Unknown Error");
}

TEST(StateUtilsCoverageTest, FormatStatuswordDiagnostics) {
    char buf[512];
    const char* out = formatStatuswordDiagnostics(0x1637, buf, sizeof(buf));
    EXPECT_EQ(out, buf);
    EXPECT_NE(std::strstr(buf, "0x1637"), nullptr);
    EXPECT_EQ(formatStatuswordDiagnostics(0x1234, nullptr, 0), std::string(""));
    char tiny[4] = {};
    formatStatuswordDiagnostics(0xFFFF, tiny, sizeof(tiny));
}

TEST(StateUtilsCoverageTest, FormatStatuswordDiagnosticsBits) {
    char buf[512];
    formatStatuswordDiagnostics(0xFFFF, buf, sizeof(buf));
    EXPECT_NE(std::strstr(buf, "OpEn"), nullptr);
}

TEST(StateUtilsCoverageTest, StatuswordStateArms) {
    char buf[512];
    // SwitchOn arm: bit1 set, OpEn clear.
    formatStatuswordDiagnostics(0x0002, buf, sizeof(buf));
    EXPECT_NE(std::strstr(buf, "SwitchOn"), nullptr);
    // ReadyToSwitchOn arm: bit0 only.
    formatStatuswordDiagnostics(0x0021, buf, sizeof(buf));
    EXPECT_NE(std::strstr(buf, "ReadyToSO"), nullptr);
    // SwitchOnDisabled arm: bit6 only.
    formatStatuswordDiagnostics(0x0040, buf, sizeof(buf));
    EXPECT_NE(std::strstr(buf, "SODisabled"), nullptr);
    // NoVolt arm: bit4 clear + fault/warn/quickstop/remote extras.
    formatStatuswordDiagnostics(0x0000, buf, sizeof(buf));
    EXPECT_NE(std::strstr(buf, "NoVolt"), nullptr);
    EXPECT_NE(std::strstr(buf, "QSActive"), nullptr);
    formatStatuswordDiagnostics(0x0208, buf, sizeof(buf));
    EXPECT_NE(std::strstr(buf, "FAULT"), nullptr);
    EXPECT_NE(std::strstr(buf, "Remote"), nullptr);
    formatStatuswordDiagnostics(0x0080, buf, sizeof(buf));
    EXPECT_NE(std::strstr(buf, "WARN"), nullptr);
}


TEST(StateUtilsCoverageTest, SdoErrorDecoderTables) {
    Raw::SDOErrorDecoder dec;
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0000), "No error");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0001), "Syntax error in mailbox message");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0002), "Unsupported protocol");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0003), "Invalid channel");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0004), "Service not supported");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0005), "Invalid header");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0006), "Size too short");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0007), "No more memory");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0008), "Invalid size");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0x0009), "Service in work");
    EXPECT_STREQ(dec.mbxErrorCodeStr(0xDEAD), "Unknown mailbox error");

    EXPECT_STREQ(dec.mbxErrorDetailStr(0x0001, 3), "offset in mailbox header");
    EXPECT_STREQ(dec.mbxErrorDetailStr(0x0001, 7), "offset in CoE header");
    EXPECT_STREQ(dec.mbxErrorDetailStr(0x0001, 9), "offset in SDO payload");
    EXPECT_STREQ(dec.mbxErrorDetailStr(0x0002, 3), "protocol-specific detail");

    EXPECT_NE(dec.sdoAbortCodeStr(0x06020000), nullptr);
}

// ============================================================================
// PDOKeepAlive
// ============================================================================

class KeepAliveCoverageTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = std::make_unique<Emulator::SlaveEmulator>();
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    EmulatedBus bus_;
    Master master_;
    Emulator::SlaveEmulator* slave_ = nullptr;
};

TEST_F(KeepAliveCoverageTest, StartKeepAliveRunsExchangeLoop) {
    auto keepalive = master_.pdo().startKeepAlive(std::chrono::milliseconds{5});
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    keepalive->stop();   // explicit stop exercises the join path
}

TEST_F(KeepAliveCoverageTest, DestructorStopsThread) {
    {
        auto keepalive = master_.pdo().startKeepAlive(std::chrono::milliseconds{5});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }  // destructor stops the worker
}

// ============================================================================
// CoEErrorStrings — coeErrorStr + coeErrorToAbortCode
// ============================================================================

TEST(StateUtilsCoverageTest, CoeErrorStrAllCodes) {
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::Ok}), "Ok");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::Timeout}), "Timeout");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::Aborted}), "Aborted");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::TransportError}),
                 "Transport error");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::QueueFull}),
                 "Queue full");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::NotConfigured}),
                 "Mailbox not configured");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::ShuttingDown}),
                 "Shutting down");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::SlaveNotFound}),
                 "Slave not found");
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError{CoE::CoEErrorCode::InternalError}),
                 "Internal error");
    EXPECT_STREQ(CoE::coeErrorStr(
                     CoE::CoEError{static_cast<CoE::CoEErrorCode>(0xFE)}),
                 "Unknown error");
    // An abort carrying a real SDO abort code decodes to its description.
    EXPECT_STREQ(CoE::coeErrorStr(CoE::CoEError::aborted(0x06020000u)),
                 "Object does not exist");
}

TEST(StateUtilsCoverageTest, CoeErrorToAbortCodeMapping) {
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::Ok}),
              SDOAbortCode::Success);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::Timeout}),
              SDOAbortCode::Timeout);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::NotConfigured}),
              SDOAbortCode::DeviceStateError);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::TransportError}),
              SDOAbortCode::GeneralError);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::QueueFull}),
              SDOAbortCode::OutOfMemory);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::ShuttingDown}),
              SDOAbortCode::DeviceStateError);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::SlaveNotFound}),
              SDOAbortCode::ObjectNotFound);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::InternalError}),
              SDOAbortCode::InternalError);
    // Aborted without a raw code falls back to TransferAborted; with one,
    // the slave's real code is preserved.
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError{CoE::CoEErrorCode::Aborted}),
              SDOAbortCode::TransferAborted);
    EXPECT_EQ(CoE::coeErrorToAbortCode(CoE::CoEError::aborted(0x06020000u)),
              SDOAbortCode::ObjectNotFound);
}
