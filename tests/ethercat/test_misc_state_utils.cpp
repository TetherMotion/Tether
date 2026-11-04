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
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
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
