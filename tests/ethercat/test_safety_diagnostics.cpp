/**
 * @file test_safety_diagnostics.cpp
 * @brief Coverage for Synapticon SafetyDiagnostics.hpp — the pure decoders
 *        and report accessors, plus an emulator-backed end-to-end run of
 *        runFullSafetyDiagnostics() against a populated object dictionary.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/drives/Synapticon/SafetyDiagnostics.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Drives::Synapticon;
using EtherCAT::Emulator::SlaveEmulator;

// ============================================================================
// Pure decoders — no bus required
// ============================================================================

TEST(SafetyDiagnostics, CiA402StateNameCoversAllStates) {
    EXPECT_STREQ(cia402StateName(0x0000), "Not ready to switch on");
    EXPECT_STREQ(cia402StateName(0x0040), "Switch on disabled");
    EXPECT_STREQ(cia402StateName(0x0021), "Ready to switch on");
    EXPECT_STREQ(cia402StateName(0x0023), "Switched on");
    EXPECT_STREQ(cia402StateName(0x0027), "Operation enabled");
    EXPECT_STREQ(cia402StateName(0x0007), "Quick stop active");
    EXPECT_STREQ(cia402StateName(0x000F), "Fault reaction active");
    EXPECT_STREQ(cia402StateName(0x0008), "Fault");
    EXPECT_STREQ(cia402StateName(0x006F), "Unknown");
    // High bits are masked out.
    EXPECT_STREQ(cia402StateName(0xFF27), "Operation enabled");
}

TEST(SafetyDiagnostics, ModuleIdentDesc) {
    EXPECT_STREQ(moduleIdentDesc(kModuleIdentNoParam),
                 "No parameter changes via master (params from OBLAC)");
    EXPECT_STREQ(moduleIdentDesc(kModuleIdentWithParam),
                 "With parameter changes via master");
    EXPECT_STREQ(moduleIdentDesc(0xDEADBEEF), "Unknown module ident");
}

TEST(SafetyDiagnostics, KnownErrorReportDecodes) {
    EXPECT_STREQ(knownErrorReport(nullptr), "unknown");
    EXPECT_STREQ(knownErrorReport("NoFault "), "No fault");
    EXPECT_NE(std::strstr(knownErrorReport("SmmNoCfg"), "not configured"),
              nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SmmFIO25"), "Black channel"),
              nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SmmFIO26"), "timeout"), nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SmmFIO27"), "CRC"), nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SmmFIO28"), "ConnectionID"),
              nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SmmFIO29"), "command"), nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SmmFIO30"), "watchdog"), nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SmmFIO31"), "address"), nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("CyclicHb"), "heartbeat"), nullptr);
    EXPECT_NE(std::strstr(knownErrorReport("SynDifHi"), "Sync"), nullptr);
    EXPECT_EQ(knownErrorReport("Whatever"), nullptr);
}

TEST(SafetyDiagnostics, FsoeCommandNameDelegates) {
    // Value 0 is a valid command name lookup through FSoEHelpers.
    EXPECT_NE(fsoeCommandName(kFSoECmdData), nullptr);
    EXPECT_NE(fsoeCommandName(0xFF), nullptr);
}

TEST(SafetyDiagnostics, SafetyModuleStateAccessors) {
    SafetyModuleState s;
    EXPECT_TRUE(s.isInSafeState());     // inputs default 0
    EXPECT_FALSE(s.motionAllowed());
    EXPECT_STREQ(s.stateSummary(), "unknown (SDO read failed)");
    EXPECT_STREQ(s.fsoeStateSummary(), "unknown (SDO read failed)");

    s.ok = true;
    s.input1 = 1;
    s.input2 = 0;
    EXPECT_TRUE(s.isInSafeState());
    EXPECT_FALSE(s.motionAllowed());
    EXPECT_STREQ(s.stateSummary(), "SAFE STATE (motion inhibited)");

    s.input2 = 1;
    EXPECT_FALSE(s.isInSafeState());
    EXPECT_TRUE(s.motionAllowed());
    EXPECT_STREQ(s.stateSummary(), "operational (motion allowed)");

    s.fsoe_read_ok = true;
    s.safe_fieldbus = 1;
    EXPECT_TRUE(s.fsoeActive());
    EXPECT_STREQ(s.fsoeStateSummary(), "active");

    s.safe_fieldbus = 0;
    EXPECT_FALSE(s.fsoeActive());
    EXPECT_STREQ(s.fsoeStateSummary(), "inactive");
}

TEST(SafetyDiagnostics, ReportAccessors) {
    SafetyDiagnosticReport r;

    // hasFault requires a successful read and a non-NoFault code.
    EXPECT_FALSE(r.hasFault());
    r.error_report_ok = true;
    std::strcpy(r.error_report, "NoFault ");
    EXPECT_FALSE(r.hasFault());
    std::strcpy(r.error_report, "SmmFIO25");
    EXPECT_TRUE(r.hasFault());

    // isInSafeState requires successful input reads.
    EXPECT_FALSE(r.isInSafeState());
    r.inputs_ok = true;
    r.input1 = 0;
    EXPECT_TRUE(r.isInSafeState());
    r.input1 = 1;
    r.input2 = 1;
    EXPECT_FALSE(r.isInSafeState());

    // fsoeActive requires a successful general-safety read.
    EXPECT_FALSE(r.fsoeActive());
    r.general_safety_ok = true;
    r.safe_fieldbus = 1;
    EXPECT_TRUE(r.fsoeActive());

    // moduleIdentMismatch requires successful ident reads and unequal values.
    EXPECT_FALSE(r.moduleIdentMismatch());
    r.module_ident_ok = true;
    r.configured_ident_pos2 = kModuleIdentNoParam;
    r.detected_ident_pos2 = kModuleIdentNoParam;
    EXPECT_FALSE(r.moduleIdentMismatch());
    r.detected_ident_pos2 = kModuleIdentWithParam;
    EXPECT_TRUE(r.moduleIdentMismatch());

    // connIdMismatch requires both address reads and unequal values.
    EXPECT_FALSE(r.connIdMismatch());
    r.safety_addr_ok = true;
    r.device_safety_address = 0x1234;
    r.safe_address = 0x1234;
    EXPECT_FALSE(r.connIdMismatch());
    r.safe_address = 0x5678;
    EXPECT_TRUE(r.connIdMismatch());
}

// ============================================================================
// Emulator-backed run of runFullSafetyDiagnostics
// ============================================================================

namespace {

class SafetyDiagnosticsBusTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000F0A, 0x9900, "safety");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    }

    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    Slave& sl() { return master_.slave(0); }

    void addU8(uint16_t index, uint8_t sub, uint8_t value) {
        slave_->addODEntry({index, sub, 0x0005, 8, 0x03,
                            {value}, "t"});
    }

    void addU16(uint16_t index, uint8_t sub, uint16_t value) {
        slave_->addODEntry({index, sub, 0x0006, 16, 0x03,
                            {static_cast<uint8_t>(value & 0xFF),
                             static_cast<uint8_t>(value >> 8)}, "t"});
    }

    void addU32(uint16_t index, uint8_t sub, uint32_t value) {
        slave_->addODEntry({index, sub, 0x0007, 32, 0x03,
                            {static_cast<uint8_t>(value & 0xFF),
                             static_cast<uint8_t>((value >> 8) & 0xFF),
                             static_cast<uint8_t>((value >> 16) & 0xFF),
                             static_cast<uint8_t>((value >> 24) & 0xFF)}, "t"});
    }

    void addString(uint16_t index, uint8_t sub, const char* value) {
        std::vector<uint8_t> data(value, value + std::strlen(value));
        slave_->addODEntry({index, sub, 0x0009,
                            static_cast<uint16_t>(data.size() * 8), 0x03,
                            std::move(data), "t"});
    }

    /// Populate every object runFullSafetyDiagnostics reads with a
    /// "healthy" configuration (FSoE active, no fault, matching addresses).
    void populateHealthy() {
        addU8(0x2611, 0x01, 1);            // safety input 1 released
        addU8(0x2611, 0x02, 1);            // safety input 2 released
        addU8(0x2620, 0x02, 1);            // safe fieldbus: FSoE active
        addU16(0x2620, 0x03, 0x1234);      // safe address
        addU8(0x6621, 0x01, 0x00);         // safety status byte 1
        addU8(0x6621, 0x02, 0xE0);         // safety status byte 2 (params valid)
        addString(0x203F, 0x01, "NoFault ");
        // 0x603F / 0x6041 / 0x1001 exist in the emulator's built-in CiA402
        // OD and are computed from its drive state — not overridable here.
        addU8(0x6770, 0x01, kFSoECmdData); // master frame cmd
        addU16(0x6770, 0x02, 0x1234);      // master ConnID
        addU16(0x6770, 0x03, 0xAAAA);      // CRC0
        addU16(0x6770, 0x04, 0xBBBB);      // CRC1
        addU8(0x6760, 0x01, kFSoECmdData); // slave frame cmd
        addU16(0x6760, 0x02, 0x1234);      // slave ConnID
        addU32(0xF030, 0x02, kModuleIdentNoParam);
        addU32(0xF050, 0x02, kModuleIdentNoParam);
        addU16(0xF980, 0x01, 0x1234);      // device safety address
        addU8(0x1001, 0x00, 0x00);         // error register
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
};

} // namespace

TEST_F(SafetyDiagnosticsBusTest, HealthyDriveReportIsFullyConsistent) {
    populateHealthy();
    auto rpt = runFullSafetyDiagnostics(sl());

    EXPECT_TRUE(rpt.inputs_ok);
    EXPECT_EQ(rpt.input1, 1);
    EXPECT_EQ(rpt.input2, 1);
    EXPECT_TRUE(rpt.general_safety_ok);
    EXPECT_EQ(rpt.safe_fieldbus, 1);
    EXPECT_EQ(rpt.safe_address, 0x1234);
    EXPECT_TRUE(rpt.statusword_ok);
    EXPECT_EQ(rpt.status_byte1, 0x00);
    EXPECT_EQ(rpt.status_byte2, 0xE0);
    ASSERT_TRUE(rpt.error_report_ok);
    EXPECT_STREQ(rpt.error_report, "NoFault ");
    EXPECT_FALSE(rpt.hasFault());
    EXPECT_TRUE(rpt.cia402_error_ok);
    EXPECT_EQ(rpt.cia402_error_code, 0);
    EXPECT_TRUE(rpt.cia402_status_ok);
    EXPECT_TRUE(rpt.master_frame_ok);
    EXPECT_EQ(rpt.master_cmd, kFSoECmdData);
    EXPECT_EQ(rpt.master_conn_id, 0x1234);
    EXPECT_TRUE(rpt.slave_frame_ok);
    EXPECT_TRUE(rpt.module_ident_ok);
    EXPECT_FALSE(rpt.moduleIdentMismatch());
    EXPECT_TRUE(rpt.safety_addr_ok);
    EXPECT_FALSE(rpt.connIdMismatch());
    EXPECT_TRUE(rpt.error_reg_ok);
    EXPECT_EQ(rpt.error_register, 0);
    EXPECT_TRUE(rpt.fsoeActive());
    EXPECT_FALSE(rpt.isInSafeState());
}

TEST_F(SafetyDiagnosticsBusTest, FaultedDriveReportDecodesFlags) {
    populateHealthy();
    // Overwrite select entries with a faulted configuration.
    addU8(0x2611, 0x01, 0);                 // input 1 in safe state
    addString(0x203F, 0x01, "SmmFIO30");    // watchdog timeout
    addU8(0x6770, 0x01, 0x00);              // no FSoE frames received
    addU8(0x6760, 0x01, 0x00);              // slave produces nothing
    addU16(0x6760, 0x02, 0x0000);
    addU32(0xF050, 0x02, kModuleIdentWithParam);  // ident mismatch
    addU16(0xF980, 0x01, 0x9999);           // address mismatch
    addU8(0x1001, 0x00, 0x01);              // generic error bit

    auto rpt = runFullSafetyDiagnostics(sl());

    EXPECT_TRUE(rpt.isInSafeState());
    ASSERT_TRUE(rpt.error_report_ok);
    EXPECT_TRUE(rpt.hasFault());
    EXPECT_STREQ(rpt.error_report, "SmmFIO30");
    EXPECT_TRUE(rpt.cia402_error_ok);
    EXPECT_TRUE(rpt.cia402_status_ok);
    EXPECT_TRUE(rpt.master_frame_ok);
    EXPECT_EQ(rpt.master_cmd, 0);
    EXPECT_EQ(rpt.slave_cmd, 0);
    EXPECT_EQ(rpt.slave_conn_id, 0);
    EXPECT_TRUE(rpt.moduleIdentMismatch());
    EXPECT_TRUE(rpt.connIdMismatch());
}

TEST_F(SafetyDiagnosticsBusTest, MissingObjectsYieldReadFailures) {
    // No safety objects added — every SDO read aborts; the report must
    // still be returned with those *_ok flags cleared (fail-safe
    // semantics).  The CiA402 objects (0x603F/0x6041/0x1001) exist in the
    // emulator's built-in OD and therefore succeed.
    auto rpt = runFullSafetyDiagnostics(sl());

    EXPECT_FALSE(rpt.inputs_ok);
    EXPECT_FALSE(rpt.general_safety_ok);
    EXPECT_FALSE(rpt.statusword_ok);
    EXPECT_FALSE(rpt.error_report_ok);
    EXPECT_TRUE(rpt.cia402_error_ok);
    EXPECT_TRUE(rpt.cia402_status_ok);
    EXPECT_FALSE(rpt.master_frame_ok);
    EXPECT_FALSE(rpt.slave_frame_ok);
    EXPECT_FALSE(rpt.module_ident_ok);
    EXPECT_FALSE(rpt.safety_addr_ok);
    EXPECT_TRUE(rpt.error_reg_ok);
    EXPECT_FALSE(rpt.hasFault());
    EXPECT_FALSE(rpt.fsoeActive());
    EXPECT_FALSE(rpt.isInSafeState());
}

TEST_F(SafetyDiagnosticsBusTest, ReadSafetyModuleStateAndSafetyAddress) {
    populateHealthy();
    auto state = readSafetyModuleState(sl());
    EXPECT_TRUE(state.ok);
    EXPECT_TRUE(state.fsoe_read_ok);
    EXPECT_TRUE(state.motionAllowed());
    EXPECT_TRUE(state.fsoeActive());

    uint16_t addr = 0;
    EXPECT_EQ(readFSoESafetyAddress(sl(), addr), SlaveError::Ok);
    EXPECT_EQ(addr, 0x1234);
}

TEST_F(SafetyDiagnosticsBusTest, ReadSafetyModuleStateFailsWithoutOD) {
    auto state = readSafetyModuleState(sl());
    EXPECT_FALSE(state.ok);
    EXPECT_FALSE(state.fsoe_read_ok);
    EXPECT_STREQ(state.stateSummary(), "unknown (SDO read failed)");

    uint16_t addr = 0;
    EXPECT_NE(readFSoESafetyAddress(sl(), addr), SlaveError::Ok);
}
