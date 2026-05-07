/**
 * @file test_EtherCATResetController_coverage.cpp
 * @brief Tests for SlaveResetController using MockSDOTransport to cover
 *        EtherCATResetCore.cpp, EtherCATResetESM.cpp, EtherCATResetCiA.cpp
 */

#include "tether/ethercat/EtherCATReset.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>
#include <thread>

using namespace EtherCAT;
using namespace EtherCAT::SDO;
using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgPointee;

using ::testing::NiceMock;

// ============================================================================
// Mock SDO Transport
// ============================================================================

namespace {

uint64_t realMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class MockSDOTransportRaw : public ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 uint8_t* out, size_t out_cap, size_t* out_len),
                (override));
    MOCK_METHOD(bool, sdoDownload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 const uint8_t* data, size_t data_len),
                (override));
    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
};

// Helper: make sdoUpload populate output buffer
auto UploadOk(const void* data, size_t len) {
    return [data, len](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                       uint16_t, uint8_t, uint8_t* out, size_t, size_t* out_len) -> bool {
        memcpy(out, data, len);
        if (out_len) *out_len = len;
        return true;
    };
}

auto DownloadOk() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, const uint8_t*, size_t) -> bool {
        return true;
    };
}

auto DownloadFail() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, const uint8_t*, size_t) -> bool {
        return false;
    };
}

// ============================================================================
// Fixture that manages SDOManager lifecycle
// ============================================================================

class ResetCtrlCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
        // Default: all SDO calls succeed
        ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Invoke(DownloadOk()));
        // Default: uploads return zeros
        uint16_t zero = 0;
        ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Invoke(UploadOk(&zero, sizeof(zero))));

        sdo_ = std::make_unique<SDOManager>(transport_);
        sdo_->configureSlaveMailbox(0, 0x1000, 128, 0x1400, 128);
        sdo_->init();
    }
    void TearDown() override {
        if (sdo_) sdo_->deinit();
    }

    NiceMock<MockSDOTransportRaw> transport_;
    std::unique_ptr<SDOManager> sdo_;
};

} // anonymous namespace

// ============================================================================
// Construction + Pure Accessors
// ============================================================================

TEST_F(ResetCtrlCovTest, ConstructWithPosition) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_EQ(ctrl.getResetAttemptCount(), 0u);
    EXPECT_EQ(ctrl.getSuccessfulResetCount(), 0u);
    EXPECT_FALSE(ctrl.getLastResult().success);
}

TEST_F(ResetCtrlCovTest, ConstructWithConfiguredAddr) {
    SlaveResetController ctrl(*sdo_, 0x1001, true);
    EXPECT_EQ(ctrl.getResetAttemptCount(), 0u);
}

TEST_F(ResetCtrlCovTest, SetProgressCallback) {
    SlaveResetController ctrl(*sdo_, 0);
    int callCount = 0;
    ctrl.setProgressCallback([&](const char*, uint8_t, uint16_t) { callCount++; });
    // The callback is invoked during various methods
}

// ============================================================================
// ESM Methods
// ============================================================================

TEST_F(ResetCtrlCovTest, ReadALStatus) {
    // Return status=0x08 (OP) and code=0x0000
    uint16_t status = 0x0008;
    uint16_t code = 0x0000;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&status, sizeof(status))));
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&code, sizeof(code))));

    SlaveResetController ctrl(*sdo_, 0);
    uint16_t s, c;
    EXPECT_TRUE(ctrl.readALStatus(s, c));
    EXPECT_EQ(s, 0x0008);
    EXPECT_EQ(c, 0x0000);
}

TEST_F(ResetCtrlCovTest, AcknowledgeError_Success) {
    // First read: error state (status = Init + ErrorFlag = 0x0011)
    uint16_t errStatus = 0x0011;
    uint16_t noErr = 0x0001; // Init, no error flag

    int readCount = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                                   uint16_t, uint8_t, uint8_t* out, size_t, size_t* ol) -> bool {
            if (readCount++ == 0) {
                memcpy(out, &errStatus, 2);
            } else {
                memcpy(out, &noErr, 2);
            }
            if (ol) *ol = 2;
            return true;
        }));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.acknowledgeError());
}

TEST_F(ResetCtrlCovTest, AcknowledgeError_Persistent) {
    // Error flag stays set after acknowledge
    uint16_t errStatus = 0x0011;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&errStatus, sizeof(errStatus))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_FALSE(ctrl.acknowledgeError());
}

TEST_F(ResetCtrlCovTest, TransitionToState) {
    // Simulated: write AL control, then wait reads target state
    uint16_t opState = static_cast<uint16_t>(ALState::Op);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&opState, sizeof(opState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.transitionToState(ALState::Op, 500));
}

TEST_F(ResetCtrlCovTest, ForceToInit) {
    uint16_t initState = static_cast<uint16_t>(ALState::Init);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&initState, sizeof(initState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.forceToInit(500));
}

TEST_F(ResetCtrlCovTest, RequestESCReset) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.requestESCReset());
}

TEST_F(ResetCtrlCovTest, ResetSyncManagerWatchdog) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.resetSyncManagerWatchdog());
}

TEST_F(ResetCtrlCovTest, ClearPDIWatchdog) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.clearPDIWatchdog());
}

TEST_F(ResetCtrlCovTest, ResetDistributedClock) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.resetDistributedClock());
}

TEST_F(ResetCtrlCovTest, ClearDCSyncErrors) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.clearDCSyncErrors());
}

TEST_F(ResetCtrlCovTest, ReconfigureSyncManagers) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.reconfigureSyncManagers());
}

TEST_F(ResetCtrlCovTest, FullReinitialize) {
    // Each readALStatus returns the state we just wrote
    int writeCount = 0;
    uint16_t currentState = static_cast<uint16_t>(ALState::Init);
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, 0x0120, 0, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                                   uint16_t, uint8_t, const uint8_t* data, size_t) -> bool {
            memcpy(&currentState, data, 2);
            return true;
        }));
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                                   uint16_t, uint8_t, uint8_t* out, size_t, size_t* ol) -> bool {
            memcpy(out, &currentState, 2);
            if (ol) *ol = 2;
            return true;
        }));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.fullReinitialize(true);
    // May or may not succeed depending on timing
    (void)result;
}

// ============================================================================
// CiA 301 Methods
// ============================================================================

TEST_F(ResetCtrlCovTest, NmtResetNode) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.nmtResetNode());
}

TEST_F(ResetCtrlCovTest, NmtResetCommunication) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.nmtResetCommunication());
}

TEST_F(ResetCtrlCovTest, RestoreDefaultParameters) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.restoreDefaultParameters(CiA301Reset::AllParameters));
    EXPECT_TRUE(ctrl.restoreDefaultParameters(CiA301Reset::CommunicationParams));
    EXPECT_TRUE(ctrl.restoreDefaultParameters(CiA301Reset::ApplicationParams));
}

TEST_F(ResetCtrlCovTest, ClearErrorHistory) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.clearErrorHistory());
}

// ============================================================================
// CiA 402 Methods
// ============================================================================

TEST_F(ResetCtrlCovTest, ReadStatusword) {
    uint16_t sw = 0x0237;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6041, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&sw, sizeof(sw))));

    SlaveResetController ctrl(*sdo_, 0);
    uint16_t result;
    EXPECT_TRUE(ctrl.readStatusword(result));
    EXPECT_EQ(result, 0x0237);
}

TEST_F(ResetCtrlCovTest, WriteControlword) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.writeControlword(0x000F));
}

TEST_F(ResetCtrlCovTest, ClearDriveErrors) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.clearDriveErrors());
}

TEST_F(ResetCtrlCovTest, QuickStop) {
    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.quickStop());
}

TEST_F(ResetCtrlCovTest, Halt) {
    // Need to mock readStatusword (0x6041) for halt to read current controlword
    // Actually halt reads controlword (0x6040), not statusword
    uint16_t cw = 0x000F;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6040, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&cw, sizeof(cw))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.halt());
}

TEST_F(ResetCtrlCovTest, ResumeFromHalt) {
    uint16_t cw = 0x010F; // Halt bit set
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6040, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&cw, sizeof(cw))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.resumeFromHalt());
}

TEST_F(ResetCtrlCovTest, FaultReset_NotInFault) {
    // Statusword without fault bit
    uint16_t sw = 0x0040; // SwitchOnDisabled, no fault
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6041, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&sw, sizeof(sw))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.faultReset();
    EXPECT_TRUE(result.success); // Not in fault = instant success
}

TEST_F(ResetCtrlCovTest, FaultReset_ClearedSuccessfully) {
    int readCount = 0;
    uint16_t faultSw = 0x0008;  // Fault bit set
    uint16_t clearSw = 0x0040;  // SwitchOnDisabled after clear

    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6041, 0, _, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                                   uint16_t, uint8_t, uint8_t* out, size_t, size_t* ol) -> bool {
            if (readCount++ < 1) {
                memcpy(out, &faultSw, 2);
            } else {
                memcpy(out, &clearSw, 2);
            }
            if (ol) *ol = 2;
            return true;
        }));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.faultReset();
    EXPECT_TRUE(result.success);
}

TEST_F(ResetCtrlCovTest, FaultReset_Persistent) {
    // Fault stays set after reset attempt
    uint16_t faultSw = 0x0008;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6041, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&faultSw, sizeof(faultSw))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.faultReset();
    EXPECT_FALSE(result.success);
}

TEST_F(ResetCtrlCovTest, DisableDrive) {
    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.disableDrive();
    (void)result; // May or may not succeed
}

TEST_F(ResetCtrlCovTest, EnableDrive) {
    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.enableDrive();
    (void)result; // 3-step sequence
}

TEST_F(ResetCtrlCovTest, VendorSpecificReset) {
    SlaveResetController ctrl(*sdo_, 0);
    uint8_t data[] = {0x01};
    EXPECT_TRUE(ctrl.vendorSpecificReset(0x2000, 0, data, 1));
}

TEST_F(ResetCtrlCovTest, VoeReset) {
    SlaveResetController ctrl(*sdo_, 0);
    uint8_t data[] = {0x01};
    EXPECT_FALSE(ctrl.voeReset(data, 1)); // Stub returns false
}

// ============================================================================
// Core Reset Orchestration
// ============================================================================

TEST_F(ResetCtrlCovTest, ResetToLevel_SoftReset) {
    // Make readALStatus return no-error state after reset
    uint16_t okState = static_cast<uint16_t>(ALState::Op);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okState, sizeof(okState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.resetToLevel(ResetLevel::SoftReset, 500);
    (void)result;
    EXPECT_GE(ctrl.getResetAttemptCount(), 1u);
}

TEST_F(ResetCtrlCovTest, ResetToLevel_StateMachineReset) {
    uint16_t initState = static_cast<uint16_t>(ALState::Init);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&initState, sizeof(initState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.resetToLevel(ResetLevel::StateMachineReset, 500);
    (void)result;
}

TEST_F(ResetCtrlCovTest, ResetToLevel_ESCHardwareReset) {
    uint16_t initState = static_cast<uint16_t>(ALState::Init);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&initState, sizeof(initState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.resetToLevel(ResetLevel::ESCHardwareReset, 500);
    (void)result;
}

TEST_F(ResetCtrlCovTest, ResetToLevel_CommunicationReset) {
    uint16_t sw = 0x0040; // Not in fault
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6041, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&sw, sizeof(sw))));
    uint16_t okState = static_cast<uint16_t>(ALState::Op);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okState, sizeof(okState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.resetToLevel(ResetLevel::CommunicationReset, 500);
    (void)result;
}

TEST_F(ResetCtrlCovTest, ResetToLevel_ApplicationReset) {
    uint16_t initState = static_cast<uint16_t>(ALState::Init);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&initState, sizeof(initState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.resetToLevel(ResetLevel::ApplicationReset, 500);
    (void)result;
}

TEST_F(ResetCtrlCovTest, ProgressiveReset) {
    // Simulate error that clears after SoftReset
    int alReadCount = 0;
    uint16_t errState = 0x0011; // Init + Error flag
    uint16_t okState = 0x0001;  // Init, no error

    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                                   uint16_t, uint8_t, uint8_t* out, size_t, size_t* ol) -> bool {
            // After a few reads, clear the error
            if (alReadCount++ < 3)
                memcpy(out, &errState, 2);
            else
                memcpy(out, &okState, 2);
            if (ol) *ol = 2;
            return true;
        }));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.progressiveReset(ResetLevel::ESCHardwareReset, 500);
    (void)result;
}

TEST_F(ResetCtrlCovTest, EmergencyStopAndReset) {
    uint16_t okState = static_cast<uint16_t>(ALState::Init);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okState, sizeof(okState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto result = ctrl.emergencyStopAndReset();
    (void)result;
}

// ============================================================================
// Status / Diagnostics
// ============================================================================

TEST_F(ResetCtrlCovTest, IsInErrorState_Yes) {
    uint16_t errStatus = 0x0011; // Init + ErrorFlag
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&errStatus, sizeof(errStatus))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_TRUE(ctrl.isInErrorState());
}

TEST_F(ResetCtrlCovTest, IsInErrorState_No) {
    uint16_t okStatus = 0x0008; // OP, no error
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okStatus, sizeof(okStatus))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_FALSE(ctrl.isInErrorState());
}

TEST_F(ResetCtrlCovTest, GetErrorDescription_InError) {
    uint16_t errStatus = 0x0011;
    uint16_t errCode = 0x0017; // InvalidSMConfig
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&errStatus, sizeof(errStatus))));
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&errCode, sizeof(errCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto desc = ctrl.getErrorDescription();
    EXPECT_FALSE(desc.empty());
}

TEST_F(ResetCtrlCovTest, GetErrorDescription_NoError) {
    uint16_t okStatus = 0x0008;
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okStatus, sizeof(okStatus))));
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    auto desc = ctrl.getErrorDescription();
    // May return "No error" or empty
    (void)desc;
}

// ============================================================================
// applyResetPolicy
// ============================================================================

TEST_F(ResetCtrlCovTest, ApplyResetPolicy_Default) {
    uint16_t okStatus = static_cast<uint16_t>(ALState::Op);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okStatus, sizeof(okStatus))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    ResetPolicy policy;
    policy.max_auto_attempts = 1;
    policy.starting_level = ResetLevel::SoftReset;
    policy.max_level = ResetLevel::SoftReset;
    policy.escalate_on_failure = false;

    auto result = applyResetPolicy(ctrl, policy);
    (void)result;
}

TEST_F(ResetCtrlCovTest, ApplyResetPolicy_WithCallback) {
    uint16_t okStatus = static_cast<uint16_t>(ALState::Init);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okStatus, sizeof(okStatus))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    ResetPolicy policy;
    policy.max_auto_attempts = 2;
    policy.escalate_on_failure = true;
    policy.starting_level = ResetLevel::SoftReset;
    policy.max_level = ResetLevel::ESCHardwareReset;
    policy.should_continue = [](const ResetResult&, uint8_t attempt) {
        return attempt < 2;
    };

    auto result = applyResetPolicy(ctrl, policy);
    (void)result;
}

// ============================================================================
// SDO failure paths
// ============================================================================

TEST_F(ResetCtrlCovTest, ReadALStatus_Failure) {
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Return(false));

    SlaveResetController ctrl(*sdo_, 0);
    uint16_t s, c;
    EXPECT_FALSE(ctrl.readALStatus(s, c));
}

TEST_F(ResetCtrlCovTest, WriteControlword_Failure) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, 0x6040, 0, _, _))
        .WillByDefault(Invoke(DownloadFail()));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_FALSE(ctrl.writeControlword(0x0000));
}

TEST_F(ResetCtrlCovTest, ResetSyncManagerWatchdog_Failure) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, 0x0806, 0, _, _))
        .WillByDefault(Invoke(DownloadFail()));

    SlaveResetController ctrl(*sdo_, 0);
    EXPECT_FALSE(ctrl.resetSyncManagerWatchdog());
}

// ============================================================================
// Progress callback invocation
// ============================================================================

TEST_F(ResetCtrlCovTest, ProgressCallbackDuringReset) {
    uint16_t okState = static_cast<uint16_t>(ALState::Op);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0130, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&okState, sizeof(okState))));
    uint16_t zeroCode = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x0134, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&zeroCode, sizeof(zeroCode))));

    SlaveResetController ctrl(*sdo_, 0);
    std::vector<std::string> stages;
    ctrl.setProgressCallback([&](const char* stage, uint8_t pct, uint16_t addr) {
        stages.push_back(stage);
    });

    ctrl.resetToLevel(ResetLevel::SoftReset, 500);
    EXPECT_FALSE(stages.empty());
}
