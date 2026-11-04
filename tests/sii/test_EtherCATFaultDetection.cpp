#include <gtest/gtest.h>

#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include <cstring>

using namespace EtherCAT;
using namespace EtherCAT::Emulator;

// Small IFaultTransport implementation that forwards to a SlaveEmulator instance.
struct EmuTransport : public IFaultTransport {
    explicit EmuTransport(SlaveEmulator* s) : slave(s) {}
    bool readRegister(uint16_t slave_index, uint16_t reg_addr, void* data, uint16_t size) override {
        (void)slave_index; // single-slave emulator used for tests
        return slave->processFPRD(reg_addr, reinterpret_cast<uint8_t*>(data), size);
    }
    bool writeRegister(uint16_t slave_index, uint16_t reg_addr, const void* data, uint16_t size) override {
        (void)slave_index;
        return slave->processFPWR(reg_addr, reinterpret_cast<const uint8_t*>(data), size);
    }
    uint64_t getTimestampMs() override { return 0; }
    void delayMs(uint32_t) override {}
    SlaveEmulator* slave;
};

TEST(EtherCATFaultDetection, EmulatorIntegration_PollAndClear) {
    auto slave = createSimpleSlave(0x1111, 0x2222, 0, 0);
    EmuTransport transport(slave.get());

    FaultDetector fd(transport);
    EXPECT_TRUE(fd.init(1));

    // Start with a healthy transition to SAFE_OP
    slave->requestState(SlaveState::PRE_OP);
    slave->requestState(SlaveState::SAFE_OP);

    auto s = fd.poll(0);
    EXPECT_FALSE(s.has_fault);
    EXPECT_EQ(s.al_status & 0x000F, static_cast<uint16_t>(SlaveState::SAFE_OP));

    // Force an invalid transition (PRE_OP -> OP is not allowed) to set an error
    slave->requestState(SlaveState::PRE_OP);
    slave->requestState(SlaveState::OP); // causes emulator to set error and AL_STATUS_CODE

    auto fault_state = fd.poll(0);
    EXPECT_TRUE(fault_state.has_fault);
    EXPECT_EQ(fault_state.al_status_code, ALStatusCode::InvalidRequestedStateChange);

    // In the emulator writing AL_CONTROL/ACK does not clear the fault in this
    // scenario — verify clear() handles the attempt and reports failure.
    EXPECT_FALSE(fd.clear(0));

    auto after_clear = fd.poll(0);
    // Fault remains active in the emulator implementation used by these tests
    EXPECT_TRUE(after_clear.has_fault);
}

TEST(EtherCATFaultDetection, EmulatorTransport_ReadTimeoutDoesNotCrash) {
    auto slave = createSimpleSlave(0x1111, 0x2222, 0, 0);
    EmuTransport transport(slave.get());
    FaultDetector fd(transport);
    EXPECT_TRUE(fd.init(1));

    // Configure the emulator to simulate a timeout on register reads
    Emulator::ErrorInjection ei;
    ei.inject_timeout = true;
    ei.timeout_register = 0; // timeout for all registers
    slave->setErrorInjection(ei);

    // Poll should handle transport read failures gracefully (no fault reported)
    auto s = fd.poll(0);
    EXPECT_FALSE(s.has_fault);
}

// ============================================================================
// Status-code name tables and manufacturer fault decoding
// ============================================================================

#include <magic_enum/magic_enum.hpp>

TEST(FaultDetectionNames, AllALStatusCodes) {
    for (auto code : magic_enum::enum_values<EtherCAT::ALStatusCode>()) {
        const char* name = EtherCAT::getALStatusCodeName(static_cast<uint16_t>(code));
        EXPECT_NE(name, nullptr);
        EXPECT_GT(std::strlen(name), 0u);
    }
    // unknown code falls through
    EXPECT_STREQ(EtherCAT::getALStatusCodeName(0xFFFE), "Unknown error");
}

TEST(FaultDetectionNames, AllCiA402ErrorCodes) {
    for (auto code : magic_enum::enum_values<EtherCAT::CiA402ErrorCode>()) {
        const char* name = EtherCAT::getCiA402ErrorCodeName(static_cast<uint16_t>(code));
        EXPECT_NE(name, nullptr);
        EXPECT_GT(std::strlen(name), 0u);
    }
    EXPECT_STREQ(EtherCAT::getCiA402ErrorCodeName(0x1234), "Unknown CiA 402 error");
}

TEST(FaultDetectionNames, ManufacturerFaultParse) {
    // known descriptions
    EXPECT_EQ(EtherCAT::ManufacturerFault::parse(741).description, std::string("No Sync (Err74.1)"));
    EXPECT_EQ(EtherCAT::ManufacturerFault::parse(740).description, std::string("DC Sync Error (Err74.0)"));
    EXPECT_EQ(EtherCAT::ManufacturerFault::parse(200).description, std::string("Over current"));
    EXPECT_EQ(EtherCAT::ManufacturerFault::parse(300).description, std::string("Over voltage"));
    EXPECT_EQ(EtherCAT::ManufacturerFault::parse(500).description, std::string("Encoder error"));
    // class/subcode split: >= 100 -> class = code/10, sub = code%10
    auto f = EtherCAT::ManufacturerFault::parse(741);
    EXPECT_EQ(f.class_code, 74);
    EXPECT_EQ(f.sub_code, 1);
    EXPECT_EQ(f.raw_code, 741);
    // < 100 -> class = code, sub = 0
    auto g = EtherCAT::ManufacturerFault::parse(42);
    EXPECT_EQ(g.class_code, 42);
    EXPECT_EQ(g.sub_code, 0);
    // unknown code with vendor/product hints takes the fallback arm
    auto u = EtherCAT::ManufacturerFault::parse(9999, 0x00000002, 0x044C2C52);
    EXPECT_EQ(u.raw_code, 9999);
    EXPECT_NE(u.description, nullptr);
}
