#include <gtest/gtest.h>

#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

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
