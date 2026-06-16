#include <gtest/gtest.h>

#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT::Emulator;

TEST(ALStatus, Roundtrip) {
    ALStatus s;
    s.state = SlaveState::SAFE_OP;
    s.error = true;
    s.id_request = true;

    uint16_t reg = s.toRegister();
    auto s2 = ALStatus::fromRegister(reg);

    EXPECT_EQ(s2.state, s.state);
    EXPECT_EQ(s2.error, s.error);
    EXPECT_EQ(s2.id_request, s.id_request);
}

TEST(SlaveEmulator, SIIReadWrite) {
    auto slave = createGenericIOSlave(0x1111, 0x2222, 4, 4);

    uint16_t vendor = 0;
    ASSERT_TRUE(slave->processSIIRead(8, &vendor));
    EXPECT_EQ(vendor, 0x1111);

    uint16_t product = 0;
    ASSERT_TRUE(slave->processSIIRead(10, &product));
    EXPECT_EQ(product, 0x2222);

    // Test write/read back
    ASSERT_TRUE(slave->processSIIWrite(100, 0xABCD));
    uint16_t val = 0;
    ASSERT_TRUE(slave->processSIIRead(100, &val));
    EXPECT_EQ(val, 0xABCD);
}

TEST(SlaveEmulator, ConfiguredAddressRead) {
    auto slave = createSimpleSlave(0x1111, 0x2222, 2, 2);
    slave->setConfiguredAddress(0x3344);

    uint8_t buf[2] = {0};
    ASSERT_TRUE(slave->processFPRD(0x0010, buf, 2));
    uint16_t addr = buf[0] | (buf[1] << 8);
    EXPECT_EQ(addr, 0x3344);
}

TEST(SlaveEmulator, StateTransitions) {
    auto slave = createSimpleSlave(0x1111, 0x2222, 0, 0);

    // INIT -> PRE_OP allowed
    slave->requestState(SlaveState::PRE_OP);
    EXPECT_EQ(slave->getState(), SlaveState::PRE_OP);
    EXPECT_FALSE(slave->getALStatus().error);

    // PRE_OP -> OP not allowed (should set error)
    slave->requestState(SlaveState::OP);
    EXPECT_TRUE(slave->getALStatus().error);

    // Clear by doing valid transitions
    slave->requestState(SlaveState::SAFE_OP);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);
    EXPECT_FALSE(slave->getALStatus().error);

    slave->requestState(SlaveState::OP);
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_FALSE(slave->getALStatus().error);
}

TEST(SlaveEmulator, DCAdvanceWithDrift) {
    auto slave = createSimpleSlave(0x1111, 0x2222, 0, 0);
    DCState& dc = slave->getDCState();
    uint64_t before = dc.system_time;

    ErrorInjection ei;
    ei.inject_dc_drift = true;
    ei.dc_drift_ppb = 1000; // 1000 ppb
    slave->setErrorInjection(ei);

    slave->advanceDCTime(1000000000ULL); // 1 second in ns
    uint64_t after = dc.system_time;

    EXPECT_EQ(after, before + 1000000000ULL + 1000ULL);
}

TEST(SlaveEmulator, LogicalMappingReadWrite) {
    auto slave = createSimpleSlave(0x1111, 0x2222, 0, 0);

    // Prepare FMMU for logical 0x2000 -> physical 0x1100, length 4, R/W enabled
    uint8_t fmmu[16] = {0};
    uint32_t logical_start = 0x2000;
    uint16_t length = 4;
    uint16_t physical_start = 0x1100;

    fmmu[0] = logical_start & 0xFF;
    fmmu[1] = (logical_start >> 8) & 0xFF;
    fmmu[2] = (logical_start >> 16) & 0xFF;
    fmmu[3] = (logical_start >> 24) & 0xFF;
    fmmu[4] = length & 0xFF;
    fmmu[5] = (length >> 8) & 0xFF;
    fmmu[6] = 0; // start bit
    fmmu[7] = 7; // end bit
    fmmu[8] = physical_start & 0xFF;
    fmmu[9] = (physical_start >> 8) & 0xFF;
    fmmu[10] = 0; // physical start bit
    fmmu[11] = 0x03; // read_enable|write_enable
    fmmu[12] = 0x01; // enabled

    // Write to FMMU 0 at address 0x0600
    ASSERT_TRUE(slave->processFPWR(0x0600, fmmu, 16));

    // Configure Sync Manager 2 to cover physical 0x1100 length 4 and enable it
    uint8_t sm_start[2] = { static_cast<uint8_t>(physical_start & 0xFF), static_cast<uint8_t>((physical_start >> 8) & 0xFF) };
    uint8_t sm_len[2] = { static_cast<uint8_t>(length & 0xFF), static_cast<uint8_t>((length >> 8) & 0xFF) };
    uint8_t sm_enable[1] = { 0x01 };

    ASSERT_TRUE(slave->processFPWR(0x0800 + 2*8 + 0, sm_start, 2)); // start addr for SM2
    ASSERT_TRUE(slave->processFPWR(0x0800 + 2*8 + 2, sm_len, 2)); // length for SM2
    ASSERT_TRUE(slave->processFPWR(0x0800 + 2*8 + 6, sm_enable, 1)); // enable SM2

    // Write logical data
    uint8_t out[4] = {1,2,3,4};
    ASSERT_TRUE(slave->processLogicalWrite(0x2000, out, 4));

    // Read back
    uint8_t in[4] = {0};
    ASSERT_TRUE(slave->processLogicalRead(0x2000, in, 4));
    for (int i = 0; i < 4; ++i) EXPECT_EQ(in[i], out[i]);
}
