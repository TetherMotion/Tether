/**
 * @file test_watchdog_sii.cpp
 * @brief Coverage for slave-core utility TUs:
 *        src/slave/core/WatchdogManager.cpp (SM watchdog counting,
 *        timeout trigger + callback, reset) and
 *        src/slave/core/SIIEmulator.cpp (EEPROM read/write command
 *        processing via the ESC register bank).
 */

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "tether/slave/core/SIIEmulator.hpp"
#include "tether/slave/core/WatchdogManager.hpp"

using namespace EtherCAT::slave;

namespace {

// ============================================================================
// WatchdogManager
// ============================================================================

TEST(WatchdogSiiCov, DisabledNeverCounts) {
    WatchdogState st;
    WatchdogManager wd(false, &st);
    wd.update(SlaveState::OP, 1000);
    EXPECT_EQ(st.smCounter, 0);
    EXPECT_FALSE(st.isSmTriggered());
}

TEST(WatchdogSiiCov, NonOpStateDoesNotCount) {
    WatchdogState st;
    WatchdogManager wd(true, &st);
    wd.update(SlaveState::SAFE_OP, 1000);
    wd.update(SlaveState::PRE_OP, 1000);
    wd.update(SlaveState::INIT, 1000);
    EXPECT_EQ(st.smCounter, 0);
    EXPECT_FALSE(st.isSmTriggered());
}

TEST(WatchdogSiiCov, OpCountsToTimeoutAndFires) {
    WatchdogState st;
    st.smTimeout = 3;
    WatchdogManager wd(true, &st);

    int cb_calls = 0;
    bool cb_pdi = true, cb_sm = false;
    wd.setCallback([&](bool pdi, bool sm) { ++cb_calls; cb_pdi = pdi; cb_sm = sm; });

    wd.update(SlaveState::OP, 100);
    wd.update(SlaveState::OP, 100);
    EXPECT_EQ(st.smCounter, 2);
    EXPECT_FALSE(st.isSmTriggered());
    EXPECT_EQ(cb_calls, 0);

    wd.update(SlaveState::OP, 100);          // hits timeout
    EXPECT_TRUE(st.isSmTriggered());
    EXPECT_EQ(cb_calls, 1);
    EXPECT_FALSE(cb_pdi);
    EXPECT_TRUE(cb_sm);
}

TEST(WatchdogSiiCov, NoCallbackStillTriggers) {
    WatchdogState st;
    st.smTimeout = 1;
    WatchdogManager wd(true, &st);           // no callback set
    wd.update(SlaveState::OP, 100);
    EXPECT_TRUE(st.isSmTriggered());
}

TEST(WatchdogSiiCov, ResetClearsBoth) {
    WatchdogState st;
    st.smTimeout = 2;
    WatchdogManager wd(true, &st);
    wd.update(SlaveState::OP, 100);
    wd.update(SlaveState::OP, 100);
    ASSERT_TRUE(st.isSmTriggered());
    wd.reset();
    EXPECT_EQ(st.smCounter, 0);
    EXPECT_FALSE(st.isSmTriggered());
    EXPECT_EQ(st.pdiCounter, 0);
}

// ============================================================================
// SIIEmulator — word access
// ============================================================================

TEST(WatchdogSiiCov, SIIReadWriteWord) {
    std::array<uint8_t, 4096> regs{};
    SIIEmulator sii(&regs);
    sii.setData({0x34, 0x12, 0x78, 0x56});
    EXPECT_EQ(sii.readWord(0), 0x1234);
    EXPECT_EQ(sii.readWord(1), 0x5678);
    EXPECT_EQ(sii.readWord(2), 0xFFFF);      // out of range
    EXPECT_TRUE(sii.writeWord(0, 0xABCD));
    EXPECT_EQ(sii.getData()[0], 0xCD);
    EXPECT_EQ(sii.getData()[1], 0xAB);
    EXPECT_FALSE(sii.writeWord(100, 0x1234)); // out of range
}

TEST(WatchdogSiiCov, SIIEmptyEepromReadsOnes) {
    std::array<uint8_t, 4096> regs{};
    SIIEmulator sii(&regs);
    EXPECT_EQ(sii.readWord(0), 0xFFFF);
    EXPECT_FALSE(sii.writeWord(0, 0));
}

// ============================================================================
// SIIEmulator — register-write command processing
// ============================================================================

TEST(WatchdogSiiCov, SIIAddressWriteStoresAddress) {
    std::array<uint8_t, 4096> regs{};
    SIIEmulator sii(&regs);
    sii.setData({0xAA, 0x55});
    uint8_t addr[4] = {0x01, 0x00, 0x00, 0x00};
    sii.handleRegisterWrite(ESCReg::SIIAddress, addr, 4);
    // Now a read command on word address 1 → out of range (only 1 word data)
    uint8_t ctrl[2] = {0x00, 0x01};          // read_op = bit 8
    sii.handleRegisterWrite(ESCReg::SIIControl, ctrl, 2);
    EXPECT_EQ(regs[ESCReg::SIIData], 0xFF);
    EXPECT_EQ(regs[ESCReg::SIIData + 1], 0xFF);
}

TEST(WatchdogSiiCov, SIIReadCommandPopulatesDataReg) {
    std::array<uint8_t, 4096> regs{};
    SIIEmulator sii(&regs);
    sii.setData({0x34, 0x12, 0x78, 0x56});
    // SIIAddress = word 1, then read command
    uint8_t addr[4] = {0x01, 0x00, 0x00, 0x00};
    sii.handleRegisterWrite(ESCReg::SIIAddress, addr, 4);
    uint8_t ctrl[2] = {0x00, 0x01};          // read_op = bit 8
    sii.handleRegisterWrite(ESCReg::SIIControl, ctrl, 2);
    EXPECT_EQ(regs[ESCReg::SIIData], 0x78);
    EXPECT_EQ(regs[ESCReg::SIIData + 1], 0x56);
    // read_op/busy cleared after processing
    EXPECT_EQ(regs[ESCReg::SIIControl + 1] & 0x03, 0);
}

TEST(WatchdogSiiCov, SIICombinedWriteThenRead) {
    std::array<uint8_t, 4096> regs{};
    SIIEmulator sii(&regs);
    sii.setData({0x11, 0x22, 0x00, 0x00});   // word0=0x2211
    // Read word 0 first — populates siiState_.data (the write source).
    uint8_t addr0[4] = {0x00, 0x00, 0x00, 0x00};
    sii.handleRegisterWrite(ESCReg::SIIAddress, addr0, 4);
    uint8_t rd[2] = {0x00, 0x01};            // read_op
    sii.handleRegisterWrite(ESCReg::SIIControl, rd, 2);
    // Combined write: control(write_op) + 32-bit address (word 1).
    uint8_t pkt[6] = {0x00, 0x02, 0x01, 0x00, 0x00, 0x00};
    sii.handleRegisterWrite(ESCReg::SIIControl, pkt, 6);
    // The previously-read word lands at word 1.
    EXPECT_EQ(sii.getData()[2], 0x11);
    EXPECT_EQ(sii.getData()[3], 0x22);
    // write bit cleared in the register bank
    EXPECT_EQ(regs[ESCReg::SIIControl + 1] & 0x02, 0);
}

TEST(WatchdogSiiCov, SIIWriteCommandControlOnly) {
    std::array<uint8_t, 4096> regs{};
    SIIEmulator sii(&regs);
    sii.setData({0x00, 0x00});
    // Data reg preset via combined SIIAddress + data write, then write_op
    uint8_t addr[4] = {0x00, 0x00, 0x00, 0x00};
    sii.handleRegisterWrite(ESCReg::SIIAddress, addr, 4);
    // write_op command via 2-byte control write
    uint8_t ctrl[2] = {0x00, 0x02};
    sii.handleRegisterWrite(ESCReg::SIIControl, ctrl, 2);
    SUCCEED();                               // exercised, no crash
}

TEST(WatchdogSiiCov, SIIUnrelatedWriteIgnored) {
    std::array<uint8_t, 4096> regs{};
    SIIEmulator sii(&regs);
    uint8_t d[4] = {1, 2, 3, 4};
    sii.handleRegisterWrite(0x0120, d, 4);   // not an SII register
    SUCCEED();
}

}  // namespace
