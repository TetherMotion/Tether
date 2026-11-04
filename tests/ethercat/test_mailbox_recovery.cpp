/**
 * @file test_mailbox_recovery.cpp
 * @brief Emulator-backed coverage for MailboxRecovery (mailbox drain +
 *        SM0/SM1 activate-cycle recovery).
 */

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <vector>
#include <thread>

#include "mocks/EmulatedBus.hpp"
#include "raw/RawConstants.hpp"
#include "raw/RawWireFormat.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

constexpr uint16_t kMbxWrAddr = 0x1000;
constexpr uint16_t kMbxWrLen = 128;
constexpr uint16_t kMbxRdAddr = 0x1400;
constexpr uint16_t kMbxRdLen = 128;

class MailboxRecoveryTest : public ::testing::Test {
protected:
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    // Bring up one emulated slave with a CoE mailbox and configure SM0/SM1
    // plus the master's SDO mailbox view.
    void bringUp() {
        auto slave = std::make_unique<SlaveEmulator>();
        EtherCAT::Emulator::SIIConfig cfg;
        cfg.vendor_id = 0x00000002;
        cfg.product_code = 0x0FA13052;
        cfg.mailbox_protocol = 0x04;  // CoE
        slave->setSIIConfig(cfg);
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));

        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(1);

        // SM0 = master->slave mailbox, SM1 = slave->master mailbox.
        configureSm(0, kMbxWrAddr, kMbxWrLen);
        configureSm(1, kMbxRdAddr, kMbxRdLen);
        master_.sdoManager(0).configureMailbox(kMbxWrAddr, kMbxWrLen,
                                               kMbxRdAddr, kMbxRdLen);

        // initSlaves' background SDO traffic keeps re-latching SM1's
        // mailbox-full flag; wait for the wire to go quiet before tests
        // script mailbox state of their own.
        settleMailbox();
    }

    void settleMailbox() {
        for (int i = 0; i < 200 && sm1MailboxFull(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void configureSm(uint8_t sm_index, uint16_t start, uint16_t len) {
        uint8_t smcfg[8] = {};
        smcfg[0] = static_cast<uint8_t>(start & 0xFF);
        smcfg[1] = static_cast<uint8_t>(start >> 8);
        smcfg[2] = static_cast<uint8_t>(len & 0xFF);
        smcfg[3] = static_cast<uint8_t>(len >> 8);
        smcfg[4] = 0x02;   // mailbox mode
        smcfg[6] = 0x01;   // enable
        ASSERT_TRUE(master_.writeRegister(SlaveAddress(0),
            RegisterAddress(static_cast<uint16_t>(0x0800 + sm_index * 8)),
            smcfg, sizeof(smcfg)));
    }

    // Push a minimal CoE SDO upload request into SM0 so the emulator queues
    // a stale response in SM1 and latches the mailbox-full flag.
    void injectStaleMailboxResponse() {
        struct __attribute__((packed)) Req {
            Raw::MbxHeader mbx;
            Raw::CoeHeader coe;
            uint8_t sdo[8];
        } req{};
        req.mbx.length_le = Raw::host_to_le16(sizeof(Raw::CoeHeader) + 8);
        req.mbx.address_le = 0;
        req.mbx.priority = 0;
        req.mbx.mbxtype = Raw::mbx_type_with_cnt(Raw::EC_MBXT_COE, 1);
        req.coe.raw_le = Raw::host_to_le16(
            Raw::coe_make_raw(0, Raw::EC_COES_SDOREQ));
        req.sdo[0] = Raw::EC_SDO_UP_REQ;  // initiate upload
        req.sdo[1] = 0x00;              // index 0x1000 lo
        req.sdo[2] = 0x10;              // index hi
        req.sdo[3] = 0x00;              // subindex
        // Feed the request through the emulator's APWR seam — it parses the
        // CoE request and latches a stale response into SM1's buffer.
        ASSERT_TRUE(slave_->processAPWR(kMbxWrAddr,
            reinterpret_cast<const uint8_t*>(&req), sizeof(req)));
    }

    // requestState() faults on an invalid (incl. same-state) transition,
    // which would set the AL error flag and fail the SM-cycle guard.
    void ensureState(SlaveState state) {
        if (slave_->getState() != state) {
            slave_->requestState(state);
        }
        ASSERT_EQ(slave_->getState(), state);
    }

    bool sm1MailboxFull() {
        uint8_t status = 0;
        if (!master_.readRegister(SlaveAddress(0),
                RegisterAddress(Raw::sm_status_address(1)), status, 200)) {
            return false;
        }
        return (status & Raw::EC_SM_STATUS_MBXFULL) != 0;
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
};

// ============================================================================
// drain()
// ============================================================================

TEST_F(MailboxRecoveryTest, DrainFailsWithoutConfiguredMailbox) {
    bus_.attach(master_);
    master_.start(bus_.iface, kEmulatedMac);
    // No initSlaves / configureMailbox — getMailbox fails.
    EXPECT_FALSE(master_.drainSlaveMailbox(0, 4));
}

TEST_F(MailboxRecoveryTest, DrainFailsWithZeroReadLength) {
    bringUp();
    master_.sdoManager(0).configureMailbox(kMbxWrAddr, kMbxWrLen,
                                           kMbxRdAddr, 0);
    EXPECT_FALSE(master_.drainSlaveMailbox(0, 4));
}

TEST_F(MailboxRecoveryTest, DrainSucceedsOnEmptyMailbox) {
    bringUp();
    EXPECT_FALSE(sm1MailboxFull());
    EXPECT_TRUE(master_.drainSlaveMailbox(0, 4));
}

TEST_F(MailboxRecoveryTest, DrainClearsStaleMailboxResponse) {
    bringUp();
    injectStaleMailboxResponse();
    ASSERT_TRUE(sm1MailboxFull());
    EXPECT_TRUE(master_.drainSlaveMailbox(0, 4));
    EXPECT_FALSE(sm1MailboxFull());
}

TEST_F(MailboxRecoveryTest, DrainWithZeroMaxDrainReturnsTrue) {
    bringUp();
    injectStaleMailboxResponse();
    // No drain attempts requested: the "nothing drained" path reports
    // success trivially even though the flag is set.
    EXPECT_TRUE(master_.drainSlaveMailbox(0, 0));
    EXPECT_TRUE(sm1MailboxFull());
}

TEST_F(MailboxRecoveryTest, DrainFailsWhenStatusReadFails) {
    bringUp();
    EtherCAT::Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    inj.timeout_register = Raw::sm_status_address(1);
    slave_->setErrorInjection(inj);
    EXPECT_FALSE(master_.drainSlaveMailbox(0, 4));
}

TEST_F(MailboxRecoveryTest, DrainReadFailureAttemptsSM1Reset) {
    bringUp();
    injectStaleMailboxResponse();
    ASSERT_TRUE(sm1MailboxFull());

    // Fail the drain buffer read only; SM status + AL reads still work.
    // The slave is in PRE-OP after initSlaves, so the SM-cycle guard refuses
    // the reset and drain reports failure.
    EtherCAT::Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    inj.timeout_register = kMbxRdAddr;
    slave_->setErrorInjection(inj);
    EXPECT_FALSE(master_.drainSlaveMailbox(0, 4));
}

// ============================================================================
// resetSM1() / resetSM0()
// ============================================================================

TEST_F(MailboxRecoveryTest, ResetSM1SucceedsInInit) {
    bringUp();
    ensureState(SlaveState::INIT);
    EXPECT_TRUE(master_.resetSlaveMailboxSM1(0));
}

TEST_F(MailboxRecoveryTest, ResetSM1StillFullReportsFailure) {
    bringUp();
    injectStaleMailboxResponse();
    ASSERT_TRUE(sm1MailboxFull());
    ensureState(SlaveState::INIT);
    // The activate cycle does not clear the emulator's latched flag, so the
    // post-reset status read still reports full.
    EXPECT_FALSE(master_.resetSlaveMailboxSM1(0));
}

TEST_F(MailboxRecoveryTest, ResetSM1SkippedOutsideInit) {
    bringUp();
    ensureState(SlaveState::PRE_OP);
    EXPECT_FALSE(master_.resetSlaveMailboxSM1(0));
}

TEST_F(MailboxRecoveryTest, ResetSM1FailsWhenALStatusUnreadable) {
    bringUp();
    EtherCAT::Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    inj.timeout_register = 0x0130;  // AL Status
    slave_->setErrorInjection(inj);
    EXPECT_FALSE(master_.resetSlaveMailboxSM1(0));
}

TEST_F(MailboxRecoveryTest, ResetSM0SucceedsInInit) {
    bringUp();
    ensureState(SlaveState::INIT);
    EXPECT_TRUE(master_.resetSlaveMailboxSM0(0));
}

TEST_F(MailboxRecoveryTest, ResetSM0SkippedOutsideInit) {
    bringUp();
    ensureState(SlaveState::PRE_OP);
    EXPECT_FALSE(master_.resetSlaveMailboxSM0(0));
}

TEST_F(MailboxRecoveryTest, ResetSM1FailsWhenPostResetStatusReadFails) {
    bringUp();
    ensureState(SlaveState::INIT);
    // SM1 status reads fail — the activate writes succeed but the
    // post-reset status check cannot verify the buffer is empty.
    EtherCAT::Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    inj.timeout_register = Raw::sm_status_address(1);
    slave_->setErrorInjection(inj);
    EXPECT_FALSE(master_.resetSlaveMailboxSM1(0));
}

TEST_F(MailboxRecoveryTest, ResetSM0FailsWhenPostResetStatusReadFails) {
    bringUp();
    ensureState(SlaveState::INIT);
    EtherCAT::Emulator::ErrorInjection inj;
    inj.inject_timeout = true;
    inj.timeout_register = Raw::sm_status_address(0);
    slave_->setErrorInjection(inj);
    EXPECT_FALSE(master_.resetSlaveMailboxSM0(0));
}

TEST_F(MailboxRecoveryTest, ResetSM0AbortsWhenCancelRequested) {
    bringUp();
    ensureState(SlaveState::INIT);
    master_.requestCancel();
    EXPECT_FALSE(master_.resetSlaveMailboxSM0(0));
}

}  // namespace
