/**
 * @file test_cia402_op_transition.cpp
 * @brief CiA402Drive PRE_OP→OP transition sequencing, PDO buffer
 *        registration, and controlledShutdown() over a real
 *        Master ↔ SlaveEmulator wire (EmulatedBus).
 *
 * Covers the parts of CiA402DriveStateMachine.cpp /
 * CiA402DrivePDO.cpp that the state-machine suite does not reach:
 * transitionToOp (both overloads and the skip-PRE_OP path),
 * prepareForSafeOp/postSafeOpForOp, registerPDOBuffers/
 * resetPDORegistration, and the three-phase controlledShutdown
 * on the SDO fallback path.
 */

#include <gtest/gtest.h>

#include <cstring>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class CiA402OpTransitionTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = EtherCAT::Emulator::createCiA402Drive(
            0x00000002, 0x0442, "TestDrive");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
        drive_ = std::make_unique<CiA402Drive>(master_, 0);
    }
    void TearDown() override {
        drive_.reset();
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    /// Writable u8/u16 OD object.
    void addU(uint16_t idx, uint8_t sub, uint32_t value, uint8_t bytes) {
        const uint16_t dt = (bytes == 1) ? 0x0005 : (bytes == 2) ? 0x0006 : 0x0007;
        std::vector<uint8_t> d(bytes);
        for (uint8_t i = 0; i < bytes; ++i)
            d[i] = static_cast<uint8_t>(value >> (8 * i));
        slave_->addODEntry({idx, sub, dt, static_cast<uint16_t>(bytes * 8),
                            0x03, d, "x"});
    }

    /// 0x1C12/0x1C13 SM→PDO assignment records (count + one PDO index).
    void addSmAssign() {
        addU(CiA301::SyncManager2PDOAssign, 0, 0, 1);
        addU(CiA301::SyncManager2PDOAssign, 1, 0x1705, 2);
        addU(CiA301::SyncManager3PDOAssign, 0, 0, 1);
        addU(CiA301::SyncManager3PDOAssign, 1, 0x1B04, 2);
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
    std::unique_ptr<CiA402Drive> drive_;
};

// ============================================================================
// transitionToOp
// ============================================================================

TEST_F(CiA402OpTransitionTest, TransitionToOpWalksFullChain) {
    addSmAssign();
    EXPECT_EQ(slave_->getState(), SlaveState::INIT);

    ASSERT_TRUE(drive_->transitionToOp(false));
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
    EXPECT_EQ(drive_->getECState(), ECState::Op);
}

TEST_F(CiA402OpTransitionTest, TransitionToOpAppliesFixedPdoMapping) {
    addSmAssign();
    drive_->assignFixedPDOs(0x1705, 0x1B04, 12, 12);

    ASSERT_TRUE(drive_->transitionToOp(true));
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
    EXPECT_EQ(drive_->getRxPDOIndex(), 0x1705);
    EXPECT_EQ(drive_->getTxPDOIndex(), 0x1B04);
}

TEST_F(CiA402OpTransitionTest, TransitionToOpSkipsPreOpWhenAlreadyThere) {
    addSmAssign();
    // Emulator enforces the transition table; AL control writes walk it.
    ASSERT_TRUE(drive_->gotoPreOp());
    ASSERT_EQ(slave_->getState(), SlaveState::PRE_OP);

    // From PRE_OP the early state read must take the need_preop=false
    // branch — transition still completes to OP.
    ASSERT_TRUE(drive_->transitionToOp(false));
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
}

/// SM2 RxPDO + SM3 TxPDO multi-PDO assignment matching the emulator's
/// physical layout (0x1100 out / 0x1180 in).
static Slave::MultiPDOAssignment makeMotionAssignment() {
    Slave::MultiPDOAssignment a;
    Slave::MultiPDOAssignment::SMConfig sm2;
    sm2.sm_index = 2;
    sm2.phys_start_addr = 0x1100;
    sm2.control_byte = 0x44;  // write direction bit set
    sm2.pdo_mappings.push_back({0x1705, 12});
    Slave::MultiPDOAssignment::SMConfig sm3;
    sm3.sm_index = 3;
    sm3.phys_start_addr = 0x1180;
    sm3.control_byte = 0x40;  // read direction bit clear
    sm3.pdo_mappings.push_back({0x1B04, 12});
    a.sm_configs = {sm2, sm3};
    return a;
}

TEST_F(CiA402OpTransitionTest, PrepareForSafeOpThenOp) {
    addSmAssign();
    ASSERT_TRUE(drive_->gotoPreOp());

    // Split path used for group-synchronised OP requests: configures
    // SMs/FMMUs via Slave::configureMultiPDOs and registers PDO buffers,
    // but deliberately leaves the slave in PRE_OP — the caller issues
    // the SAFE_OP request (per-slave or via SlaveGroup).
    ASSERT_TRUE(drive_->prepareForSafeOp(makeMotionAssignment()));
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);
    EXPECT_TRUE(drive_->isPDORegistered());

    ASSERT_TRUE(drive_->gotoSafeOp());
    ASSERT_TRUE(drive_->gotoOp());
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
}

TEST_F(CiA402OpTransitionTest, PrepareForSafeOpToleratesEmptyAssignment) {
    ASSERT_TRUE(drive_->gotoPreOp());
    // configureMultiPDOs only warns on an empty config — no PDO buffers
    // get registered and the slave stays in PRE_OP.
    EXPECT_TRUE(drive_->prepareForSafeOp(Slave::MultiPDOAssignment{}));
    EXPECT_FALSE(drive_->isPDORegistered());
    EXPECT_EQ(slave_->getState(), SlaveState::PRE_OP);
}

TEST_F(CiA402OpTransitionTest, TransitionToOpWithMultiPdoAssignment) {
    addSmAssign();
    ASSERT_TRUE(drive_->transitionToOp(makeMotionAssignment()));
    EXPECT_EQ(slave_->getState(), SlaveState::OP);
    // Buffer sizes are summed across the SM configs.
    EXPECT_EQ(drive_->getRxPDOSize(), 12);
    EXPECT_EQ(drive_->getTxPDOSize(), 12);
}

// ============================================================================
// PDO buffer registration
// ============================================================================

TEST_F(CiA402OpTransitionTest, RegisterPdoBuffersExposesMappedViews) {
    addSmAssign();
    drive_->assignFixedPDOs(0x1705, 0x1B04, 12, 12);

    ASSERT_TRUE(drive_->registerPDOBuffers());
    EXPECT_TRUE(drive_->isPDORegistered());
    EXPECT_EQ(drive_->getRxPDOSize(), 12);
    EXPECT_EQ(drive_->getTxPDOSize(), 12);
    ASSERT_NE(drive_->getRxPDOBuffer(), nullptr);
    ASSERT_NE(drive_->getTxPDOBuffer(), nullptr);

    // Re-registering is a no-op success.
    EXPECT_TRUE(drive_->registerPDOBuffers());
}

TEST_F(CiA402OpTransitionTest, ResetPdoRegistrationAllowsReregister) {
    drive_->setPDOBufferSizes(0x1705, 0x1B04, 8, 8);
    ASSERT_TRUE(drive_->registerPDOBuffers());

    drive_->resetPDORegistration();
    EXPECT_FALSE(drive_->isPDORegistered());
    EXPECT_TRUE(drive_->registerPDOBuffers());
}

TEST_F(CiA402OpTransitionTest, TypedPdoViewsReadWriteMappedBuffer) {
    struct Rx { uint16_t controlword; int32_t target; };
    struct Tx { uint16_t statusword; int32_t actual; };

    drive_->setPDOBufferSizes(0x1705, 0x1B04,
                            sizeof(Rx), sizeof(Tx));
    ASSERT_TRUE(drive_->registerPDOBuffers());

    Rx* rx = drive_->rxPDO<Rx>();
    ASSERT_NE(rx, nullptr);
    rx->controlword = 0x000F;
    rx->target = 123456;

    const auto* buf = drive_->getRxPDOBuffer();
    uint16_t cw = 0;
    std::memcpy(&cw, buf, sizeof(cw));
    EXPECT_EQ(cw, 0x000F);

    const Tx* tx = drive_->txPDO<Tx>();
    ASSERT_NE(tx, nullptr);
    EXPECT_EQ(tx->statusword, 0);  // fresh mapping is zero-initialised
}

// ============================================================================
// controlledShutdown
// ============================================================================

TEST_F(CiA402OpTransitionTest, ControlledShutdownNoneRunsBrakeAction) {
    addSmAssign();
    ASSERT_TRUE(drive_->transitionToOp(false));
    ASSERT_EQ(slave_->getState(), SlaveState::OP);

    bool brake_ran = false;
    CiA402Drive::ControlledShutdownConfig cfg;
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::None;
    cfg.brake_delay_ms = 0;
    cfg.brake_action = [&] {
        brake_ran = true;
        return true;
    };
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
    EXPECT_TRUE(brake_ran);
}

TEST_F(CiA402OpTransitionTest, ControlledShutdownInstantStopViaSdo) {
    addSmAssign();
    // Velocity objects for the SDO fallback path (no live PDO exchange).
    addU(0x60FF, 0, 0, 4);   // Target velocity
    addU(0x606C, 0, 0, 4);   // Velocity actual value

    CiA402Drive::ControlledShutdownConfig cfg;
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::InstantHardStop;
    cfg.standstill_ms = 0;
    cfg.brake_delay_ms = 0;
    EXPECT_TRUE(drive_->controlledShutdown(cfg));

    // Phase 2 writes the shutdown controlword — the emulator's CiA402
    // state machine reacts to the disable-voltage transition.
    const auto* entry = slave_->odEntry(0x60FF, 0);
    ASSERT_NE(entry, nullptr);
    ASSERT_GE(entry->data.size(), 4u);
    int32_t tv = 0;
    std::memcpy(&tv, entry->data.data(), sizeof(tv));
    EXPECT_EQ(tv, 0);
}

TEST_F(CiA402OpTransitionTest, ControlledShutdownWithoutBrakeAction) {
    CiA402Drive::ControlledShutdownConfig cfg;
    cfg.stop_mode = CiA402Drive::ShutdownStopMode::None;
    cfg.brake_delay_ms = 0;
    EXPECT_TRUE(drive_->controlledShutdown(cfg));
}

} // namespace
