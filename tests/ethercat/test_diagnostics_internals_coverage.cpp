/**
 * @file test_diagnostics_internals_coverage.cpp
 * @brief Coverage for ethercat diagnostics/health internals:
 *        CyclicHealth::describe(), the SlaveSync gate/coordinator/
 *        recovery handler, and the Master diagnostics dump helpers
 *        over an emulated bus.
 */

#include <gtest/gtest.h>

#include <stop_token>
#include <string>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CyclicHealth.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Diagnostics.hpp"
#include "tether/ethercat/SlaveSync.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/profiles/cia402/CiA402StateUtils.hpp"

using namespace EtherCAT;

// ============================================================================
// CyclicHealth::describe
// ============================================================================

TEST(CyclicHealthTest, DescribeDefaultsHaveNoRttOrSliceLines) {
    CyclicHealth h;
    auto s = h.describe();
    EXPECT_NE(s.find("cycles=0"), std::string::npos);
    EXPECT_NE(s.find("wire_loss=0"), std::string::npos);
    EXPECT_EQ(s.find("rtt_us="), std::string::npos);
    EXPECT_EQ(s.find("slice(s) stuck"), std::string::npos);
    EXPECT_EQ(s.find("image slice"), std::string::npos);
}

TEST(CyclicHealthTest, DescribeIncludesRttWhenSampled) {
    CyclicHealth h;
    h.rtt_us_min = 10;
    h.rtt_us_avg = 20;
    h.rtt_us_max = 30;
    h.rtt_samples = 5;
    auto s = h.describe();
    EXPECT_NE(s.find("rtt_us=min/10/avg/20/max/30 (5 samples)"),
              std::string::npos);
}

TEST(CyclicHealthTest, DescribeLossSplitSubtractsStale) {
    CyclicHealth h;
    h.wire_loss = 7;
    h.stale_responses = 3;
    auto s = h.describe();
    EXPECT_NE(s.find("never_returned"), std::string::npos);
    EXPECT_NE(s.find("late_replies=3"), std::string::npos);
    EXPECT_NE(s.find("≈4"), std::string::npos);
}

TEST(CyclicHealthTest, DescribeLossSplitClampsToZero) {
    CyclicHealth h;
    h.wire_loss = 1;
    h.stale_responses = 5;
    auto s = h.describe();
    EXPECT_NE(s.find("≈0"), std::string::npos);
}

TEST(CyclicHealthTest, DescribeStuckStaleWarning) {
    CyclicHealth h;
    h.slices_stuck_stale = 2;
    auto s = h.describe();
    EXPECT_NE(s.find("2 slice(s) stuck Stale"), std::string::npos);
}

TEST(CyclicHealthTest, DescribeListsFailingImageSlice) {
    CyclicHealth h;
    h.image_slice_count = 2;
    h.image_slices[0].last_status = CyclicSliceStatus::WkcError;
    h.image_slices[0].last_wkc = 1;
    h.image_slices[0].expected_wkc = 3;
    h.image_slices[0].consecutive_failures = 4;
    h.image_slices[1].last_status = CyclicSliceStatus::Ok;
    h.image_slices[1].consecutive_failures = 0;
    auto s = h.describe();
    EXPECT_NE(s.find("image slice 0:"), std::string::npos);
    EXPECT_NE(s.find("consec_fail=4"), std::string::npos);
    EXPECT_EQ(s.find("image slice 1:"), std::string::npos);
}

TEST(CyclicHealthTest, DescribeListsUntouchedDefaultSlice) {
    // A default-constructed slice reports status None (not Ok), so it is
    // listed even with zero failures.
    CyclicHealth h;
    h.image_slice_count = 1;
    auto s = h.describe();
    EXPECT_NE(s.find("image slice 0:"), std::string::npos);
}

TEST(CyclicHealthTest, DescribeListsPdoSlicesWithFailures) {
    CyclicHealth h;
    CyclicSliceHealth ok;
    ok.last_status = CyclicSliceStatus::Ok;
    ok.consecutive_failures = 0;
    CyclicSliceHealth bad;
    bad.last_status = CyclicSliceStatus::Timeout;
    bad.consecutive_failures = 2;
    h.pdo_slice_health = {ok, bad};
    auto s = h.describe();
    EXPECT_EQ(s.find("pdo slice 0:"), std::string::npos);
    EXPECT_NE(s.find("pdo slice 1:"), std::string::npos);
}

TEST(CyclicHealthTest, DescribeListsOkSliceWithConsecutiveFailures) {
    CyclicHealth h;
    CyclicSliceHealth recovered;
    recovered.last_status = CyclicSliceStatus::Ok;
    recovered.consecutive_failures = 1;
    h.pdo_slice_health = {recovered};
    auto s = h.describe();
    EXPECT_NE(s.find("pdo slice 0:"), std::string::npos);
}

// ============================================================================
// SlaveSync — gate
// ============================================================================

namespace {

class MockTransitionHandler : public Util::ISlaveTransitionHandler {
public:
    MockTransitionHandler(uint16_t idx, bool result = true)
        : idx_(idx), result_(result) {}

    bool transitionTo(ECState target, const Util::TransitionContext&) override {
        ++calls;
        last_target = target;
        return result_;
    }
    bool reinitialize() override {
        ++reinit_calls;
        return result_;
    }
    uint16_t slaveIndex() const override { return idx_; }

    int calls = 0;
    int reinit_calls = 0;
    ECState last_target = ECState::Unknown;

private:
    uint16_t idx_;
    bool result_;
};

}  // namespace

TEST(SlaveSyncGateTest, WaitAllReturnsTrueWhenAllInTarget) {
    Util::SlaveSyncGate gate;
    Util::SlaveSyncSlot a{.slave_index = 0};
    Util::SlaveSyncSlot b{.slave_index = 1};
    a.in_target_state = true;
    b.in_target_state = true;
    std::vector<Util::SlaveSyncSlot*> slots{&a, &b};
    std::stop_source src;
    EXPECT_TRUE(gate.waitAll(src.get_token(), slots));
}

TEST(SlaveSyncGateTest, WaitAllReturnsFalseOnFailure) {
    Util::SlaveSyncGate gate;
    Util::SlaveSyncSlot a{.slave_index = 0};
    a.failed = true;
    std::vector<Util::SlaveSyncSlot*> slots{&a};
    std::stop_source src;
    EXPECT_FALSE(gate.waitAll(src.get_token(), slots));
}

TEST(SlaveSyncGateTest, WaitAllReturnsFalseWhenStopRequested) {
    Util::SlaveSyncGate gate;
    Util::SlaveSyncSlot a{.slave_index = 0};  // never reaches target
    std::vector<Util::SlaveSyncSlot*> slots{&a};
    std::stop_source src;
    src.request_stop();
    EXPECT_FALSE(gate.waitAll(src.get_token(), slots));
}

TEST(SlaveSyncGateTest, WaitOneReturnsTrueInTarget) {
    Util::SlaveSyncGate gate;
    Util::SlaveSyncSlot a{.slave_index = 0};
    a.in_target_state = true;
    std::stop_source src;
    EXPECT_TRUE(gate.waitOne(src.get_token(), a));
}

TEST(SlaveSyncGateTest, WaitOneReturnsFalseOnFailure) {
    Util::SlaveSyncGate gate;
    Util::SlaveSyncSlot a{.slave_index = 0};
    a.failed = true;
    std::stop_source src;
    EXPECT_FALSE(gate.waitOne(src.get_token(), a));
}

TEST(SlaveSyncGateTest, WaitOneReturnsFalseOnStop) {
    Util::SlaveSyncGate gate;
    Util::SlaveSyncSlot a{.slave_index = 0};
    std::stop_source src;
    src.request_stop();
    EXPECT_FALSE(gate.waitOne(src.get_token(), a));
}

TEST(SlaveSyncGateTest, NotifyIsSafeWithoutWaiters) {
    Util::SlaveSyncGate gate;
    gate.notify();
}

// ============================================================================
// SlaveSync — coordinator
// ============================================================================

TEST(SlaveSyncCoordinatorTest, RegisterFindUnregister) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler h0(0), h1(1);
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &h0, .name = "A"};
    Util::SlaveSyncSlot s1{.slave_index = 1, .handler = &h1, .name = "B"};
    coord.registerSlot(s0);
    coord.registerSlot(s1);
    EXPECT_EQ(coord.slots().size(), 2u);
    EXPECT_EQ(coord.findSlot(0), &s0);
    EXPECT_EQ(coord.findSlot(7), nullptr);

    coord.unregisterSlot(0);
    EXPECT_EQ(coord.slots().size(), 1u);
    EXPECT_EQ(coord.findSlot(0), nullptr);
    coord.unregisterSlot(42);  // absent → no-op
}

TEST(SlaveSyncCoordinatorTest, ClearReadyFlags) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler h0(0);
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &h0};
    coord.registerSlot(s0);
    s0.ready = true;
    coord.clearReady(0);
    EXPECT_FALSE(s0.ready.load());
    s0.ready = true;
    coord.clearAllReady();
    EXPECT_FALSE(s0.ready.load());
    coord.clearReady(9);  // absent → no-op
}

TEST(SlaveSyncCoordinatorTest, TransitionAllToSetsFlagsAndCallsHook) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler h0(0), h1(1);
    int hook_calls = 0;
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &h0, .name = "A"};
    s0.on_reached = [&] { ++hook_calls; };
    Util::SlaveSyncSlot s1{.slave_index = 1, .handler = &h1, .name = "B"};
    coord.registerSlot(s0);
    coord.registerSlot(s1);

    Util::TransitionContext ctx;
    EXPECT_TRUE(coord.transitionAllTo(ECState::SafeOp, ctx));
    EXPECT_EQ(h0.calls, 1);
    EXPECT_EQ(h1.calls, 1);
    EXPECT_EQ(h0.last_target, ECState::SafeOp);
    EXPECT_TRUE(s0.in_target_state.load());
    EXPECT_TRUE(s1.in_target_state.load());
    EXPECT_EQ(hook_calls, 1);
}

TEST(SlaveSyncCoordinatorTest, TransitionAllToReportsFailureButContinues) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler bad(0, false), good(1, true);
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &bad, .name = "bad"};
    Util::SlaveSyncSlot s1{.slave_index = 1, .handler = &good, .name = "good"};
    coord.registerSlot(s0);
    coord.registerSlot(s1);

    Util::TransitionContext ctx{.is_recovery = true};
    EXPECT_FALSE(coord.transitionAllTo(ECState::Op, ctx));
    EXPECT_TRUE(s0.failed.load());
    EXPECT_FALSE(s0.in_target_state.load());
    EXPECT_TRUE(s1.in_target_state.load());
    EXPECT_EQ(good.calls, 1);
}

TEST(SlaveSyncCoordinatorTest, TransitionAllToSkipsNullSlots) {
    Util::SlaveSyncCoordinator coord;
    Util::SlaveSyncSlot empty{.slave_index = 0, .handler = nullptr, .name = ""};
    coord.registerSlot(empty);
    Util::TransitionContext ctx;
    EXPECT_TRUE(coord.transitionAllTo(ECState::PreOp, ctx));
}

TEST(SlaveSyncCoordinatorTest, TransitionOneUnknownSlaveFails) {
    Util::SlaveSyncCoordinator coord;
    Util::TransitionContext ctx;
    EXPECT_FALSE(coord.transitionOne(5, ECState::Op, ctx));
}

TEST(SlaveSyncCoordinatorTest, TransitionOneRespectsCallOnReached) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler h0(0);
    int hook_calls = 0;
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &h0, .name = "A"};
    s0.on_reached = [&] { ++hook_calls; };
    coord.registerSlot(s0);

    Util::TransitionContext ctx;
    EXPECT_TRUE(coord.transitionOne(0, ECState::SafeOp, ctx, false));
    EXPECT_EQ(hook_calls, 0);
    EXPECT_TRUE(coord.transitionOne(0, ECState::Op, ctx, true));
    EXPECT_EQ(hook_calls, 1);
}

TEST(SlaveSyncCoordinatorTest, TransitionAllAndWait) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler h0(0);
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &h0, .name = "A"};
    coord.registerSlot(s0);
    Util::SlaveSyncGate gate;
    std::stop_source src;
    Util::TransitionContext ctx;
    EXPECT_TRUE(coord.transitionAllAndWait(ECState::Op, ctx, gate,
                                           src.get_token()));
}

// ============================================================================
// SlaveSync — recovery handler
// ============================================================================

TEST(SyncRecoveryHandlerTest, ReinitializeSuccessCallsHook) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler h0(0);
    int hook_calls = 0;
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &h0, .name = "A"};
    s0.ready = true;
    s0.on_reached = [&] { ++hook_calls; };
    coord.registerSlot(s0);

    Util::SyncRecoveryHandler rec(coord);
    EXPECT_TRUE(rec.reinitializeSlave(0));
    EXPECT_EQ(h0.reinit_calls, 1);
    EXPECT_FALSE(s0.ready.load());
    EXPECT_TRUE(s0.in_target_state.load());
    EXPECT_EQ(hook_calls, 1);
}

TEST(SyncRecoveryHandlerTest, ReinitializeFailureSetsFailed) {
    Util::SlaveSyncCoordinator coord;
    MockTransitionHandler h0(0, false);
    Util::SlaveSyncSlot s0{.slave_index = 0, .handler = &h0, .name = "A"};
    coord.registerSlot(s0);

    Util::SyncRecoveryHandler rec(coord);
    EXPECT_FALSE(rec.reinitializeSlave(0));
    EXPECT_TRUE(s0.failed.load());
}

TEST(SyncRecoveryHandlerTest, ReinitializeUnregisteredFails) {
    Util::SlaveSyncCoordinator coord;
    Util::SyncRecoveryHandler rec(coord);
    EXPECT_FALSE(rec.reinitializeSlave(3));
}

// ============================================================================
// Master diagnostics dumps over the emulated bus
// ============================================================================

namespace {

class DiagnosticsEmulatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0x0442, "esc211");
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

    EmulatedBus bus_;
    Emulator::SlaveEmulator* slave_ = nullptr;
    Master master_;
};

}  // namespace

TEST_F(DiagnosticsEmulatorTest, MailboxConfigurationDumpRuns) {
    debugMailboxConfiguration(master_, 0, "test");
}

TEST_F(DiagnosticsEmulatorTest, ApplicationLayerDiagnosticsRuns) {
    Diagnostics::logSlaveApplicationLayerDiagnostics(master_, 0, "test");
}

TEST_F(DiagnosticsEmulatorTest, PreOperationalDiagnosticsWithoutAutoConfigure) {
    Diagnostics::PreOperationalMailboxDiagnosticsOptions opts;
    opts.attempt_auto_configure = false;
    opts.log_sdo_mailbox_config = true;
    opts.log_sii_mailbox = false;
    opts.log_sync_manager_registers = false;
    Diagnostics::logPreOperationalMailboxDiagnostics(master_, 0, "test", opts);
}

TEST_F(DiagnosticsEmulatorTest, PreOperationalDiagnosticsWithSyncManagerDump) {
    Diagnostics::PreOperationalMailboxDiagnosticsOptions opts;
    opts.attempt_auto_configure = false;
    opts.log_sdo_mailbox_config = false;
    opts.log_sii_mailbox = false;
    opts.log_sync_manager_registers = true;
    Diagnostics::logPreOperationalMailboxDiagnostics(master_, 0, "test", opts);
}
