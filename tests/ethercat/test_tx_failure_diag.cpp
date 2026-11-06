/**
 * @file test_tx_failure_diag.cpp
 * @brief Coverage for raw/TxFailureDiagnostics.cpp — the probe() early
 *        return paths, counter notifiers, worker emit loops, streak
 *        escalation, wire-probe streak trigger, and start/stop lifecycle.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#ifdef __linux__
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "raw/TxFailureDiagnostics.hpp"

using namespace EtherCAT;

namespace {

// ============================================================================
// probe() — pure/static paths
// ============================================================================

TEST(TxDiagCov, ProbeNegativeFd) {
    std::string r = TxFailureDiagnostics::probe(-1);
#ifdef __linux__
    EXPECT_NE(r.find("no TX socket fd"), std::string::npos);
#else
    EXPECT_TRUE(r.empty());
#endif
}

TEST(TxDiagCov, ProbeNonPacketSocket) {
#ifdef __linux__
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(fd, 0);
    std::string r = TxFailureDiagnostics::probe(fd);
    EXPECT_NE(r.find("not a bound AF_PACKET"), std::string::npos);
    ::close(fd);
#endif
}

TEST(TxDiagCov, ProbePacketSocketIfAvailable) {
#ifdef __linux__
    int fd = ::socket(AF_PACKET, SOCK_RAW, 0);
    if (fd < 0) {
        GTEST_SKIP() << "no CAP_NET_RAW";
    }
    // Unbound AF_PACKET → "not a bound" report (no bind()).
    std::string r = TxFailureDiagnostics::probe(fd);
    EXPECT_FALSE(r.empty());
    ::close(fd);
#endif
}

// ============================================================================
// Lifecycle
// ============================================================================

TEST(TxDiagCov, StartStopIdempotent) {
    TxFailureDiagnostics d(nullptr, nullptr, nullptr);
    d.start();
    EXPECT_EQ(d.spawns(), 1u);
    d.start();                               // already running → no re-spawn
    EXPECT_EQ(d.spawns(), 1u);
    d.stop();
    d.stop();                                // idempotent
    d.start();                               // restart works
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_GE(d.spawns(), 2u);
    // destructor stops implicitly
}

// ============================================================================
// Worker emit paths — all five categories
// ============================================================================

TEST(TxDiagCov, WorkerEmitsAllCategories) {
    int probe_calls = 0;
    int errno_calls = 0;
    int esc_calls = 0;
    TxFailureDiagnostics d(
        [&] { ++probe_calls; return std::string(); },
        [&] { ++errno_calls; return 111; },
        [&] { ++esc_calls; return std::string("ALIVE"); });
    d.start();

    d.noteSendFailure(0);                    // mask 0 → exchangeLRW msg
    d.noteSendFailure(0xFF);                 // mask → ForSlaves msg
    d.noteNoResponseSlot();
    d.noteTimeout("exchangeLRW");
    d.noteTimeout("exchangeLRW");            // suppressed counter
    d.noteWkcZero("exchangeLRW");
    d.noteStall(5'000'000);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    d.stop();
    EXPECT_GE(errno_calls, 1);               // errno_fn consulted on send fail
}

TEST(TxDiagCov, EscalationFiresOnTimeoutStreak) {
    std::atomic<int> esc_calls{0};
    TxFailureDiagnostics d(nullptr, nullptr,
                           [&] { ++esc_calls; return std::string("ALIVE"); });
    d.setEscalateAfterTimeouts(2);
    d.start();
    d.noteTimeout("xchg");
    d.noteTimeout("xchg");                   // ct=2 → crosses multiple of 2
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    d.stop();
    EXPECT_GE(esc_calls.load(), 1);
}

TEST(TxDiagCov, EscalationDisabledAtZero) {
    std::atomic<int> esc_calls{0};
    TxFailureDiagnostics d(nullptr, nullptr,
                           [&] { ++esc_calls; return std::string("X"); });
    d.setEscalateAfterTimeouts(0);
    d.start();
    for (int i = 0; i < 5; ++i) d.noteTimeout("xchg");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    d.stop();
    EXPECT_EQ(esc_calls.load(), 0);
}

TEST(TxDiagCov, EscalationEmptyVerdictStillCounts) {
    std::atomic<int> esc_calls{0};
    TxFailureDiagnostics d(nullptr, nullptr,
                           [&] { ++esc_calls; return std::string(); });
    d.setEscalateAfterTimeouts(1);
    d.start();
    d.noteTimeout("xchg");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    d.stop();
    EXPECT_GE(esc_calls.load(), 1);          // called; empty verdict not logged
}

TEST(TxDiagCov, SuccessResetsTimeoutStreak) {
    std::atomic<int> esc_calls{0};
    TxFailureDiagnostics d(nullptr, nullptr,
                           [&] { ++esc_calls; return std::string("V"); });
    d.setEscalateAfterTimeouts(3);
    d.start();
    d.noteTimeout("x");
    d.noteTimeout("x");
    d.noteSuccess();                         // resets streak before 3
    d.noteTimeout("x");
    d.noteTimeout("x");                      // ct=2 < 3 → no escalation
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    d.stop();
    EXPECT_EQ(esc_calls.load(), 0);
}

TEST(TxDiagCov, SendFailStreakTriggersWireProbe) {
    std::atomic<int> probe_calls{0};
    TxFailureDiagnostics d(
        [&] { ++probe_calls; return std::string("nic state | DIAGNOSIS: bad"); },
        nullptr, nullptr);
    d.start();
    for (int i = 0; i < 12; ++i) d.noteSendFailure(0);   // streak ≥10
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    d.stop();
    EXPECT_GE(probe_calls.load(), 1);
}

TEST(TxDiagCov, SendOkResetsStreakBelowThreshold) {
    std::atomic<int> probe_calls{0};
    TxFailureDiagnostics d(
        [&] { ++probe_calls; return std::string("report"); },
        nullptr, nullptr);
    d.start();
    for (int i = 0; i < 8; ++i) d.noteSendFailure(0);
    d.noteSendOk();                          // streak → 0
    for (int i = 0; i < 8; ++i) d.noteSendFailure(0);
    // streak never reached 10 → probe never invoked
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    d.stop();
    EXPECT_EQ(probe_calls.load(), 0);
}

TEST(TxDiagCov, NullCallbacksSafe) {
    TxFailureDiagnostics d(nullptr, nullptr, nullptr);
    d.setEscalateAfterTimeouts(1);
    d.start();
    d.noteSendFailure(1);
    d.noteTimeout("t");
    for (int i = 0; i < 11; ++i) d.noteSendFailure(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    d.stop();
    SUCCEED();                               // no crash with null fns
}

}  // namespace
