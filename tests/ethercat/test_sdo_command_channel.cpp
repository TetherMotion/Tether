/**
 * @file test_sdo_command_channel.cpp
 * @brief Coverage for SdoCommandChannel (src/ethercat/SdoCommandChannel.cpp):
 *        the Slave& free-function overloads, the command/response register
 *        protocol (reset/send/poll/wait/sendAndWait), readErrorCode,
 *        pollUntil, waitUntilMatch, and the naming/observer hooks.
 *
 * Uses a bare Slave subclass with scripted SDO reads/writes — no bus,
 * no emulator, sub-millisecond polls via a 1ms poll interval.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <span>
#include <stop_token>
#include <tuple>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SdoCommandChannel.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using namespace std::chrono_literals;

namespace {

/// Slave with scripted per-(index,sub) U32 read answers and recorded writes.
class ScriptedSlave : public Slave {
public:
    explicit ScriptedSlave(Master& m) : Slave(m, 0) {}

    std::map<std::pair<uint16_t, uint8_t>, std::deque<uint32_t>> reads;
    std::map<std::pair<uint16_t, uint8_t>, SlaveError> errors;
    std::vector<std::tuple<uint16_t, uint8_t, uint32_t>> writes;
    SlaveError write_error = SlaveError::Ok;

    void script(uint16_t idx, uint8_t sub, std::initializer_list<uint32_t> vals) {
        reads[{idx, sub}] = std::deque<uint32_t>(vals.begin(), vals.end());
    }

    SlaveError sdoReadU32(uint16_t idx, uint8_t sub, uint32_t& out) override {
        auto k = std::make_pair(idx, sub);
        auto eit = errors.find(k);
        if (eit != errors.end()) return eit->second;
        auto it = reads.find(k);
        if (it == reads.end() || it->second.empty()) return SlaveError::SDOError;
        out = it->second.front();
        if (it->second.size() > 1) it->second.pop_front();  // last value repeats
        return SlaveError::Ok;
    }
    SlaveError sdoReadU16(uint16_t idx, uint8_t sub, uint16_t& out) override {
        uint32_t v = 0;
        auto err = sdoReadU32(idx, sub, v);
        if (err == SlaveError::Ok) out = static_cast<uint16_t>(v);
        return err;
    }
    SlaveError sdoReadU8(uint16_t idx, uint8_t sub, uint8_t& out) override {
        uint32_t v = 0;
        auto err = sdoReadU32(idx, sub, v);
        if (err == SlaveError::Ok) out = static_cast<uint8_t>(v);
        return err;
    }
    SlaveError sdoWriteU32(uint16_t idx, uint8_t sub, uint32_t val) override {
        writes.emplace_back(idx, sub, val);
        return write_error;
    }
};

SdoCommandChannel::Config fastConfig() {
    SdoCommandChannel::Config c;
    c.command_index = 0xFB00;
    c.command_subindex = 0;
    c.response_index = 0xFB01;
    c.response_subindex = 0;
    c.error_index = 0xFB02;
    c.error_subindex = 0;
    c.command_timeout = 200ms;
    c.poll_interval = 1ms;
    return c;
}

// ============================================================================
// Slave& free-function overloads (cover the makeSdoReadFn adapters)
// ============================================================================

TEST(SdoChannelSlaveFnsCov, WaitForSdoValuesSlaveOverload) {
    Master m;
    ScriptedSlave s(m);
    s.script(0x6041, 0, {0x1234});

    SdoExpectation exp[] = {{0x6041, 0, 0x1234}};
    EXPECT_TRUE(waitForSdoValues(s, exp, 1ms, 100ms));
}

TEST(SdoChannelSlaveFnsCov, WaitForSdoValuesSlaveOverloadTimeout) {
    Master m;
    ScriptedSlave s(m);
    s.script(0x6041, 0, {0x1234});

    SdoExpectation exp[] = {{0x6041, 0, 0xFFFF}};   // never matches
    EXPECT_FALSE(waitForSdoValues(s, exp, 1ms, 20ms));
}

TEST(SdoChannelSlaveFnsCov, PollSdoObjectsSlaveOverload) {
    Master m;
    ScriptedSlave s(m);
    s.script(0x1000, 0, {0xAA});        // U8
    s.script(0x2000, 0, {0xBBBB});      // U16
    s.script(0x3000, 0, {0xCCCCCCCC});  // U32
    s.errors[{0x4000, 0}] = SlaveError::SDOError;

    SdoObjectEntry objs[] = {
        {0x1000, 0, 1}, {0x2000, 0, 2}, {0x3000, 0, 4}, {0x4000, 0, 4},
    };
    uint32_t values[4] = {};
    bool ok[4] = {true, true, true, true};
    size_t errCalls = 0;

    size_t n = pollSdoObjects(s, objs, std::span(values), std::span(ok),
                              {}, [&](size_t, const SdoObjectEntry&, SlaveError) {
                                  ++errCalls;
                              });
    EXPECT_EQ(n, 3u);
    EXPECT_TRUE(ok[0] && ok[1] && ok[2]);
    EXPECT_FALSE(ok[3]);
    EXPECT_EQ(values[0], 0xAAu);
    EXPECT_EQ(values[1], 0xBBBBu);
    EXPECT_EQ(values[2], 0xCCCCCCCCu);
    EXPECT_EQ(errCalls, 1u);
}

// ============================================================================
// SdoCommandChannel — command protocol
// ============================================================================

TEST(SdoChannelCov, SendCommandWritesCommandRegister) {
    Master m;
    ScriptedSlave s(m);
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");

    EXPECT_TRUE(ch.sendCommand(0x42));
    ASSERT_EQ(s.writes.size(), 1u);
    EXPECT_EQ(std::get<0>(s.writes[0]), 0xFB00);
    EXPECT_EQ(std::get<2>(s.writes[0]), 0x42u);
}

TEST(SdoChannelCov, SendCommandErrorPropagates) {
    Master m;
    ScriptedSlave s(m);
    s.write_error = SlaveError::SDOError;
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    EXPECT_FALSE(ch.sendCommand(1));
}

TEST(SdoChannelCov, PollResponseReadsRegister) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0xABCD});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");

    uint32_t out = 0;
    EXPECT_TRUE(ch.pollResponse(out));
    EXPECT_EQ(out, 0xABCDu);
}

TEST(SdoChannelCov, PollResponseErrorReturnsFalse) {
    Master m;
    ScriptedSlave s(m);
    s.errors[{0xFB01, 0}] = SlaveError::SDOError;
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    uint32_t out = 0;
    EXPECT_FALSE(ch.pollResponse(out));
}

TEST(SdoChannelCov, ResetResponseDisabledIsNoop) {
    Master m;
    ScriptedSlave s(m);
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    EXPECT_TRUE(ch.resetResponse());           // pre_command_reset off
    EXPECT_TRUE(s.writes.empty());
}

TEST(SdoChannelCov, ResetResponseEnabledWrites) {
    Master m;
    ScriptedSlave s(m);
    auto cfg = fastConfig();
    cfg.pre_command_reset = true;
    cfg.reset_value = 0x0F;
    SdoCommandChannel ch(s, cfg, "cov_chan");
    EXPECT_TRUE(ch.resetResponse());
    ASSERT_EQ(s.writes.size(), 1u);
    EXPECT_EQ(std::get<2>(s.writes[0]), 0x0Fu);
}

TEST(SdoChannelCov, ResetResponseWriteErrorFails) {
    Master m;
    ScriptedSlave s(m);
    s.write_error = SlaveError::SDOError;
    auto cfg = fastConfig();
    cfg.pre_command_reset = true;
    SdoCommandChannel ch(s, cfg, "cov_chan");
    EXPECT_FALSE(ch.resetResponse());
}

TEST(SdoChannelCov, WaitForResponseSuccess) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x00, 0x01});    // second poll returns 1
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");

    auto r = ch.waitForResponse(0x42,
        [](uint32_t, uint32_t resp, Slave&) { return resp == 1; },
        100ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Success);
    EXPECT_EQ(ch.lastResponse(), 1u);
}

TEST(SdoChannelCov, WaitForResponseRejected) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0xDEAD});
    auto cfg = fastConfig();
    cfg.failure_response = 0xDEAD;
    SdoCommandChannel ch(s, cfg, "cov_chan");

    auto r = ch.waitForResponse(0x42, {}, 100ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Rejected);
}

TEST(SdoChannelCov, WaitForResponseTimeout) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x55});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");

    auto r = ch.waitForResponse(0x42,
        [](uint32_t, uint32_t resp, Slave&) { return resp == 0x99; },
        30ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Timeout);
}

TEST(SdoChannelCov, WaitForResponseSdoErrorsRetried) {
    Master m;
    ScriptedSlave s(m);
    // First reads fail (not scripted → SDOError), then succeed — errors
    // are retried, not fatal.
    s.script(0xFB01, 0, {});
    s.reads[{0xFB01, 0}].push_back(0x7);  // after dequeuing-fail nothing…
    // Simpler: error map for first calls impossible; script one answer
    // after transient errors by pre-seeding empty then valid.
    s.errors[{0xFB01, 0}] = SlaveError::SDOError;
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    auto r = ch.waitForResponse(0x42, {}, 30ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Timeout);   // retries → timeout
}

TEST(SdoChannelCov, WaitForResponseCancelled) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x00});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    std::stop_source src;
    ch.setStopToken(src.get_token());
    src.request_stop();
    auto r = ch.waitForResponse(0x42, {}, 5000ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Cancelled);
}

TEST(SdoChannelCov, SendAndWaitFullSequence) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x01});
    auto cfg = fastConfig();
    cfg.pre_command_reset = true;
    cfg.reset_value = 0;
    cfg.post_command_reset = true;
    SdoCommandChannel ch(s, cfg, "cov_chan");

    auto r = ch.sendAndWait(0x42,
        [](uint32_t, uint32_t resp, Slave&) { return resp == 1; },
        100ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Success);
    // Writes: pre-reset(0) + command(0x42) + post-reset(0)
    ASSERT_EQ(s.writes.size(), 3u);
    EXPECT_EQ(std::get<2>(s.writes[0]), 0u);
    EXPECT_EQ(std::get<2>(s.writes[1]), 0x42u);
    EXPECT_EQ(std::get<2>(s.writes[2]), 0u);
}

TEST(SdoChannelCov, SendAndWaitResetFailureAborts) {
    Master m;
    ScriptedSlave s(m);
    s.write_error = SlaveError::SDOError;
    auto cfg = fastConfig();
    cfg.pre_command_reset = true;
    SdoCommandChannel ch(s, cfg, "cov_chan");
    auto r = ch.sendAndWait(0x42, {}, 50ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::SDOError);   // reset write failed
}

TEST(SdoChannelCov, ReadErrorCodeNoRegister) {
    Master m;
    ScriptedSlave s(m);
    auto cfg = fastConfig();
    cfg.error_index = 0;                       // not configured
    SdoCommandChannel ch(s, cfg, "cov_chan");
    EXPECT_EQ(ch.readErrorCode(), 0u);
}

TEST(SdoChannelCov, ReadErrorCodeConfigured) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB02, 0, {0xE0E0});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    EXPECT_EQ(ch.readErrorCode(), 0xE0E0u);
}

TEST(SdoChannelCov, ReadErrorCodeReadFailsReturnsZero) {
    Master m;
    ScriptedSlave s(m);
    s.errors[{0xFB02, 0}] = SlaveError::SDOError;
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    EXPECT_EQ(ch.readErrorCode(), 0u);
}

TEST(SdoChannelCov, PollUntilMatchesAndObserves) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x10, 0x11, 0x12});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");

    std::vector<uint32_t> seen;
    int observerCalls = 0;
    ch.setPollObserver([&] { ++observerCalls; });
    auto r = ch.pollUntil(0xFB01, 0,
        [](uint32_t v) { return v == 0x12; },
        100ms,
        [&](uint32_t v) { seen.push_back(v); });
    EXPECT_EQ(r, SdoCommandChannel::Result::Success);
    EXPECT_GE(seen.size(), 2u);               // 0x10, 0x11, then 0x12
    EXPECT_EQ(seen.back(), 0x12u);
    EXPECT_GT(observerCalls, 0);
}

TEST(SdoChannelCov, PollUntilTimeout) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x00});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    auto r = ch.pollUntil(0xFB01, 0,
        [](uint32_t v) { return v == 1; }, 30ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Timeout);
}

TEST(SdoChannelCov, PollUntilCancelled) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x00});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    std::stop_source src;
    ch.setStopToken(src.get_token());
    src.request_stop();
    auto r = ch.pollUntil(0xFB01, 0, nullptr, 5000ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Cancelled);
}

TEST(SdoChannelCov, WaitUntilMatchDelegates) {
    Master m;
    ScriptedSlave s(m);
    s.script(0x6041, 0, {0x77});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    SdoExpectation exp[] = {{0x6041, 0, 0x77}};
    EXPECT_TRUE(ch.waitUntilMatch(exp, 100ms));

    SdoExpectation bad[] = {{0x6041, 0, 0x01}};
    EXPECT_FALSE(ch.waitUntilMatch(bad, 20ms));
}

TEST(SdoChannelCov, ResponseNamerUsedInLogs) {
    Master m;
    ScriptedSlave s(m);
    s.script(0xFB01, 0, {0x01});
    SdoCommandChannel ch(s, fastConfig(), "cov_chan");
    bool namerCalled = false;
    ch.setResponseNamer([&](uint32_t v) {
        namerCalled = true;
        return "resp_" + std::to_string(v);
    });
    auto r = ch.waitForResponse(0x42,
        [](uint32_t, uint32_t resp, Slave&) { return resp == 1; },
        100ms);
    EXPECT_EQ(r, SdoCommandChannel::Result::Success);
    EXPECT_TRUE(namerCalled);
    EXPECT_EQ(ch.config().response_index, 0xFB01);
}

}  // namespace
