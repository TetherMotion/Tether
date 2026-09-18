/**
 * @file test_sdo_polling.cpp
 * @brief Unit tests for the generic SDO polling helpers in
 *        SdoCommandChannel.hpp — pollSdoObjects() and waitForSdoValues()
 *        exercised through the transport-agnostic SdoReadFn overloads.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <stop_token>
#include <vector>

#include "tether/ethercat/SdoCommandChannel.hpp"

using namespace EtherCAT;
using namespace std::chrono_literals;

namespace {

/// Scriptable SDO read double: serves values from a (index,subindex) map,
/// optionally scripted per call (each read pops the next queued value or
/// error for that object).
struct FakeSdo {
    using Key = uint32_t;   // index << 8 | subindex
    static constexpr Key key(uint16_t i, uint8_t s) {
        return (static_cast<Key>(i) << 8) | s;
    }

    std::map<Key, uint32_t> values;
    std::map<Key, std::deque<uint32_t>> scripted;
    std::map<Key, SlaveError> errors;
    int calls = 0;

    SdoReadFn fn() {
        return [this](uint16_t index, uint8_t subindex, uint8_t /*bytes*/,
                      uint32_t& out) -> SlaveError {
            ++calls;
            const Key k = key(index, subindex);
            if (auto it = scripted.find(k);
                it != scripted.end() && !it->second.empty()) {
                out = it->second.front();
                it->second.pop_front();
                return SlaveError::Ok;
            }
            if (auto it = errors.find(k); it != errors.end()) {
                return it->second;
            }
            if (auto it = values.find(k); it != values.end()) {
                out = it->second;
                return SlaveError::Ok;
            }
            return SlaveError::SDOError;
        };
    }
};

} // namespace

// ============================================================================
// sdoPollResultToString
// ============================================================================

TEST(SdoPolling, ResultNames) {
    EXPECT_STREQ(sdoPollResultToString(SdoPollResult::Success), "Success");
    EXPECT_STREQ(sdoPollResultToString(SdoPollResult::Rejected), "Rejected");
    EXPECT_STREQ(sdoPollResultToString(SdoPollResult::Timeout), "Timeout");
    EXPECT_STREQ(sdoPollResultToString(SdoPollResult::SDOError), "SDOError");
    EXPECT_STREQ(sdoPollResultToString(SdoPollResult::Cancelled), "Cancelled");
}

// ============================================================================
// pollSdoObjects
// ============================================================================

TEST(SdoPolling, PollObjectsReadsAllWidths) {
    FakeSdo sdo;
    sdo.values[FakeSdo::key(0x4000, 0)] = 0x11;
    sdo.values[FakeSdo::key(0x4002, 0)] = 0x2222;
    sdo.values[FakeSdo::key(0xF101, 0)] = 0x33333333;

    const SdoObjectEntry objects[] = {
        {0x4000, 0x00, 1}, {0x4002, 0x00, 2}, {0xF101, 0x00, 4},
    };
    uint32_t values[3] = {};
    bool ok[3] = {};

    const size_t n = pollSdoObjects(sdo.fn(), objects, values, ok);
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(values[0], 0x11u);
    EXPECT_EQ(values[1], 0x2222u);
    EXPECT_EQ(values[2], 0x33333333u);
    EXPECT_TRUE(ok[0] && ok[1] && ok[2]);
}

TEST(SdoPolling, PollObjectsMarksFailuresAndContinues) {
    FakeSdo sdo;
    sdo.values[FakeSdo::key(0x4000, 0)] = 0xAA;
    sdo.errors[FakeSdo::key(0x4001, 0)] = SlaveError::SDOAborted;
    sdo.values[FakeSdo::key(0x4002, 0)] = 0xCC;

    const SdoObjectEntry objects[] = {
        {0x4000, 0x00, 1}, {0x4001, 0x00, 1}, {0x4002, 0x00, 1},
    };
    uint32_t values[3] = {};
    bool ok[3] = {};
    std::vector<size_t> error_indices;

    const size_t n = pollSdoObjects(
        sdo.fn(), objects, values, ok, {},
        [&](size_t i, const SdoObjectEntry& e, SlaveError err) {
            error_indices.push_back(i);
            EXPECT_EQ(e.index, 0x4001u);
            EXPECT_EQ(err, SlaveError::SDOAborted);
        });

    EXPECT_EQ(n, 2u);                  // continued past the failure
    EXPECT_TRUE(ok[0]);
    EXPECT_FALSE(ok[1]);
    EXPECT_TRUE(ok[2]);
    ASSERT_EQ(error_indices.size(), 1u);
    EXPECT_EQ(error_indices[0], 1u);
}

TEST(SdoPolling, PollObjectsAbortBetweenObjects) {
    FakeSdo sdo;
    sdo.values[FakeSdo::key(0x4000, 0)] = 1;
    sdo.values[FakeSdo::key(0x4001, 0)] = 2;
    sdo.values[FakeSdo::key(0x4002, 0)] = 3;

    const SdoObjectEntry objects[] = {
        {0x4000, 0x00, 1}, {0x4001, 0x00, 1}, {0x4002, 0x00, 1},
    };
    uint32_t values[3] = {9, 9, 9};
    bool ok[3] = {false, false, false};

    // Abort after the first object: entries not attempted keep prior
    // values and ok flags.
    int abort_after = 1;
    const size_t n = pollSdoObjects(
        sdo.fn(), objects, values, ok,
        [&] { return sdo.calls >= abort_after; });

    EXPECT_EQ(n, 1u);
    EXPECT_EQ(sdo.calls, 1);
    EXPECT_EQ(values[0], 1u);
    EXPECT_TRUE(ok[0]);
    EXPECT_EQ(values[1], 9u);          // untouched
    EXPECT_FALSE(ok[1]);               // untouched
    EXPECT_EQ(values[2], 9u);
    EXPECT_FALSE(ok[2]);
}

// ============================================================================
// waitForSdoValues
// ============================================================================

TEST(SdoPolling, WaitForValuesImmediateMatch) {
    FakeSdo sdo;
    sdo.values[FakeSdo::key(0xF201, 0)] = 0xDEADBEEF;

    const SdoExpectation exp[] = {
        {0xF201, 0x00, 0xDEADBEEF, "FNI CRC"},
    };
    EXPECT_TRUE(waitForSdoValues(sdo.fn(), exp, 1ms, 100ms));
    EXPECT_EQ(sdo.calls, 1);           // matched on first poll
}

TEST(SdoPolling, WaitForValuesMatchesAfterChange) {
    FakeSdo sdo;
    auto& script = sdo.scripted[FakeSdo::key(0xF101, 0)];
    script = {0, 0, 6};                // reaches expected on 3rd poll

    const SdoExpectation exp[] = {
        {0xF101, 0x00, 6, "system state"},
    };
    EXPECT_TRUE(waitForSdoValues(sdo.fn(), exp, 1ms, 2000ms));
    EXPECT_EQ(sdo.calls, 3);
}

TEST(SdoPolling, WaitForValuesAllMustMatch) {
    FakeSdo sdo;
    sdo.values[FakeSdo::key(0xF201, 0)] = 0x11111111;
    sdo.values[FakeSdo::key(0xF211, 0)] = 0x22222222;

    const SdoExpectation exp[] = {
        {0xF201, 0x00, 0x11111111, "FNI"},
        {0xF211, 0x00, 0x22222222, "RSP"},
        {0xF221, 0x00, 0x33333333, "SDD"},   // never matches -> timeout
    };
    EXPECT_FALSE(waitForSdoValues(sdo.fn(), exp, 1ms, 50ms));
}

TEST(SdoPolling, WaitForValuesReadErrorFailsFast) {
    FakeSdo sdo;
    sdo.errors[FakeSdo::key(0xF201, 0)] = SlaveError::SDOError;

    const SdoExpectation exp[] = {
        {0xF201, 0x00, 0x11111111, "FNI"},
    };
    EXPECT_FALSE(waitForSdoValues(sdo.fn(), exp, 1ms, 5000ms));
    EXPECT_EQ(sdo.calls, 1);           // did not retry until timeout
}

TEST(SdoPolling, WaitForValuesStopTokenCancels) {
    FakeSdo sdo;
    sdo.values[FakeSdo::key(0xF101, 0)] = 0;   // never matches expected 6

    std::stop_source src;
    src.request_stop();

    const SdoExpectation exp[] = {
        {0xF101, 0x00, 6, "system state"},
    };
    EXPECT_FALSE(waitForSdoValues(sdo.fn(), exp, 1ms, 5000ms,
                                  src.get_token()));
    EXPECT_EQ(sdo.calls, 0);           // cancelled before first read
}

TEST(SdoPolling, WaitForValuesEmptyExpectations) {
    FakeSdo sdo;
    EXPECT_TRUE(waitForSdoValues(sdo.fn(), {}, 1ms, 10ms));
    EXPECT_EQ(sdo.calls, 0);
}
