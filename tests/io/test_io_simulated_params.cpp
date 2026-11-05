/**
 * @file test_io_simulated_params.cpp
 * @brief Coverage for the TLV command-parameter codec in
 *        SimulatedMachineService.hpp (decodeSimulatedParameters /
 *        putParameter) — every tag, truncation and unknown-tag path.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "tether/io/SimulatedMachineService.hpp"

using namespace tether::io::machine;

TEST(SimulatedParams, RoundTripAllTags) {
    std::vector<uint8_t> b;
    putParameter(b, 1, 0xF8, 1);           // mode (int8)
    putParameter(b, 2, 0x01020304, 4);     // position
    putParameter(b, 3, 0xFFFFFF9C, 4);     // delta (negative)
    putParameter(b, 4, 5000, 4);           // velocity
    putParameter(b, 5, 250, 4);            // durationMs
    putParameter(b, 6, 0x1122334455667788ULL, 8);  // alarmId

    auto p = decodeSimulatedParameters(b);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(*p->mode, static_cast<int8_t>(0xF8));
    EXPECT_EQ(*p->position, 0x01020304);
    EXPECT_EQ(*p->delta, -100);
    EXPECT_EQ(*p->velocity, 5000);
    EXPECT_EQ(*p->durationMs, 250u);
    EXPECT_EQ(*p->alarmId, 0x1122334455667788ULL);
}

TEST(SimulatedParams, EmptyInputDecodesToAllAbsent) {
    auto p = decodeSimulatedParameters({});
    ASSERT_TRUE(p.has_value());
    EXPECT_FALSE(p->mode.has_value());
    EXPECT_FALSE(p->position.has_value());
    EXPECT_FALSE(p->delta.has_value());
    EXPECT_FALSE(p->velocity.has_value());
    EXPECT_FALSE(p->durationMs.has_value());
    EXPECT_FALSE(p->alarmId.has_value());
}

TEST(SimulatedParams, TruncatedValuesFail) {
    for (const std::vector<uint8_t>& b : {
             std::vector<uint8_t>{1},                 // tag 1 needs 1 byte
             std::vector<uint8_t>{2, 0x01, 0x02},     // tag 2 needs 4
             std::vector<uint8_t>{3, 0x01},           // tag 3 needs 4
             std::vector<uint8_t>{4, 0x01, 0x02},     // tag 4 needs 4
             std::vector<uint8_t>{5, 0x01},           // tag 5 needs 4
             std::vector<uint8_t>{6, 0x01, 0x02},     // tag 6 needs 8
         }) {
        EXPECT_FALSE(decodeSimulatedParameters(b).has_value());
    }
}

TEST(SimulatedParams, UnknownTagFails) {
    EXPECT_FALSE(decodeSimulatedParameters({0x7F, 0x00}).has_value());
    // Unknown tag after a valid field also fails.
    std::vector<uint8_t> b;
    putParameter(b, 1, 3, 1);
    b.push_back(0x42);
    b.push_back(0x00);
    EXPECT_FALSE(decodeSimulatedParameters(b).has_value());
}
