#include <gtest/gtest.h>
#include "tether/profiles/cia401/CiA401IO.hpp"

using namespace CiA401;

TEST(Dummy, Smoke) {
    DigitalChannelState s;
    EXPECT_EQ(s.block_index, 0);
    EXPECT_EQ(s.bit_position, 0);
    EXPECT_FALSE(s.current_state);
    EXPECT_EQ(s.edge_mask, EdgeType::Any);
}
