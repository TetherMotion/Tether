#include <gtest/gtest.h>
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Master.hpp"

// NOTE: Removed overriding test to allow default implementation coverage when
// running coverage-enabled test binary. If you need an override test, place it
// in a separate test binary to avoid hiding the default weak symbol.

TEST(DC_TimeSource_RemovalNotice, Present) {
    // Ensure DC stats API is available and returns a valid default
    EtherCAT::Master master;
    auto stats = master.dc().getStats();
    EXPECT_EQ(stats.cycle_count, 0u);
}
