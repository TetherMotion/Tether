#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATDC.hpp"

TEST(DC_TimeSource_Defaults, CallsDefaultImplementation) {
    // Call the default implementations (weak symbols from dc_time_source.cpp)
    uint64_t t1 = EtherCAT::DC::ecdc_get_master_time_ns();
    // Should return a non-zero time on host via EspCompat mock
    EXPECT_GT(t1, 0ULL);

    // Init and deinit should be callable
    EXPECT_TRUE(EtherCAT::DC::ecdc_init_time_source());
    EtherCAT::DC::ecdc_deinit_time_source();
}
