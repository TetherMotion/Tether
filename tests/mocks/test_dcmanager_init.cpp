#include <gtest/gtest.h>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATDC.hpp"

TEST(DCManagerInitTest, InitFailsWithZeroSlaves)
{
    EtherCAT::EtherCATMaster master;
    EtherCAT::DCManager dc(master);

    EtherCAT::DC::DCConfig cfg = EtherCAT::DC::DCConfig::defaults();
    EXPECT_FALSE(dc.init(cfg, 0));  // zero slaves -> invalid
}

TEST(DCManagerInitTest, InitSucceedsWithoutNetworkInterface)
{
    EtherCAT::EtherCATMaster master;
    EtherCAT::DCManager dc(master);

    EtherCAT::DC::DCConfig cfg = EtherCAT::DC::DCConfig::defaults();
    // RawDCTransport now uses master's transport primitives;
    // no platform NI check is needed at init time.
    EXPECT_TRUE(dc.init(cfg, 1));
}
