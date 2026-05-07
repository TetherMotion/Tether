#include <gtest/gtest.h>
#include "tether/ethercat/ESIParser.hpp"

TEST(ESIParserAxia, SyncManagerCount) {
    std::vector<EtherCAT::ESI::DeviceInfo> devices;
    std::string err;
    std::string xmlpath = std::string(TEST_PROJECT_ROOT) + "/Axia80/200807 - ATI Axia EtherCAT FT.xml";
    bool ok = EtherCAT::ESI::parseESIFile(xmlpath, devices, err);
    if (!ok) {
        std::string alt = std::string(TEST_PROJECT_ROOT) + "/../Axia80/200807 - ATI Axia EtherCAT FT.xml";
        ok = EtherCAT::ESI::parseESIFile(alt, devices, err);
    }
    if (!ok) {
        std::string alt2 = "/home/uli/dev/ESP32EtherCAT/Axia80/200807 - ATI Axia EtherCAT FT.xml";
        ok = EtherCAT::ESI::parseESIFile(alt2, devices, err);
    }
    ASSERT_TRUE(ok) << err;
    ASSERT_FALSE(devices.empty());
    auto& d = devices[0];
    // This ESI explicitly declares 4 Sync Managers
    EXPECT_EQ(d.syncManagers.size(), 4u);
}
