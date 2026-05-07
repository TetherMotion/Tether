#include <gtest/gtest.h>
#include "tether/ethercat/ESIParser.hpp"

TEST(ESIParser, ParsesStepperOnlineXML) {
    std::vector<EtherCAT::ESI::DeviceInfo> devices;
    std::string err;
    std::string xmlpath = std::string(TEST_PROJECT_ROOT) + "/STEPPERONLINE_A6_Servo_V0.02.xml";
    // Try alternative common locations if not found
    bool ok = EtherCAT::ESI::parseESIFile(xmlpath, devices, err);
    if (!ok) {
        std::string alt = std::string(TEST_PROJECT_ROOT) + "/../STEPPERONLINE_A6_Servo_V0.02.xml";
        ok = EtherCAT::ESI::parseESIFile(alt, devices, err);
    }
    if (!ok) {
        // Try workspace root
        std::string alt2 = "/home/uli/dev/ESP32EtherCAT/STEPPERONLINE_A6_Servo_V0.02.xml";
        ok = EtherCAT::ESI::parseESIFile(alt2, devices, err);
    }
    ASSERT_TRUE(ok) << err;
    ASSERT_FALSE(devices.empty());
    auto& d = devices[0];
    // Expect at least one SM and the first mailbox start address to be 0x1000
    ASSERT_FALSE(d.syncManagers.empty());
    EXPECT_EQ(d.syncManagers[0].startAddress, 0x1000);
    EXPECT_EQ(d.syncManagers[0].control, 0x26);

    // PDO parsing: expect RxPDOs and TxPDOs to be present and non-empty
    ASSERT_FALSE(d.rxPdos.empty());
    ASSERT_FALSE(d.txPdos.empty());
    // Check that first RxPDO has an entry with index 0x6040 (Controlword)
    bool found6040 = false;
    for (const auto& e : d.rxPdos[0].entries) {
        if (e.index == 0x6040) found6040 = true;
    }
    EXPECT_TRUE(found6040);

    // JSON emission should contain rxPdos
    std::string j = EtherCAT::ESI::formatDeviceJSON(d);
    EXPECT_NE(j.find("\"rxPdos\""), std::string::npos);
}

