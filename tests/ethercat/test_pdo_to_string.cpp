#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>

#include "tether/ethercat/utils/PDO.hpp"

TEST(PDOToString, FormatsFieldsAndValues)
{
    const std::array<uint8_t, 8> buf = {
        0x34, 0x12, // 0x1234
        0xFF, 0xFF, // -1 as int16
        0x78, 0x56, 0x34, 0x12, // 0x12345678
    };

    const std::array<EtherCAT::Utils::PDOFieldDescriptor, 3> fields = {
        EtherCAT::Utils::PDOFieldDescriptor{0x6041, 0x00, 0, 2, "Statusword"},
        EtherCAT::Utils::PDOFieldDescriptor{0x6077, 0x00, 2, 2, "TorqueActualValue"},
        EtherCAT::Utils::PDOFieldDescriptor{0x607A, 0x00, 4, 4, "TargetPosition"},
    };

    const std::string s = EtherCAT::Utils::pdoToString(
        true, 0x1B04, buf.data(), buf.size(), fields.data(), fields.size());

    EXPECT_NE(s.find("TxPDO"), std::string::npos);
    EXPECT_NE(s.find("0x1B04"), std::string::npos);
    EXPECT_NE(s.find("Statusword"), std::string::npos);
    EXPECT_NE(s.find("0x1234"), std::string::npos);
    EXPECT_NE(s.find("0xFFFF"), std::string::npos);
    EXPECT_NE(s.find("(-1)"), std::string::npos);
    EXPECT_NE(s.find("0x12345678"), std::string::npos);
}

TEST(PDOToString, FallsBackToHexdumpWithoutFields)
{
    const std::array<uint8_t, 4> buf = {0x01, 0x02, 0x03, 0x04};

    const std::string s = EtherCAT::Utils::pdoToString(false, 0x1702, buf.data(), buf.size());

    EXPECT_NE(s.find("RxPDO"), std::string::npos);
    EXPECT_NE(s.find("0x1702"), std::string::npos);
    EXPECT_NE(s.find("0000:"), std::string::npos);
    EXPECT_NE(s.find("01 02 03 04"), std::string::npos);
}
