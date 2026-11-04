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

// ============================================================================
// Edge cases: empty / partial buffers, unnamed and odd-width fields
// ============================================================================

TEST(PDOToString, EmptyBufferWithFieldsReportsNoBuffer)
{
    const std::array<EtherCAT::Utils::PDOFieldDescriptor, 1> fields = {
        EtherCAT::Utils::PDOFieldDescriptor{0x6041, 0x00, 0, 2, "Statusword"},
    };

    const std::string s = EtherCAT::Utils::pdoToString(
        true, 0x1A00, nullptr, 0, fields.data(), fields.size());

    EXPECT_NE(s.find("Statusword"), std::string::npos);
    EXPECT_NE(s.find("<no buffer>"), std::string::npos);
}

TEST(PDOToString, EmptyBufferWithoutFieldsReportsEmptyHexdump)
{
    const std::string s = EtherCAT::Utils::pdoToString(
        false, 0x1600, nullptr, 0, nullptr, 0);

    EXPECT_NE(s.find("RxPDO 0x1600 (0 bytes)"), std::string::npos);
    EXPECT_NE(s.find("<empty>"), std::string::npos);
}

TEST(PDOToString, OutOfRangeFieldReported)
{
    const std::array<uint8_t, 2> buf = {0x11, 0x22};
    const std::array<EtherCAT::Utils::PDOFieldDescriptor, 1> fields = {
        EtherCAT::Utils::PDOFieldDescriptor{0x607A, 0x00, 1, 4, "TargetPosition"},
    };

    const std::string s = EtherCAT::Utils::pdoToString(
        true, 0x1A00, buf.data(), buf.size(), fields.data(), fields.size());

    EXPECT_NE(s.find("<out of range>"), std::string::npos);
}

TEST(PDOToString, UnnamedAndOddWidthFields)
{
    const std::array<uint8_t, 8> buf = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    };
    const std::array<EtherCAT::Utils::PDOFieldDescriptor, 3> fields = {
        EtherCAT::Utils::PDOFieldDescriptor{0x6000, 0x01, 0, 1, nullptr},
        // 3-byte field: no signed rendering, hex only.
        EtherCAT::Utils::PDOFieldDescriptor{0x6001, 0x02, 1, 3, "OddWidth"},
        // 8-byte field: signed rendering with the widest width.
        EtherCAT::Utils::PDOFieldDescriptor{0x6002, 0x03, 0, 8, "Wide"},
    };

    const std::string s = EtherCAT::Utils::pdoToString(
        false, 0x1600, buf.data(), buf.size(), fields.data(), fields.size());

    EXPECT_NE(s.find("<unnamed>"), std::string::npos);
    EXPECT_NE(s.find("OddWidth"), std::string::npos);
    EXPECT_NE(s.find("0x040302"), std::string::npos);  // little-endian
    EXPECT_NE(s.find("Wide"), std::string::npos);
    EXPECT_NE(s.find("0x0807060504030201"), std::string::npos);
}

TEST(PDOToString, HexdumpSpansMultipleLinesAndCapsAtSixteen)
{
    std::array<uint8_t, 40> buf{};
    for (size_t i = 0; i < buf.size(); ++i)
        buf[i] = static_cast<uint8_t>(i + 1);

    const std::string s = EtherCAT::Utils::pdoToString(
        false, 0x1600, buf.data(), buf.size());

    EXPECT_NE(s.find("0000:"), std::string::npos);
    EXPECT_NE(s.find("0010:"), std::string::npos);
    EXPECT_NE(s.find("0020:"), std::string::npos);
    // 40 bytes -> third line holds only 8 bytes (0x21 .. 0x28).
    EXPECT_NE(s.find("21 22 23 24 25 26 27 28"), std::string::npos);
}

TEST(PDOToString, LongFieldDescriptionUsesDynamicBuffer)
{
    // A description long enough that a formatted line exceeds the 256-byte
    // stack buffer, exercising the vsnprintf retry path.
    const std::string long_desc(300, 'D');
    const std::array<uint8_t, 2> buf = {0x01, 0x00};
    const std::array<EtherCAT::Utils::PDOFieldDescriptor, 1> fields = {
        EtherCAT::Utils::PDOFieldDescriptor{0x6041, 0x00, 0, 2, long_desc.c_str()},
    };

    const std::string s = EtherCAT::Utils::pdoToString(
        true, 0x1A00, buf.data(), buf.size(), fields.data(), fields.size());

    EXPECT_NE(s.find(long_desc), std::string::npos);
    EXPECT_NE(s.find("(1)"), std::string::npos);
}
