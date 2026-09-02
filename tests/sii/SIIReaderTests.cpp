#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/Master.hpp"
#include "sii/SIIParser.hpp"
#include "ethercat/raw/internal.hpp"

#include <cstdint>
#include <cstring>

using namespace EtherCAT::SII;

TEST(SIIReaderTests, ReadRaw32AndReadWordEvenOdd) {
    // State to capture the last requested EEPROM word address from APWR commands
    uint16_t last_cmd_addr = 0xFFFF;

    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms){
        // EEPADDR (0x0504) contains the EEPROM word address.
        // EEPCTL (0x0502) contains only the command (0x0100 for read).
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            last_cmd_addr = EtherCAT::Raw::le16_to_host(addr_le);  // little-endian on test host
            return true;
        }
        return true;  // EEPCTL, EEPConfig writes — just acknowledge
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms){
        if (ado == 0x0500) {
            // EEPConfig: ECAT has control (bit 0 = 0)
            if (out && len >= 1) {
                uint8_t cfg = 0x00;
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502) {
            // EEPSTAT: indicate not busy (0)
            if (out && len >= 2) {
                uint16_t estat_le = 0;
                std::memcpy(out, &estat_le, 2);
            }
            return true;
        }
        if (ado == 0x0508) {
            // EEPDAT: return 32-bit value derived from last_cmd_addr for deterministic testing
            uint32_t val = 0xA0000000u | static_cast<uint32_t>(last_cmd_addr);
            if (out && len >= 4) {
                std::memcpy(out, &val, 4);
            }
            return true;
        }
        return false;
    });

    SIIReader reader(master);

    // Test readDWord
    uint32_t dword = 0;
    EXPECT_TRUE(reader.readDWord(0, 0, dword));
    EXPECT_EQ(dword, 0xA0000000u | 0u);

    // Test readWord even address
    uint16_t word = 0;
    EXPECT_TRUE(reader.readWord(0, 0, word));
    EXPECT_EQ(word, static_cast<uint16_t>(dword & 0xFFFF));

    // Test readWord odd address (address 1)
    EXPECT_TRUE(reader.readWord(0, 1, word));
    // Since last_cmd_addr will be 1 for the read, expected dword = 0xA0000001
    EXPECT_EQ(word, static_cast<uint16_t>((0xA0000001u >> 16) & 0xFFFF));

    // Reset hooks
    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

TEST(SIIParserTests, ParseIdentity) {
    uint16_t last_cmd_addr = 0xFFFF;

    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms){
        // EEPADDR (0x0504) contains the EEPROM word address
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            last_cmd_addr = EtherCAT::Raw::le16_to_host(addr_le);
            return true;
        }
        return true;  // EEPCTL, EEPConfig writes — just acknowledge
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms){
        if (ado == 0x0500) {
            if (out && len >= 1) { uint8_t cfg = 0x00; std::memcpy(out, &cfg, 1); }
            return true;
        }
        if (ado == 0x0502) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        if (ado == 0x0508) {
            // Return different values depending on last_cmd_addr
            uint32_t val = 0;
            switch (last_cmd_addr) {
                case 0: val = 0x00010002; break; // config words for addresses 0/1
                case 2: val = 0x00030004; break; // config words for addresses 2/3
                case 4: val = 0x00050006; break; // config
                case 6: val = 0x00070008; break; // config
                case 8: val = 0x11112222; break; // vendor id
                case 10: val = 0x33334444; break; // product code
                case 12: val = 0x55556666; break; // revision
                case 14: val = 0x77778888; break; // serial
                case 0x14: val = 0x0009000A; break; // mbx data[0..1]
                case 0x16: val = 0x000B000C; break; // mbx data[2..3]
                case 0x18: val = 0x000D000E; break; // mbx data[4..5]
                case 0x1A: val = 0x000F0010; break; // mbx data[6..7]
                case 0x1C: val = 0x00110012; break; // mbx protocols
                default: val = 0;
            }
            if (out && len >= 4) std::memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;

    EXPECT_TRUE(parser.parseIdentity(0, data));
    EXPECT_TRUE(data.valid);
    EXPECT_EQ(data.identity.vendor_id, 0x11112222u);
    EXPECT_EQ(data.identity.product_code, 0x33334444u);
    EXPECT_EQ(data.identity.revision_number, 0x55556666u);
    EXPECT_EQ(data.identity.serial_number, 0x77778888u);

    // Mailbox assertions (based on mapping in aprd/apwr stubs)
    EXPECT_EQ(data.mailbox.bootstrap_rx_offset, static_cast<uint16_t>(10));
    EXPECT_EQ(data.mailbox.bootstrap_rx_size, static_cast<uint16_t>(9));
    EXPECT_EQ(data.mailbox.bootstrap_tx_offset, static_cast<uint16_t>(12));
    EXPECT_EQ(data.mailbox.bootstrap_tx_size, static_cast<uint16_t>(11));
    EXPECT_EQ(data.mailbox.std_rx_offset, static_cast<uint16_t>(14));
    EXPECT_EQ(data.mailbox.std_rx_size, static_cast<uint16_t>(13));
    EXPECT_EQ(data.mailbox.std_tx_offset, static_cast<uint16_t>(16));
    EXPECT_EQ(data.mailbox.std_tx_size, static_cast<uint16_t>(15));
    EXPECT_EQ(data.mailbox.protocols, static_cast<uint16_t>(18));

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

TEST(SIIReaderTests, ReadWordsAndBytes) {
    uint16_t last_cmd_addr = 0xFFFF;

    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms){
        // EEPADDR (0x0504) contains the EEPROM word address
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            last_cmd_addr = EtherCAT::Raw::le16_to_host(addr_le);
            return true;
        }
        return true;  // EEPCTL, EEPConfig writes — just acknowledge
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms){
        if (ado == 0x0500) {
            if (out && len >= 1) { uint8_t cfg = 0x00; std::memcpy(out, &cfg, 1); }
            return true;
        }
        if (ado == 0x0502) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        if (ado == 0x0508) {
            // produce deterministic dword based on last_cmd_addr
            uint32_t val = 0x10000u * static_cast<uint32_t>(last_cmd_addr) + static_cast<uint32_t>(last_cmd_addr);
            if (out && len >= 4) std::memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIReader reader(master);

    // readWords: read 4 words starting at word 2
    uint16_t buf[4] = {0};
    size_t n = reader.readWords(0, 2, buf, 4);
    EXPECT_EQ(n, 4u);
    // Validate that words correspond to produced dwords
    // For word addr 2: dword = 0x10000*2 + 2 => low word = 2, high word = 2
    EXPECT_EQ(buf[0], static_cast<uint16_t>(2));
    EXPECT_EQ(buf[1], static_cast<uint16_t>(2));

    // readBytes: read 5 bytes starting at byte address 1
    uint8_t bytes[8] = {0};
    size_t nb = reader.readBytes(0, 1, bytes, 5);
    EXPECT_EQ(nb, 5u);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}
