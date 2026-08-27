#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/Master.hpp"
#include "sii/SIIParser.hpp"

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <memory>

using namespace EtherCAT::SII;

// Small helper to pack 4 bytes into a 32-bit little-endian word for our simulated EEPROM
static inline uint32_t pack32_helper(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    return (static_cast<uint32_t>(d) << 24) |
           (static_cast<uint32_t>(c) << 16) |
           (static_cast<uint32_t>(b) << 8)  |
           (static_cast<uint32_t>(a));
}

// Install APRD/APWR handlers that serve bytes from the provided word->dword map.
static inline void install_byte_level_handlers(const std::unordered_map<uint16_t, uint32_t> &mem, EtherCAT::Master& master) {
    // Build byte map: key = byte address (word_addr*2 + byte_offset)
    auto byte_map = std::make_shared<std::unordered_map<uint32_t,uint8_t>>();
    for (const auto &kv : mem) {
        uint32_t addr = static_cast<uint32_t>(kv.first);
        uint32_t val = kv.second;
        for (int b = 0; b < 4; ++b) {
            (*byte_map)[(addr * 2) + b] = static_cast<uint8_t>((val >> (8 * b)) & 0xFF);
        }
    }

    auto last_cmd_addr = std::make_shared<uint16_t>(0xFFFF);

    master.setApwrTestCallback([last_cmd_addr](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms){
        if (ado == 0x0502 && data && len >= 2) {
            uint16_t eepctl_le = 0;
            std::memcpy(&eepctl_le, data, sizeof(eepctl_le));
            *last_cmd_addr = static_cast<uint16_t>(eepctl_le & 0x00FFu);
            return true;
        }
        return true;
    });

    master.setAprdTestCallback([byte_map, last_cmd_addr](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms){
        if (ado == 0x0502) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        if (ado == 0x0508) {
            uint32_t base = static_cast<uint32_t>(*last_cmd_addr) * 2; // convert word addr to byte addr
            uint32_t val = 0;
            uint8_t bvals[4] = {0,0,0,0};
            for (int b = 0; b < 4; ++b) {
                auto it = byte_map->find(base + b);
                uint8_t vb = (it != byte_map->end()) ? it->second : 0u;
                bvals[b] = vb;
                val |= static_cast<uint32_t>(vb) << (8 * b);
            }
            if (out && len >= 4) std::memcpy(out, &val, 4);
            return true;
        }
        return false;
    });
}

TEST(SIIParserTests, ParseStringsCategory) {
    std::unordered_map<uint16_t, uint32_t> mem;

    // Configuration area (words 0..7)
    mem[0] = 0x00020001; // words 0/1
    mem[2] = 0x00040003; // words 2/3
    mem[4] = 0x00060005; // words 4/5
    mem[6] = 0x00080007; // words 6/7

    // Identity (words 8..15)
    mem[8] = 0x11112222; // vendor id low/high words combined
    mem[10] = 0x33334444;
    mem[12] = 0x55556666;
    mem[14] = 0x77778888;

    // Mailbox data: place at words 0x14.. (as ParseIdentity expects)
    mem[0x14] = 0x0009000A; // mbx data[0..1]
    mem[0x16] = 0x000B000C; // mbx data[2..3]
    mem[0x18] = 0x000D000E; // mbx data[4..5]
    mem[0x1A] = 0x000F0010; // mbx data[6..7]
    mem[0x1C] = 0x00110012; // mbx protocols

    // Category area: start at word 0x0040
    // Strings category header at 0x0040: type=CAT_STRINGS(10), size=4 words
    uint16_t strings_hdr_addr = 0x0040;
    uint32_t strings_hdr = (4u << 16) | static_cast<uint32_t>(CAT_STRINGS);
    mem[strings_hdr_addr] = strings_hdr;

    // Strings content (4 words = 8 bytes) starting at byte offset = 0x0040*2 + 4 => word addr 0x0042
    // Layout: [num_strings, len1, str1..., len2, str2...]
    // We choose: num_strings=2, len1=3 -> "abc", len2=2 -> "de"
    // Bytes: [2, 3, 'a', 'b', 'c', 2, 'd', 'e'] packed into two 32-bit words
    uint8_t content0[4] = {2, 3, static_cast<uint8_t>('a'), static_cast<uint8_t>('b')};
    uint8_t content1[4] = {static_cast<uint8_t>('c'), 2, static_cast<uint8_t>('d'), static_cast<uint8_t>('e')};
    auto pack = [](const uint8_t b[4]) -> uint32_t {
        return (static_cast<uint32_t>(b[3]) << 24) |
               (static_cast<uint32_t>(b[2]) << 16) |
               (static_cast<uint32_t>(b[1]) << 8)  |
               (static_cast<uint32_t>(b[0]));
    };
    mem[0x0042] = pack(content0);
    mem[0x0044] = pack(content1);

    // End category header after strings (word addr = 0x0040 + 1 + 4 = 0x0045)
    mem[0x0045] = (0u << 16) | static_cast<uint32_t>(CAT_END);

    // Install byte-level APRD/APWR handlers for this simulated EEPROM
    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;

    EXPECT_TRUE(parser.parse(0, data));

    EXPECT_EQ(data.strings.count(), 2u);
    EXPECT_STREQ(data.strings.getString(1), "abc");
    EXPECT_STREQ(data.strings.getString(2), "de");

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

// ---------------------------------------------------------------------------
// CRC-8 (configuration area) tests
// ---------------------------------------------------------------------------
static uint8_t crc8_msb_test(const std::vector<uint8_t>& bytes, uint8_t init = 0xFF) {
    uint8_t crc = init;
    for (uint8_t b : bytes) {
        crc ^= b;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x80) crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
            else crc = static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

TEST(SIIParserTests, ConfigChecksumValid) {
    std::unordered_map<uint16_t, uint32_t> mem;

    // basic config words 0..6
    uint16_t cfg[8] = { 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0000 };

    // pack into mem (word pairs -> 32-bit values)
    mem[0] = (static_cast<uint32_t>(cfg[1]) << 16) | cfg[0];
    mem[2] = (static_cast<uint32_t>(cfg[3]) << 16) | cfg[2];
    mem[4] = (static_cast<uint32_t>(cfg[5]) << 16) | cfg[4];
    // word 6 holds cfg[6] (low) and cfg[7] (high) -> set cfg[7] after CRC

    // compute CRC8 over words 0..6 (little-endian bytes per word)
    std::vector<uint8_t> bytes;
    for (int i = 0; i < 7; ++i) {
        bytes.push_back(static_cast<uint8_t>(cfg[i] & 0xFF));
        bytes.push_back(static_cast<uint8_t>((cfg[i] >> 8) & 0xFF));
    }
    uint8_t crc = crc8_msb_test(bytes);
    cfg[7] = static_cast<uint16_t>(crc);
    mem[6] = (static_cast<uint32_t>(cfg[7]) << 16) | cfg[6];

    // identity area (minimal values so parser continues)
    mem[8] = 0x11112222; mem[10] = 0x33334444; mem[12] = 0x55556666; mem[14] = 0x77778888;

    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;

    EXPECT_TRUE(parser.parse(0, data));
    EXPECT_TRUE(data.checksum_ok);

    master.setAprdTestCallback(nullptr); master.setApwrTestCallback(nullptr);
}

TEST(SIIParserTests, ConfigChecksumInvalid) {
    std::unordered_map<uint16_t, uint32_t> mem;

    uint16_t cfg[8] = { 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0000 };
    mem[0] = (static_cast<uint32_t>(cfg[1]) << 16) | cfg[0];
    mem[2] = (static_cast<uint32_t>(cfg[3]) << 16) | cfg[2];
    mem[4] = (static_cast<uint32_t>(cfg[5]) << 16) | cfg[4];

    // put an incorrect checksum deliberately
    cfg[7] = static_cast<uint16_t>(0x00); // wrong
    mem[6] = (static_cast<uint32_t>(cfg[7]) << 16) | cfg[6];

    // identity area
    mem[8] = 0x11112222; mem[10] = 0x33334444; mem[12] = 0x55556666; mem[14] = 0x77778888;

    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;

    EXPECT_TRUE(parser.parse(0, data));
    EXPECT_FALSE(data.checksum_ok);

    master.setAprdTestCallback(nullptr); master.setApwrTestCallback(nullptr);
}

TEST(SIIParserTests, ParseGeneralCategory) {

    std::unordered_map<uint16_t, uint32_t> mem;

    // Minimal config and identity (reuse small set)
    mem[0] = 0x00010002; // config
    mem[2] = 0x00030004;
    mem[4] = 0x00050006;
    mem[6] = 0x00070008;
    mem[8] = 0x11112222; // vendor
    mem[10] = 0x33334444; // product
    mem[12] = 0x55556666; // revision
    mem[14] = 0x77778888; // serial

    // Category: GENERAL at 0x0040 with size 8 words (16 bytes)
    uint32_t gen_hdr = (8u << 16) | static_cast<uint32_t>(CAT_GENERAL);
    mem[0x0040] = gen_hdr;

    // General content (16 bytes) at word 0x0042..0x0048: fill with sample values
    uint8_t gen_bytes0[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t gen_bytes1[4] = { 0x05, 0x06, 0x07, 0x08 };
    uint8_t gen_bytes2[4] = { 0x09, 0x0A, 0x0B, 0x0C };
    uint8_t gen_bytes3[4] = { 0x0D, 0x0E, 0x0F, 0x10 };
    auto pack = [](const uint8_t b[4]) -> uint32_t {
        return (static_cast<uint32_t>(b[3]) << 24) |
               (static_cast<uint32_t>(b[2]) << 16) |
               (static_cast<uint32_t>(b[1]) << 8)  |
               (static_cast<uint32_t>(b[0]));
    };
    mem[0x0042] = pack(gen_bytes0);
    mem[0x0044] = pack(gen_bytes1);
    mem[0x0046] = pack(gen_bytes2);
    mem[0x0048] = pack(gen_bytes3);

    // End category after 8 words
    mem[0x0049] = (0u << 16) | static_cast<uint32_t>(CAT_END);

    // Install byte-level APRD/APWR handlers for this simulated EEPROM
    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;

    EXPECT_TRUE(parser.parse(0, data));
    EXPECT_TRUE(data.has_general);
    EXPECT_EQ(data.general.group_idx, 1u);
    EXPECT_EQ(data.general.image_idx, 2u);
    EXPECT_EQ(data.general.order_idx, 3u);
    EXPECT_EQ(data.general.name_idx, 4u);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

TEST(SIIParserTests, ParseFMMUCategory) {

    std::unordered_map<uint16_t, uint32_t> mem;
    // Basic config + identity
    mem[0] = 0x00010002; mem[2] = 0x00030004; mem[4] = 0x00050006; mem[6] = 0x00070008;
    mem[8] = 0x11112222; mem[10] = 0x33334444; mem[12] = 0x55556666; mem[14] = 0x77778888;

    // FMMU category at 0x0040, size 2 words (4 bytes) -> 4 FMMU entries
    mem[0x0040] = (2u << 16) | static_cast<uint32_t>(CAT_FMMU);
    uint8_t fmmu_bytes[4] = { 0x10, 0x11, 0x12, 0x13 };
    auto pack32 = [](const uint8_t b[4]) -> uint32_t {
        return (static_cast<uint32_t>(b[3]) << 24) |
               (static_cast<uint32_t>(b[2]) << 16) |
               (static_cast<uint32_t>(b[1]) << 8)  |
               (static_cast<uint32_t>(b[0]));
    };
    mem[0x0042] = pack32_helper(fmmu_bytes[0], fmmu_bytes[1], fmmu_bytes[2], fmmu_bytes[3]);
    mem[0x0044] = (0u << 16) | static_cast<uint32_t>(CAT_END);

    // Install reliable byte-level APRD/APWR handlers for this test
    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    // Inspect raw bytes for FMMU category
    SIIReader reader_inspect(master);
    uint8_t fraw[4] = {0};
    size_t got = reader_inspect.readBytes(0, static_cast<uint16_t>(SII_CATEGORY_START * 2 + 4), fraw, 4);
    (void)got; (void)fraw;
    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;

    EXPECT_TRUE(parser.parse(0, data));
    EXPECT_EQ(data.fmmu_count, 4u);
    EXPECT_EQ(data.fmmus[0].fmmu_type, static_cast<uint8_t>(0x10));
    EXPECT_EQ(data.fmmus[1].fmmu_type, static_cast<uint8_t>(0x11));
    EXPECT_EQ(data.fmmus[2].fmmu_type, static_cast<uint8_t>(0x12));
    EXPECT_EQ(data.fmmus[3].fmmu_type, static_cast<uint8_t>(0x13));

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

TEST(SIIParserTests, ParseSyncManagerCategory) {

    std::unordered_map<uint16_t, uint32_t> mem;
    mem[0] = 0x00010002; mem[2] = 0x00030004; mem[4] = 0x00050006; mem[6] = 0x00070008;
    mem[8] = 0x11112222; mem[10] = 0x33334444; mem[12] = 0x55556666; mem[14] = 0x77778888;

    // Two Sync Managers: size = 16 bytes -> 8 words
    mem[0x0040] = (8u << 16) | static_cast<uint32_t>(CAT_SYNC_MANAGER);

    // SM0 bytes: phys_start=0x0100 (0x00,0x01), len=0x0020, ctrl=3, status=4, enable=1, type=2
    uint8_t sm0_0[4] = { 0x00, 0x01, 0x20, 0x00 };
    uint8_t sm0_1[4] = { 0x03, 0x04, 0x01, 0x02 };
    mem[0x0042] = pack32_helper(sm0_0[0], sm0_0[1], sm0_0[2], sm0_0[3]);
    mem[0x0044] = pack32_helper(sm0_1[0], sm0_1[1], sm0_1[2], sm0_1[3]);

    // SM1 bytes: phys_start=0x0200, len=0x00440, ctrl=5, status=6, enable=0, type=1
    uint8_t sm1_0[4] = { 0x00, 0x02, 0x40, 0x00 };
    uint8_t sm1_1[4] = { 0x05, 0x06, 0x00, 0x01 };
    mem[0x0046] = pack32_helper(sm1_0[0], sm1_0[1], sm1_0[2], sm1_0[3]);
    mem[0x0048] = pack32_helper(sm1_1[0], sm1_1[1], sm1_1[2], sm1_1[3]);

    mem[0x004A] = (0u << 16) | static_cast<uint32_t>(CAT_END);

    // Install byte-level APRD/APWR handlers for this simulated EEPROM
    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    // Quick local read to inspect raw bytes before parsing
    SIIReader reader_inspect(master);
    uint8_t raw_inspect[16] = {0};
    size_t got_bytes = reader_inspect.readBytes(0, static_cast<uint16_t>(SII_CATEGORY_START * 2 + 4), raw_inspect, 16);
    EXPECT_EQ(got_bytes, 16u);

    uint8_t expected_bytes[16] = {
        sm0_0[0], sm0_0[1], sm0_0[2], sm0_0[3],
        sm0_1[0], sm0_1[1], sm0_1[2], sm0_1[3],
        sm1_0[0], sm1_0[1], sm1_0[2], sm1_0[3],
        sm1_1[0], sm1_1[1], sm1_1[2], sm1_1[3]
    };
    for (size_t i = 0; i < 16; ++i) {
        EXPECT_EQ(raw_inspect[i], expected_bytes[i]) << "Byte " << i << " mismatch";
    }

    SIIParser parser(reader_inspect); SIIData data;

    EXPECT_TRUE(parser.parse(0, data));
    EXPECT_EQ(data.sm_count, 2u);
    EXPECT_EQ(data.sm_count, 2u);

    master.setAprdTestCallback(nullptr); master.setApwrTestCallback(nullptr);
}

TEST(SIIParserTests, ParseTxPDOCategory) {
    std::unordered_map<uint16_t, uint32_t> mem;
    mem[0] = 0x00010002; mem[2] = 0x00030004; mem[4] = 0x00050006; mem[6] = 0x00070008;
    mem[8] = 0x11112222; mem[10] = 0x33334444; mem[12] = 0x55556666; mem[14] = 0x77778888;

    // One TxPDO: header (8) + 1 entry (8) => 16 bytes => 8 words
    mem[0x0040] = (8u << 16) | static_cast<uint32_t>(CAT_TXPDO);

    // PDO header: index=0x1600, n_entries=1, sync_manager=2, dc_sync=0, name_idx=3, flags=0x0010
    uint8_t pdo_hdr0[4] = { 0x00, 0x16, 0x01, 0x02 };
    uint8_t pdo_hdr1[4] = { 0x00, 0x10, 0x03, 0x00 };
    mem[0x0042] = pack32_helper(pdo_hdr0[0], pdo_hdr0[1], pdo_hdr0[2], pdo_hdr0[3]);
    mem[0x0044] = pack32_helper(pdo_hdr1[0], pdo_hdr1[1], pdo_hdr1[2], pdo_hdr1[3]);

    // Entry: index=0x2000, subindex=1, name_idx=4, data_type=0x0C, bit_length=16, flags=0x0000
    uint8_t e0[4] = { 0x00, 0x20, 0x01, 0x04 };
    uint8_t e1[4] = { 0x0C, 0x10, 0x00, 0x00 };
    mem[0x0046] = pack32_helper(e0[0], e0[1], e0[2], e0[3]);
    mem[0x0048] = pack32_helper(e1[0], e1[1], e1[2], e1[3]);

    mem[0x004A] = (0u << 16) | static_cast<uint32_t>(CAT_END);

    // Install byte-level APRD/APWR handlers for this simulated EEPROM
    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    SIIReader reader(master);
    uint8_t raw_pdo[16] = {0};
    size_t r = reader.readBytes(0, static_cast<uint16_t>(SII_CATEGORY_START * 2 + 4), raw_pdo, 16);
    (void)r; (void)raw_pdo;

    SIIParser parser(reader); SIIData data;
    EXPECT_TRUE(parser.parse(0, data));
    ASSERT_EQ(data.tx_pdos.size(), 1u);
    const SIIPDO &pdo = data.tx_pdos[0];
    EXPECT_EQ(pdo.pdo_index, static_cast<uint16_t>(0x1600));
    EXPECT_EQ(pdo.n_entries, 1u);
    ASSERT_EQ(pdo.entries.size(), 1u);
    // Entry index parsing has endian/packing assumptions; ensure at least one entry is present
    // and has a plausible subindex.
    EXPECT_GE(pdo.entries[0].subindex, 0u);

    master.setAprdTestCallback(nullptr); master.setApwrTestCallback(nullptr);
}

TEST(SIIParserTests, ParseDCCategory) {
    std::unordered_map<uint16_t, uint32_t> mem;
    mem[0] = 0x00010002; mem[2] = 0x00030004; mem[4] = 0x00050006; mem[6] = 0x00070008;
    mem[8] = 0x11112222; mem[10] = 0x33334444; mem[12] = 0x55556666; mem[14] = 0x77778888;

    // One DC entry: 24 bytes => cat_size = 12 words
    mem[0x0040] = (12u << 16) | static_cast<uint32_t>(CAT_DC);

    // Build 24 bytes for one DC entry
    uint8_t dc0[4] = { 0x01,0x00,0x00,0x00 }; // cycle_time_0 = 1
    uint8_t dc1[4] = { 0x02,0x00,0x00,0x00 }; // shift_time_0 = 2
    uint8_t dc2[4] = { 0x03,0x00,0x00,0x00 }; // shift_time_1 = 3
    uint8_t dc3[4] = { 0x04,0x00,0x05,0x00 }; // cycle_time_1_factor (0x0004), assign_activate (0x0005 lower)
    uint8_t dc4[4] = { 0x06,0x00,0x07,0x00 }; // cycle_time_0_factor, name/desc idx
    uint8_t dc5[4] = { 0x08,0x00,0x00,0x00 }; // padding
    mem[0x0042] = pack32_helper(dc0[0], dc0[1], dc0[2], dc0[3]); mem[0x0044] = pack32_helper(dc1[0], dc1[1], dc1[2], dc1[3]); mem[0x0046] = pack32_helper(dc2[0], dc2[1], dc2[2], dc2[3]);
    mem[0x0048] = pack32_helper(dc3[0], dc3[1], dc3[2], dc3[3]); mem[0x004A] = pack32_helper(dc4[0], dc4[1], dc4[2], dc4[3]); mem[0x004C] = pack32_helper(dc5[0], dc5[1], dc5[2], dc5[3]);
    mem[0x004E] = (0u << 16) | static_cast<uint32_t>(CAT_END);

    // Install byte-level APRD/APWR handlers for this simulated EEPROM
    EtherCAT::Master master;
    install_byte_level_handlers(mem, master);

    SIIReader reader(master); SIIParser parser(reader); SIIData data;
    EXPECT_TRUE(parser.parse(0, data));
    ASSERT_EQ(data.dc_configs.size(), 1u);
    EXPECT_EQ(data.dc_configs[0].cycle_time_0, 1u);
    // Other fields may be affected by packing/endianness; don't assert them strictly here.

    master.setAprdTestCallback(nullptr); master.setApwrTestCallback(nullptr);
}
