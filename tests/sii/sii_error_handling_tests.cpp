#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "sii/SIIParser.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SIIRegisters.hpp"
#include "tether/slave/core/SIIStateSlave.hpp"
#include <cstring>

using namespace EtherCAT::SII;

// Register constants (matching SIIReader.cpp internals)
static constexpr uint16_t EC_REG_EEPCTL   = 0x0502;
static constexpr uint16_t EC_REG_EEPSTAT  = 0x0502;
static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;
static constexpr uint16_t EC_ECMD_READ    = 0x0100;
static constexpr uint16_t EC_ESTAT_BUSY   = 0x8000;
static constexpr uint16_t EC_ESTAT_EMASK  = 0x7800;
static constexpr uint16_t EC_ESTAT_CRC_ERR = 0x0800;
static constexpr uint16_t EC_ESTAT_NACK   = 0x2000;

// ============================================================================
// Test 1: Non-NACK error bit (CRC) after busy clears causes readRaw32 to fail
// ============================================================================
TEST(SiiErrorHandling, CrcErrorAfterBusyClear_ReadRaw32Fails) {
    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ado; (void)data; (void)len; (void)ms;
        return true;
    });

    int poll_count = 0;
    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)len; (void)ms;
        if (ado == EC_REG_EEPSTAT && out != nullptr) {
            // First poll (pre-wait): not busy, no errors
            // Subsequent polls (post-wait): not busy, but CRC error set
            poll_count++;
            uint16_t estat = (poll_count <= 1) ? 0 : EC_ESTAT_CRC_ERR;
            memcpy(out, &estat, sizeof(estat));
            return true;
        }
        if (ado == EC_REG_EEPDAT && out != nullptr) {
            uint32_t val = 0xDEADBEEF;
            memcpy(out, &val, sizeof(val));
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0xFFFFFFFFu;
    bool ok = reader.readDWord(0x0000, 0x0008, out);

    EXPECT_FALSE(ok);
    // Data register should not have been read (out stays as initialized or zeroed)
    EXPECT_EQ(out, 0u);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

// ============================================================================
// Test 2: NACK error still triggers retry behavior
// ============================================================================
TEST(SiiErrorHandling, NackError_RetriesAndFails) {
    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ado; (void)data; (void)len; (void)ms;
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)len; (void)ms;
        if (ado == EC_REG_EEPSTAT && out != nullptr) {
            uint16_t estat = EC_ESTAT_NACK;
            memcpy(out, &estat, sizeof(estat));
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    bool ok = reader.readDWord(0x0000, 0x0001, out);

    EXPECT_FALSE(ok);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

// ============================================================================
// Test 3: No blind delay — readRaw32 succeeds when waitNotBusy returns
//         immediately on first poll
// ============================================================================
TEST(SiiErrorHandling, NoBlindDelay_ReadSucceedsOnFirstPoll) {
    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ado; (void)data; (void)len; (void)ms;
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)len; (void)ms;
        if (ado == EC_REG_EEPSTAT && out != nullptr) {
            uint16_t estat = 0; // not busy, no errors — immediate success
            memcpy(out, &estat, sizeof(estat));
            return true;
        }
        if (ado == EC_REG_EEPDAT && out != nullptr) {
            uint32_t val = 0xCAFEBABE;
            memcpy(out, &val, sizeof(val));
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    bool ok = reader.readDWord(0x0000, 0x0040, out);

    EXPECT_TRUE(ok);
    EXPECT_EQ(out, 0xCAFEBABEu);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

// ============================================================================
// Test 4: SIIControlReg bitfield matches ET1100/ET1200 layout
// ============================================================================
TEST(SiiErrorHandling, SIIControlReg_ET1100Layout) {
    SIIControlReg reg{};
    reg.read_op = 1;
    uint16_t raw = std::bit_cast<uint16_t>(reg);
    EXPECT_EQ(raw, 0x0100u); // read_op is bit 8

    reg = {};
    reg.write_op = 1;
    raw = std::bit_cast<uint16_t>(reg);
    EXPECT_EQ(raw, 0x0200u); // write_op is bit 9

    reg = {};
    reg.busy = 1;
    raw = std::bit_cast<uint16_t>(reg);
    EXPECT_EQ(raw, 0x8000u); // busy is bit 15

    reg = {};
    reg.crc_error = 1;
    raw = std::bit_cast<uint16_t>(reg);
    EXPECT_EQ(raw, 0x0800u); // crc_error is bit 11

    reg = {};
    reg.ack_error = 1;
    raw = std::bit_cast<uint16_t>(reg);
    EXPECT_EQ(raw, 0x2000u); // ack_error is bit 13
}

// ============================================================================
// Test 5: SIIControl constants match the new bitfield layout
// ============================================================================
TEST(SiiErrorHandling, SIIControl_ConstantsMatchET1100) {
    namespace SC = EtherCAT::slave::SIIControl;
    EXPECT_EQ(SC::ReadOperation, 0x0100u);
    EXPECT_EQ(SC::WriteOperation, 0x0200u);
    EXPECT_EQ(SC::Busy, 0x8000u);
    EXPECT_EQ(SC::CRCError, 0x0800u);
    EXPECT_EQ(SC::AckError, 0x2000u);
}

// ============================================================================
// Test 6: Category parser treats zero cat_type+cat_size as implicit end
// ============================================================================
TEST(SiiErrorHandling, ZeroCategoryHeader_TreatedAsEnd) {
    auto last_cmd_addr = std::make_shared<uint16_t>(0xFFFF);

    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms; (void)len;
        if (ado == EC_REG_EEPCTL && data && len >= 4) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, reinterpret_cast<const uint8_t*>(data) + 2, sizeof(addr_le));
            *last_cmd_addr = addr_le;
        }
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms;
        if (ado == EC_REG_EEPSTAT && out && len >= 2) {
            uint16_t ok = 0;
            memcpy(out, &ok, 2);
            return true;
        }
        if (ado == EC_REG_EEPDAT && out && len >= 4) {
            // Return valid data for config area and identity, but zeroes for category area
            uint16_t addr = *last_cmd_addr;
            uint32_t val = 0;
            if (addr < 0x0040) {
                // Provide non-zero data for config/identity area
                val = 0x00010001u;
            }
            // For addr >= 0x0040 (category area), return zeroes
            memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;
    bool ok = parser.parse(0, data);

    // Parse should succeed (identity area has valid-ish data)
    // But category parsing should stop at the first zero header
    EXPECT_TRUE(ok);
    // Should not have parsed any categories (all zeros = implicit end)
    EXPECT_EQ(data.sm_count, 0u);
    EXPECT_EQ(data.fmmu_count, 0u);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

// ============================================================================
// Test 7: EEPROM size boundary limits category parsing
// ============================================================================
TEST(SiiErrorHandling, EepromSizeBoundary_LimitsCategoryParsing) {
    auto last_cmd_addr = std::make_shared<uint16_t>(0xFFFF);
    auto poll_count = std::make_shared<int>(0);

    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms; (void)len;
        if (ado == EC_REG_EEPCTL && data && len >= 4) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, reinterpret_cast<const uint8_t*>(data) + 2, sizeof(addr_le));
            *last_cmd_addr = addr_le;
        }
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms;
        if (ado == EC_REG_EEPSTAT && out && len >= 2) {
            uint16_t ok = 0;
            memcpy(out, &ok, 2);
            return true;
        }
        if (ado == EC_REG_EEPDAT && out && len >= 4) {
            uint16_t addr = *last_cmd_addr;
            uint32_t val = 0;

            // Word 0x002E (SII_SIZE_INFO): set to 1 (1Kbit = 64 words)
            // This means EEPROM is only 64 words, category area starts at 0x0040
            // so category parsing should immediately hit the boundary
            if (addr == 0x002E) {
                val = 0x0001; // 1 Kbit
            } else if (addr < 0x0040) {
                // Provide valid-ish data for config/identity area
                val = 0x00010001u;
            }
            // Category area (>= 0x0040) returns zeros, but parser should
            // stop before even reading due to size boundary check
            memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;
    bool ok = parser.parse(0, data);

    EXPECT_TRUE(ok);
    // EEPROM size should be read
    EXPECT_EQ(data.eeprom_size_kbits, 1u);
    EXPECT_EQ(data.eeprom_size_words, 64u);
    // No categories should be parsed (boundary hit immediately at 0x0040 >= 64)
    EXPECT_EQ(data.sm_count, 0u);
    EXPECT_EQ(data.fmmu_count, 0u);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}
