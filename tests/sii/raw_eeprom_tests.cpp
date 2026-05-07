#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"

using namespace EtherCAT::SII;

// Re-use register constants from SIIReader.cpp (duplicate for test clarity)
static constexpr uint16_t EC_REG_EEPCTL   = 0x0502;
static constexpr uint16_t EC_REG_EEPSTAT  = 0x0502;
static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;
static constexpr uint16_t EC_ECMD_READ = 0x0100;
static constexpr uint16_t EC_ESTAT_NACK  = 0x2000;

TEST(RawEeprom, ReadRaw32_SendsReadCommandAndReturnsData) {
    // Arrange
    uint16_t captured_comm = 0;
    uint16_t captured_addr = 0;
    bool apwr_called = false;

    EtherCAT::EtherCATMaster master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado,
                    const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms; (void)len;
        if (ado == EC_REG_EEPCTL && data != nullptr && len >= 4) {
            // EepromCmd { uint16_t comm_le; uint16_t addr_le; uint16_t d2_le; }
            const uint8_t* b = reinterpret_cast<const uint8_t*>(data);
            uint16_t comm_le = *reinterpret_cast<const uint16_t*>(b);
            uint16_t addr_le = *reinterpret_cast<const uint16_t*>(b + 2);
            // Host is little-endian on tests, so little-endian fields are direct
            captured_comm = comm_le;
            captured_addr = addr_le;
            apwr_called = true;
        }
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado,
                    void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms; (void)len;
        if (ado == EC_REG_EEPSTAT && out != nullptr) {
            uint16_t estat_le = 0; // not busy, no errors (little-endian on host)
            memcpy(out, &estat_le, sizeof(estat_le));
            return true;
        }
        if (ado == EC_REG_EEPDAT && out != nullptr) {
            uint32_t edat_le = 0xDEADBEEF; // little-endian host
            memcpy(out, &edat_le, sizeof(edat_le));
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0;

    // Act
    bool ok = reader.readDWord(0x0000, 0x0042, out);

    // Assert
    EXPECT_TRUE(ok);
    EXPECT_EQ(out, 0xDEADBEEFu);
    EXPECT_TRUE(apwr_called);
    EXPECT_EQ(captured_comm, EC_ECMD_READ);
    EXPECT_EQ(captured_addr, 0x0042u);

    // Cleanup
    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

TEST(RawEeprom, ReadRaw32_FailsAfterRepeatedNack) {
    // Arrange: APRD always returns NACK in EEPSTAT
    EtherCAT::EtherCATMaster master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms){
        (void)adp; (void)ado; (void)data; (void)len; (void)ms; return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)len; (void)ms;
        if (ado == EC_REG_EEPSTAT && out != nullptr) {
            uint16_t estat_le = EC_ESTAT_NACK; // simulate NACK forever (little-endian host)
            memcpy(out, &estat_le, sizeof(estat_le));
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0xFFFFFFFFu;

    // Act
    bool ok = reader.readDWord(0x0000, 0x0001, out);

    // Assert: should fail after retry attempts
    EXPECT_FALSE(ok);
    // Output value may be undefined after failure; do not assert on 'out'.

    // Cleanup
    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}
