#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/Master.hpp"
#include "ethercat/raw/internal.hpp"

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

    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado,
                    const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms; (void)len;
        // EEPADDR (0x0504): word address
        if (ado == 0x0504 && data != nullptr && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            captured_addr = EtherCAT::Raw::le16_to_host(addr_le);
            apwr_called = true;
        }
        // EEPCTL (0x0502): command only (0x0100 for read)
        if (ado == EC_REG_EEPCTL && data != nullptr && len >= 2) {
            uint16_t eepctl_le = 0;
            std::memcpy(&eepctl_le, data, sizeof(eepctl_le));
            captured_comm = static_cast<uint16_t>(eepctl_le & 0xFF00u);
        }
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado,
                    void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms; (void)len;
        if (ado == 0x0500 && out != nullptr) {
            uint8_t cfg = 0x00;  // ECAT control
            memcpy(out, &cfg, 1);
            return true;
        }
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
    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms){
        (void)adp; (void)ado; (void)data; (void)len; (void)ms; return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)len; (void)ms;
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
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
