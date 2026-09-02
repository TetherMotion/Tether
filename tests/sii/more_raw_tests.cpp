#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/Master.hpp"
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT::SII;

static constexpr uint16_t EC_REG_EEPCTL   = 0x0502;
static constexpr uint16_t EC_REG_EEPSTAT  = 0x0502;
static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;
static constexpr uint16_t EC_ECMD_READ = 0x0100;
static constexpr uint16_t EC_ESTAT_BUSY  = 0x8000;  // Bit 15 (per ETG.1000.4)

TEST(MoreRaw, ReadBytes_ZeroCount_ReturnsZero) {
    EtherCAT::Master master; SIIReader reader(master);
    uint8_t buf[4] = {0};
    size_t n = reader.readBytes(0, 0, buf, 0);
    EXPECT_EQ(n, 0u);
}

TEST(MoreRaw, ReadString_InvalidArgs_ReturnFalse) {
    EtherCAT::Master master; SIIReader reader(master);
    char buf[8];
    EXPECT_FALSE(reader.readString(0, 0, buf, sizeof(buf))); // string_index == 0 invalid
    EXPECT_FALSE(reader.readString(0, 1, nullptr, 4)); // null buffer
    EXPECT_FALSE(reader.readString(0, 1, buf, 0)); // zero buffer size
}

TEST(MoreRaw, WaitNotBusy_Timeout_ReadDWordFails) {
    // APRD always returns BUSY on EEPSTAT
    EtherCAT::Master master;

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms;
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == EC_REG_EEPCTL || ado == EC_REG_EEPCTL) {}
        if (ado == EC_REG_EEPCTL) {}
        if (ado == EC_REG_EEPDAT) return false; // shouldn't be reached
        if (ado == EC_REG_EEPSTAT && out && len >= 2) {
            uint16_t busy = EC_ESTAT_BUSY; memcpy(out, &busy, 2); return true;
        }
        return false;
    });

    SIIReader reader(master);
    reader.setTimeout(10); // very short timeout
    uint32_t out = 0;
    bool ok = reader.readDWord(0, 0, out);
    EXPECT_FALSE(ok);

    master.setAprdTestCallback(nullptr);
}

TEST(MoreRaw, ApwrTriggersAprdResponse_ReadDWordSucceeds) {
    EtherCAT::Master master;
    master.clearAprdResponses();

    // Push EEPConfig response for forceEepromToEcat (ECAT has control)
    uint8_t eep_cfg = 0x00;
    master.pushAprdResponse(true, 0x0000, 0x0500, &eep_cfg, sizeof(eep_cfg));

    uint16_t captured_addr = 0xFFFF;

    // APWR hook: capture address from EEPADDR, push EEPDAT response on READ command
    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms)->bool {
        (void)ms;
        // Capture word address from EEPADDR (0x0504)
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            captured_addr = EtherCAT::Raw::le16_to_host(addr_le);
            return true;
        }
        // EEPCTL write (0x0502) — check for READ command and push EEPDAT response
        if (ado == EC_REG_EEPCTL && data && len >= 2) {
            uint16_t eepctl_le = 0;
            std::memcpy(&eepctl_le, data, sizeof(eepctl_le));
            uint16_t comm = static_cast<uint16_t>(eepctl_le & 0xFF00u);
            if (comm == EC_ECMD_READ) {
                // Construct value with lower 16 bits equal to the requested address for easy verification
                uint32_t val = (0xCAFEBABE & 0xFFFF0000u) | (static_cast<uint32_t>(captured_addr) & 0xFFFFu);
                master.pushAprdResponse(true, adp, EC_REG_EEPDAT, &val, sizeof(val));
            }
        }
        return true;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    EXPECT_TRUE(reader.readDWord(0, 0x0042, out));
    EXPECT_EQ((out & 0xFFFFu), static_cast<uint32_t>(0x0042u));

    master.clearAprdResponses();
    master.setApwrTestCallback(nullptr);
}
